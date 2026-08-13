//! DWMAC5 核心逻辑：DMA 初始化、TX/RX 收发、中断处理。

use alloc::collections::VecDeque;
use core::ptr::NonNull;

use dma_api::{ContiguousArray, DeviceDma, DmaDirection, DmaOp};
use rd_net::{DmaBuffer, Event, NetError};

use super::{desc, desc::DmaDesc, mdio::Mdio, regs};

/// 仅使用 DMA 通道/队列 0（首版单队列）。
pub const QUEUE_ID: usize = 0;
/// 描述符环大小（必须为 2 的幂）。
pub const QUEUE_SIZE: usize = 64;
/// 单个数据缓冲大小（最大以太网帧 + 余量）。
pub const BUFFER_SIZE: usize = 2048;
/// 描述符环 DMA 对齐。
pub const DMA_ALIGN: usize = 0x1000;
/// 设备 DMA 掩码：64 位。
///
/// K3 DWMAC 5.10a 的 hw_feature1 bits[15:14]=1 → addr64=40 位寻址。K3 全部
/// DRAM 都在 4GB 以上（PA 从 0x1_0200_0000 起），必须启用 EAME（Enhanced
/// Address Mode Enable）才能让 DMA 识别描述符 des1 的高 32 位地址。
/// 见 init_dma_bus() 写 DMA_SYS_BUS_EAME。
pub const DMA_MASK: u64 = u64::MAX;

/// DMA 通道 0（单队列实现，仅使用 channel/queue 0）。
const CHANNEL: u32 = 0;
/// DMA 软复位轮询超时。K3 的 DWMAC5 DMA 复位实测可能需要数百毫秒
/// （U-Boot eqos 驱动 swr_wait=500ms）。每次迭代约 1µs（50 次 spin_loop），
/// 1_000_000 次 ≈ 1 秒，留充分余量。
const RESET_TIMEOUT: usize = 1_000_000;

/// 共享核心状态（包在 `Arc<SpinNoIrq<..>>` 里供 Interface/Queue/IRQ 共享）。
pub type SharedCore = alloc::sync::Arc<ax_kspin::SpinNoIrq<K3GmacCore>>;

/// GMAC 驱动配置（从设备树解析 + 默认值）。
#[derive(Debug, Clone, Copy)]
pub struct K3GmacConfig {
    pub mac: [u8; 6],
    pub tx_fifo_depth: u32,
    pub rx_fifo_depth: u32,
    pub checksum_offload: bool,
    /// 静态速率（Mbps）：1000/100/10。首版不动态协商。
    pub speed_mbps: u32,
    pub full_duplex: bool,
}

impl K3GmacConfig {
    /// 队列 FIFO 深度（不小于单包缓冲大小）。
    fn queue_fifo_depths(self) -> (u32, u32) {
        (
            self.tx_fifo_depth.max(BUFFER_SIZE as u32),
            self.rx_fifo_depth.max(BUFFER_SIZE as u32),
        )
    }
}

/// K3 GMAC 核心状态：持有 MMIO、TX/RX 描述符环与索引、缓冲追踪、中断状态。
pub struct K3GmacCore {
    mmio: regs::Mmio,
    tx_ring: ContiguousArray<DmaDesc>,
    rx_ring: ContiguousArray<DmaDesc>,
    mac: [u8; 6],
    checksum_offload: bool,
    tx_fifo_depth: u32,
    rx_fifo_depth: u32,
    speed_mbps: u32,
    full_duplex: bool,
    /// TX 环：下一个待填写的描述符索引（生产者游标）。
    tx_next: usize,
    /// TX 环：下一个待回收的描述符索引（消费者游标）。
    tx_clean: usize,
    /// RX 环：下一个待预填的描述符索引（生产者游标）。
    rx_next: usize,
    /// RX 环：下一个待预填的描述符索引（与 rx_next 同步推进，保留语义对称）。
    rx_fill: usize,
    /// TX in-flight 缓冲追踪：`Some(bus_addr)` 表示该槽位已提交 DMA、尚未回收。
    tx_buffers: [Option<u64>; QUEUE_SIZE],
    /// RX in-flight 缓冲追踪：`Some(bus_addr)` 表示该槽位已预填 DMA、尚未回收。
    rx_buffers: [Option<u64>; QUEUE_SIZE],
    /// TX 已完成缓冲队列（由 reclaim_tx 推入，由上层 reclaim_tx_buffer 弹出）。
    tx_done: VecDeque<u64>,
    /// RX 已完成缓冲队列：(bus_addr, len)，由 reclaim_rx 推入，由上层弹出。
    rx_done: VecDeque<(u64, usize)>,
    irq_enabled: bool,
}

// SAFETY: K3GmacCore 持有 MMIO 指针和 DMA 内存句柄；本身无可变共享状态，
// 并发安全由外层 SpinNoIrq 保证。
unsafe impl Send for K3GmacCore {}

impl K3GmacCore {
    pub fn new(base: NonNull<u8>, dma: &DeviceDma, config: K3GmacConfig) -> Result<Self, NetError> {
        // 用 ContiguousArray（不做 make_uncached/protect 页表重映射），对照 UFS 驱动。
        // CoherentArray 会调 protect() 重映射页表为 UNCACHED，但 K3 RISC-V PTE 无 NC 位，
        // protect() 可能让 TLB/IOMMU 状态不同步，DMA 引擎读不到描述符。
        let tx_ring = dma
            .contiguous_array_zero_with_align::<DmaDesc>(
                QUEUE_SIZE,
                DMA_ALIGN,
                DmaDirection::Bidirectional,
            )
            .map_err(NetError::from)?;
        let rx_ring = dma
            .contiguous_array_zero_with_align::<DmaDesc>(
                QUEUE_SIZE,
                DMA_ALIGN,
                DmaDirection::Bidirectional,
            )
            .map_err(NetError::from)?;
        let (tx_fifo_depth, rx_fifo_depth) = config.queue_fifo_depths();

        let mut this = Self {
            // SAFETY: base 由调用方 iomap 的有效 GMAC MMIO 基址。
            mmio: unsafe { regs::Mmio::new(base) },
            tx_ring,
            rx_ring,
            mac: config.mac,
            checksum_offload: config.checksum_offload,
            tx_fifo_depth,
            rx_fifo_depth,
            speed_mbps: config.speed_mbps,
            full_duplex: config.full_duplex,
            tx_next: 0,
            tx_clean: 0,
            rx_next: 0,
            rx_fill: 0,
            tx_buffers: [None; QUEUE_SIZE],
            rx_buffers: [None; QUEUE_SIZE],
            tx_done: VecDeque::with_capacity(QUEUE_SIZE),
            rx_done: VecDeque::with_capacity(QUEUE_SIZE),
            irq_enabled: false,
        };

        this.init_hardware()?;
        Ok(this)
    }

    pub fn mac_address(&self) -> [u8; 6] {
        self.mac
    }

    pub fn enable_irq(&mut self) {
        self.mmio.write(
            regs::dma_chan_intr_ena(CHANNEL),
            regs::DMA_CHAN_INTR_DEFAULT_MASK,
        );
        self.irq_enabled = true;
    }

    pub fn disable_irq(&mut self) {
        self.mmio.write(regs::dma_chan_intr_ena(CHANNEL), 0);
        self.irq_enabled = false;
    }

    pub fn is_irq_enabled(&self) -> bool {
        self.irq_enabled
    }

    /// 中断处理：读 DMA 通道状态、清中断、回收已完成的 TX/RX，返回事件位图。
    /// 由 IRQ handler（try_lock 上下文）和轮询路径共用。
    pub fn handle_irq(&mut self) -> Event {
        let status = self.mmio.read(regs::dma_chan_status(CHANNEL));
        // 快速短路：NIS|AIS 均未置位说明无任何收发/异常事件（可能是共享 IRQ 的
        // 其他设备触发），直接返回，跳过 reclaim 与日志开销。
        if status & (regs::DMA_CHAN_STATUS_NIS | regs::DMA_CHAN_STATUS_AIS) == 0 {
            return Event::none();
        }
        // write-to-clear：回写清中断
        self.mmio.write(regs::dma_chan_status(CHANNEL), status);
        // 仅 fatal bus error 上报告警，普通事件降到 trace 避免刷屏
        if status & regs::DMA_CHAN_STATUS_FBE != 0 {
            log::warn!("k3-gmac: DMA fatal bus error, status={status:#x}");
        } else {
            log::trace!("k3-gmac: DMA channel status={status:#x}");
        }
        self.reclaim_tx();
        self.reclaim_rx();

        let mut event = Event::none();
        if (status & (regs::DMA_CHAN_STATUS_TI | regs::DMA_CHAN_STATUS_TBU)) != 0
            || !self.tx_done.is_empty()
        {
            event.tx_queue.insert(QUEUE_ID);
        }
        if (status & (regs::DMA_CHAN_STATUS_RI | regs::DMA_CHAN_STATUS_RBU)) != 0
            || !self.rx_done.is_empty()
        {
            event.rx_queue.insert(QUEUE_ID);
        }
        event
    }

    // --- TX ---

    pub fn submit_tx(&mut self, buffer: DmaBuffer) -> Result<(), NetError> {
        self.reclaim_tx();
        let index = self.tx_next;
        if self.tx_buffers[index].is_some() || desc::tx_owned(self.tx_desc(index)) {
            return Err(NetError::Retry);
        }

        let len = buffer.len.min(BUFFER_SIZE);
        let checksum = self.checksum_offload;
        desc::prepare_tx(self.tx_desc_mut(index), buffer.bus_addr, len, checksum);
        self.tx_buffers[index] = Some(buffer.bus_addr);
        self.tx_next = next(index);
        // K3 非一致性：clean 描述符 cache line，让 DMA 看到 OWN 位和地址
        self.flush_tx_desc(index);
        // 写尾指针触发 DMA 拉取新描述符（doorbell）
        let tail = self.tx_ring_dma_addr(self.tx_next) as u32;
        self.mmio.write(regs::dma_chan_tx_end(CHANNEL), tail);
        // 额外 doorbell：重写 ST 位（和 RX 的 SR doorbell 对称）。
        // 部分 DWMAC 实现在 TBU 后需要 ST 重写才能恢复 fetch engine。
        self.mmio
            .update(regs::dma_chan_tx_control(CHANNEL), 0, regs::DMA_CONTROL_ST);
        Ok(())
    }

    pub fn reclaim_tx_buffer(&mut self) -> Option<u64> {
        self.reclaim_tx();
        self.tx_done.pop_front()
    }

    // --- RX ---

    pub fn submit_rx(&mut self, buffer: DmaBuffer) -> Result<(), NetError> {
        let index = self.rx_fill;
        if self.rx_buffers[index].is_some() || desc::rx_owned(self.rx_desc(index)) {
            return Err(NetError::Retry);
        }

        desc::prepare_rx(self.rx_desc_mut(index), buffer.bus_addr);
        self.rx_buffers[index] = Some(buffer.bus_addr);
        self.rx_fill = next(index);
        // K3 非一致性：clean 描述符 cache line，让 DMA 看到 OWN|BUF1V|IOC
        self.flush_rx_desc(index);
        self.mmio.write(
            regs::dma_chan_rx_end(CHANNEL),
            self.rx_ring_dma_addr(self.rx_fill) as u32,
        );
        // 重新置 SR（doorbell）：同 submit_tx 的 ST 重写。
        self.mmio
            .update(regs::dma_chan_rx_control(CHANNEL), 0, regs::DMA_CONTROL_SR);
        Ok(())
    }

    pub fn reclaim_rx_buffer(&mut self) -> Option<(u64, usize)> {
        self.reclaim_rx();
        self.rx_done.pop_front()
    }

    // --- 硬件初始化 ---

    fn init_hardware(&mut self) -> Result<(), NetError> {
        // DMA 软复位（必须）：对照 U-Boot eqos_start + Linux stmmac_hw_setup，
        // 两者都在最开头做 DMA SWR（DMA_BUS_MODE bit0，写 1 后硬件自清）。
        // 软复位不只清寄存器，更重启 DMA 内部描述符 fetch 状态机——
        // 不做这步，U-Boot 残留状态会导致 TX DMA 引擎 cur_tx 不前进
        //（即使 ST=1、tail ptr 正确，fetch engine 卡在非 idle 状态）。
        // 失败时仅 log warn 继续执行（部分平台 SWR 自清较慢但后续仍可用）。
        if let Err(_) = self.reset_dma() {
            log::warn!("k3-gmac: DMA soft reset failed; continuing with U-Boot residual state");
        }
        self.stop_dma();
        self.program_mac_address();
        self.log_hw_features();
        self.init_dma_bus();
        self.init_mtl();
        self.init_rings();
        // 顺序对照 Linux stmmac_hw_setup：先配 MAC（core_init + 地址 + 速率 + TE/RE），
        // 最后才 start_dma（ST/SR）。避免 DMA 在 MAC 未就绪时尝试收发。
        self.init_mac_config();
        let (speed, duplex) = self.init_phy_and_get_link();
        self.apply_link_speed(speed, duplex);
        self.start_dma();
        self.log_post_init_snapshot();
        Ok(())
    }

    /// 执行完整 PHY bring-up（genphy 路径），返回协商到的 (speed, duplex)。
    /// 协商成功用协商结果；PHY 探测失败/超时则回退到 K3GmacConfig 的静态速率。
    fn init_phy_and_get_link(&self) -> (u32, bool) {
        let mdio = Mdio::new(&self.mmio, regs::STMMAC_CSR_250_300M);
        let phy_addr = mdio.find_phy();
        let Some(addr) = phy_addr else {
            log::warn!("k3-gmac: no PHY detected; using static speed config");
            return (self.speed_mbps, self.full_duplex);
        };
        log::info!("k3-gmac: detected Clause 22 PHY at address {addr}");

        match mdio.init_phy(addr, self.speed_mbps) {
            Some(state) if state.up => (state.speed_mbps, state.full_duplex),
            Some(state) => {
                // PHY 探测到但 link 未 up（自协商超时或对端没插网线）
                log::warn!(
                    "k3-gmac: PHY{addr} probed but link DOWN (aneg_done={} speed={}Mbps); using \
                     negotiated speed anyway",
                    state.aneg_complete,
                    state.speed_mbps
                );
                (state.speed_mbps, state.full_duplex)
            }
            None => {
                log::warn!("k3-gmac: PHY{addr} init_phy failed; using static config");
                (self.speed_mbps, self.full_duplex)
            }
        }
    }

    /// 启动后关键寄存器快照（debug 级别）：确认 MAC/DMA/MTL 实际写入值。
    fn log_post_init_snapshot(&self) {
        let gmac_config = self.mmio.read(regs::GMAC_CONFIG);
        let pkt_filter = self.mmio.read(regs::GMAC_PACKET_FILTER);
        let sys_bus = self.mmio.read(regs::DMA_SYS_BUS_MODE);
        let tx_ctrl = self.mmio.read(regs::dma_chan_tx_control(CHANNEL));
        let rx_ctrl = self.mmio.read(regs::dma_chan_rx_control(CHANNEL));
        let dma_status = self.mmio.read(regs::dma_chan_status(CHANNEL));
        let tx_base = self.mmio.read(regs::dma_chan_tx_base(CHANNEL));
        let tx_base_hi = self.mmio.read(regs::dma_chan_tx_base_hi(CHANNEL));
        let rx_base = self.mmio.read(regs::dma_chan_rx_base(CHANNEL));
        let rx_base_hi = self.mmio.read(regs::dma_chan_rx_base_hi(CHANNEL));
        let tx_ring_len = self.mmio.read(regs::dma_chan_tx_ring_len(CHANNEL));
        let rx_ring_len = self.mmio.read(regs::dma_chan_rx_ring_len(CHANNEL));
        let tx_end = self.mmio.read(regs::dma_chan_tx_end(CHANNEL));
        let rx_end = self.mmio.read(regs::dma_chan_rx_end(CHANNEL));
        // GMAC_CONFIG 位：RE=bit0, TE=bit1, DM=bit13, FES=bit14, PS=bit15
        // DMA 控制：ST=bit0(TX), SR=bit0(RX), OSP=bit4
        // SYS_BUS_MODE：EAME=bit11（>32 位寻址，K3 全部 DRAM 在 4GB 以上必须启用）
        log::debug!(
            "k3-gmac: post-init: GMAC_CONFIG={gmac_config:#010x} (RE={} TE={} DM={} FES={} PS={}) \
             | PKT_FILTER={pkt_filter:#010x} | SYS_BUS_MODE={sys_bus:#010x} (EAME={}) | \
             TX_CTRL={tx_ctrl:#010x} (ST={} OSP={}) RX_CTRL={rx_ctrl:#010x} (SR={} RBSZ={}) | \
             DMA_STATUS={dma_status:#010x} | TX base={tx_base:#010x} hi={tx_base_hi:#x} \
             len={tx_ring_len} end={tx_end:#010x} | RX base={rx_base:#010x} hi={rx_base_hi:#x} \
             len={rx_ring_len} end={rx_end:#010x} | TX ring={:#x} RX ring={:#x}",
            gmac_config & regs::GMAC_CONFIG_RE != 0,
            gmac_config & regs::GMAC_CONFIG_TE != 0,
            gmac_config & regs::GMAC_CONFIG_DM != 0,
            gmac_config & regs::GMAC_CONFIG_FES != 0,
            gmac_config & regs::GMAC_CONFIG_PS != 0,
            sys_bus & regs::DMA_SYS_BUS_EAME != 0,
            tx_ctrl & regs::DMA_CONTROL_ST != 0,
            tx_ctrl & regs::DMA_CONTROL_OSP != 0,
            rx_ctrl & regs::DMA_CONTROL_SR != 0,
            (rx_ctrl & regs::DMA_RBSZ_MASK) >> regs::DMA_RBSZ_SHIFT,
            self.tx_ring.dma_addr().as_u64(),
            self.rx_ring.dma_addr().as_u64(),
        );
    }

    fn reset_dma(&self) -> Result<(), NetError> {
        self.mmio
            .update(regs::DMA_BUS_MODE, 0, regs::DMA_BUS_MODE_SFT_RESET);
        // 轮询 SFT_RESET 自清：每次迭代加 spin 延时（约 1µs），
        // 总超时 ~RESET_TIMEOUT µs（匹配 Linux readl_poll_timeout 上限）。
        for i in 0..RESET_TIMEOUT {
            if (self.mmio.read(regs::DMA_BUS_MODE) & regs::DMA_BUS_MODE_SFT_RESET) == 0 {
                log::info!("k3-gmac: DMA soft reset completed after {} iterations", i + 1);
                return Ok(());
            }
            // 每次迭代延时约 1µs（50 次 spin_loop ≈ 1µs @ ~20ns/iter）
            for _ in 0..50 {
                core::hint::spin_loop();
            }
        }
        Err(other("k3-gmac DMA soft reset timed out (SFT_RESET stuck)"))
    }

    fn init_dma_bus(&self) {
        // 用绝对写（writel）而非 read-modify-write，消除 U-Boot 残留位干扰。
        // 对照 U-Boot eqos_start：所有 DMA 寄存器都用 writel 写完整值。
        // DMA_SYS_BUS_MODE：EAME + BLEN4/8/16 + RD_OSR=2（与 U-Boot 完全一致）
        self.mmio.write(
            regs::DMA_SYS_BUS_MODE,
            regs::DMA_SYS_BUS_EAME
                | regs::DMA_AXI_BLEN16
                | regs::DMA_AXI_BLEN8
                | (2u32 << regs::DMA_AXI_RD_OSR_LMT_SHIFT),
        );
        // DMA_CHAN_CONTROL：PBLX8 + DSL=6（64 字节描述符步长）
        let dsl = regs::DMA_CHAN_CONTROL_DSL_DEFAULT << regs::DMA_CHAN_CONTROL_DSL_SHIFT;
        self.mmio.write(
            regs::dma_chan_control(CHANNEL),
            regs::DMA_CHAN_CONTROL_PBLX8 | dsl,
        );
        self.mmio.write(
            regs::dma_chan_intr_ena(CHANNEL),
            regs::DMA_CHAN_INTR_DEFAULT_MASK,
        );
        // TX_CONTROL：OSP + PBL=8（与 RX 一致，PBLX8 让有效突发=64 拍）
        let pbl = 8u32 << regs::DMA_BUS_MODE_PBL_SHIFT;
        self.mmio.write(
            regs::dma_chan_tx_control(CHANNEL),
            regs::DMA_CONTROL_OSP | pbl,
        );
        // RX_CONTROL：RBSZ=2048 + RXPBL=8
        self.mmio.write(
            regs::dma_chan_rx_control(CHANNEL),
            desc::rx_buf_size_bits(BUFFER_SIZE) | (8u32 << regs::DMA_BUS_MODE_PBL_SHIFT),
        );
    }

    fn init_mtl(&self) {
        // TX 队列：Store-and-Forward + 队列使能 + FIFO 大小。
        // 用绝对写（清掉所有残留位，特别是 BIT0=FTQ=Flush TX Queue——
        // 如果 FTQ 残留为 1，TX FIFO 被持续 flush，DMA 永远无法发包）。
        let txq_size = desc::mtl_fifo_words(self.tx_fifo_depth);
        let tx_op = regs::mtl_chan_tx_op_mode(CHANNEL);
        let tx_value = regs::MTL_OP_MODE_TSF
            | regs::MTL_OP_MODE_TXQEN
            | (txq_size << regs::MTL_OP_MODE_TQS_SHIFT);
        self.mmio.write(tx_op, tx_value);

        // RX 队列：Store-and-Forward + FIFO 大小 + 可选硬件流控
        let rxq_size = desc::mtl_fifo_words(self.rx_fifo_depth);
        let rx_op = regs::mtl_chan_rx_op_mode(CHANNEL);
        let mut rx_value = (self.mmio.read(rx_op) & !regs::MTL_OP_MODE_RQS_MASK)
            | regs::MTL_OP_MODE_DIS_TCP_EF
            | regs::MTL_OP_MODE_RSF
            | (rxq_size << regs::MTL_OP_MODE_RQS_SHIFT);

        if self.rx_fifo_depth >= 4096 {
            let (rfd, rfa) = if self.rx_fifo_depth == 4096 {
                (0x03, 0x01)
            } else {
                (0x07, 0x04)
            };
            rx_value = (rx_value & !(regs::MTL_OP_MODE_RFD_MASK | regs::MTL_OP_MODE_RFA_MASK))
                | regs::MTL_OP_MODE_EHFC
                | (rfd << regs::MTL_OP_MODE_RFD_SHIFT)
                | (rfa << regs::MTL_OP_MODE_RFA_SHIFT);
        }
        self.mmio.write(rx_op, rx_value);
    }

    fn init_rings(&mut self) {
        for index in 0..QUEUE_SIZE {
            desc::clear(self.tx_desc_mut(index));
            desc::clear(self.rx_desc_mut(index));
        }
        // K3 非一致性：清零整个描述符环后 flush，确保 DMA 看到全零的初始状态
        let ring_bytes = QUEUE_SIZE * core::mem::size_of::<DmaDesc>();
        axklib::dma::op().flush(self.tx_ring.as_ptr().cast(), ring_bytes);
        axklib::dma::op().flush(self.rx_ring.as_ptr().cast(), ring_bytes);

        // TX 环基址 + 长度。
        // 不写初始 TX 尾指针——对照 U-Boot eqos_start：tail pointer 只在
        // submit_tx 时写。写 base=cur 会让引擎处于 TBU 状态，部分实现
        // 在 TBU→恢复时需要额外 doorbell（见 submit_tx 的 ST 重写）。
        let (tx_base, tx_base_hi) = desc::split_addr(self.tx_ring.dma_addr().as_u64());
        self.mmio
            .write(regs::dma_chan_tx_base_hi(CHANNEL), tx_base_hi);
        self.mmio.write(regs::dma_chan_tx_base(CHANNEL), tx_base);
        self.mmio
            .write(regs::dma_chan_tx_ring_len(CHANNEL), (QUEUE_SIZE - 1) as u32);

        // RX 环基址 + 长度（尾指针在 start_dma 后由 prefill 推进）
        let (rx_base, rx_base_hi) = desc::split_addr(self.rx_ring.dma_addr().as_u64());
        self.mmio
            .write(regs::dma_chan_rx_base_hi(CHANNEL), rx_base_hi);
        self.mmio.write(regs::dma_chan_rx_base(CHANNEL), rx_base);
        self.mmio
            .write(regs::dma_chan_rx_ring_len(CHANNEL), (QUEUE_SIZE - 1) as u32);
    }

    fn program_mac_address(&self) {
        let low = u32::from(self.mac[0])
            | (u32::from(self.mac[1]) << 8)
            | (u32::from(self.mac[2]) << 16)
            | (u32::from(self.mac[3]) << 24);
        let high = u32::from(self.mac[4]) | (u32::from(self.mac[5]) << 8) | regs::GMAC_HI_REG_AE;
        self.mmio.write(regs::GMAC_ADDR_LOW0, low);
        self.mmio.write(regs::GMAC_ADDR_HIGH0, high);
    }

    fn log_hw_features(&self) {
        log::info!(
            "k3-gmac: version={:#x} hw_feature0={:#x} hw_feature1={:#x} hw_feature2={:#x} \
             hw_feature3={:#x}",
            self.mmio.read(regs::GMAC_VERSION),
            self.mmio.read(regs::GMAC_HW_FEATURE0),
            self.mmio.read(regs::GMAC_HW_FEATURE1),
            self.mmio.read(regs::GMAC_HW_FEATURE2),
            self.mmio.read(regs::GMAC_HW_FEATURE3)
        );
    }

    /// 写 GMAC_CONFIG 的核心初始化位（JD|PS|BE|DCRS|JE + IPC + 包过滤），
    /// 不设置 TE/RE 和速率位。
    fn init_mac_config(&self) {
        let mut set = regs::GMAC_CORE_INIT;
        if self.checksum_offload {
            set |= regs::GMAC_CONFIG_IPC;
        }
        self.mmio
            .update(regs::GMAC_CONFIG, regs::GMAC_CORE_INIT, set);
        // 混杂 + 多播（最小工作配置）
        self.mmio.write(
            regs::GMAC_PACKET_FILTER,
            regs::GMAC_PACKET_FILTER_PR | regs::GMAC_PACKET_FILTER_PM,
        );
        // MAC RX Queue 0 Enable (DCB 模式) ——对照 U-Boot eqos_start + Linux dwmac4_rx_queue_enable。
        // 不写此寄存器则 MAC 不将收到的包路由到 DMA channel 0。
        self.mmio
            .update(regs::GMAC_RXQ_CTRL0, 0b11, regs::GMAC_RXQ0EN_DCB);
    }

    /// 按速率/双工写 PS|FES|DM，并使能 TE/RE。
    ///
    /// 修复旧 enable_mac 缺陷：不再写死千兆 + 强制全双工，而是按协商/配置
    /// 动态写速率位（PS+FES 是 2 位字段：00=1000M, 10=10M, 11=100M）。
    fn apply_link_speed(&self, speed_mbps: u32, full_duplex: bool) {
        let speed_bits = match speed_mbps {
            1000 => 0, // GMII/RGMII
            100 => regs::GMAC_CONFIG_FES | regs::GMAC_CONFIG_PS,
            10 => regs::GMAC_CONFIG_PS,
            _ => {
                log::warn!("k3-gmac: unsupported speed {speed_mbps} Mbps, skipping");
                return;
            }
        };
        let duplex = if full_duplex { regs::GMAC_CONFIG_DM } else { 0 };
        self.mmio.update(
            regs::GMAC_CONFIG,
            regs::GMAC_SPEED_MASK
                | regs::GMAC_CONFIG_DM
                | regs::GMAC_CONFIG_TE
                | regs::GMAC_CONFIG_RE,
            speed_bits | duplex | regs::GMAC_CONFIG_TE | regs::GMAC_CONFIG_RE,
        );
        log::info!(
            "k3-gmac: link configured speed={speed_mbps}Mbps duplex={}",
            if full_duplex { "full" } else { "half" }
        );
    }

    fn stop_dma(&self) {
        self.mmio
            .update(regs::dma_chan_tx_control(CHANNEL), regs::DMA_CONTROL_ST, 0);
        self.mmio
            .update(regs::dma_chan_rx_control(CHANNEL), regs::DMA_CONTROL_SR, 0);
        self.mmio.update(
            regs::GMAC_CONFIG,
            regs::GMAC_CONFIG_TE | regs::GMAC_CONFIG_RE,
            0,
        );
    }

    fn start_dma(&self) {
        self.mmio
            .update(regs::dma_chan_rx_control(CHANNEL), 0, regs::DMA_CONTROL_SR);
        self.mmio
            .update(regs::dma_chan_tx_control(CHANNEL), 0, regs::DMA_CONTROL_ST);
    }

    /// 回收已完成的 TX 描述符：DMA 清 OWN 后，将缓冲地址推入 `tx_done`。
    /// 由 `handle_irq`（IRQ 上下文）和 `submit_tx`/`reclaim_tx_buffer`（数据面）调用。
    fn reclaim_tx(&mut self) {
        while let Some(bus_addr) = self.tx_buffers[self.tx_clean] {
            // K3 非一致性：读 DMA 回写状态前 invalidate，丢弃 CPU 侧脏 cache
            self.inval_tx_desc(self.tx_clean);
            if desc::tx_owned(self.tx_desc(self.tx_clean)) {
                break;
            }
            if desc::tx_has_error(self.tx_desc(self.tx_clean)) {
                log::warn!(
                    "k3-gmac: TX descriptor {} completed with error",
                    self.tx_clean
                );
            }
            desc::clear(self.tx_desc_mut(self.tx_clean));
            self.tx_buffers[self.tx_clean] = None;
            self.tx_done.push_back(bus_addr);
            self.tx_clean = next(self.tx_clean);
        }
    }

    /// 回收已完成的 RX 描述符：DMA 填入数据并清 OWN 后，将 (bus_addr, len) 推入 `rx_done`。
    /// 由 `handle_irq`（IRQ 上下文）和 `reclaim_rx_buffer`（数据面）调用。
    fn reclaim_rx(&mut self) {
        while let Some(submitted) = self.rx_buffers[self.rx_next] {
            // K3 非一致性：读 DMA 回写状态前 invalidate，丢弃 CPU 侧脏 cache
            self.inval_rx_desc(self.rx_next);
            if desc::rx_owned(self.rx_desc(self.rx_next)) {
                break;
            }

            let has_err = desc::rx_has_error(self.rx_desc(self.rx_next));
            let len = if desc::rx_ready(self.rx_desc(self.rx_next)) && !has_err {
                desc::rx_len(self.rx_desc(self.rx_next)).min(BUFFER_SIZE)
            } else {
                if has_err {
                    log::warn!(
                        "k3-gmac: RX descriptor {} completed with error",
                        self.rx_next
                    );
                }
                0
            };

            desc::clear(self.rx_desc_mut(self.rx_next));
            self.rx_buffers[self.rx_next] = None;
            // 无论成功/出错都归还缓冲（出错时 len=0），上层重新投递
            self.rx_done.push_back((submitted, len));
            self.rx_next = next(self.rx_next);
        }
    }

    // --- 描述符访问辅助 ---

    fn tx_desc(&self, index: usize) -> &DmaDesc {
        &self.tx_ring.as_slice_cpu()[index]
    }

    fn tx_desc_mut(&mut self, index: usize) -> &mut DmaDesc {
        // SAFETY: CoherentArray::as_mut_slice_cpu 返回 CPU 侧可变切片；
        // 调用方保证同一 index 不被别名（由 tx_next/tx_clean 单调推进保证）。
        unsafe { &mut self.tx_ring.as_mut_slice_cpu()[index] }
    }

    fn rx_desc(&self, index: usize) -> &DmaDesc {
        &self.rx_ring.as_slice_cpu()[index]
    }

    fn rx_desc_mut(&mut self, index: usize) -> &mut DmaDesc {
        // SAFETY: 同 tx_desc_mut。
        unsafe { &mut self.rx_ring.as_mut_slice_cpu()[index] }
    }

    fn tx_ring_dma_addr(&self, index: usize) -> u64 {
        self.tx_ring.dma_addr().as_u64() + desc::ring_offset(index)
    }

    fn rx_ring_dma_addr(&self, index: usize) -> u64 {
        self.rx_ring.dma_addr().as_u64() + desc::ring_offset(index)
    }

    // --- 缓存维护（K3 是 Zicbom 非一致性平台，"coherent" 分配实为可缓存内存）---
    // 写描述符后 clean（push 到内存），让 DMA 能看到 OWN 位和地址；
    // 读 DMA 回写的描述符前 invalidate（丢弃脏 cache），让 CPU 看到设备写入。

    /// Flush（clean）单个 TX 描述符的 cache line，使 CPU 写入对 DMA 可见。
    fn flush_tx_desc(&self, index: usize) {
        // SAFETY: as_ptr 指向已分配的 coherent array；index < QUEUE_SIZE 保证偏移有效。
        let ptr = unsafe {
            NonNull::new_unchecked(
                self.tx_ring
                    .as_ptr()
                    .as_ptr()
                    .cast::<u8>()
                    .add(index * core::mem::size_of::<DmaDesc>()),
            )
        };
        axklib::dma::op().flush(ptr, core::mem::size_of::<DmaDesc>());
    }

    /// Flush 单个 RX 描述符的 cache line。
    fn flush_rx_desc(&self, index: usize) {
        // SAFETY: 同 flush_tx_desc。
        let ptr = unsafe {
            NonNull::new_unchecked(
                self.rx_ring
                    .as_ptr()
                    .as_ptr()
                    .cast::<u8>()
                    .add(index * core::mem::size_of::<DmaDesc>()),
            )
        };
        axklib::dma::op().flush(ptr, core::mem::size_of::<DmaDesc>());
    }

    /// Invalidate 单个 TX 描述符的 cache line，读 DMA 回写状态前调用。
    fn inval_tx_desc(&self, index: usize) {
        // SAFETY: 同 flush_tx_desc。
        let ptr = unsafe {
            NonNull::new_unchecked(
                self.tx_ring
                    .as_ptr()
                    .as_ptr()
                    .cast::<u8>()
                    .add(index * core::mem::size_of::<DmaDesc>()),
            )
        };
        axklib::dma::op().invalidate(ptr, core::mem::size_of::<DmaDesc>());
    }

    /// Invalidate 单个 RX 描述符的 cache line，读 DMA 回写状态前调用。
    fn inval_rx_desc(&self, index: usize) {
        // SAFETY: 同 flush_tx_desc。
        let ptr = unsafe {
            NonNull::new_unchecked(
                self.rx_ring
                    .as_ptr()
                    .as_ptr()
                    .cast::<u8>()
                    .add(index * core::mem::size_of::<DmaDesc>()),
            )
        };
        axklib::dma::op().invalidate(ptr, core::mem::size_of::<DmaDesc>());
    }
}

fn next(index: usize) -> usize {
    (index + 1) & (QUEUE_SIZE - 1)
}

fn other(msg: &'static str) -> NetError {
    NetError::Other(alloc::boxed::Box::new(rd_net::KError::Unknown(msg)))
}
