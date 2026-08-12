//! DWMAC5 核心逻辑：DMA 初始化、TX/RX 收发、中断处理。
//!
//! 修复点（相对旧实现 f7c4081be）：
//! - `enable_mac` 不再写死千兆 + 强制全双工。拆为 `init_mac_config`（写 CORE_INIT，
//!   不碰速率位）+ `apply_link_speed`（按协商/设备树静态速率写 PS|FES|DM）。
//! - 速率/双工由 `K3GmacConfig::speed_mbps` + `full_duplex` 决定，首版静态配置。

use alloc::collections::VecDeque;
use core::ptr::NonNull;

use dma_api::{CoherentArray, DeviceDma};
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
/// 设备 DMA 掩码（64 位）。
pub const DMA_MASK: u64 = u64::MAX;

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

#[derive(Clone, Copy)]
struct SubmittedRx {
    bus_addr: u64,
    len: usize,
}

/// K3 GMAC 核心状态：持有 MMIO、TX/RX 描述符环与索引、缓冲追踪、中断状态。
pub struct K3GmacCore {
    mmio: regs::Mmio,
    tx_ring: CoherentArray<DmaDesc>,
    rx_ring: CoherentArray<DmaDesc>,
    mac: [u8; 6],
    checksum_offload: bool,
    tx_fifo_depth: u32,
    rx_fifo_depth: u32,
    speed_mbps: u32,
    full_duplex: bool,
    tx_next: usize,
    tx_clean: usize,
    rx_next: usize,
    rx_fill: usize,
    tx_buffers: [Option<u64>; QUEUE_SIZE],
    rx_buffers: [Option<SubmittedRx>; QUEUE_SIZE],
    tx_done: VecDeque<u64>,
    rx_done: VecDeque<(u64, usize)>,
    irq_enabled: bool,
}

// SAFETY: K3GmacCore 持有 MMIO 指针和 DMA 内存句柄；本身无可变共享状态，
// 并发安全由外层 SpinNoIrq 保证。
unsafe impl Send for K3GmacCore {}

impl K3GmacCore {
    pub fn new(base: NonNull<u8>, dma: &DeviceDma, config: K3GmacConfig) -> Result<Self, NetError> {
        let tx_ring = dma
            .coherent_array_zero_with_align::<DmaDesc>(QUEUE_SIZE, DMA_ALIGN)
            .map_err(NetError::from)?;
        let rx_ring = dma
            .coherent_array_zero_with_align::<DmaDesc>(QUEUE_SIZE, DMA_ALIGN)
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
        desc::dma_wmb();
        // 写尾指针触发 DMA 拉取新描述符
        self.mmio.write(
            regs::dma_chan_tx_end(CHANNEL),
            self.tx_ring_dma_addr(self.tx_next) as u32,
        );
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
        self.rx_buffers[index] = Some(SubmittedRx {
            bus_addr: buffer.bus_addr,
            len: buffer.len,
        });
        self.rx_fill = next(index);
        desc::dma_wmb();
        self.mmio.write(
            regs::dma_chan_rx_end(CHANNEL),
            self.rx_ring_dma_addr(self.rx_fill) as u32,
        );
        Ok(())
    }

    pub fn reclaim_rx_buffer(&mut self) -> Option<(u64, usize)> {
        self.reclaim_rx();
        self.rx_done.pop_front()
    }

    // --- 硬件初始化 ---

    fn init_hardware(&mut self) -> Result<(), NetError> {
        self.stop_dma();
        // 尝试 DMA 软复位。K3 实测 SFT_RESET 位常卡在 1 不清零（U-Boot 初始化后
        // 的残留态），但 DMA 引擎在 U-Boot 残留配置下已就绪，复位失败可继续。
        // 见 reset_dma() 的超时 warn 日志。
        let _ = self.reset_dma();
        self.program_mac_address();
        self.log_hw_features();
        self.init_dma_bus();
        self.init_mtl();
        self.init_rings();
        self.start_dma();
        // 先写 CORE_INIT（不含速率位），再按静态速率应用 PS|FES|DM 并使能 TE/RE
        self.init_mac_config();
        self.apply_link_speed(self.speed_mbps, self.full_duplex);
        self.log_phy_probe();
        Ok(())
    }

    fn reset_dma(&self) -> Result<(), NetError> {
        self.mmio
            .update(regs::DMA_BUS_MODE, 0, regs::DMA_BUS_MODE_SFT_RESET);
        // 轮询 SFT_RESET 自清：每次迭代加 spin 延时（约 1µs），
        // 总超时 ~RESET_TIMEOUT µs（匹配 Linux readl_poll_timeout 上限）。
        for i in 0..RESET_TIMEOUT {
            if (self.mmio.read(regs::DMA_BUS_MODE) & regs::DMA_BUS_MODE_SFT_RESET) == 0 {
                log::debug!("k3-gmac: DMA reset completed after {} iterations", i + 1);
                return Ok(());
            }
            // 每次迭代延时约 1µs（50 次 spin_loop ≈ 1µs @ ~20ns/iter）
            for _ in 0..50 {
                core::hint::spin_loop();
            }
        }
        log::warn!(
            "k3-gmac: DMA soft reset timed out (SFT_RESET stuck); continuing in U-Boot residual \
             state"
        );
        Err(other("k3-gmac DMA reset timed out"))
    }

    fn init_dma_bus(&self) {
        self.mmio.update(
            regs::DMA_SYS_BUS_MODE,
            0,
            regs::DMA_SYS_BUS_FB | regs::DMA_SYS_BUS_MB | regs::DMA_SYS_BUS_AAL,
        );
        self.mmio
            .write(regs::DMA_AXI_BUS_MODE, regs::DMA_AXI_BURST_LEN_DEFAULT);
        self.mmio.write(
            regs::dma_chan_intr_ena(CHANNEL),
            regs::DMA_CHAN_INTR_DEFAULT_MASK,
        );
        self.mmio
            .update(regs::dma_chan_tx_control(CHANNEL), 0, regs::DMA_CONTROL_OSP);
        self.mmio.update(
            regs::dma_chan_rx_control(CHANNEL),
            regs::DMA_RBSZ_MASK,
            desc::rx_buf_size_bits(BUFFER_SIZE),
        );
    }

    fn init_mtl(&self) {
        // TX 队列：Store-and-Forward + 队列使能 + FIFO 大小
        let txq_size = desc::mtl_fifo_words(self.tx_fifo_depth);
        let tx_op = regs::mtl_chan_tx_op_mode(CHANNEL);
        let tx_value = (self.mmio.read(tx_op)
            & !(regs::MTL_OP_MODE_TXQEN_MASK | regs::MTL_OP_MODE_TQS_MASK))
            | regs::MTL_OP_MODE_TSF
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

        // TX 环基址 + 长度 + 尾指针
        let (tx_base, tx_base_hi) = desc::split_addr(self.tx_ring.dma_addr().as_u64());
        self.mmio
            .write(regs::dma_chan_tx_base_hi(CHANNEL), tx_base_hi);
        self.mmio.write(regs::dma_chan_tx_base(CHANNEL), tx_base);
        self.mmio
            .write(regs::dma_chan_tx_ring_len(CHANNEL), (QUEUE_SIZE - 1) as u32);
        self.mmio.write(
            regs::dma_chan_tx_end(CHANNEL),
            self.tx_ring_dma_addr(0) as u32,
        );

        // RX 环基址 + 长度 + 尾指针
        let (rx_base, rx_base_hi) = desc::split_addr(self.rx_ring.dma_addr().as_u64());
        self.mmio
            .write(regs::dma_chan_rx_base_hi(CHANNEL), rx_base_hi);
        self.mmio.write(regs::dma_chan_rx_base(CHANNEL), rx_base);
        self.mmio
            .write(regs::dma_chan_rx_ring_len(CHANNEL), (QUEUE_SIZE - 1) as u32);
        self.mmio.write(
            regs::dma_chan_rx_end(CHANNEL),
            self.rx_ring_dma_addr(0) as u32,
        );
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
        log::debug!(
            "k3-gmac: version={:#x} hw_feature0={:#x} hw_feature1={:#x} hw_feature2={:#x} \
             hw_feature3={:#x}",
            self.mmio.read(regs::GMAC_VERSION),
            self.mmio.read(regs::GMAC_HW_FEATURE0),
            self.mmio.read(regs::GMAC_HW_FEATURE1),
            self.mmio.read(regs::GMAC_HW_FEATURE2),
            self.mmio.read(regs::GMAC_HW_FEATURE3)
        );
    }

    fn log_phy_probe(&self) {
        // CSR 时钟分频：K3 的 stmmaceth 时钟在 250-300MHz 范围，对应 CR=5
        // （来源：U-Boot eqos_spacemit_k3_config / Linux stmmac CR_250_300）
        let mdio = Mdio::new(&self.mmio, regs::STMMAC_CSR_250_300M);
        match mdio.find_phy() {
            Some(addr) => log::info!("k3-gmac: detected Clause 22 PHY at address {addr}"),
            None => log::warn!("k3-gmac: no Clause 22 PHY detected during probe"),
        }
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

    fn reclaim_tx(&mut self) {
        while let Some(bus_addr) = self.tx_buffers[self.tx_clean] {
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

    fn reclaim_rx(&mut self) {
        while let Some(submitted) = self.rx_buffers[self.rx_next] {
            if desc::rx_owned(self.rx_desc(self.rx_next)) {
                break;
            }

            let len = if desc::rx_ready(self.rx_desc(self.rx_next))
                && !desc::rx_has_error(self.rx_desc(self.rx_next))
            {
                desc::rx_len(self.rx_desc(self.rx_next)).min(submitted.len)
            } else {
                if desc::rx_has_error(self.rx_desc(self.rx_next)) {
                    log::warn!(
                        "k3-gmac: RX descriptor {} completed with error",
                        self.rx_next
                    );
                }
                0
            };

            desc::clear(self.rx_desc_mut(self.rx_next));
            self.rx_buffers[self.rx_next] = None;
            if len > 0 {
                self.rx_done.push_back((submitted.bus_addr, len));
            } else {
                // 出错时仍归还缓冲（长度 0），上层可重新投递
                self.rx_done.push_back((submitted.bus_addr, 0));
            }
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
}

fn next(index: usize) -> usize {
    (index + 1) & (QUEUE_SIZE - 1)
}

fn other(msg: &'static str) -> NetError {
    NetError::Other(alloc::boxed::Box::new(rd_net::KError::Unknown(msg)))
}
