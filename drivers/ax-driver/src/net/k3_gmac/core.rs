//! Minimal DWMAC5 core for SpacemiT K3 GMAC.

use alloc::collections::VecDeque;
use core::ptr::NonNull;

use ax_kspin::SpinRaw;
use dma_api::{CoherentArray, DeviceDma};
use rd_net::{DmaBuffer, Event, NetError};

use super::{desc, desc::DmaDesc, mdio::Mdio, regs};

pub const QUEUE_ID: usize = 0;
pub const QUEUE_SIZE: usize = 64;
pub const BUFFER_SIZE: usize = 2048;
pub const DMA_ALIGN: usize = 0x1000;
pub const DMA_MASK: u64 = u64::MAX;

const CHANNEL: u32 = 0;
const RESET_TIMEOUT: usize = 100_000;

pub type SharedCore = alloc::sync::Arc<SpinRaw<K3GmacCore>>;

#[derive(Debug, Clone, Copy)]
pub struct K3GmacConfig {
    pub mac: [u8; 6],
    pub tx_fifo_depth: u32,
    pub rx_fifo_depth: u32,
    pub checksum_offload: bool,
}

impl K3GmacConfig {
    pub fn queue_fifo_depths(self) -> (u32, u32) {
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

pub struct K3GmacCore {
    mmio: regs::Mmio,
    tx_ring: CoherentArray<DmaDesc>,
    rx_ring: CoherentArray<DmaDesc>,
    mac: [u8; 6],
    checksum_offload: bool,
    tx_fifo_depth: u32,
    rx_fifo_depth: u32,
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
            mmio: unsafe { regs::Mmio::new(base) },
            tx_ring,
            rx_ring,
            mac: config.mac,
            checksum_offload: config.checksum_offload,
            tx_fifo_depth,
            rx_fifo_depth,
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

    pub fn handle_irq(&mut self) -> Event {
        let status = self.mmio.read(regs::dma_chan_status(CHANNEL));
        if status != 0 {
            self.mmio.write(regs::dma_chan_status(CHANNEL), status);
        }
        if (status
            & (regs::DMA_CHAN_STATUS_NIS | regs::DMA_CHAN_STATUS_AIS | regs::DMA_CHAN_STATUS_FBE))
            != 0
        {
            log::warn!("k3-gmac: DMA channel status={status:#x}");
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

    fn init_hardware(&mut self) -> Result<(), NetError> {
        self.stop_dma();
        self.reset_dma()?;
        self.program_mac_address();
        self.log_hw_features();
        self.log_phy_probe();
        self.init_dma_bus();
        self.init_mtl();
        self.init_rings();
        self.start_dma();
        self.enable_mac();
        Ok(())
    }

    fn reset_dma(&self) -> Result<(), NetError> {
        self.mmio
            .update(regs::DMA_BUS_MODE, 0, regs::DMA_BUS_MODE_SFT_RESET);
        for _ in 0..RESET_TIMEOUT {
            if (self.mmio.read(regs::DMA_BUS_MODE) & regs::DMA_BUS_MODE_SFT_RESET) == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
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
        let txq_size = desc::mtl_fifo_words(self.tx_fifo_depth);
        let tx_op = regs::mtl_chan_tx_op_mode(CHANNEL);
        let tx_value = (self.mmio.read(tx_op)
            & !(regs::MTL_OP_MODE_TXQEN_MASK | regs::MTL_OP_MODE_TQS_MASK))
            | regs::MTL_OP_MODE_TSF
            | regs::MTL_OP_MODE_TXQEN
            | (txq_size << regs::MTL_OP_MODE_TQS_SHIFT);
        self.mmio.write(tx_op, tx_value);

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

        // Drain posted base-address writes so the DMA engine observes the
        // programmed rings before start_dma (k3-ufs does the same around its
        // UTRD base registers).
        let _ = self.mmio.read(regs::dma_chan_tx_base_hi(CHANNEL));
        let _ = self.mmio.read(regs::dma_chan_rx_base_hi(CHANNEL));
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
            "k3-gmac: hw_feature0={:#x} hw_feature1={:#x} hw_feature2={:#x} hw_feature3={:#x}",
            self.mmio.read(regs::GMAC_HW_FEATURE0),
            self.mmio.read(regs::GMAC_HW_FEATURE1),
            self.mmio.read(regs::GMAC_HW_FEATURE2),
            self.mmio.read(regs::GMAC_HW_FEATURE3)
        );
    }

    fn log_phy_probe(&self) {
        let mdio = Mdio::new(&self.mmio);
        match mdio.find_phy() {
            Some(addr) => log::warn!("k3-gmac: detected Clause 22 PHY at address {addr}"),
            None => log::warn!("k3-gmac: no Clause 22 PHY detected during probe"),
        }
    }

    fn enable_mac(&self) {
        self.mmio.write(
            regs::GMAC_PACKET_FILTER,
            regs::GMAC_PACKET_FILTER_PM | regs::GMAC_PACKET_FILTER_PR,
        );
        self.mmio.update(
            regs::GMAC_CONFIG,
            regs::GMAC_CONFIG_FES | regs::GMAC_CONFIG_PS,
            regs::GMAC_CONFIG_DM
                | regs::GMAC_CONFIG_IPC
                | regs::GMAC_CONFIG_TE
                | regs::GMAC_CONFIG_RE,
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
            // DMA cleared OWN: order later descriptor field reads (Linux dma_rmb()).
            desc::dma_rmb();
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
            // DMA cleared OWN: order descriptor and payload reads (Linux dma_rmb()).
            desc::dma_rmb();

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
                self.rx_done.push_back((submitted.bus_addr, 0));
            }
            self.rx_next = next(self.rx_next);
        }
    }

    fn tx_desc(&self, index: usize) -> &DmaDesc {
        &self.tx_ring.as_slice_cpu()[index]
    }

    fn tx_desc_mut(&mut self, index: usize) -> &mut DmaDesc {
        unsafe { &mut self.tx_ring.as_mut_slice_cpu()[index] }
    }

    fn rx_desc(&self, index: usize) -> &DmaDesc {
        &self.rx_ring.as_slice_cpu()[index]
    }

    fn rx_desc_mut(&mut self, index: usize) -> &mut DmaDesc {
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
