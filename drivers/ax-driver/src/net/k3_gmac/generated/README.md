# K3 GMAC Generated Files

`bindings.rs` is generated from `wrapper.h` with bindgen.  The wrapper includes
the copied Linux STMMAC hardware headers and defines only the minimal Linux
types/macros needed for register and descriptor ABI generation.

Regenerate from the repository root with:

```bash
bindgen drivers/ax-driver/src/net/k3_gmac/generated/wrapper.h \
  --use-core \
  --ctypes-prefix core::ffi \
  --no-layout-tests \
  --allowlist-var '^(GMAC|DMA|MTL|TDES|RDES|TX_|RX_|PHY_|RMII_|RGMII_|EMAC_|WAKE_|LPI_|AXI_|DWMAC|STMMAC|K3GMAC).*' \
  --allowlist-type '^(dma_desc|dma_extended_desc|dma_edesc|dwmac4_addrs|dwmac4_irq_status|power_event|k3gmac_generated_values)$' \
  -- -Idrivers/ax-driver/src/net/k3_gmac/generated \
     -Idrivers/ax-driver/src/net/k3_gmac/linux/stmmac \
  > drivers/ax-driver/src/net/k3_gmac/generated/bindings.rs
```

The full Linux C files are kept under `../linux/` as reference material.  They
depend on Linux netdev, phylink, clk, reset, DMA, and platform-device APIs, so
they are not part of the live no_std build.
