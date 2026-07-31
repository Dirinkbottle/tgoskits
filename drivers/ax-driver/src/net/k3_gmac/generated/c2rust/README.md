# C2Rust Translation Notes

`k3_gmac_reference.c` is a freestanding extraction of the K3 syscon glue and the
DWMAC descriptor/DMA/MAC register sequences used by the live Rust driver.  It was
translated with:

```bash
c2rust transpile \
  --emit-no-std \
  --emit-modules \
  --preserve-unused-functions \
  --translate-const-macros experimental \
  --output-dir drivers/ax-driver/src/net/k3_gmac/generated/c2rust/out \
  drivers/ax-driver/src/net/k3_gmac/generated/c2rust/k3_gmac_reference.c \
  -- -std=c11
```

The generated reference is in `out/src/k3_gmac_reference.rs`.

The copied Linux sources are under `../../linux/stmmac/`.  A direct c2rust pass
over those files still requires the Linux kernel generated header tree and large
stubs for netdev, phylink, clk/reset, regmap, DMA, and platform-device APIs.
Treat the c2rust output as reference-only: the live no_std driver path is the
small Rust module set in `drivers/ax-driver/src/net/k3_gmac`.
