Two example backends are provided, both based on the LPCXpresso55S28 demo board, using the onboard WM8904 DAC.

## LPC55S28 (LPCXpresso55S28)

These examples work on the
[LPCXpresso55S28](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/lpcxpresso-boards/lpcxpresso55s28-development-board:LPC55S28-EVK)
development board. They use the onboard WM8904 codec, so don't require any
additional hardware for audio I/O.

They can be programmed and debugged using the CMSIS-DAP that's on the board,
though due to the security features you may need to reset into the DFU
bootloader first.

```
cargo embed --release
```

Enable logging with `DEFMT_LOG` when building, but beware that enabling defmt
logs can cause timing failures, and _must_ be drained by the host, as it is
blocking.

They work on either USBFS or USBHS, but not both at the same time. The default
is USBHS. If you would like to run the example on USBFS, disable default
features and select USBFS:

```
cargo embed --release --no-default-features --features usbfs
```

### Interrupt-driven (`lpc55s28-evk`)

Running at 32bit/48khz. Simultaneous input and output. Works on USBFS and USBHS.

This is a minimal implementation intended to demonstrate the fundamental
structure of the class driver. The architecture is not recommended for a production
device, but it does work reliably on the happy path.

This is intended primarily as a reference implementation to aid understanding of
the class driver.

### DMA-based (`lpc55s28-evk-dma`)

Running at 32bit/96khz. Output only. Works on USBFS and USBHS.

A more realistic and robust implementation using DMA. It fills a static ring
buffer as data comes in from USB, while the DMA chases it around the ring,
draining into the TX FIFO. This is efficient and decouples interrupt latency
from data delivery. It uses the DMA interrupt to track consumed slots, but as
long as the USB doesn't catch up to the read slot before this happens, there is
a lot of slack for other things to be happening.

This is a more useful demonstration, but is still lacking correct handling of
edge cases, error conditions and so on you would want in a fully fleshed out
implementation. Particularly, it behaves poorly in underrun, since the DMA will
keep emitting from the ring regardless of whether the data is valid, which
sounds terrible.
