# usbd-uac2 Examples

This repository contains example implementations of a USB Audio Class 2 (UAC2) device.

Two example backends are provided, both based on the LPCXpresso55S28 demo board, using the onboard WM8904 DAC:

- **Interrupt-driven example** (`lpc55s28-evk`)
- **DMA-based example** (`lpc55s28-evk-dma`)


## Examples Overview

### Interrupt-driven (`lpc55s28-evk`)

Running at 32bit/48khz. It can't keep up at 96khz. Works on USBFS and USBHS.

This is a minimal implementation intended to demonstrate the fundamental
structure of the class driver. It fills a `bbqueue` as data comes in from USB,
and drains it into the I2S FIFO in the I2S interrupt. This requires a lot of
time-critical CPU work managing buffers. Particularly, the USB peripheral driver
uses a lot of interrupt-free critical sections which can cause late interrupts
and underruns.

This is intended primarily as a learning/reference implementation

### DMA-based (`lpc55s28-evk-dma`)

Running at 32bit/96khz. Works on USBFS and USBHS.

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

## Running the Examples

You can flash and run either example using `cargo embed`:

```sh
cargo embed --release --example lpc55s28-evk --features usbfs
