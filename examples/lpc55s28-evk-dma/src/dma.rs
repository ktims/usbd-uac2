use hal::Syscon;
use hal::peripherals::syscon::ClockControl;

use crate::{hal, pac};
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::ptr::copy_nonoverlapping;
use core::sync::atomic::{AtomicUsize, Ordering, compiler_fence};

pub const DMA0_FLEXCOMM7_TX: u8 = 19;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct DmaDescriptor {
    pub xfercfg: u32,
    pub src_end: *const u8,
    pub dst_end: *mut u32,
    pub next: *const DmaDescriptor,
}

impl defmt::Format for DmaDescriptor {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "xfercfg={:x} src_end={:x} dst_end={:x} next={:x}",
            self.xfercfg,
            self.src_end,
            self.dst_end,
            self.next
        )
    }
}

// Channel descriptor table; linked from SRAMBASE
#[repr(C, align(512))]
pub struct DescriptorTable {
    pub d: [DmaDescriptor; 32],
}
// Our ring that we will transition to once the transfer begins
#[repr(C)]
pub struct RingDescriptors<const N: usize> {
    pub d: [DmaDescriptor; N],
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PushResult {
    pub written: usize,
    pub dropped: usize,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum ConfigError {
    SlotTooLarge,
    SlotTooSmall,
    SlotNotAligned,
    UnsupportedWidth,
}

#[derive(Debug)]
pub enum DmaError {
    Underrun,
}
impl core::error::Error for DmaError {}
impl core::fmt::Display for DmaError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("DmaUnderrun")
    }
}

/// Slot-based DMA ring
pub struct DmaRing<const N: usize, const MAX_SLOT_BYTES: usize> {
    dma: pac::DMA0,

    /// Destination peripheral register (FIFO write register)
    dst_reg: *mut u32,

    // SAFETY: only written by USB task (on start)
    pub(crate) channel_desc: UnsafeCell<DescriptorTable>,
    // SAFETY: only written by USB task (on start)
    pub(crate) desc: UnsafeCell<RingDescriptors<N>>,
    slots: UnsafeCell<[[u8; MAX_SLOT_BYTES]; N]>,

    /// Effective bytes per slot. Maybe be smaller than MAX_SLOT_BYTES (e.g. at lower sample rates), as the setup is designed for constant rate not constant size.
    slot_bytes: usize,
    /// How many bytes to transfer to the FIFO
    word_bytes: usize,

    // SAFETY: producer only
    write_slot: UnsafeCell<usize>,
    write_off: UnsafeCell<usize>,

    produced: AtomicUsize,
    consumed: AtomicUsize,

    /// Leave at least one slot empty so producer never overwrites a slot DMA may still read.
    safety_gap: usize,
    pub produced_bytes: AtomicUsize,
    pub consumed_bytes: AtomicUsize,
}

impl<const N: usize, const MAX_SLOT_BYTES: usize> DmaRing<N, MAX_SLOT_BYTES> {
    /// Construct using PAC DMA0 + &mut SYSCON + a destination FIFO register.
    pub fn new(
        dma: pac::DMA0,
        syscon: &mut Syscon,
        dst_reg: *mut u32,
        word_bytes: usize,
    ) -> Result<Self, ConfigError> {
        if word_bytes != 1 && word_bytes != 2 && word_bytes != 4 {
            return Err(ConfigError::UnsupportedWidth);
        }
        // Start the DMA0 clock
        dma.enable_clock(syscon);

        Ok(Self {
            dma,
            dst_reg: dst_reg,
            channel_desc: UnsafeCell::new(DescriptorTable {
                d: [DmaDescriptor {
                    xfercfg: 0,
                    src_end: core::ptr::null(),
                    dst_end: core::ptr::null_mut(),
                    next: core::ptr::null(),
                }; 32],
            }),
            desc: UnsafeCell::new(RingDescriptors {
                d: [DmaDescriptor {
                    xfercfg: 0,
                    src_end: core::ptr::null(),
                    dst_end: core::ptr::null_mut(),
                    next: core::ptr::null(),
                }; N],
            }),
            slots: UnsafeCell::new([[0u8; MAX_SLOT_BYTES]; N]),
            slot_bytes: MAX_SLOT_BYTES,
            word_bytes,
            write_slot: UnsafeCell::new(0),
            write_off: UnsafeCell::new(0),
            produced: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
            safety_gap: 1,
            produced_bytes: AtomicUsize::new(0),
            consumed_bytes: AtomicUsize::new(0),
        })
    }

    /// Optional: adjust safety gap (defaults to 1 empty slot).
    pub fn set_safety_gap(&mut self, gap_slots: usize) {
        self.safety_gap = gap_slots.min(N);
    }
    pub fn slot_size(&self) -> usize {
        self.slot_bytes
    }
    pub fn set_slot_size(&mut self, slot_bytes: usize) -> Result<(), ConfigError> {
        if slot_bytes == 0 {
            return Err(ConfigError::SlotTooSmall);
        }
        if slot_bytes > MAX_SLOT_BYTES {
            return Err(ConfigError::SlotTooLarge);
        }
        if slot_bytes % self.word_bytes != 0 {
            return Err(ConfigError::SlotNotAligned);
        }
        self.slot_bytes = slot_bytes;
        self.reset_producer();
        Ok(())
    }

    /// Producer: copy into ring; commits whole slots; reports overflow by returning dropped bytes.
    pub fn push(&self, mut data: &[u8]) -> PushResult {
        let mut written = 0usize;

        let write_slot = unsafe { &mut *self.write_slot.get() };
        let write_off = unsafe { &mut *self.write_off.get() };

        let slots = unsafe { &mut *self.slots.get() };
        defmt::debug!(
            "produced={} consumed={} fill={}",
            self.produced(),
            self.consumed(),
            self.fill_slots()
        );
        while !data.is_empty() {
            if self.is_full_for_producer() {
                break;
            }

            let cap = self.slot_bytes - *write_off;
            let n = core::cmp::min(cap, data.len());

            unsafe {
                let dst = slots[*write_slot].as_mut_ptr().add(*write_off);
                copy_nonoverlapping(data.as_ptr(), dst, n);
            }

            *write_off += n;
            written += n;
            data = &data[n..];

            if *write_off == self.slot_bytes {
                // publish completed slot
                compiler_fence(Ordering::Release);
                self.produced.fetch_add(1, Ordering::Release);

                *write_slot = (*write_slot + 1) % N;
                *write_off = 0;
            }
        }

        self.produced_bytes.fetch_add(written, Ordering::Release);

        PushResult {
            written,
            dropped: data.len(),
        }
    }

    /// Call from DMA IRQ bookkeeping when a slot has been consumed.
    pub fn advance_consumed(&self, slots: usize) -> Result<(), DmaError> {
        let produced = self.produced.load(Ordering::Acquire);
        let consumed = self.consumed.load(Ordering::Relaxed);
        if consumed < produced {
            self.consumed.fetch_add(slots, Ordering::Release);
            self.consumed_bytes
                .fetch_add(slots * self.slot_bytes, Ordering::Relaxed);
            Ok(())
        } else {
            defmt::error!("DMA underrun!");
            Err(DmaError::Underrun)
        }
    }

    pub fn produced(&self) -> usize {
        self.produced.load(Ordering::Acquire)
    }
    pub fn produced_bytes(&self) -> usize {
        self.produced_bytes.load(Ordering::Acquire)
    }
    pub fn consumed(&self) -> usize {
        self.consumed.load(Ordering::Acquire)
    }
    pub fn consumed_bytes(&self) -> usize {
        loop {
            let consumed_start = self.consumed.load(Ordering::Acquire);

            let reg_1 = self.dma.channel19.xfercfg.read().bits() as usize >> 16 & 0x3ff;
            let reg_2 = self.dma.channel19.xfercfg.read().bits() as usize >> 16 & 0x3ff;

            let consumed_end = self.consumed.load(Ordering::Acquire);

            if consumed_start == consumed_end && reg_1 == reg_2 {
                // 1. Map the hardware remaining countdown into a clean byte count
                let remaining_bytes = if reg_1 == 0x3ff {
                    0 // 0x3FF means all transfers completed, 0 bytes remaining
                } else {
                    // Formula from NXP manual: (XFERCOUNT + 1) * Data Width
                    (reg_1 + 1) * self.word_bytes
                };

                // 2. Total bytes consumed in this specific active slot
                let active_slot_consumed = self.slot_bytes - remaining_bytes;

                // 3. Combine with your software index history accumulator
                return consumed_start * self.slot_bytes + active_slot_consumed;
            }
        }
    }

    pub fn fill_slots(&self) -> usize {
        self.produced().wrapping_sub(self.consumed())
    }

    pub fn init(&self) {
        self.init_descriptors();

        // Descriptor table base
        let desc = unsafe { &*self.desc.get() };
        let base = self.channel_desc.get() as u32;
        self.dma.srambase.write(|w| unsafe { w.bits(base) });
        self.dma
            .channel19
            .cfg
            .write(|w| w.periphreqen().enabled().hwtrigen().disabled());
        self.dma
            .channel19
            .xfercfg
            .write(|w| unsafe { w.bits(desc.d[0].xfercfg) });

        self.dma.enableclr0.write(|w| unsafe { w.bits(1 << 19) });
        self.dma.ctrl.write(|w| w.enable().enabled());
        self.dma.setvalid0.write(|w| unsafe { w.bits(1 << 19) });
        self.dma.intenset0.write(|w| unsafe { w.bits(1 << 19) });

        self.dma.settrig0.write(|w| unsafe { w.bits(1 << 19) });
    }

    pub fn run(&self) {
        self.dma.enableset0.write(|w| unsafe { w.bits(1 << 19) });
    }

    pub fn stop(&self) {
        self.dma.enableclr0.write(|w| unsafe { w.bits(1 << 19) });
        nb::block!(if (self.dma.busy0.read().bits() & 1 << 19) == 0 {
            Ok(())
        } else {
            Err(nb::Error::<Infallible>::WouldBlock)
        });
        self.dma.abort0.write(|w| unsafe { w.bits(1 << 19) });
        self.reset_producer();
    }

    fn reset_producer(&self) {
        unsafe {
            *(&mut *self.write_slot.get()) = 0;
            *(&mut *self.write_off.get()) = 0;
        }
        self.produced.store(0, Ordering::Relaxed);
        self.produced_bytes.store(0, Ordering::Relaxed);
        self.consumed.store(0, Ordering::Relaxed);
        self.consumed_bytes.store(0, Ordering::Relaxed);
    }

    fn is_full_for_producer(&self) -> bool {
        let fill = self.fill_slots();
        fill >= N.wrapping_sub(self.safety_gap)
    }
    fn reset_producer_init_only(&self) {
        unsafe {
            *self.write_slot.get() = 0;
        }
        unsafe {
            *self.write_off.get() = 0;
        }

        self.produced.store(0, Ordering::Relaxed);
        self.consumed.store(0, Ordering::Relaxed);

        self.produced_bytes.store(0, Ordering::Relaxed);
        self.consumed_bytes.store(0, Ordering::Relaxed);
    }

    fn init_descriptors(&self) {
        let slots = unsafe { &mut *self.slots.get() };
        let desc = unsafe { &mut *self.desc.get() };
        let chan_desc = unsafe { &mut *self.channel_desc.get() };
        defmt::debug!("slots base: &{:x}", self.slots.get());

        // Pre-fill with silence so underrun replays silence.
        for i in 0..N {
            slots[i][..self.slot_bytes].fill(0);
        }

        let transfers = (self.slot_bytes / self.word_bytes) as u32;

        for i in 0..N {
            let src_start = slots[i].as_ptr() as usize;
            let src_end = (src_start + self.slot_bytes - self.word_bytes) as *const u8;

            let next = &desc.d[(i + 1) % N] as *const DmaDescriptor;

            desc.d[i] = DmaDescriptor {
                xfercfg: encode_xfercfg(
                    true,  // valid
                    true,  // reload
                    false, // swtrig (we use XFERCFG SWTRIG kick)
                    false, // clrtrig
                    true,  // intA
                    false, // intB
                    self.word_bytes as u32,
                    1, // src_inc
                    0, // dst_inc
                    transfers,
                ),
                src_end,
                dst_end: self.dst_reg,
                next,
            };
        }
        chan_desc.d[19] = desc.d[0];
        chan_desc.d[19].xfercfg = 0;

        // reset producer indices + counters (init-only action)
        self.reset_producer_init_only();
    }
}

unsafe impl<const N: usize, const MAX_SLOT_BYTES: usize> Sync for DmaRing<N, MAX_SLOT_BYTES> {}

/// XFERCFG encoding follows the common LPC DMA layout:
/// - SETINTA at bit4, SETINTB at bit5
/// - WIDTH at bits 9:8
/// - SRCINC at bits 13:12
/// - DSTINC at bits 15:14
/// - XFERCOUNT at bits 25:16
/// This layout is shown in LPC DMA examples. [5](https://www.kernel.org/doc/html/latest/core-api/dma-api-howto.html)
fn encode_xfercfg(
    cfgvalid: bool,
    reload: bool,
    swtrig: bool,
    clrtrig: bool,
    inta: bool,
    intb: bool,
    width_bytes: u32,
    src_inc: u32,
    dst_inc: u32,
    transfers: u32,
) -> u32 {
    let width_code = match width_bytes {
        1 => 0,
        2 => 1,
        4 => 2,
        _ => 0,
    };

    let count_field = transfers.saturating_sub(1) & 0x3FF;

    ((cfgvalid as u32) << 0)
        | ((reload as u32) << 1)
        | ((swtrig as u32) << 2)
        | ((clrtrig as u32) << 3)
        | ((inta as u32) << 4)
        | ((intb as u32) << 5)
        | ((width_code & 0x3) << 8)
        | ((src_inc & 0x3) << 12)
        | ((dst_inc & 0x3) << 14)
        | (count_field << 16)
}
