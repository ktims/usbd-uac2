#![no_main]
#![no_std]

extern crate panic_probe;
#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault()
}

use core::cell::UnsafeCell;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicUsize, Ordering};
use cortex_m_rt::entry;
use defmt;
use defmt::debug;
use defmt_rtt as _;
use hal::Syscon;
use hal::drivers::{Timer, UsbBus, pins};
use hal::prelude::*;
use hal::raw as pac;
use hal::time::Hertz;
use heapless::spsc::Consumer;
use lpc55_hal::{self as hal};
use pac::interrupt;
use static_cell::StaticCell;
use usb_device::{
    bus::{self},
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    endpoint::IsochronousSynchronizationType,
};
use usbd_uac2::UsbIsochronousFeedback;
use usbd_uac2::{
    self, AudioClassConfig, RangeEntry, TerminalConfig, UsbAudioClass, UsbAudioClockImpl, UsbSpeed,
    constants::{FunctionCode, TerminalType},
    descriptors::{ChannelConfig, ClockType, FormatType1, LockDelay},
};

mod hw;
mod wm8904;

const CODEC_I2C_ADDR: u8 = 0b0011010;
// Fo = M/(N*2*P) * Fin
// Fo = 3072/(125*2*8) * 16MHz = 24.576MHz
const AUDIO_PLL: hw::PllConstants = hw::PllConstants::new(125, 3072, 8);
const FIFO_LENGTH: usize = 2048; // frames
const MCLK_FREQ: u32 = 12288000;
const SAMPLE_RATE: u32 = 48000;
type SampleType = (i32, i32);

struct Clock {}
impl Clock {
    const RATES: [RangeEntry<u32>; 1] = [RangeEntry::new_fixed(SAMPLE_RATE)];
}
impl UsbAudioClockImpl for Clock {
    const CLOCK_TYPE: usbd_uac2::descriptors::ClockType = ClockType::InternalFixed;
    const SOF_SYNC: bool = false;
    fn get_sample_rate(&self) -> core::result::Result<u32, usbd_uac2::UsbAudioClassError> {
        Ok(Clock::RATES[0].min)
    }
    fn get_rates(
        &self,
    ) -> core::result::Result<&[usbd_uac2::RangeEntry<u32>], usbd_uac2::UsbAudioClassError> {
        Ok(&Clock::RATES)
    }
    fn get_clock_validity(&self) -> core::result::Result<bool, usbd_uac2::UsbAudioClassError> {
        Ok(true)
    }
}

#[derive(Default)]
struct PerfCounters {
    frames: AtomicUsize,
    min_fill: AtomicUsize,
    avg_fill: AtomicUsize,
    queue_underflows: AtomicUsize,
    queue_overflows: AtomicUsize,
    audio_underflows: AtomicUsize,
}

impl PerfCounters {
    fn reset(&self) {
        self.frames.store(0, Ordering::Relaxed);
        self.min_fill.store(FIFO_LENGTH, Ordering::Relaxed);
        self.avg_fill.store(FIFO_LENGTH / 2, Ordering::Relaxed);
        self.queue_underflows.store(0, Ordering::Relaxed);
        self.queue_overflows.store(0, Ordering::Relaxed);
        self.audio_underflows.store(0, Ordering::Relaxed);
    }
}

impl defmt::Format for PerfCounters {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "frames: {} min_fill: {} avg fill: {} a_underflows: {} q_underflows: {} q_overflows: {}",
            self.frames.load(Ordering::Relaxed),
            self.min_fill.load(Ordering::Relaxed),
            self.avg_fill.load(Ordering::Relaxed),
            self.audio_underflows.load(Ordering::Relaxed),
            self.queue_underflows.load(Ordering::Relaxed),
            self.queue_overflows.load(Ordering::Relaxed)
        )
    }
}

static FIFO_CONSUMER_STORE: StaticCell<Consumer<SampleType>> = StaticCell::new();
static mut FIFO_CONSUMER: *mut Consumer<SampleType> = null_mut();

static PERF: PerfCounters = PerfCounters {
    frames: AtomicUsize::new(0),
    min_fill: AtomicUsize::new(FIFO_LENGTH),
    avg_fill: AtomicUsize::new(FIFO_LENGTH / 2),
    queue_underflows: AtomicUsize::new(0),
    queue_overflows: AtomicUsize::new(0),
    audio_underflows: AtomicUsize::new(0),
};

#[interrupt]
fn FLEXCOMM7() {
    let i2s = unsafe { &*pac::I2S7::ptr() };

    if i2s.fifostat.read().txlvl().bits() == 0 {
        PERF.audio_underflows.fetch_add(1, Ordering::Relaxed);
    }

    // refil the buffer to 4 frames / 8 samples
    while i2s.fifostat.read().txlvl().bits() <= 6 {
        let fifo = unsafe { &mut *FIFO_CONSUMER };
        if let Some((l, r)) = fifo.dequeue() {
            i2s.fifowr.write(|w| unsafe { w.bits(l as u32) });
            i2s.fifowr.write(|w| unsafe { w.bits(r as u32) });
        } else {
            PERF.queue_underflows.fetch_add(1, Ordering::Relaxed);
            PERF.min_fill.fetch_add(0, Ordering::Relaxed);
            i2s.fifowr.write(|w| unsafe { w.bits(0) });
            i2s.fifowr.write(|w| unsafe { w.bits(0) });
        }
    }
}

struct Audio<'a> {
    running: AtomicBool,
    i2s: I2sTx,
    producer: UnsafeCell<heapless::spsc::Producer<'a, SampleType>>,
    integrator: AtomicI32,
}
impl<'a> Audio<'a> {
    fn start(&self) {
        self.running.store(true, Ordering::Relaxed);
        defmt::info!("playback starting, enabling interrupts");
        self.i2s.i2s.fifointenclr.write(|w| w.txlvl().set_bit());
        // FIFO threshold trigger enable
        self.i2s
            .i2s
            .fifotrig
            .modify(|_, w| unsafe { w.txlvl().bits(4).txlvlena().enabled() });
        // FIFO level interrupt enable
        self.i2s.i2s.fifointenset.modify(|_, w| w.txlvl().enabled());
        unsafe { pac::NVIC::unmask(pac::Interrupt::FLEXCOMM7) };
    }
    fn stop(&self) {
        self.running.store(true, Ordering::Relaxed);
        defmt::info!("playback stopped: {}", PERF);
        PERF.reset();
        pac::NVIC::mask(pac::Interrupt::FLEXCOMM7);
    }
}
impl<'a, B: bus::UsbBus> UsbAudioClass<'a, B> for Audio<'_> {
    fn alternate_setting_changed<CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>>(
        &self,
        ac: &mut usbd_uac2::AudioClass<'a, B, CS, AU>,
        terminal: usb_device::UsbDirection,
        alt_setting: u8,
    ) {
        match alt_setting {
            0 => self.stop(),
            1 => self.start(),
            _ => defmt::error!("unexpected alt setting {}", alt_setting),
        }
    }
    fn audio_data_rx(&self, ep: &usb_device::endpoint::Endpoint<'a, B, usb_device::endpoint::Out>) {
        let mut buf = [0; 384];
        let len = match ep.read(&mut buf) {
            Ok(len) => len,
            Err(e) => {
                defmt::error!("usb error in rx callback");
                return;
            }
        };
        let buf = &buf[..len];
        for sample in buf.chunks_exact(8).map(|b| {
            (
                i32::from_le_bytes(b[..4].try_into().unwrap()),
                i32::from_le_bytes(b[4..].try_into().unwrap()),
            )
        }) {
            if let Err(e) = unsafe { (*self.producer.get()).enqueue(sample) } {
                PERF.queue_overflows.fetch_add(1, Ordering::Relaxed);
                // defmt::error!("overflowed fifo, len: {}", unsafe {
                //     (*self.producer.get()).len()
                // });
            }
        }
    }
    fn feedback(&self) -> Option<UsbIsochronousFeedback> {
        const TARGET: i32 = FIFO_LENGTH as i32 / 2 - 64;
        const NOMINAL: i32 = 48 << 16;

        let queuelen = unsafe { (*self.producer.get()).len() };
        let error = (queuelen as i32 - TARGET).clamp(-32, 32);

        // --- integrator ---
        let scaled_error = error / 64;

        let new_i = self.integrator.fetch_add(scaled_error, Ordering::Relaxed) + scaled_error;
        let clamped = new_i.clamp(-131072, 131072);

        // leak + store final value
        let leaked = clamped - (clamped >> 8);
        self.integrator.store(leaked, Ordering::Relaxed);

        // reset on large deviation
        if error.abs() > 96 {
            self.integrator.store(0, Ordering::Relaxed);
        }

        // --- gains ---
        let p = error / 128;
        let i = leaked / 32768;

        // correction
        let correction = (-(p + i)).clamp(-32, 32);
        let v = NOMINAL + (correction << 10);

        // EMA (unchanged, already correct)
        let ema = PERF.avg_fill.load(Ordering::Relaxed);
        let new = ((ema * 1023) + queuelen + 512) >> 10;
        PERF.avg_fill.store(new, Ordering::Relaxed);

        defmt::debug!(
            "q:{} p:{} i:{} err:{} fb:{}+{}",
            queuelen,
            p,
            i,
            error,
            NOMINAL >> 16,
            correction
        );

        Some(UsbIsochronousFeedback::new(v as u32))
    }
}

// Set PLL0 to 24.576MHz, start, and wait for lock
// This is not exposed by lpc55-hal, unfortunately. Copy their implementation here.
fn init_audio_pll() {
    let syscon = unsafe { &*pac::SYSCON::ptr() };
    let pmc = unsafe { &*pac::PMC::ptr() };
    let anactrl = unsafe { &*pac::ANACTRL::ptr() };

    debug!("start clk_in");
    pmc.pdruncfg0
        .modify(|_, w| w.pden_xtal32m().poweredon().pden_ldoxo32m().poweredon());
    syscon.clock_ctrl.modify(|_, w| w.clkin_ena().enable());
    anactrl
        .xo32m_ctrl
        .modify(|_, w| w.enable_system_clk_out().enable());

    debug!("init pll: {}", AUDIO_PLL);
    pmc.pdruncfg0
        .modify(|_, w| w.pden_pll0().poweredoff().pden_pll0_sscg().poweredoff());
    syscon.pll0clksel.write(|w| w.sel().enum_0x1()); // clk_in
    syscon.pll0ctrl.write(|w| unsafe {
        w.clken()
            .enable()
            .seli()
            .bits(AUDIO_PLL.seli)
            .selp()
            .bits(AUDIO_PLL.selp)
    });

    syscon
        .pll0ndec
        .write(|w| unsafe { w.ndiv().bits(AUDIO_PLL.n) });
    syscon.pll0ndec.write(|w| unsafe {
        w.ndiv().bits(AUDIO_PLL.n).nreq().set_bit() // latch
    });

    syscon
        .pll0pdec
        .write(|w| unsafe { w.pdiv().bits(AUDIO_PLL.p) });
    syscon.pll0pdec.write(|w| unsafe {
        w.pdiv().bits(AUDIO_PLL.p).preq().set_bit() // latch
    });

    syscon.pll0sscg0.write(|w| unsafe { w.md_lbs().bits(0) });

    syscon
        .pll0sscg1
        .write(|w| unsafe { w.mdiv_ext().bits(AUDIO_PLL.m).sel_ext().set_bit() });
    syscon.pll0sscg1.write(|w| unsafe {
        w.mdiv_ext()
            .bits(AUDIO_PLL.m)
            .sel_ext()
            .set_bit()
            .mreq()
            .set_bit() // latch
            .md_req()
            .set_bit() // latch
    });

    pmc.pdruncfg0
        .modify(|_, w| w.pden_pll0().poweredon().pden_pll0_sscg().poweredon());
    debug!("pll0 wait for lock");
    let mut i = 0usize;
    while syscon.pll0stat.read().lock().bit_is_clear() {
        i += 1;
    }
    debug!("pll0 locked after {} tries", i);
}

pub struct I2sTx {
    pub i2s: pac::I2S7,
}

pub fn init_i2s(mut fc7: pac::FLEXCOMM7, i2s7: pac::I2S7, syscon: &mut Syscon) -> I2sTx {
    defmt::debug!("init i2s");
    // Enable BOTH
    syscon.reset(&mut fc7);
    syscon.enable_clock(&mut fc7);

    unsafe {
        pac::IOCON::ptr().as_ref().unwrap().pio1_31.modify(|_, w| {
            w.func()
                .alt1()
                .mode()
                .inactive()
                .slew()
                .fast()
                .invert()
                .disabled()
                .digimode()
                .digital()
                .od()
                .normal()
        });
        pac::SYSCON::ptr()
            .as_ref()
            .unwrap()
            .fcclksel7()
            .modify(|_, w| w.sel().enum_0x5()); // MCLK
        pac::SYSCON::ptr()
            .as_ref()
            .unwrap()
            .mclkclksel
            .modify(|_, w| w.sel().enum_0x1()); // PLL0
        pac::SYSCON::ptr()
            .as_ref()
            .unwrap()
            .mclkdiv
            .modify(|_, w| w.div().bits(1).halt().run().reset().released()); // div by 2 = PLL0 fout / 2 = 12.288MHz
        pac::SYSCON::ptr()
            .as_ref()
            .unwrap()
            .mclkio
            .modify(|_, w| w.mclkio().output());
    };

    // Select I2S TX function
    fc7.pselid.write(|w| w.persel().i2s_transmit());

    let regs = i2s7;

    // Enable TX FIFO only
    regs.fifocfg.modify(|_, w| {
        w.enabletx()
            .enabled()
            .enablerx()
            .disabled()
            .dmatx()
            .disabled()
            .txi2se0()
            .zero()
    });

    // Flush
    regs.fifocfg.modify(|_, w| w.emptytx().set_bit());

    regs.cfg2
        .modify(|_, w| unsafe { w.position().bits(0).framelen().bits(63) });

    regs.div.modify(|_, w| unsafe { w.div().bits(3) }); // Clock source is MCLK (24MHz on FRO96) / 4 = 3MHz

    // Config
    regs.cfg1.modify(|_, w| unsafe {
        w.mstslvcfg()
            .normal_master()
            .onechannel()
            .dual_channel()
            .datalen()
            .bits(31)
            .mainenable()
            .enabled()
            .mode()
            .classic_mode()
            .datapause()
            .normal()
    });

    I2sTx { i2s: regs }
}

#[entry]
fn main() -> ! {
    let hal = hal::new();

    let mut anactrl = hal.anactrl;
    let mut pmc = hal.pmc;
    let mut syscon = hal.syscon;

    let mut gpio = hal.gpio.enabled(&mut syscon);
    let mut iocon = hal.iocon.enabled(&mut syscon);

    debug!("start");

    let mut red_led = pins::Pio1_6::take()
        .unwrap()
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(hal::drivers::pins::Level::Low); // start turned on

    debug!("iocon");
    let usb0_vbus_pin = pins::Pio0_22::take()
        .unwrap()
        .into_usb0_vbus_pin(&mut iocon);
    let codec_i2c_pins = (
        pins::Pio1_20::take().unwrap().into_i2c4_scl_pin(&mut iocon),
        pins::Pio1_21::take().unwrap().into_i2c4_sda_pin(&mut iocon),
    );
    let codec_i2s_pins = (
        pins::Pio0_21::take().unwrap().into_spi7_sck_pin(&mut iocon),
        pins::Pio0_20::take().unwrap().into_i2s7_sda_pin(&mut iocon),
        pins::Pio0_19::take().unwrap().into_i2s7_ws_pin(&mut iocon),
        pins::Pio1_31::take().unwrap(), // MCLK
    );

    // iocon.disabled(&mut syscon).release(); // save the environment :)

    debug!("clocks");
    let clocks = hal::ClockRequirements::default()
        .system_frequency(96.MHz())
        .configure(&mut anactrl, &mut pmc, &mut syscon)
        .unwrap();
    let mut _delay_timer = Timer::new(
        hal.ctimer
            .0
            .enabled(&mut syscon, clocks.support_1mhz_fro_token().unwrap()),
    );
    init_audio_pll();

    debug!("peripherals");

    let i2c_peripheral = hal
        .flexcomm
        .4
        .enabled_as_i2c(&mut syscon, &clocks.support_flexcomm_token().unwrap());
    let mut i2c_bus = I2cMaster::new(
        i2c_peripheral,
        codec_i2c_pins,
        Hertz::try_from(400.kHz()).unwrap(),
    );

    let i2s_peripheral = {
        let fc7 = hal.flexcomm.7.release();
        init_i2s(fc7.0, fc7.2, &mut syscon)
    };

    let usb_peripheral = hal.usbhs.enabled_as_device(
        &mut anactrl,
        &mut pmc,
        &mut syscon,
        &mut _delay_timer,
        clocks.support_usbhs_token().unwrap(),
    );

    let usb_bus = UsbBus::new(usb_peripheral, usb0_vbus_pin);
    let clock = Clock {};

    defmt::debug!("codec init");
    wm8904::init_codec(&mut i2c_bus);
    let queue = cortex_m::singleton!(
        : heapless::spsc::Queue<SampleType, FIFO_LENGTH>
        = heapless::spsc::Queue::new()
    )
    .unwrap();
    let (producer, consumer) = queue.split();

    let consumer_ref = FIFO_CONSUMER_STORE.init(consumer);
    unsafe { FIFO_CONSUMER = consumer_ref as *mut _ };

    // i2s_sine_test(&i2s_peripheral.i2s);
    let audio = Audio {
        i2s: i2s_peripheral,
        producer: UnsafeCell::new(producer),
        running: AtomicBool::new(false),
        integrator: AtomicI32::new(0),
    };

    let config = AudioClassConfig::new(UsbSpeed::High, FunctionCode::Other, &clock, &audio)
        .with_input_config(TerminalConfig::new(
            2,
            1,
            2,
            FormatType1 {
                bit_resolution: 32,
                bytes_per_sample: 4,
            },
            TerminalType::ExtLineConnector,
            ChannelConfig::default_chans(2),
            IsochronousSynchronizationType::Asynchronous,
            LockDelay::Undefined(0),
            None,
        ))
        .with_output_config(TerminalConfig::new(
            4,
            1,
            2,
            FormatType1 {
                bit_resolution: 32,
                bytes_per_sample: 4,
            },
            TerminalType::ExtLineConnector,
            ChannelConfig::default_chans(2),
            IsochronousSynchronizationType::Asynchronous,
            LockDelay::Milliseconds(10),
            None,
        ));

    let mut uac2 = config.build(&usb_bus).unwrap();

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0xcc1d))
        .composite_with_iads()
        .strings(&[StringDescriptors::default()
            .manufacturer("VE7XEN")
            .product("Guac")
            .serial_number("123456789")])
        .unwrap()
        .max_packet_size_0(64)
        .unwrap()
        .device_class(0xef)
        .device_sub_class(0x02)
        .device_protocol(0x01)
        .build();

    defmt::info!("main loop");

    loop {
        usb_dev.poll(&mut [&mut uac2]);
        // audio.poll();
        red_led.set_high().ok(); // Turn off
    }
}
