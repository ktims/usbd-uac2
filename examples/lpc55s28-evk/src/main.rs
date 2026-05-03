#![no_main]
#![no_std]

extern crate panic_probe;
#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault()
}

use core::cell::RefCell;

use cortex_m_rt::entry;
use defmt;
use defmt::debug;
use defmt_rtt as _;
use embedded_io::Write;
use hal::Syscon;
use hal::drivers::{Timer, UsbBus, pins};
use hal::peripherals::flexcomm::Flexcomm7;
use hal::prelude::*;
use hal::raw as pac;
use hal::time::Hertz;
use heapless::spsc::Queue;
use lpc55_hal::drivers::clocks::Pll;
use lpc55_hal::peripherals::syscon::ClockControl;
use lpc55_hal::raw::{FLEXCOMM7, I2S7};
use lpc55_hal::{self as hal};

use usb_device::{
    bus::{self},
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    endpoint::IsochronousSynchronizationType,
};
use usbd_uac2::{
    self, AudioClassConfig, RangeEntry, TerminalConfig, UsbAudioClass, UsbAudioClockImpl, UsbSpeed,
    constants::{FunctionCode, TerminalType},
    descriptors::{ChannelConfig, ClockType, FormatType1, LockDelay},
};

const CODEC_I2C_ADDR: u8 = 0b0011010;

const SINE_LUT: [i32; 32] = [
    0, 1636536, 3210180, 4660460, 5931640, 6974871, 7750062, 8227422, 8388607, 8227422, 7750062,
    6974871, 5931640, 4660460, 3210180, 1636536, 0, -1636536, -3210180, -4660460, -5931640,
    -6974871, -7750062, -8227422, -8388607, -8227422, -7750062, -6974871, -5931640, -4660460,
    -3210180, -1636536,
];

pub fn i2s_sine_test(i2s: &pac::I2S7) -> ! {
    let mut idx = 0;
    let mut count = 0usize;

    defmt::debug!("starting sine test");

    loop {
        if i2s.fifostat.read().txnotfull().bit_is_set() {
            let sample = SINE_LUT[idx] * 32;

            // ✅ Left channel
            i2s.fifowr.write(|w| unsafe { w.bits(sample as u32) });

            // wait for space if needed
            while !i2s.fifostat.read().txnotfull().bit_is_set() {}

            // ✅ Right channel
            i2s.fifowr.write(|w| unsafe { w.bits(sample as u32) });

            idx = (idx + 1) & (SINE_LUT.len() - 1);
            count += 1;
            if count.is_multiple_of(48000) {
                defmt::debug!("frames sent: {}", count)
            }
        }
    }
}

struct Clock {}
impl Clock {
    const RATES: [RangeEntry<u32>; 1] = [RangeEntry::new_fixed(48000)];
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

struct Audio {
    running: RefCell<bool>,
    i2s: I2sTx,
    queue: RefCell<heapless::spsc::Queue<(u32, u32), 352>>,
}
impl Audio {
    fn poll(&self) {
        if !*self.running.borrow() {
            return;
        }
        let stat = self.i2s.i2s.fifostat.read();

        if stat.txerr().bit_is_set() {
            self.i2s.i2s.fifostat.modify(|_, w| w.txerr().set_bit());
            // defmt::error!("fifo tx error, txlvl: {}", stat.txlvl().bits());
        }
        if stat.txlvl().bits() <= 6 {
            // fifo is 8 deep
            if let Some(sample) = self.queue.borrow_mut().dequeue() {
                self.i2s
                    .i2s
                    .fifowr
                    .write(|w| unsafe { w.bits(sample.0 as u32) });
                self.i2s
                    .i2s
                    .fifowr
                    .write(|w| unsafe { w.bits(sample.1 as u32) });
            } else {
                // defmt::error!("queue underflow");
                self.i2s.i2s.fifowr.write(|w| unsafe { w.bits(0 as u32) });
            }
        }
    }
}
impl<'a, B: bus::UsbBus> UsbAudioClass<'a, B> for Audio {
    fn alternate_setting_changed<CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>>(
        &self,
        ac: &mut usbd_uac2::AudioClass<'a, B, CS, AU>,
        terminal: usb_device::UsbDirection,
        alt_setting: u8,
    ) {
        match alt_setting {
            0 => *self.running.borrow_mut() = false,
            1 => *self.running.borrow_mut() = true,
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
                u32::from_le_bytes(b[..4].try_into().unwrap()),
                u32::from_le_bytes(b[4..].try_into().unwrap()),
            )
        }) {
            self.queue.borrow_mut().enqueue(sample).ok(); // TODO: ok is not ok here, it means we have overflowed the 
        }
    }
}

// copied from NXP SDK WM8904_Init
fn init_codec<T>(i2c: &mut T)
where
    T: _embedded_hal_blocking_i2c_WriteRead + _embedded_hal_blocking_i2c_Write,
{
    let mut buf = [0u8; 2];
    match i2c.write_read(CODEC_I2C_ADDR, &[0], &mut buf) {
        Ok(_) => {
            let chip_id = ((buf[0] as u16) << 8) | buf[1] as u16;
            defmt::debug!("Read chip ID: {:x}", chip_id)
        }
        Err(_) => defmt::error!("Error reading I2C"),
    }

    i2c.write(CODEC_I2C_ADDR, &[0x16, 0x00, 0x0f]).ok(); // clock rates 2 = OPCLK_ENA | CLK_SYS_ENA | CLK_DSP_ENA | TOCLK_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x6c, 0x01, 0x00]).ok(); // write sequencer 0 ENA
    i2c.write(CODEC_I2C_ADDR, &[0x6f, 0x01, 0x00]).ok(); // write sequencer 3 START, INDEX=0
    // wait on write sequencer
    defmt::debug!("[codec] waiting on write seq");
    loop {
        let mut buf = [0; 2];
        i2c.write_read(CODEC_I2C_ADDR, &[0x70], &mut buf).ok();
        if buf[1] & 1 == 0 {
            break;
        }
    }
    defmt::debug!("[codec] write seq done");
    i2c.write(CODEC_I2C_ADDR, &[0x14, 0x00, 0x00]).ok(); // clock rates 0
    i2c.write(CODEC_I2C_ADDR, &[0x0c, 0x00, 0x03]).ok(); // power management 0 = INL_ENA | INR_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x0e, 0x00, 0x03]).ok(); // power management 2 = HPL_PGA_ENA | HPR_PGA_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x0f, 0x00, 0x03]).ok(); // power management 3 = LINEOUTL_ENA | LINEOUTR_ENA

    i2c.write(CODEC_I2C_ADDR, &[0x12, 0x00, 0x0f]).ok(); // power management 6 = DACL_ENA | DACR_ENA | ADCL_ENA | ADCR_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x0a, 0x00, 0x01]).ok(); // analog adc 0 = ADC_OSR128
    i2c.write(CODEC_I2C_ADDR, &[0x18, 0x00, 0x50]).ok(); // audio if 0 = AIFADCR_SRC | AIFDACR_SRC
    i2c.write(CODEC_I2C_ADDR, &[0x21, 0x00, 0x40]).ok(); // dac digital 1 = DAC_OSR128
    i2c.write(CODEC_I2C_ADDR, &[0x2c, 0x00, 0x05]).ok(); // analog lin 0 = 0dB (unmute)
    i2c.write(CODEC_I2C_ADDR, &[0x2d, 0x00, 0x05]).ok(); // analog rin 0 = 0dB (unmute)
    i2c.write(CODEC_I2C_ADDR, &[0x39, 0x00, 0x39]).ok(); // analog out1 left = vol=0dB
    i2c.write(CODEC_I2C_ADDR, &[0x3a, 0x00, 0x39]).ok(); // analog out1 right = vol=0dB
    i2c.write(CODEC_I2C_ADDR, &[0x3b, 0x00, 0x39]).ok(); // analog out2 left = vol=0dB
    i2c.write(CODEC_I2C_ADDR, &[0x3c, 0x00, 0x39]).ok(); // analog out2 right = vol=0dB
    i2c.write(CODEC_I2C_ADDR, &[0x43, 0x00, 0x03]).ok(); // dc server 0 = HPOUTL_ENA | HPOUTR_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x5a, 0x00, 0xff]).ok(); // analog hp 0 = remove all shorts etc
    i2c.write(CODEC_I2C_ADDR, &[0x5e, 0x00, 0xff]).ok(); // analog lineout 0 = remove all shorts etc
    i2c.write(CODEC_I2C_ADDR, &[0x68, 0x00, 0x01]).ok(); // enable class w charge pump
    i2c.write(CODEC_I2C_ADDR, &[0x62, 0x00, 0x01]).ok(); // enable charge pump
    i2c.write(CODEC_I2C_ADDR, &[0x19, 0x00, 0x0e]).ok(); // audio if 1 = i2s, 32 bits mode
    i2c.write(CODEC_I2C_ADDR, &[0x15, (0x05 << 2), 0x05]).ok(); // sys clock rate 512fs, sample rate 48
    i2c.write(CODEC_I2C_ADDR, &[0x16, 0x00, 0x0f]).ok(); // clock rates 2 = CLK_SYS_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x1a, 0x00, 0x08]).ok(); // audio interface 2 = no gpio, sysclk / 8
    i2c.write(CODEC_I2C_ADDR, &[0x1b, 0x00, 0x00]).ok(); // audio interface 3 = input lrclock
    i2c.write(CODEC_I2C_ADDR, &[0x3d, 0x00, 0x00]).ok(); // analog out12 zc = play source = dac
    i2c.write(CODEC_I2C_ADDR, &[0x1e, 0x01, 0xff]).ok(); // dac vol left = update left/right = 0dB
}

pub struct I2sTx {
    pub i2s: I2S7,
}

pub fn init_i2s(mut fc7: FLEXCOMM7, mut i2s7: I2S7, syscon: &mut Syscon) -> I2sTx {
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
            .modify(|_, w| w.sel().enum_0x0()); // FRO 96MHz
        pac::SYSCON::ptr()
            .as_ref()
            .unwrap()
            .mclkdiv
            .modify(|_, w| w.div().bits(3).halt().run().reset().released()); // div by 4 = 24MHz
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

    regs.cfg2
        .modify(|_, w| unsafe { w.position().bits(0).framelen().bits(63) });

    regs.div.modify(|_, w| unsafe { w.div().bits(7) }); // Clock source is MCLK (24MHz on FRO96) / 8 = 3MHz

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
    // TODO: figure out how to configure the PLL for a more suitable audio clock.
    let clocks = hal::ClockRequirements::default()
        // .system_frequency(24.mhz())
        // .system_frequency(72.mhz())
        .system_frequency(96.MHz())
        .configure(&mut anactrl, &mut pmc, &mut syscon)
        .unwrap();
    let mut _delay_timer = Timer::new(
        hal.ctimer
            .0
            .enabled(&mut syscon, clocks.support_1mhz_fro_token().unwrap()),
    );

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
    init_codec(&mut i2c_bus);

    // i2s_sine_test(&i2s_peripheral.i2s);
    let audio = Audio {
        i2s: i2s_peripheral,
        queue: RefCell::new(heapless::spsc::Queue::new()),
        running: RefCell::new(false),
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
        audio.poll();
        red_led.set_high().ok(); // Turn off
    }
}
