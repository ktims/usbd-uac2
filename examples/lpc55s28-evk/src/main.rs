#![no_main]
#![no_std]

// extern crate panic_semihosting;
extern crate panic_halt;
use core::cell::OnceCell;
use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_io::{ErrorType, Write};
use nb;

#[allow(unused_imports)]
use hal::prelude::*;
#[allow(unused_imports)]
use lpc55_hal as hal;
use lpc55_hal::{
    drivers::{
        Serial,
        pins::{PinId, Pio0_29, Pio0_30},
        serial::Tx,
    },
    peripherals::flexcomm::Usart0,
    typestates::pin::{
        flexcomm::{Usart, UsartPins},
        function::{FC0_RXD_SDA_MOSI_DATA, FC0_TXD_SCL_MISO_WS},
        state::Special,
    },
};

use core::convert::Infallible;
use defmt;
use defmt_rtt as _;
use hal::drivers::{Timer, UsbBus, pins};
use static_cell::StaticCell;
use usb_device::{
    bus,
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    endpoint::IsochronousSynchronizationType,
};
use usbd_uac2::{
    self, AudioClassConfig, RangeEntry, TerminalConfig, USB_CLASS_AUDIO, UsbAudioClass,
    UsbAudioClockImpl, UsbSpeed,
    constants::{FunctionCode, TerminalType},
    descriptors::{ChannelConfig, ClockSource, ClockType, FormatType1, LockDelay},
};

type SERIAL_RX_PIN = hal::Pin<Pio0_29, Special<FC0_RXD_SDA_MOSI_DATA>>;
type SERIAL_TX_PIN = hal::Pin<Pio0_30, Special<FC0_TXD_SCL_MISO_WS>>;
type SERIAL_PINS = (SERIAL_TX_PIN, SERIAL_RX_PIN);

static SERIAL: StaticCell<DefmtUart<Usart0>> = StaticCell::new();

pub struct DefmtUart<U>(pub Tx<U>)
where
    U: Usart;

impl<U> ErrorType for DefmtUart<U>
where
    U: Usart,
{
    type Error = Infallible;
}
impl<U> Write for DefmtUart<U>
where
    U: Usart,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        buf.iter().map(|c| nb::block!(self.0.write(*c))).last();
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // Blocking write, so flush is a no-op
        Ok(())
    }
}

struct Clock {}
impl Clock {
    const RATES: [RangeEntry<u32>; 1] = [RangeEntry::new_fixed(44100)];
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
}

struct Audio {}
impl<'a, B: bus::UsbBus> UsbAudioClass<'a, B> for Audio {}

#[entry]
fn main() -> ! {
    let hal = hal::new();

    let mut anactrl = hal.anactrl;
    let mut pmc = hal.pmc;
    let mut syscon = hal.syscon;

    let mut gpio = hal.gpio.enabled(&mut syscon);
    let mut iocon = hal.iocon.enabled(&mut syscon);

    let mut red_led = pins::Pio1_6::take()
        .unwrap()
        .into_gpio_pin(&mut iocon, &mut gpio)
        .into_output(hal::drivers::pins::Level::Low); // start turned on

    let usb0_vbus_pin = pins::Pio0_22::take()
        .unwrap()
        .into_usb0_vbus_pin(&mut iocon);

    let serial_rx_pin = pins::Pio0_29::take()
        .unwrap()
        .into_usart0_rx_pin(&mut iocon);
    let serial_tx_pin = pins::Pio0_30::take()
        .unwrap()
        .into_usart0_tx_pin(&mut iocon);

    iocon.disabled(&mut syscon).release(); // save the environment :)

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

    let usart = hal
        .flexcomm
        .0
        .enabled_as_usart(&mut syscon, &clocks.support_flexcomm_token().unwrap());

    let serial_config = hal::drivers::serial::config::Config::default().speed(115_200.Hz());
    let serial = Serial::new(usart, (serial_tx_pin, serial_rx_pin), serial_config).split();
    let serial_tx = DefmtUart(serial.0);

    // defmt_serial::defmt_serial(SERIAL.init(serial_tx));

    let usb_peripheral = hal.usbhs.enabled_as_device(
        &mut anactrl,
        &mut pmc,
        &mut syscon,
        &mut _delay_timer,
        clocks.support_usbhs_token().unwrap(),
    );

    let usb_bus = UsbBus::new(usb_peripheral, usb0_vbus_pin);
    let clock = Clock {};
    let audio = Audio {};

    let config = AudioClassConfig::new(UsbSpeed::High, FunctionCode::Other, &clock, &audio)
        .with_input_config(TerminalConfig::new(
            2,
            1,
            2,
            FormatType1 {
                bit_resolution: 24,
                bytes_per_sample: 4,
            },
            TerminalType::ExtLineConnector,
            ChannelConfig::default_chans(2),
            IsochronousSynchronizationType::Adaptive,
            LockDelay::Milliseconds(10),
            None,
        ))
        .with_output_config(TerminalConfig::new(
            4,
            1,
            2,
            FormatType1 {
                bit_resolution: 24,
                bytes_per_sample: 4,
            },
            TerminalType::ExtLineConnector,
            ChannelConfig::default_chans(2),
            IsochronousSynchronizationType::Adaptive,
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
    let mut need_zlp = false;
    let mut buf = [0u8; 8];
    let mut size = 0;
    let mut buf_in_use = false;
    loop {
        // if !usb_dev.poll(&mut []) {
        // if !usb_dev.poll(&mut [&mut serial]) {
        if !usb_dev.poll(&mut [&mut uac2]) {
            continue;
        }

        // let mut buf = [0u8; 512];

        // match serial.read(&mut buf) {
        //     Ok(count) if count > 0 => {
        //         assert!(count == 1);
        //         // hprintln!("received some data on the serial port: {:?}", &buf[..count]).ok();
        //         // cortex_m_semihosting::hprintln!("received:\n{}", core::str::from_utf8(&buf[..count]).unwrap()).ok();
        //         red_led.set_low().ok(); // Turn on

        //         // cortex_m_semihosting::hprintln!("read {:?}", &buf[..count]).ok();
        //         cortex_m_semihosting::hprintln!("read {:?}", count).ok();

        //         // Echo back in upper case
        //         for c in buf[0..count].iter_mut() {
        //             if (0x61 <= *c && *c <= 0x7a) || (0x41 <= *c && *c <= 0x5a) {
        //                 *c ^= 0x20;
        //             }
        //         }

        //         let mut write_offset = 0;
        //         while write_offset < count {
        //             match serial.write(&buf[write_offset..count]) {
        //                 Ok(len) if len > 0 => {
        //                     write_offset += len;
        //                     cortex_m_semihosting::hprintln!("wrote {:?}", len).ok();

        //                 },
        //                 _ => {},
        //             }
        //         }

        //         // hprintln!("wrote it back").ok();
        //     }
        //     _ => {}
        // }

        red_led.set_high().ok(); // Turn off
    }
}
