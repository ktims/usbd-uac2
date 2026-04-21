use lpc55_pac as pac;
use usbd_uac2::{self, UsbSpeed, UsbSpeedProvider};

struct Lpc55UsbSpeedProvider {}

impl UsbSpeedProvider for Lpc55UsbSpeedProvider {
    fn speed(&self) -> usbd_uac2::UsbSpeed {
        let regs = unsafe { &*pac::USB1::ptr() };
        match regs.devcmdstat.read().speed().bits() {
            1 => UsbSpeed::Full,
            2 => UsbSpeed::High,
            3 => UsbSpeed::Super,
            _ => panic!("Unknown USB speed"),
        }
    }
}

fn main() {
    println!("Hello, world!");
}
