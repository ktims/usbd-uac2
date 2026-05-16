//! Contains hardware setup unrelated to Usb Audio Class implementation

use crate::Syscon;
use crate::hal;
use crate::{MCLK_FREQ, SAMPLE_RATE, pac};

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use defmt::debug;
use hal::{
    Enabled, Iocon, Pin,
    drivers::pins,
    peripherals::syscon::{ClockControl, ResetControl},
    traits::wg::digital::v2::{OutputPin, ToggleableOutputPin},
    typestates::pin::{gpio::direction::Output, state::Gpio},
};

pub(crate) struct PllConstants {
    pub m: u16,   // 1-65535
    pub n: u8,    // 1-255
    pub p: u8,    // 1-31
    pub selp: u8, // 5 bits
    pub seli: u8, // 6 bits
}

impl PllConstants {
    pub(crate) const fn new(n: u8, m: u16, p: u8) -> Self {
        assert!(n != 0, "1 <= N <= 255");
        assert!(m != 0, "1 <= M <= 65535");
        assert!(p != 0 && p <= 31, "1 <= P <= 31");

        // Following ripped from lpc55-hal and made const
        // UM 4.6.6.3.2
        let selp = {
            let v = (m >> 2) + 1;
            if v < 31 { v } else { 31 }
        } as u8;

        let seli = {
            let v = match m {
                m if m >= 8000 => 1,
                m if m >= 122 => 8000 / m,
                _ => 2 * (m >> 2) + 3,
            };

            if v < 63 { v } else { 63 }
        } as u8;
        // let seli = min(2*(m >> 2) + 3, 63);
        Self {
            n,
            m,
            p,
            selp,
            seli,
        }
    }
}
impl defmt::Format for PllConstants {
    fn format(&self, fmt: defmt::Formatter) {
        let factor = f32::from(self.m) / (f32::from(self.n) * 2.0 * f32::from(self.p));

        defmt::write!(
            fmt,
            "m: {} n: {} p: {} selp: {} seli: {} fout: fin * {}",
            self.m,
            self.n,
            self.p,
            self.selp,
            self.seli,
            factor
        );
    }
}

// Fo = M/(N*2*P) * Fin
// Fo = 3072/(125*2*8) * 16MHz = 24.576MHz
const AUDIO_PLL: PllConstants = PllConstants::new(125, 3072, 8);

// Set PLL0 to 24.576MHz, start, and wait for lock
// This is not exposed by lpc55-hal, unfortunately. Copy their implementation here.
pub(crate) fn init_audio_pll() {
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

    debug!("init pll0: {}", AUDIO_PLL);
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

pub struct I2sHandles {
    pub tx: pac::I2S7,
    pub rx: pac::I2S6,
}

pub fn init_i2s(
    fc7: pac::FLEXCOMM7,
    i2s7: pac::I2S7,
    fc6: pac::FLEXCOMM6,
    i2s6: pac::I2S6,
    syscon: &mut Syscon,
) -> I2sHandles {
    defmt::debug!("init i2s");
    // Enable BOTH
    fc7.clear_reset(syscon);
    fc7.enable_clock(syscon);
    fc6.clear_reset(syscon);
    fc6.enable_clock(syscon);
    {
        let sc = unsafe { pac::SYSCON::ptr().as_ref().unwrap() };
        let ioc = unsafe { pac::IOCON::ptr().as_ref().unwrap() };
        // MCLK source
        //
        sc.mclkclksel.write(|w| w.sel().enum_0x1()); // PLL0
        // MCLK div
        sc.mclkdiv
            .write(|w| unsafe { w.div().bits(1).halt().run().reset().released() }); // div by 2 = PLL0 fout / 2 = 12.288MHz, max for WM8904 @ 96k
        // MCLK out config
        ioc.pio1_31.modify(|_, w| {
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
        // FC7 clock
        sc.fcclksel7().modify(|_, w| w.sel().enum_0x5()); // MCLK
        // FC6 clock
        sc.fcclksel6().modify(|_, w| w.sel().enum_0x5()); // MCLK
        // MCLK out
        sc.mclkio.modify(|_, w| w.mclkio().output());

        // Enable clock for sysctl, it's not mapped into Peripherals
        sc.ahbclkctrlset[2].write(|w| unsafe { w.bits(1 << 15) });
        while sc.ahbclkctrl2.read().sysctl().is_disable() {}
        let sysctrl = unsafe { pac::SYSCTL::ptr().as_ref().unwrap() };
        sysctrl.sharedctrlset[0].write(|w| w.sharedscksel().flexcomm7().sharedwssel().flexcomm7()); // FC7 drives shared SCK, WS
        sysctrl.fcctrlsel[7].write(|w| {
            w.sckinsel()
                .shared_set0_i2s_signals()
                .wsinsel()
                .shared_set0_i2s_signals()
        }); // FC7 uses shared set
        sysctrl.fcctrlsel[6].write(|w| {
            w.sckinsel()
                .shared_set0_i2s_signals()
                .wsinsel()
                .shared_set0_i2s_signals()
        });

        // for _ in 0..1000 {
        //     cortex_m::asm::nop();
        // }
    }

    // Select I2S TX function
    fc7.pselid.write(|w| w.persel().i2s_transmit());
    // Select I2S RX function
    fc6.pselid.write(|w| w.persel().i2s_receive());

    let out_regs = i2s7;
    let in_regs = i2s6;

    // Enable TX FIFO only
    out_regs.fifocfg.write(|w| {
        w.enabletx()
            .enabled()
            .txi2se0() // transmit 0s when empty - only supported option for 32b data
            .zero()
            .emptytx() // reset the tx queue
            .set_bit()
    });

    out_regs
        .cfg2
        .write(|w| unsafe { w.position().bits(0).framelen().bits(63) }); // framelen = 64

    let bclk_div = (MCLK_FREQ / SAMPLE_RATE / 64) as u16;
    out_regs
        .div
        .write(|w| unsafe { w.div().bits(bclk_div - 1) }); // Clock source is MCLK (12.288MHz) / 4 = 3MHz

    // TX Config
    out_regs.cfg1.write(|w| unsafe {
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

    // Enable RX FIFO only
    in_regs
        .fifocfg
        .write(|w| w.enablerx().enabled().emptyrx().set_bit());
    in_regs
        .cfg2
        .write(|w| unsafe { w.position().bits(0).framelen().bits(63) }); // framelen = 64
    in_regs.div.write(|w| unsafe { w.div().bits(0) });
    in_regs.cfg1.write(|w| unsafe {
        w.mstslvcfg()
            .normal_slave_mode()
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

    I2sHandles {
        tx: out_regs,
        rx: in_regs,
    }
}

pub struct SharedLed<T: OutputPin> {
    inner: UnsafeCell<T>,
}
unsafe impl<T: OutputPin> Sync for SharedLed<T> {}
impl<T: OutputPin> SharedLed<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
        }
    }
    pub fn on(&self) {
        unsafe {
            (*self.inner.get()).set_low().ok();
        }
    }
    pub fn off(&self) {
        unsafe {
            (*self.inner.get()).set_high().ok();
        }
    }
}
impl<T: OutputPin + ToggleableOutputPin> SharedLed<T> {
    pub fn toggle(&self) {
        unsafe {
            (*self.inner.get()).toggle().ok();
        }
    }
}

type RedLed = Pin<pins::Pio1_6, Gpio<Output>>;
type GreenLed = Pin<pins::Pio1_7, Gpio<Output>>;
type BlueLed = Pin<pins::Pio1_4, Gpio<Output>>;
pub static RED_LED: MaybeUninit<SharedLed<RedLed>> = MaybeUninit::uninit();
pub static GREEN_LED: MaybeUninit<SharedLed<GreenLed>> = MaybeUninit::uninit();
pub static BLUE_LED: MaybeUninit<SharedLed<BlueLed>> = MaybeUninit::uninit();

pub fn init_leds(iocon: &mut Iocon<Enabled>, gpio: &mut hal::Gpio<Enabled>) {
    let red_led = SharedLed::new(
        pins::Pio1_6::take()
            .unwrap()
            .into_gpio_pin(iocon, gpio)
            .into_output_low(),
    );
    let green_led = SharedLed::new(
        pins::Pio1_7::take()
            .unwrap()
            .into_gpio_pin(iocon, gpio)
            .into_output_low(),
    );
    let blue_led = SharedLed::new(
        pins::Pio1_4::take()
            .unwrap()
            .into_gpio_pin(iocon, gpio)
            .into_output_low(),
    );
    unsafe {
        core::ptr::write(RED_LED.as_ptr() as *mut SharedLed<RedLed>, red_led);
        core::ptr::write(GREEN_LED.as_ptr() as *mut SharedLed<GreenLed>, green_led);
        core::ptr::write(BLUE_LED.as_ptr() as *mut SharedLed<BlueLed>, blue_led);
    }
}
pub fn red_led() -> &'static SharedLed<RedLed> {
    unsafe { &*RED_LED.as_ptr() }
}
pub fn green_led() -> &'static SharedLed<GreenLed> {
    unsafe { &*GREEN_LED.as_ptr() }
}
pub fn blue_led() -> &'static SharedLed<BlueLed> {
    unsafe { &*BLUE_LED.as_ptr() }
}
