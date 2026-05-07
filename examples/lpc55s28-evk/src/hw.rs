//! Contains hardware setup unrelated to Usb Audio Class implementation

use crate::Syscon;
use crate::{MCLK_FREQ, SAMPLE_RATE, pac};
use defmt::debug;
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
            .modify(|_, w| w.div().bits(1).halt().run().reset().released()); // div by 2 = PLL0 fout / 2 = 12.288MHz, max for WM8904 @ 96k
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
        .modify(|_, w| unsafe { w.position().bits(0).framelen().bits(63) }); // framelen = 64

    let bclk_div = (MCLK_FREQ / SAMPLE_RATE / 64) as u16;
    regs.div
        .modify(|_, w| unsafe { w.div().bits(bclk_div - 1) }); // Clock source is MCLK (12.288MHz) / 4 = 3MHz

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
