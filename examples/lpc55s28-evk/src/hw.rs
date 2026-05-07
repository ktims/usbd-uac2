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
