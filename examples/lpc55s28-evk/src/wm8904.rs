use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use defmt::warn;

use crate::{CODEC_I2C_ADDR, MCLK_FREQ, SAMPLE_RATE, SampleType};

// copied from NXP SDK WM8904_Init
pub(crate) fn init_codec<T>(i2c: &mut T)
where
    T: _embedded_hal_blocking_i2c_WriteRead + _embedded_hal_blocking_i2c_Write,
{
    let mut buf = [0u8; 2];
    match i2c.write_read(CODEC_I2C_ADDR, &[0], &mut buf) {
        Ok(_) => {
            let chip_id = ((buf[0] as u16) << 8) | buf[1] as u16;
            defmt::info!("Read chip ID: {:x}", chip_id)
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
    i2c.write(CODEC_I2C_ADDR, &[0x0c, 0x00, 0x00]).ok(); // power management 0 = IN PGAs disabled
    i2c.write(CODEC_I2C_ADDR, &[0x0e, 0x00, 0x03]).ok(); // power management 2 = HPL_PGA_ENA | HPR_PGA_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x0f, 0x00, 0x00]).ok(); // power management 3 = line outs disabled

    i2c.write(CODEC_I2C_ADDR, &[0x12, 0x00, 0x0c]).ok(); // power management 6 = DACL_ENA | DACR_ENA
    i2c.write(CODEC_I2C_ADDR, &[0x0a, 0x00, 0x00]).ok(); // analog adc 0 = ADC_OSR128
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
    let aif_wl = match core::mem::size_of::<SampleType>() {
        4 => 0, // 16 bits per sample
        8 => 3, // 32 bits per sample
        _ => {
            warn!("only handling 16 or 32 bit samples for now");
            8
        }
    };
    i2c.write(CODEC_I2C_ADDR, &[0x19, 0x00, (aif_wl << 2) | 2])
        .ok(); // audio if 1 = i2s, aif_wl

    // Calculate sysclk vs. fs ratio. SYSCLK = MCLK
    let fs_ratio = MCLK_FREQ / SAMPLE_RATE;
    if !MCLK_FREQ.is_multiple_of(SAMPLE_RATE) {
        warn!("sample rate should be a multiple of mclk")
    }
    let clk_sys_rate: u16 = match fs_ratio {
        64 => 0,
        128 => 1,
        192 => 2,
        256 => 3,
        384 => 4,
        512 => 5,
        768 => 6,
        1024 => 7,
        1408 => 8,
        1536 => 9,
        _ => {
            warn!("unsupport ratio {}", fs_ratio);
            0
        }
    };
    let sample_rate: u16 = match SAMPLE_RATE {
        r if r < 11025 => 0, // 0-11024
        r if r < 16000 => 1, // 11025 - 15999
        r if r < 22050 => 2, // 16000 - 22049
        r if r < 32000 => 3, // 22050 - 31999
        r if r < 44100 => 4, // 32000 - 44099
        _ => 5,              // 44100+
    };
    let clock_rates_1 = ((clk_sys_rate << 10) | sample_rate).to_be_bytes();

    i2c.write(CODEC_I2C_ADDR, &[0x15, clock_rates_1[0], clock_rates_1[1]])
        .ok(); // sys clock rate 512fs, sample rate 48

    i2c.write(CODEC_I2C_ADDR, &[0x16, 0x00, 0x0f]).ok(); // clock rates 2 = CLK_SYS_ENA

    // Calculate bclk_div
    let bits_per_frame = core::mem::size_of::<SampleType>() * 8;
    let bits_per_second = bits_per_frame as u32 * SAMPLE_RATE;
    let bclk_div = MCLK_FREQ / bits_per_second;
    i2c.write(CODEC_I2C_ADDR, &[0x1a, 0x00, bclk_div as u8])
        .ok(); // audio interface 2 = no gpio, bclk_div

    i2c.write(CODEC_I2C_ADDR, &[0x1b, 0x00, 0x00]).ok(); // audio interface 3 = input lrclock
    i2c.write(CODEC_I2C_ADDR, &[0x3d, 0x00, 0x00]).ok(); // analog out12 zc = play source = dac
    i2c.write(CODEC_I2C_ADDR, &[0x1e, 0x01, 0xff]).ok(); // dac vol left = update left/right = 0dB
}
