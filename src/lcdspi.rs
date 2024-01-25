//! 9-bit LCD-specific bit-banged SPI

use crate::{
    pit::Pit,
    spi::{BitsPerTransfer, ChipSelect, Spi, SpiMode},
};

const SPI_FREQ_HZ: u32 = 10_000_000;

/// LCD configuration implementation. Uses SPI for communication and PIT for generating
/// delays.
pub struct LcdSpi {
    spi: Spi,
    cs: ChipSelect,
    pit: Pit,
    curr_clock_freq: u32,
}

impl LcdSpi {
    /// Create instance
    pub fn new(mut spi: Spi, cs: ChipSelect, curr_clock_freq: u32, pit: Pit) -> Self {
        spi.init();
        spi.init_cs(cs, BitsPerTransfer::Bits9, SpiMode::Mode0, true);
        spi.set_bitrate(curr_clock_freq, cs, SPI_FREQ_HZ);
        spi.master_enable(true);
        spi.set_enabled(true);

        LcdSpi {
            spi,
            cs,
            curr_clock_freq,
            pit,
        }
    }

    pub fn send_command(&mut self, cmd: u8) {
        self.send_bits(true, cmd);
    }

    pub fn send_data(&mut self, dat: u8) {
        self.send_bits(false, dat);
    }

    fn send_bits(&mut self, is_cmd: bool, bits: u8) {
        let data = (!is_cmd as u16) << 8 | bits as u16;
        self.spi.select_cs(self.cs);
        self.spi.write_hw(data).expect("send 9-bit word");
        let _ = self.spi.read_hw().expect("dummy read");
        self.spi.release_cs();
    }

    pub fn run_init_sequence(&mut self) {
        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x13);

        self.send_command(0xEF);
        self.send_data(0x08);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x10);

        self.send_command(0xC0);
        self.send_data(0x63);
        self.send_data(0x00);

        self.send_command(0xC1); // Porch control
        self.send_data(0x10); // VBP
        self.send_data(0x02); // VFP

        self.send_command(0xC2);
        self.send_data(0x01);
        self.send_data(0x02);

        self.send_command(0xCC);
        self.send_data(0x10);

        self.send_command(0xB0);
        self.send_data(0xC0);
        self.send_data(0x0C);
        self.send_data(0x92);
        self.send_data(0x0C);
        self.send_data(0x10);
        self.send_data(0x05);
        self.send_data(0x02);
        self.send_data(0x0D);
        self.send_data(0x07);
        self.send_data(0x21);
        self.send_data(0x04);
        self.send_data(0x53);
        self.send_data(0x11);
        self.send_data(0x6A);
        self.send_data(0x32);
        self.send_data(0x1F);

        self.send_command(0xB1);
        self.send_data(0xC0);
        self.send_data(0x87);
        self.send_data(0xCF);
        self.send_data(0x0C);
        self.send_data(0x10);
        self.send_data(0x06);
        self.send_data(0x00);
        self.send_data(0x03);
        self.send_data(0x08);
        self.send_data(0x1D);
        self.send_data(0x06);
        self.send_data(0x54);
        self.send_data(0x12);
        self.send_data(0xE6);
        self.send_data(0xEC);
        self.send_data(0x0F);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x11);

        self.send_command(0xB0);
        self.send_data(0x5D);

        self.send_command(0xB1);
        self.send_data(0x52);

        self.send_command(0xB2);
        self.send_data(0x82);

        self.send_command(0xB3);
        self.send_data(0x80);

        self.send_command(0xB5);
        self.send_data(0x42);

        self.send_command(0xB7);
        self.send_data(0x85);

        self.send_command(0xB8);
        self.send_data(0x20);

        self.send_command(0xC0);
        self.send_data(0x09);

        self.send_command(0xC1);
        self.send_data(0x78);

        self.send_command(0xC2);
        self.send_data(0x78);

        self.send_command(0xD0);
        self.send_data(0x88);

        self.send_command(0xEE);
        self.send_data(0x42);

        self.pit.busy_wait_ms(self.curr_clock_freq, 100);

        self.send_command(0xE0);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x02);

        self.send_command(0xE1);
        self.send_data(0x04);
        self.send_data(0xA0);
        self.send_data(0x06);
        self.send_data(0xA0);
        self.send_data(0x05);
        self.send_data(0xA0);
        self.send_data(0x07);
        self.send_data(0xA0);
        self.send_data(0x00);
        self.send_data(0x44);
        self.send_data(0x44);

        self.send_command(0xE2);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x33);
        self.send_data(0x33);
        self.send_data(0x01);
        self.send_data(0xA0);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x01);
        self.send_data(0xA0);
        self.send_data(0x00);
        self.send_data(0x00);

        self.send_command(0xE3);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x33);
        self.send_data(0x33);

        self.send_command(0xE4);
        self.send_data(0x44);
        self.send_data(0x44);

        self.send_command(0xE5);
        self.send_data(0x0C);
        self.send_data(0x30);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x0E);
        self.send_data(0x32);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x08);
        self.send_data(0x2C);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x0A);
        self.send_data(0x2E);
        self.send_data(0xA0);
        self.send_data(0xA0);

        self.send_command(0xE6);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x33);
        self.send_data(0x33);

        self.send_command(0xE7);
        self.send_data(0x44);
        self.send_data(0x44);

        self.send_command(0xE8);
        self.send_data(0x0D);
        self.send_data(0x31);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x0F);
        self.send_data(0x33);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x09);
        self.send_data(0x2D);
        self.send_data(0xA0);
        self.send_data(0xA0);
        self.send_data(0x0B);
        self.send_data(0x2F);
        self.send_data(0xA0);
        self.send_data(0xA0);

        self.send_command(0xEB);
        self.send_data(0x00);
        self.send_data(0x01);
        self.send_data(0xE4);
        self.send_data(0xE4);
        self.send_data(0x44);
        self.send_data(0x88);
        self.send_data(0x00);

        self.send_command(0xED);
        self.send_data(0xFF);
        self.send_data(0xF5);
        self.send_data(0x47);
        self.send_data(0x6F);
        self.send_data(0x0B);
        self.send_data(0xA1);
        self.send_data(0xA2);
        self.send_data(0xBF);
        self.send_data(0xFB);
        self.send_data(0x2A);
        self.send_data(0x1A);
        self.send_data(0xB0);
        self.send_data(0xF6);
        self.send_data(0x74);
        self.send_data(0x5F);
        self.send_data(0xFF);

        self.send_command(0xEF);
        self.send_data(0x08);
        self.send_data(0x08);
        self.send_data(0x08);
        self.send_data(0x40);
        self.send_data(0x3F);
        self.send_data(0x64);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x13);

        self.send_command(0xE8);
        self.send_data(0x00);
        self.send_data(0x0E);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x00);

        self.send_command(0x11);
        self.pit.busy_wait_ms(self.curr_clock_freq, 200);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x13);

        self.send_command(0xE8);
        self.send_data(0x00);
        self.send_data(0x0C);
        self.pit.busy_wait_ms(self.curr_clock_freq, 10);

        self.send_command(0xE8);
        self.send_data(0x00);
        self.send_data(0x00);

        self.send_command(0xFF);
        self.send_data(0x77);
        self.send_data(0x01);
        self.send_data(0x00);
        self.send_data(0x00);
        self.send_data(0x00);

        self.send_command(0x3A); // COLMOD
        self.send_data(0x70); // 24 bit

        self.send_command(0x29); // DISPON

        self.pit.busy_wait_ms(self.curr_clock_freq, 50);
    }
}
