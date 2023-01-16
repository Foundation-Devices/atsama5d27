//! Power Management Controller.

use utralib::utra::pmc::{PMC_PCR, PMC_PCR_PID};
use utralib::*;

/// Peripheral ID in the AT91 system.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum PeripheralId {
    Fiq = 0, /* FIQ Interrupt ID */

    #[doc(hidden)]
    Reserved1 = 1, /* Reserved */

    Arm = 2,       /* Performance Monitor Unit */
    Pit = 3,       /* Periodic Interval Timer Interrupt */
    Wdt = 4,       /* Watchdog Timer Interrupt */
    Gmac = 5,      /* Ethernet MAC */
    Xdmac0 = 6,    /* DMA Controller 0 */
    Xdmac1 = 7,    /* DMA Controller 1 */
    Icm = 8,       /* Integrity Check Monitor */
    Aes = 9,       /* Advanced Encryption Standard */
    Aesb = 10,     /* AES bridge */
    Tdes = 11,     /* Triple Data Encryption Standard */
    Sha = 12,      /* SHA Signature */
    Mpddrc = 13,   /* MPDDR Controller */
    Matrix1 = 14,  /* H32MX, 32-bit AHB Matrix */
    Matrix0 = 15,  /* H64MX, 64-bit AHB Matrix */
    Secumod = 16,  /* Secure Module */
    Hsmc = 17,     /* Multi-bit ECC interrupt */
    Pioa = 18,     /* Parallel I/O Controller A */
    Flexcom0 = 19, /* FLEXCOM0 */
    Flexcom1 = 20, /* FLEXCOM1 */
    Flexcom2 = 21, /* FLEXCOM2 */
    Flexcom3 = 22, /* FLEXCOM3 */
    Flexcom4 = 23, /* FLEXCOM4 */
    Uart0 = 24,    /* UART0 */
    Uart1 = 25,    /* UART1 */
    Uart2 = 26,    /* UART2 */
    Uart3 = 27,    /* UART3 */
    Uart4 = 28,    /* UART4 */
    Twi0 = 29,     /* Two-wire Interface 0 */
    Twi1 = 30,     /* Two-wire Interface 1 */
    Sdmmc0 = 31,   /* Secure Data Memory Card Controller 0 */
    Sdmmc1 = 32,   /* Secure Data Memory Card Controller 1 */
    Spi0 = 33,     /* Serial Peripheral Interface 0 */
    Spi1 = 34,     /* Serial Peripheral Interface 1 */
    Tc0 = 35,      /* Timer Counter 0 (ch. 0,1,2) */
    Tc1 = 36,      /* Timer Counter 1 (ch. 3,4,5) */

    #[doc(hidden)]
    Reserved37 = 37, /* Reserved */

    Pwm = 38, /* Pulse Width Modulation Controller0 (ch. 0,1,2,3) */

    #[doc(hidden)]
    Reserved39 = 39, /* Reserved */

    Adc = 40,         /* Touch Screen ADC Controller */
    Uhphs = 41,       /* USB Host High Speed */
    Udphs = 42,       /* USB Device High Speed */
    Ssc0 = 43,        /* Serial Synchronous Controller 0 */
    Ssc1 = 44,        /* Serial Synchronous Controller 1 */
    Lcdc = 45,        /* LCD Controller */
    Isi = 46,         /* Image Sensor Interface */
    Trng = 47,        /* True Random Number Generator */
    Pdmic = 48,       /* Pulse Density Modulation Interface Controller */
    Irq = 49,         /* IRQ Interrupt ID */
    Sfc = 50,         /* Fuse Controller */
    Securam = 51,     /* Secure RAM */
    Qspi0 = 52,       /* QSPI0 */
    Qspi1 = 53,       /* QSPI1 */
    I2sc0 = 54,       /* Inter-IC Sound Controller 0 */
    I2sc1 = 55,       /* Inter-IC Sound Controller 1 */
    Can0Int0 = 56,    /* MCAN 0 Interrupt0 */
    Can1Int0 = 57,    /* MCAN 1 Interrupt0 */
    Ptc = 58,         /* Peripheral Touch Controller */
    Classd = 59,      /* Audio Class D Amplifier */
    Sfr = 60,         /* Special Function Register */
    Saic = 61,        /* Secured Advanced Interrupt Controller */
    Aic = 62,         /* Advanced Interrupt Controller */
    L2cc = 63,        /* L2 Cache Controller */
    Can0Int1 = 64,    /* MCAN 0 Interrupt1 */
    Can1Int1 = 65,    /* MCAN 1 Interrupt1 */
    GmacQ1 = 66,      /* GMAC Queue 1 Interrupt */
    GmacQ2 = 67,      /* GMAC Queue 2 Interrupt */
    Piob = 68,        /* Parallel I/O Controller B */
    Pioc = 69,        /* Parallel I/O Controller C */
    Piod = 70,        /* Parallel I/O Controller D */
    Sdmmc0Timer = 71, /* Secure Data Memory Card Controller 0 */
    Sdmmc1Timer = 72, /* Secure Data Memory Card Controller 1 */

    #[doc(hidden)]
    Reserved73 = 73, /* Reserved */

    Sys = 74,    /* System Controller Interrupt */
    Acc = 75,    /* Analog Comparator */
    Rxlp = 76,   /* UART Low-Power */
    Sfrbu = 77,  /* Special Function Register BackUp */
    Chipid = 78, /* Chip ID */
}

impl From<u8> for PeripheralId {
    fn from(value: u8) -> Self {
        use PeripheralId::*;
        match value {
            0 => Fiq,
            1 => Reserved1,
            2 => Arm,
            3 => Pit,
            4 => Wdt,
            5 => Gmac,
            6 => Xdmac0,
            7 => Xdmac1,
            8 => Icm,
            9 => Aes,
            10 => Aesb,
            11 => Tdes,
            12 => Sha,
            13 => Mpddrc,
            14 => Matrix0,
            15 => Matrix1,
            16 => Secumod,
            17 => Hsmc,
            18 => Pioa,
            19 => Flexcom0,
            20 => Flexcom1,
            21 => Flexcom2,
            22 => Flexcom3,
            23 => Flexcom4,
            24 => Uart0,
            25 => Uart1,
            26 => Uart2,
            27 => Uart3,
            28 => Uart4,
            29 => Twi0,
            30 => Twi1,
            31 => Sdmmc0,
            32 => Sdmmc1,
            33 => Spi0,
            34 => Spi1,
            35 => Tc0,
            36 => Tc1,
            37 => Reserved37,
            38 => Pwm,
            39 => Reserved39,
            40 => Adc,
            41 => Uhphs,
            42 => Udphs,
            43 => Ssc0,
            44 => Ssc1,
            45 => Lcdc,
            46 => Isi,
            47 => Trng,
            48 => Pdmic,
            49 => Irq,
            50 => Sfc,
            51 => Securam,
            52 => Qspi0,
            53 => Qspi1,
            54 => I2sc0,
            55 => I2sc1,
            56 => Can0Int0,
            57 => Can1Int0,
            58 => Ptc,
            59 => Classd,
            60 => Sfr,
            61 => Saic,
            62 => Aic,
            63 => L2cc,
            64 => Can0Int0,
            65 => Can1Int1,
            66 => GmacQ1,
            67 => GmacQ2,
            68 => Piob,
            69 => Pioc,
            70 => Piod,
            71 => Sdmmc0Timer,
            72 => Sdmmc1Timer,
            73 => Reserved73,
            74 => Sys,
            75 => Acc,
            76 => Rxlp,
            77 => Sfrbu,
            78 => Chipid,

            _ => panic!("Invalid peripheral ID: {}", value),
        }
    }
}

const PMC_PCR_PID_MASK: u32 = 0x3F;
const PMC_PCR_DIV_SET: u32 = 0x3_u32 << 16;
const PMC_PCR_EN_SET: u32 = 0x1 << 28;
const PMC_PCR_CMD_SET: u32 = 0x1 << 12;

pub struct Pmc {
    base_addr: u32,
}

impl Default for Pmc {
    fn default() -> Pmc {
        Self::new()
    }
}

impl Pmc {
    pub fn new() -> Self {
        Self {
            base_addr: HW_PMC_BASE as u32,
        }
    }

    /// Creates PMC instance with a different base address. Used with virtual memory
    pub fn with_alt_base_addr(base_addr: u32) -> Self {
        Self { base_addr }
    }

    /// Turns on the peripheral's clock source.
    pub fn enable_peripheral_clock(&mut self, pid: PeripheralId) {
        let mut pmc_csr = CSR::new(self.base_addr as *mut u32);
        pmc_csr.wfo(PMC_PCR_PID, pid as u32 & PMC_PCR_PID_MASK);

        let mut val = pmc_csr.r(PMC_PCR);
        val &= !PMC_PCR_DIV_SET;
        val |= PMC_PCR_CMD_SET | PMC_PCR_EN_SET;

        pmc_csr.wo(PMC_PCR, val);
    }

    /// Disables the peripheral's clock source.
    pub fn disable_peripheral_clock(&mut self, pid: PeripheralId) {
        let mut pmc_csr = CSR::new(self.base_addr as *mut u32);
        pmc_csr.wfo(PMC_PCR_PID, pid as u32 & PMC_PCR_PID_MASK);

        let mut val = pmc_csr.r(PMC_PCR);
        val &= !PMC_PCR_EN_SET;
        val |= PMC_PCR_CMD_SET;

        pmc_csr.wo(PMC_PCR, val);
    }
}
