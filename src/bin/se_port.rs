use {
    atsama5d27::uart::{Uart, Uart1},
    core::fmt::Write,
    cryptoauthlib::ATCAPacket,
};

/// Bytes [16..84) of chip config area.
const SE_CONFIG_1: [u8; 68] = [
    0xe1, 0x00, 0x61, 0x00, 0x00, 0x00, 0x8f, 0x80, 0x8f, 0x80, 0x8f, 0x43, 0xaf, 0x80, 0x00, 0x43,
    0x00, 0x43, 0x8f, 0x80, 0x00, 0x00, 0xc3, 0x43, 0x00, 0x43, 0xce, 0x4e, 0x00, 0x00, 0x00, 0x00,
    0x8f, 0x4e, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
];

/// Bytes [90..128) of chip config area.
const SE_CONFIG_2: [u8; 38] = [
    0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x5c, 0x00, 0xbc, 0x01, 0xfc, 0x01, 0xbc, 0x01,
    0x9c, 0x01, 0x9c, 0x01, 0xbc, 0x01, 0x3c, 0x00, 0xdc, 0x03, 0x9c, 0x01, 0xdc, 0x01, 0x3c, 0x00,
    0x3c, 0x00, 0xdc, 0x01, 0x3c, 0x00,
];

/// Slot numbers for the SE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slot {
    PairingSecret = 1,
    PinStretch = 2,
    PinHash = 3,
    PinAttempt = 4,
    LastGood = 5,
    MatchCount = 6,
    SupplyChain = 7,
    Seed = 9,
    UserFirmwarePubkey = 10,
    FirmwareTimestamp = 11,
    FirmwareHash = 14,
}

#[derive(Debug, Clone)]
pub struct RomSecrets {
    pub pairing_secret: [u8; 32],
    pub serial_number: [u8; 9],
    pub otp_key: [u8; 72],
    pub hash_cache_secret: [u8; 32],
}

/// SE block size in bytes.
const BLOCK_SIZE: usize = 32;

/// One-time config and lockdown of the chip
///
/// CONCERN: Must not be possible to call this function after replacing
/// the chip deployed originally. But key secrets would have been lost
/// by then anyway... looks harmless, and regardless once the datazone
/// is locked, none of this code will work... but:
///
/// IMPORTANT: If they blocked the real chip, and provided a blank one for
/// us to write the (existing) pairing secret into, they would see the pairing
/// secret in cleartext. They could then restore original chip and access freely.
///
/// PASSPORT NOTE: We can eliminate the above by having the factory bootloader
///                be different than the normal bootloader. The factory bootloader
///                will have the one-time setup code only, not the runtime code.
///                The normal bootloader will NOT have the one-time setup code,
///                but WILL have the main runtime code. So swapping in blank
///                SE would not trigger us to write the pairing secret in the clear.
pub fn setup_config(secrets: RomSecrets) -> Result<(), Error> {
    unsafe {
        let mut config = [0; 128];
        cryptoauthlib::atcab_read_config_zone(config.as_mut_ptr()).into_result()?;
        cryptoauthlib::hal_delay_ms(100);

        // Setup steps:
        // - write config zone data
        // - lock that
        // - write pairing secret (test it works)
        // - pick RNG value for words secret (and forget it)
        // - set all PIN values to known value (zeros)
        // - set all money secrets to knonw value (zeros)
        // - lock the data zone
        if config[87] == 0x55 {
            // config is still unlocked
            config[16..16 + SE_CONFIG_1.len()].copy_from_slice(&SE_CONFIG_1);
            config[90..90 + SE_CONFIG_2.len()].copy_from_slice(&SE_CONFIG_2);

            cryptoauthlib::atcab_write_config_zone(config.as_ptr()).into_result()?;
            cryptoauthlib::hal_delay_ms(100);

            cryptoauthlib::atcab_lock_config_zone_crc(crc16(&config)).into_result()?;
            cryptoauthlib::hal_delay_ms(100);
        }

        if config[86] == 0x55 {
            // data is still unlocked

            let unlocked = u16::from_le_bytes([config[88], config[89]]);

            // TODO Might need to turn this into a proper function
            let write_slot = |slot: Slot, data: &[u8]| -> Result<(), Error> {
                if unlocked & (1 << (slot as u8)) == 0 {
                    return Ok(());
                }
                assert!(data.len() % BLOCK_SIZE == 0);
                for (i, c) in data.chunks(BLOCK_SIZE).enumerate() {
                    let i = i as u16;
                    let slot = slot as u16;
                    unsafe {
                        cryptoauthlib::atcab_write(
                            0x80 | 2,
                            (i << 8) | (slot << 3),
                            c.as_ptr(),
                            core::ptr::null(),
                        )
                        .into_result()?;
                    }
                    cryptoauthlib::hal_delay_ms(100);
                }
                Ok(())
            };

            let lock_slot = |slot: Slot| -> Result<(), Error> {
                if unlocked & (1 << (slot as u8)) == 0 {
                    return Ok(());
                }
                cryptoauthlib::atcab_lock_data_slot(slot as u16).into_result()?;
                cryptoauthlib::hal_delay_ms(100);
                Ok(())
            };

            let write_slot_random = |slot: Slot| -> Result<(), Error> {
                // TODO In KeyOS, this should actually be random
                let data = [0x00; BLOCK_SIZE];
                write_slot(slot, &data)
            };

            write_slot(Slot::PairingSecret, &secrets.pairing_secret)?;

            write_slot_random(Slot::PinStretch)?;
            lock_slot(Slot::PinStretch)?;

            write_slot_random(Slot::PinAttempt)?;
            lock_slot(Slot::PinAttempt)?;

            write_slot(Slot::PinHash, &[0; BLOCK_SIZE])?;
            write_slot(Slot::LastGood, &[0; BLOCK_SIZE])?;
            write_slot(Slot::FirmwareTimestamp, &[0; BLOCK_SIZE])?;
            write_slot(Slot::FirmwareHash, &[0; BLOCK_SIZE])?;

            // TODO Not sure what to do with the supply chain slot, in Gen 2 it reads
            // flash, what should be done here?

            write_slot(Slot::Seed, &[0; BLOCK_SIZE * 3])?;
            write_slot(Slot::UserFirmwarePubkey, &[0; BLOCK_SIZE * 3])?;

            let mut match_count = [0u8; BLOCK_SIZE];
            match_count[0..4].copy_from_slice(1024u32.to_le_bytes().as_ref());
            match_count[4..8].copy_from_slice(1024u32.to_le_bytes().as_ref());
            write_slot(Slot::MatchCount, &match_count)?;

            cryptoauthlib::atcab_lock_data_zone().into_result()?;
            cryptoauthlib::hal_delay_ms(100);
        }

        Ok(())
    }
}

fn crc16(data: &[u8]) -> u16 {
    let poly = 0x8005u16;
    let mut crc = 0u16;
    for d in data {
        for i in 0..8 {
            let data_bit = (d >> i) & 1;
            let crc_bit = (crc >> 15) as u8;
            crc <<= 1;
            if data_bit != crc_bit {
                crc ^= poly;
            }
        }
    }
    crc
}

#[derive(Debug)]
pub struct Error(pub i32);

trait AtcaStatusIntoResult {
    fn into_result(self) -> Result<(), Error>;
}

impl AtcaStatusIntoResult for i32 {
    fn into_result(self) -> Result<(), Error> {
        if self == 0 {
            Ok(())
        } else {
            Err(Error(self))
        }
    }
}
