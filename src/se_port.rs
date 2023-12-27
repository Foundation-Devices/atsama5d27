use {
    crate::uart::{Uart, Uart1},
    core::fmt::Write,
    sha2::{Digest, Sha256},
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
#[repr(u16)]
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
pub fn setup_config(secrets: &RomSecrets) -> Result<(), Error> {
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

/// The tempkey stored on the SE chip and used for various operations.
#[derive(Debug, Clone, Copy)]
struct TempKey([u8; 32]);

/// Load Tempkey with a nonce value that we both know, but
/// is random and we both know is random! Tricky!
fn se_pick_nonce(num_in: &[u8; 20]) -> Result<TempKey, Error> {
    let mut rand_out = [0; 32];
    unsafe {
        // Nonce command returns the RNG result, but not contents of TempKey
        cryptoauthlib::atcab_nonce_rand(num_in.as_ptr(), rand_out.as_mut_ptr()).into_result()?;
    }
    // Hash stuff appropriately to get same number as chip did.
    // TempKey on the chip will be set to the output of SHA256 over
    // a message composed of my challenge, the RNG and 3 bytes of constants:
    //
    // return sha256(rndout + num_in + b'\x16\0\0').digest()
    let mut hasher = Sha256::new();
    hasher.update(rand_out);
    hasher.update(num_in);
    hasher.update(&[0x16, 0, 0]);
    let tempkey = hasher.finalize();
    let mut result = [0; 32];
    result.copy_from_slice(tempkey.as_slice());
    Ok(TempKey(result))
}

/// CAUTION: The result from this function could be modified by an
/// active attacker on the bus because the one-byte response from the chip
/// is easily replaced. This command is useful for us to authorize actions
/// inside the 508a/608a, like use of a specific key, but not for us to
/// authenticate the 508a/608a or its contents/state.
fn se_checkmac(slot: Slot, secret: &[u8; 32]) -> Result<(), Error> {
    // TODO These two should be random in KeyOS
    let od = [1; 32];
    let num_in = [2; 20];
    let tempkey = se_pick_nonce(&num_in).unwrap();
    // Hash nonce and lots of other bits together
    let mut hasher = Sha256::new();
    hasher.update(secret);
    hasher.update(tempkey.0);
    hasher.update(&od[0..4]);
    hasher.update(&[0; 8]);
    hasher.update(&od[4..7]);
    hasher.update(&[0xEE]);
    hasher.update(&od[7..11]);
    hasher.update(&[0x01, 0x23]);
    hasher.update(&od[11..13]);
    let mut resp = [0; 32];
    resp.copy_from_slice(hasher.finalize().as_slice());
    let rc = unsafe {
        // Content doesn't matter, but nice and visible:
        let challenge = b"(C) 2020 Foundation Devices Inc.";
        cryptoauthlib::atcab_checkmac(
            0x01,
            slot as u16,
            challenge.as_ptr(),
            resp.as_ptr(),
            od.as_ptr(),
        )
        .into_result()?;
    };
    Ok(())
}

/// Do on-chip hashing, with lots of iterations.
///
/// - using HMAC-SHA256 with keys that are known only to the 608a.
/// - rate limiting factor here is communication time w/ 608a, not algos.
/// - caution: result here is not confidential
/// - cost of each iteration, approximately: 8ms
/// - but our time to do each iteration is limited by software SHA256 in se_pair_unlock
pub fn se_stretch_iter(
    mut msg: [u8; 32],
    iterations: usize,
    secrets: &RomSecrets,
) -> Result<[u8; 32], Error> {
    let mut digest = [0; 32];
    for _ in 0..iterations {
        unsafe {
            // Must unlock again, because pin_stretch is an auth'd key.
            se_pair_unlock(secrets)?;

            // Start SHA w/ HMAC setup
            let status = cryptoauthlib::atcab_sha_base(
                4,
                Slot::PinStretch as u16,
                core::ptr::null(),
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
            .into_result()?;
            let mut digest_len = digest.len() as u16;
            // Send the contents to be hashed. Place the result in the output buffer.
            let status = cryptoauthlib::atcab_sha_base(
                (3 << 6) | 2,
                msg.len() as u16,
                msg.as_ptr(),
                digest.as_mut_ptr(),
                (&mut digest_len) as *mut u16,
            )
            .into_result()?;
            msg = digest;
        }
    }
    Ok(digest)
}

fn se_pair_unlock(secrets: &RomSecrets) -> Result<(), Error> {
    const MAX_ATTEMPTS: usize = 3;
    for _ in 0..MAX_ATTEMPTS - 1 {
        if se_checkmac(Slot::PairingSecret, &secrets.pairing_secret).is_ok() {
            return Ok(());
        }
    }
    se_checkmac(Slot::PairingSecret, &secrets.pairing_secret)
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

impl Error {
    pub fn is_checkmac_fail(&self) -> bool {
        self.0 == cryptoauthlib::ATCA_CHECKMAC_VERIFY_FAILED
    }
}
