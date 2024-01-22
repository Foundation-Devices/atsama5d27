// TODO Leave a comment explaining where all this code came from (pins.c).

use zeroize::ZeroizeOnDrop;

use {
    core::fmt::Write,
    sha2::{Digest, Sha256},
};

// TODO Move the hardcoded error constants (Error(-1), etc) into const members of Error

// Number of iterations for KDF
const KDF_ITER_WORDS: usize = 2;
const KDF_ITER_PIN: usize = 8; // about ? seconds (measured in-system)
const SE_SECRET_LEN: usize = 72;

const OP_GENDIG: u8 = 0x15;
const OP_WRITE: u8 = 0x12;
const OP_MAC: u8 = 0x08;

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

// TODO There are probably other places in this module that could benefit from zeroize
#[derive(ZeroizeOnDrop)]
pub struct Pin(pub [u8; 32]);

#[derive(ZeroizeOnDrop)]
pub struct Secret(pub [u8; SE_SECRET_LEN]);

/// Slot numbers for the SE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
enum Slot {
    None = 0,
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

/// Pretty sure it doesn't matter, but adding some salt into our PIN->bytes[32] code
/// based on the purpose of the PIN code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
enum PinPurpose {
    Normal = 0x334d1858,
    AntiPhishWords = 0x2e6d6773,
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
pub fn setup_config(secrets: &mut RomSecrets) -> Result<(), Error> {
    unsafe {
        let mut config = with_retries(|| {
            let mut config = [0; 128];
            cryptoauthlib::atcab_read_config_zone(config.as_mut_ptr()).into_result()?;
            Ok(config)
        })?;

        with_retries(|| {
            cryptoauthlib::atcab_read_serial_number(secrets.serial_number.as_mut_ptr())
                .into_result()
        })?;

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

            with_retries(|| cryptoauthlib::atcab_write_config_zone(config.as_ptr()).into_result())?;

            with_retries(|| {
                cryptoauthlib::atcab_lock_config_zone_crc(crc16(&config)).into_result()
            })?;
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
                    with_retries(|| {
                        cryptoauthlib::atcab_write(
                            0x80 | 2,
                            (i << 8) | (slot << 3),
                            c.as_ptr(),
                            core::ptr::null(),
                        )
                        .into_result()
                    })?;
                }
                Ok(())
            };

            let lock_slot = |slot: Slot| -> Result<(), Error> {
                if unlocked & (1 << (slot as u8)) == 0 {
                    return Ok(());
                }
                with_retries(|| cryptoauthlib::atcab_lock_data_slot(slot as u16).into_result())?;
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

            with_retries(|| cryptoauthlib::atcab_lock_data_zone().into_result())?;
        }

        Ok(())
    }
}

/// The tempkey stored on the SE chip and used for various operations.
#[derive(Debug, Clone, Copy)]
struct TempKey([u8; 32]);

/// Load Tempkey with a nonce value that we both know, but
/// is random and we both know is random! Tricky!
fn se_pick_nonce(num_in: [u8; 20]) -> Result<TempKey, Error> {
    let rand_out = unsafe {
        // Nonce command returns the RNG result, but not contents of TempKey
        with_retries(|| {
            let mut rand_out = [0; 32];
            cryptoauthlib::atcab_random(rand_out.as_mut_ptr()).into_result()?;
            Ok(rand_out)
        })?
    };
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

// TODO This is NOT the problem
/// CAUTION: The result from this function could be modified by an
/// active attacker on the bus because the one-byte response from the chip
/// is easily replaced. This command is useful for us to authorize actions
/// inside the 508a/608a, like use of a specific key, but not for us to
/// authenticate the 508a/608a or its contents/state.
fn se_checkmac(slot: Slot, secret: [u8; 32]) -> Result<(), Error> {
    // TODO The first 13 bytes of this should be random, the rest should be zero
    let od = [0; 32];
    // TODO This should be random in KeyOS
    let num_in = [2; 20];
    let tempkey = se_pick_nonce(num_in)?;
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
    unsafe {
        with_retries(|| {
            // Content doesn't matter, but nice and visible:
            let challenge = b"(C) 2020 Foundation Devices Inc.";
            cryptoauthlib::atcab_checkmac(
                0x01,
                slot as u16,
                challenge.as_ptr(),
                resp.as_ptr(),
                od.as_ptr(),
            )
            .into_result()
        })?;
    };
    Ok(())
}

/// Check the chip produces a hash over various things the same way we would
/// meaning that we both know the shared secret and the state of stuff in
/// the 508a is what we expect.
fn se_checkmac_hard(slot: Slot, secret: [u8; 32], secrets: &RomSecrets) -> Result<(), Error> {
    let digest = se_gendig_slot(slot, secret)?;
    // NOTE: we use this sometimes when we know the value is wrong, like
    // checking for blank pin codes... so not a huge error/security issue
    // if wrong here.
    if !se_is_correct_tempkey(digest, secrets)? {
        Err(Error(-2))
    } else {
        Ok(())
    }
}

fn se_gendig_slot(slot: Slot, slot_contents: [u8; 32]) -> Result<[u8; 32], Error> {
    // TODO In KeyOS, this should actually be random
    let num_in = [0x00; 20];
    let tempkey = se_pick_nonce(num_in)?;

    unsafe {
        with_retries(|| {
            // Using Zone=2="Data" => "KeyID specifies a slot in the Data zone".
            cryptoauthlib::atcab_gendig(0x2, slot as u16, core::ptr::null(), 0).into_result()
        })?;
    }

    // We now have to match the digesting (hashing) that has happened on
    // the chip. No feedback at this point if it's right tho.
    //
    //   msg = hkey + b'\x15\x02' + ustruct.pack("<H", slot_num)
    //   msg += b'\xee\x01\x23' + (b'\0'*25) + challenge
    //   assert len(msg) == 32+1+1+2+1+2+25+32
    let mut hasher = Sha256::new();
    let args = [OP_GENDIG, 2, slot as u8, 0, 0xEE, 0x01, 0x23];
    hasher.update(slot_contents);
    hasher.update(args);
    hasher.update([0u8; 25]);
    hasher.update(tempkey.0);
    let mut digest = [0; 32];
    digest.copy_from_slice(hasher.finalize().as_slice());
    Ok(digest)
}

/// Construct a digest over one of the two counters. Track what we think
/// the digest should be, and ask the chip to do the same. Verify we match
/// using MAC command (done elsewhere).
fn se_gendig_counter(counter_num: u16, expected_value: u32) -> Result<[u8; 32], Error> {
    // TODO In KeyOS, this should actually be random
    let num_in = [0; 20];
    let tempkey = se_pick_nonce(num_in)?;

    unsafe {
        with_retries(|| {
            // Using Zone=4="Counter" => "KeyID specifies the monotonic counter ID".
            cryptoauthlib::atcab_gendig(0x4, counter_num, core::ptr::null(), 0).into_result()
        })?;
    }

    // we now have to match the digesting (hashing) that has happened on
    // the chip. No feedback at this point if it's right tho.
    //
    //   msg = hkey + b'\x15\x02' + ustruct.pack("<H", slot_num)
    //   msg += b'\xee\x01\x23' + (b'\0'*25) + challenge
    //   assert len(msg) == 32+1+1+2+1+2+25+32
    //
    let args = [OP_GENDIG, 0x4, counter_num as u8, 0, 0xEE, 0x01, 0x23, 0x0];
    let mut hasher = Sha256::new();
    hasher.update([0; 32]);
    hasher.update(args);
    hasher.update(expected_value.to_le_bytes());
    hasher.update([0; 20]);
    hasher.update(tempkey.0);
    let mut digest = [0; 32];
    digest.copy_from_slice(hasher.finalize().as_slice());
    Ok(digest)
}

/// Check that TempKey is holding what we think it does. Uses the MAC
/// command over contents of Tempkey and our shared secret.
fn se_is_correct_tempkey(expected_tempkey: [u8; 32], secrets: &RomSecrets) -> Result<bool, Error> {
    let mode: u8 = (1 << 6)   // include full serial number
                 | (0 << 2)   // TempKey.SourceFlag == 0 == 'rand'
                 | (0 << 1)   // first 32 bytes are the shared secret
                 | (1 << 0); // second 32 bytes are tempkey

    let resp = unsafe {
        with_retries(|| {
            let mut resp = [0; 32];
            cryptoauthlib::atcab_mac(
                mode,
                Slot::PairingSecret as u16,
                core::ptr::null(),
                (&mut resp).as_mut_ptr(),
            )
            .into_result()?;
            Ok(resp)
        })?
    };

    // Duplicate the hash process, and then compare.
    let mut hasher = Sha256::new();
    hasher.update(&secrets.pairing_secret);
    hasher.update(&expected_tempkey);

    let fixed = [
        OP_MAC,
        mode,
        Slot::PairingSecret as u8,
        0x0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // eight zeros
        0,
        0,
        0, // three zeros
        0xEE,
    ];
    hasher.update(fixed);
    hasher.update(&secrets.serial_number[4..8]);
    hasher.update(&secrets.serial_number[..4]);

    let mut actual = [0; 32];
    actual.copy_from_slice(hasher.finalize().as_slice());

    Ok(constant_time_eq::constant_time_eq_32(&actual, &resp))
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
    for _ in 0..iterations {
        // Must unlock again, because pin_stretch is an auth'd key.
        se_pair_unlock(secrets)?;
        msg = se_hmac32(Slot::PinStretch, msg)?;
    }
    Ok(msg)
}

#[derive(Debug, Clone)]
pub enum LoginAttempt {
    Success(CachedMainPin),
    Failure {
        num_fails: u32,
        attempts_left: u32,
        reason: LoginFailureReason,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoginFailureReason {
    IncorrectPin,
    SeFailure,
}

struct LastSuccess {
    num_fails: u32,
    attempts_left: u32,
}

/// Do the PIN check.
pub fn pin_login_attempt(pin: &Pin, secrets: &RomSecrets) -> Result<LoginAttempt, Error> {
    // Hash up the pin now, assuming we'll use it on main PIN.
    let mid_digest = pin_hash_attempt(Slot::None, Some(pin), secrets)?;
    // Do mixin.
    let digest = se_mixin_key(Slot::None, mid_digest, secrets)?;
    // Register an attempt on the pin.
    let mut digest = se_mixin_key(Slot::PinAttempt, mid_digest, secrets)?;

    let pin_kn = Slot::PinHash;
    let secret_kn = Slot::Seed;

    if !is_main_pin(digest, secrets)? {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "!is_main_pin"
        )
        .ok();
        // PIN code is just wrong.
        // - nothing to update, since the chip's done it already
        return Ok(match get_last_success(secrets) {
            Ok(LastSuccess {
                num_fails,
                attempts_left,
            }) => LoginAttempt::Failure {
                num_fails,
                attempts_left,
                reason: LoginFailureReason::IncorrectPin,
            },
            Err(_) => LoginAttempt::Failure {
                num_fails: 0,
                attempts_left: 0,
                reason: LoginFailureReason::IncorrectPin,
            },
        });
    }

    // change the various counters, since this worked
    if updates_for_good_login(digest, secrets).is_err() {
        // Update args with latest attempts remaining
        // TODO Same as above
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "updates_for_good_login"
        )
        .ok();
        let LastSuccess {
            num_fails,
            attempts_left,
        } = get_last_success(secrets)?;
        //printf("GOOD PIN: attempts_left: %lu  num_fails: %lu\n", args->attempts_left, args->num_fails);
        return Ok(LoginAttempt::Failure {
            num_fails,
            attempts_left,
            reason: LoginFailureReason::SeFailure,
        });
    }

    // SUCCESS! "digest" holds a working value. Save it.
    let cached_pin = pin_cache_save(digest, secrets)?;

    let mut ts = [0u8; SE_SECRET_LEN];
    // TODO This is intentionally all zeros
    let num_in = [0; 20];

    unsafe {
        with_retries(|| {
            cryptoauthlib::atcab_read_enc(
                secret_kn as u16,
                0,
                ts[0..32].as_mut_ptr(),
                digest.as_mut_ptr(),
                pin_kn as u16,
                num_in.as_ptr(),
            )
            .into_result()
        })?;
        with_retries(|| {
            cryptoauthlib::atcab_read_enc(
                secret_kn as u16,
                1,
                ts[32..64].as_mut_ptr(),
                digest.as_mut_ptr(),
                pin_kn as u16,
                num_in.as_ptr(),
            )
            .into_result()
        })?;
        with_retries(|| {
            cryptoauthlib::atcab_read_enc(
                secret_kn as u16,
                2,
                ts[64..].as_mut_ptr(),
                digest.as_mut_ptr(),
                pin_kn as u16,
                num_in.as_ptr(),
            )
            .into_result()
        })?;
    }

    Ok(LoginAttempt::Success(cached_pin))
}

pub fn change_pin(old_pin: Option<&Pin>, new_pin: &Pin, secrets: &RomSecrets) -> Result<(), Error> {
    // What pin do they need to know to make their change?
    let required_kn = Slot::PinHash;
    // What slot (key number) are updating?
    let target_slot = Slot::PinHash;

    // No real need to re-prove PIN knowledge.
    // If they tricked us to get to this point, doesn't matter as
    // below the SE validates it all again.

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "calling pin_hash_attempt"
    )
    .ok();

    // Determine they know hash protecting the pin to be changed.
    let required_digest = pin_hash_attempt(required_kn, old_pin, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called pin_hash_attempt"
    )
    .ok();

    // Check the old pin provided, is right.
    se_pair_unlock(secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_pair_unlock"
    )
    .ok();
    if se_checkmac(required_kn, required_digest).is_err() {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "se_checkmac error"
        )
        .ok();
        // they got old PIN wrong, we won't be able to help them
        // NOTE: altho we are changing flow based on result of ae_checkmac() here,
        // if the response is faked by an active bus attacker, it doesn't matter
        // because the change to the dataslot below will fail due to wrong PIN.
        return Err(Error::OLD_AUTH_FAIL);
    }

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "se_checkmac ok"
    )
    .ok();

    // Calculate new PIN hashed value: will be slow for main pin.
    let new_digest = pin_hash_attempt(required_kn, Some(new_pin), secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "got new_digest"
    )
    .ok();
    let num_in = [0; 20];
    se_encrypted_write(
        target_slot,
        required_kn,
        required_digest,
        &new_digest,
        secrets,
    )?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_encrypted_write"
    )
    .ok();

    // Main pin is changing; reset counter to zero (good login) and our cache.
    pin_cache_save(new_digest, secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called pin_cache_save"
    )
    .ok();
    updates_for_good_login(new_digest, secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called updates_for_good_login"
    )
    .ok();

    Ok(())
}

pub fn change_secret(
    old_pin: Option<&Pin>,
    new_pin: &Pin,
    secrets: &RomSecrets,
) -> Result<(), Error> {
    // What pin do they need to know to make their change?
    let required_kn = Slot::PinHash;
    // What slot (key number) are updating?
    let target_slot = Slot::PinHash;

    // No real need to re-prove PIN knowledge.
    // If they tricked us to get to this point, doesn't matter as
    // below the SE validates it all again.

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "calling pin_hash_attempt"
    )
    .ok();

    // Determine they know hash protecting the pin to be changed.
    let required_digest = pin_hash_attempt(required_kn, old_pin, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called pin_hash_attempt"
    )
    .ok();

    // Check the old pin provided, is right.
    se_pair_unlock(secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_pair_unlock"
    )
    .ok();
    if se_checkmac(required_kn, required_digest).is_err() {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "se_checkmac error"
        )
        .ok();
        // they got old PIN wrong, we won't be able to help them
        // NOTE: altho we are changing flow based on result of ae_checkmac() here,
        // if the response is faked by an active bus attacker, it doesn't matter
        // because the change to the dataslot below will fail due to wrong PIN.
        return Err(Error::OLD_AUTH_FAIL);
    }

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "se_checkmac ok"
    )
    .ok();

    // Calculate new PIN hashed value: will be slow for main pin.
    let new_digest = pin_hash_attempt(required_kn, Some(new_pin), secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "got new_digest"
    )
    .ok();
    // TODO This is intentionally all zeros
    let num_in = [0; 20];
    se_encrypted_write(
        target_slot,
        required_kn,
        required_digest,
        &new_digest,
        secrets,
    )?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_encrypted_write"
    )
    .ok();

    // Main pin is changing; reset counter to zero (good login) and our cache.
    pin_cache_save(new_digest, secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called pin_cache_save"
    )
    .ok();
    updates_for_good_login(new_digest, secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called updates_for_good_login"
    )
    .ok();

    Ok(())
}

fn se_encrypted_write(
    data_slot: Slot,
    write_kn: Slot,
    write_key: [u8; 32],
    data: &[u8],
    secrets: &RomSecrets,
) -> Result<(), Error> {
    for (num_blk, blk) in data.chunks(32).enumerate() {
        let mut tmp = [0; 32];
        tmp[..blk.len()].copy_from_slice(blk);
        se_encrypted_write32(data_slot, num_blk, write_kn, write_key, tmp, secrets)?;
    }
    Ok(())
}

fn se_encrypted_write32(
    data_slot: Slot,
    blk: usize,
    write_kn: Slot,
    write_key: [u8; 32],
    data: [u8; 32],
    secrets: &RomSecrets,
) -> Result<(), Error> {
    se_pair_unlock(secrets)?;

    // Generate a hash over shared secret and rng.
    let digest = se_gendig_slot(write_kn, write_key)?;

    // encrypt the data to be written.
    let mut body = [0; 32];

    body.iter_mut()
        .zip(data.iter())
        .zip(digest.iter())
        .for_each(|((b, d), k)| {
            *b = *d ^ *k;
        });

    // make auth-mac to go with
    //    SHA-256(TempKey, Opcode, Param1, Param2, SN<8>, SN<0:1>, <25 bytes of zeros>, PlainTextData)
    //    msg = (dig
    //        + ustruct.pack('<bbH', OP.Write, args['p1'], args['p2'])
    //        + b'\xee\x01\x23'
    //        + (b'\0'*25)
    //        + new_value)
    //    assert len(msg) == 32+1+1+2+1+2+25+32
    //
    let mut hasher = Sha256::new();

    let p1 = 0x80 | 2; // 32 bytes into a data slot
    let p2_lsb = ((data_slot as u16) << 3) as u8;
    let p2_msb = blk as u8;

    let args = [OP_WRITE, p1, p2_lsb, p2_msb, 0xEE, 0x01, 0x23];

    hasher.update(digest);
    hasher.update(args);
    hasher.update([0; 25]);
    hasher.update(data);

    let mut mac = [0; 32];
    mac.copy_from_slice(hasher.finalize().as_slice());

    unsafe {
        with_retries(|| {
            cryptoauthlib::atcab_write(
                p1,
                ((p2_msb as u16) << 8) | p2_lsb as u16,
                body.as_ptr(),
                mac.as_ptr(),
            )
            .into_result()
        })
    }
}

/// Read state about previous attempt(s) from AE. Calculate number of failures,
/// and how many attempts are left. The need for verifing the values from AE is
/// not really so strong with the 608a, since it's all enforced on that side, but
/// we'll do it anyway.
fn get_last_success(secrets: &RomSecrets) -> Result<LastSuccess, Error> {
    let slot = Slot::LastGood;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "inside get_last_success!"
    )
    .ok();

    se_pair_unlock(secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_pair_unlock"
    )
    .ok();

    // Read counter value of last-good login. Important that this be authenticated.
    // - using first 32-bits only, others will be zero
    let padded = unsafe {
        with_retries(|| {
            let mut padded = [0; 32];
            cryptoauthlib::atcab_read_zone(2, slot as u16, 0, 0, padded.as_mut_ptr(), 32)
                .into_result()?;
            Ok(padded)
        })?
    };

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called atcab_read_zone"
    )
    .ok();

    se_pair_unlock(secrets)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called second se_pair_unlock"
    )
    .ok();
    let tempkey = se_gendig_slot(slot, padded)?;
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_gendig_slot"
    )
    .ok();

    if !se_is_correct_tempkey(tempkey, secrets)? {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "!se_is_correct_tempkey"
        )
        .ok();
        return Err(Error(-3));
    }

    // Read two values from data slots
    let lastgood = read_slot_as_counter(Slot::LastGood, secrets)?;
    let mut match_count = read_slot_as_counter(Slot::MatchCount, secrets)?;

    // Read the monotonically-increasing counter
    let counter = se_get_counter(0, secrets)?;

    let num_fails = if lastgood > counter {
        // monkey business, but impossible, right?!
        99
    } else {
        counter - lastgood
    };

    // NOTE: 5LSB of match_count should be stored as zero.
    match_count &= !31;
    let attempts_left = if counter < match_count {
        // typical case: some number of attempts left before death
        match_count - counter
    } else {
        // we're a brick now, but maybe say that nicer to customer
        0
    };

    Ok(LastSuccess {
        num_fails,
        attempts_left,
    })
}

/// Read (typically a) counter value held in a dataslot.
/// Important that this be authenticated.
///
/// - using first 32-bits only, others will be zero/ignored
/// - but need to read whole thing for the digest check
fn read_slot_as_counter(slot: Slot, secrets: &RomSecrets) -> Result<u32, Error> {
    se_pair_unlock(secrets)?;
    let padded = unsafe {
        with_retries(|| {
            let mut padded = [0; 32];
            cryptoauthlib::atcab_read_zone(2, slot as u16, 0, 0, padded.as_mut_ptr(), 32)
                .into_result()?;
            Ok(padded)
        })?
    };

    se_pair_unlock(secrets)?;
    let tempkey = se_gendig_slot(slot, padded)?;

    if !se_is_correct_tempkey(tempkey, secrets)? {
        return Err(Error(-1));
    }

    Ok(u32::from_le_bytes(padded[..4].try_into().unwrap()))
}

fn is_main_pin(digest: [u8; 32], secrets: &RomSecrets) -> Result<bool, Error> {
    se_pair_unlock(secrets)?;
    Ok(se_checkmac_hard(Slot::PinHash, digest, secrets).is_ok())
}

const MAX_TARGET_ATTEMPTS: u32 = 21;

/// User got the main PIN right: update the attempt counters,
/// to document this (lastgood) and also bump the match counter if needed
fn updates_for_good_login(digest: [u8; 32], secrets: &RomSecrets) -> Result<(), Error> {
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "inside updates_for_good_login"
    )
    .ok();

    let count = se_get_counter(0, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_get_counter"
    )
    .ok();

    // The weird math here is because the match count slot in the SE ignores the least
    // significant 5 bits, so the match count must be a multiple of 32. When a good
    // login occurs, we need to update both the match count and the monotonic counter.
    //
    // For example, if the monotonic counter was 19 and the match count was 32, and the
    // user just provided the correct PIN, you would normally just bump the match count
    // to 33, but since that is not a multiple of 32, we have to bump it to 64. That
    // would then give 64-19 = 45 login attempts remaining though, so further down,
    // in se_add_counter(), we bump the monotonic counter in a loop until there are
    // MAX_TARGET_ATTEMPTS left (match count - counter0 = MAX_TARGET_ATTEMPTS_LEFT).
    let mc = (count + MAX_TARGET_ATTEMPTS + 32) & !31;

    let bump = (mc - MAX_TARGET_ATTEMPTS) - count;

    // The SE won't let the counter go past the match count, so we have to update the
    // match count first.

    // Set the new "match count"
    let mut tmp = [0; 32];
    tmp[0..4].copy_from_slice(&mc.to_le_bytes());
    tmp[4..8].copy_from_slice(&mc.to_le_bytes());
    se_encrypted_write(Slot::MatchCount, Slot::PinHash, digest, &tmp, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_encrypted_write for MatchCount"
    )
    .ok();

    // Increment the counter a bunch to get to that-13
    let new_count = se_add_counter(0, bump, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_add_counter"
    )
    .ok();

    // Update the "last good" counter
    let mut tmp = [0; 32];
    tmp[0..4].copy_from_slice(&new_count.to_le_bytes());
    se_encrypted_write32(Slot::LastGood, 0, Slot::PinHash, digest, tmp, secrets)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_encrypted_write32 for LastGood"
    )
    .ok();

    // NOTE: Some or all of the above writes could be blocked (trashed) by an
    // active MitM attacker, but that would be pointless since these are authenticated
    // writes, which have a MAC. They can't change the written value, due to the MAC, so
    // all they can do is block the write, and not control it's value. Therefore, they will
    // just be reducing attempt. Also, rate limiting not affected by anything here.

    Ok(())
}

/// Add-to and return a one-way counter's value. Have to go up in
/// single-unit steps, but we can loop.
fn se_add_counter(counter_number: u16, incr: u32, secrets: &RomSecrets) -> Result<u32, Error> {
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "inside se_add_counter"
    )
    .ok();

    // TODO Can I store the counter as 4 bytes instead
    let mut result = 0;
    for _ in 0..incr {
        unsafe {
            with_retries(|| {
                cryptoauthlib::atcab_counter_increment(counter_number, (&mut result) as *mut u32)
                    .into_result()
            })?;
        }
    }

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called cryptoauthlib::atcab_counter_increment X times"
    )
    .ok();

    // IMPORTANT: Always verify the counter's value because otherwise
    // nothing prevents an active MitM changing the value that we think
    // we just read. They could also stop us from incrementing the counter.
    let digest = se_gendig_counter(counter_number, result)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_gendig_counter"
    )
    .ok();

    if !se_is_correct_tempkey(digest, secrets)? {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "!se_is_correct_tempkey 2"
        )
        .ok();
        return Err(Error(-1));
    }

    Ok(result)
}

/// Read a one-way counter.
fn se_get_counter(counter_number: u16, secrets: &RomSecrets) -> Result<u32, Error> {
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "inside se_get_counter"
    )
    .ok();

    let result = unsafe {
        with_retries(|| {
            let mut result = 0;
            cryptoauthlib::atcab_counter_read(counter_number, (&mut result) as *mut u32)
                .into_result()?;
            Ok(result)
        })?
    };

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called cryptoauthlib::atcab_counter_read"
    )
    .ok();

    // IMPORTANT: Always verify the counter's value because otherwise
    // nothing prevents an active MitM changing the value that we think
    // we just read.
    let digest = se_gendig_counter(counter_number, result)?;

    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "called se_gendig_counter"
    )
    .ok();

    if !se_is_correct_tempkey(digest, secrets)? {
        writeln!(
            crate::uart::Uart::<crate::uart::Uart1>::new(),
            "!se_is_correct_tempkey"
        )
        .ok();
        return Err(Error(-1));
    }

    Ok(result)
}

#[derive(Debug, Clone)]
struct CachedMainPin([u8; 32]);

fn pin_cache_save(digest: [u8; 32], secrets: &RomSecrets) -> Result<CachedMainPin, Error> {
    // encrypt w/ rom secret + SRAM seed value
    let mut value = [0; 32];
    if !constant_time_eq::constant_time_eq_32(&digest, &[0; 32]) {
        value = pin_cache_get_key(secrets);
        value
            .iter_mut()
            .zip(digest.into_iter())
            .for_each(|(a, b)| *a ^= b);
    }

    // TODO I don't think this is needed anymore? This used to store the cached main PIN in a
    // global variable
    // Keep a copy around so we can do other auth'd actions later like set a user firmware pubkey
    // memcpy(g_cached_main_pin, value, 32);

    Ok(CachedMainPin(value))
}

/// Per-boot unique key.
fn pin_cache_get_key(secrets: &RomSecrets) -> [u8; 32] {
    Sha256::digest(&secrets.hash_cache_secret).into()
}

fn se_hmac32(slot: Slot, msg: [u8; 32]) -> Result<[u8; 32], Error> {
    let mut digest = [0; 32];
    unsafe {
        with_retries(|| {
            // Start SHA w/ HMAC setup
            cryptoauthlib::atcab_sha_base(
                4,
                slot as u16,
                core::ptr::null(),
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
            .into_result()
        })?;
        let mut digest_len = digest.len() as u16;
        with_retries(|| {
            // Send the contents to be hashed. Place the result in the output buffer.
            cryptoauthlib::atcab_sha_base(
                (3 << 6) | 2,
                msg.len() as u16,
                msg.as_ptr(),
                digest.as_mut_ptr(),
                (&mut digest_len) as *mut u16,
            )
            .into_result()
        })?;
    }
    Ok(digest)
}

/// Apply HMAC using secret in chip as a HMAC key, then encrypt
/// the result a little because read in clear over bus.
fn se_mixin_key(slot: Slot, msg: [u8; 32], secrets: &RomSecrets) -> Result<[u8; 32], Error> {
    se_pair_unlock(secrets)?;

    let result = if slot != Slot::None {
        se_hmac32(slot, msg)?
    } else {
        [0; 32]
    };

    // Final value was just read over bus w/o any protection, but
    // we won't be using that, instead, mix in the pairing secret.
    //
    // Concern: what if mitm gave us some zeros or other known pattern here. We will
    // use the value provided in cleartext[sic--it's not] write back shortly (to test it).
    // Solution: one more SHA256, and to be safe, mixin lots of values!

    let mut hasher = Sha256::new();
    hasher.update(secrets.pairing_secret);
    hasher.update(msg);
    hasher.update(&[slot as u8]);
    hasher.update(result);

    let mut result = [0; 32];
    result.copy_from_slice(hasher.finalize().as_slice());
    Ok(result)
}

fn se_pair_unlock(secrets: &RomSecrets) -> Result<(), Error> {
    writeln!(
        crate::uart::Uart::<crate::uart::Uart1>::new(),
        "inside se_pair_unlock"
    )
    .ok();
    const MAX_ATTEMPTS: usize = 3;
    for _ in 0..MAX_ATTEMPTS - 1 {
        if se_checkmac(Slot::PairingSecret, secrets.pairing_secret).is_ok() {
            return Ok(());
        }
    }
    se_checkmac(Slot::PairingSecret, secrets.pairing_secret)
}

fn pin_hash_attempt(
    slot: Slot,
    pin: Option<&Pin>,
    secrets: &RomSecrets,
) -> Result<[u8; 32], Error> {
    let Some(pin) = pin else {
        // "Blank" pin: all zeros, no hashing.
        return Ok([0; 32]);
    };

    // quick local hashing
    let tmp = pin_hash(Some(pin), PinPurpose::Normal, secrets);

    // main pin needs mega hashing
    let result = se_stretch_iter(tmp, KDF_ITER_PIN, secrets)?;

    // CAUTION: at this point, we just read the value off the bus
    // in clear text. Don't use that value directly.

    if slot == Slot::None {
        // let the caller do either/both of the below mixins
        return Ok(result);
    }

    if slot == Slot::PinHash {
        se_mixin_key(Slot::PinAttempt, result, secrets)
    } else {
        se_mixin_key(Slot::None, result, secrets)
    }
}

fn pin_hash(pin: Option<&Pin>, purpose: PinPurpose, secrets: &RomSecrets) -> [u8; 32] {
    // Used for supply chain validation too...not sure if that is less than MAX_PIN_LEN yet.
    // debug_assert!(pin.len() <= MAX_PIN_LEN);

    // TODO Same mistake here
    let Some(pin) = pin else {
        // "Blank" pin: all zero.
        return [0; 32];
    };

    let mut hasher = Sha256::new();
    hasher.update(secrets.pairing_secret);
    hasher.update((purpose as u32).to_le_bytes());
    hasher.update(&pin.0);
    hasher.update(secrets.otp_key);
    let result = hasher.finalize();

    // and a second-sha256 on that, just in case.
    let mut hasher = Sha256::new();
    hasher.update(result);
    let mut result = [0; 32];
    result.copy_from_slice(hasher.finalize().as_slice());
    result
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

fn with_retries<T>(mut f: impl FnMut() -> Result<T, Error>) -> Result<T, Error> {
    const MAX_RETRIES: u32 = 5;
    for i in 1..MAX_RETRIES {
        match f() {
            Ok(result) => return Ok(result),
            Err(Error::EXECUTION_ERROR) | Err(Error::COMM_FAIL) => unsafe {
                writeln!(
                    crate::uart::Uart::<crate::uart::Uart1>::new(),
                    "XXX RETRYING"
                )
                .ok();
                // TODO Log every retry to measure how much time is being wasted, or have a counter
                cryptoauthlib::hal_delay_ms(i * 200);
            },
            Err(e) => return Err(e),
        }
    }
    f()
}

#[derive(Debug, PartialEq, Eq)]
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
    /// Existing pin is wrong (during change attempt).
    const OLD_AUTH_FAIL: Error = Error(-113);

    const EXECUTION_ERROR: Error = Error(cryptoauthlib::ATCA_EXECUTION_ERROR);
    const COMM_FAIL: Error = Error(cryptoauthlib::ATCA_COMM_FAIL);

    pub fn is_checkmac_fail(&self) -> bool {
        self.0 == cryptoauthlib::ATCA_CHECKMAC_VERIFY_FAILED
    }
}
