//! Host tests for `esc_passthrough::client` and `esc_passthrough::line`, against the simulated
//! bootloader in `tests/common/sim.rs` (written from the bootloader's `main.c` and checked on its
//! own in `tests/simulator.rs`).
//!
//! Sources of the expected wire bytes (BL = `_plan/ref/AM32-bootloader/bootloader/main.c`, AP =
//! `ardupilot/libraries/AP_BLHeli/AP_BLHeli.cpp`; CRCs from a separate script, the same ones as
//! in `tests/bootloader.rs`):
//! - connect: the 21 init bytes, AP:796 and BL:604-627;
//! - read: set address `FF 00 AH AL` + CRC then `03 n` + CRC, AP:756 and AP:766, BL:669-713 and
//!   771-813;
//! - write: set address, set buffer `FE 00 0|1 len` + CRC (no answer, BL:715-737, AP:890-899),
//!   the payload + CRC (BL:583-600, AP:907-912), program `01 01` + CRC (BL:636-667,
//!   AP:921-928);
//! - erase: set address, `02 01` + CRC (BL:753-769, AP:870-876); keep-alive `FD 00` + CRC (BL:739-751,
//!   AP:857-867); run `00 00 00 00` (BL:629-634, AP:881-886).
//!
//! "Works again" below means: after the error, a keep-alive, then a write and a read-back at an
//! application address, succeed on the same line.

mod common;

use common::block_on::block_on;
use common::ref_crc_arc;
use common::sim::{EscModel, Fault, LineErrorAt, Sim, SimError, Wire};
use esc_passthrough::bootloader::{self, Ack, DeviceInfo};
use esc_passthrough::client::{Client, Error, Step, Timeouts};
use esc_passthrough::line::EscLines;
use proptest::prelude::*;
use std::collections::VecDeque;

type Err = Error<SimError>;

const SET_ADDRESS_2000: [u8; 6] = [0xFF, 0x00, 0x20, 0x00, 0x29, 0xD4];
const SET_ADDRESS_0400: [u8; 6] = [0xFF, 0x00, 0x04, 0x00, 0x32, 0xD4];
const SET_BUFFER_4: [u8; 6] = [0xFE, 0x00, 0x00, 0x04, 0x30, 0x2B];
const PAYLOAD_4: [u8; 6] = [0xDE, 0xAD, 0xBE, 0xEF, 0x9B, 0xE5];
const PROGRAM: [u8; 4] = [0x01, 0x01, 0xC0, 0x50];
const ERASE: [u8; 4] = [0x02, 0x01, 0xC0, 0xA0];
const KEEP_ALIVE: [u8; 4] = [0xFD, 0x00, 0x40, 0x90];
const READ_4: [u8; 4] = [0x03, 0x04, 0x01, 0x33];
const RUN: [u8; 4] = [0, 0, 0, 0];
const INIT: [u8; 21] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, b'B', b'L', b'H', b'e', b'l', b'i', 0xF4, 0x7D,
];
const APP: u16 = 0x2000;
const SETTINGS: u16 = 0x7C00;

fn client_on(sim: Sim, line: u8) -> Client<Sim> {
    let mut client = Client::new(sim);
    client.select(line).unwrap();
    client.line_mut().clear_log();
    client
}

/// One ESC on line 0, answers readable all at once.
fn client() -> Client<Sim> {
    client_on(Sim::single().burst(), 0)
}

fn pattern(len: usize, seed: u8) -> Vec<u8> {
    (0..len)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed))
        .collect()
}

fn esc(client: &Client<Sim>) -> &EscModel {
    client.line().esc(0)
}

fn writes(client: &Client<Sim>) -> Vec<Vec<u8>> {
    client.line().written(0).to_vec()
}

fn shapes(client: &Client<Sim>) -> Vec<Wire> {
    client.line().log().iter().map(Wire::shape).collect()
}

fn read_back(client: &mut Client<Sim>, address: u16, len: usize) -> Result<Vec<u8>, Err> {
    let mut buf = vec![0u8; len];
    block_on(client.read(address, &mut buf))?;
    Ok(buf)
}

/// After an error the line must still be usable.
fn works_again(client: &mut Client<Sim>) {
    block_on(client.keep_alive()).unwrap();
    let data = pattern(9, 0x40);
    block_on(client.write(0x3000, &data)).unwrap();
    assert_eq!(read_back(client, 0x3000, 9).unwrap(), data);
}

// ------------------------------------------------------------------ Timeouts

fn changed(f: impl FnOnce(&mut Timeouts)) -> Timeouts {
    let mut t = Timeouts::new();
    f(&mut t);
    t
}

#[test]
fn timeouts_new_has_the_ardupilot_defaults() {
    // AP:705 (1000 per byte), AP_BLHeli.h:278 (2 ms), AP:899 (5 ms), AP:912 (40 ms), AP:941 (500
    // ms), AP:876 (3 s).
    let t = Timeouts::new();
    assert_eq!(t.byte_us, 1_000);
    assert_eq!(t.ack_us, 2_000);
    assert_eq!(t.set_buffer_us, 5_000);
    assert_eq!(t.payload_us, 40_000);
    assert_eq!(t.program_us, 500_000);
    assert_eq!(t.erase_us, 3_000_000);
    assert_eq!(t.run_us, 2_000);
}

#[test]
fn timeouts_default_is_new() {
    assert_eq!(Timeouts::default(), Timeouts::new());
}

#[test]
fn with_byte_us_sets_only_byte_us() {
    let t = Timeouts::new().with_byte_us(7);
    assert_eq!(t, changed(|t| t.byte_us = 7));
}

#[test]
fn with_ack_us_sets_only_ack_us() {
    let t = Timeouts::new().with_ack_us(7);
    assert_eq!(t, changed(|t| t.ack_us = 7));
}

#[test]
fn with_set_buffer_us_sets_only_set_buffer_us() {
    let t = Timeouts::new().with_set_buffer_us(7);
    assert_eq!(t, changed(|t| t.set_buffer_us = 7));
}

#[test]
fn with_payload_us_sets_only_payload_us() {
    let t = Timeouts::new().with_payload_us(7);
    assert_eq!(t, changed(|t| t.payload_us = 7));
}

#[test]
fn with_program_us_sets_only_program_us() {
    let t = Timeouts::new().with_program_us(7);
    assert_eq!(t, changed(|t| t.program_us = 7));
}

#[test]
fn with_erase_us_sets_only_erase_us() {
    let t = Timeouts::new().with_erase_us(7);
    assert_eq!(t, changed(|t| t.erase_us = 7));
}

#[test]
fn with_run_us_sets_only_run_us() {
    let t = Timeouts::new().with_run_us(7);
    assert_eq!(t, changed(|t| t.run_us = 7));
}

#[test]
fn timeouts_new_is_const() {
    const T: Timeouts = Timeouts::new().with_ack_us(9).with_erase_us(10);
    assert_eq!((T.ack_us, T.erase_us), (9, 10));
}

// ------------------------------------------------------------------ Client accessors

#[test]
fn client_new_uses_the_default_timeouts() {
    let client = Client::new(Sim::single());
    assert_eq!(*client.timeouts(), Timeouts::new());
}

#[test]
fn timeouts_returns_what_with_timeouts_set() {
    let t = Timeouts::new().with_ack_us(123);
    let client = Client::new(Sim::single()).with_timeouts(t);
    assert_eq!(*client.timeouts(), t);
}

#[test]
fn with_timeouts_changes_the_waits_the_operations_pass_to_the_line() {
    let t = Timeouts::new()
        .with_ack_us(123)
        .with_set_buffer_us(456)
        .with_payload_us(789)
        .with_program_us(1011)
        .with_erase_us(1213)
        .with_byte_us(10)
        .with_run_us(14);
    let mut client = client().with_timeouts(t);
    block_on(client.keep_alive()).unwrap();
    block_on(client.write(APP, &[1, 2, 3, 4])).unwrap();
    block_on(client.erase(APP)).unwrap();
    block_on(client.connect()).unwrap();
    block_on(client.read(APP, &mut [0; 5])).unwrap();
    block_on(client.run_application()).unwrap();
    let timeouts: Vec<u32> = client
        .line()
        .log()
        .iter()
        .filter_map(|w| match w {
            Wire::Read { timeout_us, .. } if *timeout_us != 0 => Some(*timeout_us),
            _ => None,
        })
        .collect();
    assert_eq!(
        timeouts,
        // keep-alive; write: set address, set buffer, payload, program; erase: set address,
        // erase; connect: 9 bytes x byte_us; read: set address, 8 bytes x byte_us; run.
        [123, 123, 456, 789, 1011, 123, 1213, 90, 123, 80, 14]
    );
}

#[test]
fn line_returns_the_lines_for_inspection() {
    let client = Client::new(Sim::four());
    assert!(client.line().log().is_empty());
}

#[test]
fn line_mut_reaches_the_lines_to_select_one_and_to_break() {
    let mut client = Client::new(Sim::four().burst());
    client.line_mut().select(3).unwrap();
    client.line_mut().set_break(true).unwrap();
    assert!(client.line().break_on());
    client.line_mut().set_break(false).unwrap();
    assert_eq!(client.line_mut().select(4), Err(SimError::NoSuchLine(4)));
}

#[test]
fn into_line_gives_the_lines_back_with_their_history() {
    let mut client = client();
    block_on(client.keep_alive()).unwrap();
    let sim = client.into_line();
    assert_eq!(sim.written(0), [KEEP_ALIVE.to_vec()]);
}

// ------------------------------------------------------------------ connect

#[test]
fn connect_sends_the_init_and_returns_the_device_info() {
    let mut client = client();
    let info = block_on(client.connect()).unwrap();
    assert_eq!(writes(&client), [INIT.to_vec()]);
    assert_eq!(info.pin_code, 0x14);
    assert_eq!(info.flash_size_code, 0x1F);
    assert_eq!(info.tail, [0x06, 0x06]);
    assert_eq!(info.protocol_version, 3);
    assert_eq!(info.signature(), 0x1F06);
    assert_eq!(
        shapes(&client),
        [
            Wire::Read {
                cap: 32,
                timeout_us: 0
            },
            Wire::Write(vec![0; 21]),
            Wire::Read {
                cap: 9,
                timeout_us: 9_000
            },
        ]
    );
    works_again(&mut client);
}

#[test]
fn connect_collects_a_reply_that_arrives_a_byte_at_a_time() {
    // Default sim: one byte time (520 us) between bytes, so every read returns one byte.
    let mut client = client_on(Sim::single(), 0);
    let info = block_on(client.connect()).unwrap();
    assert_eq!(info.pin_code, 0x14);
    let reads: Vec<(usize, u32)> = client
        .line()
        .log()
        .iter()
        .filter_map(|w| match w {
            Wire::Read { cap, timeout_us } if *timeout_us != 0 => Some((*cap, *timeout_us)),
            _ => None,
        })
        .collect();
    // The first byte may take 9 byte times (AP:705), each next one 1 ms.
    let mut expected = vec![(9, 9_000)];
    expected.extend((1..9).rev().map(|cap| (cap, 1_000)));
    assert_eq!(reads, expected);
}

#[test]
fn connect_works_with_the_reply_split_by_the_line() {
    let mut client = client_on(Sim::single().burst().with_max_per_read(2), 0);
    assert_eq!(block_on(client.connect()).unwrap().pin_code, 0x14);
    works_again(&mut client);
}

// ------------------------------------------------------------------ read

#[test]
fn read_sets_the_address_then_reads_and_checks_the_reply() {
    let mut client = client();
    client.line_mut().esc_mut(0).flash_mut()[0x2000..0x2004]
        .copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
    let data = read_back(&mut client, APP, 4).unwrap();
    assert_eq!(data, [0xDE, 0xAD, 0xBE, 0xEF]);
    assert_eq!(
        writes(&client),
        [SET_ADDRESS_2000.to_vec(), READ_4.to_vec()]
    );
    assert_eq!(
        shapes(&client),
        [
            Wire::Read {
                cap: 32,
                timeout_us: 0
            },
            Wire::Write(vec![0; 6]),
            Wire::Read {
                cap: 1,
                timeout_us: 2_000
            },
            Wire::Write(vec![0; 4]),
            Wire::Read {
                cap: 7,
                timeout_us: 7_000
            },
        ]
    );
}

#[test]
fn read_of_256_bytes_is_sent_as_zero_and_returns_all_of_them() {
    let mut client = client();
    let image = pattern(256, 3);
    client.line_mut().esc_mut(0).flash_mut()[0x2000..0x2100].copy_from_slice(&image);
    assert_eq!(read_back(&mut client, APP, 256).unwrap(), image);
    assert_eq!(writes(&client)[1], [0x03, 0x00, 0x00, 0xF0]);
}

#[test]
fn read_reaches_the_magic_addresses_without_knowing_them() {
    let mut client = client();
    let name = read_back(&mut client, 0x0021, 10).unwrap();
    assert_eq!(name, b"SIM_TARGET");
    let block = read_back(&mut client, 0x0020, 4).unwrap();
    assert_eq!(block[0], 1);
    // 0x22 continues after the last read.
    let next = read_back(&mut client, 0x0022, 2).unwrap();
    assert_eq!(next, esc(&client).flash()[0x7C04..0x7C06]);
}

#[test]
fn read_leaves_the_buffer_alone_when_the_reply_is_bad() {
    let mut client = client();
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::CorruptCrc]);
    let mut buf = [0x5Au8; 8];
    assert!(block_on(client.read(APP, &mut buf)).is_err());
    assert_eq!(buf, [0x5A; 8]);
}

#[test]
fn read_refuses_a_buffer_of_0_or_more_than_256_bytes_before_sending() {
    let mut client = client();
    assert_eq!(
        block_on(client.read(APP, &mut [])),
        Err(Error::InvalidLength)
    );
    assert_eq!(
        block_on(client.read(APP, &mut [0; 257])),
        Err(Error::InvalidLength)
    );
    assert!(client.line().log().is_empty());
    assert!(writes(&client).is_empty());
}

// ------------------------------------------------------------------ write

#[test]
fn write_sends_set_address_set_buffer_payload_and_program() {
    let mut client = client();
    block_on(client.write(APP, &[0xDE, 0xAD, 0xBE, 0xEF])).unwrap();
    assert_eq!(
        writes(&client),
        [
            SET_ADDRESS_2000.to_vec(),
            SET_BUFFER_4.to_vec(),
            PAYLOAD_4.to_vec(),
            PROGRAM.to_vec()
        ]
    );
    assert_eq!(
        &esc(&client).flash()[0x2000..0x2004],
        [0xDE, 0xAD, 0xBE, 0xEF]
    );
    // After set buffer the client waits 5 ms for an answer that does not come (AP:899) and goes
    // on; payload 40 ms (AP:912); program 500 ms (AP:941).
    assert_eq!(
        shapes(&client),
        [
            Wire::Read {
                cap: 32,
                timeout_us: 0
            },
            Wire::Write(vec![0; 6]),
            Wire::Read {
                cap: 1,
                timeout_us: 2_000
            },
            Wire::Write(vec![0; 6]),
            Wire::Read {
                cap: 1,
                timeout_us: 5_000
            },
            Wire::Write(vec![0; 6]),
            Wire::Read {
                cap: 1,
                timeout_us: 40_000
            },
            Wire::Write(vec![0; 4]),
            Wire::Read {
                cap: 1,
                timeout_us: 500_000
            },
        ]
    );
}

#[test]
fn write_of_256_bytes_sets_the_buffer_with_a_high_byte_of_one() {
    let mut client = client();
    block_on(client.write(APP, &pattern(256, 9))).unwrap();
    let sent = writes(&client);
    assert_eq!(sent[1], [0xFE, 0x00, 0x01, 0x00, 0x30, 0x78]);
    assert_eq!(sent[2].len(), 258);
}

#[test]
fn write_then_read_returns_what_was_written_at_the_settings_and_an_application_address() {
    // The settings address both as the magic 0x20 and as the wire address 0x7C00. The bootloader
    // writes its own version into byte 2 of a block longer than two bytes (BL:650-658, version
    // 19 in Inc/version.h): the data that comes back is what the ESC holds, not the exact bytes
    // sent. The client does not look at it.
    for len in [1usize, 2, 184, 255, 256] {
        for address in [0x0020u16, SETTINGS, APP] {
            let mut client = client();
            let data = pattern(len, len as u8);
            block_on(client.write(address, &data)).unwrap();
            let mut expected = data.clone();
            if address != APP && len > 2 {
                expected[2] = common::sim::BOOTLOADER_VERSION;
            }
            assert_eq!(
                read_back(&mut client, address, len).unwrap(),
                expected,
                "len {len} at {address:#06X}"
            );
            let offset = if address == APP { 0x2000 } else { 0x7C00 };
            assert_eq!(&esc(&client).flash()[offset..offset + len], &expected[..]);
            works_again(&mut client);
        }
    }
}

#[test]
fn write_below_the_application_is_refused_and_changes_nothing() {
    let mut client = client();
    let before = esc(&client).flash().to_vec();
    assert_eq!(
        block_on(client.write(0x0400, &[1, 2, 3, 4])),
        Err(Error::Refused {
            step: Step::Program,
            ack: Ack::BadCommand
        })
    );
    assert_eq!(writes(&client)[0], SET_ADDRESS_0400);
    assert_eq!(esc(&client).flash(), &before[..]);
    works_again(&mut client);
}

#[test]
fn write_to_a_part_that_wants_aligned_writes_is_refused_at_program() {
    // Mcu/f421/Src/eeprom.c:11: address and length multiples of 4 (the settings block's 184
    // bytes pass, one byte does not).
    let sim = Sim::new(vec![EscModel::f421(0x14).with_write_granule(4)]).burst();
    let mut client = client_on(sim, 0);
    assert_eq!(
        block_on(client.write(APP, &[1])),
        Err(Error::Refused {
            step: Step::Program,
            ack: Ack::BadCommand
        })
    );
    block_on(client.write(APP, &pattern(184, 1))).unwrap();
    works_again_aligned(&mut client);
}

fn works_again_aligned(client: &mut Client<Sim>) {
    block_on(client.keep_alive()).unwrap();
    block_on(client.write(0x3000, &[1, 2, 3, 4])).unwrap();
}

#[test]
fn write_refuses_data_of_0_or_more_than_256_bytes_before_sending() {
    let mut client = client();
    assert_eq!(block_on(client.write(APP, &[])), Err(Error::InvalidLength));
    assert_eq!(
        block_on(client.write(APP, &[0; 257])),
        Err(Error::InvalidLength)
    );
    assert!(client.line().log().is_empty());
}

// ------------------------------------------------------------------ erase, keep-alive, run

#[test]
fn erase_sends_set_address_then_02_01() {
    let mut client = client();
    block_on(client.erase(APP)).unwrap();
    assert_eq!(writes(&client), [SET_ADDRESS_2000.to_vec(), ERASE.to_vec()]);
    // Three seconds for the acknowledge (AP:876).
    assert_eq!(
        shapes(&client).last(),
        Some(&Wire::Read {
            cap: 1,
            timeout_us: 3_000_000
        })
    );
    works_again(&mut client);
}

#[test]
fn erase_below_the_application_is_refused() {
    let mut client = client();
    assert_eq!(
        block_on(client.erase(0x0400)),
        Err(Error::Refused {
            step: Step::Erase,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
}

#[test]
fn keep_alive_treats_the_c1_as_success() {
    let mut client = client();
    block_on(client.keep_alive()).unwrap();
    assert_eq!(writes(&client), [KEEP_ALIVE.to_vec()]);
    assert_eq!(
        shapes(&client),
        [
            Wire::Read {
                cap: 32,
                timeout_us: 0
            },
            Wire::Write(vec![0; 4]),
            Wire::Read {
                cap: 1,
                timeout_us: 2_000
            },
        ]
    );
}

#[test]
fn run_application_sends_four_zero_bytes_and_the_esc_starts_its_application() {
    let mut client = client();
    block_on(client.run_application()).unwrap();
    assert_eq!(writes(&client), [RUN.to_vec()]);
    assert!(esc(&client).running());
    // The ESC is gone: nothing answers any more, and run still returns Ok.
    block_on(client.run_application()).unwrap();
    assert_eq!(
        block_on(client.keep_alive()),
        Err(Error::Timeout {
            step: Step::KeepAlive
        })
    );
}

#[test]
fn run_application_when_the_esc_refuses_to_jump_leaves_it_in_the_bootloader() {
    // BL:426-436: the first byte of the settings block must be 1. The client cannot tell.
    let mut client = client();
    client.line_mut().esc_mut(0).flash_mut()[0x7C00] = 0xFF;
    block_on(client.run_application()).unwrap();
    assert!(!esc(&client).running());
    works_again(&mut client);
}

#[test]
fn run_application_accepts_silence_and_refuses_other_answers() {
    let mut client = client();
    client.line_mut().fault(0, Fault::NoAnswer);
    block_on(client.run_application()).unwrap();
    let mut client = self::client();
    client.line_mut().fault(0, Fault::Replace(vec![0x55]));
    assert_eq!(
        block_on(client.run_application()),
        Err(Error::Unexpected {
            step: Step::Run,
            byte: 0x55
        })
    );
    let mut client = self::client();
    client.line_mut().fault(0, Fault::Replace(vec![0xC1, 0xC1]));
    assert_eq!(
        block_on(client.run_application()),
        Err(Error::Unexpected {
            step: Step::Run,
            byte: 0xC1
        })
    );
}

// ------------------------------------------------------------------ four lines

#[test]
fn connect_on_each_of_four_lines_talks_only_to_the_selected_line() {
    let mut client = Client::new(Sim::four().burst());
    for line in 0..4u8 {
        client.select(line).unwrap();
        let info = block_on(client.connect()).unwrap();
        assert_eq!(info.pin_code, 0x10 + line);
        for other in 0..4usize {
            let expected = usize::from(other <= usize::from(line));
            assert_eq!(
                client.line().written(other).len(),
                expected,
                "line {other} after connecting line {line}"
            );
        }
    }
    for line in 0..4 {
        assert_eq!(client.line().written(line), [INIT.to_vec()]);
        assert_eq!(client.line().esc(line).answers().len(), 1);
    }
}

#[test]
fn read_write_and_erase_on_one_line_change_only_that_lines_flash() {
    let mut client = Client::new(Sim::four().burst());
    let before: Vec<Vec<u8>> = (0..4)
        .map(|i| client.line().esc(i).flash().to_vec())
        .collect();
    client.select(2).unwrap();
    block_on(client.write(APP, &pattern(100, 5))).unwrap();
    block_on(client.erase(APP)).unwrap();
    let mut buf = [0u8; 100];
    block_on(client.read(APP, &mut buf)).unwrap();
    assert_eq!(buf[..], pattern(100, 5)[..]);
    for (i, original) in before.iter().enumerate() {
        assert_eq!(
            client.line().esc(i).flash() == &original[..],
            i != 2,
            "line {i}"
        );
        assert_eq!(client.line().written(i).is_empty(), i != 2, "line {i}");
    }
}

// ------------------------------------------------------------------ every error variant

#[test]
fn invalid_length_is_returned_with_nothing_sent() {
    let mut client = client();
    assert_eq!(
        block_on(client.write(APP, &[0; 300])),
        Err(Err::InvalidLength)
    );
    assert!(client.line().log().is_empty());
    works_again(&mut client);
}

#[test]
fn line_error_names_the_step_the_line_failed_in() {
    // The first read of every operation is the stale-byte drain.
    let mut client = client();
    client.line_mut().inject_line_error(LineErrorAt::NextRead);
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Line {
            step: Step::Drain,
            error: SimError::Injected
        })
    );
    works_again(&mut client);

    client.line_mut().inject_line_error(LineErrorAt::NextWrite);
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Line {
            step: Step::Init,
            error: SimError::Injected
        })
    );
    works_again(&mut client);

    // Drain passes, the wait for the init reply fails; the ESC did answer.
    client
        .line_mut()
        .inject_line_error(LineErrorAt::ReadAfter(1));
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Line {
            step: Step::Init,
            error: SimError::Injected
        })
    );
    works_again(&mut client);

    client.line_mut().inject_line_error(LineErrorAt::NextWrite);
    assert_eq!(
        block_on(client.write(APP, &[1])),
        Err(Error::Line {
            step: Step::SetAddress,
            error: SimError::Injected
        })
    );
    works_again(&mut client);

    // Drain, set address ack, then the wait after set buffer.
    client
        .line_mut()
        .inject_line_error(LineErrorAt::ReadAfter(2));
    assert_eq!(
        block_on(client.write(APP, &[1])),
        Err(Error::Line {
            step: Step::SetBuffer,
            error: SimError::Injected
        })
    );
    // set buffer was sent and the ESC now waits for the payload: the next frame is taken as one.
    assert!(esc(&client).expecting_payload());
}

#[test]
fn line_error_in_run_and_in_read_are_reported_with_their_steps() {
    let mut client = client();
    client
        .line_mut()
        .inject_line_error(LineErrorAt::ReadAfter(1));
    assert_eq!(
        block_on(client.run_application()),
        Err(Error::Line {
            step: Step::Run,
            error: SimError::Injected
        })
    );
    let mut client = self::client();
    client
        .line_mut()
        .inject_line_error(LineErrorAt::ReadAfter(2));
    assert_eq!(
        block_on(client.read(APP, &mut [0; 4])),
        Err(Error::Line {
            step: Step::Read,
            error: SimError::Injected
        })
    );
    works_again(&mut client);
}

#[test]
fn timeout_for_silence_at_every_step() {
    let cases: Vec<(&str, Vec<Fault>, Step)> = vec![
        ("connect", vec![Fault::NoAnswer], Step::Init),
        ("set address", vec![Fault::NoAnswer], Step::SetAddress),
        ("payload", vec![Fault::Pass, Fault::NoAnswer], Step::Payload),
        (
            "program",
            vec![Fault::Pass, Fault::Pass, Fault::NoAnswer],
            Step::Program,
        ),
        ("read", vec![Fault::Pass, Fault::NoAnswer], Step::Read),
        ("erase", vec![Fault::Pass, Fault::NoAnswer], Step::Erase),
        ("keep-alive", vec![Fault::NoAnswer], Step::KeepAlive),
    ];
    for (name, faults, step) in cases {
        let mut client = client();
        client.line_mut().faults(0, faults);
        let result = match name {
            "connect" => block_on(client.connect()).map(|_| ()),
            "set address" | "payload" | "program" => block_on(client.write(APP, &[1, 2, 3, 4])),
            "read" => block_on(client.read(APP, &mut [0; 4])),
            "erase" => block_on(client.erase(APP)),
            _ => block_on(client.keep_alive()),
        };
        assert_eq!(result, Err(Error::Timeout { step }), "{name}");
        works_again(&mut client);
    }
}

#[test]
fn a_late_answer_is_a_timeout_and_the_next_operation_throws_the_stale_byte_away() {
    let mut client = client();
    client.line_mut().fault(0, Fault::Late(10_000));
    assert_eq!(
        block_on(client.keep_alive()),
        Err(Error::Timeout {
            step: Step::KeepAlive
        })
    );
    // The caller lets the line settle (a session would wait or retry); the C1 is then waiting.
    client.line_mut().advance(20_000);
    works_again(&mut client);
}

#[test]
fn short_when_the_reply_stops_in_the_middle() {
    let mut client = client();
    client.line_mut().fault(0, Fault::Truncated(5));
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Short {
            step: Step::Init,
            got: 5,
            expected: 9
        })
    );
    works_again(&mut client);

    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::Truncated(6)]);
    assert_eq!(
        block_on(client.read(APP, &mut [0; 8])),
        Err(Error::Short {
            step: Step::Read,
            got: 6,
            expected: 11
        })
    );
    works_again(&mut client);

    // One byte that is not an acknowledge is short, not a refusal.
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::Truncated(1)]);
    assert_eq!(
        block_on(client.read(APP, &mut [0; 8])),
        Err(Error::Short {
            step: Step::Read,
            got: 1,
            expected: 11
        })
    );
    works_again(&mut client);
}

#[test]
fn bad_crc_when_the_esc_sends_data_whose_crc_does_not_fit() {
    let mut client = client();
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::CorruptCrc]);
    // Flash holds 0xFF there; the CRC of FF FF FF FF is 0x9401 (CRC-16/ARC, computed with the
    // test's bit-serial reference), its low byte was damaged.
    assert_eq!(ref_crc_arc(&[0xFF; 4]), 0x9401);
    assert_eq!(
        block_on(client.read(APP, &mut [0; 4])),
        Err(Error::BadCrc {
            step: Step::Read,
            expected: 0x9401,
            received: 0x9400
        })
    );
    works_again(&mut client);
}

#[test]
fn refused_when_the_esc_answers_c1_or_c2() {
    // Natural C1: the ESC refuses an address, a program and an erase.
    let mut client = client();
    assert_eq!(
        block_on(client.read(0x0010, &mut [0; 4])),
        Err(Error::Refused {
            step: Step::SetAddress,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
    assert_eq!(
        block_on(client.erase(0x0400)),
        Err(Error::Refused {
            step: Step::Erase,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
    // Injected acknowledges.
    client.line_mut().fault(0, Fault::WrongAck(0xC2));
    assert_eq!(
        block_on(client.read(APP, &mut [0; 4])),
        Err(Error::Refused {
            step: Step::SetAddress,
            ack: Ack::BadCrc
        })
    );
    works_again(&mut client);
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::WrongAck(0xC2)]);
    assert_eq!(
        block_on(client.write(APP, &[1, 2, 3, 4])),
        Err(Error::Refused {
            step: Step::Payload,
            ack: Ack::BadCrc
        })
    );
    works_again(&mut client);
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::Pass, Fault::WrongAck(0xC1)]);
    assert_eq!(
        block_on(client.write(APP, &[1, 2, 3, 4])),
        Err(Error::Refused {
            step: Step::Program,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
    // A refused read is one byte (BL:782-786, 815).
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::Replace(vec![0xC1])]);
    assert_eq!(
        block_on(client.read(APP, &mut [0; 4])),
        Err(Error::Refused {
            step: Step::Read,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
    // A garbled init is answered with one byte as well.
    client.line_mut().fault(0, Fault::Replace(vec![0xC2]));
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Refused {
            step: Step::Init,
            ack: Ack::BadCrc
        })
    );
    works_again(&mut client);
    // Nine bytes that end in a refusal.
    client.line_mut().fault(0, Fault::WrongAck(0xC1));
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Refused {
            step: Step::Init,
            ack: Ack::BadCommand
        })
    );
    works_again(&mut client);
    // keep-alive: C2 means the ESC saw a bad CRC.
    client.line_mut().fault(0, Fault::WrongAck(0xC2));
    assert_eq!(
        block_on(client.keep_alive()),
        Err(Error::Refused {
            step: Step::KeepAlive,
            ack: Ack::BadCrc
        })
    );
    works_again(&mut client);
}

#[test]
fn refused_at_set_buffer_when_the_esc_saw_a_damaged_set_buffer() {
    // The only answer the ESC can give to set buffer is C2 for a bad CRC (BL:719-722).
    let mut client = client();
    client.line_mut().corrupt_burst(0, 1, 5, 0x01);
    assert_eq!(
        block_on(client.write(APP, &[1, 2, 3, 4])),
        Err(Error::Refused {
            step: Step::SetBuffer,
            ack: Ack::BadCrc
        })
    );
    assert!(!esc(&client).expecting_payload());
    works_again(&mut client);
}

const KEEP_ALIVE_FRAME: [u8; 4] = KEEP_ALIVE;

/// The bursts `line` received, from index `from`.
fn bursts_from(client: &Client<Sim>, line: usize, from: usize) -> Vec<Vec<u8>> {
    client.line().written(line)[from..].to_vec()
}

#[test]
fn a_refused_payload_is_resynced_before_the_next_write_which_lands_where_it_was_asked() {
    // BL:583-600: after a bad payload CRC the ESC still takes the next frame for the payload,
    // and a valid frame passes the check (a frame followed by its CRC has CRC 0): without the
    // resync the next write's set address would be answered 30 and ignored, and the write would
    // land at the failed write's address (the old version of this test showed exactly that).
    let mut client = client();
    block_on(client.write(APP, &[1, 2, 3, 4])).unwrap();
    client.line_mut().corrupt_burst(0, 2, 0, 0xFF);
    assert_eq!(
        block_on(client.write(0x3000, &[9, 9, 9, 9])),
        Err(Error::Refused {
            step: Step::Payload,
            ack: Ack::BadCrc
        })
    );
    assert!(esc(&client).expecting_payload());
    assert!(client.needs_resync());
    let before = esc(&client).flash().to_vec();
    let mark = esc(&client).bursts().len();
    let answers = esc(&client).answers().len();
    client.line_mut().clear_log();

    block_on(client.write(0x4000, &[5, 6, 7, 8])).unwrap();

    assert!(!client.needs_resync());
    assert!(!esc(&client).expecting_payload());
    // The image: only 0x4000.. changed.
    let mut expected = before;
    expected[0x4000..0x4004].copy_from_slice(&[5, 6, 7, 8]);
    assert_eq!(esc(&client).flash(), &expected[..]);
    // The wire: one keep-alive, then the four frames of the write.
    let sent = bursts_from(&client, 0, mark);
    assert_eq!(sent.len(), 5);
    assert_eq!(sent[0], KEEP_ALIVE_FRAME);
    assert_eq!(sent[1], [0xFF, 0x00, 0x40, 0x00, 0x01, 0xD4]);
    // The keep-alive was taken as the awaited payload and answered 30.
    assert_eq!(esc(&client).answers()[answers], [0x30]);
    let shape = shapes(&client);
    assert_eq!(
        shape[..4],
        [
            Wire::Read {
                cap: 32,
                timeout_us: 0
            },
            Wire::Write(vec![0; 4]),
            Wire::Read {
                cap: 1,
                timeout_us: 2_000
            },
            Wire::Write(vec![0; 6]),
        ]
    );
    works_again(&mut client);
}

#[test]
fn a_marked_line_whose_esc_was_not_stuck_answers_the_resync_c1_and_the_write_is_correct() {
    // The payload was accepted but its acknowledge was lost: marked, not stuck.
    let mut client = client();
    client.line_mut().faults(0, [Fault::Pass, Fault::NoAnswer]);
    assert_eq!(
        block_on(client.write(APP, &[1, 2, 3, 4])),
        Err(Error::Timeout {
            step: Step::Payload
        })
    );
    assert!(client.needs_resync());
    assert!(!esc(&client).expecting_payload());
    let mark = esc(&client).bursts().len();
    let answers = esc(&client).answers().len();
    block_on(client.write(0x4000, &[5, 6, 7, 8])).unwrap();
    assert_eq!(bursts_from(&client, 0, mark)[0], KEEP_ALIVE_FRAME);
    assert_eq!(esc(&client).answers()[answers], [0xC1]);
    assert_eq!(&esc(&client).flash()[0x4000..0x4004], [5, 6, 7, 8]);
    assert!(!client.needs_resync());
}

#[test]
fn a_read_after_a_failed_write_returns_the_right_data() {
    let mut client = client();
    block_on(client.write(APP, &[1, 2, 3, 4])).unwrap();
    client.line_mut().corrupt_burst(0, 2, 0, 0xFF);
    assert!(block_on(client.write(0x3000, &[9, 9, 9, 9])).is_err());
    // Without the resync the read's set address would be swallowed and the read refused or
    // answered from the wrong address.
    assert_eq!(read_back(&mut client, APP, 4).unwrap(), [1, 2, 3, 4]);
    assert_eq!(esc(&client).flash()[0x3000..0x3004], [0xFF; 4]);
}

#[test]
fn every_operation_resyncs_a_marked_line_first() {
    for op in 0..6 {
        let mut client = client();
        client.line_mut().corrupt_burst(0, 1, 5, 0x01); // damaged set buffer: C2, ESC not stuck
        assert!(block_on(client.write(APP, &[1])).is_err());
        assert!(client.needs_resync());
        let mark = esc(&client).bursts().len();
        match op {
            0 => drop(block_on(client.connect()).unwrap()),
            1 => block_on(client.read(APP, &mut [0; 2])).unwrap(),
            2 => block_on(client.write(APP, &[7])).unwrap(),
            3 => block_on(client.erase(APP)).unwrap(),
            4 => block_on(client.keep_alive()).unwrap(),
            _ => block_on(client.run_application()).unwrap(),
        }
        assert_eq!(
            bursts_from(&client, 0, mark)[0],
            KEEP_ALIVE_FRAME,
            "op {op}"
        );
        assert!(!client.needs_resync());
    }
}

#[test]
fn the_mark_is_per_line() {
    let mut client = Client::new(Sim::four().burst());
    client.select(1).unwrap();
    client.line_mut().corrupt_burst(1, 2, 0, 0xFF);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    assert!(client.needs_resync());
    client.select(2).unwrap();
    assert!(!client.needs_resync());
    block_on(client.write(APP, &[5, 6, 7, 8])).unwrap();
    // Line 2 saw its write and nothing else.
    assert_eq!(client.line().written(2).len(), 4);
    assert_eq!(client.line().written(2)[0], SET_ADDRESS_2000);
    // Back on line 1 the keep-alive comes first.
    client.select(1).unwrap();
    assert!(client.needs_resync());
    let seen = client.line().written(1).len();
    block_on(client.keep_alive()).unwrap();
    assert_eq!(client.line().written(1).len(), seen + 2);
    assert_eq!(client.line().written(1)[seen], KEEP_ALIVE_FRAME);
    assert!(!client.needs_resync());
    assert!(!client.line().esc(1).expecting_payload());
}

#[test]
fn resync_on_an_unmarked_line_sends_one_keep_alive_and_works() {
    let mut client = client();
    assert!(!client.needs_resync());
    block_on(client.resync()).unwrap();
    assert_eq!(writes(&client), [KEEP_ALIVE_FRAME.to_vec()]);
    assert_eq!(esc(&client).answers(), [vec![0xC1]]);
    works_again(&mut client);
}

#[test]
fn resync_clears_the_mark_of_a_stuck_esc() {
    let mut client = client();
    client.line_mut().corrupt_burst(0, 2, 0, 0xFF);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    assert!(esc(&client).expecting_payload() && client.needs_resync());
    block_on(client.resync()).unwrap();
    assert!(!esc(&client).expecting_payload() && !client.needs_resync());
}

#[test]
fn a_resync_that_fails_keeps_the_mark_and_starts_nothing() {
    let mut client = client();
    client.line_mut().faults(0, [Fault::Pass, Fault::NoAnswer]);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    let cases: [(Fault, Err); 3] = [
        (Fault::NoAnswer, Error::Timeout { step: Step::Resync }),
        (
            Fault::WrongAck(0x55),
            Error::Unexpected {
                step: Step::Resync,
                byte: 0x55,
            },
        ),
        (
            Fault::WrongAck(0xC2),
            Error::Refused {
                step: Step::Resync,
                ack: Ack::BadCrc,
            },
        ),
    ];
    for (fault, error) in cases {
        let seen = esc(&client).bursts().len();
        client.line_mut().fault(0, fault);
        assert_eq!(block_on(client.write(0x4000, &[5, 6, 7, 8])), Err(error));
        assert!(client.needs_resync());
        // Only the keep-alive went out.
        assert_eq!(bursts_from(&client, 0, seen), [KEEP_ALIVE_FRAME.to_vec()]);
    }
    block_on(client.write(0x4000, &[5, 6, 7, 8])).unwrap();
    assert!(!client.needs_resync());
}

#[test]
fn a_resync_line_error_is_reported_with_its_step() {
    let mut client = client();
    client.line_mut().faults(0, [Fault::Pass, Fault::NoAnswer]);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    client.line_mut().inject_line_error(LineErrorAt::NextWrite);
    assert_eq!(
        block_on(client.resync()),
        Err(Error::Line {
            step: Step::Resync,
            error: SimError::Injected
        })
    );
    assert!(client.needs_resync());
}

#[test]
fn a_line_error_after_set_buffer_marks_the_line() {
    let mut client = client();
    client
        .line_mut()
        .inject_line_error(LineErrorAt::ReadAfter(2));
    assert!(block_on(client.write(APP, &[1, 2])).is_err());
    assert!(client.needs_resync());
    assert!(esc(&client).expecting_payload());
    works_again(&mut client);
    assert!(!esc(&client).expecting_payload());
}

#[test]
fn a_write_that_fails_after_the_payload_was_acknowledged_does_not_mark_the_line() {
    let mut client = client();
    assert!(block_on(client.write(0x0400, &[1, 2, 3, 4])).is_err());
    assert!(!client.needs_resync());
    let mut client = self::client();
    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::Pass, Fault::NoAnswer]);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    assert!(!client.needs_resync());
    // And a failure before set buffer.
    client.line_mut().fault(0, Fault::NoAnswer);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    assert!(!client.needs_resync());
}

#[test]
fn a_one_byte_payload_the_esc_still_waits_for_cannot_be_cleared_by_the_keep_alive() {
    // The ESC waiting for 1 + 2 bytes keeps only the first three of the keep-alive and answers
    // C2 (BL:1020-1027, 586): the limit named in the module note. The error is reported, the
    // mark stays.
    let mut client = client();
    client.line_mut().corrupt_burst(0, 2, 0, 0xFF);
    assert!(block_on(client.write(APP, &[7])).is_err());
    assert!(esc(&client).expecting_payload());
    assert_eq!(
        block_on(client.resync()),
        Err(Error::Refused {
            step: Step::Resync,
            ack: Ack::BadCrc
        })
    );
    assert!(client.needs_resync());
    assert!(esc(&client).expecting_payload());
}

#[test]
fn a_line_chosen_without_select_is_tracked_as_its_own_slot() {
    let mut client = Client::new(Sim::single().burst());
    client.line_mut().select(0).unwrap();
    client.line_mut().faults(0, [Fault::Pass, Fault::NoAnswer]);
    assert!(block_on(client.write(APP, &[1, 2, 3, 4])).is_err());
    assert!(client.needs_resync());
    block_on(client.keep_alive()).unwrap();
    assert!(!client.needs_resync());
}

#[test]
fn select_forwards_to_the_line_and_remembers_it() {
    let mut client = Client::new(Sim::four().burst());
    assert_eq!(client.select(3), Ok(()));
    assert_eq!(client.line().log(), [Wire::Select(3)]);
    block_on(client.keep_alive()).unwrap();
    assert_eq!(client.line().written(3).len(), 1);
    // A refusal by the line is reported with its step and changes nothing.
    assert_eq!(
        client.select(4),
        Err(Error::Line {
            step: Step::Select,
            error: SimError::NoSuchLine(4)
        })
    );
    block_on(client.keep_alive()).unwrap();
    assert_eq!(client.line().written(3).len(), 2);
}

#[test]
fn select_refuses_a_line_above_31_without_forwarding_it() {
    let mut client = Client::new(Sim::four().burst());
    assert_eq!(client.select(32), Err(Error::InvalidLine));
    assert_eq!(client.select(255), Err(Error::InvalidLine));
    assert!(client.line().log().is_empty());
}

#[test]
fn needs_resync_is_false_on_a_fresh_client() {
    assert!(!Client::new(Sim::single()).needs_resync());
}

#[test]
fn unexpected_when_a_byte_is_not_an_acknowledge_or_not_the_signature() {
    let mut client = client();
    client.line_mut().fault(0, Fault::WrongAck(0x55));
    assert_eq!(
        block_on(client.erase(APP)),
        Err(Error::Unexpected {
            step: Step::SetAddress,
            byte: 0x55
        })
    );
    works_again(&mut client);

    client.line_mut().fault(0, Fault::WrongAck(0x55));
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Unexpected {
            step: Step::Init,
            byte: 0x55
        })
    );
    works_again(&mut client);

    // "472...": the first wrong byte is reported.
    client.line_mut().fault(
        0,
        Fault::Replace(vec![b'4', b'7', b'2', 0x14, 0x1F, 6, 6, 3, 0x30]),
    );
    assert_eq!(
        block_on(client.connect()),
        Err(Error::Unexpected {
            step: Step::Init,
            byte: b'2'
        })
    );
    works_again(&mut client);

    client
        .line_mut()
        .faults(0, [Fault::Pass, Fault::WrongAck(0x55)]);
    assert_eq!(
        block_on(client.read(APP, &mut [0; 4])),
        Err(Error::Unexpected {
            step: Step::Read,
            byte: 0x55
        })
    );
    works_again(&mut client);

    // Keep-alive answered 30: not what it is for.
    client.line_mut().fault(0, Fault::WrongAck(0x30));
    assert_eq!(
        block_on(client.keep_alive()),
        Err(Error::Unexpected {
            step: Step::KeepAlive,
            byte: 0x30
        })
    );
    works_again(&mut client);
}

// ------------------------------------------------------------------ a scripted line

/// A line that answers from a script: each read with a timeout takes the next chunk.
struct Script {
    chunks: VecDeque<Vec<u8>>,
    written: Vec<Vec<u8>>,
}

impl Script {
    fn new(chunks: &[&[u8]]) -> Self {
        Self {
            chunks: chunks.iter().map(|c| c.to_vec()).collect(),
            written: Vec::new(),
        }
    }
}

impl EscLines for Script {
    type Error = ();
    fn select(&mut self, _line: u8) -> Result<(), ()> {
        Ok(())
    }
    async fn write(&mut self, bytes: &[u8]) -> Result<(), ()> {
        self.written.push(bytes.to_vec());
        Ok(())
    }
    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, ()> {
        if timeout_us == 0 {
            return Ok(0);
        }
        let Some(mut chunk) = self.chunks.pop_front() else {
            return Ok(0);
        };
        let n = chunk.len().min(buf.len());
        buf[..n].copy_from_slice(&chunk[..n]);
        if n < chunk.len() {
            self.chunks.push_front(chunk.split_off(n));
        }
        Ok(n)
    }
    fn set_break(&mut self, _on: bool) -> Result<(), ()> {
        Ok(())
    }
}

#[test]
fn set_buffer_answered_with_30_is_tolerated_and_anything_else_is_not() {
    // AP:899-905 goes on after brNONE or brSUCCESS and fails on any other byte.
    let mut ok = Client::new(Script::new(&[&[0x30], &[0x30], &[0x30], &[0x30]]));
    assert_eq!(block_on(ok.write(APP, &[1])), Ok(()));
    assert_eq!(ok.line().written.len(), 4);

    let mut refused = Client::new(Script::new(&[&[0x30], &[0xC1]]));
    assert_eq!(
        block_on(refused.write(APP, &[1])),
        Err(Error::Refused {
            step: Step::SetBuffer,
            ack: Ack::BadCommand
        })
    );
    let mut odd = Client::new(Script::new(&[&[0x30], &[0x77]]));
    assert_eq!(
        block_on(odd.write(APP, &[1])),
        Err(Error::Unexpected {
            step: Step::SetBuffer,
            byte: 0x77
        })
    );
    // The payload is not sent after a refusal.
    assert_eq!(odd.line().written.len(), 2);
}

#[test]
fn a_line_that_returns_more_than_the_buffer_holds_does_not_panic() {
    struct Liar;
    impl EscLines for Liar {
        type Error = ();
        fn select(&mut self, _line: u8) -> Result<(), ()> {
            Ok(())
        }
        async fn write(&mut self, _bytes: &[u8]) -> Result<(), ()> {
            Ok(())
        }
        async fn read(&mut self, buf: &mut [u8], _timeout_us: u32) -> Result<usize, ()> {
            buf.fill(0x30);
            Ok(buf.len() + 100)
        }
        fn set_break(&mut self, _on: bool) -> Result<(), ()> {
            Ok(())
        }
    }
    let mut client = Client::new(Liar);
    // The drain loop is bounded too (16 reads), and the replies are clamped.
    let _ = block_on(client.connect());
    let _ = block_on(client.read(APP, &mut [0; 4]));
    let _ = block_on(client.write(APP, &[1, 2]));
    let _ = block_on(client.keep_alive());
}

// ------------------------------------------------------------------ stale bytes

#[test]
fn every_operation_first_throws_away_bytes_an_earlier_exchange_left_behind() {
    let mut client = client();
    // An answer of two bytes: the first is read (and is wrong), the second stays on the line.
    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    assert_eq!(
        block_on(client.keep_alive()),
        Err(Error::Unexpected {
            step: Step::KeepAlive,
            byte: 0x55
        })
    );
    block_on(client.keep_alive()).unwrap();

    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    let _ = block_on(client.keep_alive());
    assert_eq!(block_on(client.connect()).unwrap().pin_code, 0x14);
    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    let _ = block_on(client.keep_alive());
    block_on(client.erase(APP)).unwrap();
    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    let _ = block_on(client.keep_alive());
    block_on(client.write(APP, &[1, 2])).unwrap();
    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    let _ = block_on(client.keep_alive());
    assert_eq!(read_back(&mut client, APP, 2).unwrap(), [1, 2]);
    client.line_mut().fault(0, Fault::Replace(vec![0x55, 0x66]));
    let _ = block_on(client.keep_alive());
    block_on(client.run_application()).unwrap();
}

// ------------------------------------------------------------------ proptests

fn app_range() -> impl Strategy<Value = u16> {
    // Up to 0x7A00 + 256 stays below the file name at 0x7BE0.
    0x1000u16..=0x7A00
}

fn data_pair() -> impl Strategy<Value = (Vec<u8>, Vec<u8>)> {
    (1usize..=256).prop_flat_map(|len| {
        (
            prop::collection::vec(any::<u8>(), len),
            prop::collection::vec(any::<u8>(), len),
        )
    })
}

fn round_trip(sim: Sim, address: u16, data: &[u8]) -> (Vec<Wire>, Vec<u8>) {
    let mut client = client_on(sim, 0);
    block_on(client.write(address, data)).unwrap();
    let mut buf = vec![0u8; data.len()];
    block_on(client.read(address, &mut buf)).unwrap();
    (shapes(&client), buf)
}

proptest! {
    #[test]
    fn the_sequence_of_line_calls_does_not_depend_on_the_data_bytes(
        address in app_range(),
        (one, two) in data_pair(),
        slow in any::<bool>(),
        per_read in prop::option::of(1usize..=8),
    ) {
        let make = || {
            let mut sim = Sim::single();
            if !slow {
                sim = sim.burst();
            }
            if let Some(n) = per_read {
                sim = sim.with_max_per_read(n);
            }
            sim
        };
        let (shape_one, back_one) = round_trip(make(), address, &one);
        let (shape_two, back_two) = round_trip(make(), address, &two);
        prop_assert_eq!(shape_one, shape_two);
        prop_assert_eq!(back_one, one);
        prop_assert_eq!(back_two, two);
    }

    #[test]
    fn random_writes_are_read_back_and_land_only_where_asked(
        address in app_range(),
        (data, _) in data_pair(),
    ) {
        let mut client = client();
        let before = esc(&client).flash().to_vec();
        block_on(client.write(address, &data)).unwrap();
        let after = esc(&client).flash();
        let start = address as usize;
        prop_assert_eq!(&after[start..start + data.len()], &data[..]);
        prop_assert_eq!(&after[..start], &before[..start]);
        prop_assert_eq!(&after[start + data.len()..], &before[start + data.len()..]);
    }

    #[test]
    fn a_damaged_read_reply_is_never_ok_with_wrong_data(
        address in app_range(),
        len in 1usize..=256,
        seed in any::<u8>(),
        at in 0usize..259,
        xor in 1u8..=255,
        extra in prop::collection::vec(any::<u8>(), 0..8),
        per_read in 1usize..=300,
    ) {
        let mut client = client_on(Sim::single().burst().with_max_per_read(per_read), 0);
        let held = pattern(len, seed);
        let start = address as usize;
        client.line_mut().esc_mut(0).flash_mut()[start..start + len].copy_from_slice(&held);
        // The true reply, one byte of it changed (CRC-16 catches every single-byte change).
        let mut reply = held.clone();
        reply.extend_from_slice(&ref_crc_arc(&held).to_le_bytes());
        reply.push(0x30);
        let index = at % reply.len();
        reply[index] ^= xor;
        client.line_mut().faults(0, [Fault::Pass, Fault::Replace(reply)]);
        let mut buf = vec![0u8; len];
        prop_assert!(block_on(client.read(address, &mut buf)).is_err());

        // The true reply followed by extra bytes: Ok, with the held data only.
        let mut reply = held.clone();
        reply.extend_from_slice(&ref_crc_arc(&held).to_le_bytes());
        reply.push(0x30);
        reply.extend_from_slice(&extra);
        client.line_mut().faults(0, [Fault::Pass, Fault::Replace(reply)]);
        block_on(client.read(address, &mut buf)).unwrap();
        prop_assert_eq!(&buf, &held);
    }

    #[test]
    fn arbitrary_bytes_from_the_line_never_panic_and_never_pass_for_other_data(
        address in app_range(),
        len in 1usize..=64,
        garbage in prop::collection::vec(any::<u8>(), 0..300),
        replies in prop::collection::vec(prop::collection::vec(any::<u8>(), 0..24), 0..4),
        per_read in 1usize..=300,
        which in 0usize..6,
    ) {
        let mut client = client_on(Sim::single().burst().with_max_per_read(per_read), 0);
        let held = pattern(len, 77);
        let start = address as usize;
        client.line_mut().esc_mut(0).flash_mut()[start..start + len].copy_from_slice(&held);
        // Reads: the ESC's set-address answer is honest, the reply is garbage. Ok means the
        // garbage was itself a well-formed reply, and then the data is the garbage's, which
        // the CRC cannot tell from the truth (probability 2^-24 for random bytes).
        client.line_mut().faults(0, [Fault::Pass, Fault::Replace(garbage.clone())]);
        let mut buf = vec![0u8; len];
        if block_on(client.read(address, &mut buf)).is_ok() {
            let mut well_formed = buf.clone();
            well_formed.extend_from_slice(&ref_crc_arc(&buf).to_le_bytes());
            well_formed.push(0x30);
            prop_assert!(buf == held || garbage.starts_with(&well_formed));
        }
        // Every operation with arbitrary answers: no panic, whatever comes back.
        let faults = replies.into_iter().map(Fault::Replace);
        client.line_mut().faults(0, faults);
        let _ = match which {
            0 => block_on(client.connect()).map(|_| ()),
            1 => block_on(client.write(address, &held)),
            2 => block_on(client.read(address, &mut buf)),
            3 => block_on(client.erase(address)),
            4 => block_on(client.keep_alive()),
            _ => block_on(client.run_application()),
        };
    }
}

#[test]
fn device_info_comes_back_as_the_parser_would_read_it() {
    // The client returns the crate's own DeviceInfo; compare with the parser on the ESC's bytes.
    let mut client = client();
    let info = block_on(client.connect()).unwrap();
    let from_parser: DeviceInfo = bootloader::parse_init_reply(&esc(&client).answers()[0]).unwrap();
    assert_eq!(info, from_parser);
}
