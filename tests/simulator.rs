//! Host tests of the simulated bootloader itself (`tests/common/sim.rs`): raw frames in, raw
//! bytes out, every sequence worked out by hand from the bootloader's `main.c` (BL =
//! `_plan/ref/AM32-bootloader/bootloader/main.c`) with the CRCs computed by a separate script.
//! The crate's encoders are not used here: the simulator is the independent party the client
//! tests run against, so it is checked on its own first.
//!
//! The ESC under test is the F421-like model: 32 KB, settings block at wire address 0x7C00
//! (also reached by the magic address 0x20), application from 0x1000, pin code 0x14, flash size
//! code 0x1F (BL:214-218), protocol version 3 (BL:242), bootloader version 19 (`Inc/version.h`).

mod common;

use common::block_on::block_on;
use common::sim::{EscModel, Fault, LineErrorAt, Sim, SimError, Wire};
use esc_passthrough::line::EscLines;

const INIT: [u8; 21] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, b'B', b'L', b'H', b'e', b'l', b'i', 0xF4, 0x7D,
];
const INFO: [u8; 9] = [0x34, 0x37, 0x31, 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30];
const SET_ADDRESS_7C00: [u8; 6] = [0xFF, 0x00, 0x7C, 0x00, 0x10, 0xD4];
const SET_ADDRESS_2000: [u8; 6] = [0xFF, 0x00, 0x20, 0x00, 0x29, 0xD4];
const SET_ADDRESS_0400: [u8; 6] = [0xFF, 0x00, 0x04, 0x00, 0x32, 0xD4];
const SET_ADDRESS_0010: [u8; 6] = [0xFF, 0x00, 0x00, 0x10, 0x31, 0xD8];
const SET_ADDRESS_20: [u8; 6] = [0xFF, 0x00, 0x00, 0x20, 0x31, 0xCC];
const SET_ADDRESS_23: [u8; 6] = [0xFF, 0x00, 0x00, 0x23, 0x71, 0xCD];
const SET_BUFFER_4: [u8; 6] = [0xFE, 0x00, 0x00, 0x04, 0x30, 0x2B];
const PAYLOAD_DEAD_BEEF: [u8; 6] = [0xDE, 0xAD, 0xBE, 0xEF, 0x9B, 0xE5];
const PROGRAM: [u8; 4] = [0x01, 0x01, 0xC0, 0x50];
const ERASE: [u8; 4] = [0x02, 0x01, 0xC0, 0xA0];
const KEEP_ALIVE: [u8; 4] = [0xFD, 0x00, 0x40, 0x90];
const READ_4: [u8; 4] = [0x03, 0x04, 0x01, 0x33];
const RUN: [u8; 4] = [0, 0, 0, 0];

fn sim() -> Sim {
    let mut sim = Sim::single().burst();
    sim.select(0).unwrap();
    sim
}

/// Sends one burst and returns everything the ESC answers.
fn exchange(sim: &mut Sim, bytes: &[u8]) -> Vec<u8> {
    block_on(async {
        sim.write(bytes).await.unwrap();
        let mut out = Vec::new();
        let mut buf = [0u8; 300];
        loop {
            let n = sim.read(&mut buf, 20_000).await.unwrap();
            if n == 0 {
                return out;
            }
            out.extend_from_slice(&buf[..n]);
        }
    })
}

#[test]
fn init_gets_the_nine_byte_device_info() {
    // BL:604-627 match at rxBuffer[20]/[12]/[13]; BL:242 and 254-292 the reply.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &INIT), INFO);
}

#[test]
fn init_is_matched_at_three_offsets_and_swallowed_when_it_does_not_match() {
    let mut sim = sim();
    // BL:604-611: byte 16 is 0x7D, byte 8 is 13, byte 9 is 66.
    let mut short = [0u8; 17];
    short[8] = 13;
    short[9] = 66;
    short[16] = 0x7D;
    assert_eq!(exchange(&mut sim, &short), INFO);
    // Same shape, wrong bytes 8 and 9: returns with no answer (BL:610).
    short[8] = 12;
    assert_eq!(exchange(&mut sim, &short), []);
    // BL:621-627: byte 40 is 0x7D, byte 32 is 13, byte 33 is 66.
    let mut long = [0u8; 41];
    long[32] = 13;
    long[33] = 66;
    long[40] = 0x7D;
    assert_eq!(exchange(&mut sim, &long), INFO);
    // The text "BLHeli" and the F4 are not looked at: only 13, 66 and 0x7D count.
    let mut odd = INIT;
    odd[14..20].fill(0x55);
    assert_eq!(exchange(&mut sim, &odd), INFO);
}

#[test]
fn set_address_answers_30_and_refuses_the_reserved_range() {
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(sim.esc(0).address(), 0x0800_7C00);
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(sim.esc(0).address(), 0x0800_2000);
    // BL:703-706: other addresses below 1024 are refused with C1; the address variable was
    // already overwritten (BL:690), so it is not forgotten.
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_0010), [0xC1]);
    assert_eq!(sim.esc(0).address(), 0x10);
    assert_eq!(sim.esc(0).invalid_command(), 1);
    // 1024 is the first address that maps to flash (BL:707-709).
    let set_1024 = [0xFF, 0x00, 0x04, 0x00, 0x32, 0xD4];
    assert_eq!(exchange(&mut sim, &set_1024), [0x30]);
    assert_eq!(sim.esc(0).address(), 0x0800_0400);
    assert_eq!(sim.esc(0).invalid_command(), 0);
}

#[test]
fn set_address_magic_addresses_select_the_areas() {
    let mut sim = sim();
    // BL:303-319: 0x20 settings block, 0x21 file name (32 bytes below it), 0x22 continue,
    // 0x23 the devinfo struct.
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_20), [0x30]);
    assert_eq!(sim.esc(0).address(), 0x0800_7C00);
    assert_eq!(
        exchange(&mut sim, &[0xFF, 0x00, 0x00, 0x21, 0xF0, 0x0C]),
        [0x30]
    );
    assert_eq!(sim.esc(0).address(), 0x0800_7BE0);
    // 0x22 before any read: the continue address is 0 and the answer is still 30.
    assert_eq!(
        exchange(&mut sim, &[0xFF, 0x00, 0x00, 0x22, 0xB0, 0x0D]),
        [0x30]
    );
    assert_eq!(sim.esc(0).address(), 0);
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_23), [0x30]);
    assert_eq!(sim.esc(0).address(), 0x0800_0D00);
}

#[test]
fn bad_crc_is_answered_c2_and_counted() {
    let mut sim = sim();
    let mut frame = SET_ADDRESS_7C00;
    frame[5] ^= 0x01;
    assert_eq!(exchange(&mut sim, &frame), [0xC2]);
    assert_eq!(sim.esc(0).invalid_command(), 1);
    assert_eq!(sim.esc(0).address(), 0);
    // BL:586-587 ... every command checks its CRC and answers C2 (BL:645, 676, 722, 744, 758, 775).
    for frame in [PROGRAM, ERASE, KEEP_ALIVE, READ_4] {
        let mut bad = frame;
        bad[3] ^= 0x80;
        assert_eq!(exchange(&mut sim, &bad), [0xC2], "{frame:02X?}");
    }
    let mut bad = SET_BUFFER_4;
    bad[5] ^= 0x01;
    assert_eq!(exchange(&mut sim, &bad), [0xC2]);
    assert!(!sim.esc(0).expecting_payload());
    // A good command clears the count (BL:547-553).
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(sim.esc(0).invalid_command(), 0);
}

#[test]
fn set_buffer_has_no_answer_and_the_payload_is_acknowledged() {
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    assert!(sim.esc(0).expecting_payload());
    assert_eq!(exchange(&mut sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    assert!(!sim.esc(0).expecting_payload());
}

#[test]
fn program_writes_the_payload_and_read_returns_it() {
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    assert_eq!(exchange(&mut sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    assert_eq!(exchange(&mut sim, &PROGRAM), [0x30]);
    assert_eq!(
        &sim.esc(0).flash()[0x2000..0x2004],
        [0xDE, 0xAD, 0xBE, 0xEF]
    );
    // Read 4 at 0x2000: data, CRC low byte first, 30 (BL:771-813).
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(
        exchange(&mut sim, &READ_4),
        [0xDE, 0xAD, 0xBE, 0xEF, 0x9B, 0xE5, 0x30]
    );
}

#[test]
fn a_bad_payload_crc_is_answered_c2_and_the_esc_keeps_waiting_for_a_payload() {
    // BL:583-600: incoming_payload_no_command is only cleared by a payload with a good CRC.
    // The configurator's simulator clears it either way (see the disagreement list).
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    let mut bad = PAYLOAD_DEAD_BEEF;
    bad[4] ^= 0xFF;
    assert_eq!(exchange(&mut sim, &bad), [0xC2]);
    assert!(sim.esc(0).expecting_payload());
    assert_eq!(sim.esc(0).invalid_command(), 1);
    // Garbage that is not a CRC-fitting frame is refused again and the ESC still waits.
    assert_eq!(exchange(&mut sim, &[0x12, 0x34, 0x56]), [0xC2]);
    assert!(sim.esc(0).expecting_payload());
    // A payload of the right size whose CRC fits ends it.
    assert_eq!(exchange(&mut sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    assert!(!sim.esc(0).expecting_payload());
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
}

#[test]
fn while_it_waits_for_a_payload_the_esc_takes_the_next_valid_frame_for_one_and_acknowledges_it() {
    // A frame followed by its own CRC has CRC 0 (start 0, no final xor), and the bytes the ESC
    // has not been sent are 0 (BL:1010 clears the receive buffer): any valid command that is
    // not longer than size + 2 bytes passes the payload CRC check at BL:586 and is answered 30
    // and NOT executed. A client that sends set address after a refused payload gets 30 and
    // the ESC keeps its old address.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    let mut bad = PAYLOAD_DEAD_BEEF;
    bad[4] ^= 0xFF;
    assert_eq!(exchange(&mut sim, &bad), [0xC2]);
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(
        sim.esc(0).address(),
        0x0800_2000,
        "not 0x7C00: it was not executed"
    );
    assert!(!sim.esc(0).expecting_payload());
}

#[test]
fn a_payload_takes_at_most_its_size_and_one_more_byte() {
    // BL:1020-1027: with 4 expected, size + 2 = 6 bytes are stored, the next one is read and
    // dropped. A 6-byte payload followed by anything is still a payload.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    let mut long = PAYLOAD_DEAD_BEEF.to_vec();
    long.extend_from_slice(&[0x11, 0x22, 0x33]);
    assert_eq!(exchange(&mut sim, &long), [0x30]);
}

#[test]
fn program_below_the_application_is_refused_and_changes_nothing() {
    let mut sim = sim();
    let before = sim.esc(0).flash().to_vec();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_0400), [0x30]);
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    assert_eq!(exchange(&mut sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    // BL:576-579, 645-649: address < application address gives C1.
    assert_eq!(exchange(&mut sim, &PROGRAM), [0xC1]);
    assert_eq!(sim.esc(0).flash(), &before[..]);
    assert_eq!(sim.esc(0).invalid_command(), 1);
}

#[test]
fn program_with_no_address_is_refused() {
    // The address is 0 after a read and after start-up; 0 is below the application.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &PROGRAM), [0xC1]);
}

#[test]
fn program_at_the_settings_address_stamps_the_bootloader_version_into_byte_2() {
    // BL:650-658: address == EEPROM_START_ADD and the payload longer than 2 bytes.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(exchange(&mut sim, &SET_BUFFER_4), []);
    assert_eq!(exchange(&mut sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    assert_eq!(exchange(&mut sim, &PROGRAM), [0x30]);
    assert_eq!(
        &sim.esc(0).flash()[0x7C00..0x7C04],
        [0xDE, 0xAD, 19, 0xEF],
        "byte 2 is the bootloader version"
    );
    // Two bytes or fewer are written as they are (the condition is "> 2"): set buffer 2,
    // payload A5 5A.
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(
        exchange(&mut sim, &[0xFE, 0x00, 0x00, 0x02, 0xB0, 0x29]),
        []
    );
    assert_eq!(exchange(&mut sim, &[0xA5, 0x5A, 0xFB, 0x6B]), [0x30]);
    assert_eq!(exchange(&mut sim, &PROGRAM), [0x30]);
    assert_eq!(&sim.esc(0).flash()[0x7C00..0x7C04], [0xA5, 0x5A, 19, 0xEF]);
}

#[test]
fn program_is_refused_when_the_part_cannot_write_that_alignment() {
    // Mcu/f421/Src/eeprom.c:11-13: address and length must be multiples of 4, else false and
    // the bootloader answers C1 (BL:660-662). Other parts: 8 bytes, or 2 on the F051.
    let mut esc_sim = Sim::new(vec![EscModel::f421(0x14).with_write_granule(4)]).burst();
    esc_sim.select(0).unwrap();
    let sim = &mut esc_sim;
    assert_eq!(exchange(sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(exchange(sim, &[0xFE, 0x00, 0x00, 0x01, 0xF0, 0x28]), []);
    assert_eq!(exchange(sim, &[0x5A, 0x80, 0x3B]), [0x30]);
    assert_eq!(exchange(sim, &PROGRAM), [0xC1]);
    assert_eq!(sim.esc(0).flash()[0x2000], 0xFF);
    // Four bytes at an aligned address go through.
    assert_eq!(exchange(sim, &SET_ADDRESS_2000), [0x30]);
    assert_eq!(exchange(sim, &SET_BUFFER_4), []);
    assert_eq!(exchange(sim, &PAYLOAD_DEAD_BEEF), [0x30]);
    assert_eq!(exchange(sim, &PROGRAM), [0x30]);
    assert_eq!(sim.esc(0).flash()[0x2000], 0xDE);
}

#[test]
fn erase_checks_the_address_only() {
    let mut sim = sim();
    // BL:753-769: C1 below the application, 30 at or above it, nothing erased.
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_0400), [0x30]);
    assert_eq!(exchange(&mut sim, &ERASE), [0xC1]);
    let before = sim.esc(0).flash().to_vec();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(exchange(&mut sim, &ERASE), [0x30]);
    assert_eq!(sim.esc(0).flash(), &before[..]);
    // The address is kept by erase (only a read forgets it).
    assert_eq!(exchange(&mut sim, &ERASE), [0x30]);
}

#[test]
fn read_returns_data_crc_and_30_then_forgets_the_address() {
    let mut sim = sim();
    // The file name 32 bytes below the settings block: "SIM_TARGET" then 0xFF.
    assert_eq!(
        exchange(&mut sim, &[0xFF, 0x00, 0x00, 0x21, 0xF0, 0x0C]),
        [0x30]
    );
    let reply = exchange(&mut sim, &READ_4);
    assert_eq!(&reply[..4], b"SIM_");
    assert_eq!(reply.len(), 7);
    assert_eq!(reply[6], 0x30);
    // BL:810: the address is forgotten; the next read is refused with C1 (BL:782-786).
    assert_eq!(exchange(&mut sim, &READ_4), [0xC1]);
    assert_eq!(sim.esc(0).address(), 0);
    // BL:807: continue address = address + n, reachable through 0x22.
    assert_eq!(
        exchange(&mut sim, &[0xFF, 0x00, 0x00, 0x22, 0xB0, 0x0D]),
        [0x30]
    );
    let next = exchange(&mut sim, &READ_4);
    assert_eq!(&next[..4], b"TARG");
}

#[test]
fn read_of_zero_means_256() {
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    let reply = exchange(&mut sim, &[0x03, 0x00, 0x00, 0xF0]);
    assert_eq!(reply.len(), 256 + 3);
    assert_eq!(reply[0], 1);
    assert_eq!(reply[258], 0x30);
}

#[test]
fn read_of_the_devinfo_address_returns_the_struct() {
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_23), [0x30]);
    let reply = exchange(&mut sim, &[0x03, 0x1B, 0x40, 0xFB]);
    // BL:244-286, packed: magic 5925e3da and 4eb863d9 little endian, the nine device-info
    // bytes, length 27, shift 0, then firmware 0x1000, file name 0x7BE0, settings 0x7C00,
    // tune 0x7C30 as 16-bit values.
    let expected: [u8; 27] = [
        0xDA, 0xE3, 0x25, 0x59, 0xD9, 0x63, 0xB8, 0x4E, //
        0x34, 0x37, 0x31, 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30, //
        27, 0, 0x00, 0x10, 0xE0, 0x7B, 0x00, 0x7C, 0x30, 0x7C,
    ];
    assert_eq!(&reply[..27], expected);
    assert_eq!(reply.len(), 30);
    assert_eq!(reply[29], 0x30);
}

#[test]
fn keep_alive_answers_c1_without_counting_it() {
    let mut sim = sim();
    // BL:739-751: serialwriteOneChar(0xC1) directly, invalid_command untouched.
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
    assert_eq!(sim.esc(0).invalid_command(), 0);
}

#[test]
fn unknown_commands_are_answered_c1_and_counted() {
    let mut sim = sim();
    // BL:815-816.
    assert_eq!(exchange(&mut sim, &[0x55, 0x00, 0x00, 0x00]), [0xC1]);
    assert_eq!(exchange(&mut sim, &[0x04, 0x01, 0x00, 0x00]), [0xC1]);
    assert_eq!(sim.esc(0).invalid_command(), 2);
}

#[test]
fn one_hundred_and_one_bad_commands_make_the_bootloader_jump() {
    // BL:1278: invalid_command > 100 calls jump(); with a valid application it leaves.
    let mut sim = sim();
    for i in 1..=100 {
        assert_eq!(exchange(&mut sim, &[0x55, 0, 0, 0]), [0xC1]);
        assert_eq!(sim.esc(0).invalid_command(), i);
        assert!(!sim.esc(0).running());
    }
    assert_eq!(exchange(&mut sim, &[0x55, 0, 0, 0]), [0xC1]);
    assert!(sim.esc(0).running());
    // A running application answers nothing.
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), []);
}

#[test]
fn the_jump_is_refused_without_the_first_settings_byte_and_the_count_restarts() {
    // BL:426-436: CHECK_EEPROM_BEFORE_JUMP, the first byte of the block at the settings address
    // must be 1; otherwise invalid_command = 0 and the bootloader stays.
    let mut sim = sim();
    sim.esc_mut(0).flash_mut()[0x7C00] = 0xFF;
    for _ in 0..101 {
        assert_eq!(exchange(&mut sim, &[0x55, 0, 0, 0]), [0xC1]);
    }
    assert!(!sim.esc(0).running());
    assert_eq!(sim.esc(0).invalid_command(), 0);
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
}

#[test]
fn the_jump_is_refused_when_the_application_vectors_are_out_of_range() {
    // BL:438-461.
    let mut sim = sim();
    sim.esc_mut(0).flash_mut()[0x1003] = 0x00; // stack pointer 0x0000_1000: not in RAM
    assert_eq!(exchange(&mut sim, &RUN), [0xC1]);
    assert!(!sim.esc(0).running());
    sim.esc_mut(0).flash_mut()[0x1003] = 0x20;
    sim.esc_mut(0).flash_mut()[0x1007] = 0x09; // entry point 0x0900_1101: beyond 256 KB
    assert_eq!(exchange(&mut sim, &RUN), [0xC1]);
    assert!(!sim.esc(0).running());
}

#[test]
fn run_is_answered_c1_and_then_the_application_starts() {
    // BL:629-634 sets invalid_command = 101 and does not return; BL:815 then answers C1; BL:1278
    // jumps. (The facts file and the configurator's simulator say "no answer": a disagreement.)
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &RUN), [0xC1]);
    assert!(sim.esc(0).running());
}

#[test]
fn run_needs_bytes_one_to_three_zero_and_does_not_look_at_the_rest() {
    // BL:629-633 looks at rxBuffer[1], [2] and [3] only; the CRC of 00 00 is 0000.
    let mut sim = sim();
    assert_eq!(exchange(&mut sim, &[0x00, 0x00, 0x00, 0x01]), [0xC1]);
    assert!(!sim.esc(0).running());
    assert_eq!(sim.esc(0).invalid_command(), 1);
    assert_eq!(exchange(&mut sim, &[0x00, 0x00, 0x00, 0x00, 0x77]), [0xC1]);
    assert!(sim.esc(0).running());
}

// ------------------------------------------------------------------ the line and the faults

#[test]
fn a_line_that_is_not_selected_or_not_there_is_an_error() {
    let mut sim = Sim::four().burst();
    assert_eq!(
        block_on(sim.write(&KEEP_ALIVE)),
        Err(SimError::NoLineSelected)
    );
    assert_eq!(sim.select(4), Err(SimError::NoSuchLine(4)));
    assert_eq!(sim.select(3), Ok(()));
    assert_eq!(
        exchange(&mut sim, &INIT),
        [0x34, 0x37, 0x31, 0x13, 0x1F, 6, 6, 3, 0x30]
    );
}

#[test]
fn traffic_is_recorded_per_line() {
    let mut sim = Sim::four().burst();
    sim.select(2).unwrap();
    exchange(&mut sim, &KEEP_ALIVE);
    sim.select(0).unwrap();
    exchange(&mut sim, &INIT);
    assert_eq!(sim.written(0), [INIT.to_vec()]);
    assert_eq!(sim.written(2), [KEEP_ALIVE.to_vec()]);
    assert!(sim.written(1).is_empty() && sim.written(3).is_empty());
    assert_eq!(sim.esc(2).answers(), [vec![0xC1]]);
    assert!(matches!(sim.log()[0], Wire::Select(2)));
}

#[test]
fn faults_change_the_next_answer_only() {
    let mut sim = sim();
    sim.fault(0, Fault::NoAnswer);
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), []);
    sim.fault(0, Fault::WrongAck(0x55));
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0x55]);
    sim.fault(0, Fault::Replace(vec![1, 2, 3]));
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [1, 2, 3]);
    sim.fault(0, Fault::Truncated(4));
    assert_eq!(exchange(&mut sim, &INIT), &INFO[..4]);
    sim.fault(0, Fault::Pass);
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
}

#[test]
fn a_corrupted_crc_fault_flips_the_low_crc_byte_of_a_read_reply() {
    let mut sim = sim();
    exchange(&mut sim, &SET_ADDRESS_2000);
    sim.fault(0, Fault::CorruptCrc);
    let reply = exchange(&mut sim, &READ_4);
    // Flash is 0xFF there: CRC of FF FF FF FF is 0x9401, so the low byte 01 becomes 00.
    let mut good_sim = self::sim();
    exchange(&mut good_sim, &SET_ADDRESS_2000);
    let good = exchange(&mut good_sim, &READ_4);
    assert_eq!(reply[4], good[4] ^ 1);
    assert_eq!(
        (reply[..4].to_vec(), reply[5..].to_vec()),
        (good[..4].to_vec(), good[5..].to_vec())
    );
}

#[test]
fn a_late_answer_is_readable_only_after_its_delay() {
    let mut sim = sim();
    sim.fault(0, Fault::Late(10_000));
    block_on(async {
        sim.write(&KEEP_ALIVE).await.unwrap();
        let mut buf = [0u8; 4];
        assert_eq!(sim.read(&mut buf, 5_000).await, Ok(0));
        assert_eq!(sim.read(&mut buf, 0).await, Ok(0));
        assert_eq!(sim.read(&mut buf, 20_000).await, Ok(1));
        assert_eq!(buf[0], 0xC1);
    });
}

#[test]
fn answers_arrive_one_byte_time_apart_unless_the_line_buffers() {
    let mut slow = Sim::single();
    slow.select(0).unwrap();
    block_on(async {
        slow.write(&INIT).await.unwrap();
        let mut buf = [0u8; 9];
        // 10 bits of 52 us per byte: one byte per read when the timeout is short.
        assert_eq!(slow.read(&mut buf, 600).await, Ok(1));
        assert_eq!(slow.read(&mut buf, 600).await, Ok(1));
        assert_eq!(slow.read(&mut buf, 100).await, Ok(0));
    });
    let mut split = Sim::single().burst().with_max_per_read(1);
    split.select(0).unwrap();
    block_on(async {
        split.write(&INIT).await.unwrap();
        let mut buf = [0u8; 9];
        for _ in 0..9 {
            assert_eq!(split.read(&mut buf, 600).await, Ok(1));
        }
        assert_eq!(split.read(&mut buf, 600).await, Ok(0));
    });
}

#[test]
fn line_errors_can_be_injected_and_the_break_is_recorded() {
    let mut sim = sim();
    sim.inject_line_error(LineErrorAt::NextWrite);
    assert_eq!(block_on(sim.write(&KEEP_ALIVE)), Err(SimError::Injected));
    assert_eq!(block_on(sim.write(&KEEP_ALIVE)), Ok(()));
    sim.inject_line_error(LineErrorAt::ReadAfter(1));
    let mut buf = [0u8; 4];
    assert_eq!(block_on(sim.read(&mut buf, 0)), Ok(1));
    assert_eq!(block_on(sim.read(&mut buf, 0)), Err(SimError::Injected));
    sim.set_break(true).unwrap();
    assert!(sim.break_on());
    // Bytes written while every line is held low reach nobody.
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), []);
    sim.set_break(false).unwrap();
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
}

#[test]
fn a_damaged_request_is_what_the_esc_sees_and_the_record_keeps_the_original() {
    let mut sim = sim();
    sim.corrupt_burst(0, 1, 5, 0x01);
    assert_eq!(exchange(&mut sim, &KEEP_ALIVE), [0xC1]);
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0xC2]);
    assert_eq!(exchange(&mut sim, &SET_ADDRESS_7C00), [0x30]);
    assert_eq!(sim.esc(0).bursts()[1], SET_ADDRESS_7C00);
}
