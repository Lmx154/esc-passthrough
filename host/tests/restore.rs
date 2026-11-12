//! `esc-passthrough-restore` (the `write` feature) against the fake board: the crate's simulated
//! bootloaders with a 4-byte write unit, behind the crate's session, joined to the tool by
//! in-memory pipes. No serial port is opened except where a test says it is a path that does not
//! exist.
#![cfg(feature = "write")]

mod common;

use common::sim::{Fault, Sim, Wire};
use common::{
    BLOCK_AT, BLOCK_LEN, Board, Finished, block_of, controllers, flashes, quick, start_board,
};
use esc_passthrough::session::Exit;
use esc_passthrough_host::link::Link;
use esc_passthrough_host::write::cli::{self, CONFIRMATION_MISSING, Command};
use esc_passthrough_host::write::{self, Outcome, RestoreOptions, RestoreReport};
use std::collections::VecDeque;
use std::io;
use std::time::Duration;

fn restore_options(esc: u8, file: Vec<u8>, allow_change: bool) -> RestoreOptions {
    RestoreOptions {
        esc,
        file,
        address: 0x7C00,
        allow_change,
        timing: quick(),
    }
}

fn session(
    sim: Sim,
    grant: bool,
    options: &RestoreOptions,
) -> (RestoreReport, String, Vec<Vec<u8>>, Finished) {
    let Board { handle, mut tool } = start_board(sim, grant, 4);
    let mut out = Vec::new();
    let report = write::run(&mut tool, options, &mut out);
    let sent = std::mem::take(&mut tool.sent);
    drop(tool);
    let finished = handle.join().expect("the board thread ended");
    (report, String::from_utf8(out).unwrap(), sent, finished)
}

/// First bytes of the writes on the lines: set-buffer is `FE`, program is `01`, erase `02`.
fn line_writes(sim: &Sim) -> Vec<Vec<u8>> {
    sim.log()
        .iter()
        .filter_map(|wire| match wire {
            Wire::Write(bytes) => Some(bytes.clone()),
            _ => None,
        })
        .collect()
}

fn count_first(writes: &[Vec<u8>], first: u8, len: Option<usize>) -> usize {
    writes
        .iter()
        .filter(|w| w.first() == Some(&first) && len.is_none_or(|n| w.len() == n))
        .count()
}

fn commands(sent: &[Vec<u8>]) -> Vec<u8> {
    sent.iter()
        .map(|frame| if frame[0] == b'$' { frame[4] } else { frame[1] })
        .collect()
}

/// CRC-16/ARC, polynomial 0xA001 reflected, start 0 (the bootloader's CRC).
fn crc_arc(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// A good-looking read answer with a block that is not the one written.
fn wrong_read_answer(block: &[u8]) -> Fault {
    wrong_read_answer_at(block, 50)
}

/// Like `wrong_read_answer`, differing from `block` at exactly `offset`.
fn wrong_read_answer_at(block: &[u8], offset: usize) -> Fault {
    let mut wrong = block.to_vec();
    wrong[offset] ^= 0x55;
    let [low, high] = crc_arc(&wrong).to_le_bytes();
    wrong.extend([low, high, 0x30]);
    Fault::Replace(wrong)
}

fn changed_offsets(before: &[u8], after: &[u8]) -> Vec<usize> {
    before
        .iter()
        .zip(after)
        .enumerate()
        .filter_map(|(at, (a, b))| (a != b).then_some(at))
        .collect()
}

// ---------------------------------------------------------------- the write

#[test]
fn an_identical_write_back_leaves_the_flash_byte_identical_with_one_write_sequence() {
    let sim = controllers(4);
    let before = flashes(&sim);
    let block = block_of(&sim, 0);
    assert_eq!(
        block[2], 19,
        "the bootloader's version stamp equals byte 2 of this block"
    );

    let (report, text, sent, finished) =
        session(sim, true, &restore_options(0, block.clone(), false));

    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(
        report.outcome,
        Outcome::Verified {
            offset2_only: false
        }
    );
    assert_eq!((report.writes_sent, report.writes_acked), (1, 1));
    assert_eq!(finished.exit, Exit::Exited);
    assert_eq!(flashes(&finished.sim), before, "byte-identical");

    // The line log: exactly one set-buffer, one payload, one program, at the block's address.
    let writes = line_writes(&finished.sim);
    assert_eq!(count_first(&writes, 0xFE, None), 1);
    assert_eq!(count_first(&writes, 0x01, Some(4)), 1, "program");
    assert_eq!(count_first(&writes, 0x02, None), 0, "no erase");
    assert_eq!(count_first(&writes, 0x00, Some(4)), 0, "no run command");
    let at = writes.iter().position(|w| w[0] == 0xFE).unwrap();
    assert_eq!(
        writes[at - 1][..4],
        [0xFF, 0x00, 0x7C, 0x00],
        "set address first"
    );
    assert_eq!(writes[at][..4], [0xFE, 0x00, 0x00, BLOCK_LEN as u8]);
    assert_eq!(writes[at + 1].len(), BLOCK_LEN + 2, "payload and its CRC");
    assert_eq!(writes[at + 1][..BLOCK_LEN], block[..]);
    assert_eq!(writes[at + 2][0], 0x01);

    // The tool's requests: opening, init, read, ONE write, read-back, exit.
    assert_eq!(
        commands(&sent),
        [1, 2, 130, 131, 245, 0x37, 0x3A, 0x3B, 0x3A, 0x34]
    );
    let frame = sent.iter().find(|f| f[0] == 0x2F && f[1] == 0x3B).unwrap();
    assert_eq!(frame[2..5], [0x7C, 0x00, BLOCK_LEN as u8]);
    assert_eq!(frame[5..5 + BLOCK_LEN], block[..]);
    assert!(
        text.contains("read-back: 0x00-0xB7 identical to FILE"),
        "{text}"
    );
    assert!(
        text.contains("FINAL: WRITTEN controller 1: 184 bytes at wire address 0x7C00, 1 write(s), read back and verified"),
        "{text}"
    );
    // The break comes last.
    let log = finished.sim.log();
    assert_eq!(log[log.len() - 2], Wire::Break(true));
}

#[test]
fn a_different_file_is_refused_by_default_nothing_is_written_and_the_status_is_two() {
    let sim = controllers(4);
    let before = flashes(&sim);
    let mut file = block_of(&sim, 0);
    file[10] ^= 0xFF;
    file[100] ^= 0x01;

    let (report, text, sent, finished) = session(sim, true, &restore_options(0, file, false));

    assert_eq!(report.status(), 2, "{text}");
    assert_eq!(report.outcome, Outcome::IdenticalOnlyRefused);
    assert_eq!(report.changes, [10, 100]);
    assert_eq!(report.writes_sent, 0);
    assert_eq!(flashes(&finished.sim), before);
    let writes = line_writes(&finished.sim);
    assert_eq!(count_first(&writes, 0xFE, None), 0, "no set-buffer");
    assert_eq!(count_first(&writes, 0x01, Some(4)), 0, "no program");
    assert!(
        sent.iter().all(|f| f[0] == b'$' || f[1] != 0x3B),
        "no write request"
    );
    assert_eq!(
        *commands(&sent).last().unwrap(),
        0x34,
        "passthrough is left"
    );
    assert_eq!(finished.exit, Exit::Exited);
    assert!(text.contains("0x0A: "), "{text}");
    assert!(text.contains("-> "), "{text}");
    assert!(
        text.contains("FINAL: NOTHING WRITTEN to any controller"),
        "{text}"
    );
}

#[test]
fn allow_change_changes_exactly_the_bytes_of_the_block_that_differ() {
    let sim = controllers(4);
    let before = flashes(&sim);
    let mut file = block_of(&sim, 0);
    file[10] ^= 0xFF;
    file[100] ^= 0x01;
    file[183] ^= 0x80;

    let (report, text, _, finished) = session(sim, true, &restore_options(0, file.clone(), true));

    assert_eq!(report.status(), 0, "{text}");
    let after = flashes(&finished.sim);
    assert_eq!(
        changed_offsets(&before[0], &after[0]),
        [BLOCK_AT + 10, BLOCK_AT + 100, BLOCK_AT + 183]
    );
    assert_eq!(after[0][BLOCK_AT..BLOCK_AT + BLOCK_LEN], file[..]);
    for line in 1..4 {
        assert_eq!(after[line], before[line], "controller {}", line + 1);
    }
    assert!(
        text.contains("0x0A: ") && text.contains("--allow-change given"),
        "{text}"
    );
}

#[test]
fn a_difference_at_offset_two_only_is_reported_separately_and_counts_as_verified() {
    let sim = controllers(4);
    let before = flashes(&sim);
    let file = block_of(&sim, 1);
    assert_ne!(file[2], 19);

    let (report, text, _, finished) = session(sim, true, &restore_options(1, file, false));

    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(report.outcome, Outcome::Verified { offset2_only: true });
    assert!(
        text.contains("identical to FILE except offset 0x02") && text.contains("version"),
        "{text}"
    );
    let after = flashes(&finished.sim);
    assert_eq!(changed_offsets(&before[1], &after[1]), [BLOCK_AT + 2]);
    assert!(text.contains("controller 2: 184 bytes"), "{text}");
}

#[test]
fn a_write_whose_payload_fails_once_is_repeated_and_ends_verified() {
    let mut sim = controllers(4);
    let before = flashes(&sim);
    let block = block_of(&sim, 0);
    // Answers of controller 1: device info, set address, read | set address, PAYLOAD (bad CRC).
    sim.faults(
        0,
        std::iter::repeat_n(Fault::Pass, 4).chain([Fault::WrongAck(0xC2)]),
    );

    let (report, text, sent, finished) = session(sim, true, &restore_options(0, block, false));

    assert_eq!(report.status(), 0, "{text}");
    assert_eq!((report.writes_sent, report.writes_acked), (2, 1));
    assert!(
        text.contains("write answered with error ACK_D_GENERAL_ERROR"),
        "{text}"
    );
    assert!(text.contains("write 2 of 3"), "{text}");
    assert_eq!(
        sent.iter().filter(|f| f[0] == 0x2F && f[1] == 0x3B).count(),
        2
    );
    assert_eq!(count_first(&line_writes(&finished.sim), 0xFE, None), 2);
    assert_eq!(flashes(&finished.sim), before);
}

#[test]
fn a_read_back_that_keeps_differing_ends_with_status_four_after_exactly_three_writes() {
    let mut sim = controllers(4);
    let block = block_of(&sim, 0);
    // Per write: set address, payload, program; then the read-back: set address, read answer.
    let mut faults = Vec::new();
    faults.extend(std::iter::repeat_n(Fault::Pass, 3)); // device info, set address, first read
    for _ in 0..3 {
        faults.extend(std::iter::repeat_n(Fault::Pass, 3));
        faults.push(Fault::Pass);
        faults.push(wrong_read_answer(&block));
    }
    sim.faults(0, faults);

    let (report, text, sent, finished) =
        session(sim, true, &restore_options(0, block.clone(), false));

    assert_eq!(report.status(), 4, "{text}");
    assert_eq!(report.outcome, Outcome::NotVerified);
    assert_eq!(report.writes_sent, 3);
    assert_eq!(
        sent.iter().filter(|f| f[0] == 0x2F && f[1] == 0x3B).count(),
        3
    );
    assert_eq!(count_first(&line_writes(&finished.sim), 0xFE, None), 3);
    assert!(text.contains("NOT VERIFIED after 3 writes"), "{text}");
    assert!(
        text.contains("0x32-0x32: FILE"),
        "the differing range is printed\n{text}"
    );
    assert!(
        text.contains("Block read back from the controller:"),
        "{text}"
    );
    assert_eq!(report.readback.as_ref().unwrap()[50], block[50] ^ 0x55);
    assert!(
        text.contains("FINAL: WRITTEN BUT NOT VERIFIED controller 1"),
        "{text}"
    );
    assert_eq!(*commands(&sent).last().unwrap(), 0x34);
}

fn a_single_byte_difference_in_the_read_back_at(offset: usize) {
    let mut sim = controllers(4);
    let block = block_of(&sim, 0);
    let mut faults = Vec::new();
    faults.extend(std::iter::repeat_n(Fault::Pass, 3));
    for _ in 0..3 {
        faults.extend(std::iter::repeat_n(Fault::Pass, 4));
        faults.push(wrong_read_answer_at(&block, offset));
    }
    sim.faults(0, faults);

    let (report, text, _, _) = session(sim, true, &restore_options(0, block.clone(), false));

    assert_eq!(report.status(), 4, "{text}");
    assert_eq!(report.outcome, Outcome::NotVerified);
    assert_eq!((report.writes_sent, report.writes_acked), (3, 3));
    assert_eq!(
        report.readback.as_ref().unwrap()[offset],
        block[offset] ^ 0x55
    );
    let range = format!("0x{offset:02X}-0x{offset:02X}: FILE");
    assert!(text.contains(&range), "{range}\n{text}");
    assert!(
        text.contains("FINAL: WRITTEN BUT NOT VERIFIED controller 1"),
        "{text}"
    );
}

#[test]
fn a_read_back_differing_only_at_the_first_offset_is_not_verified() {
    a_single_byte_difference_in_the_read_back_at(0x00);
}

#[test]
fn a_read_back_differing_only_in_the_second_half_is_not_verified() {
    a_single_byte_difference_in_the_read_back_at(0x5C);
}

#[test]
fn a_read_back_differing_only_at_the_last_offset_is_not_verified() {
    a_single_byte_difference_in_the_read_back_at(BLOCK_LEN - 1);
}

/// Every write is answered with an error (the payload's acknowledgement), then the read-back
/// taken after the loop is `last`.
fn all_writes_refused(last: Fault) -> (RestoreReport, String) {
    let mut sim = controllers(4);
    let block = block_of(&sim, 0);
    let mut faults = Vec::new();
    faults.extend(std::iter::repeat_n(Fault::Pass, 3));
    for _ in 0..3 {
        faults.push(Fault::Pass);
        faults.push(Fault::WrongAck(0xC2));
        faults.push(Fault::Pass); // the program step still answers
    }
    faults.push(Fault::Pass);
    faults.push(last);
    sim.faults(0, faults);
    let (report, text, _, _) = session(sim, true, &restore_options(0, block, false));
    (report, text)
}

#[test]
fn when_no_write_was_accepted_the_final_line_does_not_say_written_and_status_is_four() {
    let (report, text) = all_writes_refused(Fault::Pass);
    assert_eq!(report.status(), 4, "{text}");
    assert_eq!(report.outcome, Outcome::NotVerified);
    assert_eq!((report.writes_sent, report.writes_acked), (3, 0));
    assert!(
        text.contains("FINAL: NO WRITE ACCEPTED by controller 1: 3 write(s) of 184 bytes at wire address 0x7C00 sent, all answered with an error; the block in the controller is unchanged (identical to the block read before the write)"),
        "{text}"
    );
    assert!(!text.contains("WRITTEN BUT"), "{text}");

    let block = report.before.clone().unwrap();
    let (report, text) = all_writes_refused(wrong_read_answer(&block));
    assert_eq!(report.status(), 4, "{text}");
    assert!(
        text.contains("FINAL: NO WRITE ACCEPTED by controller 1: 3 write(s) of 184 bytes at wire address 0x7C00 sent, all answered with an error; the block in the controller DIFFERS from the block read before the write: act on it"),
        "{text}"
    );
    assert!(!text.contains("WRITTEN BUT"), "{text}");
}

// ---------------------------------------------------------------- the other statuses

#[test]
fn a_refused_passthrough_is_status_three_and_nothing_reaches_the_lines() {
    let sim = controllers(4);
    let block = block_of(&sim, 0);
    let (report, text, sent, finished) = session(sim, false, &restore_options(0, block, false));
    assert_eq!(report.status(), 3, "{text}");
    assert_eq!(sent.len(), 5);
    assert!(finished.sim.log().is_empty());
    assert!(text.contains("FINAL: NOTHING WRITTEN"), "{text}");
}

#[test]
fn a_controller_the_board_does_not_have_is_status_one_with_no_write_sent() {
    let sim = controllers(4);
    let block = block_of(&sim, 0);
    let (report, text, sent, finished) = session(sim, true, &restore_options(4, block, false));
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!(report.writes_sent, 0);
    assert!(sent.iter().all(|f| f[0] == b'$' || f[1] != 0x3B));
    assert!(
        text.contains("controller 5 asked for, the board reports 4"),
        "{text}"
    );
    assert_eq!(finished.exit, Exit::Exited);
}

#[test]
fn a_controller_that_never_answers_is_status_one_and_says_no_write_was_sent() {
    let mut sim = controllers(4);
    sim.faults(0, std::iter::repeat_n(Fault::NoAnswer, 100));
    let block = block_of(&sim, 0);
    let (report, text, _, finished) = session(sim, true, &restore_options(0, block, false));
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!(report.writes_sent, 0);
    assert!(
        text.contains("FINAL: NOTHING WRITTEN to any controller (no write was sent)"),
        "{text}"
    );
    assert_eq!(count_first(&line_writes(&finished.sim), 0xFE, None), 0);
}

// ---------------------------------------------------------------- a write that gets no reply

/// Answers each write with the next scripted reply (`None` is silence).
struct Scripted {
    script: VecDeque<Option<Vec<u8>>>,
    sent: Vec<Vec<u8>>,
    pending: VecDeque<u8>,
}

impl Link for Scripted {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.sent.push(bytes.to_vec());
        if let Some(Some(reply)) = self.script.pop_front() {
            self.pending.extend(reply);
        }
        Ok(())
    }
    fn read_some(&mut self, buf: &mut [u8], _timeout: Duration) -> io::Result<usize> {
        if self.pending.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(self.pending.len());
        for slot in buf.iter_mut().take(n) {
            *slot = self.pending.pop_front().unwrap_or_default();
        }
        Ok(n)
    }
    fn discard_input(&mut self) -> io::Result<()> {
        self.pending.clear();
        Ok(())
    }
}

fn crc_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn fw_reply(command: u8, address: u16, params: &[u8], ack: u8) -> Option<Vec<u8>> {
    let count = if params.len() == 256 {
        0
    } else {
        params.len() as u8
    };
    let mut frame = vec![0x2E, command, (address >> 8) as u8, address as u8, count];
    frame.extend_from_slice(params);
    frame.push(ack);
    frame.extend(crc_xmodem(&frame).to_be_bytes());
    Some(frame)
}

fn msp_reply(command: u8, payload: &[u8]) -> Option<Vec<u8>> {
    let mut frame = vec![b'$', b'M', b'>', payload.len() as u8, command];
    frame.extend_from_slice(payload);
    frame.push(frame[3..].iter().fold(0, |a, b| a ^ b));
    Some(frame)
}

#[test]
fn a_write_without_a_usable_reply_is_status_one_and_says_a_write_was_sent() {
    let block = [7u8; 184];
    let mut motors = [0u8; 10];
    motors[6] = 1;
    let script = vec![
        msp_reply(1, &[0, 1, 42]),
        msp_reply(2, b"ARDU"),
        msp_reply(130, &[0, 0, 0, 252, 0, 0, 0, 0]),
        msp_reply(131, &motors),
        msp_reply(245, &[1]),
        fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0),
        fw_reply(0x3A, 0x7C00, &block, 0),
        None, // the write: silence
        fw_reply(0x34, 0, &[0], 0),
    ];
    let mut link = Scripted {
        script: script.into(),
        sent: Vec::new(),
        pending: VecDeque::new(),
    };
    let mut out = Vec::new();
    let report = write::run(
        &mut link,
        &restore_options(0, block.to_vec(), false),
        &mut out,
    );
    let text = String::from_utf8(out).unwrap();
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!((report.writes_sent, report.writes_acked), (1, 0));
    assert!(
        text.contains("FINAL: A WRITE WAS SENT to controller 1 (1 sent, 0 acknowledged)"),
        "{text}"
    );
    assert_eq!(
        commands(&link.sent),
        [1, 2, 130, 131, 245, 0x37, 0x3A, 0x3B, 0x34]
    );
}

// ---------------------------------------------------------------- the command line

fn parse(args: &[&str]) -> Result<Command, String> {
    cli::parse(args.iter().map(|s| s.to_string()))
}

#[test]
fn without_the_confirmation_flag_the_parser_refuses_so_nothing_can_be_opened() {
    assert_eq!(
        parse(&["--esc", "1", "--file", "block.bin"]),
        Err(CONFIRMATION_MISSING.to_string())
    );
    let Ok(Command::Run(args)) = parse(&[
        "--esc",
        "2",
        "--file",
        "block.bin",
        "--yes-write-to-esc",
        "--allow-change",
    ]) else {
        panic!("rejected");
    };
    assert_eq!(
        (args.esc, args.address, args.length, args.allow_change),
        (2, 0x7C00, 184, true)
    );
    for bad in [
        &["--yes-write-to-esc", "--file", "f"][..],
        &["--yes-write-to-esc", "--esc", "1"],
        &["--yes-write-to-esc", "--esc", "0", "--file", "f"],
        &["--yes-write-to-esc", "--esc", "9", "--file", "f"],
        &[
            "--yes-write-to-esc",
            "--esc",
            "1",
            "--file",
            "f",
            "--length",
            "1",
        ],
        &["--yes-write-to-esc", "--esc", "1", "--file", "f", "--bogus"],
        &[
            "--yes-write-to-esc",
            "--esc",
            "1",
            "--file",
            "f",
            "--port",
            "a",
            "--vid-pid",
            "1:2",
        ],
    ] {
        assert!(parse(bad).is_err(), "{bad:?}");
    }
}

#[test]
fn the_binary_stops_before_opening_the_port_without_the_confirmation_flag() {
    let dir = std::env::temp_dir().join(format!("esc-host-restore-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join("block.bin");
    std::fs::write(&file, [0u8; 184]).unwrap();
    let missing_port = dir.join("no-such-port");
    let run = |extra: &[&str]| {
        std::process::Command::new(env!("CARGO_BIN_EXE_esc-passthrough-restore"))
            .args(["--esc", "1", "--file"])
            .arg(&file)
            .arg("--port")
            .arg(&missing_port)
            .args(extra)
            .output()
            .unwrap()
    };
    // Without the flag: refused as a command line fault (64) before any open is attempted.
    let refused = run(&[]);
    assert_eq!(refused.status.code(), Some(64));
    let stderr = String::from_utf8_lossy(&refused.stderr);
    assert!(
        stderr.contains("--yes-write-to-esc is required"),
        "{stderr}"
    );
    assert!(stderr.contains("nothing was opened"), "{stderr}");
    assert!(!String::from_utf8_lossy(&refused.stdout).contains("port:"));
    // Control: with the flag the same command line gets as far as trying to open the (missing) path.
    let attempted = run(&["--yes-write-to-esc"]);
    assert_eq!(attempted.status.code(), Some(1));
    let stderr = String::from_utf8_lossy(&attempted.stderr);
    assert!(stderr.contains("open "), "{stderr}");
    assert!(
        String::from_utf8_lossy(&attempted.stdout).contains("NOTHING WRITTEN"),
        "{}",
        String::from_utf8_lossy(&attempted.stdout)
    );
    std::fs::remove_dir_all(&dir).unwrap();
}
