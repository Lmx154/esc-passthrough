//! The tool's library against a FAKE BOARD in the same process.
//!
//! Wiring: two in-memory byte pipes join the tool (this thread, as a [`Link`]) to a second thread
//! that runs `esc_passthrough::session::Passthrough::run` (the crate under test: the flight
//! controller side) on a tiny executor. The lender hands the session the crate's simulated
//! bootloaders (`tests/common/sim.rs` of the crate, included by path, not copied) as the
//! controllers' lines. The fake's delay returns at once for the zero-length park and break; it
//! never returns for the long idle wait, which therefore never fires.
//!
//! No serial port is opened anywhere in this file.

mod common;

use common::sim::{EscModel, Fault, Sim, Wire};
use common::{Board, Finished, block_of, flashes, four_controllers, quick, start_board};
use esc_passthrough::session::Exit;
use esc_passthrough_host::tool::{self, Comparison, Options, Report};

fn options() -> Options {
    Options {
        timing: quick(),
        ..Options::new()
    }
}

/// Runs the tool against the board, then lets the tool's end go and collects the board.
fn session(sim: Sim, grant: bool, options: &Options) -> (Report, String, Vec<Vec<u8>>, Finished) {
    let Board { handle, mut tool } = start_board(sim, grant, 4);
    let mut out = Vec::new();
    let report = tool::run(&mut tool, options, &mut out);
    let sent = std::mem::take(&mut tool.sent);
    drop(tool);
    let finished = handle.join().expect("the board thread ended");
    (report, String::from_utf8(out).unwrap(), sent, finished)
}

// ---------------------------------------------------------------- the proof

#[test]
fn the_report_equals_what_the_simulator_holds_and_nothing_was_written() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let expected: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    assert!(
        expected.windows(2).all(|pair| pair[0] != pair[1]),
        "four different blocks"
    );

    let (report, text, sent, finished) = session(sim, true, &options());

    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(finished.exit, Exit::Exited);
    assert_eq!(report.api_version, Some([0, 1, 42]));
    assert_eq!(report.variant.as_deref(), Some("ARDU"));
    assert_eq!(report.battery_decivolts, Some(252));
    assert_eq!(report.motor_count, Some(4));
    assert_eq!(report.passthrough, tool::Passthrough::Granted(4));
    assert_eq!(report.escs.len(), 4);
    for (line, esc) in report.escs.iter().enumerate() {
        assert_eq!(esc.signature, Some(0x1F06), "{text}");
        assert_eq!(esc.pin_code, Some(0x10 + line as u8));
        assert_eq!(esc.interface_mode, Some(4));
        assert_eq!(
            esc.block.as_deref(),
            Some(&expected[line][..]),
            "controller {line}"
        );
        assert_eq!(esc.error, None);
    }
    assert!(report.exit_sent && report.exit_answered);
    assert!(text.contains("controller 4:"), "{text}");
    assert!(
        text.contains("0000  01 "),
        "the block is printed as hex\n{text}"
    );
    assert!(text.contains(" ms (try 1 of 10)"), "{text}");

    // The simulator's flash is byte-identical before and after: nothing was written.
    assert_eq!(flashes(&finished.sim), before);

    // The tool's own requests: only MSP, init-flash, read and exit.
    let commands: Vec<u8> = sent
        .iter()
        .map(|frame| if frame[0] == b'$' { frame[4] } else { frame[1] })
        .collect();
    assert_eq!(
        commands,
        [
            1, 2, 130, 131, 245, 0x37, 0x3A, 0x37, 0x3A, 0x37, 0x3A, 0x37, 0x3A, 0x34
        ]
    );

    // What reached the controllers' lines: the 21-byte init, set-address (FF) and read (03);
    // no set-buffer (FE), program (01), erase (02) or run (00 00 00 00).
    for line in 0..4 {
        let bursts = finished.sim.written(line);
        assert_eq!(bursts.len(), 3, "init, set address, read on line {line}");
        assert_eq!(bursts[0].len(), 21);
        assert_eq!(bursts[1][0], 0xFF);
        assert_eq!(bursts[2][0], 0x03);
        assert!(!finished.sim.esc(line).running());
    }
}

#[test]
fn the_line_log_shows_the_exit_break_after_interface_exit_and_nothing_after_it() {
    let (report, text, sent, finished) = session(four_controllers(), true, &options());
    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(sent.last().map(|f| f[1]), Some(0x34));
    let log = finished.sim.log();
    let breaks: Vec<usize> = log
        .iter()
        .enumerate()
        .filter_map(|(at, wire)| matches!(wire, Wire::Break(_)).then_some(at))
        .collect();
    assert_eq!(
        breaks,
        [log.len() - 2, log.len() - 1],
        "one break pair, at the very end"
    );
    assert_eq!(log[log.len() - 2], Wire::Break(true));
    assert_eq!(log[log.len() - 1], Wire::Break(false));
    assert!(
        log[..log.len() - 2]
            .iter()
            .any(|w| matches!(w, Wire::Write(_)))
    );
    assert!(!finished.sim.break_on());
}

#[test]
fn no_exit_leaves_the_session_to_end_by_the_closed_link_and_sends_no_exit() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let (report, text, sent, finished) = session(
        sim,
        true,
        &Options {
            no_exit: true,
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert!(!report.exit_sent);
    assert!(sent.iter().all(|f| f[0] == b'$' || f[1] != 0x34));
    // Without an exit request the session ends when the link closes (and then breaks the lines).
    assert_eq!(finished.exit, Exit::Closed);
    assert_eq!(flashes(&finished.sim), before);
}

#[test]
fn a_refusal_is_reported_as_exit_status_three_with_no_traffic_on_the_lines() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let (report, text, sent, finished) = session(sim, false, &options());
    assert_eq!(report.status(), 3, "{text}");
    assert_eq!(report.passthrough, tool::Passthrough::Refused);
    assert!(text.contains("REFUSED"), "{text}");
    assert_eq!(sent.len(), 5, "the five MSP requests only");
    assert!(finished.sim.log().is_empty(), "{:?}", finished.sim.log());
    assert_eq!(flashes(&finished.sim), before);
    assert_eq!(finished.exit, Exit::Closed);
}

#[test]
fn comparison_files_are_reported_equal_or_by_differing_offsets() {
    let sim = four_controllers();
    let mut files: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    let (report, text, _, _) = session(
        four_controllers(),
        true,
        &Options {
            compare: files.clone(),
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert!(
        report
            .escs
            .iter()
            .all(|esc| esc.comparison == Comparison::Equal)
    );
    assert_eq!(text.matches("compare: equal").count(), 4, "{text}");

    files[2][10] ^= 0x01;
    files[2][183] ^= 0x80;
    let (report, text, _, _) = session(
        four_controllers(),
        true,
        &Options {
            compare: files,
            ..options()
        },
    );
    assert_eq!(report.status(), 2, "{text}");
    assert_eq!(report.escs[1].comparison, Comparison::Equal);
    assert_eq!(
        report.escs[2].comparison,
        Comparison::Differs {
            offsets: vec![10, 183],
            block_len: 184,
            file_len: 184
        }
    );
    assert!(text.contains("differing offsets 0x0A 0xB7"), "{text}");
}

#[test]
fn blocks_are_saved_as_escn_bin() {
    let sim = four_controllers();
    let expected: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    let (report, _, _, _) = session(sim, true, &options());
    let dir = std::env::temp_dir().join(format!("esc-host-fake-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let paths = tool::save_blocks(&dir, &report).unwrap();
    assert_eq!(paths.len(), 4);
    for (n, path) in paths.iter().enumerate() {
        assert_eq!(
            path.file_name().unwrap().to_str().unwrap(),
            format!("esc{}.bin", n + 1)
        );
        assert_eq!(std::fs::read(path).unwrap(), expected[n]);
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_controller_silent_for_three_tries_is_read_and_one_silent_for_twelve_is_status_one() {
    let mut sim = four_controllers();
    let before = flashes(&sim);
    // Controller 2 stays silent for its first three init tries; controller 3 for more than ten.
    sim.faults(1, std::iter::repeat_n(Fault::NoAnswer, 3));
    sim.faults(2, std::iter::repeat_n(Fault::NoAnswer, 12));
    let (report, text, sent, finished) = session(sim, true, &options());
    assert_eq!(report.status(), 1, "{text}");
    assert!(report.escs[0].error.is_none());
    assert!(report.escs[1].error.is_none(), "{text}");
    assert!(text.contains("(try 4 of 10)"), "{text}");
    assert!(
        report.escs[2]
            .error
            .as_deref()
            .unwrap_or("")
            .contains("max retries reached")
    );
    assert!(report.escs[3].error.is_none());
    assert!(report.escs[3].block.is_some());
    let inits_for_third = sent
        .iter()
        .filter(|f| f[0] == 0x2F && f[1] == 0x37 && f[5] == 2)
        .count();
    assert_eq!(inits_for_third, 10);
    assert_eq!(flashes(&finished.sim), before);
}

#[test]
fn asking_for_more_controllers_than_the_board_has_is_status_one() {
    let (report, text, _, finished) = session(
        four_controllers(),
        true,
        &Options {
            escs: Some(5),
            ..options()
        },
    );
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!(report.escs.len(), 5);
    assert!(report.escs[..4].iter().all(|esc| esc.block.is_some()));
    assert!(
        report.escs[4]
            .error
            .as_deref()
            .unwrap_or("")
            .contains("ACK_I_INVALID_CHANNEL"),
        "{text}"
    );
    assert_eq!(finished.exit, Exit::Exited);
}

#[test]
fn a_single_controller_board_is_read_with_the_default_count() {
    let sim = Sim::new(vec![EscModel::f421(0x14)]);
    let before = sim.esc(0).flash().to_vec();
    let Board { handle, mut tool } = start_board(sim, true, 1);
    let mut out = Vec::new();
    let report = tool::run(&mut tool, &options(), &mut out);
    drop(tool);
    let finished = handle.join().unwrap();
    assert_eq!(report.status(), 0, "{}", String::from_utf8_lossy(&out));
    assert_eq!(report.escs.len(), 1);
    assert_eq!(report.escs[0].pin_code, Some(0x14));
    assert_eq!(finished.sim.esc(0).flash(), before);
}

// ---------------------------------------------------------------- --dump

#[test]
fn a_dump_spanning_several_chunks_equals_the_simulators_image() {
    let sim = four_controllers();
    // 700 bytes at 0x1000: chunks of 256, 256 and 188 bytes.
    let images: Vec<Vec<u8>> = (0..4)
        .map(|line| sim.esc(line).flash()[0x1000..0x1000 + 700].to_vec())
        .collect();
    let before = flashes(&sim);
    let dump = tool::Dump {
        start: 0x1000,
        length: 700,
    };
    let (report, text, sent, finished) = session(
        sim,
        true,
        &Options {
            dump: Some(dump),
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    for (line, esc) in report.escs.iter().enumerate() {
        assert_eq!(
            esc.dump.as_deref(),
            Some(&images[line][..]),
            "controller {line}"
        );
    }
    assert!(
        text.contains("dump: 700 bytes at 0x1000 in 3 chunk(s)"),
        "{text}"
    );
    assert!(text.contains("dump: 700 bytes in "), "{text}");
    // Only reads: per controller init, settings read, three chunk reads; then the exit.
    let commands: Vec<u8> = sent
        .iter()
        .map(|frame| if frame[0] == b'$' { frame[4] } else { frame[1] })
        .collect();
    let mut expected = vec![1, 2, 130, 131, 245];
    for _ in 0..4 {
        expected.extend([0x37, 0x3A, 0x3A, 0x3A, 0x3A]);
    }
    expected.push(0x34);
    assert_eq!(commands, expected);
    assert_eq!(flashes(&finished.sim), before);

    // The files are named after the range.
    let dir = std::env::temp_dir().join(format!("esc-host-dump-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let paths = tool::save_dumps(&dir, &report, dump).unwrap();
    assert_eq!(paths.len(), 4);
    assert_eq!(
        paths[1].file_name().unwrap().to_str().unwrap(),
        "esc2-flash-0x1000-0x02bc.bin"
    );
    assert_eq!(std::fs::read(&paths[1]).unwrap(), images[1]);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_dump_prints_progress_every_sixteen_chunks_and_the_total_time() {
    let sim = Sim::new(vec![EscModel::f421(0x14)]);
    let image = sim.esc(0).flash()[0x1000..0x1000 + 8192].to_vec();
    let Board { handle, mut tool } = start_board(sim, true, 1);
    let mut out = Vec::new();
    let options = Options {
        dump: Some(tool::Dump {
            start: 0x1000,
            length: 8192,
        }),
        ..options()
    };
    let report = tool::run(&mut tool, &options, &mut out);
    drop(tool);
    handle.join().unwrap();
    let text = String::from_utf8(out).unwrap();
    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(report.escs[0].dump.as_deref(), Some(&image[..]));
    assert!(
        text.contains("dump: 16 of 32 chunks, 4096 of 8192 bytes"),
        "{text}"
    );
    assert!(
        text.contains("dump: 32 of 32 chunks, 8192 of 8192 bytes"),
        "{text}"
    );
    assert_eq!(text.matches(" of 32 chunks, ").count(), 2, "{text}");
}

#[test]
fn a_refused_chunk_is_reported_with_its_address_and_the_file_is_not_written() {
    // Addresses below 0x400 are refused by the simulated bootloader.
    let dump = tool::Dump {
        start: 0x0300,
        length: 0x200,
    };
    let (report, text, _, finished) = session(
        four_controllers(),
        true,
        &Options {
            dump: Some(dump),
            ..options()
        },
    );
    assert_eq!(report.status(), 1, "{text}");
    for esc in &report.escs {
        assert!(esc.dump.is_none());
        assert!(esc.block.is_some(), "the settings block was still read");
        assert!(
            esc.error
                .as_deref()
                .unwrap_or("")
                .contains("chunk 1 of 2 at address 0x0300"),
            "{esc:?}"
        );
    }
    assert!(
        text.contains("dump failed: chunk 1 of 2 at address 0x0300"),
        "{text}"
    );
    let dir = std::env::temp_dir().join(format!("esc-host-nodump-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    assert!(tool::save_dumps(&dir, &report, dump).unwrap().is_empty());
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 0);
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(finished.exit, Exit::Exited);
}

#[test]
fn a_dump_that_fails_for_one_controller_leaves_the_others_dumped() {
    let mut sim = four_controllers();
    // Controller 2 answers its init and settings read (3 answers), then nothing.
    sim.faults(
        1,
        std::iter::repeat_n(Fault::Pass, 3).chain(std::iter::repeat_n(Fault::NoAnswer, 200)),
    );
    let dump = tool::Dump {
        start: 0x1000,
        length: 300,
    };
    let (report, text, _, _) = session(
        sim,
        true,
        &Options {
            dump: Some(dump),
            ..options()
        },
    );
    assert_eq!(report.status(), 1, "{text}");
    assert!(report.escs[0].dump.is_some());
    assert!(report.escs[1].dump.is_none());
    assert!(
        report.escs[1]
            .error
            .as_deref()
            .unwrap_or("")
            .contains("chunk 1 of 2 at address 0x1000")
    );
    assert!(report.escs[2].dump.is_some() && report.escs[3].dump.is_some());
}
