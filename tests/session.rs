//! The session against the simulated bootloader: the configurator's conversation end to end, the
//! exits, the error paths and the configuration.
//!
//! The expected bytes are laid out here by hand. The short MSP and four-way frames are literal
//! (their checksums computed with a separate script); the long reads are built by the frame builders
//! of `session_support` from the simulator's flash, with the bit-serial reference CRC.
#![allow(dead_code)]

mod common;
#[path = "common/session_support.rs"]
mod support;
#[path = "common/session_talk.rs"]
mod talk;

use common::ref_crc_arc;
use common::sim::{Fault, Sim, Wire};
use esc_passthrough::client::Timeouts;
use esc_passthrough::host::HostInfo;
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Passthrough, ReleaseError, SessionConfig};
use std::cell::Cell;
use std::rc::Rc;
use support::*;
use talk::*;

#[test]
fn run_conversation_end_to_end() {
    let sim = four_escs();
    let talk = full_talk(&sim);
    let requests = talk.replies.len();
    let mut rig = rig(talk.script, vec![Some(sim)], SessionConfig::new());
    let battery_calls = Rc::new(Cell::new(0));
    let counter = battery_calls.clone();
    let exit = run_with_battery(&mut rig, &[], move || {
        counter.set(counter.get() + 1);
        BatteryState::new(16_800)
    });
    assert_eq!(exit, Exit::Exited);
    assert_eq!(battery_calls.get(), 1);

    // Every reply byte, one write each, in order.
    assert_eq!(rig.host.written(), talk.replies);
    // Invariant 5.2: nothing was written before a complete request, one reply per request.
    assert_eq!(rig.host.complete_requests(), requests);
    assert_eq!(rig.host.writes(), requests);
    assert_eq!(rig.host.early_writes(), 0);
    assert_eq!(rig.host.undelivered(), 0);

    // The lender was asked once, at the passthrough request.
    assert_eq!(rig.lender.asked, 1);

    let log = rig.world.log();
    let lend_at = log.iter().position(|(_, e)| *e == Ev::Lend).unwrap();
    let grant_write = log
        .iter()
        .position(|(_, e)| *e == Ev::HostWrite(REP_PASSTHROUGH_4.to_vec()))
        .unwrap();
    assert!(lend_at < grant_write);
    let grant_us = log[grant_write].0;

    // Nothing on any line before park_ms (3000) have passed since the grant, and the delay was the
    // session's own request.
    let first_line = log
        .iter()
        .position(|(_, e)| matches!(e, Ev::Line(_)))
        .unwrap();
    assert!(log[first_line].0 >= grant_us + 3_000_000);
    assert!(
        log[first_line].0 < grant_us + 3_100_000,
        "park, then at once"
    );
    assert!(
        log[grant_write..first_line]
            .iter()
            .all(|(_, e)| { matches!(e, Ev::Delay(_) | Ev::Flush | Ev::HostWrite(_)) })
    );
    assert!(rig.world.delays().contains(&(grant_us, 3_000)));

    // Traffic only on the selected line: each line's bootloader got exactly its own bursts.
    let lines = rig.session.release().unwrap();
    for line in 0..4 {
        assert_eq!(lines.sim.written(line), expected_bursts(), "line {line}");
    }
    // And selection: every write and read follows a select of the line of that controller; the
    // order is init 0 (select 0), its reads, init 1 (select 1), ...
    let events = rig.world.line_events();
    let mut active = None;
    let mut seen = Vec::new();
    for (_, wire) in &events {
        match wire {
            Wire::Select(line) => active = Some(*line),
            Wire::Write(_) | Wire::Read { .. } => {
                if seen.last() != active.as_ref() {
                    seen.push(active.unwrap());
                }
            }
            Wire::Break(_) => {}
        }
    }
    assert_eq!(seen, vec![0, 1, 2, 3]);

    // The exit: one break, after the last traffic and after the exit reply, low for at least
    // break_ms on all lines (set_break is for all), then released.
    let exit_reply = log
        .iter()
        .position(|(_, e)| *e == Ev::HostWrite(REP_EXIT.to_vec()))
        .unwrap();
    let ons = break_on_positions(&events);
    assert_eq!(ons.len(), 1);
    assert_eq!(events.len(), ons[0] + 2);
    assert_eq!(events[ons[0] + 1].1, Wire::Break(false));
    assert!(events[ons[0] + 1].0 - events[ons[0]].0 >= 25_000);
    let break_in_log = log
        .iter()
        .position(|(_, e)| *e == Ev::Line(Wire::Break(true)))
        .unwrap();
    assert!(exit_reply < break_in_log);
    assert!(!lines.sim.break_on());
}

// ---------------------------------------------------------------------------------------------
// helpers for the single-conversation tests

/// The passthrough request, its reply, and the 2 s wait.
fn granted(talk: &mut Talk) {
    talk.ask(&REQ_PASSTHROUGH, REP_PASSTHROUGH_4.to_vec());
    talk.script.push(Step::AfterReplyMs(2000));
}

fn init(talk: &mut Talk, i: usize) {
    talk.ask(&REQ_INIT[i], REP_INIT[i].to_vec());
}

fn exit_talk(talk: &mut Talk) {
    talk.ask(&REQ_EXIT, REP_EXIT.to_vec());
}

/// Runs `talk` on a four-ESC simulator and checks the replies, invariant 5.2 and the exit.
fn converse(talk: Talk, sim: Sim, config: SessionConfig) -> (Rig, Exit<HostError>) {
    let expected = talk.replies.clone();
    let requests = expected.len();
    let mut rig = rig(talk.script, vec![Some(sim)], config);
    let exit = run(&mut rig, &[]);
    assert_eq!(rig.host.written(), expected);
    assert_answered(&rig, requests);
    (rig, exit)
}

/// Invariant 5.2: a write only after a complete request, one reply per request.
fn assert_answered(rig: &Rig, requests: usize) {
    assert_eq!(rig.host.early_writes(), 0);
    assert_eq!(rig.host.complete_requests(), requests);
    assert_eq!(rig.host.writes(), requests);
}

fn line_ops(rig: &Rig) -> Vec<Wire> {
    rig.world
        .line_events()
        .into_iter()
        .map(|(_, w)| w)
        .filter(|w| !matches!(w, Wire::Break(_)))
        .collect()
}

fn breaks(rig: &Rig) -> usize {
    break_on_positions(&rig.world.line_events()).len()
}

fn bursts(rig: &mut Rig, line: usize) -> Vec<Vec<u8>> {
    let lines = rig.session.release().unwrap();
    lines.sim.written(line).to_vec()
}

fn with_crc_arc(mut frame: Vec<u8>) -> Vec<u8> {
    let crc = ref_crc_arc(&frame);
    frame.push(crc as u8);
    frame.push((crc >> 8) as u8);
    frame
}

const INIT_BURST: [u8; 21] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, b'B', b'L', b'H', b'e', b'l', b'i', 0xF4, 0x7D,
];

// ---------------------------------------------------------------------------------------------
// exits

#[test]
fn run_returns_idle_after_idle_ms_without_a_request() {
    let mut rig = rig(
        vec![Step::Hang],
        vec![Some(four_escs())],
        SessionConfig::new(),
    );
    assert_eq!(run(&mut rig, &[]), Exit::Idle);
    assert_eq!(rig.world.now_ms(), 5_000);
    assert!(!rig.session.holds_lines());
    assert_eq!(rig.lender.asked, 0);
    assert!(rig.world.line_events().is_empty());
}

#[test]
fn idle_time_is_not_restarted_by_bytes_that_are_no_request() {
    // Noise at 0, 3 s and 4.5 s (MAVLink-like bytes that are no MSP request): idle at 5 s sharp.
    let script = vec![
        noise(&[0xFD, 0x09, 0x00]),
        Step::AtMs(3_000),
        noise(&[0x24, 0x4D, 0x3C, 0x00, 0x01, 0x02]),
        Step::AtMs(4_500),
        noise(&[0x55]),
        Step::Hang,
    ];
    let mut rig = rig(script, vec![], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Idle);
    assert_eq!(rig.world.now_ms(), 5_000);
    assert!(rig.host.written().is_empty());
}

#[test]
fn idle_time_restarts_with_each_valid_request() {
    let script = vec![
        Step::AtMs(4_000),
        request(&REQ_API_VERSION),
        Step::AtMs(8_000),
        noise(&[0x00]),
        Step::Hang,
    ];
    let mut rig = rig(script, vec![], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Idle);
    assert_eq!(rig.world.now_ms(), 9_000);
    assert_eq!(rig.host.written(), vec![REP_API_VERSION.to_vec()]);
}

#[test]
#[should_panic(expected = "pending with no timer")]
fn silence_alone_ends_nothing_while_lines_are_held() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.script.push(Step::Hang);
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    let _ = run(&mut rig, &[]);
}

#[test]
fn run_returns_exited_after_interface_exit_with_the_break_done() {
    let mut talk = Talk::new();
    granted(&mut talk);
    exit_talk(&mut talk);
    let (mut rig, exit) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
    assert_eq!(breaks(&rig), 1);
    assert!(rig.session.holds_lines());
    let lines = rig.session.release().unwrap();
    assert!(!lines.sim.break_on());
    assert!(!rig.session.holds_lines());
}

#[test]
fn run_returns_host_error_on_a_read_error_without_lines() {
    let mut rig = rig(
        vec![request(&REQ_API_VERSION), Step::Fail],
        vec![Some(four_escs())],
        SessionConfig::new(),
    );
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Read));
    assert_eq!(breaks(&rig), 0);
    assert_eq!(rig.session.release().err(), Some(ReleaseError::NotHeld));
    assert_eq!(rig.lender.asked, 0);
}

#[test]
fn run_returns_host_error_on_a_read_error_with_lines_held_after_the_lines_exit() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.script.push(Step::Fail);
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Read));
    let events = rig.world.line_events();
    assert_eq!(break_on_positions(&events).len(), 1);
    assert_eq!(events.last().unwrap().1, Wire::Break(false));
    assert!(rig.session.release().is_ok());
}

#[test]
fn run_returns_host_error_on_a_write_error_without_lines() {
    let mut rig = rig(
        vec![request(&REQ_API_VERSION)],
        vec![],
        SessionConfig::new(),
    );
    rig.host.state.borrow_mut().fail_write_at = Some(0);
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Write));
    assert_eq!(breaks(&rig), 0);
    assert!(!rig.session.holds_lines());
}

#[test]
fn run_returns_host_error_on_a_write_error_with_lines_held_after_the_lines_exit() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    rig.host.state.borrow_mut().fail_write_at = Some(1);
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Write));
    // The init was executed, its reply could not be written, then the lines exit.
    let events = rig.world.line_events();
    assert_eq!(break_on_positions(&events).len(), 1);
    assert_eq!(events.last().unwrap().1, Wire::Break(false));
    assert!(rig.session.release().is_ok());
}

#[test]
fn run_returns_host_error_on_a_flush_error() {
    let mut rig = rig(
        vec![request(&REQ_API_VERSION)],
        vec![],
        SessionConfig::new(),
    );
    rig.host.state.borrow_mut().fail_flush_at = Some(0);
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Write));
}

#[test]
fn run_returns_closed_when_the_stream_ends_without_and_with_lines() {
    let mut rig_a = rig(vec![Step::Eof], vec![], SessionConfig::new());
    assert_eq!(run(&mut rig_a, &[]), Exit::Closed);
    assert_eq!(breaks(&rig_a), 0);

    let mut talk = Talk::new();
    granted(&mut talk);
    talk.script.push(Step::Eof);
    let mut rig_b = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    assert_eq!(run(&mut rig_b, &[]), Exit::Closed);
    assert_eq!(breaks(&rig_b), 1);
    assert!(rig_b.session.release().is_ok());

    // A write that takes no byte is the same.
    let mut rig_c = rig(
        vec![request(&REQ_API_VERSION)],
        vec![],
        SessionConfig::new(),
    );
    rig_c.host.state.borrow_mut().zero_write_at = Some(0);
    assert_eq!(run(&mut rig_c, &[]), Exit::Closed);
}

#[test]
fn run_survives_a_stream_that_accepts_a_few_bytes_per_write() {
    let mut talk = Talk::new();
    msp_half(&mut talk);
    exit_talk_after_grant(&mut talk);
    let replies: Vec<u8> = talk.replies.concat();
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    {
        let mut st = rig.host.state.borrow_mut();
        st.max_write = 5;
        st.max_read = 3;
        st.strict = false;
    }
    assert_eq!(run(&mut rig, &[]), Exit::Exited);
    assert_eq!(rig.host.written().concat(), replies);
}

fn exit_talk_after_grant(talk: &mut Talk) {
    exit_talk(talk);
}

#[test]
fn run_refused_by_the_lender_stays_in_msp_and_a_second_attempt_is_granted() {
    let mut talk = Talk::new();
    talk.ask(&REQ_PASSTHROUGH, REP_MSP_ERROR_PASSTHROUGH.to_vec());
    talk.ask(&REQ_API_VERSION, REP_API_VERSION.to_vec());
    granted(&mut talk);
    init(&mut talk, 1);
    exit_talk(&mut talk);
    let expected = talk.replies.clone();
    let mut rig = rig(
        talk.script,
        vec![None, Some(four_escs())],
        SessionConfig::new(),
    );
    assert_eq!(run(&mut rig, &[]), Exit::Exited);
    assert_eq!(rig.host.written(), expected);
    assert_eq!(rig.lender.asked, 2);
    assert_answered(&rig, expected.len());
    // Nothing on the lines before the grant of the second attempt + park.
    let grant_us = rig
        .world
        .host_writes()
        .iter()
        .find(|(_, b)| *b == REP_PASSTHROUGH_4.to_vec())
        .unwrap()
        .0;
    assert!(
        rig.world
            .line_events()
            .iter()
            .all(|(t, _)| *t >= grant_us + 3_000_000)
    );
}

#[test]
fn run_restarted_does_the_lines_exit_keeps_the_lines_and_answers_in_msp() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    // The configurator is closed and reopened without InterfaceExit: it starts over.
    talk.ask(&REQ_API_VERSION, REP_API_VERSION.to_vec());
    granted(&mut talk);
    init(&mut talk, 2);
    exit_talk(&mut talk);
    let (mut rig, exit) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
    // Asked once; the lines were kept through the restart.
    assert_eq!(rig.lender.asked, 1);
    // Two exits: one at the restart, one at InterfaceExit.
    assert_eq!(breaks(&rig), 2);
    let log = rig.world.log();
    let grants: Vec<u64> = log
        .iter()
        .filter(|(_, e)| *e == Ev::HostWrite(REP_PASSTHROUGH_4.to_vec()))
        .map(|(t, _)| *t)
        .collect();
    assert_eq!(grants.len(), 2);
    // The restart's reply (api version) comes after the first break was released.
    let restart_reply = log
        .iter()
        .position(|(_, e)| *e == Ev::HostWrite(REP_API_VERSION.to_vec()))
        .unwrap();
    let first_off = log
        .iter()
        .position(|(_, e)| *e == Ev::Line(Wire::Break(false)))
        .unwrap();
    assert!(first_off < restart_reply);
    // The second grant waits park_ms again before any line traffic.
    let second_grant = log
        .iter()
        .rposition(|(_, e)| *e == Ev::HostWrite(REP_PASSTHROUGH_4.to_vec()))
        .unwrap();
    let next_line = log[second_grant..]
        .iter()
        .find(|(_, e)| matches!(e, Ev::Line(_)))
        .unwrap();
    assert!(next_line.0 >= grants[1] + 3_000_000);
    assert!(rig.session.release().is_ok());
}

#[test]
fn run_restarted_by_a_request_with_a_payload_answers_it_as_msp_does() {
    let mut talk = Talk::new();
    granted(&mut talk);
    // MSP_SET_PASSTHROUGH with a payload: the MSP error reply, after the lines exit.
    let with_payload = msp_request(SET_PASSTHROUGH, &[1]);
    talk.ask(&with_payload, REP_MSP_ERROR_PASSTHROUGH.to_vec());
    talk.ask(&REQ_API_VERSION, REP_API_VERSION.to_vec());
    let expected = talk.replies.clone();
    let mut rig = rig(
        talk.script,
        vec![Some(four_escs())],
        SessionConfig::new().with_idle_ms(1000),
    );
    // Lines are held, so the idle time does not apply; the stream then ends.
    rig.host.push(Step::Eof);
    assert_eq!(run(&mut rig, &[]), Exit::Closed);
    assert_eq!(rig.host.written(), expected);
    assert_eq!(breaks(&rig), 1);
}

#[test]
fn run_takes_the_preload_first() {
    let mut talk = Talk::new();
    granted(&mut talk);
    exit_talk(&mut talk);
    // The caller has already read the trigger request (and the passthrough request after it).
    let mut preload = REQ_API_VERSION.to_vec();
    preload.extend_from_slice(&REQ_PASSTHROUGH);
    let mut rig = rig(
        talk.script[1..].iter().map(clone_step).collect(),
        vec![Some(four_escs())],
        SessionConfig::new(),
    );
    rig.host.state.borrow_mut().strict = false;
    rig.host.state.borrow_mut().complete_requests = 2;
    assert_eq!(run(&mut rig, &preload), Exit::Exited);
    assert_eq!(
        rig.host.written(),
        vec![
            REP_API_VERSION.to_vec(),
            REP_PASSTHROUGH_4.to_vec(),
            REP_EXIT.to_vec()
        ]
    );
}

fn clone_step(step: &Step) -> Step {
    match step {
        Step::Send { bytes, request } => Step::Send {
            bytes: bytes.clone(),
            request: *request,
        },
        Step::AfterReplyMs(ms) => Step::AfterReplyMs(*ms),
        Step::AtMs(ms) => Step::AtMs(*ms),
        Step::Fail => Step::Fail,
        Step::Eof => Step::Eof,
        Step::Hang => Step::Hang,
    }
}

#[test]
fn run_answers_the_battery_request_with_the_value_at_the_moment() {
    let mut talk = Talk::new();
    talk.ask(&REQ_BATTERY, REP_BATTERY_16800.to_vec());
    talk.ask(&REQ_API_VERSION, REP_API_VERSION.to_vec());
    talk.ask(&REQ_BATTERY, REP_BATTERY_12600.to_vec());
    let expected = talk.replies.clone();
    let mut rig = rig(talk.script, vec![], SessionConfig::new());
    let calls = Rc::new(Cell::new(0u32));
    let counter = calls.clone();
    let exit = run_with_battery(&mut rig, &[], move || {
        counter.set(counter.get() + 1);
        BatteryState::new(if counter.get() == 1 { 16_800 } else { 12_600 })
    });
    assert_eq!(exit, Exit::Idle);
    assert_eq!(calls.get(), 2);
    assert_eq!(rig.host.written(), expected);
}

#[test]
fn run_answers_an_unknown_msp_command_with_the_error_reply() {
    let mut talk = Talk::new();
    talk.ask(&msp_request(99, &[]), msp_error(99));
    let expected = talk.replies.clone();
    let mut rig = rig(talk.script, vec![], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Idle);
    assert_eq!(rig.host.written(), expected);
}

// ---------------------------------------------------------------------------------------------
// error paths and the mapping of each command

#[test]
fn run_answers_a_controller_index_out_of_range_with_no_line_traffic() {
    let mut talk = Talk::new();
    granted(&mut talk);
    // esc_count is 2 here: index 2 and 255 are out of range, for init and for reset.
    talk.ask(
        &fw_request(CMD_INIT_FLASH, 0, &[2]),
        fw_reply(CMD_INIT_FLASH, 0, &[2], ACK_INVALID_CHANNEL),
    );
    talk.ask(
        &fw_request(CMD_INIT_FLASH, 0, &[255]),
        fw_reply(CMD_INIT_FLASH, 0, &[255], ACK_INVALID_CHANNEL),
    );
    talk.ask(
        &fw_request(CMD_RESET, 0, &[2]),
        fw_reply(CMD_RESET, 0, &[2], ACK_INVALID_CHANNEL),
    );
    exit_talk(&mut talk);
    let mut expected = talk.replies.clone();
    expected[0] = vec![0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x02, 0xF6];
    let mut rig = rig(
        talk.script,
        vec![Some(four_escs())],
        SessionConfig::new().with_esc_count(2),
    );
    assert_eq!(run(&mut rig, &[]), Exit::Exited);
    assert_eq!(rig.host.written(), expected);
    assert!(line_ops(&rig).is_empty());
}

#[test]
fn run_answers_a_request_before_any_init_with_the_general_error() {
    let mut talk = Talk::new();
    granted(&mut talk);
    let general = |cmd: u8, address: u16, params: &[u8], echo: u8| {
        (
            fw_request(cmd, address, params),
            fw_reply(cmd, address, &[echo], ACK_GENERAL_ERROR),
        )
    };
    for (request_bytes, reply) in [
        general(CMD_READ, 0x7C00, &[184], 184),
        general(CMD_WRITE, 0x7C00, &[1, 2, 3], 0),
        general(CMD_PAGE_ERASE, 0, &[3], 3),
        general(CMD_TEST_ALIVE, 0, &[0], 0),
    ] {
        talk.ask(&request_bytes, reply);
    }
    exit_talk(&mut talk);
    let (rig, exit) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
    assert!(line_ops(&rig).is_empty());
}

#[test]
fn run_answers_a_client_timeout_mid_read_with_the_general_error_and_goes_on() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.ask(
        &REQ_READ_FILE_NAME,
        fw_reply(CMD_READ, 0x7BE0, &[32], ACK_GENERAL_ERROR),
    );
    let sim0 = four_escs();
    let settings = sim0.esc(0).flash()[SETTINGS_AT..SETTINGS_AT + 184].to_vec();
    talk.ask(
        &REQ_READ_SETTINGS,
        fw_reply(CMD_READ, 0x7C00, &settings, ACK_OK),
    );
    exit_talk(&mut talk);
    let mut sim = sim0;
    // init answer, set address answer, then the read answer is lost.
    sim.faults(0, [Fault::Pass, Fault::Pass, Fault::NoAnswer]);
    let (_rig, exit) = converse(talk, sim, SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
}

#[test]
fn run_recovers_a_stuck_payload_through_the_session() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    let data: Vec<u8> = (0..40).collect();
    // The payload frame arrives damaged: the write fails, the ESC keeps waiting for a payload.
    talk.ask(
        &fw_request(CMD_WRITE, 0x2000, &data),
        fw_reply(CMD_WRITE, 0x2000, &[0], ACK_GENERAL_ERROR),
    );
    // The next request finds the line marked: a keep-alive first (taken as the payload), then the
    // read works.
    talk.ask(
        &fw_request(CMD_READ, 0x2000, &[4]),
        fw_reply(CMD_READ, 0x2000, &[0xFF; 4], ACK_OK),
    );
    // And a write after that lands where it was asked.
    talk.ask(
        &fw_request(CMD_WRITE, 0x2000, &data),
        fw_reply(CMD_WRITE, 0x2000, &[0], ACK_OK),
    );
    exit_talk(&mut talk);
    let mut sim = four_escs();
    // Bursts: init 0, set address 1, set buffer 2, payload 3.
    sim.corrupt_burst(0, 3, 0, 0xFF);
    let (mut rig, exit) = converse(talk, sim, SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
    let keep_alive = with_crc_arc(vec![0xFD, 0x00]);
    let seen = bursts(&mut rig, 0);
    assert!(seen.contains(&keep_alive));
    let lines = rig.session.release().err();
    assert_eq!(lines, Some(ReleaseError::NotHeld));
}

#[test]
fn run_writes_a_settings_block_and_reads_it_back() {
    let settings: Vec<u8> = (0..184u32).map(|i| (i * 3 + 1) as u8).collect();
    let mut readback = settings.clone();
    // The bootloader puts its own version in byte 2 of a write at the settings address
    // (BL:650-658), so the read-back differs there and only there.
    readback[2] = common::sim::BOOTLOADER_VERSION;
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.ask(
        &fw_request(CMD_WRITE, 0x7C00, &settings),
        fw_reply(CMD_WRITE, 0x7C00, &[0], ACK_OK),
    );
    talk.ask(
        &REQ_READ_SETTINGS,
        fw_reply(CMD_READ, 0x7C00, &readback, ACK_OK),
    );
    exit_talk(&mut talk);
    let (rig, exit) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(exit, Exit::Exited);
    let lines_ops = line_ops(&rig);
    // The write: set address, set buffer, (the wait for no answer), payload, program.
    let writes: Vec<Vec<u8>> = lines_ops
        .iter()
        .filter_map(|w| match w {
            Wire::Write(b) => Some(b.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(writes[1], with_crc_arc(vec![0xFF, 0x00, 0x7C, 0x00]));
    assert_eq!(writes[2], with_crc_arc(vec![0xFE, 0x00, 0x00, 184]));
    assert_eq!(writes[3], with_crc_arc(settings.clone())[..].to_vec());
    assert_eq!(writes[4], with_crc_arc(vec![0x01, 0x01]));
}

#[test]
fn run_maps_test_alive_to_a_keep_alive_on_the_connected_controller() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.ask(
        &fw_request(CMD_TEST_ALIVE, 0, &[0]),
        fw_reply(CMD_TEST_ALIVE, 0, &[0], ACK_GENERAL_ERROR),
    );
    init(&mut talk, 1);
    talk.ask(
        &fw_request(CMD_TEST_ALIVE, 0, &[0]),
        fw_reply(CMD_TEST_ALIVE, 0, &[0], ACK_OK),
    );
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    let seen = bursts(&mut rig, 1);
    assert_eq!(
        seen,
        vec![INIT_BURST.to_vec(), with_crc_arc(vec![0xFD, 0x00])]
    );
}

#[test]
fn run_disconnects_after_a_failed_test_alive() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.ask(
        &fw_request(CMD_TEST_ALIVE, 0, &[0]),
        fw_reply(CMD_TEST_ALIVE, 0, &[0], ACK_GENERAL_ERROR),
    );
    talk.ask(
        &REQ_READ_FILE_NAME,
        fw_reply(CMD_READ, 0x7BE0, &[32], ACK_GENERAL_ERROR),
    );
    exit_talk(&mut talk);
    let mut sim = four_escs();
    sim.faults(0, [Fault::Pass, Fault::NoAnswer]);
    let (mut rig, _) = converse(talk, sim, SessionConfig::new());
    // No read command reached the ESC after the failed keep-alive.
    assert_eq!(bursts(&mut rig, 0).len(), 2);
}

#[test]
fn run_maps_device_reset_to_the_run_command_and_disconnects() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 1);
    talk.ask(
        &fw_request(CMD_RESET, 0, &[1]),
        fw_reply(CMD_RESET, 0, &[1], ACK_OK),
    );
    talk.ask(
        &REQ_READ_FILE_NAME,
        fw_reply(CMD_READ, 0x7BE0, &[32], ACK_GENERAL_ERROR),
    );
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    let seen = bursts(&mut rig, 1);
    assert_eq!(seen, vec![INIT_BURST.to_vec(), vec![0, 0, 0, 0]]);
}

#[test]
fn run_maps_a_refused_line_selection_to_the_general_error() {
    // Controller 0 is mapped to line 9, which the simulator does not have: select fails, and the
    // reset and the init both get the general error with the controller index.
    let config = SessionConfig::new().with_line_map([9, 1, 2, 3, 4, 5, 6, 7]);
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.ask(
        &fw_request(CMD_RESET, 0, &[0]),
        fw_reply(CMD_RESET, 0, &[0], ACK_GENERAL_ERROR),
    );
    talk.ask(
        &REQ_INIT[0],
        fw_reply(CMD_INIT_FLASH, 0, &[0], ACK_GENERAL_ERROR),
    );
    exit_talk(&mut talk);
    let (_rig, exit) = converse(talk, four_escs(), config);
    assert_eq!(exit, Exit::Exited);
}

#[test]
fn run_maps_page_erase_to_set_address_and_erase_at_page_times_1024() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.ask(
        &fw_request(CMD_PAGE_ERASE, 0x0123, &[5]),
        fw_reply(CMD_PAGE_ERASE, 0x0123, &[5], ACK_OK),
    );
    // Page 0 is address 0, below the application: the bootloader refuses.
    talk.ask(
        &fw_request(CMD_PAGE_ERASE, 0, &[0]),
        fw_reply(CMD_PAGE_ERASE, 0, &[0], ACK_GENERAL_ERROR),
    );
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    let seen = bursts(&mut rig, 0);
    assert_eq!(seen[1], with_crc_arc(vec![0xFF, 0x00, 0x14, 0x00]));
    assert_eq!(seen[2], with_crc_arc(vec![0x02, 0x01]));
    assert_eq!(seen[3], with_crc_arc(vec![0xFF, 0x00, 0x00, 0x00]));
}

#[test]
fn run_answers_a_write_of_one_byte_with_the_invalid_parameter_acknowledge() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    talk.ask(
        &fw_request(CMD_WRITE, 0x2000, &[7]),
        fw_reply(CMD_WRITE, 0x2000, &[0], ACK_INVALID_PARAM),
    );
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(bursts(&mut rig, 0), vec![INIT_BURST.to_vec()]);
}

#[test]
fn run_reads_256_bytes_for_a_count_of_zero() {
    let sim = four_escs();
    let flash = sim.esc(0).flash()[0x2000..0x2100].to_vec();
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    // The parameter is 0: 256 bytes (the reply's count byte is 0 as well).
    talk.ask(
        &fw_request(CMD_READ, 0x2000, &[0]),
        fw_reply(CMD_READ, 0x2000, &flash, ACK_OK),
    );
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, sim, SessionConfig::new());
    assert_eq!(bursts(&mut rig, 0)[2], with_crc_arc(vec![0x03, 0x00]));
}

#[test]
fn run_answers_housekeeping_without_line_traffic() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.ask(
        &fw_request(CMD_PROTOCOL_VERSION, 0, &[0]),
        REP_PROTOCOL_VERSION.to_vec(),
    );
    talk.ask(
        &fw_request(CMD_INTERFACE_NAME, 0x1234, &[0]),
        fw_reply(
            CMD_INTERFACE_NAME,
            0x1234,
            &[4, b'A', b'R', b'D', b'U'],
            ACK_OK,
        ),
    );
    talk.ask(
        &fw_request(CMD_INTERFACE_VERSION, 0, &[0]),
        fw_reply(CMD_INTERFACE_VERSION, 0, &[200, 5], ACK_OK),
    );
    exit_talk(&mut talk);
    let (rig, _) = converse(talk, four_escs(), SessionConfig::new());
    assert!(line_ops(&rig).is_empty());
}

#[test]
fn run_answers_the_other_commands_as_ardupilot_does_for_an_arm_bootloader() {
    let mut talk = Talk::new();
    granted(&mut talk);
    init(&mut talk, 0);
    for (cmd, params, param_back, ack) in [
        (CMD_VERIFY, vec![1u8, 2], 0u8, ACK_INVALID_CMD),
        (CMD_READ_EEPROM, vec![4], 0, ACK_INVALID_CMD),
        (CMD_WRITE_EEPROM, vec![1, 2], 0, ACK_GENERAL_ERROR),
        (CMD_ERASE_ALL, vec![0], 0, ACK_INVALID_CMD),
        (CMD_C2CK_LOW, vec![0], 0, ACK_INVALID_CMD),
        (0x36, vec![0], 0, ACK_INVALID_CMD),
        (CMD_SET_MODE, vec![4], 4, ACK_OK),
    ] {
        talk.ask(
            &fw_request(cmd, 0x0042, &params),
            fw_reply(cmd, 0x0042, &[param_back], ack),
        );
    }
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    assert_eq!(bursts(&mut rig, 0), vec![INIT_BURST.to_vec()]);
}

#[test]
fn run_answers_a_four_way_request_with_a_bad_crc_without_line_traffic() {
    let mut talk = Talk::new();
    granted(&mut talk);
    let mut bad = fw_request(CMD_INIT_FLASH, 0, &[0]);
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    talk.ask(&bad, fw_reply(CMD_INIT_FLASH, 0, &[0], 0x03));
    exit_talk(&mut talk);
    let (rig, _) = converse(talk, four_escs(), SessionConfig::new());
    assert!(line_ops(&rig).is_empty());
}

#[test]
fn run_maps_the_controller_index_to_the_line_through_the_line_map() {
    let mut talk = Talk::new();
    granted(&mut talk);
    // Controller 0 is on line 3, controller 1 on line 2 (pin codes 0x13 and 0x12).
    talk.ask(
        &REQ_INIT[0],
        fw_reply(CMD_INIT_FLASH, 0, &[0x06, 0x1F, 0x13, 0x04], ACK_OK),
    );
    talk.ask(
        &REQ_INIT[1],
        fw_reply(CMD_INIT_FLASH, 0, &[0x06, 0x1F, 0x12, 0x04], ACK_OK),
    );
    exit_talk(&mut talk);
    let config = SessionConfig::new().with_line_map([3, 2, 1, 0, 4, 5, 6, 7]);
    let (rig, _) = converse(talk, four_escs(), config);
    let selects: Vec<Wire> = line_ops(&rig)
        .into_iter()
        .filter(|w| matches!(w, Wire::Select(_)))
        .collect();
    assert_eq!(selects, vec![Wire::Select(3), Wire::Select(2)]);
}

#[test]
fn run_uses_the_configured_esc_count_info_and_timeouts() {
    let config = SessionConfig::new()
        .with_esc_count(2)
        .with_info(HostInfo::new().with_protocol_version(9).with_motor_count(6))
        .with_timeouts(Timeouts::new().with_ack_us(1234));
    let mut talk = Talk::new();
    talk.ask(
        &REQ_MOTOR_CONFIG,
        msp_reply(MOTOR_CONFIG, &[0, 0, 0, 0, 0, 0, 6, 0, 0, 0]),
    );
    talk.ask(&REQ_PASSTHROUGH, msp_reply(SET_PASSTHROUGH, &[2]));
    talk.script.push(Step::AfterReplyMs(2000));
    talk.ask(
        &fw_request(CMD_PROTOCOL_VERSION, 0, &[0]),
        fw_reply(CMD_PROTOCOL_VERSION, 0, &[9], ACK_OK),
    );
    talk.ask(
        &fw_request(CMD_READ, 0x0100, &[4]),
        fw_reply(CMD_READ, 0x0100, &[4], ACK_GENERAL_ERROR),
    );
    init(&mut talk, 0);
    talk.ask(
        &fw_request(CMD_READ, 0x1000, &[4]),
        fw_reply(CMD_READ, 0x1000, &[0, 0x10, 0, 0x20], ACK_OK),
    );
    exit_talk(&mut talk);
    let (rig, _) = converse(talk, four_escs(), config);
    let ack_waits: Vec<u32> = line_ops(&rig)
        .iter()
        .filter_map(|w| match w {
            Wire::Read { timeout_us, .. } if *timeout_us == 1234 => Some(*timeout_us),
            _ => None,
        })
        .collect();
    assert!(!ack_waits.is_empty());
}

// ---------------------------------------------------------------------------------------------
// finish, release, holds_lines, config

#[test]
fn release_is_refused_before_the_lines_are_held_and_while_the_exit_is_owed() {
    let mut rig = rig(vec![], vec![], SessionConfig::new());
    assert_eq!(rig.session.release().err(), Some(ReleaseError::NotHeld));
    assert!(!rig.session.holds_lines());
}

#[test]
fn finish_does_the_exit_once_and_is_idempotent() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.script.push(Step::Fail);
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Host(HostError::Read));
    let before = rig.world.line_events().len();
    finish(&mut rig);
    finish(&mut rig);
    assert_eq!(rig.world.line_events().len(), before);
    assert!(rig.session.holds_lines());
    assert!(rig.session.release().is_ok());
    finish(&mut rig);
}

#[test]
fn finish_after_a_cancel_does_the_owed_exit_and_release_then_succeeds() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.script.push(Step::Eof);
    let mut rig = rig(talk.script, vec![Some(four_escs())], SessionConfig::new());
    rig.world.reset_points();
    let mut rx = rig.host.rx();
    let mut tx = rig.host.tx();
    // Awaits: read, lend, write, flush, then the park delay: drop there.
    rig.world.trip_at(Some(4));
    let done = rig.world.run_until_trip(rig.session.run(
        &[],
        &mut rx,
        &mut tx,
        &mut rig.lender,
        &mut rig.delay,
        || BatteryState::new(0),
    ));
    assert!(done.is_none());
    assert!(rig.session.holds_lines());
    assert_eq!(rig.session.release().err(), Some(ReleaseError::ExitOwed));
    finish(&mut rig);
    assert_eq!(breaks(&rig), 1);
    assert!(rig.session.release().is_ok());
}

#[test]
fn finish_keeps_the_exit_owed_and_retries_when_the_line_refuses_the_break() {
    let mut talk = Talk::new();
    granted(&mut talk);
    talk.script.push(Step::Fail);
    let world = World::new();
    let host = Host::new(&world, talk.script);
    let mut lines = TracedLines::new(&world, four_escs());
    lines.fail_breaks = 1;
    let mut lender = TestLender::new(&world, vec![Some(lines)]);
    let mut delay = VDelay(world.clone());
    let mut session = Passthrough::new(SessionConfig::new());
    let mut rx = host.rx();
    let mut tx = host.tx();
    let exit = world.run(
        session.run(&[], &mut rx, &mut tx, &mut lender, &mut delay, || {
            BatteryState::new(0)
        }),
    );
    assert_eq!(exit, Exit::Host(HostError::Read));
    // The first set_break(true) failed: the exit is still owed and release refuses.
    assert_eq!(session.release().err(), Some(ReleaseError::ExitOwed));
    world.run(session.finish(&mut delay));
    assert!(session.release().is_ok());
    let events = world.line_events();
    assert_eq!(
        events
            .iter()
            .filter(|(_, w)| *w == Wire::Break(true))
            .count(),
        2
    );
    assert_eq!(events.last().unwrap().1, Wire::Break(false));
}

#[test]
fn new_config_has_the_defaults() {
    let config = SessionConfig::new();
    assert_eq!(config.esc_count, 4);
    assert_eq!(config.line_map, [0, 1, 2, 3, 4, 5, 6, 7]);
    assert_eq!(config.park_ms, 3_000);
    assert_eq!(config.break_ms, 25);
    assert_eq!(config.idle_ms, 5_000);
    assert_eq!(config.timeouts, Timeouts::new());
    assert_eq!(config.info.motor_count, 4);
    assert_eq!(config, SessionConfig::default());
}

#[test]
fn new_holds_nothing_and_asks_for_nothing() {
    let session = Passthrough::<TracedLines>::new(SessionConfig::new());
    assert!(!session.holds_lines());
}

#[test]
fn holds_lines_is_true_from_the_grant_until_release() {
    let mut talk = Talk::new();
    granted(&mut talk);
    exit_talk(&mut talk);
    let (mut rig, _) = converse(talk, four_escs(), SessionConfig::new());
    assert!(rig.session.holds_lines());
    assert!(rig.session.release().is_ok());
    assert!(!rig.session.holds_lines());
    assert_eq!(rig.session.release().err(), Some(ReleaseError::NotHeld));
}

#[test]
fn lend_is_asked_once_for_the_grant_and_a_refusal_is_the_msp_error() {
    let mut talk = Talk::new();
    talk.ask(&REQ_PASSTHROUGH, REP_MSP_ERROR_PASSTHROUGH.to_vec());
    talk.ask(&REQ_PASSTHROUGH, REP_MSP_ERROR_PASSTHROUGH.to_vec());
    let expected = talk.replies.clone();
    let mut rig = rig(talk.script, vec![None, None], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Idle);
    assert_eq!(rig.lender.asked, 2);
    assert_eq!(rig.host.written(), expected);
    assert!(rig.world.line_events().is_empty());
}

#[test]
fn with_esc_count_limits_to_1_to_8_and_sets_the_motor_count() {
    assert_eq!(SessionConfig::new().with_esc_count(0).esc_count, 1);
    assert_eq!(SessionConfig::new().with_esc_count(8).esc_count, 8);
    assert_eq!(SessionConfig::new().with_esc_count(200).esc_count, 8);
    assert_eq!(SessionConfig::new().with_esc_count(3).info.motor_count, 3);
}

#[test]
fn with_line_map_sets_the_map() {
    let config = SessionConfig::new().with_line_map([7, 6, 5, 4, 3, 2, 1, 0]);
    assert_eq!(config.line_map, [7, 6, 5, 4, 3, 2, 1, 0]);
}

#[test]
fn with_park_ms_sets_the_park_time() {
    assert_eq!(SessionConfig::new().with_park_ms(1).park_ms, 1);
}

#[test]
fn with_break_ms_sets_the_break_time() {
    assert_eq!(SessionConfig::new().with_break_ms(2).break_ms, 2);
}

#[test]
fn with_idle_ms_sets_the_idle_time() {
    assert_eq!(SessionConfig::new().with_idle_ms(3).idle_ms, 3);
}

#[test]
fn with_timeouts_sets_the_client_timeouts() {
    let timeouts = Timeouts::new().with_erase_us(5);
    assert_eq!(
        SessionConfig::new().with_timeouts(timeouts).timeouts,
        timeouts
    );
}

#[test]
fn with_info_sets_the_host_info() {
    let info = HostInfo::new().with_motor_count(1);
    assert_eq!(SessionConfig::new().with_info(info).info, info);
}
