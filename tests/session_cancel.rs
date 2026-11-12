//! Cancellation: `Passthrough::run` dropped at every await of the configurator's conversation.
//!
//! The doubles count their awaits (host read, host write and flush, delay, lend, line write and
//! read). A dry run counts them; then for each index `k` a fresh run is dropped at the `k`th, and
//! `finish()` and `release()` must leave the lines released after exactly one exit break, and a
//! fresh session on the returned lines must work.
#![allow(dead_code)]

mod common;
#[path = "common/session_support.rs"]
mod support;
#[path = "common/session_talk.rs"]
mod talk;

use common::sim::Wire;
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Passthrough, ReleaseError, SessionConfig};
use support::*;
use talk::*;

/// Runs the whole conversation once and returns the number of awaits it has.
fn count_points() -> usize {
    let sim = four_escs();
    let talk = full_talk(&sim);
    let mut rig = rig(talk.script, vec![Some(sim)], SessionConfig::new());
    rig.world.reset_points();
    assert_eq!(run(&mut rig, &[]), Exit::Exited);
    rig.world.points()
}

/// The short conversation a fresh session must be able to hold on the returned lines.
fn fresh_session_works(world: &std::rc::Rc<World>, lines: TracedLines) {
    world.clear_log();
    // The simulator keeps its own clock, which the world's delays do not move: let the answers a
    // dropped exchange left in flight arrive (the real lines exit and the park take seconds).
    let mut lines = lines;
    lines.sim.advance(1_000_000);
    let mut talk = Talk::new();
    talk.ask(&REQ_PASSTHROUGH, REP_PASSTHROUGH_4.to_vec());
    talk.script.push(Step::AfterReplyMs(2000));
    talk.ask(&REQ_INIT[2], REP_INIT[2].to_vec());
    let flash = lines.sim.esc(2).flash();
    talk.ask(
        &REQ_READ_FILE_NAME,
        fw_reply(
            CMD_READ,
            0x7BE0,
            &flash[FILE_NAME_AT..FILE_NAME_AT + 32],
            ACK_OK,
        ),
    );
    talk.ask(&REQ_EXIT, REP_EXIT.to_vec());
    let host = Host::new(world, talk.script);
    let mut lender = TestLender::new(world, vec![Some(lines)]);
    let mut delay = VDelay(world.clone());
    let mut session = Passthrough::new(SessionConfig::new());
    let mut rx = host.rx();
    let mut tx = host.tx();
    let exit = world.run(
        session.run(&[], &mut rx, &mut tx, &mut lender, &mut delay, || {
            BatteryState::new(0)
        }),
    );
    assert_eq!(exit, Exit::Exited);
    assert_eq!(host.written(), talk.replies);
    assert!(session.release().is_ok());
}

#[test]
fn run_dropped_at_every_await_leaves_one_exit_break_and_usable_lines() {
    let points = count_points();
    assert!(points > 200, "the conversation has {points} awaits");
    println!("awaits in the conversation: {points}");
    let mut lent_cases = 0;
    let mut early_cases = 0;
    for k in 0..points {
        let sim = four_escs();
        let talk = full_talk(&sim);
        let mut rig = rig(talk.script, vec![Some(sim)], SessionConfig::new());
        rig.world.reset_points();
        rig.world.trip_at(Some(k));
        let mut rx = rig.host.rx();
        let mut tx = rig.host.tx();
        let done = rig.world.run_until_trip(rig.session.run(
            &[],
            &mut rx,
            &mut tx,
            &mut rig.lender,
            &mut rig.delay,
            || BatteryState::new(16_800),
        ));
        assert!(done.is_none(), "k = {k} did not trip");
        // Invariant 5.2 holds at every cut.
        assert_eq!(rig.host.early_writes(), 0, "k = {k}");

        let lent = rig.lender.offers.is_empty();
        if lent {
            lent_cases += 1;
            assert!(rig.session.holds_lines(), "k = {k}");
            assert_eq!(
                rig.session.release().err(),
                Some(ReleaseError::ExitOwed),
                "k = {k}"
            );
        } else {
            early_cases += 1;
            assert!(!rig.session.holds_lines(), "k = {k}");
            assert_eq!(rig.session.release().err(), Some(ReleaseError::NotHeld));
        }

        finish(&mut rig);
        finish(&mut rig);

        let events = rig.world.line_events();
        if !lent {
            assert!(events.is_empty(), "k = {k}");
            continue;
        }
        let ons = break_on_positions(&events);
        assert_eq!(ons.len(), 1, "k = {k}");
        // After the last line traffic: nothing but the release follows the break.
        assert_eq!(events.len(), ons[0] + 2, "k = {k}");
        assert_eq!(events[ons[0] + 1].1, Wire::Break(false), "k = {k}");
        assert!(events[ons[0] + 1].0 - events[ons[0]].0 >= 25_000, "k = {k}");
        assert!(
            events[..ons[0]]
                .iter()
                .all(|(_, w)| !matches!(w, Wire::Break(_)))
        );

        let lines = match rig.session.release() {
            Ok(lines) => lines,
            Err(error) => panic!("k = {k}: release gave {error:?}"),
        };
        assert!(!lines.sim.break_on(), "k = {k}");
        assert!(!rig.session.holds_lines());
        fresh_session_works(&rig.world, lines);
    }
    assert!(
        lent_cases > 100 && early_cases > 5,
        "{lent_cases} {early_cases}"
    );
}

#[test]
fn run_called_again_after_a_cancel_in_the_break_completes_the_exit_first() {
    let points = count_points();
    let sim = four_escs();
    let talk = full_talk(&sim);
    let mut rig = rig(talk.script, vec![Some(sim)], SessionConfig::new());
    rig.world.reset_points();
    // The last await is the delay of the exit break.
    rig.world.trip_at(Some(points - 1));
    let mut rx = rig.host.rx();
    let mut tx = rig.host.tx();
    let done = rig.world.run_until_trip(rig.session.run(
        &[],
        &mut rx,
        &mut tx,
        &mut rig.lender,
        &mut rig.delay,
        || BatteryState::new(0),
    ));
    assert!(done.is_none());
    assert!(rig.world.line_events().last().unwrap().1 == Wire::Break(true));

    // No finish(): the next run does the owed exit before it reads.
    rig.host.push(request(&REQ_API_VERSION));
    rig.host.push(Step::Eof);
    let exit = run(&mut rig, &[]);
    assert_eq!(exit, Exit::Closed);
    let events = rig.world.line_events();
    assert_eq!(break_on_positions(&events).len(), 1);
    assert_eq!(events.last().unwrap().1, Wire::Break(false));
    assert_eq!(
        rig.host.written().last().unwrap(),
        &REP_API_VERSION.to_vec()
    );
    assert!(rig.session.release().is_ok());
}
