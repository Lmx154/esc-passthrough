//! Invariant 3.7: the session never looks inside payloads. For random payloads, addresses and
//! flash contents the line operations (order, kinds, lengths, timeouts) are the same, and the bytes
//! the host receives are exactly the simulator's bytes.
#![allow(dead_code)]

mod common;
#[path = "common/session_support.rs"]
mod support;
#[path = "common/session_talk.rs"]
mod talk;

use common::sim::Wire;
use esc_passthrough::session::{Exit, SessionConfig};
use proptest::prelude::*;
use support::*;
use talk::*;

struct Case {
    write_len: usize,
    write_address: u16,
    read_address: u16,
    read_count: usize,
}

/// One conversation: connect, read, write, read back, exit. Returns the shape of the line calls
/// and checks the host's replies against the simulator.
fn conversation(case: &Case, payload: &[u8], fill: u8) -> Vec<Wire> {
    let mut sim = four_escs();
    for (i, byte) in sim.esc_mut(0).flash_mut()[0x1100..0x7000]
        .iter_mut()
        .enumerate()
    {
        *byte = (i as u8).wrapping_mul(31) ^ fill;
    }
    let read_from = usize::from(case.read_address);
    let initial = sim.esc(0).flash()[read_from..read_from + case.read_count].to_vec();
    let count_param = |n: usize| if n == 256 { 0 } else { n as u8 };

    let mut talk = Talk::new();
    talk.ask(&REQ_PASSTHROUGH, REP_PASSTHROUGH_4.to_vec());
    talk.script.push(Step::AfterReplyMs(2000));
    talk.ask(&REQ_INIT[0], REP_INIT[0].to_vec());
    talk.ask(
        &fw_request(CMD_READ, case.read_address, &[count_param(case.read_count)]),
        fw_reply(CMD_READ, case.read_address, &initial, ACK_OK),
    );
    let data = &payload[..case.write_len];
    talk.ask(
        &fw_request(CMD_WRITE, case.write_address, data),
        fw_reply(CMD_WRITE, case.write_address, &[0], ACK_OK),
    );
    talk.ask(
        &fw_request(CMD_READ, case.write_address, &[count_param(case.write_len)]),
        fw_reply(CMD_READ, case.write_address, data, ACK_OK),
    );
    talk.ask(&REQ_EXIT, REP_EXIT.to_vec());

    let mut rig = rig(talk.script, vec![Some(sim)], SessionConfig::new());
    assert_eq!(run(&mut rig, &[]), Exit::Exited);
    assert_eq!(rig.host.written(), talk.replies);
    let lines = rig.session.release().unwrap();
    let at = usize::from(case.write_address);
    assert_eq!(&lines.sim.esc(0).flash()[at..at + case.write_len], data);
    rig.world
        .line_events()
        .iter()
        .map(|(_, wire)| wire.shape())
        .collect()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn line_operations_do_not_depend_on_the_payload(
        write_len in 2usize..=256,
        write_address in 0x1100u16..0x6000,
        read_address in 0x1100u16..0x7000,
        read_count in 1usize..=256,
        payload_a in proptest::collection::vec(any::<u8>(), 256),
        payload_b in proptest::collection::vec(any::<u8>(), 256),
        fill_a in any::<u8>(),
        fill_b in any::<u8>(),
    ) {
        let case = Case { write_len, write_address, read_address, read_count };
        let a = conversation(&case, &payload_a, fill_a);
        let b = conversation(&case, &payload_b, fill_b);
        prop_assert_eq!(a, b);
    }
}
