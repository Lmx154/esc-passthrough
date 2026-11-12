//! The shared parts of the session tests: the rig, the configurator's conversation as literal
//! vectors (see `tests/session.rs` for where each comes from), and the helpers that run it.
#![allow(dead_code)]

use crate::common::ref_crc_arc;
use crate::common::sim::Sim;
use crate::support::*;
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Passthrough, SessionConfig};
use std::rc::Rc;

pub struct Rig {
    pub world: Rc<World>,
    pub host: Host,
    pub session: Passthrough<TracedLines>,
    pub lender: TestLender,
    pub delay: VDelay,
}

pub fn rig(script: Vec<Step>, offers: Vec<Option<Sim>>, config: SessionConfig) -> Rig {
    let world = World::new();
    let host = Host::new(&world, script);
    let offers = offers
        .into_iter()
        .map(|sim| sim.map(|sim| TracedLines::new(&world, sim)))
        .collect();
    Rig {
        lender: TestLender::new(&world, offers),
        delay: VDelay(world.clone()),
        session: Passthrough::new(config),
        host,
        world,
    }
}

pub fn run(rig: &mut Rig, preload: &[u8]) -> Exit<HostError> {
    run_with_battery(rig, preload, || BatteryState::new(16_800))
}

pub fn run_with_battery(
    rig: &mut Rig,
    preload: &[u8],
    battery: impl FnMut() -> BatteryState,
) -> Exit<HostError> {
    let mut rx = rig.host.rx();
    let mut tx = rig.host.tx();
    rig.world.run(rig.session.run(
        preload,
        &mut rx,
        &mut tx,
        &mut rig.lender,
        &mut rig.delay,
        battery,
    ))
}

pub fn finish(rig: &mut Rig) {
    rig.world.run(rig.session.finish(&mut rig.delay));
}

/// The requests and replies of the configurator's conversation.
pub struct Talk {
    pub script: Vec<Step>,
    pub replies: Vec<Vec<u8>>,
}

impl Talk {
    pub fn new() -> Self {
        Self {
            script: Vec::new(),
            replies: Vec::new(),
        }
    }

    /// The configurator sends `request` and expects `reply`.
    pub fn ask(&mut self, request_bytes: &[u8], reply: Vec<u8>) {
        self.script.push(request(request_bytes));
        self.replies.push(reply);
    }
}

// Literal vectors. MSP requests of the configurator (SerialDevice.vue:618-635 sends the first
// four with `Msp.sendWithPromise`, commands.queue.ts:57-119; the fifth is :655): `$M<`, length 0,
// command, checksum = command.
pub const REQ_API_VERSION: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
pub const REQ_FC_VARIANT: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x02, 0x02];
pub const REQ_BATTERY: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x82, 0x82];
pub const REQ_MOTOR_CONFIG: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x83, 0x83];
pub const REQ_PASSTHROUGH: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5];

pub const REP_API_VERSION: [u8; 9] = [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2A, 0x29];
pub const REP_FC_VARIANT: [u8; 10] = [0x24, 0x4D, 0x3E, 0x04, 0x02, 0x41, 0x52, 0x44, 0x55, 0x04];
pub const REP_BATTERY_16800: [u8; 17] = [
    0x24, 0x4D, 0x3E, 0x0B, 0x82, 0x00, 0x00, 0x00, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x06,
    0xB7,
];
pub const REP_BATTERY_12600: [u8; 17] = [
    0x24, 0x4D, 0x3E, 0x0B, 0x82, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEC, 0x04,
    0x1F,
];
pub const REP_MOTOR_CONFIG_4: [u8; 16] = [
    0x24, 0x4D, 0x3E, 0x0A, 0x83, 0, 0, 0, 0, 0, 0, 0x04, 0, 0, 0, 0x8D,
];
pub const REP_PASSTHROUGH_4: [u8; 7] = [0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0];
pub const REP_MSP_ERROR_PASSTHROUGH: [u8; 6] = [0x24, 0x4D, 0x21, 0x00, 0xF5, 0xF5];

// Four-way requests of four_way.ts: InitFlash with the controller index in the parameter and
// address 0 (:200-203), the three reads of getInfo (:257, :280, :299), InterfaceExit with the
// default parameter [0] (:442 `params = [0]`, SerialDevice.vue:816).
pub const REQ_INIT: [[u8; 8]; 4] = [
    [0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00],
    [0x2F, 0x37, 0x00, 0x00, 0x01, 0x01, 0xB8, 0x21],
    [0x2F, 0x37, 0x00, 0x00, 0x01, 0x02, 0x88, 0x42],
    [0x2F, 0x37, 0x00, 0x00, 0x01, 0x03, 0x98, 0x63],
];
pub const REP_INIT: [[u8; 12]; 4] = [
    [
        0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x10, 0x04, 0x00, 0xCE, 0x9A,
    ],
    [
        0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x11, 0x04, 0x00, 0xF9, 0xAA,
    ],
    [
        0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x12, 0x04, 0x00, 0xA0, 0xFA,
    ],
    [
        0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x13, 0x04, 0x00, 0x97, 0xCA,
    ],
];
pub const REQ_READ_DEVINFO: [u8; 8] = [0x2F, 0x3A, 0x00, 0x23, 0x01, 0x40, 0x1E, 0x28];
pub const REQ_READ_FILE_NAME: [u8; 8] = [0x2F, 0x3A, 0x7B, 0xE0, 0x01, 0x20, 0x51, 0x43];
pub const REQ_READ_SETTINGS: [u8; 8] = [0x2F, 0x3A, 0x7C, 0x00, 0x01, 0xB8, 0xA2, 0xEE];
pub const REQ_EXIT: [u8; 8] = [0x2F, 0x34, 0x00, 0x00, 0x01, 0x00, 0x46, 0xD2];
pub const REP_EXIT: [u8; 9] = [0x2E, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x42, 0x63];
pub const REP_PROTOCOL_VERSION: [u8; 9] = [0x2E, 0x31, 0x00, 0x00, 0x01, 0x6B, 0x00, 0xD6, 0xB2];

/// The MSP half of the conversation, up to and including the passthrough reply, and the wait.
pub fn msp_half(talk: &mut Talk) {
    talk.ask(&REQ_API_VERSION, REP_API_VERSION.to_vec());
    talk.ask(&REQ_FC_VARIANT, REP_FC_VARIANT.to_vec());
    talk.ask(&REQ_BATTERY, REP_BATTERY_16800.to_vec());
    talk.ask(&REQ_MOTOR_CONFIG, REP_MOTOR_CONFIG_4.to_vec());
    talk.ask(&REQ_PASSTHROUGH, REP_PASSTHROUGH_4.to_vec());
    // SerialDevice.vue:657 and :701: `await delay(2000)` after the reply.
    talk.script.push(Step::AfterReplyMs(2000));
}

/// What the configurator reads from controller `i` after connecting: the reads of getInfo.
pub fn controller_half(talk: &mut Talk, i: usize, sim: &Sim) {
    talk.ask(&REQ_INIT[i], REP_INIT[i].to_vec());
    let flash = sim.esc(i).flash();
    talk.ask(
        &REQ_READ_DEVINFO,
        fw_reply(
            CMD_READ,
            0x0023,
            &flash[DEVINFO_AT..DEVINFO_AT + 64],
            ACK_OK,
        ),
    );
    talk.ask(
        &REQ_READ_FILE_NAME,
        fw_reply(
            CMD_READ,
            0x7BE0,
            &flash[FILE_NAME_AT..FILE_NAME_AT + 32],
            ACK_OK,
        ),
    );
    talk.ask(
        &REQ_READ_SETTINGS,
        fw_reply(
            CMD_READ,
            0x7C00,
            &flash[SETTINGS_AT..SETTINGS_AT + 184],
            ACK_OK,
        ),
    );
}

pub fn full_talk(sim: &Sim) -> Talk {
    let mut talk = Talk::new();
    msp_half(&mut talk);
    for i in 0..4 {
        controller_half(&mut talk, i, sim);
    }
    talk.ask(&REQ_EXIT, REP_EXIT.to_vec());
    talk
}

/// What the bootloader of one controller must have received, byte for byte: the init, then for each
/// of the three reads set address (`FF 00 AH AL`, CRC low first) and read (`03 n`, CRC low first).
pub fn expected_bursts() -> Vec<Vec<u8>> {
    let with_crc = |mut frame: Vec<u8>| {
        let crc = ref_crc_arc(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        frame
    };
    let mut bursts = vec![
        [
            vec![0u8; 12],
            vec![0x0D],
            b"BLHeli".to_vec(),
            vec![0xF4, 0x7D],
        ]
        .concat(),
    ];
    for (address, count) in [(0x0023u16, 64u8), (0x7BE0, 32), (0x7C00, 184)] {
        bursts.push(with_crc(vec![
            0xFF,
            0x00,
            (address >> 8) as u8,
            address as u8,
        ]));
        bursts.push(with_crc(vec![0x03, count]));
    }
    bursts
}
