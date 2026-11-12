//! Host tests for `esc_passthrough::host`.
//!
//! Sources (CFG = `_plan/ref/am32-configurator`, SIM = `CFG/ci/bootloader_sim.py`, AP =
//! `ardupilot/libraries/AP_BLHeli/AP_BLHeli.cpp`, HDR = `.../blheli_4way_protocol.h`, MSPH =
//! `ardupilot/libraries/AP_MSP/msp_protocol.h`):
//!
//! - The connect conversation, as the configurator runs it (`CFG/components/SerialDevice.vue:
//!   618-663`): `MSP_API_VERSION`, `MSP_FC_VARIANT`, `MSP_BATTERY_STATE`, `MSP_MOTOR_CONFIG` (for an
//!   ArduPilot-type controller; `getTypeMotorCommand` picks 104 for inav and emuf only), then
//!   `MSP_SET_PASSTHROUGH` and a wait of 2000 ms, then per ESC `InitFlash` (`CFG/src/communication/
//!   four_way.ts:205-210`: parameter `[target]`, address 0), `DeviceRead` at the wire address with
//!   `[n]` (`four_way.ts:326-337`), and at the end `InterfaceExit` (`four_way.ts:386-389`, the
//!   configurator does not wait for its reply). Every MSP request has an empty payload and is
//!   `24 4D 3C 00 cmd cmd` (length 0, checksum = command).
//! - Replies: MSP `24 4D 3E`, as in `tests/msp.rs` (AP:370-380, 478-526, 572-600); an MSP error is
//!   `24 4D 21 00 cmd cmd` (SIM:524); four-way `2E`, command, address, count, parameters,
//!   acknowledge, CRC16-XMODEM (`four_way.ts:433-478`, AP:610-623), which must echo the request's
//!   command and address (`four_way.ts:400-402`).
//! - Housekeeping answers: protocol version 107 (HDR:123, AP:993-998), name `04 'A' 'R' 'D' 'U'`
//!   (AP:1000-1005), version 20005 as `C8 05` (HDR:119-133, AP:1006-1011).
//! - Failure reply: command and address of the request, one parameter byte (the request's first),
//!   the acknowledge (AP:1057-1060, 1080-1083). ArduPilot never answers a bad CRC (AP:294-300, the
//!   code is only defined at HDR:160); this crate answers with that shape and `03`.
//!
//! All byte vectors were laid out by hand from these formulas and their CRCs computed with a
//! separate script (bit-serial CRC16-XMODEM, check value 0x31C3 for "123456789"); none comes from
//! the crate's encoders. The values 25.2 V, 30 A, 1234 mAh, 4 motors, pin code 0x14, signature
//! 0x1F06 and the settings address 0x7C00 are examples.

mod common;

use common::ref_crc_xmodem;
use esc_passthrough::EncodeError;
use esc_passthrough::fourway::{Ack, Command, Request};
use esc_passthrough::host::{
    DEFAULT_INTERFACE_NAME, DEFAULT_INTERFACE_VERSION, DEFAULT_PROTOCOL_VERSION, DecisionError,
    Event, HostInfo, HostSide, MAX_REPLY_LEN, Mode, answer_housekeeping, encode_failure,
    encode_success,
};
use esc_passthrough::msp::{ApiVersion, BatteryState};
use proptest::prelude::*;

// ------------------------------------------------------------------- vectors

const MSP_API_REQ: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
const MSP_VARIANT_REQ: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x02, 0x02];
const MSP_BATTERY_REQ: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x82, 0x82];
const MSP_MOTOR_REQ: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x83, 0x83];
const MSP_PASSTHROUGH_REQ: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5];

const MSP_API_REPLY_1_46: [u8; 9] = [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2E, 0x2D];
const MSP_API_REPLY_1_42: [u8; 9] = [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2A, 0x29];
const MSP_VARIANT_REPLY: [u8; 10] = [0x24, 0x4D, 0x3E, 0x04, 0x02, 0x41, 0x52, 0x44, 0x55, 0x04];
const MSP_BATTERY_REPLY: [u8; 17] = [
    0x24, 0x4D, 0x3E, 0x0B, 0x82, 0x00, 0x00, 0x00, 0xFC, 0xD2, 0x04, 0xB8, 0x0B, 0x00, 0xD8, 0x09,
    0xC1,
];
const MSP_MOTOR_REPLY: [u8; 16] = [
    0x24, 0x4D, 0x3E, 0x0A, 0x83, 0, 0, 0, 0, 0, 0, 0x04, 0, 0, 0, 0x8D,
];
const MSP_PASSTHROUGH_REPLY_4: [u8; 7] = [0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0];
const MSP_ERROR_245: [u8; 6] = [0x24, 0x4D, 0x21, 0x00, 0xF5, 0xF5];
const MSP_ERROR_99: [u8; 6] = [0x24, 0x4D, 0x21, 0x00, 0x63, 0x63];
const MSP_ERROR_104: [u8; 6] = [0x24, 0x4D, 0x21, 0x00, 0x68, 0x68];

const FW_INIT_REQ: [u8; 8] = [0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00];
const FW_INIT_2_REQ: [u8; 8] = [0x2F, 0x37, 0x00, 0x00, 0x01, 0x02, 0x88, 0x42];
const FW_READ_REQ: [u8; 8] = [0x2F, 0x3A, 0x7C, 0x00, 0x01, 0x04, 0xC4, 0xB9];
const FW_EXIT_REQ: [u8; 8] = [0x2F, 0x34, 0x00, 0x00, 0x01, 0x00, 0x46, 0xD2];
const FW_PROTOCOL_REQ: [u8; 8] = [0x2F, 0x31, 0x00, 0x00, 0x01, 0x00, 0x65, 0x85];
const FW_NAME_REQ: [u8; 8] = [0x2F, 0x32, 0x00, 0x00, 0x01, 0x00, 0x8B, 0x57];
const FW_VERSION_REQ: [u8; 8] = [0x2F, 0x33, 0x00, 0x00, 0x01, 0x00, 0x21, 0x06];
const FW_ALIVE_REQ: [u8; 8] = [0x2F, 0x30, 0x00, 0x00, 0x01, 0x00, 0xCF, 0xD4];
// DeviceWrite at 0x7C00 with the parameters AA 24 4D 3C 00 01 01 BB: the MSP request inside.
const FW_WRITE_WITH_MSP_INSIDE_REQ: [u8; 15] = [
    0x2F, 0x3B, 0x7C, 0x00, 0x08, 0xAA, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01, 0xBB, 0xC3, 0xDB,
];

const FW_INIT_REPLY: [u8; 12] = [
    0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A,
];
const FW_READ_REPLY: [u8; 12] = [
    0x2E, 0x3A, 0x7C, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x00, 0x6E, 0xD4,
];
const FW_EXIT_REPLY: [u8; 9] = [0x2E, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x42, 0x63];
const FW_PROTOCOL_REPLY: [u8; 9] = [0x2E, 0x31, 0x00, 0x00, 0x01, 0x6B, 0x00, 0xD6, 0xB2];
const FW_NAME_REPLY: [u8; 13] = [
    0x2E, 0x32, 0x00, 0x00, 0x05, 0x04, 0x41, 0x52, 0x44, 0x55, 0x00, 0xE6, 0x5A,
];
const FW_VERSION_REPLY: [u8; 10] = [0x2E, 0x33, 0x00, 0x00, 0x02, 0xC8, 0x05, 0x00, 0xF9, 0xBD];
const FW_WRITE_REPLY: [u8; 9] = [0x2E, 0x3B, 0x7C, 0x00, 0x01, 0x00, 0x00, 0x11, 0xCD];
// Bad CRC answers: command, address and first parameter of the bad frame, acknowledge 03.
const FW_BAD_CRC_INIT_2_REPLY: [u8; 9] = [0x2E, 0x37, 0x00, 0x00, 0x01, 0x02, 0x03, 0xDA, 0x82];
const FW_BAD_CRC_READ_REPLY: [u8; 9] = [0x2E, 0x3A, 0x7C, 0x00, 0x01, 0x04, 0x03, 0xA8, 0xCA];
// Failure replies for InitFlash of ESC 5 (acknowledge 08, invalid channel).
const FW_FAIL_INIT_5_REPLY: [u8; 9] = [0x2E, 0x37, 0x00, 0x00, 0x01, 0x05, 0x08, 0xF2, 0x7E];

fn conversation_info() -> HostInfo {
    HostInfo::new()
        .with_api_version(ApiVersion::new(0, 1, 46))
        .with_battery(
            BatteryState::new(25_200)
                .with_current_ma(Some(30_000))
                .with_charge_mah(Some(1234)),
        )
        .with_motor_count(4)
}

// -------------------------------------------------------------- the test caller

#[derive(Debug, Clone, PartialEq, Eq)]
enum Item {
    Reply(Vec<u8>),
    ReplyFailed(EncodeError),
    Passthrough,
    Granted(Vec<u8>),
    Refused(Vec<u8>),
    FourWay(Command, u16, Vec<u8>),
    Answered(Vec<u8>),
    Restarted(u8, Vec<u8>),
    Exited,
}

enum After {
    Nothing,
    Decide,
    Exit,
}

/// What the application does: feeds bytes, acts on every event.
struct Caller {
    host: HostSide,
    info: HostInfo,
    out: Vec<u8>,
    items: Vec<Item>,
    /// Bytes fed so far when each event was reported (the item list has more entries than events).
    at: Vec<usize>,
    fed: usize,
    /// How many `MSP_SET_PASSTHROUGH` requests to refuse before granting.
    refusals: usize,
    /// Answer four-way requests (otherwise only record them).
    answer: bool,
    /// Call `exited()` on `Restarted`.
    exit_on_restart: bool,
}

impl Caller {
    fn new(info: HostInfo, out_len: usize) -> Self {
        Self {
            host: HostSide::new(),
            info,
            out: vec![0; out_len],
            items: Vec::new(),
            at: Vec::new(),
            fed: 0,
            refusals: 0,
            answer: true,
            exit_on_restart: true,
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        let mut rest = bytes;
        while !rest.is_empty() {
            let (used, event) = self.host.feed(rest, &self.info, &mut self.out);
            assert!(used <= rest.len());
            self.fed += used;
            rest = &rest[used..];
            let mut after = After::Nothing;
            if let Some(event) = event {
                assert!(used >= 1, "an event consumed no byte");
                self.at.push(self.fed);
                match event {
                    Event::Reply { len } => {
                        assert!(len >= 1 && len <= self.out.len());
                        self.items.push(Item::Reply(self.out[..len].to_vec()));
                    }
                    Event::ReplyFailed(error) => self.items.push(Item::ReplyFailed(error)),
                    Event::PassthroughRequested => {
                        self.items.push(Item::Passthrough);
                        after = After::Decide;
                    }
                    Event::FourWay(request) => {
                        self.items.push(Item::FourWay(
                            request.command,
                            request.address,
                            request.params.to_vec(),
                        ));
                        if self.answer {
                            let mut reply = [0u8; 300];
                            let result = match request.command {
                                Command::DeviceInitFlash => {
                                    encode_success(&request, &[0x06, 0x1F, 0x14, 0x04], &mut reply)
                                }
                                Command::DeviceRead => {
                                    encode_success(&request, &[1, 2, 3, 4], &mut reply)
                                }
                                Command::DeviceWrite => encode_success(&request, &[0], &mut reply),
                                Command::InterfaceExit => {
                                    after = After::Exit;
                                    encode_success(&request, &[0], &mut reply)
                                }
                                Command::ProtocolGetVersion
                                | Command::InterfaceGetName
                                | Command::InterfaceGetVersion => {
                                    answer_housekeeping(&self.info, &request, &mut reply)
                                        .expect("housekeeping command")
                                }
                                _ => encode_failure(&request, Ack::IInvalidCmd, &mut reply),
                            };
                            let len = result.expect("reply fits");
                            self.items.push(Item::Answered(reply[..len].to_vec()));
                        }
                    }
                    Event::Restarted(request) => {
                        self.items
                            .push(Item::Restarted(request.command, request.payload.to_vec()));
                        if self.exit_on_restart {
                            after = After::Exit;
                        }
                    }
                    _ => panic!("unknown event"),
                }
            }
            match after {
                After::Nothing => {}
                After::Decide => {
                    let mut reply = [0u8; 16];
                    if self.refusals > 0 {
                        self.refusals -= 1;
                        let len = self.host.refuse(&mut reply).expect("refuse");
                        self.items.push(Item::Refused(reply[..len].to_vec()));
                    } else {
                        let len = self.host.grant(4, &mut reply).expect("grant");
                        self.items.push(Item::Granted(reply[..len].to_vec()));
                    }
                }
                After::Exit => {
                    self.host.exited();
                    self.items.push(Item::Exited);
                }
            }
        }
    }

    fn feed_chunks(&mut self, bytes: &[u8], cuts: &[usize]) {
        let mut points: Vec<usize> = cuts.iter().map(|c| c % (bytes.len() + 1)).collect();
        points.push(0);
        points.push(bytes.len());
        points.sort_unstable();
        for pair in points.windows(2) {
            self.feed(&bytes[pair[0]..pair[1]]);
        }
    }
}

fn cat(parts: &[&[u8]]) -> Vec<u8> {
    parts.concat()
}

/// A caller that is already in four-way mode.
fn four_way_caller() -> Caller {
    let mut caller = Caller::new(conversation_info(), 64);
    caller.feed(&MSP_PASSTHROUGH_REQ);
    assert_eq!(caller.host.mode(), Mode::FourWay);
    caller.items.clear();
    caller.at.clear();
    caller.fed = 0;
    caller
}

// ------------------------------------------------------------- the conversation

fn conversation_bytes() -> Vec<u8> {
    cat(&[
        &MSP_API_REQ,
        &MSP_VARIANT_REQ,
        &MSP_BATTERY_REQ,
        &MSP_MOTOR_REQ,
        &MSP_PASSTHROUGH_REQ,
        &FW_INIT_REQ,
        &FW_READ_REQ,
        &FW_EXIT_REQ,
        &MSP_API_REQ,
    ])
}

fn conversation_items() -> Vec<Item> {
    vec![
        Item::Reply(MSP_API_REPLY_1_46.to_vec()),
        Item::Reply(MSP_VARIANT_REPLY.to_vec()),
        Item::Reply(MSP_BATTERY_REPLY.to_vec()),
        Item::Reply(MSP_MOTOR_REPLY.to_vec()),
        Item::Passthrough,
        Item::Granted(MSP_PASSTHROUGH_REPLY_4.to_vec()),
        Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
        Item::Answered(FW_INIT_REPLY.to_vec()),
        Item::FourWay(Command::DeviceRead, 0x7C00, vec![4]),
        Item::Answered(FW_READ_REPLY.to_vec()),
        Item::FourWay(Command::InterfaceExit, 0, vec![0]),
        Item::Answered(FW_EXIT_REPLY.to_vec()),
        Item::Exited,
        Item::Reply(MSP_API_REPLY_1_46.to_vec()),
    ]
}

/// Bytes fed when each event is reported: the end of each request.
const CONVERSATION_ENDS: [usize; 9] = [6, 12, 18, 24, 30, 38, 46, 54, 60];

#[test]
fn feed_runs_the_connect_conversation_byte_by_byte() {
    let stream = conversation_bytes();
    assert_eq!(stream.len(), 60);
    let mut caller = Caller::new(conversation_info(), 64);
    for byte in &stream {
        caller.feed(std::slice::from_ref(byte));
    }
    assert_eq!(caller.items, conversation_items());
    // Invariant 5.2: an event, and with it a reply, comes at the last byte of a request and
    // at no other byte.
    assert_eq!(caller.at, CONVERSATION_ENDS);
    assert_eq!(caller.host.mode(), Mode::Msp);
}

#[test]
fn feed_runs_the_connect_conversation_in_one_slice() {
    let stream = conversation_bytes();
    let mut caller = Caller::new(conversation_info(), 64);
    caller.feed(&stream);
    assert_eq!(caller.items, conversation_items());
    assert_eq!(caller.at, CONVERSATION_ENDS);
}

#[test]
fn feed_gives_the_same_events_and_replies_at_every_split_of_the_conversation() {
    let stream = conversation_bytes();
    for split in 0..=stream.len() {
        let mut caller = Caller::new(conversation_info(), 64);
        caller.feed(&stream[..split]);
        caller.feed(&stream[split..]);
        assert_eq!(caller.items, conversation_items(), "split {split}");
        assert_eq!(caller.at, CONVERSATION_ENDS, "split {split}");
    }
    // Three parts, every pair of cuts.
    for first in 0..=stream.len() {
        for second in first..=stream.len() {
            let mut caller = Caller::new(conversation_info(), 64);
            caller.feed(&stream[..first]);
            caller.feed(&stream[first..second]);
            caller.feed(&stream[second..]);
            assert_eq!(caller.items, conversation_items(), "cuts {first} {second}");
        }
    }
}

// --------------------------------------------------------------------- pub fns

#[test]
fn host_info_new_holds_arduino_pilots_values() {
    let info = HostInfo::new();
    assert_eq!(info.api_version, ApiVersion::new(0, 1, 42));
    assert_eq!(info.fc_variant, *b"ARDU");
    assert_eq!(info.battery, BatteryState::new(0));
    assert_eq!(info.motor_count, 0);
    // HDR:123, AP:1002, HDR:119-133 (20005 / 100 = 200, 20005 % 100 = 5).
    assert_eq!(info.protocol_version, 107);
    assert_eq!(info.interface_name, &[4, 0x41, 0x52, 0x44, 0x55]);
    assert_eq!(info.interface_version, [200, 5]);
    assert_eq!(DEFAULT_PROTOCOL_VERSION, 107);
    assert_eq!(DEFAULT_INTERFACE_NAME, &[4, b'A', b'R', b'D', b'U']);
    assert_eq!(DEFAULT_INTERFACE_VERSION, [200, 5]);
    assert_eq!(HostInfo::default(), info);
}

#[test]
fn with_api_version_sets_the_triple_and_the_reply() {
    let info = HostInfo::new().with_api_version(ApiVersion::new(0, 1, 46));
    assert_eq!(info.api_version, ApiVersion::new(0, 1, 46));
    let mut caller = Caller::new(info, 32);
    caller.feed(&MSP_API_REQ);
    assert_eq!(caller.items, [Item::Reply(MSP_API_REPLY_1_46.to_vec())]);
    // The default is 0.1.42.
    let mut caller = Caller::new(HostInfo::new(), 32);
    caller.feed(&MSP_API_REQ);
    assert_eq!(caller.items, [Item::Reply(MSP_API_REPLY_1_42.to_vec())]);
}

#[test]
fn with_fc_variant_sets_the_four_bytes_and_the_reply() {
    let info = HostInfo::new().with_fc_variant(*b"BTFL");
    assert_eq!(info.fc_variant, *b"BTFL");
    let mut caller = Caller::new(info, 32);
    caller.feed(&MSP_VARIANT_REQ);
    assert_eq!(
        caller.items,
        [Item::Reply(vec![
            0x24, 0x4D, 0x3E, 0x04, 0x02, 0x42, 0x54, 0x46, 0x4C, 0x1A
        ])]
    );
    let mut caller = Caller::new(HostInfo::new(), 32);
    caller.feed(&MSP_VARIANT_REQ);
    assert_eq!(caller.items, [Item::Reply(MSP_VARIANT_REPLY.to_vec())]);
}

#[test]
fn with_battery_sets_the_state_and_the_reply() {
    let battery = BatteryState::new(25_200)
        .with_current_ma(Some(30_000))
        .with_charge_mah(Some(1234));
    let info = HostInfo::new().with_battery(battery);
    assert_eq!(info.battery, battery);
    let mut caller = Caller::new(info, 32);
    caller.feed(&MSP_BATTERY_REQ);
    assert_eq!(caller.items, [Item::Reply(MSP_BATTERY_REPLY.to_vec())]);
    // 16.0 V alone: 0.1 V steps 160 at byte 3, 0.01 V steps 1600 = 0x0640 at bytes 9-10.
    let mut caller = Caller::new(HostInfo::new().with_battery(BatteryState::new(16_000)), 32);
    caller.feed(&MSP_BATTERY_REQ);
    assert_eq!(
        caller.items,
        [Item::Reply(vec![
            0x24, 0x4D, 0x3E, 0x0B, 0x82, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x40, 0x06, 0x6F
        ])]
    );
}

#[test]
fn with_motor_count_sets_the_count_and_the_reply() {
    let info = HostInfo::new().with_motor_count(4);
    assert_eq!(info.motor_count, 4);
    let mut caller = Caller::new(info, 32);
    caller.feed(&MSP_MOTOR_REQ);
    assert_eq!(caller.items, [Item::Reply(MSP_MOTOR_REPLY.to_vec())]);
}

#[test]
fn with_protocol_version_sets_the_byte_and_the_answer() {
    let info = HostInfo::new().with_protocol_version(108);
    assert_eq!(info.protocol_version, 108);
    let request = Request {
        command: Command::ProtocolGetVersion,
        address: 0,
        params: &[0],
    };
    let mut out = [0u8; 16];
    let len = answer_housekeeping(&info, &request, &mut out)
        .expect("answered")
        .expect("fits");
    // The frame of FW_PROTOCOL_REPLY with 0x6C in place of 0x6B; its CRC from the script.
    assert_eq!(
        &out[..len],
        [0x2E, 0x31, 0x00, 0x00, 0x01, 0x6C, 0x00, 0x4F, 0x25]
    );
}

#[test]
fn with_interface_name_sets_the_bytes_and_the_answer() {
    static NAME: [u8; 2] = *b"AB";
    let info = HostInfo::new().with_interface_name(&NAME);
    assert_eq!(info.interface_name, b"AB");
    let request = Request {
        command: Command::InterfaceGetName,
        address: 0,
        params: &[0],
    };
    let mut out = [0u8; 16];
    let len = answer_housekeeping(&info, &request, &mut out)
        .expect("answered")
        .expect("fits");
    assert_eq!(
        &out[..len - 2],
        [0x2E, 0x32, 0x00, 0x00, 0x02, 0x41, 0x42, 0x00]
    );
    let crc = ref_crc_xmodem(&out[..len - 2]).to_be_bytes();
    assert_eq!(&out[len - 2..len], crc);
}

#[test]
fn with_interface_version_sets_the_bytes_and_the_answer() {
    let info = HostInfo::new().with_interface_version([201, 7]);
    assert_eq!(info.interface_version, [201, 7]);
    let request = Request {
        command: Command::InterfaceGetVersion,
        address: 0,
        params: &[0],
    };
    let mut out = [0u8; 16];
    let len = answer_housekeeping(&info, &request, &mut out)
        .expect("answered")
        .expect("fits");
    assert_eq!(
        &out[..len - 2],
        [0x2E, 0x33, 0x00, 0x00, 0x02, 201, 7, 0x00]
    );
    let crc = ref_crc_xmodem(&out[..len - 2]).to_be_bytes();
    assert_eq!(&out[len - 2..len], crc);
}

#[test]
fn host_side_new_starts_in_msp_mode_and_default_is_the_same() {
    assert_eq!(HostSide::new().mode(), Mode::Msp);
    assert_eq!(HostSide::default().mode(), Mode::Msp);
    // Nothing received: nothing to report.
    let mut host = HostSide::new();
    let mut out = [0u8; 32];
    assert_eq!(host.feed(&[], &HostInfo::new(), &mut out).0, 0);
    assert!(host.feed(&[], &HostInfo::new(), &mut out).1.is_none());
}

#[test]
fn mode_follows_grant_refuse_and_exited() {
    let mut host = HostSide::new();
    let info = HostInfo::new();
    let mut out = [0u8; 32];
    assert_eq!(host.mode(), Mode::Msp);
    let (used, event) = host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    assert_eq!((used, event), (6, Some(Event::PassthroughRequested)));
    // Waiting for the decision: still MSP.
    assert_eq!(host.mode(), Mode::Msp);
    assert_eq!(host.refuse(&mut out), Ok(6));
    assert_eq!(host.mode(), Mode::Msp);
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    assert_eq!(host.grant(2, &mut out), Ok(7));
    assert_eq!(host.mode(), Mode::FourWay);
    host.exited();
    assert_eq!(host.mode(), Mode::Msp);
}

#[test]
fn feed_answers_the_four_msp_requests_from_the_info() {
    let info = conversation_info();
    let cases: [(&[u8], &[u8]); 4] = [
        (&MSP_API_REQ, &MSP_API_REPLY_1_46),
        (&MSP_VARIANT_REQ, &MSP_VARIANT_REPLY),
        (&MSP_BATTERY_REQ, &MSP_BATTERY_REPLY),
        (&MSP_MOTOR_REQ, &MSP_MOTOR_REPLY),
    ];
    for (request, reply) in cases {
        let mut host = HostSide::new();
        let mut out = [0u8; MAX_REPLY_LEN];
        let (used, event) = host.feed(request, &info, &mut out);
        assert_eq!(used, 6);
        assert_eq!(event, Some(Event::Reply { len: reply.len() }));
        assert_eq!(&out[..reply.len()], reply);
        assert_eq!(host.mode(), Mode::Msp);
    }
    assert_eq!(MAX_REPLY_LEN, 17);
}

#[test]
fn feed_stops_at_the_first_event_and_leaves_the_rest() {
    let mut host = HostSide::new();
    let info = conversation_info();
    let mut out = [0u8; 32];
    let stream = cat(&[&MSP_API_REQ, &MSP_VARIANT_REQ]);
    let (used, event) = host.feed(&stream, &info, &mut out);
    assert_eq!((used, event), (6, Some(Event::Reply { len: 9 })));
    assert_eq!(&out[..9], &MSP_API_REPLY_1_46);
    // Nothing of the second request was looked at; it now gets its own reply.
    let (used, event) = host.feed(&stream[6..], &info, &mut out);
    assert_eq!((used, event), (6, Some(Event::Reply { len: 10 })));
    assert_eq!(&out[..10], &MSP_VARIANT_REPLY);
    // Nothing but noise: all of it used, no event.
    let (used, event) = host.feed(&[0, 1, 2, 3], &info, &mut out);
    assert_eq!((used, event), (4, None));
}

#[test]
fn feed_reports_set_passthrough_and_grant_replies_with_the_count() {
    let mut caller = Caller::new(conversation_info(), 32);
    caller.feed(&MSP_PASSTHROUGH_REQ);
    assert_eq!(
        caller.items,
        [
            Item::Passthrough,
            Item::Granted(MSP_PASSTHROUGH_REPLY_4.to_vec())
        ]
    );
    assert_eq!(caller.at, [6]);
    assert_eq!(caller.host.mode(), Mode::FourWay);
}

#[test]
fn grant_replies_with_the_count_and_switches_to_four_way() {
    let mut host = HostSide::new();
    let info = conversation_info();
    let mut out = [0u8; 32];
    // Nothing waiting yet.
    assert_eq!(host.grant(4, &mut out), Err(DecisionError::NotRequested));
    assert_eq!(host.mode(), Mode::Msp);
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    // A buffer that is too small leaves the request waiting and the mode unchanged.
    let mut small = [0u8; 6];
    assert_eq!(
        host.grant(4, &mut small),
        Err(DecisionError::Encode(EncodeError::BufferTooSmall {
            needed: 7
        }))
    );
    assert_eq!(host.mode(), Mode::Msp);
    assert_eq!(host.grant(4, &mut out), Ok(7));
    assert_eq!(&out[..7], &MSP_PASSTHROUGH_REPLY_4);
    assert_eq!(host.mode(), Mode::FourWay);
    // Decided: a second grant has nothing to answer.
    assert_eq!(host.grant(4, &mut out), Err(DecisionError::NotRequested));
}

#[test]
fn refuse_replies_with_the_msp_error_and_the_second_attempt_works() {
    let mut caller = Caller::new(conversation_info(), 32);
    caller.refusals = 1;
    let stream = cat(&[&MSP_PASSTHROUGH_REQ, &MSP_API_REQ, &MSP_PASSTHROUGH_REQ]);
    caller.feed(&stream);
    assert_eq!(
        caller.items,
        [
            Item::Passthrough,
            Item::Refused(MSP_ERROR_245.to_vec()),
            Item::Reply(MSP_API_REPLY_1_46.to_vec()),
            Item::Passthrough,
            Item::Granted(MSP_PASSTHROUGH_REPLY_4.to_vec()),
        ]
    );
    assert_eq!(caller.at, [6, 12, 18]);
    assert_eq!(caller.host.mode(), Mode::FourWay);

    // Without a request waiting, refuse does nothing; with a small buffer it keeps waiting.
    let mut host = HostSide::new();
    let mut out = [0u8; 32];
    assert_eq!(host.refuse(&mut out), Err(DecisionError::NotRequested));
    host.feed(&MSP_PASSTHROUGH_REQ, &conversation_info(), &mut out);
    let mut small = [0u8; 5];
    assert_eq!(
        host.refuse(&mut small),
        Err(DecisionError::Encode(EncodeError::BufferTooSmall {
            needed: 6
        }))
    );
    assert_eq!(host.refuse(&mut out), Ok(6));
    assert_eq!(&out[..6], &MSP_ERROR_245);
}

#[test]
fn feed_reports_the_waiting_request_again_until_it_is_decided() {
    let mut host = HostSide::new();
    let info = conversation_info();
    let mut out = [0u8; 32];
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    let stream = cat(&[&MSP_API_REQ]);
    // Nothing is processed before the caller has acted.
    assert_eq!(
        host.feed(&stream, &info, &mut out),
        (0, Some(Event::PassthroughRequested))
    );
    assert_eq!(
        host.feed(&stream, &info, &mut out),
        (0, Some(Event::PassthroughRequested))
    );
    assert_eq!(host.refuse(&mut out), Ok(6));
    assert_eq!(
        host.feed(&stream, &info, &mut out),
        (6, Some(Event::Reply { len: 9 }))
    );
}

#[test]
fn feed_answers_an_unknown_msp_command_with_the_error_reply() {
    let mut caller = Caller::new(conversation_info(), 32);
    // 99 (unknown) and 104 (MSP_MOTOR, which the configurator asks only of inav and emuf).
    caller.feed(&cat(&[
        &[0x24, 0x4D, 0x3C, 0x00, 0x63, 0x63],
        &[0x24, 0x4D, 0x3C, 0x00, 0x68, 0x68],
        &MSP_API_REQ,
    ]));
    assert_eq!(
        caller.items,
        [
            Item::Reply(MSP_ERROR_99.to_vec()),
            Item::Reply(MSP_ERROR_104.to_vec()),
            Item::Reply(MSP_API_REPLY_1_46.to_vec()),
        ]
    );
    assert_eq!(caller.at, [6, 12, 18]);
    assert_eq!(caller.host.mode(), Mode::Msp);
}

#[test]
fn feed_answers_set_passthrough_with_a_payload_with_the_error_reply() {
    // AP:572-600 reads a 2-byte payload as another protocol; this crate only knows the request
    // without payload and does not report the other as a request to enter four-way.
    let mut caller = Caller::new(conversation_info(), 32);
    caller.feed(&[0x24, 0x4D, 0x3C, 0x02, 0xF5, 0x00, 0x01, 0xF6]);
    assert_eq!(caller.items, [Item::Reply(MSP_ERROR_245.to_vec())]);
    assert_eq!(caller.at, [8]);
    assert_eq!(caller.host.mode(), Mode::Msp);
    let mut out = [0u8; 8];
    assert_eq!(
        caller.host.grant(4, &mut out),
        Err(DecisionError::NotRequested)
    );
}

#[test]
fn feed_gives_nothing_for_a_malformed_msp_frame() {
    let cases: [&[u8]; 6] = [
        // Bad checksum.
        &[0x24, 0x4D, 0x3C, 0x00, 0x01, 0x02],
        // A reply, an error reply and an MSP v2 frame.
        &[0x24, 0x4D, 0x3E, 0x00, 0x01, 0x01],
        &[0x24, 0x4D, 0x21, 0x00, 0x01, 0x01],
        &[0x24, 0x58, 0x3C, 0x00, 0x01, 0x01],
        // Noise, and a four-way request in MSP mode.
        &[0x00, 0xFF, 0x12, 0x34],
        &FW_INIT_REQ,
    ];
    for case in cases {
        let mut caller = Caller::new(conversation_info(), 32);
        caller.feed(case);
        assert!(caller.items.is_empty(), "{case:02X?}: {:?}", caller.items);
        // And the host side still works.
        caller.feed(&MSP_API_REQ);
        assert_eq!(
            caller.items,
            [Item::Reply(MSP_API_REPLY_1_46.to_vec())],
            "{case:02X?}"
        );
    }
}

#[test]
fn feed_hands_out_four_way_requests_with_command_address_and_parameters() {
    let mut caller = four_way_caller();
    caller.answer = false;
    caller.feed(&cat(&[
        &FW_INIT_REQ,
        &FW_READ_REQ,
        &FW_WRITE_WITH_MSP_INSIDE_REQ,
    ]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::FourWay(Command::DeviceRead, 0x7C00, vec![4]),
            Item::FourWay(
                Command::DeviceWrite,
                0x7C00,
                vec![0xAA, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01, 0xBB]
            ),
        ]
    );
    assert_eq!(caller.at, [8, 16, 31]);
}

#[test]
fn feed_hands_out_a_command_it_does_not_know_and_a_256_byte_block() {
    let mut caller = four_way_caller();
    caller.answer = false;
    // Command 0x36 (the removed GetID) with one parameter; and a 256-byte write sent with count 0.
    let mut unknown = vec![0x2F, 0x36, 0x12, 0x34, 0x01, 0x77];
    let crc = ref_crc_xmodem(&unknown).to_be_bytes();
    unknown.extend_from_slice(&crc);
    let params: Vec<u8> = (0..=255u8).collect();
    let mut block = vec![0x2F, 0x3B, 0x7C, 0x00, 0x00];
    block.extend_from_slice(&params);
    let crc = ref_crc_xmodem(&block).to_be_bytes();
    block.extend_from_slice(&crc);
    caller.feed(&cat(&[&unknown, &block]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::Unknown(0x36), 0x1234, vec![0x77]),
            Item::FourWay(Command::DeviceWrite, 0x7C00, params),
        ]
    );
}

#[test]
fn feed_answers_a_four_way_request_with_a_bad_crc_and_does_not_hand_it_out() {
    let mut bad_init = FW_INIT_2_REQ;
    bad_init[7] ^= 0x01;
    let mut bad_read = FW_READ_REQ;
    bad_read[6] ^= 0x80;
    let mut caller = four_way_caller();
    caller.feed(&cat(&[&bad_init, &FW_INIT_REQ, &bad_read, &FW_READ_REQ]));
    assert_eq!(
        caller.items,
        [
            Item::Reply(FW_BAD_CRC_INIT_2_REPLY.to_vec()),
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
            Item::Reply(FW_BAD_CRC_READ_REPLY.to_vec()),
            Item::FourWay(Command::DeviceRead, 0x7C00, vec![4]),
            Item::Answered(FW_READ_REPLY.to_vec()),
        ]
    );
    // Each answer comes at the last byte of its request.
    assert_eq!(caller.at, [8, 16, 24, 32]);
    assert_eq!(caller.host.mode(), Mode::FourWay);
}

#[test]
fn feed_restarts_a_four_way_frame_at_a_last_byte_that_is_the_start_byte() {
    // The bad frame's last byte is 0x2F; the parser looks at it again as a start, so the bytes
    // after it complete a request of their own: `2F` + `37 00 00 01 00 A8 00`.
    let mut stream = vec![0x2F, 0x3A, 0x7C, 0x00, 0x01, 0x04, 0xC4, 0x2F];
    stream.extend_from_slice(&FW_INIT_REQ[1..]);
    let mut caller = four_way_caller();
    caller.answer = false;
    for byte in &stream {
        caller.feed(std::slice::from_ref(byte));
    }
    assert_eq!(
        caller.items,
        [
            Item::Reply(FW_BAD_CRC_READ_REPLY.to_vec()),
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
        ]
    );
    assert_eq!(caller.at, [8, 15]);
    // The same stream in one slice.
    let mut caller = four_way_caller();
    caller.answer = false;
    caller.feed(&stream);
    assert_eq!(caller.items.len(), 2);
    assert_eq!(caller.at, [8, 15]);
}

#[test]
fn feed_reports_a_valid_msp_request_between_four_way_frames_as_restarted() {
    let mut caller = four_way_caller();
    caller.exit_on_restart = false;
    let stream = cat(&[&FW_INIT_REQ, &[0x00, 0xFF], &MSP_API_REQ, &FW_READ_REQ]);
    caller.feed(&stream);
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
            Item::Restarted(1, vec![]),
            Item::FourWay(Command::DeviceRead, 0x7C00, vec![4]),
            Item::Answered(FW_READ_REPLY.to_vec()),
        ]
    );
    assert_eq!(caller.at, [8, 16, 24]);
    // The host side does not decide: the mode is four-way until the caller says `exited()`.
    assert_eq!(caller.host.mode(), Mode::FourWay);

    // With the configurator's recovery: the session ends and MSP answers the next request.
    let mut caller = four_way_caller();
    caller.feed(&cat(&[&FW_INIT_REQ, &MSP_API_REQ, &MSP_API_REQ]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
            Item::Restarted(1, vec![]),
            Item::Exited,
            Item::Reply(MSP_API_REPLY_1_46.to_vec()),
        ]
    );
    assert_eq!(caller.host.mode(), Mode::Msp);
}

#[test]
fn feed_reports_any_valid_msp_request_with_its_payload_as_restarted() {
    let mut caller = four_way_caller();
    caller.exit_on_restart = false;
    caller.feed(&cat(&[
        &MSP_PASSTHROUGH_REQ,
        &[0x24, 0x4D, 0x3C, 0x02, 0xF5, 0x00, 0x01, 0xF6],
    ]));
    assert_eq!(
        caller.items,
        [
            Item::Restarted(0xF5, vec![]),
            Item::Restarted(0xF5, vec![0x00, 0x01]),
        ]
    );
}

#[test]
fn feed_does_not_offer_bytes_inside_a_four_way_frame_to_the_msp_parser() {
    // The six bytes of a restart inside the parameters of a write: one request, no restart.
    let mut caller = four_way_caller();
    caller.exit_on_restart = false;
    for byte in &FW_WRITE_WITH_MSP_INSIDE_REQ {
        caller.feed(std::slice::from_ref(byte));
    }
    assert_eq!(
        caller.items,
        [
            Item::FourWay(
                Command::DeviceWrite,
                0x7C00,
                vec![0xAA, 0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01, 0xBB]
            ),
            Item::Answered(FW_WRITE_REPLY.to_vec()),
        ]
    );
    // And at every split, with a real restart after it.
    let stream = cat(&[&FW_WRITE_WITH_MSP_INSIDE_REQ, &MSP_API_REQ]);
    for split in 0..=stream.len() {
        let mut caller = four_way_caller();
        caller.exit_on_restart = false;
        caller.feed(&stream[..split]);
        caller.feed(&stream[split..]);
        assert_eq!(caller.items.len(), 3, "split {split}: {:?}", caller.items);
        assert_eq!(caller.items[2], Item::Restarted(1, vec![]), "split {split}");
    }
}

#[test]
fn feed_forgets_a_partial_msp_request_when_a_four_way_frame_starts() {
    // `$M` between frames, then a four-way frame, then `<` 00 01 01: not a request.
    let mut caller = four_way_caller();
    caller.exit_on_restart = false;
    caller.feed(&cat(&[
        &[0x24, 0x4D],
        &FW_INIT_REQ,
        &[0x3C, 0x00, 0x01, 0x01],
    ]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
        ]
    );
}

#[test]
fn feed_ignores_msp_errors_in_four_way_mode() {
    let mut caller = four_way_caller();
    caller.exit_on_restart = false;
    caller.feed(&cat(&[
        // Bad checksum, a reply frame, a v2 frame.
        &[0x24, 0x4D, 0x3C, 0x00, 0x01, 0x02],
        &[0x24, 0x4D, 0x3E, 0x00, 0x01, 0x01],
        &[0x24, 0x58, 0x3C, 0x00, 0x01, 0x01],
        &FW_INIT_REQ,
    ]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
        ]
    );
}

#[test]
fn exited_returns_to_msp_mode_and_drops_what_was_pending() {
    let mut caller = four_way_caller();
    caller.feed(&cat(&[&FW_INIT_REQ, &FW_EXIT_REQ, &MSP_VARIANT_REQ]));
    assert_eq!(
        caller.items,
        [
            Item::FourWay(Command::DeviceInitFlash, 0, vec![0]),
            Item::Answered(FW_INIT_REPLY.to_vec()),
            Item::FourWay(Command::InterfaceExit, 0, vec![0]),
            Item::Answered(FW_EXIT_REPLY.to_vec()),
            Item::Exited,
            Item::Reply(MSP_VARIANT_REPLY.to_vec()),
        ]
    );
    assert_eq!(caller.host.mode(), Mode::Msp);

    // A request waiting for its decision is dropped, and a half-received frame with it.
    let mut host = HostSide::new();
    let info = conversation_info();
    let mut out = [0u8; 32];
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    host.exited();
    assert_eq!(host.grant(4, &mut out), Err(DecisionError::NotRequested));
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    host.grant(4, &mut out).unwrap();
    host.feed(&FW_INIT_REQ[..5], &info, &mut out);
    host.exited();
    assert_eq!(
        host.feed(&MSP_API_REQ, &info, &mut out),
        (6, Some(Event::Reply { len: 9 }))
    );
    // Four-way again after another grant: the old half frame is gone.
    host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out);
    host.grant(4, &mut out).unwrap();
    let (used, event) = host.feed(&FW_INIT_REQ, &info, &mut out);
    assert_eq!(used, 8);
    assert!(matches!(
        event,
        Some(Event::FourWay(Request {
            command: Command::DeviceInitFlash,
            address: 0,
            params: [0],
        }))
    ));
}

#[test]
fn feed_reports_a_reply_that_does_not_fit_and_goes_on() {
    // MSP: the battery reply needs 17 bytes.
    let info = conversation_info();
    let mut host = HostSide::new();
    let mut small = [0u8; 16];
    assert_eq!(
        host.feed(&MSP_BATTERY_REQ, &info, &mut small),
        (
            6,
            Some(Event::ReplyFailed(EncodeError::BufferTooSmall {
                needed: 17
            }))
        )
    );
    assert_eq!(small, [0u8; 16]);
    let mut empty: [u8; 0] = [];
    assert_eq!(
        host.feed(&MSP_API_REQ, &info, &mut empty),
        (
            6,
            Some(Event::ReplyFailed(EncodeError::BufferTooSmall {
                needed: 9
            }))
        )
    );
    let mut out = [0u8; 17];
    assert_eq!(
        host.feed(&MSP_API_REQ, &info, &mut out),
        (6, Some(Event::Reply { len: 9 }))
    );

    // Four-way, the bad CRC reply needs 9.
    let mut caller = four_way_caller();
    caller.out = vec![0; 8];
    let mut bad = FW_INIT_2_REQ;
    bad[7] ^= 1;
    caller.feed(&cat(&[&bad, &FW_INIT_REQ]));
    assert_eq!(
        caller.items[0],
        Item::ReplyFailed(EncodeError::BufferTooSmall { needed: 9 })
    );
    assert_eq!(
        caller.items[1],
        Item::FourWay(Command::DeviceInitFlash, 0, vec![0])
    );
}

#[test]
fn answer_housekeeping_answers_the_three_commands_without_a_motor_controller() {
    let info = HostInfo::new();
    let cases: [(&[u8], &[u8]); 3] = [
        (&FW_PROTOCOL_REQ, &FW_PROTOCOL_REPLY),
        (&FW_NAME_REQ, &FW_NAME_REPLY),
        (&FW_VERSION_REQ, &FW_VERSION_REPLY),
    ];
    for (request, reply) in cases {
        let mut caller = four_way_caller();
        caller.info = info;
        caller.feed(request);
        assert_eq!(caller.items.len(), 2);
        assert_eq!(caller.items[1], Item::Answered(reply.to_vec()));
    }
    // The address of the request is echoed.
    let request = Request {
        command: Command::InterfaceGetVersion,
        address: 0x1234,
        params: &[0],
    };
    let mut out = [0u8; 16];
    let len = answer_housekeeping(&info, &request, &mut out)
        .unwrap()
        .unwrap();
    assert_eq!(&out[..5], [0x2E, 0x33, 0x12, 0x34, 0x02]);
    assert_eq!(len, 10);
}

#[test]
fn answer_housekeeping_declines_every_other_command() {
    let info = HostInfo::new();
    for code in 0..=255u8 {
        let mut out = [0u8; 16];
        let command = Command::from_code(code);
        let request = Request {
            command,
            address: 0,
            params: &[0],
        };
        let answer = answer_housekeeping(&info, &request, &mut out);
        if matches!(code, 0x31..=0x33) {
            assert!(answer.is_some(), "{code:#04x}");
        } else {
            // In particular TestAlive (AP:979-992 talks to the ESC) and the exit are the caller's.
            assert!(answer.is_none(), "{code:#04x}");
            assert_eq!(out, [0u8; 16], "{code:#04x} wrote");
        }
    }
}

#[test]
fn answer_housekeeping_reports_a_short_buffer_and_an_unusable_name() {
    let request = Request {
        command: Command::InterfaceGetName,
        address: 0,
        params: &[0],
    };
    let mut small = [0u8; 12];
    assert_eq!(
        answer_housekeeping(&HostInfo::new(), &request, &mut small),
        Some(Err(EncodeError::BufferTooSmall { needed: 13 }))
    );
    let mut out = [0u8; 300];
    let empty = HostInfo::new().with_interface_name(&[]);
    assert_eq!(
        answer_housekeeping(&empty, &request, &mut out),
        Some(Err(EncodeError::InvalidLength))
    );
    static LONG: [u8; 257] = [b'x'; 257];
    let long = HostInfo::new().with_interface_name(&LONG);
    assert_eq!(
        answer_housekeeping(&long, &request, &mut out),
        Some(Err(EncodeError::InvalidLength))
    );
    static FULL: [u8; 256] = [b'y'; 256];
    let full = HostInfo::new().with_interface_name(&FULL);
    assert_eq!(
        answer_housekeeping(&full, &request, &mut out),
        Some(Ok(264))
    );
    // A count of 256 is sent as 0.
    assert_eq!(out[4], 0);
}

#[test]
fn encode_success_echoes_the_command_and_address_with_acknowledge_ok() {
    let init = Request {
        command: Command::DeviceInitFlash,
        address: 0,
        params: &[0],
    };
    let mut out = [0u8; 16];
    let len = encode_success(&init, &[0x06, 0x1F, 0x14, 0x04], &mut out).unwrap();
    assert_eq!(&out[..len], &FW_INIT_REPLY);
    let read = Request {
        command: Command::DeviceRead,
        address: 0x7C00,
        params: &[4],
    };
    let len = encode_success(&read, &[1, 2, 3, 4], &mut out).unwrap();
    assert_eq!(&out[..len], &FW_READ_REPLY);
    let exit = Request {
        command: Command::InterfaceExit,
        address: 0,
        params: &[0],
    };
    let len = encode_success(&exit, &[0], &mut out).unwrap();
    assert_eq!(&out[..len], &FW_EXIT_REPLY);
    assert_eq!(
        encode_success(&exit, &[], &mut out),
        Err(EncodeError::InvalidLength)
    );
    let mut small = [0u8; 8];
    assert_eq!(
        encode_success(&exit, &[0], &mut small),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
}

#[test]
fn encode_failure_echoes_the_command_address_and_first_parameter() {
    let init = Request {
        command: Command::DeviceInitFlash,
        address: 0,
        params: &[5],
    };
    let mut out = [0u8; 16];
    let len = encode_failure(&init, Ack::IInvalidChannel, &mut out).unwrap();
    assert_eq!(&out[..len], &FW_FAIL_INIT_5_REPLY);
    // A 256-byte request echoes its first byte only.
    let params = [7u8; 256];
    let write = Request {
        command: Command::DeviceWrite,
        address: 0x7C00,
        params: &params,
    };
    let len = encode_failure(&write, Ack::DGeneralError, &mut out).unwrap();
    assert_eq!(len, 9);
    assert_eq!(&out[..7], [0x2E, 0x3B, 0x7C, 0x00, 0x01, 0x07, 0x0F]);
    let crc = ref_crc_xmodem(&out[..7]).to_be_bytes();
    assert_eq!(&out[7..9], crc);
    let mut small = [0u8; 8];
    assert_eq!(
        encode_failure(&init, Ack::IInvalidChannel, &mut small),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
}

#[test]
fn every_event_and_error_variant_is_produced() {
    // Reply, ReplyFailed, PassthroughRequested, FourWay and Restarted, DecisionError::NotRequested
    // and DecisionError::Encode are produced by the tests above; this one checks them together.
    let info = conversation_info();
    let mut host = HostSide::new();
    let mut out = [0u8; 32];
    let mut small = [0u8; 2];
    assert!(matches!(
        host.feed(&MSP_API_REQ, &info, &mut out).1,
        Some(Event::Reply { .. })
    ));
    assert!(matches!(
        host.feed(&MSP_API_REQ, &info, &mut small).1,
        Some(Event::ReplyFailed(_))
    ));
    assert_eq!(host.grant(1, &mut out), Err(DecisionError::NotRequested));
    assert!(matches!(
        host.feed(&MSP_PASSTHROUGH_REQ, &info, &mut out).1,
        Some(Event::PassthroughRequested)
    ));
    assert!(matches!(
        host.grant(1, &mut small),
        Err(DecisionError::Encode(_))
    ));
    host.grant(1, &mut out).unwrap();
    assert!(matches!(
        host.feed(&FW_INIT_REQ, &info, &mut out).1,
        Some(Event::FourWay(_))
    ));
    assert!(matches!(
        host.feed(&MSP_API_REQ, &info, &mut out).1,
        Some(Event::Restarted(_))
    ));
}

// ---------------------------------------------------------------- properties

fn marker_byte() -> impl Strategy<Value = u8> {
    prop_oneof![
        4 => any::<u8>(),
        2 => Just(b'$'),
        1 => Just(b'M'),
        1 => Just(b'<'),
        2 => Just(0x2Fu8),
        1 => Just(0u8),
        1 => Just(1u8),
    ]
}

/// A four-way request laid out by hand from `four_way.ts:144-175`.
fn four_way_frame(command: u8, address: u16, params: &[u8]) -> Vec<u8> {
    let mut frame = vec![0x2F, command, (address >> 8) as u8, address as u8];
    frame.push(if params.len() == 256 {
        0
    } else {
        params.len() as u8
    });
    frame.extend_from_slice(params);
    let crc = ref_crc_xmodem(&frame);
    frame.push((crc >> 8) as u8);
    frame.push(crc as u8);
    frame
}

/// An MSP request `24 4D 3C length command payload checksum`.
fn msp_frame(command: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0x24, 0x4D, 0x3C, payload.len() as u8, command];
    frame.extend_from_slice(payload);
    let checksum = frame[3..].iter().fold(0, |acc, b| acc ^ b);
    frame.push(checksum);
    frame
}

#[derive(Debug, Clone)]
enum MspPiece {
    /// A valid request for one of 1, 2, 130, 131, 245 or another command.
    Request(u8),
    /// A request whose checksum is wrong.
    BadChecksum(u8),
    /// Bytes without a `$`.
    Noise(Vec<u8>),
}

fn msp_piece() -> impl Strategy<Value = MspPiece> {
    prop_oneof![
        4 => prop_oneof![Just(1u8), Just(2), Just(130), Just(131), Just(245), any::<u8>()]
            .prop_map(MspPiece::Request),
        1 => any::<u8>().prop_map(MspPiece::BadChecksum),
        2 => prop::collection::vec(any::<u8>().prop_filter("no $", |b| *b != 0x24), 0..8)
            .prop_map(MspPiece::Noise),
    ]
}

proptest! {
    // The MSP answers, against an oracle that lays the replies out by hand, for every split.
    #[test]
    fn msp_mode_replies_match_the_oracle_at_every_split(
        pieces in prop::collection::vec(msp_piece(), 0..12),
        cuts in prop::collection::vec(any::<usize>(), 0..10),
    ) {
        let mut stream = Vec::new();
        let mut expected_items = Vec::new();
        let mut expected_at = Vec::new();
        for piece in &pieces {
            match piece {
                MspPiece::Request(command) => {
                    stream.extend(msp_frame(*command, &[]));
                    match command {
                        1 => expected_items.push(Item::Reply(MSP_API_REPLY_1_46.to_vec())),
                        2 => expected_items.push(Item::Reply(MSP_VARIANT_REPLY.to_vec())),
                        130 => expected_items.push(Item::Reply(MSP_BATTERY_REPLY.to_vec())),
                        131 => expected_items.push(Item::Reply(MSP_MOTOR_REPLY.to_vec())),
                        245 => {
                            expected_items.push(Item::Passthrough);
                            expected_items.push(Item::Refused(MSP_ERROR_245.to_vec()));
                        }
                        other => expected_items
                            .push(Item::Reply(vec![0x24, 0x4D, 0x21, 0x00, *other, *other])),
                    }
                    expected_at.push(stream.len());
                }
                MspPiece::BadChecksum(command) => {
                    let mut frame = msp_frame(*command, &[]);
                    let last = frame.len() - 1;
                    frame[last] ^= 0x40;
                    stream.extend(frame);
                }
                MspPiece::Noise(bytes) => stream.extend_from_slice(bytes),
            }
        }
        let mut caller = Caller::new(conversation_info(), 32);
        caller.refusals = usize::MAX;
        caller.feed_chunks(&stream, &cuts);
        prop_assert_eq!(caller.items, expected_items);
        prop_assert_eq!(caller.at, expected_at);
        prop_assert_eq!(caller.host.mode(), Mode::Msp);
    }
}

#[derive(Debug, Clone)]
enum FourWayPiece {
    Request(u8, u16, Vec<u8>),
    /// A request whose high CRC byte is changed by the mask.
    BadCrc(u8, u16, Vec<u8>, u8),
    /// A valid MSP request with a command that is not `0x2F`.
    MspRestart(u8),
    /// Bytes other than `$` and 0x2F.
    Noise(Vec<u8>),
}

fn four_way_piece() -> impl Strategy<Value = FourWayPiece> {
    let params = || prop::collection::vec(marker_byte(), 1..=256);
    prop_oneof![
        4 => (any::<u8>(), any::<u16>(), params())
            .prop_map(|(c, a, p)| FourWayPiece::Request(c, a, p)),
        2 => (any::<u8>(), any::<u16>(), params(), 1..=255u8)
            .prop_map(|(c, a, p, m)| FourWayPiece::BadCrc(c, a, p, m)),
        1 => any::<u8>().prop_filter("not 0x2F", |c| *c != 0x2F).prop_map(FourWayPiece::MspRestart),
        2 => prop::collection::vec(
            any::<u8>().prop_filter("no $ or 0x2F", |b| *b != 0x24 && *b != 0x2F), 0..8
        ).prop_map(FourWayPiece::Noise),
    ]
}

proptest! {
    // Four-way mode against an oracle: valid requests are handed out whole, bad ones are answered
    // with the invalid-CRC reply laid out by hand and not handed out, a valid MSP request between
    // frames is a restart, whatever the split, and each event comes at the last byte of its frame.
    #[test]
    fn four_way_mode_events_match_the_oracle_at_every_split(
        pieces in prop::collection::vec(four_way_piece(), 0..8),
        cuts in prop::collection::vec(any::<usize>(), 0..10),
    ) {
        let mut stream = Vec::new();
        let mut expected_items = Vec::new();
        let mut expected_at = Vec::new();
        for piece in &pieces {
            match piece {
                FourWayPiece::Request(command, address, params) => {
                    stream.extend(four_way_frame(*command, *address, params));
                    expected_items.push(Item::FourWay(
                        Command::from_code(*command), *address, params.clone()));
                    expected_at.push(stream.len());
                }
                FourWayPiece::BadCrc(command, address, params, mask) => {
                    let mut frame = four_way_frame(*command, *address, params);
                    let high = frame.len() - 2;
                    frame[high] ^= mask;
                    // A bad frame that ends in 0x2F starts the next frame at that byte; the
                    // oracle does not model that here (see the test of that name).
                    prop_assume!(frame[frame.len() - 1] != 0x2F);
                    stream.extend(&frame);
                    // Reply: command, address, one parameter (the first), acknowledge 03, CRC.
                    let mut reply = vec![
                        0x2E, *command, (*address >> 8) as u8, *address as u8, 1, params[0], 0x03,
                    ];
                    let crc = ref_crc_xmodem(&reply);
                    reply.push((crc >> 8) as u8);
                    reply.push(crc as u8);
                    expected_items.push(Item::Reply(reply));
                    expected_at.push(stream.len());
                }
                FourWayPiece::MspRestart(command) => {
                    stream.extend(msp_frame(*command, &[]));
                    expected_items.push(Item::Restarted(*command, vec![]));
                    expected_at.push(stream.len());
                }
                FourWayPiece::Noise(bytes) => stream.extend_from_slice(bytes),
            }
        }
        let mut caller = four_way_caller();
        caller.answer = false;
        caller.exit_on_restart = false;
        caller.feed_chunks(&stream, &cuts);
        prop_assert_eq!(caller.items, expected_items);
        prop_assert_eq!(caller.at, expected_at);
        prop_assert_eq!(caller.host.mode(), Mode::FourWay);
    }

    // Arbitrary bytes, in both modes, with output buffers from nothing to roomy, never panic;
    // every call uses at most the slice it was given and stops with an event or the end; a reply
    // never comes before at least a whole minimal request has been fed since the last event.
    #[test]
    fn arbitrary_bytes_never_panic_and_no_reply_comes_early(
        chunks in prop::collection::vec(prop::collection::vec(marker_byte(), 0..80), 0..12),
        out_len in 0..24usize,
        start_in_four_way in any::<bool>(),
        refusals in 0..3usize,
    ) {
        let mut caller = Caller::new(conversation_info(), out_len);
        caller.refusals = refusals;
        caller.answer = false;
        caller.exit_on_restart = false;
        if start_in_four_way {
            caller.host.feed(&MSP_PASSTHROUGH_REQ, &caller.info, &mut [0u8; 8]);
            caller.host.grant(4, &mut [0u8; 8]).unwrap();
        }
        let mut last_event_at = 0;
        for chunk in &chunks {
            let mut rest = chunk.as_slice();
            while !rest.is_empty() {
                let before_items = caller.items.len();
                let before_fed = caller.fed;
                // One byte per call: the position of every event is exact.
                caller.feed(&rest[..1]);
                rest = &rest[1..];
                prop_assert_eq!(caller.fed, before_fed + 1);
                if caller.items.len() > before_items {
                    let since = caller.fed - last_event_at;
                    for item in &caller.items[before_items..] {
                        // The shortest request is 6 bytes (MSP); a reply is never written
                        // before that many bytes arrived since the previous event.
                        if matches!(item, Item::Reply(_) | Item::ReplyFailed(_)) {
                            prop_assert!(since >= 6, "reply after {since} bytes");
                        }
                    }
                    last_event_at = caller.fed;
                }
            }
        }
        // And in whole chunks the result is the same.
        let mut again = Caller::new(conversation_info(), out_len);
        again.refusals = refusals;
        again.answer = false;
        again.exit_on_restart = false;
        if start_in_four_way {
            again.host.feed(&MSP_PASSTHROUGH_REQ, &again.info, &mut [0u8; 8]);
            again.host.grant(4, &mut [0u8; 8]).unwrap();
        }
        for chunk in &chunks {
            again.feed(chunk);
        }
        prop_assert_eq!(again.items, caller.items);
    }

    // Splits of the whole connect conversation, at random, give the same events and replies.
    #[test]
    fn random_splits_of_the_conversation_do_not_change_it(
        cuts in prop::collection::vec(any::<usize>(), 0..20),
    ) {
        let stream = conversation_bytes();
        let mut caller = Caller::new(conversation_info(), 64);
        caller.feed_chunks(&stream, &cuts);
        prop_assert_eq!(caller.items, conversation_items());
        prop_assert_eq!(caller.at, CONVERSATION_ENDS);
    }
}

#[test]
fn feed_hands_out_a_command_that_needs_an_esc_like_any_other_request() {
    // TestAlive is handed out like any request: the host side has no ESC to ask.
    let mut caller = four_way_caller();
    caller.feed(&FW_ALIVE_REQ);
    assert_eq!(
        caller.items[0],
        Item::FourWay(Command::InterfaceTestAlive, 0, vec![0])
    );
}
