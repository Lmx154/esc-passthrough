//! Host tests for `esc_passthrough::msp`.
//!
//! Sources of the golden vectors (CFG = `_plan/ref/am32-configurator`, SIM = `CFG/ci/
//! bootloader_sim.py`, AP = `ardupilot/libraries/AP_BLHeli/AP_BLHeli.cpp`, LIB = the npm package
//! `@am32/serial-msp` 0.1.0 read from unpkg, not on disk):
//! - the five requests are what the configurator sends: `encodeMspV1` (LIB dist/msp/codec.js:
//!   19-34: `$M<`, length, command, payload, XOR of length, command and payload), called with
//!   no payload from CFG components/SerialDevice.vue:618 (API_VERSION), 639 (FC_VARIANT), 644
//!   (BATTERY_STATE), 649 (MOTOR_CONFIG for every variant but inav/emuf), 655 (SET_PASSTHROUGH);
//! - reply layouts: AP:370-376 (API version), 377-380 (variant), 478-510 (battery), 512-526
//!   (motor config), 572-600 (passthrough), SIM:508-525; what the configurator reads from them:
//!   CFG src/communication/commands.queue.ts:57-62, 64-86, 89-98, 115-119, 99-104;
//! - the error reply `$M!` length 0, command, checksum: SIM:523; the configurator's parser
//!   accepts the `!` direction: LIB dist/msp/parser.js:28.
//!
//! No captured traffic exists in the sources; the bytes were laid out by hand from the formulas
//! above and their checksums computed with a separate script.

mod common;

use esc_passthrough::EncodeError;
use esc_passthrough::msp::{
    ApiVersion, BatteryState, Command, MAX_FRAME_LEN, MAX_PAYLOAD, MSP_API_VERSION,
    MSP_BATTERY_STATE, MSP_FC_VARIANT, MSP_MOTOR_CONFIG, MSP_SET_PASSTHROUGH, MotorConfig,
    ParseError, Request, RequestParser, encode_api_version, encode_battery_state, encode_error,
    encode_fc_variant, encode_motor_config, encode_reply, encode_set_passthrough,
};
use proptest::prelude::*;

#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    Request(u8, Vec<u8>),
    Error(ParseError),
}

fn own(result: Result<Request<'_>, ParseError>) -> Event {
    match result {
        Ok(request) => Event::Request(request.command, request.payload.to_vec()),
        Err(error) => Event::Error(error),
    }
}

fn run_by_byte(bytes: &[u8]) -> Vec<Event> {
    let mut parser = RequestParser::new();
    let mut events = Vec::new();
    for &byte in bytes {
        if let Some(result) = parser.push(byte) {
            events.push(own(result));
        }
    }
    events
}

fn run_chunks(bytes: &[u8], cuts: &[usize]) -> Vec<Event> {
    let mut points: Vec<usize> = cuts.iter().map(|c| c % (bytes.len() + 1)).collect();
    points.push(0);
    points.push(bytes.len());
    points.sort_unstable();
    let mut parser = RequestParser::new();
    let mut events = Vec::new();
    for pair in points.windows(2) {
        let mut rest = &bytes[pair[0]..pair[1]];
        while !rest.is_empty() {
            let (used, event) = parser.feed(rest);
            if let Some(result) = event {
                events.push(own(result));
            }
            rest = &rest[used..];
        }
    }
    events
}

/// A request as the configurator builds it (LIB codec.js:19-34).
fn request_bytes(command: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![b'$', b'M', b'<', payload.len() as u8, command];
    frame.extend_from_slice(payload);
    let checksum = frame[3..].iter().fold(0, |acc, b| acc ^ b);
    frame.push(checksum);
    frame
}

/// An independent whole-buffer parser with the documented resynchronisation rules.
fn reference_events(stream: &[u8]) -> Vec<Event> {
    let mut events = Vec::new();
    let mut i = 0;
    while i < stream.len() {
        if stream[i] != b'$' {
            i += 1;
            continue;
        }
        let Some(&second) = stream.get(i + 1) else {
            break;
        };
        if second != b'M' {
            events.push(Event::Error(ParseError::BadHeader));
            i += 1;
            continue;
        }
        let Some(&third) = stream.get(i + 2) else {
            break;
        };
        if third != b'<' {
            events.push(Event::Error(ParseError::BadHeader));
            i += 2;
            continue;
        }
        let Some(&length) = stream.get(i + 3) else {
            break;
        };
        let checksum_at = i + 5 + usize::from(length);
        let Some(&received) = stream.get(checksum_at) else {
            break;
        };
        let expected = stream[i + 3..checksum_at].iter().fold(0, |acc, b| acc ^ b);
        if expected == received {
            events.push(Event::Request(
                stream[i + 4],
                stream[i + 5..checksum_at].to_vec(),
            ));
            i = checksum_at + 1;
        } else {
            events.push(Event::Error(ParseError::BadChecksum { expected, received }));
            i = if received == b'$' {
                checksum_at
            } else {
                checksum_at + 1
            };
        }
    }
    events
}

/// Checks a reply frame from first principles and returns (direction, command, payload).
fn decode_frame(frame: &[u8]) -> (u8, u8, Vec<u8>) {
    assert_eq!(&frame[..2], b"$M");
    let length = usize::from(frame[3]);
    assert_eq!(frame.len(), length + 6);
    let checksum = frame[3..frame.len() - 1].iter().fold(0, |acc, b| acc ^ b);
    assert_eq!(frame[frame.len() - 1], checksum);
    (frame[2], frame[4], frame[5..5 + length].to_vec())
}

fn encoded(f: impl FnOnce(&mut [u8]) -> Result<usize, EncodeError>) -> Vec<u8> {
    let mut out = [0xAAu8; 300];
    let n = f(&mut out).unwrap();
    out[..n].to_vec()
}

// ---------------------------------------------------------------- Command, Request

#[test]
fn from_code_sorts_the_five_supported_commands_and_the_rest() {
    assert_eq!(Command::from_code(1), Command::ApiVersion);
    assert_eq!(Command::from_code(2), Command::FcVariant);
    assert_eq!(Command::from_code(130), Command::BatteryState);
    assert_eq!(Command::from_code(131), Command::MotorConfig);
    assert_eq!(Command::from_code(245), Command::SetPassthrough);
    assert_eq!(Command::from_code(3), Command::Unsupported(3));
    assert_eq!(Command::from_code(104), Command::Unsupported(104));
    assert_eq!(Command::from_code(0), Command::Unsupported(0));
    assert_eq!(Command::from_code(255), Command::Unsupported(255));
}

#[test]
fn code_inverts_from_code_for_every_byte() {
    for code in 0..=255u8 {
        assert_eq!(Command::from_code(code).code(), code);
    }
    assert_eq!(Command::ApiVersion.code(), MSP_API_VERSION);
    assert_eq!(Command::FcVariant.code(), MSP_FC_VARIANT);
    assert_eq!(Command::BatteryState.code(), MSP_BATTERY_STATE);
    assert_eq!(Command::MotorConfig.code(), MSP_MOTOR_CONFIG);
    assert_eq!(Command::SetPassthrough.code(), MSP_SET_PASSTHROUGH);
}

#[test]
fn kind_reports_supported_and_unsupported_requests() {
    let mut parser = RequestParser::new();
    let mut bytes = request_bytes(131, &[]);
    bytes.extend(request_bytes(104, &[1, 2]));
    let mut kinds = Vec::new();
    for byte in bytes {
        if let Some(Ok(request)) = parser.push(byte) {
            kinds.push(request.kind());
        }
    }
    assert_eq!(kinds, [Command::MotorConfig, Command::Unsupported(104)]);
}

// ---------------------------------------------------------------- parser

#[test]
fn new_parser_waits_for_a_dollar_and_yields_nothing_for_noise() {
    let mut parser = RequestParser::new();
    for byte in [0u8, 0x2F, b'M', b'<', 0xFF, b'x'] {
        assert!(parser.push(byte).is_none());
    }
}

#[test]
fn default_request_parser_behaves_like_new() {
    let mut parser = RequestParser::default();
    let events: Vec<_> = request_bytes(1, &[])
        .into_iter()
        .filter_map(|b| parser.push(b).map(own))
        .collect();
    assert_eq!(events, [Event::Request(1, vec![])]);
}

#[test]
fn push_parses_the_five_requests_as_the_configurator_sends_them() {
    // LIB codec.js:19-34 with no payload; SerialDevice.vue:618, 639, 644, 649, 655.
    let golden: [(&[u8], u8); 5] = [
        (&[0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01], 1),
        (&[0x24, 0x4D, 0x3C, 0x00, 0x02, 0x02], 2),
        (&[0x24, 0x4D, 0x3C, 0x00, 0x82, 0x82], 130),
        (&[0x24, 0x4D, 0x3C, 0x00, 0x83, 0x83], 131),
        (&[0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5], 245),
    ];
    for (bytes, command) in golden {
        assert_eq!(request_bytes(command, &[]), bytes);
        let mut parser = RequestParser::new();
        let events: Vec<_> = bytes
            .iter()
            .filter_map(|&b| parser.push(b).map(own))
            .collect();
        assert_eq!(events, [Event::Request(command, vec![])]);
    }
}

#[test]
fn push_returns_the_payload_of_a_request_and_reports_other_commands() {
    let frame = request_bytes(214, &[0xE8, 0x03, 0xE8, 0x03]);
    assert_eq!(
        run_by_byte(&frame),
        [Event::Request(214, vec![0xE8, 0x03, 0xE8, 0x03])]
    );
}

#[test]
fn push_accepts_a_dollar_inside_the_payload() {
    let frame = request_bytes(7, b"$M<\x00\x01\x01");
    assert_eq!(
        run_by_byte(&frame),
        [Event::Request(7, b"$M<\x00\x01\x01".to_vec())]
    );
}

#[test]
fn push_accepts_the_largest_payload() {
    let payload = vec![0x5A; MAX_PAYLOAD];
    let frame = request_bytes(9, &payload);
    assert_eq!(frame.len(), MAX_FRAME_LEN);
    assert_eq!(run_by_byte(&frame), [Event::Request(9, payload)]);
}

#[test]
fn push_reports_bad_checksum_with_both_values() {
    let mut frame = request_bytes(245, &[]);
    *frame.last_mut().unwrap() ^= 0x01;
    assert_eq!(
        run_by_byte(&frame),
        [Event::Error(ParseError::BadChecksum {
            expected: 0xF5,
            received: 0xF4
        })]
    );
}

#[test]
fn push_reports_bad_header_for_replies_error_replies_and_v2_frames() {
    for direction in *b">!" {
        let mut frame = request_bytes(1, &[]);
        frame[2] = direction;
        assert_eq!(run_by_byte(&frame), [Event::Error(ParseError::BadHeader)]);
    }
    // MSP v2 request: `$X<`.
    let v2 = [b'$', b'X', b'<', 0, 1, 0, 0, 0, 0x4A];
    assert_eq!(run_by_byte(&v2), [Event::Error(ParseError::BadHeader)]);
    // A lone dollar followed by noise.
    assert_eq!(run_by_byte(b"$x"), [Event::Error(ParseError::BadHeader)]);
}

#[test]
fn push_resynchronises_on_a_dollar_that_breaks_a_header() {
    // `$$M<...`: the second `$` starts the frame.
    let mut stream = vec![b'$'];
    stream.extend(request_bytes(2, &[]));
    assert_eq!(
        run_by_byte(&stream),
        [
            Event::Error(ParseError::BadHeader),
            Event::Request(2, vec![])
        ]
    );
    // `$M$M<...`.
    let mut stream = vec![b'$', b'M'];
    stream.extend(request_bytes(2, &[]));
    assert_eq!(
        run_by_byte(&stream),
        [
            Event::Error(ParseError::BadHeader),
            Event::Request(2, vec![])
        ]
    );
}

#[test]
fn push_resynchronises_on_a_checksum_byte_that_is_a_dollar() {
    // A frame whose checksum byte is wrong and equals `$`, then a full frame.
    let mut bad = request_bytes(2, &[]);
    *bad.last_mut().unwrap() = b'$';
    let mut stream = bad;
    stream.extend_from_slice(b"M<");
    stream.extend_from_slice(&[0, 1, 1]);
    let events = run_by_byte(&stream);
    assert_eq!(events.len(), 2);
    assert!(matches!(
        events[0],
        Event::Error(ParseError::BadChecksum { .. })
    ));
    assert_eq!(events[1], Event::Request(1, vec![]));
}

#[test]
fn push_skips_noise_between_frames() {
    let mut stream = vec![0x00, 0xFF, b'M', b'<', 0x2F];
    stream.extend(request_bytes(130, &[]));
    stream.extend([1, 2, 3]);
    stream.extend(request_bytes(131, &[]));
    assert_eq!(
        run_by_byte(&stream),
        [Event::Request(130, vec![]), Event::Request(131, vec![])]
    );
}

#[test]
fn feed_reports_consumed_bytes_and_resumes_in_the_next_call() {
    let mut stream = request_bytes(1, &[]);
    stream.extend(request_bytes(2, &[]));
    let mut parser = RequestParser::new();
    // Split in the middle of the second frame, after the first event.
    let (used, event) = parser.feed(&stream[..9]);
    assert_eq!(used, 6);
    assert_eq!(event.map(own), Some(Event::Request(1, vec![])));
    let (used, event) = parser.feed(&stream[6..9]);
    assert_eq!((used, event.is_none()), (3, true));
    let (used, event) = parser.feed(&stream[9..]);
    assert_eq!(used, 3);
    assert_eq!(event.map(own), Some(Event::Request(2, vec![])));
    let (used, event) = parser.feed(&[]);
    assert_eq!((used, event.is_none()), (0, true));
}

#[test]
fn reset_drops_a_partial_frame() {
    let frame = request_bytes(130, &[]);
    let mut parser = RequestParser::new();
    for &byte in &frame[..4] {
        assert!(parser.push(byte).is_none());
    }
    parser.reset();
    // Without the reset, these bytes would finish the old frame.
    for &byte in &frame[4..] {
        assert!(parser.push(byte).is_none());
    }
    let events: Vec<_> = frame
        .iter()
        .filter_map(|&b| parser.push(b).map(own))
        .collect();
    assert_eq!(events, [Event::Request(130, vec![])]);
}

// ---------------------------------------------------------------- encoders

#[test]
fn encode_reply_writes_the_frame_and_rejects_bad_arguments() {
    assert_eq!(
        encoded(|out| encode_reply(0x55, &[1, 2, 3], out)),
        [
            0x24,
            0x4D,
            0x3E,
            0x03,
            0x55,
            1,
            2,
            3,
            0x03 ^ 0x55 ^ 1 ^ 2 ^ 3
        ]
    );
    assert_eq!(
        encoded(|out| encode_reply(7, &[], out)),
        [0x24, 0x4D, 0x3E, 0, 7, 7]
    );
    let big = [0xC3; MAX_PAYLOAD];
    assert_eq!(
        encoded(|out| encode_reply(7, &big, out)).len(),
        MAX_FRAME_LEN
    );
    let too_big = [0u8; MAX_PAYLOAD + 1];
    let mut out = [0u8; 400];
    assert_eq!(
        encode_reply(7, &too_big, &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert!(out.iter().all(|&b| b == 0));
}

#[test]
fn encode_reply_reports_a_short_buffer_and_writes_nothing() {
    let mut out = [0xAAu8; 8];
    assert_eq!(
        encode_reply(7, &[1, 2, 3], &mut out),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
    assert_eq!(out, [0xAA; 8]);
    let mut exact = [0u8; 9];
    assert_eq!(encode_reply(7, &[1, 2, 3], &mut exact), Ok(9));
    assert_eq!(
        encode_reply(7, &[1, 2, 3], &mut []),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
}

#[test]
fn encode_error_writes_the_msp_error_reply() {
    // SIM:523: b'$M!' + bytes([0, cmd, cmd]).
    assert_eq!(
        encoded(|out| encode_error(245, out)),
        [0x24, 0x4D, 0x21, 0x00, 0xF5, 0xF5]
    );
    let mut small = [0u8; 5];
    assert_eq!(
        encode_error(245, &mut small),
        Err(EncodeError::BufferTooSmall { needed: 6 })
    );
}

#[test]
fn new_api_version_holds_the_triple() {
    let version = ApiVersion::new(0, 1, 46);
    assert_eq!((version.protocol, version.major, version.minor), (0, 1, 46));
}

#[test]
fn default_api_version_is_all_zero() {
    assert_eq!(ApiVersion::default(), ApiVersion::new(0, 0, 0));
}

#[test]
fn encode_api_version_writes_protocol_major_minor() {
    // SIM:510 sends (0, 1, 46); AP:372 sends the same three fields in this order.
    assert_eq!(
        encoded(|out| encode_api_version(ApiVersion::new(0, 1, 46), out)),
        [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2E, 0x2D]
    );
    assert_eq!(
        encoded(|out| encode_api_version(ApiVersion::new(0, 1, 42), out)),
        [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2A, 0x29]
    );
    let mut small = [0u8; 8];
    assert_eq!(
        encode_api_version(ApiVersion::default(), &mut small),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
}

#[test]
fn encode_fc_variant_writes_four_ascii_bytes() {
    // AP:377-380 (ARDUPILOT_IDENTIFIER, 4 bytes); the configurator compares the text with
    // 'BTFL', 'KISS', 'INAV', 'ARDU', 'EMUF' (commands.queue.ts:64-86).
    assert_eq!(
        encoded(|out| encode_fc_variant(*b"ARDU", out)),
        [0x24, 0x4D, 0x3E, 0x04, 0x02, 0x41, 0x52, 0x44, 0x55, 0x04]
    );
    let mut small = [0u8; 9];
    assert_eq!(
        encode_fc_variant(*b"ARDU", &mut small),
        Err(EncodeError::BufferTooSmall { needed: 10 })
    );
}

#[test]
fn new_battery_state_has_no_current_and_no_charge() {
    let state = BatteryState::new(25_200);
    assert_eq!(
        (state.millivolts, state.current_ma, state.charge_mah),
        (25_200, None, None)
    );
}

#[test]
fn with_current_ma_sets_the_current() {
    assert_eq!(
        BatteryState::new(1).with_current_ma(Some(7)).current_ma,
        Some(7)
    );
    assert_eq!(
        BatteryState::new(1)
            .with_current_ma(Some(7))
            .with_current_ma(None)
            .current_ma,
        None
    );
}

#[test]
fn with_charge_mah_sets_the_charge() {
    assert_eq!(
        BatteryState::new(1).with_charge_mah(Some(9)).charge_mah,
        Some(9)
    );
}

#[test]
fn default_battery_state_equals_new_zero() {
    assert_eq!(BatteryState::default(), BatteryState::new(0));
}

#[test]
fn encode_battery_state_writes_the_eleven_byte_layout() {
    // AP:478-510 layout; the configurator reads voltage x10 at byte 3 (commands.queue.ts:89-98).
    // 25.2 V, 30 A, 1234 mAh.
    let state = BatteryState::new(25_200)
        .with_current_ma(Some(30_000))
        .with_charge_mah(Some(1234));
    assert_eq!(
        encoded(|out| encode_battery_state(state, out)),
        [
            0x24, 0x4D, 0x3E, 0x0B, 0x82, //
            0x00, 0x00, 0x00, 0xFC, 0xD2, 0x04, 0xB8, 0x0B, 0x00, 0xD8, 0x09, //
            0xC1
        ]
    );
    // No current, no charge.
    assert_eq!(
        encoded(|out| encode_battery_state(BatteryState::new(0), out)),
        [
            0x24, 0x4D, 0x3E, 0x0B, 0x82, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x89
        ]
    );
    let mut small = [0u8; 16];
    assert_eq!(
        encode_battery_state(state, &mut small),
        Err(EncodeError::BufferTooSmall { needed: 17 })
    );
}

#[test]
fn encode_battery_state_rounds_and_saturates() {
    let payload = |state| decode_frame(&encoded(|out| encode_battery_state(state, out))).2;
    // 12.649 V rounds to 12.6 V (126) and 1265 (12.65 V); 12.650 V rounds to 12.7 V (127).
    assert_eq!(payload(BatteryState::new(12_649))[3], 126);
    assert_eq!(payload(BatteryState::new(12_650))[3], 127);
    assert_eq!(
        payload(BatteryState::new(12_649))[9..],
        1265u16.to_le_bytes()
    );
    // 34 V does not fit the byte at 3 (25.5 V) and saturates; the 0.01 V field holds it.
    assert_eq!(payload(BatteryState::new(34_000))[3], 255);
    assert_eq!(
        payload(BatteryState::new(34_000))[9..],
        3400u16.to_le_bytes()
    );
    // Everything saturates instead of wrapping.
    let all = BatteryState::new(u32::MAX)
        .with_current_ma(Some(u32::MAX))
        .with_charge_mah(Some(u32::MAX));
    assert_eq!(
        payload(all),
        [0, 0, 0, 255, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0xFF, 0xFF]
    );
    // 1 A is 100 in 0.01 A; 4 mA rounds down, 5 mA up.
    assert_eq!(
        payload(BatteryState::new(0).with_current_ma(Some(1000)))[6..8],
        100u16.to_le_bytes()
    );
    assert_eq!(
        payload(BatteryState::new(0).with_current_ma(Some(4)))[6..8],
        0u16.to_le_bytes()
    );
    assert_eq!(
        payload(BatteryState::new(0).with_current_ma(Some(5)))[6..8],
        1u16.to_le_bytes()
    );
}

#[test]
fn new_motor_config_holds_the_count() {
    assert_eq!(MotorConfig::new(4).motor_count, 4);
}

#[test]
fn default_motor_config_has_no_motors() {
    assert_eq!(MotorConfig::default(), MotorConfig::new(0));
}

#[test]
fn encode_motor_config_puts_the_count_at_byte_six() {
    // AP:512-526 and SIM:515-517 (`<HHHBBBB`): byte 6 of the payload is the motor count
    // (commands.queue.ts:115-119). The other nine bytes are 0.
    assert_eq!(
        encoded(|out| encode_motor_config(MotorConfig::new(4), out)),
        [
            0x24, 0x4D, 0x3E, 0x0A, 0x83, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0x8D
        ]
    );
    let mut small = [0u8; 15];
    assert_eq!(
        encode_motor_config(MotorConfig::new(4), &mut small),
        Err(EncodeError::BufferTooSmall { needed: 16 })
    );
}

#[test]
fn encode_set_passthrough_writes_the_esc_count() {
    // AP:572-600 replies with the motor count as the only byte; the configurator reads byte 0
    // (commands.queue.ts:99-104).
    assert_eq!(
        encoded(|out| encode_set_passthrough(4, out)),
        [0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0]
    );
    let mut small = [0u8; 6];
    assert_eq!(
        encode_set_passthrough(4, &mut small),
        Err(EncodeError::BufferTooSmall { needed: 7 })
    );
}

// ---------------------------------------------------------------- properties

fn payload_strategy(max: usize) -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(any::<u8>(), 0..=max)
}

proptest! {
    #[test]
    fn roundtrip_request_through_every_feeding_style(
        command in any::<u8>(),
        payload in payload_strategy(MAX_PAYLOAD),
        cuts in prop::collection::vec(any::<usize>(), 0..6),
    ) {
        let frame = request_bytes(command, &payload);
        let expected = [Event::Request(command, payload.clone())];
        prop_assert_eq!(run_by_byte(&frame), expected.clone());
        prop_assert_eq!(run_chunks(&frame, &cuts), expected.clone());
        prop_assert_eq!(reference_events(&frame), expected);
    }

    #[test]
    fn roundtrip_reply_matches_an_independent_decoder(
        command in any::<u8>(),
        payload in payload_strategy(MAX_PAYLOAD),
    ) {
        let frame = encoded(|out| encode_reply(command, &payload, out));
        let (direction, decoded_command, decoded_payload) = decode_frame(&frame);
        prop_assert_eq!(direction, b'>');
        prop_assert_eq!(decoded_command, command);
        prop_assert_eq!(decoded_payload, payload);
        let error = encoded(|out| encode_error(command, out));
        prop_assert_eq!(decode_frame(&error), (b'!', command, vec![]));
    }

    #[test]
    fn typed_replies_decode_to_their_fields(
        protocol in any::<u8>(), major in any::<u8>(), minor in any::<u8>(),
        variant in any::<[u8; 4]>(),
        millivolts in any::<u32>(),
        current in prop::option::of(any::<u32>()),
        charge in prop::option::of(any::<u32>()),
        count in any::<u8>(),
    ) {
        let api = encoded(|out| encode_api_version(ApiVersion::new(protocol, major, minor), out));
        prop_assert_eq!(decode_frame(&api), (b'>', 1, vec![protocol, major, minor]));
        let fc = encoded(|out| encode_fc_variant(variant, out));
        prop_assert_eq!(decode_frame(&fc), (b'>', 2, variant.to_vec()));
        let state = BatteryState::new(millivolts).with_current_ma(current).with_charge_mah(charge);
        let battery = decode_frame(&encoded(|out| encode_battery_state(state, out)));
        prop_assert_eq!((battery.0, battery.1, battery.2.len()), (b'>', 130, 11));
        // Within half a unit of the exact value, or saturated.
        let p = battery.2;
        let deci = f64::from(millivolts) / 100.0;
        prop_assert!((f64::from(p[3]) - deci.min(255.0)).abs() <= 0.5);
        let centi = f64::from(millivolts) / 10.0;
        prop_assert!((f64::from(u16::from_le_bytes([p[9], p[10]])) - centi.min(65535.0)).abs() <= 0.5);
        let amps = f64::from(current.unwrap_or(0)) / 10.0;
        prop_assert!((f64::from(u16::from_le_bytes([p[6], p[7]])) - amps.min(65535.0)).abs() <= 0.5);
        let drawn = f64::from(charge.unwrap_or(0)).min(65535.0);
        prop_assert_eq!(f64::from(u16::from_le_bytes([p[4], p[5]])), drawn);
        let motor = decode_frame(&encoded(|out| encode_motor_config(MotorConfig::new(count), out)));
        prop_assert_eq!((motor.1, motor.2.len(), motor.2[6]), (131, 10, count));
        let pass = decode_frame(&encoded(|out| encode_set_passthrough(count, out)));
        prop_assert_eq!(pass, (b'>', 245, vec![count]));
    }

    #[test]
    fn arbitrary_bytes_never_panic_and_match_the_reference_parser(
        stream in prop::collection::vec(common::marker_biased_byte(), 0..600),
        cuts in prop::collection::vec(any::<usize>(), 0..8),
    ) {
        let reference = reference_events(&stream);
        prop_assert_eq!(run_by_byte(&stream), reference.clone());
        // Splitting anywhere changes nothing.
        prop_assert_eq!(run_chunks(&stream, &cuts), reference.clone());
        // Every request it yields has a correct checksum over bytes that are in the stream.
        for event in &reference {
            if let Event::Request(command, payload) = event {
                let frame = request_bytes(*command, payload);
                prop_assert!(stream.windows(frame.len()).any(|w| w == frame.as_slice()));
            }
        }
    }

    #[test]
    fn frames_between_garbage_are_all_found(
        frames in prop::collection::vec((any::<u8>(), payload_strategy(20)), 1..5),
        gaps in prop::collection::vec(prop::collection::vec(any::<u8>(), 0..6), 5),
        cuts in prop::collection::vec(any::<usize>(), 0..8),
    ) {
        // Gaps without `$` cannot hide a frame start.
        let mut stream = Vec::new();
        let mut expected = Vec::new();
        for (i, (command, payload)) in frames.iter().enumerate() {
            stream.extend(gaps[i].iter().filter(|&&b| b != b'$'));
            stream.extend(request_bytes(*command, payload));
            expected.push(Event::Request(*command, payload.clone()));
        }
        prop_assert_eq!(run_chunks(&stream, &cuts), expected);
    }

    #[test]
    fn corrupting_a_covered_byte_is_rejected_and_the_next_frame_survives(
        command in any::<u8>(),
        payload in payload_strategy(60),
        position in any::<prop::sample::Index>(),
        flip in 1..=255u8,
    ) {
        let frame = request_bytes(command, &payload);
        // Command, payload and checksum bytes: XOR catches every single-byte change.
        let covered = frame.len() - 4;
        let index = 4 + position.index(covered);
        let mut corrupted = frame.clone();
        corrupted[index] ^= flip;
        corrupted.extend(request_bytes(1, &[]));
        let events = run_by_byte(&corrupted);
        prop_assert!(matches!(events[0], Event::Error(ParseError::BadChecksum { .. })), "{:?}", events);
        prop_assert_eq!(events.last(), Some(&Event::Request(1, vec![])));
        prop_assert!(events[..events.len() - 1].iter().all(|e| matches!(e, Event::Error(_))));
        prop_assert_eq!(run_chunks(&corrupted, &[3, 9]), events);
    }

    #[test]
    fn corrupting_a_marker_byte_never_yields_the_frame_and_the_next_survives(
        command in any::<u8>().prop_filter("not a marker", |c| !b"$M<".contains(c)),
        payload in prop::collection::vec(any::<u8>().prop_filter("no dollar", |b| *b != b'$'), 0..40),
        index in 0..3usize,
        value in any::<u8>(),
    ) {
        let frame = request_bytes(command, &payload);
        prop_assume!(value != frame[index]);
        let mut corrupted = frame.clone();
        corrupted[index] = value;
        corrupted.extend(request_bytes(1, &[]));
        let events = run_by_byte(&corrupted);
        prop_assert_eq!(events.last(), Some(&Event::Request(1, vec![])));
        prop_assert!(events[..events.len() - 1].iter().all(|e| matches!(e, Event::Error(_))));
    }

    #[test]
    fn corrupting_the_length_never_accepts_a_frame_with_a_bad_checksum(
        command in any::<u8>(),
        payload in payload_strategy(60),
        value in any::<u8>(),
        tail in prop::collection::vec(common::marker_biased_byte(), 0..40),
    ) {
        let mut corrupted = request_bytes(command, &payload);
        prop_assume!(value != corrupted[3]);
        corrupted[3] = value;
        corrupted.extend(&tail);
        // Whatever it accepts is checked against the reference, which recomputes the XOR from
        // the stream itself.
        prop_assert_eq!(run_by_byte(&corrupted), reference_events(&corrupted));
    }
}
