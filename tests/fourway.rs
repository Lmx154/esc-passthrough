//! Host tests for `esc_passthrough::fourway`.
//!
//! Sources of the golden vectors (CFG = `_plan/ref/am32-configurator/src/communication/
//! four_way.ts`, SIM = `_plan/ref/am32-configurator/ci/bootloader_sim.py`, HDR = `ardupilot/
//! libraries/AP_BLHeli/blheli_4way_protocol.h`, AP = `.../AP_BLHeli.cpp`):
//! - request framing `2F`, command, address high, address low, count, parameters, CRC high, CRC
//!   low, and an empty parameter list sent as one `00`: CFG:144-175; CRC16-XMODEM: CFG:177-189;
//! - reply framing `2E`, command, address, count, parameters, acknowledge, CRC: CFG:433-478,
//!   AP:608-623, SIM:549-553;
//! - command codes: CFG:86-102 and HDR:28-114 (HDR also has 0x40); acknowledge codes: CFG:104-116
//!   and HDR:158-164;
//! - InitFlash is `[target]` at address 0 (CFG:205-210) and its reply is [signature low,
//!   signature high, pin code, interface mode] (AP:1053-1086); DeviceRead is `[n]` at the wire
//!   address with 256 sent as 0 (CFG:326-337); DeviceWrite sends the data at the wire address
//!   (CFG:493-496); InterfaceExit and TestAlive send `[0]` (CFG:350, 386-389, 623); a one-byte
//!   `00` reply answers write, exit, alive and reset (AP:979-992, 1012-1029, 1134-1160).
//!
//! No captured traffic exists in the sources; the bytes were laid out by hand from the formulas
//! above and their CRCs computed with a separate script. The values 0x7C00 (an F421 settings
//! address on the wire), pin code 0x14 and signature 0x1F06 are examples.

mod common;

use common::{marker_biased_byte, ref_crc_xmodem};
use esc_passthrough::EncodeError;
use esc_passthrough::fourway::{
    Ack, Command, MAX_PARAMS, MAX_REPLY_LEN, MAX_REQUEST_LEN, ParseError, Request, RequestParser,
    crc16_xmodem, crc16_xmodem_update, encode_reply,
};
use proptest::prelude::*;

#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    Request(Command, u16, Vec<u8>),
    Error(ParseError),
}

fn own(result: Result<Request<'_>, ParseError>) -> Event {
    match result {
        Ok(request) => Event::Request(request.command, request.address, request.params.to_vec()),
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

/// A request as the configurator builds it (CFG:144-175), CRC from the bit-serial reference.
fn request_bytes(command: u8, address: u16, params: &[u8]) -> Vec<u8> {
    assert!((1..=256).contains(&params.len()));
    let mut frame = vec![0x2F, command, (address >> 8) as u8, address as u8];
    frame.push(if params.len() == 256 {
        0
    } else {
        params.len() as u8
    });
    frame.extend_from_slice(params);
    let crc = ref_crc_xmodem(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

fn reference_events(stream: &[u8]) -> Vec<Event> {
    let mut events = Vec::new();
    let mut i = 0;
    while i < stream.len() {
        if stream[i] != 0x2F {
            i += 1;
            continue;
        }
        let Some(&count) = stream.get(i + 4) else {
            break;
        };
        let n = if count == 0 { 256 } else { usize::from(count) };
        let crc_at = i + 5 + n;
        if stream.len() < crc_at + 2 {
            break;
        }
        let expected = ref_crc_xmodem(&stream[i..crc_at]);
        let received = u16::from_be_bytes([stream[crc_at], stream[crc_at + 1]]);
        if expected == received {
            events.push(Event::Request(
                Command::from_code(stream[i + 1]),
                u16::from_be_bytes([stream[i + 2], stream[i + 3]]),
                stream[i + 5..crc_at].to_vec(),
            ));
            i = crc_at + 2;
        } else {
            events.push(Event::Error(ParseError::BadCrc { expected, received }));
            i = if stream[crc_at + 1] == 0x2F {
                crc_at + 1
            } else {
                crc_at + 2
            };
        }
    }
    events
}

/// Checks a reply frame from first principles: (command, address, params, ack).
fn decode_reply(frame: &[u8]) -> (u8, u16, Vec<u8>, u8) {
    assert_eq!(frame[0], 0x2E);
    let n = if frame[4] == 0 {
        256
    } else {
        usize::from(frame[4])
    };
    assert_eq!(frame.len(), n + 8);
    let crc = ref_crc_xmodem(&frame[..frame.len() - 2]);
    assert_eq!(&frame[frame.len() - 2..], &crc.to_be_bytes());
    (
        frame[1],
        u16::from_be_bytes([frame[2], frame[3]]),
        frame[5..5 + n].to_vec(),
        frame[5 + n],
    )
}

fn encoded(
    command: Command,
    address: u16,
    params: &[u8],
    ack: Ack,
) -> Result<Vec<u8>, EncodeError> {
    let mut out = [0xAAu8; 300];
    let n = encode_reply(command, address, params, ack, &mut out)?;
    Ok(out[..n].to_vec())
}

// ---------------------------------------------------------------- CRC

#[test]
fn crc16_xmodem_matches_the_published_check_value() {
    // CRC-16/XMODEM catalogue entry: check value for "123456789".
    assert_eq!(crc16_xmodem(b"123456789"), 0x31C3);
    assert_eq!(ref_crc_xmodem(b"123456789"), 0x31C3);
    assert_eq!(crc16_xmodem(&[]), 0);
}

#[test]
fn crc16_xmodem_equals_the_bit_serial_reference() {
    for data in [&[0u8][..], &[0xFF], &[0x2F, 0x37, 0, 0, 1, 0], &[0xA5; 300]] {
        assert_eq!(crc16_xmodem(data), ref_crc_xmodem(data));
    }
    // The golden InitFlash request of this file.
    assert_eq!(crc16_xmodem(&[0x2F, 0x37, 0x00, 0x00, 0x01, 0x00]), 0xA800);
}

#[test]
fn crc16_xmodem_update_folds_bytes_one_at_a_time() {
    let data = b"123456789";
    let crc = data.iter().fold(0, |crc, &b| crc16_xmodem_update(crc, b));
    assert_eq!(crc, crc16_xmodem(data));
    assert_eq!(crc16_xmodem_update(0, 0), 0);
    assert_eq!(crc16_xmodem_update(0, 1), 0x1021);
}

// ---------------------------------------------------------------- Command, Ack

#[test]
fn from_code_knows_every_code_of_the_protocol_header() {
    // HDR:28-114 (0x36 was removed; 0x40 is cmd_DeviceVerify).
    let table: [(u8, Command); 16] = [
        (0x30, Command::InterfaceTestAlive),
        (0x31, Command::ProtocolGetVersion),
        (0x32, Command::InterfaceGetName),
        (0x33, Command::InterfaceGetVersion),
        (0x34, Command::InterfaceExit),
        (0x35, Command::DeviceReset),
        (0x37, Command::DeviceInitFlash),
        (0x38, Command::DeviceEraseAll),
        (0x39, Command::DevicePageErase),
        (0x3A, Command::DeviceRead),
        (0x3B, Command::DeviceWrite),
        (0x3C, Command::DeviceC2ckLow),
        (0x3D, Command::DeviceReadEeprom),
        (0x3E, Command::DeviceWriteEeprom),
        (0x3F, Command::InterfaceSetMode),
        (0x40, Command::DeviceVerify),
    ];
    for (code, command) in table {
        assert_eq!(Command::from_code(code), command);
    }
    for code in [0x00, 0x2E, 0x2F, 0x36, 0x41, 0xFF] {
        assert_eq!(Command::from_code(code), Command::Unknown(code));
    }
}

#[test]
fn code_inverts_from_code_for_every_byte() {
    for code in 0..=255u8 {
        assert_eq!(Command::from_code(code).code(), code);
    }
    assert_eq!(Command::Unknown(0x36).code(), 0x36);
}

#[test]
fn from_code_ack_knows_every_acknowledge_code() {
    // CFG:104-116 (all eleven); HDR:158-164 has seven of them.
    let table: [(u8, Ack); 11] = [
        (0x00, Ack::Ok),
        (0x01, Ack::IUnknownError),
        (0x02, Ack::IInvalidCmd),
        (0x03, Ack::IInvalidCrc),
        (0x04, Ack::IVerifyError),
        (0x05, Ack::DInvalidCommand),
        (0x06, Ack::DCommandFailed),
        (0x07, Ack::DUnknownError),
        (0x08, Ack::IInvalidChannel),
        (0x09, Ack::IInvalidParam),
        (0x0F, Ack::DGeneralError),
    ];
    for (code, ack) in table {
        assert_eq!(Ack::from_code(code), ack);
        assert_eq!(ack.code(), code);
    }
    for code in [0x0A, 0x0E, 0x10, 0x30, 0xFF] {
        assert_eq!(Ack::from_code(code), Ack::Unknown(code));
    }
}

#[test]
fn code_ack_inverts_from_code_for_every_byte() {
    for code in 0..=255u8 {
        assert_eq!(Ack::from_code(code).code(), code);
    }
}

// ---------------------------------------------------------------- parser

#[test]
fn new_parser_waits_for_the_start_byte() {
    let mut parser = RequestParser::new();
    for byte in [0u8, 0x2E, b'$', 0xFF, 0x30] {
        assert!(parser.push(byte).is_none());
    }
}

#[test]
fn default_request_parser_behaves_like_new() {
    let frame = request_bytes(0x30, 0, &[0]);
    let mut parser = RequestParser::default();
    let events: Vec<_> = frame
        .iter()
        .filter_map(|&b| parser.push(b).map(own))
        .collect();
    assert_eq!(
        events,
        [Event::Request(Command::InterfaceTestAlive, 0, vec![0])]
    );
}

#[test]
fn push_parses_init_flash_as_the_configurator_sends_it() {
    // CFG:205-210: InitFlash, parameters [target 0], address 0.
    let bytes = [0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00];
    assert_eq!(request_bytes(0x37, 0, &[0]), bytes);
    assert_eq!(
        run_by_byte(&bytes),
        [Event::Request(Command::DeviceInitFlash, 0, vec![0])]
    );
}

#[test]
fn push_parses_device_read_as_the_configurator_sends_it() {
    // CFG:326-337: read 4 bytes at wire address 0x7C00.
    let bytes = [0x2F, 0x3A, 0x7C, 0x00, 0x01, 0x04, 0xC4, 0xB9];
    assert_eq!(request_bytes(0x3A, 0x7C00, &[4]), bytes);
    assert_eq!(
        run_by_byte(&bytes),
        [Event::Request(Command::DeviceRead, 0x7C00, vec![4])]
    );
}

#[test]
fn push_parses_device_write_interface_exit_test_alive_and_reset() {
    let write = [
        0x2F, 0x3B, 0x7C, 0x00, 0x04, 0xDE, 0xAD, 0xBE, 0xEF, 0x34, 0x80,
    ];
    assert_eq!(
        request_bytes(0x3B, 0x7C00, &[0xDE, 0xAD, 0xBE, 0xEF]),
        write
    );
    let exit = [0x2F, 0x34, 0x00, 0x00, 0x01, 0x00, 0x46, 0xD2];
    let alive = [0x2F, 0x30, 0x00, 0x00, 0x01, 0x00, 0xCF, 0xD4];
    let reset = [0x2F, 0x35, 0x00, 0x00, 0x01, 0x00, 0xEC, 0x83];
    let mut stream = Vec::new();
    for frame in [&write[..], &exit, &alive, &reset] {
        stream.extend_from_slice(frame);
    }
    assert_eq!(
        run_by_byte(&stream),
        [
            Event::Request(Command::DeviceWrite, 0x7C00, vec![0xDE, 0xAD, 0xBE, 0xEF]),
            Event::Request(Command::InterfaceExit, 0, vec![0]),
            Event::Request(Command::InterfaceTestAlive, 0, vec![0]),
            Event::Request(Command::DeviceReset, 0, vec![0]),
        ]
    );
}

#[test]
fn push_reports_an_unknown_command_code() {
    let frame = request_bytes(0x36, 0x1234, &[9]);
    assert_eq!(
        run_by_byte(&frame),
        [Event::Request(Command::Unknown(0x36), 0x1234, vec![9])]
    );
}

#[test]
fn push_treats_a_count_of_zero_as_256_parameters() {
    let params: Vec<u8> = (0..=255).collect();
    let frame = request_bytes(0x3B, 0x0400, &params);
    assert_eq!(frame[4], 0);
    assert_eq!(frame.len(), MAX_REQUEST_LEN);
    assert_eq!(
        run_by_byte(&frame),
        [Event::Request(Command::DeviceWrite, 0x0400, params)]
    );
}

#[test]
fn push_accepts_the_start_byte_inside_parameters_address_and_crc() {
    let frame = request_bytes(0x3B, 0x2F2F, &[0x2F, 0x2F, 0x2F]);
    assert_eq!(
        run_by_byte(&frame),
        [Event::Request(Command::DeviceWrite, 0x2F2F, vec![0x2F; 3])]
    );
}

#[test]
fn push_reports_bad_crc_with_both_values() {
    let mut frame = request_bytes(0x37, 0, &[0]);
    *frame.last_mut().unwrap() ^= 1; // CRC low 0x00 -> 0x01
    assert_eq!(
        run_by_byte(&frame),
        [Event::Error(ParseError::BadCrc {
            expected: 0xA800,
            received: 0xA801
        })]
    );
}

#[test]
fn push_resynchronises_after_a_crc_error_even_on_a_start_byte() {
    let mut bad = request_bytes(0x30, 0, &[0]);
    *bad.last_mut().unwrap() = 0x2F; // wrong, and it starts the next frame
    // The next frame's 0x2F is the `bad` frame's last byte; the rest follows.
    let good = request_bytes(0x30, 0, &[0]);
    let mut stream = bad;
    stream.extend_from_slice(&good[1..]);
    let events = run_by_byte(&stream);
    assert_eq!(events.len(), 2);
    assert!(matches!(events[0], Event::Error(ParseError::BadCrc { .. })));
    assert_eq!(
        events[1],
        Event::Request(Command::InterfaceTestAlive, 0, vec![0])
    );
}

#[test]
fn push_skips_noise_between_frames() {
    let mut stream = vec![0x00, 0xFF, b'$', b'M', b'<'];
    stream.extend(request_bytes(0x30, 0, &[0]));
    stream.extend([0x2E, 0x01]);
    stream.extend(request_bytes(0x31, 0, &[0]));
    let events = run_by_byte(&stream);
    // The stray 0x2E and 0x01 are not a frame start.
    assert_eq!(
        events,
        [
            Event::Request(Command::InterfaceTestAlive, 0, vec![0]),
            Event::Request(Command::ProtocolGetVersion, 0, vec![0]),
        ]
    );
}

#[test]
fn feed_reports_consumed_bytes_and_resumes_in_the_next_call() {
    let first = request_bytes(0x30, 0, &[0]);
    let second = request_bytes(0x3A, 0x7C00, &[4]);
    let mut stream = first.clone();
    stream.extend(&second);
    let mut parser = RequestParser::new();
    let (used, event) = parser.feed(&stream[..first.len() + 3]);
    assert_eq!(used, first.len());
    assert_eq!(
        event.map(own),
        Some(Event::Request(Command::InterfaceTestAlive, 0, vec![0]))
    );
    let (used, event) = parser.feed(&stream[first.len()..first.len() + 3]);
    assert_eq!((used, event.is_none()), (3, true));
    let (used, event) = parser.feed(&stream[first.len() + 3..]);
    assert_eq!(used, second.len() - 3);
    assert_eq!(
        event.map(own),
        Some(Event::Request(Command::DeviceRead, 0x7C00, vec![4]))
    );
    let (used, event) = parser.feed(&[]);
    assert_eq!((used, event.is_none()), (0, true));
}

#[test]
fn reset_drops_a_partial_frame() {
    let frame = request_bytes(0x3A, 0x7C00, &[4]);
    let mut parser = RequestParser::new();
    for &byte in &frame[..5] {
        assert!(parser.push(byte).is_none());
    }
    parser.reset();
    for &byte in &frame[5..] {
        assert!(parser.push(byte).is_none());
    }
    let events: Vec<_> = frame
        .iter()
        .filter_map(|&b| parser.push(b).map(own))
        .collect();
    assert_eq!(
        events,
        [Event::Request(Command::DeviceRead, 0x7C00, vec![4])]
    );
}

// ---------------------------------------------------------------- encoder

#[test]
fn encode_reply_writes_the_init_flash_reply() {
    // Signature 0x1F06 (low byte first), pin code 0x14, interface mode 4 (HDR:171).
    assert_eq!(
        encoded(
            Command::DeviceInitFlash,
            0,
            &[0x06, 0x1F, 0x14, 0x04],
            Ack::Ok
        )
        .unwrap(),
        [
            0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A
        ]
    );
}

#[test]
fn encode_reply_writes_the_device_read_reply() {
    assert_eq!(
        encoded(Command::DeviceRead, 0x7C00, &[1, 2, 3, 4], Ack::Ok).unwrap(),
        [
            0x2E, 0x3A, 0x7C, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x00, 0x6E, 0xD4
        ]
    );
    // A refused read: one parameter byte and an error acknowledge (AP:1095-1105).
    assert_eq!(
        encoded(Command::DeviceRead, 0x7C00, &[0], Ack::DGeneralError).unwrap(),
        [0x2E, 0x3A, 0x7C, 0x00, 0x01, 0x00, 0x0F, 0xA5, 0x82]
    );
}

#[test]
fn encode_reply_writes_the_exit_and_write_replies() {
    assert_eq!(
        encoded(Command::InterfaceExit, 0, &[0], Ack::Ok).unwrap(),
        [0x2E, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x42, 0x63]
    );
    assert_eq!(
        encoded(Command::DeviceWrite, 0x7C00, &[0], Ack::Ok).unwrap(),
        [0x2E, 0x3B, 0x7C, 0x00, 0x01, 0x00, 0x00, 0x11, 0xCD]
    );
}

#[test]
fn encode_reply_writes_256_parameters_with_a_count_of_zero() {
    let params: Vec<u8> = (0..=255).rev().collect();
    let frame = encoded(Command::DeviceRead, 0x0400, &params, Ack::Ok).unwrap();
    assert_eq!(frame.len(), MAX_REPLY_LEN);
    assert_eq!(frame[4], 0);
    assert_eq!(decode_reply(&frame), (0x3A, 0x0400, params, 0));
}

#[test]
fn encode_reply_rejects_empty_and_oversized_parameters() {
    let mut out = [0u8; 400];
    assert_eq!(
        encode_reply(Command::DeviceRead, 0, &[], Ack::Ok, &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert_eq!(
        encode_reply(
            Command::DeviceRead,
            0,
            &[0; MAX_PARAMS + 1],
            Ack::Ok,
            &mut out
        ),
        Err(EncodeError::InvalidLength)
    );
    assert!(out.iter().all(|&b| b == 0));
}

#[test]
fn encode_reply_reports_a_short_buffer_and_writes_nothing() {
    let mut out = [0xAAu8; 11];
    assert_eq!(
        encode_reply(Command::DeviceRead, 0, &[1, 2, 3, 4], Ack::Ok, &mut out),
        Err(EncodeError::BufferTooSmall { needed: 12 })
    );
    assert_eq!(out, [0xAA; 11]);
    assert_eq!(
        encode_reply(Command::DeviceRead, 0, &[1], Ack::Ok, &mut []),
        Err(EncodeError::BufferTooSmall { needed: 9 })
    );
    let mut exact = [0u8; 12];
    assert_eq!(
        encode_reply(Command::DeviceRead, 0, &[1, 2, 3, 4], Ack::Ok, &mut exact),
        Ok(12)
    );
}

// ---------------------------------------------------------------- properties

fn is_bad_crc(event: &Event) -> bool {
    matches!(event, Event::Error(ParseError::BadCrc { .. }))
}

fn params_strategy(max: usize) -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(any::<u8>(), 1..=max)
}

proptest! {
    #[test]
    fn roundtrip_request_through_every_feeding_style(
        command in any::<u8>(),
        address in any::<u16>(),
        params in params_strategy(MAX_PARAMS),
        cuts in prop::collection::vec(any::<usize>(), 0..6),
    ) {
        let frame = request_bytes(command, address, &params);
        let expected = [Event::Request(Command::from_code(command), address, params.clone())];
        prop_assert_eq!(run_by_byte(&frame), expected.clone());
        prop_assert_eq!(run_chunks(&frame, &cuts), expected.clone());
        prop_assert_eq!(reference_events(&frame), expected);
    }

    #[test]
    fn roundtrip_reply_matches_an_independent_decoder(
        command in any::<u8>(),
        address in any::<u16>(),
        params in params_strategy(MAX_PARAMS),
        ack in any::<u8>(),
    ) {
        let frame = encoded(Command::from_code(command), address, &params, Ack::from_code(ack)).unwrap();
        prop_assert_eq!(decode_reply(&frame), (command, address, params, ack));
    }

    #[test]
    fn arbitrary_bytes_never_panic_and_match_the_reference_parser(
        stream in prop::collection::vec(marker_biased_byte(), 0..900),
        cuts in prop::collection::vec(any::<usize>(), 0..8),
    ) {
        let reference = reference_events(&stream);
        prop_assert_eq!(run_by_byte(&stream), reference.clone());
        prop_assert_eq!(run_chunks(&stream, &cuts), reference.clone());
        // Every request it yields has a correct CRC over bytes that are in the stream.
        for event in &reference {
            if let Event::Request(command, address, params) = event {
                let frame = request_bytes(command.code(), *address, params);
                prop_assert!(stream.windows(frame.len()).any(|w| w == frame.as_slice()));
            }
        }
    }

    #[test]
    fn frames_between_garbage_are_all_found(
        frames in prop::collection::vec((any::<u8>(), any::<u16>(), params_strategy(20)), 1..5),
        gaps in prop::collection::vec(prop::collection::vec(any::<u8>(), 0..6), 5),
        cuts in prop::collection::vec(any::<usize>(), 0..8),
    ) {
        let mut stream = Vec::new();
        let mut expected = Vec::new();
        for (i, (command, address, params)) in frames.iter().enumerate() {
            stream.extend(gaps[i].iter().filter(|&&b| b != 0x2F));
            stream.extend(request_bytes(*command, *address, params));
            expected.push(Event::Request(Command::from_code(*command), *address, params.clone()));
        }
        prop_assert_eq!(run_chunks(&stream, &cuts), expected);
    }

    #[test]
    fn corrupting_a_covered_byte_is_rejected_and_the_next_frame_survives(
        command in any::<u8>(),
        address in any::<u16>(),
        params in params_strategy(60),
        position in any::<prop::sample::Index>(),
        flip in 1..=255u8,
    ) {
        let frame = request_bytes(command, address, &params);
        // Everything the CRC covers or is: command, address, parameters, CRC; not the count.
        let positions: Vec<usize> = [1, 2, 3].into_iter().chain(5..frame.len()).collect();
        let index = positions[position.index(positions.len())];
        let mut corrupted = frame.clone();
        corrupted[index] ^= flip;
        // A last byte that reads 0x2F is looked at as a frame start (documented), and would
        // swallow the next frame; it is also the one case of a lost start.
        prop_assume!(corrupted[frame.len() - 1] != 0x2F);
        corrupted.extend(request_bytes(0x30, 0, &[0]));
        let events = run_by_byte(&corrupted);
        prop_assert_eq!(events.len(), 2, "{:?}", events);
        prop_assert!(is_bad_crc(&events[0]), "{:?}", events);
        prop_assert_eq!(&events[1], &Event::Request(Command::InterfaceTestAlive, 0, vec![0]));
        prop_assert_eq!(run_chunks(&corrupted, &[2, 7]), events);
    }

    #[test]
    fn corrupting_the_start_byte_never_yields_the_frame_and_the_next_survives(
        command in any::<u8>(),
        address in any::<u16>(),
        params in params_strategy(40),
        value in any::<u8>(),
    ) {
        let frame = request_bytes(command, address, &params);
        prop_assume!(value != 0x2F);
        prop_assume!(!frame[1..].contains(&0x2F));
        let mut corrupted = frame.clone();
        corrupted[0] = value;
        corrupted.extend(request_bytes(0x30, 0, &[0]));
        prop_assert_eq!(
            run_by_byte(&corrupted),
            [Event::Request(Command::InterfaceTestAlive, 0, vec![0])]
        );
    }

    #[test]
    fn corrupting_the_count_never_accepts_a_frame_with_a_bad_crc(
        command in any::<u8>(),
        address in any::<u16>(),
        params in params_strategy(60),
        value in any::<u8>(),
        tail in prop::collection::vec(marker_biased_byte(), 0..40),
    ) {
        let mut corrupted = request_bytes(command, address, &params);
        prop_assume!(value != corrupted[4]);
        corrupted[4] = value;
        corrupted.extend(&tail);
        prop_assert_eq!(run_by_byte(&corrupted), reference_events(&corrupted));
    }
}
