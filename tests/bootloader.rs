//! Host tests for `esc_passthrough::bootloader`.
//!
//! Sources of the golden vectors (BL = `_plan/ref/AM32-bootloader/bootloader/main.c`, CFG =
//! `_plan/ref/am32-configurator`, SIM = `CFG/ci/bootloader_sim.py`, AP = `ardupilot/libraries/
//! AP_BLHeli/AP_BLHeli.cpp`):
//! - CRC16, polynomial 0xA001 reflected, start 0, low byte first: BL:486-511 (also SIM:89-99,
//!   CFG src/communication/direct.ts:67-91, AP:628-643);
//! - init, 21 bytes without CRC: matched at BL:604-627, sent by AP:796 and CFG direct.ts:104-106;
//! - init reply, 9 bytes `'4' '7' '1'`, pin code, flash size code, `06`, `06`, protocol version,
//!   `30`: BL:242, 254-292 (flash size codes 0x1F, 0x35, 0x2B at BL:216, 221, 226);
//! - set address `FF 00 AH AL` + CRC: BL:669-713, AP:756; set buffer `FE 00 {01 or 00} len` +
//!   CRC and no reply: BL:715-737, AP:890-891 (CFG direct.ts sends `FE 00 00 00` for 256, which
//!   BL:728-731 reads as a size of 0: a disagreement, not followed); payload + CRC and `30` or
//!   `C2`: BL:583-600; program `01 01` + CRC: BL:636-667, AP:923; read `03 n` (0 is 256) + CRC,
//!   reply data + CRC + `30`: BL:771-813, AP:766; keep-alive `FD 00` + CRC, reply `C1` on
//!   purpose: BL:739-751, AP:859-867; run `00 00 00 00`: BL:629-634, CFG direct.ts:218;
//! - acknowledge codes `30`, `C1`, `C2`: BL:547-565.
//!
//! No captured traffic exists in the sources; the bytes were laid out by hand from the formulas
//! above and their CRCs computed with a separate script. The address 0x7C00 (an F421 settings
//! address on the wire), pin code 0x14 and the payload DE AD BE EF are examples.

mod common;

use common::ref_crc_arc;
use esc_passthrough::EncodeError;
use esc_passthrough::bootloader::{
    Ack, DeviceInfo, INIT, INIT_LEN, INIT_REPLY_LEN, MAX_DATA, ParseError, crc16, crc16_update,
    encode_erase, encode_init, encode_keep_alive, encode_payload, encode_program, encode_read,
    encode_run, encode_set_address, encode_set_buffer, parse_ack, parse_init_reply,
    parse_read_reply,
};
use proptest::prelude::*;

fn encoded(f: impl FnOnce(&mut [u8]) -> Result<usize, EncodeError>) -> Vec<u8> {
    let mut out = [0xAAu8; 300];
    let n = f(&mut out).unwrap();
    out[..n].to_vec()
}

fn with_crc(body: &[u8]) -> Vec<u8> {
    let mut frame = body.to_vec();
    frame.extend_from_slice(&ref_crc_arc(body).to_le_bytes());
    frame
}

fn read_reply(data: &[u8]) -> Vec<u8> {
    let mut reply = with_crc(data);
    reply.push(0x30);
    reply
}

// ---------------------------------------------------------------- CRC

#[test]
fn crc16_matches_the_published_check_value() {
    // CRC-16/ARC catalogue entry (poly 0x8005 reflected = 0xA001, init 0): "123456789" -> 0xBB3D.
    assert_eq!(crc16(b"123456789"), 0xBB3D);
    assert_eq!(ref_crc_arc(b"123456789"), 0xBB3D);
    assert_eq!(crc16(&[]), 0);
    assert_eq!(crc16(&[0, 0]), 0);
}

#[test]
fn crc16_equals_the_bit_serial_reference() {
    for data in [
        &[0u8][..],
        &[0xFF],
        &[0xFF, 0x00, 0x7C, 0x00],
        &[0xA5; 300],
        &INIT[..],
    ] {
        assert_eq!(crc16(data), ref_crc_arc(data));
    }
    // The golden set-address frame of this file.
    assert_eq!(crc16(&[0xFF, 0x00, 0x7C, 0x00]), 0xD410);
}

#[test]
fn crc16_update_folds_bytes_one_at_a_time() {
    let data = b"123456789";
    let crc = data.iter().fold(0, |crc, &b| crc16_update(crc, b));
    assert_eq!(crc, crc16(data));
    assert_eq!(crc16_update(0, 0), 0);
    assert_eq!(crc16_update(0, 1), 0xC0C1);
}

// ---------------------------------------------------------------- Ack

#[test]
fn from_code_ack_knows_exactly_three_codes() {
    assert_eq!(Ack::from_code(0x30), Some(Ack::Good));
    assert_eq!(Ack::from_code(0xC1), Some(Ack::BadCommand));
    assert_eq!(Ack::from_code(0xC2), Some(Ack::BadCrc));
    for code in (0..=255u8).filter(|c| ![0x30, 0xC1, 0xC2].contains(c)) {
        assert_eq!(Ack::from_code(code), None);
    }
}

#[test]
fn code_inverts_from_code_ack() {
    for ack in [Ack::Good, Ack::BadCommand, Ack::BadCrc] {
        assert_eq!(Ack::from_code(ack.code()), Some(ack));
    }
    assert_eq!(
        (Ack::Good.code(), Ack::BadCommand.code(), Ack::BadCrc.code()),
        (0x30, 0xC1, 0xC2)
    );
}

// ---------------------------------------------------------------- encoders

#[test]
fn encode_init_writes_the_21_byte_sequence() {
    let golden = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
        0x0D, 0x42, 0x4C, 0x48, 0x65, 0x6C, 0x69, 0xF4, 0x7D,
    ];
    assert_eq!(INIT, golden);
    assert_eq!(INIT_LEN, 21);
    assert_eq!(encoded(encode_init), golden);
    let mut small = [0u8; 20];
    assert_eq!(
        encode_init(&mut small),
        Err(EncodeError::BufferTooSmall { needed: 21 })
    );
    assert_eq!(small, [0; 20]);
}

#[test]
fn encode_set_address_writes_ff_00_address_and_crc() {
    assert_eq!(
        encoded(|o| encode_set_address(0x7C00, o)),
        [0xFF, 0x00, 0x7C, 0x00, 0x10, 0xD4]
    );
    // The magic address of the settings block (BL:303).
    assert_eq!(
        encoded(|o| encode_set_address(0x0020, o)),
        [0xFF, 0x00, 0x00, 0x20, 0x31, 0xCC]
    );
    let mut small = [0u8; 5];
    assert_eq!(
        encode_set_address(0x7C00, &mut small),
        Err(EncodeError::BufferTooSmall { needed: 6 })
    );
}

#[test]
fn encode_set_buffer_writes_the_length_with_a_high_byte_for_256() {
    assert_eq!(
        encoded(|o| encode_set_buffer(4, o)),
        [0xFE, 0x00, 0x00, 0x04, 0x30, 0x2B]
    );
    assert_eq!(
        encoded(|o| encode_set_buffer(184, o)),
        [0xFE, 0x00, 0x00, 0xB8, 0x31, 0x9A]
    );
    assert_eq!(
        encoded(|o| encode_set_buffer(256, o)),
        [0xFE, 0x00, 0x01, 0x00, 0x30, 0x78]
    );
    let mut out = [0u8; 8];
    assert_eq!(
        encode_set_buffer(0, &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert_eq!(
        encode_set_buffer(257, &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert_eq!(
        encode_set_buffer(4, &mut out[..5]),
        Err(EncodeError::BufferTooSmall { needed: 6 })
    );
}

#[test]
fn encode_payload_writes_the_data_then_its_crc() {
    assert_eq!(
        encoded(|o| encode_payload(&[0xDE, 0xAD, 0xBE, 0xEF], o)),
        [0xDE, 0xAD, 0xBE, 0xEF, 0x9B, 0xE5]
    );
    let big = [0x5A; MAX_DATA];
    assert_eq!(encoded(|o| encode_payload(&big, o)), with_crc(&big));
    let mut out = [0u8; 300];
    assert_eq!(
        encode_payload(&[], &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert_eq!(
        encode_payload(&[0; 257], &mut out),
        Err(EncodeError::InvalidLength)
    );
    assert_eq!(
        encode_payload(&[1, 2, 3], &mut out[..4]),
        Err(EncodeError::BufferTooSmall { needed: 5 })
    );
}

#[test]
fn encode_program_writes_01_01_and_crc() {
    assert_eq!(encoded(encode_program), [0x01, 0x01, 0xC0, 0x50]);
    let mut small = [0u8; 3];
    assert_eq!(
        encode_program(&mut small),
        Err(EncodeError::BufferTooSmall { needed: 4 })
    );
}

#[test]
fn encode_erase_writes_02_01_and_crc() {
    // BL:753-769 (CMD_ERASE_FLASH 0x02, rxBuffer[1] is not looked at), AP:870-876 sends {02, 01};
    // CRC 0xA0C0 (low byte first) worked out with a separate script.
    assert_eq!(encoded(encode_erase), [0x02, 0x01, 0xC0, 0xA0]);
    assert_eq!(encoded(encode_erase), with_crc(&[0x02, 0x01]));
    let mut small = [0u8; 3];
    assert_eq!(
        encode_erase(&mut small),
        Err(EncodeError::BufferTooSmall { needed: 4 })
    );
}

#[test]
fn encode_read_writes_03_n_and_crc_with_256_sent_as_zero() {
    assert_eq!(encoded(|o| encode_read(32, o)), [0x03, 0x20, 0x01, 0x28]);
    assert_eq!(encoded(|o| encode_read(184, o)), [0x03, 0xB8, 0x00, 0x82]);
    assert_eq!(encoded(|o| encode_read(256, o)), [0x03, 0x00, 0x00, 0xF0]);
    assert_eq!(encoded(|o| encode_read(1, o)), with_crc(&[0x03, 0x01]));
    let mut out = [0u8; 4];
    assert_eq!(encode_read(0, &mut out), Err(EncodeError::InvalidLength));
    assert_eq!(encode_read(257, &mut out), Err(EncodeError::InvalidLength));
    assert_eq!(
        encode_read(32, &mut out[..3]),
        Err(EncodeError::BufferTooSmall { needed: 4 })
    );
}

#[test]
fn encode_keep_alive_writes_fd_00_and_crc() {
    assert_eq!(encoded(encode_keep_alive), [0xFD, 0x00, 0x40, 0x90]);
    let mut small = [0u8; 3];
    assert_eq!(
        encode_keep_alive(&mut small),
        Err(EncodeError::BufferTooSmall { needed: 4 })
    );
}

#[test]
fn encode_run_writes_four_zero_bytes() {
    assert_eq!(encoded(encode_run), [0x00, 0x00, 0x00, 0x00]);
    let mut small = [0u8; 3];
    assert_eq!(
        encode_run(&mut small),
        Err(EncodeError::BufferTooSmall { needed: 4 })
    );
}

#[test]
fn encoders_make_the_program_sequence_of_a_four_byte_write() {
    // AP:888-942 order: set address, set buffer, payload, program.
    let mut stream = Vec::new();
    stream.extend(encoded(|o| encode_set_address(0x7C00, o)));
    stream.extend(encoded(|o| encode_set_buffer(4, o)));
    stream.extend(encoded(|o| encode_payload(&[0xDE, 0xAD, 0xBE, 0xEF], o)));
    stream.extend(encoded(encode_program));
    assert_eq!(
        stream,
        [
            0xFF, 0x00, 0x7C, 0x00, 0x10, 0xD4, //
            0xFE, 0x00, 0x00, 0x04, 0x30, 0x2B, //
            0xDE, 0xAD, 0xBE, 0xEF, 0x9B, 0xE5, //
            0x01, 0x01, 0xC0, 0x50,
        ]
    );
}

// ---------------------------------------------------------------- parsers

#[test]
fn parse_ack_returns_the_three_codes_as_values() {
    assert_eq!(parse_ack(&[0x30]), Ok(Ack::Good));
    // The keep-alive expects C1 (BL:749, AP:857-867).
    assert_eq!(parse_ack(&[0xC1]), Ok(Ack::BadCommand));
    assert_eq!(parse_ack(&[0xC2]), Ok(Ack::BadCrc));
}

#[test]
fn parse_ack_reports_length_and_unknown_codes() {
    assert_eq!(parse_ack(&[]), Err(ParseError::Truncated));
    assert_eq!(parse_ack(&[0x30, 0x30]), Err(ParseError::TrailingBytes));
    assert_eq!(parse_ack(&[0x00]), Err(ParseError::UnknownAck(0x00)));
    assert_eq!(parse_ack(&[0xFF]), Err(ParseError::UnknownAck(0xFF)));
}

#[test]
fn parse_init_reply_reads_the_device_info() {
    // BL:285: '4','7','1', pin code, flash size code, 06, 06, protocol version 3, 30.
    let reply = [b'4', b'7', b'1', 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30];
    let info = parse_init_reply(&reply).unwrap();
    assert_eq!(info.pin_code, 0x14);
    assert_eq!(info.flash_size_code, 0x1F);
    assert_eq!(info.tail, [0x06, 0x06]);
    assert_eq!(info.protocol_version, 3);
    assert_eq!(INIT_REPLY_LEN, reply.len());
    for flash_size_code in [0x1F, 0x35, 0x2B] {
        let mut other = reply;
        other[4] = flash_size_code;
        assert_eq!(
            parse_init_reply(&other).unwrap().flash_size_code,
            flash_size_code
        );
    }
}

#[test]
fn signature_puts_the_flash_size_code_in_the_high_byte() {
    // CFG direct.ts:120: (infoBuffer[4] << 8) | infoBuffer[5]; 0x1F06 is the 32 KB F421.
    let info = parse_init_reply(&[b'4', b'7', b'1', 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30]).unwrap();
    assert_eq!(info.signature(), 0x1F06);
    let info = parse_init_reply(&[b'4', b'7', b'1', 0x14, 0x2B, 0x06, 0x06, 0x03, 0x30]).unwrap();
    assert_eq!(info.signature(), 0x2B06);
}

#[test]
fn parse_init_reply_reports_each_kind_of_failure() {
    let good = [b'4', b'7', b'1', 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30];
    assert_eq!(parse_init_reply(&good[..8]), Err(ParseError::Truncated));
    assert_eq!(parse_init_reply(&[]), Err(ParseError::Truncated));
    let mut longer = good.to_vec();
    longer.push(0);
    assert_eq!(parse_init_reply(&longer), Err(ParseError::TrailingBytes));
    let mut wrong = good;
    wrong[2] = b'2';
    assert_eq!(parse_init_reply(&wrong), Err(ParseError::BadSignature));
    let mut refused = good;
    refused[8] = 0xC1;
    assert_eq!(
        parse_init_reply(&refused),
        Err(ParseError::Rejected(Ack::BadCommand))
    );
    refused[8] = 0xC2;
    assert_eq!(
        parse_init_reply(&refused),
        Err(ParseError::Rejected(Ack::BadCrc))
    );
    refused[8] = 0x00;
    assert_eq!(
        parse_init_reply(&refused),
        Err(ParseError::UnknownAck(0x00))
    );
}

#[test]
fn parse_read_reply_returns_the_data() {
    // Data 01 02 03 04, CRC 0FA1 (A1 first), acknowledge 30.
    let reply = [0x01, 0x02, 0x03, 0x04, 0xA1, 0x0F, 0x30];
    assert_eq!(read_reply(&[1, 2, 3, 4]), reply);
    assert_eq!(parse_read_reply(&reply, 4), Ok(&reply[..4]));
    let block: Vec<u8> = (0..=255).collect();
    let big = read_reply(&block);
    assert_eq!(parse_read_reply(&big, 256), Ok(block.as_slice()));
}

#[test]
fn parse_read_reply_reports_each_kind_of_failure() {
    let reply = [0x01, 0x02, 0x03, 0x04, 0xA1, 0x0F, 0x30];
    assert_eq!(parse_read_reply(&reply, 0), Err(ParseError::InvalidLength));
    assert_eq!(
        parse_read_reply(&reply, 257),
        Err(ParseError::InvalidLength)
    );
    assert_eq!(parse_read_reply(&reply[..6], 4), Err(ParseError::Truncated));
    // A refused read is one byte; it is Truncated here and readable with parse_ack.
    assert_eq!(parse_read_reply(&[0xC1], 4), Err(ParseError::Truncated));
    let mut longer = reply.to_vec();
    longer.push(0x30);
    assert_eq!(parse_read_reply(&longer, 4), Err(ParseError::TrailingBytes));
    let mut bad_data = reply;
    bad_data[1] ^= 0x10;
    assert_eq!(
        parse_read_reply(&bad_data, 4),
        Err(ParseError::BadCrc {
            expected: crc16(&bad_data[..4]),
            received: 0x0FA1
        })
    );
    let mut refused = reply;
    refused[6] = 0xC2;
    assert_eq!(
        parse_read_reply(&refused, 4),
        Err(ParseError::Rejected(Ack::BadCrc))
    );
    refused[6] = 0x31;
    assert_eq!(
        parse_read_reply(&refused, 4),
        Err(ParseError::UnknownAck(0x31))
    );
}

// ---------------------------------------------------------------- properties

proptest! {
    #[test]
    fn encoders_match_an_independent_decoder(
        address in any::<u16>(),
        len in 1..=256usize,
        data in prop::collection::vec(any::<u8>(), 1..=256),
    ) {
        let [high, low] = address.to_be_bytes();
        prop_assert_eq!(encoded(|o| encode_set_address(address, o)), with_crc(&[0xFF, 0, high, low]));
        let frame = encoded(|o| encode_set_buffer(len, o));
        prop_assert_eq!(frame.len(), 6);
        prop_assert_eq!((frame[0], frame[1]), (0xFE, 0x00));
        let decoded = if frame[2] == 1 { 256 } else { usize::from(frame[3]) };
        prop_assert_eq!(decoded, len);
        prop_assert!(frame[2] <= 1 && (frame[2] == 0 || frame[3] == 0));
        prop_assert_eq!(&frame[4..], &ref_crc_arc(&frame[..4]).to_le_bytes());
        let frame = encoded(|o| encode_read(len, o));
        prop_assert_eq!(frame.len(), 4);
        prop_assert_eq!(frame[0], 3);
        prop_assert_eq!(if frame[1] == 0 { 256 } else { usize::from(frame[1]) }, len);
        prop_assert_eq!(&frame[2..], &ref_crc_arc(&frame[..2]).to_le_bytes());
        let frame = encoded(|o| encode_payload(&data, o));
        prop_assert_eq!(&frame[..data.len()], data.as_slice());
        prop_assert_eq!(&frame[data.len()..], &ref_crc_arc(&data).to_le_bytes());
    }

    #[test]
    fn roundtrip_read_reply_and_every_prefix_is_truncated(
        data in prop::collection::vec(any::<u8>(), 1..=256),
    ) {
        let n = data.len();
        let reply = read_reply(&data);
        prop_assert_eq!(parse_read_reply(&reply, n), Ok(data.as_slice()));
        for cut in 0..reply.len() {
            prop_assert_eq!(parse_read_reply(&reply[..cut], n), Err(ParseError::Truncated));
        }
        let mut longer = reply.clone();
        longer.push(0);
        prop_assert_eq!(parse_read_reply(&longer, n), Err(ParseError::TrailingBytes));
    }

    #[test]
    fn a_corrupted_byte_in_a_read_reply_is_never_accepted(
        data in prop::collection::vec(any::<u8>(), 1..=256),
        position in any::<prop::sample::Index>(),
        flip in 1..=255u8,
    ) {
        let n = data.len();
        let mut reply = read_reply(&data);
        let index = position.index(reply.len());
        reply[index] ^= flip;
        let result = parse_read_reply(&reply, n);
        prop_assert!(result.is_err(), "{:?}", result);
        if index < n + 2 {
            let is_bad_crc = matches!(result, Err(ParseError::BadCrc { .. }));
            prop_assert!(is_bad_crc);
        }
    }

    #[test]
    fn roundtrip_init_reply(
        pin_code in any::<u8>(),
        flash_size_code in any::<u8>(),
        tail in any::<[u8; 2]>(),
        version in any::<u8>(),
    ) {
        let reply = [b'4', b'7', b'1', pin_code, flash_size_code, tail[0], tail[1], version, 0x30];
        let info = parse_init_reply(&reply).unwrap();
        prop_assert_eq!(
            (info.pin_code, info.flash_size_code, info.tail, info.protocol_version),
            (pin_code, flash_size_code, tail, version)
        );
        prop_assert_eq!(info.signature(), u16::from_be_bytes([flash_size_code, tail[0]]));
    }

    #[test]
    fn arbitrary_bytes_never_panic_and_accepted_replies_check_out(
        bytes in prop::collection::vec(any::<u8>(), 0..300),
        n in 0..300usize,
    ) {
        if let Ok(ack) = parse_ack(&bytes) {
            prop_assert_eq!(bytes.len(), 1);
            prop_assert_eq!(ack.code(), bytes[0]);
        }
        if let Ok(info) = parse_init_reply(&bytes) {
            prop_assert_eq!(&bytes[..3], b"471");
            prop_assert_eq!(bytes[8], 0x30);
            prop_assert_eq!(info.pin_code, bytes[3]);
        }
        if let Ok(data) = parse_read_reply(&bytes, n) {
            prop_assert_eq!(bytes.len(), n + 3);
            prop_assert_eq!(data, &bytes[..n]);
            prop_assert_eq!(&bytes[n..n + 2], &ref_crc_arc(data).to_le_bytes());
            prop_assert_eq!(bytes[n + 2], 0x30);
        }
    }

    #[test]
    fn arbitrary_data_reply_shapes_never_panic(
        data in prop::collection::vec(any::<u8>(), 1..=256),
        n in 0..300usize,
    ) {
        // A well-formed reply for a different n: never a panic, never data with a bad CRC.
        let reply = read_reply(&data);
        if let Ok(accepted) = parse_read_reply(&reply, n) {
            prop_assert_eq!(accepted, data.as_slice());
        }
    }
}

#[test]
fn device_info_is_a_plain_value() {
    let info = parse_init_reply(&[b'4', b'7', b'1', 1, 2, 6, 6, 3, 0x30]).unwrap();
    let copy: DeviceInfo = info;
    assert_eq!(copy, info);
}
