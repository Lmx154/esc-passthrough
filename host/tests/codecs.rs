//! The tool's codecs against fixed vectors, the retry rule, the refusal path and the read-only
//! rule, on a scripted link (no port, no fake board).
//!
//! The vectors are those of the crate's `tests/msp.rs` and `tests/fourway.rs`, which were checked
//! against the configurator's source; the CRC is also computed here bit by bit.

use esc_passthrough_host::fourway::{self, CheckError, ParseError as FwParse, Request as Fw};
use esc_passthrough_host::link::Link;
use esc_passthrough_host::msp::{self, ParseError as MspParse, Reply, Request as Msp};
use esc_passthrough_host::tool::{self, Comparison, Options, Passthrough, Timing};
use std::collections::VecDeque;
use std::io;
use std::time::Duration;

/// CRC16-XMODEM, one bit at a time through a shift register (an independent reference).
fn ref_crc_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for bit in (0..8).rev() {
            let feedback = ((crc >> 15) & 1) ^ u16::from((byte >> bit) & 1);
            crc <<= 1;
            if feedback == 1 {
                crc ^= 0x1021;
            }
        }
    }
    crc
}

fn fw_reply(command: u8, address: u16, params: &[u8], ack: u8) -> Vec<u8> {
    let count = if params.len() == 256 {
        0
    } else {
        params.len() as u8
    };
    let mut frame = vec![0x2E, command, (address >> 8) as u8, address as u8, count];
    frame.extend_from_slice(params);
    frame.push(ack);
    let crc = ref_crc_xmodem(&frame);
    frame.extend(crc.to_be_bytes());
    frame
}

fn msp_reply(command: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![b'$', b'M', b'>', payload.len() as u8, command];
    frame.extend_from_slice(payload);
    frame.push(frame[3..].iter().fold(0, |a, b| a ^ b));
    frame
}

// ---------------------------------------------------------------- MSP vectors

#[test]
fn msp_request_encode_matches_the_five_golden_requests() {
    // Crate tests/msp.rs:213-221; SerialDevice.vue:618, 639, 644, 649, 655.
    let golden: [(Msp, [u8; 6]); 5] = [
        (Msp::ApiVersion, [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01]),
        (Msp::FcVariant, [0x24, 0x4D, 0x3C, 0x00, 0x02, 0x02]),
        (Msp::BatteryState, [0x24, 0x4D, 0x3C, 0x00, 0x82, 0x82]),
        (Msp::MotorConfig, [0x24, 0x4D, 0x3C, 0x00, 0x83, 0x83]),
        (Msp::SetPassthrough, [0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5]),
    ];
    for (request, bytes) in golden {
        assert_eq!(request.encode(), bytes, "{}", request.name());
        assert_eq!(msp::frame_len(&[]), None);
    }
}

#[test]
fn msp_parse_reply_accepts_the_golden_replies() {
    // Crate tests/msp.rs:453-470, 523-545, 601-632.
    let api = [0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2E, 0x2D];
    assert_eq!(
        msp::parse_reply(&api, Msp::ApiVersion),
        Ok(Reply::Data(vec![0, 1, 46]))
    );
    let variant = [0x24, 0x4D, 0x3E, 0x04, 0x02, 0x41, 0x52, 0x44, 0x55, 0x04];
    assert_eq!(
        msp::parse_reply(&variant, Msp::FcVariant),
        Ok(Reply::Data(b"ARDU".to_vec()))
    );
    let battery = [
        0x24, 0x4D, 0x3E, 0x0B, 0x82, 0x00, 0x00, 0x00, 0xFC, 0xD2, 0x04, 0xB8, 0x0B, 0x00, 0xD8,
        0x09, 0xC1,
    ];
    let Ok(Reply::Data(data)) = msp::parse_reply(&battery, Msp::BatteryState) else {
        panic!("battery reply rejected");
    };
    assert_eq!((data.len(), data[3]), (11, 0xFC));
    let motors = [
        0x24, 0x4D, 0x3E, 0x0A, 0x83, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0x8D,
    ];
    let Ok(Reply::Data(data)) = msp::parse_reply(&motors, Msp::MotorConfig) else {
        panic!("motor config reply rejected");
    };
    assert_eq!(data[6], 4);
    let passthrough = [0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0];
    assert_eq!(
        msp::parse_reply(&passthrough, Msp::SetPassthrough),
        Ok(Reply::Data(vec![4]))
    );
    // The error reply (crate tests/msp.rs:428-440).
    let error = [0x24, 0x4D, 0x21, 0x00, 0xF5, 0xF5];
    assert_eq!(
        msp::parse_reply(&error, Msp::SetPassthrough),
        Ok(Reply::Refused)
    );
}

#[test]
fn msp_parse_reply_rejects_every_kind_of_fault() {
    let good = [0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0];
    let p = |bytes: &[u8]| msp::parse_reply(bytes, Msp::SetPassthrough);
    assert_eq!(p(&[]), Err(MspParse::TooShort));
    assert_eq!(p(&good[..6]), Err(MspParse::TooShort));
    let mut bad = good;
    bad[0] = b'#';
    assert_eq!(p(&bad), Err(MspParse::BadStart(b'#')));
    bad = good;
    bad[1] = b'X';
    assert_eq!(p(&bad), Err(MspParse::BadMarker(b'X')));
    bad = good;
    bad[2] = b'<';
    assert_eq!(p(&bad), Err(MspParse::BadDirection(b'<')));
    bad = good;
    bad[6] ^= 1;
    assert_eq!(
        p(&bad),
        Err(MspParse::BadChecksum {
            expected: 0xF0,
            received: 0xF1
        })
    );
    assert_eq!(
        msp::parse_reply(&good, Msp::ApiVersion),
        Err(MspParse::WrongCommand {
            expected: 1,
            got: 0xF5
        })
    );
    // The frame length follows the length byte and a wrong first byte stops the reader at once.
    assert_eq!(msp::frame_len(&good[..3]), None);
    assert_eq!(msp::frame_len(&good[..4]), Some(7));
    assert_eq!(msp::frame_len(&[0x2E]), Some(1));
}

// ---------------------------------------------------------------- four-way vectors

#[test]
fn fourway_request_encode_matches_the_golden_requests() {
    // Crate tests/fourway.rs:280-315.
    assert_eq!(
        Fw::InitFlash { esc: 0 }.encode().unwrap(),
        [0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00]
    );
    assert_eq!(
        Fw::DeviceRead {
            address: 0x7C00,
            length: 4
        }
        .encode()
        .unwrap(),
        [0x2F, 0x3A, 0x7C, 0x00, 0x01, 0x04, 0xC4, 0xB9]
    );
    assert_eq!(
        Fw::InterfaceExit.encode().unwrap(),
        [0x2F, 0x34, 0x00, 0x00, 0x01, 0x00, 0x46, 0xD2]
    );
    assert_eq!(
        Fw::TestAlive.encode().unwrap(),
        [0x2F, 0x30, 0x00, 0x00, 0x01, 0x00, 0xCF, 0xD4]
    );
}

#[test]
fn fourway_read_request_for_the_settings_block_and_the_length_limits() {
    let frame = Fw::DeviceRead {
        address: 0x7C00,
        length: 184,
    }
    .encode()
    .unwrap();
    assert_eq!(frame[..6], [0x2F, 0x3A, 0x7C, 0x00, 0x01, 184]);
    assert_eq!(frame[6..], ref_crc_xmodem(&frame[..6]).to_be_bytes());
    // 256 is sent as 0 (four_way.ts:159).
    let frame = Fw::DeviceRead {
        address: 0,
        length: 256,
    }
    .encode()
    .unwrap();
    assert_eq!(frame[5], 0);
    assert!(
        Fw::DeviceRead {
            address: 0,
            length: 0
        }
        .encode()
        .is_err()
    );
    assert!(
        Fw::DeviceRead {
            address: 0,
            length: 257
        }
        .encode()
        .is_err()
    );
    // Controller 3 is carried in the parameter, the address stays 0 (four_way.ts:206-209).
    let frame = Fw::InitFlash { esc: 3 }.encode().unwrap();
    assert_eq!(frame[2..6], [0, 0, 1, 3]);
}

#[test]
fn fourway_crc_equals_the_bit_serial_reference() {
    assert_eq!(fourway::crc16_xmodem(&[]), 0);
    assert_eq!(fourway::crc16_xmodem(&[0x2F, 0x37, 0, 0, 1, 0]), 0xA800);
    for data in [&[0u8][..], &[0xFF], &[0x2F, 0x37, 0, 0, 1, 0], &[0xA5; 300]] {
        assert_eq!(fourway::crc16_xmodem(data), ref_crc_xmodem(data));
    }
}

#[test]
fn fourway_parse_reply_accepts_the_golden_replies() {
    // Crate tests/fourway.rs:459-496.
    let init = [
        0x2E, 0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A,
    ];
    let reply = fourway::parse_reply(&init).unwrap();
    assert_eq!(
        (
            reply.command,
            reply.address,
            reply.params.as_slice(),
            reply.ack
        ),
        (0x37, 0, &[0x06, 0x1F, 0x14, 0x04][..], 0)
    );
    assert_eq!(reply.check(&Fw::InitFlash { esc: 0 }), Ok(()));
    let read = [
        0x2E, 0x3A, 0x7C, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x00, 0x6E, 0xD4,
    ];
    let reply = fourway::parse_reply(&read).unwrap();
    assert_eq!((reply.address, reply.params), (0x7C00, vec![1, 2, 3, 4]));
    let exit = [0x2E, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x42, 0x63];
    assert_eq!(
        fourway::parse_reply(&exit)
            .unwrap()
            .check(&Fw::InterfaceExit),
        Ok(())
    );
    // A refused read: one parameter byte and an error acknowledge.
    let refused = [0x2E, 0x3A, 0x7C, 0x00, 0x01, 0x00, 0x0F, 0xA5, 0x82];
    let reply = fourway::parse_reply(&refused).unwrap();
    assert_eq!(
        reply.check(&Fw::DeviceRead {
            address: 0x7C00,
            length: 1
        }),
        Err(CheckError::Ack(0x0F))
    );
    // 256 parameters carry a count of 0.
    let params = [0xA5u8; 256];
    let big = fw_reply(0x3A, 0x7C00, &params, 0);
    assert_eq!(big[4], 0);
    assert_eq!(fourway::frame_len(&big[..5]), Some(264));
    assert_eq!(fourway::parse_reply(&big).unwrap().params.len(), 256);
}

#[test]
fn fourway_parse_reply_rejects_every_kind_of_fault() {
    let good = [
        0x2E, 0x3A, 0x7C, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x00, 0x6E, 0xD4,
    ];
    assert_eq!(fourway::parse_reply(&[]), Err(FwParse::TooShort));
    assert_eq!(fourway::parse_reply(&good[..11]), Err(FwParse::TooShort));
    assert_eq!(fourway::parse_reply(&good[..3]), Err(FwParse::TooShort));
    let mut bad = good;
    bad[0] = 0x2F;
    assert_eq!(fourway::parse_reply(&bad), Err(FwParse::BadStart(0x2F)));
    bad = good;
    bad[6] ^= 0x10;
    assert!(matches!(
        fourway::parse_reply(&bad),
        Err(FwParse::BadCrc { .. })
    ));
    bad = good;
    bad[11] ^= 1;
    assert!(matches!(
        fourway::parse_reply(&bad),
        Err(FwParse::BadCrc { .. })
    ));
    // Command and address must be the request's (four_way.ts:400-402).
    let reply = fourway::parse_reply(&good).unwrap();
    assert!(matches!(
        reply.check(&Fw::DeviceRead {
            address: 0x7C01,
            length: 4
        }),
        Err(CheckError::Stale { .. })
    ));
    assert!(matches!(
        reply.check(&Fw::InterfaceExit),
        Err(CheckError::Stale { .. })
    ));
    assert_eq!(fourway::frame_len(&[0x2F]), Some(1));
    assert_eq!(fourway::frame_len(&[0x2E, 0x3A, 0, 0]), None);
    assert_eq!(fourway::ack_name(0x0F), "ACK_D_GENERAL_ERROR");
    assert_eq!(fourway::ack_name(0x42), "unknown acknowledge 0x42");
}

// ---------------------------------------------------------------- the read-only rule

fn every_request() -> Vec<Fw> {
    let requests = vec![
        Fw::InitFlash { esc: 0 },
        Fw::InitFlash { esc: 7 },
        Fw::DeviceRead {
            address: 0x7C00,
            length: 184,
        },
        Fw::TestAlive,
        Fw::InterfaceExit,
    ];
    // Adding a variant to the enum stops this match from compiling, and with it this test: the
    // new request has to be listed above, and the allowed set below has to be changed on purpose.
    for request in &requests {
        match request {
            Fw::InitFlash { .. } | Fw::DeviceRead { .. } | Fw::TestAlive | Fw::InterfaceExit => {}
        }
    }
    requests
}

#[test]
fn fourway_requests_are_only_init_read_test_alive_and_exit() {
    // 0x30 test-alive, 0x34 interface-exit, 0x37 init-flash, 0x3A device-read.
    let allowed = [0x30u8, 0x34, 0x37, 0x3A];
    for request in every_request() {
        let frame = request.encode().unwrap();
        assert!(
            allowed.contains(&frame[1]),
            "{}: {:02X}",
            request.name(),
            frame[1]
        );
        assert_eq!(frame[1], request.command());
    }
}

// ---------------------------------------------------------------- a scripted link

/// Answers each write with the next scripted reply (`None` is silence) and records the writes.
struct Scripted {
    script: VecDeque<Option<Vec<u8>>>,
    sent: Vec<Vec<u8>>,
    pending: VecDeque<u8>,
}

impl Scripted {
    fn new(script: Vec<Option<Vec<u8>>>) -> Self {
        Self {
            script: script.into(),
            sent: Vec::new(),
            pending: VecDeque::new(),
        }
    }
    fn four_way_frames(&self) -> Vec<&Vec<u8>> {
        self.sent
            .iter()
            .filter(|f| f.first() == Some(&0x2F))
            .collect()
    }
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

fn quick() -> Timing {
    Timing {
        after_passthrough: Duration::ZERO,
        retry_delay: Duration::ZERO,
        recover_first_delay: Duration::ZERO,
        recover_delay: Duration::ZERO,
        recover_tries: 3,
        ..Timing::configurator()
    }
}

fn options() -> Options {
    Options {
        timing: quick(),
        ..Options::new()
    }
}

const BLOCK: [u8; 184] = {
    let mut block = [0u8; 184];
    let mut i = 0;
    while i < 184 {
        block[i] = (i as u8).wrapping_mul(3).wrapping_add(1);
        i += 1;
    }
    block
};

/// The five MSP replies of a healthy board with `count` controllers.
fn msp_script(count: u8) -> Vec<Option<Vec<u8>>> {
    let mut battery = [0u8; 11];
    battery[3] = 252;
    let mut motors = [0u8; 10];
    motors[6] = count;
    vec![
        Some(msp_reply(1, &[0, 1, 42])),
        Some(msp_reply(2, b"ARDU")),
        Some(msp_reply(130, &battery)),
        Some(msp_reply(131, &motors)),
        Some(msp_reply(245, &[count])),
    ]
}

fn init_ok() -> Option<Vec<u8>> {
    Some(fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0))
}

fn read_ok(block: &[u8]) -> Option<Vec<u8>> {
    Some(fw_reply(0x3A, 0x7C00, block, 0))
}

fn exit_ok() -> Option<Vec<u8>> {
    Some(fw_reply(0x34, 0, &[0], 0))
}

fn run(link: &mut Scripted, options: &Options) -> (tool::Report, String) {
    let mut out = Vec::new();
    let report = tool::run(link, options, &mut out);
    (report, String::from_utf8(out).unwrap())
}

#[test]
fn run_on_a_healthy_scripted_board_reports_everything_and_exits_zero() {
    let mut script = msp_script(1);
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(report.api_version, Some([0, 1, 42]));
    assert_eq!(report.variant.as_deref(), Some("ARDU"));
    assert_eq!(report.battery_decivolts, Some(252));
    assert_eq!(report.motor_count, Some(1));
    assert_eq!(report.passthrough, Passthrough::Granted(1));
    let esc = &report.escs[0];
    assert_eq!(
        (esc.signature, esc.pin_code, esc.interface_mode),
        (Some(0x1F06), Some(0x14), Some(4))
    );
    assert_eq!(esc.block.as_deref(), Some(&BLOCK[..]));
    assert!(report.exit_sent && report.exit_answered);
    assert!(text.contains("battery 25.2 V"), "{text}");
    assert!(
        text.contains("signature 0x1F06, pin code 0x14, interface mode 4"),
        "{text}"
    );
    assert!(text.contains("[cmd_DeviceInitFlash] reply in"), "{text}");
    assert!(text.contains(" ms (try 1 of 10)"), "{text}");
    // The MSP requests go out in the configurator's order, then init, read, exit.
    let order: Vec<u8> = link
        .sent
        .iter()
        .map(|f| f[if f[0] == b'$' { 4 } else { 1 }])
        .collect();
    assert_eq!(order, [1, 2, 130, 131, 245, 0x37, 0x3A, 0x34]);
    assert_eq!(
        link.sent[6],
        Fw::DeviceRead {
            address: 0x7C00,
            length: 184
        }
        .encode()
        .unwrap()
    );
}

// ---------------------------------------------------------------- the retry rule

#[test]
fn init_flash_is_retried_on_every_kind_of_bad_try_and_succeeds_at_the_seventh() {
    let mut good_with_bad_crc = fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0);
    let last = good_with_bad_crc.len() - 1;
    good_with_bad_crc[last] ^= 1;
    let mut wrong_start = fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0);
    wrong_start[0] = 0x2F;
    let truncated = fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0)[..7].to_vec();
    let mut script = msp_script(1);
    script.extend([
        None,                                                     // silence
        Some(good_with_bad_crc),                                  // CRC
        Some(wrong_start),                                        // start byte
        Some(truncated), // incomplete when the time runs out
        Some(fw_reply(0x3A, 0x7C00, &[1, 2, 3, 4], 0)), // a late reply to another request
        Some(fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0x0F)), // acknowledge: general error
        init_ok(),
        read_ok(&BLOCK),
        exit_ok(),
    ]);
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 0, "{text}");
    assert!(text.contains("try 7 of 10)"), "{text}");
    for reason in [
        "no reply (silence)",
        "checksum mismatch",
        "invalid message start: 0x2F",
        "reply incomplete (7 bytes)",
        "stale reply",
        "error: ACK_D_GENERAL_ERROR",
    ] {
        assert!(text.contains(reason), "missing {reason:?} in\n{text}");
    }
    let inits = link
        .four_way_frames()
        .iter()
        .filter(|f| f[1] == 0x37)
        .count();
    assert_eq!(inits, 7);
}

#[test]
fn init_flash_gives_up_after_exactly_ten_tries_and_the_run_still_exits() {
    let mut script = msp_script(1);
    script.extend(std::iter::repeat_n(None, 10)); // ten silent tries
    script.push(Some(fw_reply(0x37, 0, &[0x06, 0x1F, 0x14, 0x04], 0))); // an eleventh would succeed
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 1, "{text}");
    let esc = &report.escs[0];
    assert!(esc.block.is_none());
    assert!(
        esc.error
            .as_deref()
            .unwrap_or("")
            .contains("max retries reached"),
        "{esc:?}"
    );
    let frames = link.four_way_frames();
    assert_eq!(frames.len(), 11, "ten init-flash tries and the exit");
    assert!(frames[..10].iter().all(|f| f[1] == 0x37));
    assert_eq!(frames[10][1], 0x34);
    assert!(text.contains("try 10 of 10 failed"), "{text}");
}

#[test]
fn the_retry_delay_is_kept_between_tries_and_not_after_the_last() {
    let mut script = msp_script(1);
    script.extend([None, None, None, init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let delayed = Options {
        timing: Timing {
            retry_delay: Duration::from_millis(40),
            ..quick()
        },
        ..Options::new()
    };
    let start = std::time::Instant::now();
    let (report, _) = run(&mut link, &delayed);
    let took = start.elapsed();
    assert_eq!(report.status(), 0);
    assert!(
        took >= Duration::from_millis(120),
        "three pauses of 40 ms, took {took:?}"
    );
    assert!(took < Duration::from_millis(1500), "took {took:?}");

    // Ten silent tries: nine pauses, none after the tenth.
    let mut script = msp_script(1);
    script.extend(std::iter::repeat_n(None, 10));
    let mut link = Scripted::new(script);
    let start = std::time::Instant::now();
    let (report, _) = run(&mut link, &delayed);
    let took = start.elapsed();
    assert_eq!(report.status(), 1);
    assert!(
        took >= Duration::from_millis(360),
        "nine pauses of 40 ms, took {took:?}"
    );
    assert!(took < Duration::from_millis(1500), "took {took:?}");
}

#[test]
fn a_read_reply_of_the_wrong_size_is_a_bad_try_and_the_read_is_retried() {
    let mut script = msp_script(1);
    script.extend([
        init_ok(),
        read_ok(&BLOCK[..100]),
        read_ok(&BLOCK),
        exit_ok(),
    ]);
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 0, "{text}");
    assert!(
        text.contains("read reply has 100 bytes, asked for 184"),
        "{text}"
    );
    assert_eq!(report.escs[0].block.as_deref(), Some(&BLOCK[..]));
}

#[test]
fn a_controller_that_does_not_answer_does_not_stop_the_next_one() {
    let mut script = msp_script(2);
    script.extend(std::iter::repeat_n(None, 10));
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 1, "{text}");
    assert!(report.escs[0].error.is_some() && report.escs[0].block.is_none());
    assert!(report.escs[1].error.is_none() && report.escs[1].block.is_some());
    // The second controller is asked by its index.
    assert_eq!(link.four_way_frames()[10][5], 1);
}

// ---------------------------------------------------------------- the refusal path

#[test]
fn a_refused_passthrough_is_reported_and_exit_status_three_and_nothing_else_is_sent() {
    let mut script = msp_script(4);
    script.pop();
    script.push(Some(vec![0x24, 0x4D, 0x21, 0x00, 0xF5, 0xF5]));
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.passthrough, Passthrough::Refused);
    assert_eq!(report.status(), 3);
    assert!(text.contains("REFUSED"), "{text}");
    assert!(report.escs.is_empty());
    assert!(!report.exit_sent);
    assert_eq!(
        link.sent.len(),
        5,
        "the five MSP requests and nothing after"
    );
    assert!(link.four_way_frames().is_empty());
}

#[test]
fn a_board_that_never_answers_is_exit_status_one_after_the_recovery_exit() {
    let mut link = Scripted::new(vec![]);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!(report.passthrough, Passthrough::NotAsked);
    // API version, interface-exit, then the three recovery tries (SerialDevice.vue:618-629).
    let order: Vec<u8> = link
        .sent
        .iter()
        .map(|f| f[if f[0] == b'$' { 4 } else { 1 }])
        .collect();
    assert_eq!(order, [1, 0x34, 1, 1, 1]);
}

#[test]
fn the_recovery_exit_lets_a_board_stuck_in_four_way_answer_msp_again() {
    let mut script = vec![None, exit_ok()];
    script.extend(msp_script(1));
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let (report, text) = run(&mut link, &options());
    assert_eq!(report.status(), 0, "{text}");
}

#[test]
fn a_silent_passthrough_request_is_status_one_and_the_exit_is_still_sent() {
    let mut script = msp_script(4);
    script.pop();
    let mut link = Scripted::new(script);
    let (report, _) = run(&mut link, &options());
    assert_eq!(report.status(), 1);
    assert!(report.exit_sent);
    assert_eq!(link.four_way_frames().len(), 1);
}

// ---------------------------------------------------------------- compare, save, options

#[test]
fn compare_reports_equal_and_the_differing_offsets() {
    assert_eq!(tool::compare(&BLOCK, &BLOCK), Comparison::Equal);
    let mut other = BLOCK;
    other[0] ^= 1;
    other[183] ^= 0x80;
    assert_eq!(
        tool::compare(&BLOCK, &other),
        Comparison::Differs {
            offsets: vec![0, 183],
            block_len: 184,
            file_len: 184
        }
    );
    assert_eq!(
        tool::compare(&BLOCK, &BLOCK[..180]),
        Comparison::Differs {
            offsets: vec![],
            block_len: 184,
            file_len: 180
        }
    );
}

#[test]
fn a_difference_is_exit_status_two_and_an_extra_file_too() {
    let mut other = BLOCK.to_vec();
    other[5] ^= 0xFF;
    let mut script = msp_script(1);
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let with_difference = Options {
        compare: vec![other],
        ..options()
    };
    let (report, text) = run(&mut link, &with_difference);
    assert_eq!(report.status(), 2, "{text}");
    assert!(
        text.contains("compare: DIFFERENT (1 byte(s) differ)"),
        "{text}"
    );
    assert!(text.contains("differing offsets 0x05"), "{text}");

    let mut script = msp_script(1);
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let extra = Options {
        compare: vec![BLOCK.to_vec(), BLOCK.to_vec()],
        ..options()
    };
    let (report, text) = run(&mut link, &extra);
    assert_eq!(report.escs[0].comparison, Comparison::Equal);
    assert_eq!(report.status(), 2, "{text}");
    assert!(text.contains("compare: equal"), "{text}");
}

#[test]
fn no_exit_sends_no_interface_exit() {
    let mut script = msp_script(1);
    script.extend([init_ok(), read_ok(&BLOCK)]);
    let mut link = Scripted::new(script);
    let (report, text) = run(
        &mut link,
        &Options {
            no_exit: true,
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert!(!report.exit_sent);
    assert!(link.four_way_frames().iter().all(|f| f[1] != 0x34));
}

#[test]
fn escs_overrides_the_count_the_board_reports() {
    let mut script = msp_script(4);
    script.extend([init_ok(), read_ok(&BLOCK), exit_ok()]);
    let mut link = Scripted::new(script);
    let (report, text) = run(
        &mut link,
        &Options {
            escs: Some(1),
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(report.escs.len(), 1);
    assert!(text.contains("--escs 1 overrides the board's 4"), "{text}");
}

#[test]
fn save_blocks_writes_escn_bin_from_one() {
    let mut script = msp_script(2);
    script.extend([
        init_ok(),
        read_ok(&BLOCK),
        init_ok(),
        read_ok(&BLOCK[..184]),
        exit_ok(),
    ]);
    let mut link = Scripted::new(script);
    let (report, _) = run(&mut link, &options());
    let dir = std::env::temp_dir().join(format!("esc-host-save-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let paths = tool::save_blocks(&dir, &report).unwrap();
    assert_eq!(paths, [dir.join("esc1.bin"), dir.join("esc2.bin")]);
    assert_eq!(std::fs::read(&paths[0]).unwrap(), BLOCK);
    assert_eq!(std::fs::read(&paths[1]).unwrap(), BLOCK);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn read_timeout_is_the_configurators_rule() {
    // four_way.ts:40, 327: max(timeout, max(2000, 120 ms per byte)).
    let timing = Timing::configurator();
    assert_eq!(timing.read_timeout(184), Duration::from_millis(22_080));
    assert_eq!(timing.read_timeout(4), Duration::from_millis(2000));
    assert_eq!(timing.tries, 10);
    assert_eq!(timing.retry_delay, Duration::from_millis(250));
    assert_eq!(timing.init_timeout, Duration::from_millis(2000));
    assert_eq!(timing.after_passthrough, Duration::from_millis(2000));
}
