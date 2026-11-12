//! The four-way interface, the tool's side: the encoder of the four requests this tool may send,
//! the parser and checks of the reply, CRC16-XMODEM. Written for this tool from the configurator
//! (`_plan/ref/am32-configurator/src/communication/four_way.ts`) and
//! `_plan/facts-esc-passthrough.md` section 6; it does not use the `esc-passthrough` crate.
//!
//! Request: `2F`, command, address high, address low, parameter count (256 is sent as 0), parameters,
//! CRC high, CRC low (`four_way.ts:144-175`). Reply: `2E`, command, address high, address low,
//! count (0 is 256), parameters, acknowledge, CRC high, CRC low (`four_way.ts:433-478`).
//!
//! READ-ONLY RULE: [`Request`] has four variants and [`Request::encode`] is the only way to build a
//! request frame. None of them writes, erases or resets a controller; the command number of each is
//! a constant in this file, and no other command number appears in the tool.

use crate::link::{ExchangeError, Link, exchange};
use std::time::Duration;

const REQUEST_START: u8 = 0x2F;
const REPLY_START: u8 = 0x2E;
const CMD_TEST_ALIVE: u8 = 0x30; // four_way.ts:87
const CMD_INTERFACE_EXIT: u8 = 0x34; // four_way.ts:91
const CMD_INIT_FLASH: u8 = 0x37; // four_way.ts:93
const CMD_DEVICE_READ: u8 = 0x3A; // four_way.ts:96

/// Parameter bytes of the largest request or reply.
pub const MAX_PARAMS: usize = 256;

/// The requests this tool can make.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// `cmd_DeviceInitFlash` for controller `esc` (0-based), address 0 (`four_way.ts:205-210`).
    InitFlash {
        /// The controller index, carried in the one parameter byte.
        esc: u8,
    },
    /// `cmd_DeviceRead` of `length` bytes (1 to 256) at the wire `address` (`four_way.ts:326-337`).
    DeviceRead {
        /// The wire address.
        address: u16,
        /// How many bytes.
        length: u16,
    },
    /// `cmd_InterfaceTestAlive`, parameter `[0]` (the default of `four_way.ts:350`).
    TestAlive,
    /// `cmd_InterfaceExit`, parameter `[0]`; no reply is awaited by the configurator
    /// (`four_way.ts:386-389`).
    InterfaceExit,
}

/// Why a request cannot be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidLength(pub u16);

impl std::fmt::Display for InvalidLength {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "read length {} is not in 1 to {MAX_PARAMS}", self.0)
    }
}

impl Request {
    /// The command number.
    pub const fn command(&self) -> u8 {
        match self {
            Self::InitFlash { .. } => CMD_INIT_FLASH,
            Self::DeviceRead { .. } => CMD_DEVICE_READ,
            Self::TestAlive => CMD_TEST_ALIVE,
            Self::InterfaceExit => CMD_INTERFACE_EXIT,
        }
    }

    /// The configurator's name for it.
    pub const fn name(&self) -> &'static str {
        match self {
            Self::InitFlash { .. } => "cmd_DeviceInitFlash",
            Self::DeviceRead { .. } => "cmd_DeviceRead",
            Self::TestAlive => "cmd_InterfaceTestAlive",
            Self::InterfaceExit => "cmd_InterfaceExit",
        }
    }

    /// The address field.
    pub const fn address(&self) -> u16 {
        match self {
            Self::DeviceRead { address, .. } => *address,
            _ => 0,
        }
    }

    /// The frame.
    ///
    /// # Errors
    ///
    /// [`InvalidLength`] for a read of 0 or more than 256 bytes.
    pub fn encode(&self) -> Result<Vec<u8>, InvalidLength> {
        let param = match self {
            Self::InitFlash { esc } => *esc,
            Self::DeviceRead { length, .. } => match *length {
                256 => 0,
                1..=255 => *length as u8,
                other => return Err(InvalidLength(other)),
            },
            Self::TestAlive | Self::InterfaceExit => 0,
        };
        let [high, low] = self.address().to_be_bytes();
        let mut frame = vec![REQUEST_START, self.command(), high, low, 1, param];
        let [crc_high, crc_low] = crc16_xmodem(&frame).to_be_bytes();
        frame.extend([crc_high, crc_low]);
        Ok(frame)
    }
}

/// CRC16-XMODEM: polynomial 0x1021, start 0, high byte first (`four_way.ts:177-189`).
pub fn crc16_xmodem(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The name of an acknowledge code (`four_way.ts:104-116`).
pub fn ack_name(ack: u8) -> String {
    match ack {
        0x00 => "ACK_OK".into(),
        0x01 => "ACK_I_UNKNOWN_ERROR".into(),
        0x02 => "ACK_I_INVALID_CMD".into(),
        0x03 => "ACK_I_INVALID_CRC".into(),
        0x04 => "ACK_I_VERIFY_ERROR".into(),
        0x05 => "ACK_D_INVALID_COMMAND".into(),
        0x06 => "ACK_D_COMMAND_FAILED".into(),
        0x07 => "ACK_D_UNKNOWN_ERROR".into(),
        0x08 => "ACK_I_INVALID_CHANNEL".into(),
        0x09 => "ACK_I_INVALID_PARAM".into(),
        0x0F => "ACK_D_GENERAL_ERROR".into(),
        other => format!("unknown acknowledge 0x{other:02X}"),
    }
}

/// A reply frame that passed the parser (start, length, CRC). It is not yet known to answer a
/// given request: see [`Reply::check`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reply {
    /// The command echoed.
    pub command: u8,
    /// The address echoed.
    pub address: u16,
    /// The parameters (1 to 256 bytes).
    pub params: Vec<u8>,
    /// The acknowledge code.
    pub ack: u8,
}

/// Why bytes are not a reply frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// The first byte is not `2E` (`four_way.ts:437-440`).
    BadStart(u8),
    /// Fewer bytes than the frame needs (`four_way.ts:442-453`).
    TooShort,
    /// The CRC is wrong (`four_way.ts:466-472`).
    BadCrc {
        /// Computed.
        expected: u16,
        /// Received.
        received: u16,
    },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadStart(b) => write!(f, "invalid message start: 0x{b:02X}"),
            Self::TooShort => write!(f, "not enough data"),
            Self::BadCrc { expected, received } => write!(
                f,
                "checksum mismatch, received 0x{received:04X}, calculated 0x{expected:04X}"
            ),
        }
    }
}

/// Once enough of a reply has arrived to know its length, the length of the whole frame. A first
/// byte that is not `2E` gives 1, so the reader stops at once and the parser names the fault.
pub fn frame_len(received: &[u8]) -> Option<usize> {
    match received.first() {
        None => None,
        Some(&REPLY_START) => received.get(4).map(|&count| {
            8 + if count == 0 {
                MAX_PARAMS
            } else {
                usize::from(count)
            }
        }),
        Some(_) => Some(1),
    }
}

/// Parses one reply frame the way `four_way.ts:433-478` does.
///
/// # Errors
///
/// [`ParseError`].
pub fn parse_reply(bytes: &[u8]) -> Result<Reply, ParseError> {
    let first = *bytes.first().ok_or(ParseError::TooShort)?;
    if first != REPLY_START {
        return Err(ParseError::BadStart(first));
    }
    let count = match bytes.get(4) {
        None => return Err(ParseError::TooShort),
        Some(0) => MAX_PARAMS,
        Some(&n) => usize::from(n),
    };
    let (Some(&command), Some(&addr_high), Some(&addr_low), Some(params), Some(&ack)) = (
        bytes.get(1),
        bytes.get(2),
        bytes.get(3),
        bytes.get(5..5 + count),
        bytes.get(5 + count),
    ) else {
        return Err(ParseError::TooShort);
    };
    let (Some(&crc_high), Some(&crc_low), Some(covered)) = (
        bytes.get(6 + count),
        bytes.get(7 + count),
        bytes.get(..6 + count),
    ) else {
        return Err(ParseError::TooShort);
    };
    let received = u16::from_be_bytes([crc_high, crc_low]);
    let expected = crc16_xmodem(covered);
    if expected != received {
        return Err(ParseError::BadCrc { expected, received });
    }
    Ok(Reply {
        command,
        address: u16::from_be_bytes([addr_high, addr_low]),
        params: params.to_vec(),
        ack,
    })
}

/// Why a well-formed reply is not an acceptable answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CheckError {
    /// The command or the address is not the request's: a late reply to an earlier request
    /// (`four_way.ts:394-402`).
    Stale {
        /// Command and address of the reply.
        got: (u8, u16),
        /// Command and address of the request.
        wanted: (u8, u16),
    },
    /// The acknowledge is not `ACK_OK` (`four_way.ts:403`).
    Ack(u8),
}

impl std::fmt::Display for CheckError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stale { got, wanted } => write!(
                f,
                "stale reply for command 0x{:02X} @0x{:X}, wanted 0x{:02X} @0x{:X}, discarded",
                got.0, got.1, wanted.0, wanted.1
            ),
            Self::Ack(ack) => write!(f, "error: {}", ack_name(*ack)),
        }
    }
}

impl Reply {
    /// Command and address must equal the request's, and the acknowledge must be `ACK_OK`.
    ///
    /// # Errors
    ///
    /// [`CheckError`].
    pub fn check(&self, request: &Request) -> Result<(), CheckError> {
        if self.command != request.command() || self.address != request.address() {
            return Err(CheckError::Stale {
                got: (self.command, self.address),
                wanted: (request.command(), request.address()),
            });
        }
        if self.ack != 0 {
            return Err(CheckError::Ack(self.ack));
        }
        Ok(())
    }
}

/// Why one try gave no accepted reply.
#[derive(Debug)]
pub enum Error {
    /// No frame arrived.
    Exchange(ExchangeError),
    /// A frame arrived and is not a reply frame.
    Invalid(ParseError),
    /// A reply frame, but not an acceptable answer.
    Rejected(CheckError),
    /// The request cannot be encoded.
    Encode(InvalidLength),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Exchange(error) => error.fmt(f),
            Self::Invalid(error) => error.fmt(f),
            Self::Rejected(error) => error.fmt(f),
            Self::Encode(error) => error.fmt(f),
        }
    }
}

/// One try: sends `request`, waits at most `timeout` for a frame, parses and checks it. The
/// parameters of the reply are not looked at here.
///
/// # Errors
///
/// [`Error`].
pub fn transact<L: Link + ?Sized>(
    link: &mut L,
    request: &Request,
    timeout: Duration,
) -> Result<(Reply, Duration), Error> {
    let frame = request.encode().map_err(Error::Encode)?;
    let (bytes, took) = exchange(link, &frame, timeout, frame_len).map_err(Error::Exchange)?;
    let reply = parse_reply(&bytes).map_err(Error::Invalid)?;
    reply.check(request).map_err(Error::Rejected)?;
    Ok((reply, took))
}
