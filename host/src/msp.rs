//! MSP v1, the tool's side: the encoder of the five requests the configurator sends and the parser
//! of the reply. Written for this tool from the configurator's behaviour
//! (`_plan/ref/am32-configurator/components/SerialDevice.vue:618-655`, `src/communication/
//! commands.queue.ts:57-119`) and `_plan/facts-esc-passthrough.md` section 5; it does not use the
//! `esc-passthrough` crate.
//!
//! Frame: `$` `M` direction length command payload checksum; direction `<` request, `>` reply,
//! `!` error reply; checksum = XOR of length, command and payload. The only requests this module
//! can build are the five of [`Request`], none with a payload.

use crate::link::{ExchangeError, Link, exchange};
use std::time::Duration;

/// The requests of the configurator's connect sequence, and nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// `MSP_API_VERSION` (1), `SerialDevice.vue:618`.
    ApiVersion,
    /// `MSP_FC_VARIANT` (2), `SerialDevice.vue:639`.
    FcVariant,
    /// `MSP_BATTERY_STATE` (130), `SerialDevice.vue:644`.
    BatteryState,
    /// `MSP_MOTOR_CONFIG` (131), `SerialDevice.vue:649` (the configurator asks `MSP_MOTOR` of
    /// INAV and EMUF; this tool does not).
    MotorConfig,
    /// `MSP_SET_PASSTHROUGH` (245), `SerialDevice.vue:655`.
    SetPassthrough,
}

impl Request {
    /// The command number.
    pub const fn code(self) -> u8 {
        match self {
            Self::ApiVersion => 1,
            Self::FcVariant => 2,
            Self::BatteryState => 130,
            Self::MotorConfig => 131,
            Self::SetPassthrough => 245,
        }
    }

    /// The configurator's name for it.
    pub const fn name(self) -> &'static str {
        match self {
            Self::ApiVersion => "MSP_API_VERSION",
            Self::FcVariant => "MSP_FC_VARIANT",
            Self::BatteryState => "MSP_BATTERY_STATE",
            Self::MotorConfig => "MSP_MOTOR_CONFIG",
            Self::SetPassthrough => "MSP_SET_PASSTHROUGH",
        }
    }

    /// The request frame: `$ M < 0 command command` (length 0, so the checksum is the command).
    pub const fn encode(self) -> [u8; 6] {
        let code = self.code();
        [b'$', b'M', b'<', 0, code, code]
    }
}

/// A reply that passed every check.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// Direction `>`: the payload.
    Data(Vec<u8>),
    /// Direction `!`: the flight controller refuses the request.
    Refused,
}

/// Why bytes are not the reply to the request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// The first byte is not `$`.
    BadStart(u8),
    /// Fewer bytes than the frame needs.
    TooShort,
    /// The second byte is not `M`.
    BadMarker(u8),
    /// The direction byte is neither `>` nor `!`.
    BadDirection(u8),
    /// The checksum byte is wrong.
    BadChecksum {
        /// Computed over length, command and payload.
        expected: u8,
        /// The byte received.
        received: u8,
    },
    /// The reply is to another command.
    WrongCommand {
        /// The request's command.
        expected: u8,
        /// The reply's command.
        got: u8,
    },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadStart(b) => write!(f, "invalid start byte 0x{b:02X}"),
            Self::TooShort => write!(f, "frame too short"),
            Self::BadMarker(b) => write!(f, "second byte 0x{b:02X} is not 'M'"),
            Self::BadDirection(b) => write!(f, "direction byte 0x{b:02X} is neither '>' nor '!'"),
            Self::BadChecksum { expected, received } => write!(
                f,
                "checksum mismatch, received 0x{received:02X}, calculated 0x{expected:02X}"
            ),
            Self::WrongCommand { expected, got } => {
                write!(f, "reply is to command {got}, expected {expected}")
            }
        }
    }
}

/// XOR of all bytes.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, byte| acc ^ byte)
}

/// Once enough of a reply has arrived to know its length, the length of the whole frame. A first
/// byte that is not `$` gives 1, so the reader stops at once and the parser names the fault.
pub fn frame_len(received: &[u8]) -> Option<usize> {
    match received.first() {
        None => None,
        Some(b'$') => received.get(3).map(|&len| 6 + usize::from(len)),
        Some(_) => Some(1),
    }
}

/// Checks a reply the way the configurator's parser does: start byte, `M`, direction, length,
/// checksum, and that it answers `request`.
///
/// # Errors
///
/// [`ParseError`].
pub fn parse_reply(bytes: &[u8], request: Request) -> Result<Reply, ParseError> {
    let first = *bytes.first().ok_or(ParseError::TooShort)?;
    if first != b'$' {
        return Err(ParseError::BadStart(first));
    }
    let (Some(&marker), Some(&direction), Some(&len), Some(&command)) =
        (bytes.get(1), bytes.get(2), bytes.get(3), bytes.get(4))
    else {
        return Err(ParseError::TooShort);
    };
    if marker != b'M' {
        return Err(ParseError::BadMarker(marker));
    }
    if direction != b'>' && direction != b'!' {
        return Err(ParseError::BadDirection(direction));
    }
    let total = 6 + usize::from(len);
    let (Some(covered), Some(&received)) = (bytes.get(3..total - 1), bytes.get(total - 1)) else {
        return Err(ParseError::TooShort);
    };
    let expected = checksum(covered);
    if expected != received {
        return Err(ParseError::BadChecksum { expected, received });
    }
    if command != request.code() {
        return Err(ParseError::WrongCommand {
            expected: request.code(),
            got: command,
        });
    }
    if direction == b'!' {
        return Ok(Reply::Refused);
    }
    Ok(Reply::Data(
        bytes.get(5..total - 1).unwrap_or_default().to_vec(),
    ))
}

/// Why a request got no usable reply.
#[derive(Debug)]
pub enum Error {
    /// No frame arrived.
    Exchange(ExchangeError),
    /// A frame arrived and is not the reply.
    Invalid(ParseError),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Exchange(error) => error.fmt(f),
            Self::Invalid(error) => error.fmt(f),
        }
    }
}

/// Sends `request` and returns its validated reply and the time it took.
///
/// # Errors
///
/// [`Error`].
pub fn transact<L: Link + ?Sized>(
    link: &mut L,
    request: Request,
    timeout: Duration,
) -> Result<(Reply, Duration), Error> {
    let (frame, took) =
        exchange(link, &request.encode(), timeout, frame_len).map_err(Error::Exchange)?;
    let reply = parse_reply(&frame, request).map_err(Error::Invalid)?;
    Ok((reply, took))
}
