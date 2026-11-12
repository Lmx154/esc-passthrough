//! MSP v1: the requests the AM32 configurator sends a flight controller and the replies it reads.
//!
//! Frame: `$` `M` direction length command payload checksum. The direction is `<` for a request,
//! `>` for a reply and `!` for an error reply; the checksum is the XOR of length, command and
//! payload. Only v1 is handled: a `$X` (v2) frame is a framing error.
//!
//! The configurator needs five messages: [`MSP_API_VERSION`], [`MSP_FC_VARIANT`],
//! [`MSP_BATTERY_STATE`], [`MSP_MOTOR_CONFIG`] and [`MSP_SET_PASSTHROUGH`]. Any other command
//! number parses and is reported as [`Command::Unsupported`]; this module has no reply for it
//! (use [`encode_error`] if the application wants to refuse).

use crate::EncodeError;
use crate::error::Writer;

/// `MSP_API_VERSION`.
pub const MSP_API_VERSION: u8 = 1;
/// `MSP_FC_VARIANT`.
pub const MSP_FC_VARIANT: u8 = 2;
/// `MSP_BATTERY_STATE`.
pub const MSP_BATTERY_STATE: u8 = 130;
/// `MSP_MOTOR_CONFIG`.
pub const MSP_MOTOR_CONFIG: u8 = 131;
/// `MSP_SET_PASSTHROUGH`.
pub const MSP_SET_PASSTHROUGH: u8 = 245;

/// Largest payload of an MSP v1 frame (the length is one byte). The request parser stores this
/// much.
pub const MAX_PAYLOAD: usize = 255;
/// Bytes around the payload: `$`, `M`, direction, length, command, checksum.
pub const FRAME_OVERHEAD: usize = 6;
/// Longest MSP v1 frame.
pub const MAX_FRAME_LEN: usize = MAX_PAYLOAD + FRAME_OVERHEAD;
/// Payload length of the `MSP_BATTERY_STATE` reply.
pub const BATTERY_STATE_PAYLOAD_LEN: usize = 11;
/// Payload length of the `MSP_MOTOR_CONFIG` reply.
pub const MOTOR_CONFIG_PAYLOAD_LEN: usize = 10;

/// A request command number, sorted into the five this crate knows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Command {
    /// [`MSP_API_VERSION`].
    ApiVersion,
    /// [`MSP_FC_VARIANT`].
    FcVariant,
    /// [`MSP_BATTERY_STATE`].
    BatteryState,
    /// [`MSP_MOTOR_CONFIG`].
    MotorConfig,
    /// [`MSP_SET_PASSTHROUGH`].
    SetPassthrough,
    /// Any other command number; the crate does not answer it.
    Unsupported(u8),
}

impl Command {
    /// Sorts a command number.
    pub const fn from_code(code: u8) -> Self {
        match code {
            MSP_API_VERSION => Self::ApiVersion,
            MSP_FC_VARIANT => Self::FcVariant,
            MSP_BATTERY_STATE => Self::BatteryState,
            MSP_MOTOR_CONFIG => Self::MotorConfig,
            MSP_SET_PASSTHROUGH => Self::SetPassthrough,
            other => Self::Unsupported(other),
        }
    }

    /// The command number.
    pub const fn code(self) -> u8 {
        match self {
            Self::ApiVersion => MSP_API_VERSION,
            Self::FcVariant => MSP_FC_VARIANT,
            Self::BatteryState => MSP_BATTERY_STATE,
            Self::MotorConfig => MSP_MOTOR_CONFIG,
            Self::SetPassthrough => MSP_SET_PASSTHROUGH,
            Self::Unsupported(code) => code,
        }
    }
}

/// A complete, checksum-verified request. The payload borrows the parser's storage and is valid
/// until the parser is used again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request<'a> {
    /// The command number as received.
    pub command: u8,
    /// The payload (empty for the configurator's five requests).
    pub payload: &'a [u8],
}

impl Request<'_> {
    /// The command number sorted into [`Command`].
    pub const fn kind(&self) -> Command {
        Command::from_code(self.command)
    }
}

/// Why the parser rejected bytes. After either error the parser is ready for the next frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ParseError {
    /// A `$` was not followed by `M` and then `<` (a reply, an error reply or a v2 frame, or
    /// noise).
    BadHeader,
    /// The checksum byte did not match.
    BadChecksum {
        /// The checksum computed over length, command and payload.
        expected: u8,
        /// The checksum byte received.
        received: u8,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum State {
    Idle,
    Dollar,
    M,
    Length,
    Command,
    Payload,
    Checksum,
}

enum Step {
    Pending,
    Frame,
    Error(ParseError),
}

/// Incremental MSP v1 request parser. Feed it bytes one at a time with [`push`](Self::push) or in
/// slices with [`feed`](Self::feed), split anywhere.
///
/// Bytes outside a frame are skipped until a `$`. A frame is consumed by its declared length: a
/// payload may contain `$`, and a corrupted length byte can make the parser swallow part of the
/// next frame (the checksum then fails and the parser resynchronises on the next `$`). A byte
/// that ends a failed frame or breaks a header is looked at again as a possible `$`, so a frame
/// start is never lost to the error before it.
///
/// Storage: 255 payload bytes (the largest MSP v1 payload) plus a few counters, inside the
/// struct; nothing is allocated.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RequestParser {
    state: State,
    command: u8,
    len: usize,
    pos: usize,
    checksum: u8,
    buf: [u8; MAX_PAYLOAD],
}

impl Default for RequestParser {
    fn default() -> Self {
        Self::new()
    }
}

impl RequestParser {
    /// A parser waiting for a `$`.
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            command: 0,
            len: 0,
            pos: 0,
            checksum: 0,
            buf: [0; MAX_PAYLOAD],
        }
    }

    /// Drops a partly received frame, for example after a pause in the stream.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Feeds one byte. `None`: nothing to report yet. `Some(Ok(request))`: a complete request.
    /// `Some(Err(e))`: a framing or checksum error.
    pub fn push(&mut self, byte: u8) -> Option<Result<Request<'_>, ParseError>> {
        match self.step(byte) {
            Step::Pending => None,
            Step::Frame => Some(Ok(self.request())),
            Step::Error(error) => Some(Err(error)),
        }
    }

    /// Feeds bytes until the first request or error. Returns how many bytes were consumed (the
    /// one that completed the event included) and the event. Call again with the rest of the
    /// slice; `(bytes.len(), None)` means all of it was consumed with nothing to report.
    pub fn feed(&mut self, bytes: &[u8]) -> (usize, Option<Result<Request<'_>, ParseError>>) {
        for (index, &byte) in bytes.iter().enumerate() {
            match self.step(byte) {
                Step::Pending => {}
                Step::Frame => return (index + 1, Some(Ok(self.request()))),
                Step::Error(error) => return (index + 1, Some(Err(error))),
            }
        }
        (bytes.len(), None)
    }

    fn request(&self) -> Request<'_> {
        Request {
            command: self.command,
            payload: self.buf.get(..self.len).unwrap_or(&[]),
        }
    }

    fn restart_on(byte: u8) -> State {
        if byte == b'$' {
            State::Dollar
        } else {
            State::Idle
        }
    }

    fn header_error(&mut self, byte: u8) -> Step {
        self.state = Self::restart_on(byte);
        Step::Error(ParseError::BadHeader)
    }

    fn step(&mut self, byte: u8) -> Step {
        match self.state {
            State::Idle => {
                if byte == b'$' {
                    self.state = State::Dollar;
                }
                Step::Pending
            }
            State::Dollar => {
                if byte == b'M' {
                    self.state = State::M;
                    Step::Pending
                } else {
                    self.header_error(byte)
                }
            }
            State::M => {
                if byte == b'<' {
                    self.state = State::Length;
                    Step::Pending
                } else {
                    self.header_error(byte)
                }
            }
            State::Length => {
                self.len = usize::from(byte);
                self.pos = 0;
                self.checksum = byte;
                self.state = State::Command;
                Step::Pending
            }
            State::Command => {
                self.command = byte;
                self.checksum ^= byte;
                self.state = if self.len == 0 {
                    State::Checksum
                } else {
                    State::Payload
                };
                Step::Pending
            }
            State::Payload => {
                if let Some(slot) = self.buf.get_mut(self.pos) {
                    *slot = byte;
                }
                self.pos += 1;
                self.checksum ^= byte;
                if self.pos >= self.len {
                    self.state = State::Checksum;
                }
                Step::Pending
            }
            State::Checksum => {
                if byte == self.checksum {
                    self.state = State::Idle;
                    Step::Frame
                } else {
                    self.state = Self::restart_on(byte);
                    Step::Error(ParseError::BadChecksum {
                        expected: self.checksum,
                        received: byte,
                    })
                }
            }
        }
    }
}

fn encode_frame(
    direction: u8,
    command: u8,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let length = u8::try_from(payload.len()).map_err(|_| EncodeError::InvalidLength)?;
    let mut writer = Writer::new(out, payload.len() + FRAME_OVERHEAD)?;
    let checksum = payload.iter().fold(length ^ command, |acc, &b| acc ^ b);
    writer.put(b'$');
    writer.put(b'M');
    writer.put(direction);
    writer.put(length);
    writer.put(command);
    writer.put_all(payload);
    writer.put(checksum);
    Ok(writer.len())
}

/// Writes a reply frame (`$M>`) with any command number and payload. Returns the frame length
/// (`payload.len() + 6`).
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] for a payload over 255 bytes, [`EncodeError::BufferTooSmall`]
/// if `out` is shorter than the frame.
pub fn encode_reply(command: u8, payload: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_frame(b'>', command, payload, out)
}

/// Writes the MSP error reply for `command`: `$M!`, length 0, the command, checksum (6 bytes).
/// This is the form the configurator's own simulator answers with (`ci/bootloader_sim.py`); how the
/// configurator reacts to it has not been run.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 6 bytes.
pub fn encode_error(command: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_frame(b'!', command, &[], out)
}

/// Contents of the `MSP_API_VERSION` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ApiVersion {
    /// MSP protocol version byte (0 for MSP v1 on Betaflight and ArduPilot).
    pub protocol: u8,
    /// API major version.
    pub major: u8,
    /// API minor version.
    pub minor: u8,
}

impl ApiVersion {
    /// An API version triple.
    pub const fn new(protocol: u8, major: u8, minor: u8) -> Self {
        Self {
            protocol,
            major,
            minor,
        }
    }
}

impl Default for ApiVersion {
    fn default() -> Self {
        Self::new(0, 0, 0)
    }
}

/// Writes the `MSP_API_VERSION` reply: payload `[protocol, major, minor]` (9 bytes in all).
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_api_version(version: ApiVersion, out: &mut [u8]) -> Result<usize, EncodeError> {
    let payload = [version.protocol, version.major, version.minor];
    encode_reply(MSP_API_VERSION, &payload, out)
}

/// Writes the `MSP_FC_VARIANT` reply: the four ASCII bytes of the variant, for example `*b"ARDU"`
/// (10 bytes in all). The configurator knows `BTFL`, `KISS`, `INAV`, `ARDU` and `EMUF`; any other
/// four bytes make it log an error.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_fc_variant(variant: [u8; 4], out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_reply(MSP_FC_VARIANT, &variant, out)
}

/// Contents of the `MSP_BATTERY_STATE` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct BatteryState {
    /// Battery voltage in millivolts.
    pub millivolts: u32,
    /// Battery current in milliamps, if known (`None` is sent as 0).
    pub current_ma: Option<u32>,
    /// Charge used in milliamp hours, if known (`None` is sent as 0).
    pub charge_mah: Option<u32>,
}

impl BatteryState {
    /// A battery at `millivolts` with no current and no charge figure.
    pub const fn new(millivolts: u32) -> Self {
        Self {
            millivolts,
            current_ma: None,
            charge_mah: None,
        }
    }

    /// Sets the current in milliamps.
    pub const fn with_current_ma(mut self, current_ma: Option<u32>) -> Self {
        self.current_ma = current_ma;
        self
    }

    /// Sets the charge used in milliamp hours.
    pub const fn with_charge_mah(mut self, charge_mah: Option<u32>) -> Self {
        self.charge_mah = charge_mah;
        self
    }
}

impl Default for BatteryState {
    fn default() -> Self {
        Self::new(0)
    }
}

/// `value / divisor` rounded to nearest (ties up), `divisor` not 0, limited to `max`.
fn scaled(value: u32, divisor: u32, max: u32) -> u32 {
    let rounded = value.saturating_add(divisor / 2) / divisor;
    rounded.min(max)
}

fn le16(value: u32) -> [u8; 2] {
    u16::try_from(value).unwrap_or(u16::MAX).to_le_bytes()
}

/// Writes the `MSP_BATTERY_STATE` reply (11-byte payload, 17 bytes in all), laid out the way the
/// configurator and ArduPilot use it:
///
/// | Bytes | Content |
/// |---|---|
/// | 0 | cell count: always 0 (not provided) |
/// | 1-2 | capacity in mAh, little endian: always 0 (not provided) |
/// | 3 | voltage in 0.1 V, rounded, saturating at 255 (25.5 V); the configurator reads this one |
/// | 4-5 | charge used in mAh, little endian, saturating |
/// | 6-7 | current in 0.01 A, rounded, little endian, saturating |
/// | 8 | state: 0 |
/// | 9-10 | voltage in 0.01 V, rounded, little endian, saturating |
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_battery_state(state: BatteryState, out: &mut [u8]) -> Result<usize, EncodeError> {
    let decivolts = u8::try_from(scaled(state.millivolts, 100, 255)).unwrap_or(u8::MAX);
    let charge = le16(state.charge_mah.unwrap_or(0));
    let current = le16(scaled(state.current_ma.unwrap_or(0), 10, 65_535));
    let centivolts = le16(scaled(state.millivolts, 10, 65_535));
    let [charge_lo, charge_hi] = charge;
    let [current_lo, current_hi] = current;
    let [volts_lo, volts_hi] = centivolts;
    let payload: [u8; BATTERY_STATE_PAYLOAD_LEN] = [
        0, 0, 0, decivolts, charge_lo, charge_hi, current_lo, current_hi, 0, volts_lo, volts_hi,
    ];
    encode_reply(MSP_BATTERY_STATE, &payload, out)
}

/// Contents of the `MSP_MOTOR_CONFIG` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct MotorConfig {
    /// Number of motors (ESCs) the flight controller drives.
    pub motor_count: u8,
}

impl MotorConfig {
    /// A motor configuration with `motor_count` motors.
    pub const fn new(motor_count: u8) -> Self {
        Self { motor_count }
    }
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self::new(0)
    }
}

/// Writes the `MSP_MOTOR_CONFIG` reply (10-byte payload, 16 bytes in all): byte 6 is the motor
/// count, which is all the configurator reads; the throttle limits (bytes 0-5), pole count and
/// feature flags (bytes 7-9) are 0.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_motor_config(config: MotorConfig, out: &mut [u8]) -> Result<usize, EncodeError> {
    let payload: [u8; MOTOR_CONFIG_PAYLOAD_LEN] = [0, 0, 0, 0, 0, 0, config.motor_count, 0, 0, 0];
    encode_reply(MSP_MOTOR_CONFIG, &payload, out)
}

/// Writes the `MSP_SET_PASSTHROUGH` reply: one byte, the number of ESCs the four-way interface
/// will offer (7 bytes in all).
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_set_passthrough(esc_count: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_reply(MSP_SET_PASSTHROUGH, &[esc_count], out)
}
