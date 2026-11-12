//! The four-way interface: the framing between the configurator and the flight controller after
//! `MSP_SET_PASSTHROUGH`.
//!
//! Request: `0x2F`, command, address high, address low, parameter count (0 means 256),
//! parameters, CRC16 high, CRC16 low. Reply: `0x2E`, command, address high, address low,
//! parameter count (0 means 256), parameters, acknowledge, CRC16 high, CRC16 low. The CRC is
//! CRC16-XMODEM (polynomial 0x1021, start 0, high byte first) over everything before it.
//!
//! A frame always carries at least one parameter (the configurator sends one `0x00` where it has
//! none) and at most 256. Parameters are opaque bytes here.

use crate::EncodeError;
use crate::error::Writer;

/// First byte of a request (the configurator to the flight controller).
pub const REQUEST_START: u8 = 0x2F;
/// First byte of a reply.
pub const REPLY_START: u8 = 0x2E;
/// Most parameter bytes a frame can carry. The request parser stores this much.
pub const MAX_PARAMS: usize = 256;
/// Bytes of a request around its parameters: start, command, address (2), count, CRC (2).
pub const REQUEST_OVERHEAD: usize = 7;
/// Bytes of a reply around its parameters: start, command, address (2), count, acknowledge, CRC
/// (2).
pub const REPLY_OVERHEAD: usize = 8;
/// Longest request.
pub const MAX_REQUEST_LEN: usize = MAX_PARAMS + REQUEST_OVERHEAD;
/// Longest reply.
pub const MAX_REPLY_LEN: usize = MAX_PARAMS + REPLY_OVERHEAD;

/// One step of CRC16-XMODEM (polynomial 0x1021, MSB first).
pub const fn crc16_xmodem_update(crc: u16, byte: u8) -> u16 {
    let mut crc = crc ^ ((byte as u16) << 8);
    let mut bit = 0;
    while bit < 8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ 0x1021
        } else {
            crc << 1
        };
        bit += 1;
    }
    crc
}

/// CRC16-XMODEM of `bytes` with start value 0. The check value of `b"123456789"` is 0x31C3.
pub const fn crc16_xmodem(bytes: &[u8]) -> u16 {
    let mut crc = 0;
    let mut rest = bytes;
    while let [byte, tail @ ..] = rest {
        crc = crc16_xmodem_update(crc, *byte);
        rest = tail;
    }
    crc
}

/// A four-way command code: every code the protocol header defines, plus the codes it does not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// 0x30 `cmd_InterfaceTestAlive`.
    InterfaceTestAlive,
    /// 0x31 `cmd_ProtocolGetVersion`.
    ProtocolGetVersion,
    /// 0x32 `cmd_InterfaceGetName`.
    InterfaceGetName,
    /// 0x33 `cmd_InterfaceGetVersion`.
    InterfaceGetVersion,
    /// 0x34 `cmd_InterfaceExit`.
    InterfaceExit,
    /// 0x35 `cmd_DeviceReset`.
    DeviceReset,
    /// 0x37 `cmd_DeviceInitFlash`.
    DeviceInitFlash,
    /// 0x38 `cmd_DeviceEraseAll`.
    DeviceEraseAll,
    /// 0x39 `cmd_DevicePageErase`.
    DevicePageErase,
    /// 0x3A `cmd_DeviceRead`.
    DeviceRead,
    /// 0x3B `cmd_DeviceWrite`.
    DeviceWrite,
    /// 0x3C `cmd_DeviceC2CK_LOW`.
    DeviceC2ckLow,
    /// 0x3D `cmd_DeviceReadEEprom`.
    DeviceReadEeprom,
    /// 0x3E `cmd_DeviceWriteEEprom`.
    DeviceWriteEeprom,
    /// 0x3F `cmd_InterfaceSetMode`.
    InterfaceSetMode,
    /// 0x40 `cmd_DeviceVerify`.
    DeviceVerify,
    /// A code the protocol header does not define (0x36, the removed `cmd_DeviceGetID`, is one).
    /// [`Command::from_code`] never builds this variant for a defined code.
    Unknown(u8),
}

impl Command {
    /// Sorts a command code.
    pub const fn from_code(code: u8) -> Self {
        match code {
            0x30 => Self::InterfaceTestAlive,
            0x31 => Self::ProtocolGetVersion,
            0x32 => Self::InterfaceGetName,
            0x33 => Self::InterfaceGetVersion,
            0x34 => Self::InterfaceExit,
            0x35 => Self::DeviceReset,
            0x37 => Self::DeviceInitFlash,
            0x38 => Self::DeviceEraseAll,
            0x39 => Self::DevicePageErase,
            0x3A => Self::DeviceRead,
            0x3B => Self::DeviceWrite,
            0x3C => Self::DeviceC2ckLow,
            0x3D => Self::DeviceReadEeprom,
            0x3E => Self::DeviceWriteEeprom,
            0x3F => Self::InterfaceSetMode,
            0x40 => Self::DeviceVerify,
            other => Self::Unknown(other),
        }
    }

    /// The command code.
    pub const fn code(self) -> u8 {
        match self {
            Self::InterfaceTestAlive => 0x30,
            Self::ProtocolGetVersion => 0x31,
            Self::InterfaceGetName => 0x32,
            Self::InterfaceGetVersion => 0x33,
            Self::InterfaceExit => 0x34,
            Self::DeviceReset => 0x35,
            Self::DeviceInitFlash => 0x37,
            Self::DeviceEraseAll => 0x38,
            Self::DevicePageErase => 0x39,
            Self::DeviceRead => 0x3A,
            Self::DeviceWrite => 0x3B,
            Self::DeviceC2ckLow => 0x3C,
            Self::DeviceReadEeprom => 0x3D,
            Self::DeviceWriteEeprom => 0x3E,
            Self::InterfaceSetMode => 0x3F,
            Self::DeviceVerify => 0x40,
            Self::Unknown(code) => code,
        }
    }
}

/// A four-way acknowledge code (the byte before the CRC of a reply).
///
/// 0x00, 0x02, 0x03, 0x04, 0x08, 0x09 and 0x0F are in the protocol header; 0x01, 0x05, 0x06 and
/// 0x07 are only in the configurator's table (`four_way.ts`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Ack {
    /// 0x00 `ACK_OK`.
    Ok,
    /// 0x01 `ACK_I_UNKNOWN_ERROR`.
    IUnknownError,
    /// 0x02 `ACK_I_INVALID_CMD`.
    IInvalidCmd,
    /// 0x03 `ACK_I_INVALID_CRC`.
    IInvalidCrc,
    /// 0x04 `ACK_I_VERIFY_ERROR`.
    IVerifyError,
    /// 0x05 `ACK_D_INVALID_COMMAND`.
    DInvalidCommand,
    /// 0x06 `ACK_D_COMMAND_FAILED`.
    DCommandFailed,
    /// 0x07 `ACK_D_UNKNOWN_ERROR`.
    DUnknownError,
    /// 0x08 `ACK_I_INVALID_CHANNEL`.
    IInvalidChannel,
    /// 0x09 `ACK_I_INVALID_PARAM`.
    IInvalidParam,
    /// 0x0F `ACK_D_GENERAL_ERROR`.
    DGeneralError,
    /// Any other code. [`Ack::from_code`] never builds this variant for a defined code.
    Unknown(u8),
}

impl Ack {
    /// Sorts an acknowledge code.
    pub const fn from_code(code: u8) -> Self {
        match code {
            0x00 => Self::Ok,
            0x01 => Self::IUnknownError,
            0x02 => Self::IInvalidCmd,
            0x03 => Self::IInvalidCrc,
            0x04 => Self::IVerifyError,
            0x05 => Self::DInvalidCommand,
            0x06 => Self::DCommandFailed,
            0x07 => Self::DUnknownError,
            0x08 => Self::IInvalidChannel,
            0x09 => Self::IInvalidParam,
            0x0F => Self::DGeneralError,
            other => Self::Unknown(other),
        }
    }

    /// The acknowledge code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Ok => 0x00,
            Self::IUnknownError => 0x01,
            Self::IInvalidCmd => 0x02,
            Self::IInvalidCrc => 0x03,
            Self::IVerifyError => 0x04,
            Self::DInvalidCommand => 0x05,
            Self::DCommandFailed => 0x06,
            Self::DUnknownError => 0x07,
            Self::IInvalidChannel => 0x08,
            Self::IInvalidParam => 0x09,
            Self::DGeneralError => 0x0F,
            Self::Unknown(code) => code,
        }
    }
}

/// A complete, CRC-verified request. The parameters borrow the parser's storage and are valid
/// until the parser is used again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request<'a> {
    /// The command; a code the header does not define is [`Command::Unknown`].
    pub command: Command,
    /// The address field, high byte first on the wire.
    pub address: u16,
    /// The parameters, 1 to 256 bytes.
    pub params: &'a [u8],
}

/// Why the parser rejected a frame. The parser is ready for the next frame afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ParseError {
    /// The CRC did not match.
    BadCrc {
        /// The CRC computed over the frame.
        expected: u16,
        /// The CRC received.
        received: u16,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum State {
    Idle,
    Command,
    AddressHigh,
    AddressLow,
    Count,
    Params,
    CrcHigh,
    CrcLow,
}

enum Step {
    Pending,
    Frame,
    Error(ParseError),
}

/// Incremental four-way request parser. Feed it bytes one at a time with [`push`](Self::push) or
/// in slices with [`feed`](Self::feed), split anywhere.
///
/// Bytes outside a frame are skipped until `0x2F`. A frame is consumed by its declared count: a
/// parameter may be `0x2F`, and a corrupted count can make the parser swallow part of the next
/// frame (the CRC then fails and the parser resynchronises on the next `0x2F`). The last byte of
/// a frame that fails its CRC is looked at again as a possible `0x2F`, so a frame start is never
/// lost to the error before it.
///
/// Storage: 256 parameter bytes (the largest block) plus a few counters, inside the struct;
/// nothing is allocated.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RequestParser {
    state: State,
    command: u8,
    address: u16,
    len: usize,
    pos: usize,
    crc: u16,
    crc_high: u8,
    buf: [u8; MAX_PARAMS],
}

impl Default for RequestParser {
    fn default() -> Self {
        Self::new()
    }
}

impl RequestParser {
    /// A parser waiting for `0x2F`.
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            command: 0,
            address: 0,
            len: 0,
            pos: 0,
            crc: 0,
            crc_high: 0,
            buf: [0; MAX_PARAMS],
        }
    }

    /// Drops a partly received frame, for example after a pause in the stream.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Feeds one byte. `None`: nothing to report yet. `Some(Ok(request))`: a complete request.
    /// `Some(Err(e))`: a CRC error.
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
            command: Command::from_code(self.command),
            address: self.address,
            params: self.buf.get(..self.len).unwrap_or(&[]),
        }
    }

    fn start(&mut self) {
        self.crc = crc16_xmodem_update(0, REQUEST_START);
        self.state = State::Command;
    }

    fn step(&mut self, byte: u8) -> Step {
        match self.state {
            State::Idle => {
                if byte == REQUEST_START {
                    self.start();
                }
                Step::Pending
            }
            State::Command => {
                self.command = byte;
                self.crc = crc16_xmodem_update(self.crc, byte);
                self.state = State::AddressHigh;
                Step::Pending
            }
            State::AddressHigh => {
                self.address = u16::from(byte) << 8;
                self.crc = crc16_xmodem_update(self.crc, byte);
                self.state = State::AddressLow;
                Step::Pending
            }
            State::AddressLow => {
                self.address |= u16::from(byte);
                self.crc = crc16_xmodem_update(self.crc, byte);
                self.state = State::Count;
                Step::Pending
            }
            State::Count => {
                self.len = if byte == 0 {
                    MAX_PARAMS
                } else {
                    usize::from(byte)
                };
                self.pos = 0;
                self.crc = crc16_xmodem_update(self.crc, byte);
                self.state = State::Params;
                Step::Pending
            }
            State::Params => {
                if let Some(slot) = self.buf.get_mut(self.pos) {
                    *slot = byte;
                }
                self.pos += 1;
                self.crc = crc16_xmodem_update(self.crc, byte);
                if self.pos >= self.len {
                    self.state = State::CrcHigh;
                }
                Step::Pending
            }
            State::CrcHigh => {
                self.crc_high = byte;
                self.state = State::CrcLow;
                Step::Pending
            }
            State::CrcLow => {
                let received = u16::from_be_bytes([self.crc_high, byte]);
                if received == self.crc {
                    self.state = State::Idle;
                    Step::Frame
                } else {
                    let expected = self.crc;
                    if byte == REQUEST_START {
                        self.start();
                    } else {
                        self.state = State::Idle;
                    }
                    Step::Error(ParseError::BadCrc { expected, received })
                }
            }
        }
    }
}

/// Writes a reply: `0x2E`, command, address, parameter count, parameters, acknowledge, CRC16.
/// Returns the frame length (`params.len() + 8`). 256 parameters are written with the count 0.
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] unless `params` holds 1 to 256 bytes (a count of 0 on the wire
/// means 256, so an empty reply cannot be written; send one `0x00` as the protocol does),
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_reply(
    command: Command,
    address: u16,
    params: &[u8],
    ack: Ack,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    if params.is_empty() || params.len() > MAX_PARAMS {
        return Err(EncodeError::InvalidLength);
    }
    let count = u8::try_from(params.len()).unwrap_or(0);
    let mut writer = Writer::new(out, params.len() + REPLY_OVERHEAD)?;
    let [address_high, address_low] = address.to_be_bytes();
    let mut crc = 0;
    for byte in [
        REPLY_START,
        command.code(),
        address_high,
        address_low,
        count,
    ]
    .iter()
    .chain(params)
    .chain(&[ack.code()])
    {
        crc = crc16_xmodem_update(crc, *byte);
        writer.put(*byte);
    }
    let [crc_high, crc_low] = crc.to_be_bytes();
    writer.put(crc_high);
    writer.put(crc_low);
    Ok(writer.len())
}
