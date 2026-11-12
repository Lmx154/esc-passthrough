//! The AM32 bootloader's one-wire byte sequences, host to ESC and back.
//!
//! The functions write the bytes a host sends and read the bytes the ESC answers. They know
//! nothing of timing, of the wire or of what an address points to: addresses are `u16`s as sent,
//! payloads are opaque.
//!
//! Every command except the 21-byte init and the payload is two or four bytes followed by the
//! CRC16 of those bytes (polynomial 0xA001 reflected, start 0, low byte first, see [`crc16`]).
//!
//! | Command | Bytes before the CRC | ESC answers |
//! |---|---|---|
//! | [`encode_init`] | 12 x `00`, `0D`, `BLHeli`, `F4`, `7D` (no CRC) | 9 bytes, [`parse_init_reply`] |
//! | [`encode_set_address`] | `FF 00` address-high address-low | [`Ack`] |
//! | [`encode_set_buffer`] | `FE 00` 0-or-1 length | nothing |
//! | [`encode_payload`] | the payload, then its CRC | [`Ack`] |
//! | [`encode_program`] | `01 01` | [`Ack`] |
//! | [`encode_read`] | `03` n (0 means 256) | data, CRC, [`Ack`] ([`parse_read_reply`]) |
//! | [`encode_keep_alive`] | `FD 00` | [`Ack::BadCommand`] on purpose |
//! | [`encode_run`] | `00 00` | nothing; the ESC starts its application |

use crate::EncodeError;
use crate::error::Writer;

/// Length of the init sequence.
pub const INIT_LEN: usize = 21;
/// Length of the ESC's init reply.
pub const INIT_REPLY_LEN: usize = 9;
/// Most data bytes a buffer, a payload or a read can carry.
pub const MAX_DATA: usize = 256;
/// Bytes a read reply carries around its data: CRC (2) and acknowledge.
pub const READ_REPLY_OVERHEAD: usize = 3;

/// The init sequence a host sends to wake the bootloader: 12 zero bytes, `0D`, `"BLHeli"`, `F4`,
/// `7D`. It carries no CRC.
pub const INIT: [u8; INIT_LEN] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, b'B', b'L', b'H', b'e', b'l', b'i', 0xF4, 0x7D,
];

/// One step of the bootloader CRC16 (polynomial 0xA001, reflected).
pub const fn crc16_update(crc: u16, byte: u8) -> u16 {
    let mut crc = crc ^ byte as u16;
    let mut bit = 0;
    while bit < 8 {
        crc = if crc & 1 != 0 {
            (crc >> 1) ^ 0xA001
        } else {
            crc >> 1
        };
        bit += 1;
    }
    crc
}

/// The bootloader CRC16 of `bytes`: polynomial 0xA001 reflected, start 0 (CRC-16/ARC). It goes
/// on the wire low byte first. The check value of `b"123456789"` is 0xBB3D.
pub const fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0;
    let mut rest = bytes;
    while let [byte, tail @ ..] = rest {
        crc = crc16_update(crc, *byte);
        rest = tail;
    }
    crc
}

/// The bootloader's one-byte acknowledge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Ack {
    /// 0x30: good.
    Good,
    /// 0xC1: bad command. A keep-alive is answered with this on purpose.
    BadCommand,
    /// 0xC2: bad CRC.
    BadCrc,
}

impl Ack {
    /// The acknowledge for a byte, or `None` for any other byte.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0x30 => Some(Self::Good),
            0xC1 => Some(Self::BadCommand),
            0xC2 => Some(Self::BadCrc),
            _ => None,
        }
    }

    /// The byte on the wire.
    pub const fn code(self) -> u8 {
        match self {
            Self::Good => 0x30,
            Self::BadCommand => 0xC1,
            Self::BadCrc => 0xC2,
        }
    }
}

fn encode_with_crc(body: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    let mut writer = Writer::new(out, body.len() + 2)?;
    let [crc_low, crc_high] = crc16(body).to_le_bytes();
    writer.put_all(body);
    writer.put(crc_low);
    writer.put(crc_high);
    Ok(writer.len())
}

/// Writes the 21-byte [`INIT`] sequence.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 21 bytes.
pub fn encode_init(out: &mut [u8]) -> Result<usize, EncodeError> {
    let mut writer = Writer::new(out, INIT_LEN)?;
    writer.put_all(&INIT);
    Ok(writer.len())
}

/// Writes set address: `FF 00`, address high, address low, CRC (6 bytes). The ESC answers with
/// an [`Ack`]. The address is not interpreted.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 6 bytes.
pub fn encode_set_address(address: u16, out: &mut [u8]) -> Result<usize, EncodeError> {
    let [high, low] = address.to_be_bytes();
    encode_with_crc(&[0xFF, 0x00, high, low], out)
}

/// Writes set buffer length: `FE 00`, 1 if `len` is 256 and 0 otherwise, `len` modulo 256, CRC
/// (6 bytes). The ESC does not answer; the payload follows ([`encode_payload`]).
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] unless `len` is 1 to 256, [`EncodeError::BufferTooSmall`] if
/// `out` is shorter than 6 bytes.
pub fn encode_set_buffer(len: usize, out: &mut [u8]) -> Result<usize, EncodeError> {
    if len == 0 || len > MAX_DATA {
        return Err(EncodeError::InvalidLength);
    }
    let [low, high] = (len as u16).to_le_bytes();
    encode_with_crc(&[0xFE, 0x00, high, low], out)
}

/// Writes a payload followed by its CRC (`data.len() + 2` bytes). The ESC answers with an
/// [`Ack`]. The bytes are not interpreted.
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] unless `data` holds 1 to 256 bytes,
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than the frame.
pub fn encode_payload(data: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    if data.is_empty() || data.len() > MAX_DATA {
        return Err(EncodeError::InvalidLength);
    }
    encode_with_crc(data, out)
}

/// Writes program: `01 01`, CRC (4 bytes). The ESC writes the buffer at the address and answers
/// with an [`Ack`].
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 4 bytes.
pub fn encode_program(out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_with_crc(&[0x01, 0x01], out)
}

/// Writes read: `03`, `n` (256 is sent as 0), CRC (4 bytes). The ESC answers with `n` bytes, their
/// CRC and an [`Ack`] ([`parse_read_reply`]).
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] unless `n` is 1 to 256, [`EncodeError::BufferTooSmall`] if
/// `out` is shorter than 4 bytes.
pub fn encode_read(n: usize, out: &mut [u8]) -> Result<usize, EncodeError> {
    if n == 0 || n > MAX_DATA {
        return Err(EncodeError::InvalidLength);
    }
    let [low, _] = (n as u16).to_le_bytes();
    encode_with_crc(&[0x03, low], out)
}

/// Writes keep-alive: `FD 00`, CRC (4 bytes). A good ESC answers [`Ack::BadCommand`].
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 4 bytes.
pub fn encode_keep_alive(out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_with_crc(&[0xFD, 0x00], out)
}

/// Writes run: `00 00 00 00` (the two command bytes and their CRC, which is zero). The ESC does
/// not answer; it starts its application.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 4 bytes.
pub fn encode_run(out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_with_crc(&[0x00, 0x00], out)
}

/// Why a reply was not accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ParseError {
    /// Fewer bytes than the reply needs; read more and parse again.
    Truncated,
    /// More bytes than the reply has.
    TrailingBytes,
    /// A requested read length outside 1 to 256.
    InvalidLength,
    /// The init reply does not start with `471`.
    BadSignature,
    /// The CRC of a read reply did not match.
    BadCrc {
        /// The CRC computed over the data.
        expected: u16,
        /// The CRC received.
        received: u16,
    },
    /// The acknowledge byte is not one of the three codes.
    UnknownAck(u8),
    /// A reply that should carry [`Ack::Good`] carries another acknowledge.
    Rejected(Ack),
}

/// What the ESC's init reply tells the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct DeviceInfo {
    /// Pin code (byte 3).
    pub pin_code: u8,
    /// Flash size code (byte 4): 0x1F for 32 KB, 0x35 for 64 KB, 0x2B for 128 KB.
    pub flash_size_code: u8,
    /// Bytes 5 and 6, both 0x06 in AM32.
    pub tail: [u8; 2],
    /// Bootloader protocol version (byte 7).
    pub protocol_version: u8,
}

impl DeviceInfo {
    /// The 16-bit signature as the configurator reads it: flash size code in the high byte, byte
    /// 5 in the low byte (0x1F06 for a 32 KB part).
    pub const fn signature(&self) -> u16 {
        let [first, _] = self.tail;
        u16::from_be_bytes([self.flash_size_code, first])
    }
}

fn check_exact(reply: &[u8], expected: usize) -> Result<(), ParseError> {
    match reply.len().cmp(&expected) {
        core::cmp::Ordering::Less => Err(ParseError::Truncated),
        core::cmp::Ordering::Greater => Err(ParseError::TrailingBytes),
        core::cmp::Ordering::Equal => Ok(()),
    }
}

fn require_good(code: u8) -> Result<(), ParseError> {
    match Ack::from_code(code) {
        Some(Ack::Good) => Ok(()),
        Some(other) => Err(ParseError::Rejected(other)),
        None => Err(ParseError::UnknownAck(code)),
    }
}

/// Reads the ESC's reply to a lone acknowledge: exactly one byte. A refusal ([`Ack::BadCommand`],
/// [`Ack::BadCrc`]) is a value, not an error: the keep-alive expects [`Ack::BadCommand`].
///
/// # Errors
///
/// [`ParseError::Truncated`] for no byte, [`ParseError::TrailingBytes`] for more than one,
/// [`ParseError::UnknownAck`] for any other byte.
pub fn parse_ack(reply: &[u8]) -> Result<Ack, ParseError> {
    check_exact(reply, 1)?;
    let code = reply.first().copied().unwrap_or(0);
    Ack::from_code(code).ok_or(ParseError::UnknownAck(code))
}

/// Reads the ESC's 9-byte init reply: `4 7 1`, pin code, flash size code, two bytes of 0x06,
/// protocol version, and the acknowledge `30`.
///
/// # Errors
///
/// [`ParseError::Truncated`] or [`ParseError::TrailingBytes`] unless `reply` is 9 bytes,
/// [`ParseError::BadSignature`] unless it starts with `471`, [`ParseError::Rejected`] or
/// [`ParseError::UnknownAck`] unless its last byte is `30`.
pub fn parse_init_reply(reply: &[u8]) -> Result<DeviceInfo, ParseError> {
    check_exact(reply, INIT_REPLY_LEN)?;
    match reply {
        [
            b'4',
            b'7',
            b'1',
            pin_code,
            flash_size_code,
            tail0,
            tail1,
            protocol_version,
            ack,
        ] => {
            require_good(*ack)?;
            Ok(DeviceInfo {
                pin_code: *pin_code,
                flash_size_code: *flash_size_code,
                tail: [*tail0, *tail1],
                protocol_version: *protocol_version,
            })
        }
        _ => Err(ParseError::BadSignature),
    }
}

/// Reads the ESC's reply to a read of `n` bytes: the data, its CRC (low byte first) and the
/// acknowledge `30` (`n + 3` bytes). Returns the data, borrowed from `reply`.
///
/// A read the ESC refuses is answered with one acknowledge byte and nothing else; a one-byte
/// `reply` is therefore [`ParseError::Truncated`] here, and the caller decides (with its
/// timeout) whether to wait for more or to read it with [`parse_ack`].
///
/// # Errors
///
/// [`ParseError::InvalidLength`] unless `n` is 1 to 256, [`ParseError::Truncated`] or
/// [`ParseError::TrailingBytes`] unless `reply` is `n + 3` bytes, [`ParseError::BadCrc`] if the
/// CRC does not match the data, [`ParseError::Rejected`] or [`ParseError::UnknownAck`] unless the
/// last byte is `30`.
pub fn parse_read_reply(reply: &[u8], n: usize) -> Result<&[u8], ParseError> {
    if n == 0 || n > MAX_DATA {
        return Err(ParseError::InvalidLength);
    }
    check_exact(reply, n + READ_REPLY_OVERHEAD)?;
    let (data, tail) = reply.split_at_checked(n).ok_or(ParseError::Truncated)?;
    let &[crc_low, crc_high, ack] = tail else {
        return Err(ParseError::Truncated);
    };
    let expected = crc16(data);
    let received = u16::from_le_bytes([crc_low, crc_high]);
    if expected != received {
        return Err(ParseError::BadCrc { expected, received });
    }
    require_good(ack)?;
    Ok(data)
}
