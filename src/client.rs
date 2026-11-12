//! The flight controller's side of the AM32 bootloader: the exchanges on one ESC's line.
//!
//! A [`Client`] owns an [`EscLines`] and runs one operation at a time on the selected line.
//! Choose the line with [`Client::select`], not with [`EscLines::select`] through
//! [`Client::line_mut`]: the client remembers the line to keep a little state per line (below).
//! Each operation is one exchange or a fixed sequence of them, built from the encoders and
//! reply parsers of [`bootloader`](crate::bootloader), and returns a typed [`Error`] that says
//! which [`Step`] failed and how. The client never looks inside the data it carries and does not
//! know what an address means. It has no timer: every wait is a `timeout_us` handed to
//! [`EscLines::read`].
//!
//! | Operation | Wire sequence (the ESC's answer in parentheses) |
//! |---|---|
//! | [`Client::connect`] | init, 21 bytes (9 bytes) |
//! | [`Client::read`] | set address (`30`), read (data, CRC, `30`) |
//! | [`Client::write`] | set address (`30`), set buffer (nothing; waits [`Timeouts::set_buffer_us`]), payload (`30`), program (`30`) |
//! | [`Client::erase`] | set address (`30`), erase (`30`) |
//! | [`Client::keep_alive`] | keep-alive (`C1`, which is success) |
//! | [`Client::run_application`] | run (a `C1`, then whatever the application sends, dropped) |
//!
//! Every operation first reads the line with a timeout of 0 until it returns nothing, to throw
//! away bytes an earlier, failed exchange left behind. Bytes that are still in flight are not
//! caught by that.
//!
//! # State the ESC keeps, and the resync
//!
//! The bootloader is not stateless. After set buffer the ESC takes the next frame it receives as
//! the payload, whatever it is, until a payload with a good CRC has arrived. If a payload is
//! refused (`C2`) or lost, the ESC keeps waiting, and it then passes the next valid command as
//! a payload, answers `30` and does not run it: a following set address would be acknowledged
//! and ignored, and a write would land at the old address. The client closes that hazard:
//!
//! - A [`Client::write`] that fails after set buffer went onto the wire and before the
//!   payload's `30` came marks the selected line ([`Client::needs_resync`]).
//! - The next operation on a marked line, or [`Client::resync`] called by the user, first sends
//!   one keep-alive and accepts `30` (the ESC was waiting for a payload and took it) or `C1`
//!   (it was not). Anything else is an error, the mark stays and the operation is not started.
//!   A keep-alive is harmless in both states.
//! - A write is 2 to 256 bytes. A single byte is refused before anything is sent, because no
//!   supported part programs one byte (the smallest write unit is 2) and an ESC waiting for a
//!   one-byte payload is the one state the keep-alive cannot clear (it would keep only the
//!   first three bytes of the probe and answer `C2`). For every length from 2 the probe works.
//! - Inherent to the protocol, and not removable here: if the set-buffer frame is lost on the
//!   wire, the bootloader reads the payload that follows as commands.
//!
//! Other state: a read forgets the address, and each operation here sets the address itself.
//! A program is refused (`C1`) below the ESC's application, and on most parts also for an
//! address or length that is not a multiple of the part's flash word.
//!
//! # Time
//!
//! The defaults in [`Timeouts`] are ArduPilot's waits. An operation can take as long as the sum
//! of its waits (a write: a few ms plus up to 40 ms plus up to 500 ms); an erase up to 3 s.
use crate::EncodeError;
use crate::bootloader::{
    self as bl, Ack, DeviceInfo, INIT_REPLY_LEN, MAX_DATA, ParseError, READ_REPLY_OVERHEAD,
};
use crate::line::EscLines;

/// A payload and its CRC, the longest frame the client builds.
const FRAME_LEN: usize = MAX_DATA + 2;
/// The longest reply: 256 data bytes, CRC and acknowledge.
const REPLY_LEN: usize = MAX_DATA + READ_REPLY_OVERHEAD;
/// The smallest write: no supported part programs a single byte.
const MIN_WRITE: usize = 2;
const DRAIN_CHUNK: usize = 32;
const DRAIN_MAX_READS: usize = 16;

/// The waits of each step, in microseconds. Build with [`Timeouts::new`] and the `with_`
/// setters; the struct is `#[non_exhaustive]`.
///
/// Defaults are ArduPilot's waits (`AP_BLHeli.cpp`, AP below) and agree with the bootloader's own
/// timing: a byte takes 10 bits of 52 us = 520 us at 19200 baud (`main.c` `BITTIME`), the ESC
/// waits one bit time before it answers, and it answers a command in well under 1 ms except
/// program and erase, which are flash operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Timeouts {
    /// Time allowed per byte of a multi-byte reply (init reply, read reply). The wait for the
    /// first byte is this times the length of the reply, as ArduPilot allows the whole reply
    /// (AP:699-705: "byte time is 520us so 1000 per byte should be more than enough"); each
    /// further byte may take this long to follow. Default 1000.
    pub byte_us: u32,
    /// Wait for the acknowledge of set address and the `C1` of keep-alive. Default 2000
    /// (`BL_GetACK`'s default, AP_BLHeli.h:278, used at AP:760 and AP:863).
    pub ack_us: u32,
    /// Wait after set buffer, which has no acknowledge: ArduPilot waits 5 ms "to match
    /// betaflight timing" (AP:899). Default 5000.
    pub set_buffer_us: u32,
    /// Wait for the acknowledge of a payload. Default 40000 (AP:912).
    pub payload_us: u32,
    /// Wait for the acknowledge of program. Default 500000 (AP:941, `BL_WriteFlash`).
    pub program_us: u32,
    /// Wait for the acknowledge of erase. Default 3000000 (AP:876).
    pub erase_us: u32,
    /// Wait after run for the `C1` the bootloader sends before it jumps (`main.c` falls through
    /// to its bad-command answer after the run command). ArduPilot does not read after run
    /// (AP:881-886); this is the keep-alive's wait. Default 2000.
    pub run_us: u32,
}

impl Timeouts {
    /// The defaults.
    pub const fn new() -> Self {
        Self {
            byte_us: 1_000,
            ack_us: 2_000,
            set_buffer_us: 5_000,
            payload_us: 40_000,
            program_us: 500_000,
            erase_us: 3_000_000,
            run_us: 2_000,
        }
    }

    /// Sets [`byte_us`](Self::byte_us).
    #[must_use]
    pub const fn with_byte_us(mut self, us: u32) -> Self {
        self.byte_us = us;
        self
    }

    /// Sets [`ack_us`](Self::ack_us).
    #[must_use]
    pub const fn with_ack_us(mut self, us: u32) -> Self {
        self.ack_us = us;
        self
    }

    /// Sets [`set_buffer_us`](Self::set_buffer_us).
    #[must_use]
    pub const fn with_set_buffer_us(mut self, us: u32) -> Self {
        self.set_buffer_us = us;
        self
    }

    /// Sets [`payload_us`](Self::payload_us).
    #[must_use]
    pub const fn with_payload_us(mut self, us: u32) -> Self {
        self.payload_us = us;
        self
    }

    /// Sets [`program_us`](Self::program_us).
    #[must_use]
    pub const fn with_program_us(mut self, us: u32) -> Self {
        self.program_us = us;
        self
    }

    /// Sets [`erase_us`](Self::erase_us).
    #[must_use]
    pub const fn with_erase_us(mut self, us: u32) -> Self {
        self.erase_us = us;
        self
    }

    /// Sets [`run_us`](Self::run_us).
    #[must_use]
    pub const fn with_run_us(mut self, us: u32) -> Self {
        self.run_us = us;
        self
    }
}

impl Default for Timeouts {
    fn default() -> Self {
        Self::new()
    }
}

/// Which exchange of an operation went wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Step {
    /// The zero-timeout read that clears stale bytes before an operation.
    Drain,
    /// Selecting a line.
    Select,
    /// The keep-alive that finds out whether the ESC still waits for a payload.
    Resync,
    /// The init sequence and its 9-byte reply.
    Init,
    /// Set address and its acknowledge.
    SetAddress,
    /// Set buffer and the (unexpected) answer to it.
    SetBuffer,
    /// The payload and its acknowledge.
    Payload,
    /// Program and its acknowledge.
    Program,
    /// Erase and its acknowledge.
    Erase,
    /// The read command and its reply: data, CRC, acknowledge.
    Read,
    /// Keep-alive and its answer.
    KeepAlive,
    /// Run and what follows it.
    Run,
}

/// Why an operation failed. `E` is the line's own error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// A length argument is outside its range (1 to 256 for a read, 2 to 256 for a write).
    /// Nothing was sent.
    InvalidLength,
    /// A line number above 31; [`Client::select`] keeps state for lines 0 to 31 only.
    InvalidLine,
    /// The line reported an error.
    Line {
        /// The exchange that was running.
        step: Step,
        /// The line's error.
        error: E,
    },
    /// No byte came within the time allowed.
    Timeout {
        /// The exchange that got no answer.
        step: Step,
    },
    /// Some bytes came, fewer than the reply has, and then nothing within the time allowed.
    Short {
        /// The exchange whose reply was short.
        step: Step,
        /// Bytes received.
        got: usize,
        /// Bytes the reply has.
        expected: usize,
    },
    /// The CRC of a reply from the ESC did not match its data.
    BadCrc {
        /// The exchange whose reply failed.
        step: Step,
        /// The CRC computed over the data received.
        expected: u16,
        /// The CRC the ESC sent.
        received: u16,
    },
    /// The ESC refused: it answered `C1` ([`Ack::BadCommand`]) or `C2` ([`Ack::BadCrc`]) where
    /// `30` was expected (a read refused is answered with that one byte).
    Refused {
        /// The exchange the ESC refused.
        step: Step,
        /// The ESC's answer.
        ack: Ack,
    },
    /// A byte that no reply of this exchange can have: an acknowledge that is none of `30`,
    /// `C1`, `C2`; a wrong init signature (the first wrong byte); a `30` where the keep-alive
    /// wants `C1`.
    Unexpected {
        /// The exchange that got the byte.
        step: Step,
        /// The byte.
        byte: u8,
    },
}

/// The bootloader client over the lines `L`.
#[derive(Debug)]
pub struct Client<L> {
    line: L,
    timeouts: Timeouts,
    selected: Option<u8>,
    /// Bit n: line n may still wait for a payload. Bit 32: the line chosen without `select`.
    unsure: u64,
}

const UNSELECTED_BIT: u32 = 32;

impl<L: EscLines> Client<L> {
    /// A client with the default [`Timeouts`].
    pub const fn new(line: L) -> Self {
        Self {
            line,
            timeouts: Timeouts::new(),
            selected: None,
            unsure: 0,
        }
    }

    /// Replaces the [`Timeouts`].
    #[must_use]
    pub const fn with_timeouts(mut self, timeouts: Timeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    /// The current [`Timeouts`].
    pub const fn timeouts(&self) -> &Timeouts {
        &self.timeouts
    }

    /// The lines.
    pub const fn line(&self) -> &L {
        &self.line
    }

    /// The lines, for [`EscLines::select`] and [`EscLines::set_break`].
    pub const fn line_mut(&mut self) -> &mut L {
        &mut self.line
    }

    /// Gives the lines back.
    pub fn into_line(self) -> L {
        self.line
    }

    /// Makes `line` (0 to 31) the active one and remembers it. Use this instead of
    /// [`EscLines::select`] on [`Client::line_mut`]: a line chosen behind the client's back is not
    /// tracked, and its marks (see [`Client::needs_resync`]) would be filed under the wrong line.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidLine`] above 31 (nothing is forwarded); [`Error::Line`] with
    /// [`Step::Select`] if the line refuses, and the remembered line is then unchanged.
    pub fn select(&mut self, line: u8) -> Result<(), Error<L::Error>> {
        if u32::from(line) >= UNSELECTED_BIT {
            return Err(Error::InvalidLine);
        }
        self.line.select(line).map_err(|error| Error::Line {
            step: Step::Select,
            error,
        })?;
        self.selected = Some(line);
        Ok(())
    }

    /// True if a failed write left the selected line in a state where the ESC may still wait for
    /// a payload. The next operation on it, or [`Client::resync`], clears that.
    pub const fn needs_resync(&self) -> bool {
        self.unsure & self.slot_bit() != 0
    }

    /// Sends one keep-alive on the selected line and accepts `30` (the ESC was waiting for a
    /// payload and took the keep-alive for it) or `C1` (it was not); the mark of the line is
    /// cleared. Every operation does this by itself first on a marked line; calling it on an
    /// unmarked line is harmless.
    ///
    /// # Errors
    ///
    /// [`Error`] with [`Step::Resync`]: [`Error::Timeout`], [`Error::Refused`] for `C2` (see the
    /// limit in the module note), [`Error::Unexpected`] for another byte, [`Error::Line`]. The
    /// mark stays.
    pub async fn resync(&mut self) -> Result<(), Error<L::Error>> {
        self.drain().await?;
        self.probe().await
    }

    /// Sends the 21-byte init and reads the 9-byte reply.
    ///
    /// # Errors
    ///
    /// [`Error`] with [`Step::Init`]: [`Error::Timeout`] for silence, [`Error::Short`],
    /// [`Error::Refused`] for a lone `C1` or `C2` or a reply that ends in one,
    /// [`Error::Unexpected`] for a wrong signature or an unknown acknowledge.
    pub async fn connect(&mut self) -> Result<DeviceInfo, Error<L::Error>> {
        self.prepare().await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_init(&mut frame);
        self.send_frame(Step::Init, &frame, len).await?;
        let mut reply = [0u8; INIT_REPLY_LEN];
        let got = self.receive(Step::Init, &mut reply).await?;
        let reply = reply.get(..got).unwrap_or_default();
        if reply.len() < INIT_REPLY_LEN {
            return Err(short(Step::Init, reply, INIT_REPLY_LEN));
        }
        bl::parse_init_reply(reply).map_err(|error| match error {
            ParseError::BadSignature => Error::Unexpected {
                step: Step::Init,
                byte: first_wrong_signature_byte(reply),
            },
            other => from_parse(Step::Init, reply, other),
        })
    }

    /// Reads `buf.len()` bytes (1 to 256) at `address`: set address, read, and the reply
    /// checked (CRC and acknowledge). `buf` is written only when the whole reply is good.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidLength`] unless `buf` holds 1 to 256 bytes (nothing is sent); otherwise
    /// [`Error`] with [`Step::SetAddress`] or [`Step::Read`]: [`Error::Timeout`],
    /// [`Error::Short`], [`Error::BadCrc`], [`Error::Refused`], [`Error::Unexpected`],
    /// [`Error::Line`].
    pub async fn read(&mut self, address: u16, buf: &mut [u8]) -> Result<(), Error<L::Error>> {
        let wanted = buf.len();
        if wanted == 0 || wanted > MAX_DATA {
            return Err(Error::InvalidLength);
        }
        self.prepare().await?;
        self.set_address(address).await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_read(wanted, &mut frame);
        self.send_frame(Step::Read, &frame, len).await?;
        let expected = wanted + READ_REPLY_OVERHEAD;
        let mut reply = [0u8; REPLY_LEN];
        let window = reply.get_mut(..expected).ok_or(Error::InvalidLength)?;
        let got = self.receive(Step::Read, window).await?;
        let reply = reply.get(..got).unwrap_or_default();
        if reply.len() < expected {
            return Err(short(Step::Read, reply, expected));
        }
        let data = bl::parse_read_reply(reply, wanted)
            .map_err(|error| from_parse(Step::Read, reply, error))?;
        buf.copy_from_slice(data);
        Ok(())
    }

    /// Writes `data` (2 to 256 bytes) at `address`: set address, set buffer, payload, program.
    /// Set buffer has no acknowledge; the client waits [`Timeouts::set_buffer_us`] and goes on.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidLength`] unless `data` holds 2 to 256 bytes (nothing is sent, the line is
    /// not marked); otherwise
    /// [`Error`] with [`Step::SetAddress`], [`Step::SetBuffer`] (the ESC answered `C1`/`C2`, or
    /// an unknown byte; a `30` is tolerated as ArduPilot does), [`Step::Payload`] or
    /// [`Step::Program`]; or the error of the resync that a marked line gets first. A failure
    /// after set buffer was sent and before the payload's `30` marks the line (see the module's
    /// note on the resync).
    pub async fn write(&mut self, address: u16, data: &[u8]) -> Result<(), Error<L::Error>> {
        if data.len() < MIN_WRITE || data.len() > MAX_DATA {
            return Err(Error::InvalidLength);
        }
        self.prepare().await?;
        self.set_address(address).await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_set_buffer(data.len(), &mut frame);
        self.unsure |= self.slot_bit();
        self.send_frame(Step::SetBuffer, &frame, len).await?;
        let mut stray = [0u8; 1];
        let got = self
            .line
            .read(&mut stray, self.timeouts.set_buffer_us)
            .await
            .map_err(|error| Error::Line {
                step: Step::SetBuffer,
                error,
            })?;
        if got > 0 {
            let byte = stray.first().copied().unwrap_or(0);
            match Ack::from_code(byte) {
                Some(Ack::Good) => {}
                Some(ack) => {
                    return Err(Error::Refused {
                        step: Step::SetBuffer,
                        ack,
                    });
                }
                None => {
                    return Err(Error::Unexpected {
                        step: Step::SetBuffer,
                        byte,
                    });
                }
            }
        }
        let len = bl::encode_payload(data, &mut frame);
        self.send_frame(Step::Payload, &frame, len).await?;
        self.expect_good(Step::Payload, self.timeouts.payload_us)
            .await?;
        self.unsure &= !self.slot_bit();
        let len = bl::encode_program(&mut frame);
        self.send_frame(Step::Program, &frame, len).await?;
        self.expect_good(Step::Program, self.timeouts.program_us)
            .await
    }

    /// Erases: set address, then the erase command. The bootloader only checks that the address
    /// is at or above its application and answers; it erases nothing by itself, the page is
    /// erased by the next program at a page-aligned address on parts that do so.
    ///
    /// # Errors
    ///
    /// [`Error`] with [`Step::SetAddress`] or [`Step::Erase`] (`C1` below the application).
    pub async fn erase(&mut self, address: u16) -> Result<(), Error<L::Error>> {
        self.prepare().await?;
        self.set_address(address).await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_erase(&mut frame);
        self.send_frame(Step::Erase, &frame, len).await?;
        self.expect_good(Step::Erase, self.timeouts.erase_us).await
    }

    /// Sends keep-alive. The bootloader answers `C1` on purpose: that is success.
    ///
    /// # Errors
    ///
    /// [`Error`] with [`Step::KeepAlive`]: [`Error::Timeout`]; [`Error::Refused`] for `C2` (it
    /// saw a bad CRC); [`Error::Unexpected`] for `30` or any other byte.
    pub async fn keep_alive(&mut self) -> Result<(), Error<L::Error>> {
        self.prepare().await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_keep_alive(&mut frame);
        self.send_frame(Step::KeepAlive, &frame, len).await?;
        match self
            .expect_ack(Step::KeepAlive, self.timeouts.ack_us)
            .await?
        {
            Ack::BadCommand => Ok(()),
            Ack::BadCrc => Err(Error::Refused {
                step: Step::KeepAlive,
                ack: Ack::BadCrc,
            }),
            Ack::Good => Err(Error::Unexpected {
                step: Step::KeepAlive,
                byte: Ack::Good.code(),
            }),
        }
    }

    /// Sends run (`00 00 00 00`): the bootloader starts its application unless its own checks
    /// refuse. It sends a `C1` first; this reads and drops whatever arrives (bounded: 16 reads of 32 bytes)
    /// until [`Timeouts::run_us`] passes with nothing, because a started application may put
    /// bytes on the line. `Ok` does not say whether the ESC jumped.
    ///
    /// # Errors
    ///
    /// [`Error::Line`] with [`Step::Run`] (or an error of the resync of a marked line); what
    /// arrives after run is never an error.
    pub async fn run_application(&mut self) -> Result<(), Error<L::Error>> {
        self.prepare().await?;
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_run(&mut frame);
        self.send_frame(Step::Run, &frame, len).await?;
        // The bootloader sends a C1 first and an application that has started may put bytes on
        // the line: read and drop whatever comes until `run_us` passes with nothing.
        let mut scratch = [0u8; DRAIN_CHUNK];
        for _ in 0..DRAIN_MAX_READS {
            let got = self
                .line
                .read(&mut scratch, self.timeouts.run_us)
                .await
                .map_err(|error| Error::Line {
                    step: Step::Run,
                    error,
                })?;
            if got == 0 {
                break;
            }
        }
        Ok(())
    }

    const fn slot_bit(&self) -> u64 {
        match self.selected {
            Some(line) => 1 << line,
            None => 1 << UNSELECTED_BIT,
        }
    }

    /// The start of every operation: drop stale bytes, and on a marked line resync.
    async fn prepare(&mut self) -> Result<(), Error<L::Error>> {
        self.drain().await?;
        if self.needs_resync() {
            self.probe().await?;
        }
        Ok(())
    }

    async fn probe(&mut self) -> Result<(), Error<L::Error>> {
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_keep_alive(&mut frame);
        self.send_frame(Step::Resync, &frame, len).await?;
        match self.expect_ack(Step::Resync, self.timeouts.ack_us).await? {
            Ack::Good | Ack::BadCommand => {
                self.unsure &= !self.slot_bit();
                Ok(())
            }
            Ack::BadCrc => Err(Error::Refused {
                step: Step::Resync,
                ack: Ack::BadCrc,
            }),
        }
    }

    async fn drain(&mut self) -> Result<(), Error<L::Error>> {
        let mut scratch = [0u8; DRAIN_CHUNK];
        for _ in 0..DRAIN_MAX_READS {
            let got = self
                .line
                .read(&mut scratch, 0)
                .await
                .map_err(|error| Error::Line {
                    step: Step::Drain,
                    error,
                })?;
            if got == 0 {
                break;
            }
        }
        Ok(())
    }

    async fn send_frame(
        &mut self,
        step: Step,
        frame: &[u8],
        len: Result<usize, EncodeError>,
    ) -> Result<(), Error<L::Error>> {
        let len = len.map_err(|_| Error::InvalidLength)?;
        let bytes = frame.get(..len).ok_or(Error::InvalidLength)?;
        self.line
            .write(bytes)
            .await
            .map_err(|error| Error::Line { step, error })
    }

    /// Fills `buf` from the line, waiting the first byte `byte_us` times the length of `buf`
    /// and each later one `byte_us`. Returns how many bytes came; fewer than `buf.len()` means
    /// the line went quiet.
    async fn receive(&mut self, step: Step, buf: &mut [u8]) -> Result<usize, Error<L::Error>> {
        let mut got = 0;
        let first_us = u32::try_from(buf.len())
            .unwrap_or(u32::MAX)
            .saturating_mul(self.timeouts.byte_us);
        while let Some(rest) = buf.get_mut(got..) {
            if rest.is_empty() {
                break;
            }
            let timeout_us = if got == 0 {
                first_us
            } else {
                self.timeouts.byte_us
            };
            let n = self
                .line
                .read(rest, timeout_us)
                .await
                .map_err(|error| Error::Line { step, error })?;
            if n == 0 {
                break;
            }
            got += n.min(rest.len());
        }
        Ok(got)
    }

    async fn expect_ack(&mut self, step: Step, timeout_us: u32) -> Result<Ack, Error<L::Error>> {
        let mut byte = [0u8; 1];
        let got = self
            .line
            .read(&mut byte, timeout_us)
            .await
            .map_err(|error| Error::Line { step, error })?;
        if got == 0 {
            return Err(Error::Timeout { step });
        }
        let byte = byte.first().copied().unwrap_or(0);
        Ack::from_code(byte).ok_or(Error::Unexpected { step, byte })
    }

    async fn expect_good(&mut self, step: Step, timeout_us: u32) -> Result<(), Error<L::Error>> {
        match self.expect_ack(step, timeout_us).await? {
            Ack::Good => Ok(()),
            ack => Err(Error::Refused { step, ack }),
        }
    }

    async fn set_address(&mut self, address: u16) -> Result<(), Error<L::Error>> {
        let mut frame = [0u8; FRAME_LEN];
        let len = bl::encode_set_address(address, &mut frame);
        self.send_frame(Step::SetAddress, &frame, len).await?;
        self.expect_good(Step::SetAddress, self.timeouts.ack_us)
            .await
    }
}

/// The error for a reply that stopped early: nothing is a timeout, a lone `C1` or `C2` is the
/// ESC refusing (a refused read or a garbled init is answered with that one byte), the rest is
/// short.
fn short<E>(step: Step, got: &[u8], expected: usize) -> Error<E> {
    match got {
        [] => Error::Timeout { step },
        [byte] => match Ack::from_code(*byte) {
            Some(ack @ (Ack::BadCommand | Ack::BadCrc)) => Error::Refused { step, ack },
            _ => Error::Short {
                step,
                got: 1,
                expected,
            },
        },
        _ => Error::Short {
            step,
            got: got.len(),
            expected,
        },
    }
}

fn from_parse<E>(step: Step, reply: &[u8], error: ParseError) -> Error<E> {
    match error {
        ParseError::BadCrc { expected, received } => Error::BadCrc {
            step,
            expected,
            received,
        },
        ParseError::Rejected(ack) => Error::Refused { step, ack },
        ParseError::UnknownAck(byte) => Error::Unexpected { step, byte },
        ParseError::InvalidLength => Error::InvalidLength,
        _ => Error::Unexpected {
            step,
            byte: reply.last().copied().unwrap_or(0),
        },
    }
}

fn first_wrong_signature_byte(reply: &[u8]) -> u8 {
    reply
        .iter()
        .zip(b"471")
        .find(|(got, want)| got != want)
        .map_or(0, |(got, _)| *got)
}
