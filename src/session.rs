//! The session: a configurator's requests arriving on a byte stream, executed on the motor
//! controllers' lines and answered.
//!
//! [`Passthrough`] joins [`HostSide`] (what the configurator asks) to [`Client`] (what a controller's
//! bootloader understands). It reads a byte stream (`embedded_io_async::Read`), writes the replies
//! to another (`embedded_io_async::Write`), waits with the caller's `DelayNs` and takes the lines
//! from the caller's [`Lender`] at the one moment the design allows: the `MSP_SET_PASSTHROUGH`
//! request. It owns no timer, names no transport and never looks inside a payload.
//!
//! # The conversation
//!
//! 1. **MSP phase.** The requests [`HostSide`] answers on its own are answered (the battery figure
//!    comes from the `battery` callback at the moment of the request). If `idle_ms` passes
//!    without a complete valid MSP request while the lines are not held, [`Passthrough::run`]
//!    returns [`Exit::Idle`]; bytes that are no request do not restart that time.
//! 2. **The grant.** On `MSP_SET_PASSTHROUGH` the lender is asked ([`Lender::lend`]). `None` is
//!    answered with the MSP error reply and the session stays in the MSP phase. `Some(lines)`: the
//!    lines are kept in a [`Client`], the reply carries the ESC count, and then `park_ms` pass (the
//!    caller's delay) with the lines held high and silent before any line operation. While it
//!    passes the host stream is not read. A session that already holds lines (the configurator
//!    started over, below) is not asked again.
//! 3. **Four-way requests.** Each is answered exactly once, with its command and address echoed.
//!    The line operation each one starts is in the table below; nothing else ever starts one,
//!    except the lines exit.
//! 4. **Exit.** After `InterfaceExit`'s reply, all lines are pulled low together for `break_ms`
//!    and released (the bootloader jumps to the application after 20 ms of low); only then is the
//!    exit done and [`Passthrough::release`] allowed. A host read or write error does the same
//!    before [`Exit::Host`] is returned, and so does the end of the stream ([`Exit::Closed`]).
//! 5. **A configurator that starts over** (a valid MSP request in four-way mode): the lines exit,
//!    the lines are kept, and the request is answered in the MSP phase. The next grant waits
//!    `park_ms` again. While lines are held, silence ends nothing.
//!
//! | Request | Line operation | Reply (parameters; acknowledge) |
//! |---|---|---|
//! | `ProtocolGetVersion`, `InterfaceGetName`, `InterfaceGetVersion` | none | [`host::answer_housekeeping`] |
//! | `InterfaceTestAlive` | keep-alive on the connected controller | `[0]`; OK, or general error if none is connected or the keep-alive fails (the controller is then no longer connected) |
//! | `DeviceInitFlash` `[i]` | select `line_map[i]`, connect (one try) | `[sig low, sig high, pin code, 4]`; OK. General error with `[i]` if it fails. Invalid channel with `[i]` and no line traffic if `i >= esc_count` |
//! | `DeviceRead` `[n]` (0 is 256) | read `n` bytes at the request's address | the bytes; OK. General error with `[n]` if none is connected or the read fails |
//! | `DeviceWrite` (2 to 256 bytes) | write at the request's address | `[0]`; OK. Invalid parameter (no traffic) for any other length, general error if none is connected or it fails |
//! | `DevicePageErase` `[page]` | erase at `page << 10` (16 bits, as ArduPilot does for an ARM bootloader) | `[page]`; OK, or general error |
//! | `DeviceReset` `[i]` | select `line_map[i]`, run | `[i]`; OK. General error with `[i]` if it fails. Invalid channel as for init |
//! | `InterfaceExit` | the lines exit after the reply | `[0]`; OK |
//! | `InterfaceSetMode` `[m]` | none | `[m]`; OK (ArduPilot's answer; the session keeps no mode) |
//! | `DeviceWriteEeprom` | none | `[0]`; general error (ArduPilot's answer for an ARM bootloader) |
//! | `DeviceVerify`, `DeviceReadEeprom`, `DeviceEraseAll`, `DeviceC2ckLow`, any other code | none | `[0]`; invalid command |
//!
//! A client error of any kind becomes the general-error acknowledge and the session goes on. It
//! never retries and never waits beyond the client's own timeouts.
//!
//! # What the caller guarantees
//!
//! - The stream's `read` is **cancel-safe**: a `read` future that is dropped before it completes
//!   has taken no byte from the stream. The MSP phase races the read against the idle delay and
//!   drops the read when the delay wins.
//! - The lender lends only when the vehicle may be handled (disarmed).
//! - After cancelling [`Passthrough::run`] (dropping its future at any await) the caller calls
//!   [`Passthrough::finish`] and, once it has returned, may [`Passthrough::release`] the lines.
//!   Without `finish` the controllers stay inert in their bootloaders until they are power-cycled.
//!
//! # Cancellation
//!
//! Everything `finish` and `release` need is kept in the [`Passthrough`], not in the future
//! of `run`: whether lines are held, whether the lines exit is owed or half done, the controller
//! that is connected. A `run` dropped during the exit break leaves the lines low; `finish` waits
//! the whole `break_ms` again before it releases them, and pulls them low a second time never.
//! The host stream's bytes that `run` had read but not yet used are lost with the future.
#![allow(async_fn_in_trait)]

use core::convert::Infallible;
use core::future::{Future, pending, poll_fn};
use core::pin::{Pin, pin};
use core::task::Poll;

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, Write};

use crate::client::{Client, Timeouts};
use crate::fourway::{self, Ack, Command, MAX_PARAMS};
use crate::host::{self, Event, HostInfo, HostSide, Mode};
use crate::line::EscLines;
use crate::msp;

/// Controllers the session can address.
pub const MAX_ESC: usize = 8;
/// Controllers by default.
pub const DEFAULT_ESC_COUNT: u8 = 4;
/// Default time the lines are held high and silent after the grant: a full second more than the 2 s
/// after which a controller without a signal resets into its bootloader (measured on four AM32
/// controllers: 1.5 s was too short from the not-armed state, 2.0 s and more always worked).
pub const DEFAULT_PARK_MS: u32 = 3_000;
/// Default time all lines are held low at the exit (the bootloader jumps after 20 ms).
pub const DEFAULT_BREAK_MS: u32 = 25;
/// Default time without a valid MSP request before the lines are granted.
pub const DEFAULT_IDLE_MS: u32 = 5_000;

const READ_LEN: usize = 64;
const MIN_WRITE: usize = 2;
/// Interface mode the init reply reports: ArduPilot's `imARM_BLB`.
const INTERFACE_MODE_ARM: u8 = 4;
const PAGE_SHIFT: u32 = 10;

/// Values of a session. Build it with [`SessionConfig::new`] and the `with_` setters; the struct is
/// `#[non_exhaustive]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SessionConfig {
    /// Number of controllers, 1 to 8. The `MSP_SET_PASSTHROUGH` reply carries it, and a controller
    /// index at or above it gets the invalid-channel acknowledge. Default 4.
    pub esc_count: u8,
    /// The line number (for [`EscLines::select`]) of each controller index. Default: the identity.
    pub line_map: [u8; MAX_ESC],
    /// How long the lines are held high and silent after the grant before the first byte goes out
    /// on any line. Default 3000.
    pub park_ms: u32,
    /// How long all lines are held low at the exit. Default 25.
    pub break_ms: u32,
    /// How long the MSP phase waits for a valid request while the lines are not held. Default
    /// 5000.
    pub idle_ms: u32,
    /// The waits of the bootloader client.
    pub timeouts: Timeouts,
    /// The identity values of the MSP and housekeeping replies.
    pub info: HostInfo,
}

impl SessionConfig {
    /// The defaults.
    pub const fn new() -> Self {
        Self {
            esc_count: DEFAULT_ESC_COUNT,
            line_map: [0, 1, 2, 3, 4, 5, 6, 7],
            park_ms: DEFAULT_PARK_MS,
            break_ms: DEFAULT_BREAK_MS,
            idle_ms: DEFAULT_IDLE_MS,
            timeouts: Timeouts::new(),
            info: HostInfo::new().with_motor_count(DEFAULT_ESC_COUNT),
        }
    }

    /// Sets [`esc_count`](Self::esc_count), limited to 1 to 8, and the motor count of
    /// [`info`](Self::info) with it. Call [`with_info`](Self::with_info) after this to set the
    /// motor count apart.
    #[must_use]
    pub const fn with_esc_count(mut self, esc_count: u8) -> Self {
        let esc_count = if esc_count < 1 {
            1
        } else if esc_count > MAX_ESC as u8 {
            MAX_ESC as u8
        } else {
            esc_count
        };
        self.esc_count = esc_count;
        self.info = self.info.with_motor_count(esc_count);
        self
    }

    /// Sets [`line_map`](Self::line_map).
    #[must_use]
    pub const fn with_line_map(mut self, line_map: [u8; MAX_ESC]) -> Self {
        self.line_map = line_map;
        self
    }

    /// Sets [`park_ms`](Self::park_ms).
    #[must_use]
    pub const fn with_park_ms(mut self, park_ms: u32) -> Self {
        self.park_ms = park_ms;
        self
    }

    /// Sets [`break_ms`](Self::break_ms).
    #[must_use]
    pub const fn with_break_ms(mut self, break_ms: u32) -> Self {
        self.break_ms = break_ms;
        self
    }

    /// Sets [`idle_ms`](Self::idle_ms).
    #[must_use]
    pub const fn with_idle_ms(mut self, idle_ms: u32) -> Self {
        self.idle_ms = idle_ms;
        self
    }

    /// Sets [`timeouts`](Self::timeouts).
    #[must_use]
    pub const fn with_timeouts(mut self, timeouts: Timeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    /// Sets [`info`](Self::info).
    #[must_use]
    pub const fn with_info(mut self, info: HostInfo) -> Self {
        self.info = info;
        self
    }
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// How the firmware hands over the lines when the configurator asks for them.
pub trait Lender {
    /// The lines handed over.
    type Lines: EscLines;

    /// Hands the lines over, or returns `None` to refuse (for example while the vehicle is armed).
    /// The session calls it at `MSP_SET_PASSTHROUGH` only, and never while it holds lines.
    ///
    /// A future dropped before it completes must not have given the lines away.
    async fn lend(&mut self) -> Option<Self::Lines>;
}

/// Why [`Passthrough::run`] returned. The lines exit is done before it returns, if lines were
/// held.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Exit<E> {
    /// `idle_ms` passed without a valid MSP request, before the lines were granted.
    Idle,
    /// The configurator sent `InterfaceExit`.
    Exited,
    /// The host stream reported end of input: a read returned no byte, or a write accepted none.
    Closed,
    /// The host stream failed to read or write.
    Host(E),
}

/// Why [`Passthrough::release`] gave nothing back. The session is unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ReleaseError {
    /// The session holds no lines.
    NotHeld,
    /// The lines are held and the exit break is not done: call [`Passthrough::finish`].
    ExitOwed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owed {
    /// The exit break is done, or the lines were never granted.
    No,
    /// The lines were taken and the break has not started.
    Yes,
    /// The lines are low and not yet released.
    Low,
}

/// A session over lines `L`: the host side, the bootloader client and the state in between. See
/// the module documentation.
#[derive(Debug)]
pub struct Passthrough<L: EscLines> {
    config: SessionConfig,
    host: HostSide,
    lines: Option<Client<L>>,
    owed: Owed,
    parked: bool,
    connected: Option<u8>,
}

enum Outcome {
    Reply(usize),
    Passthrough,
    FourWay {
        command: Command,
        address: u16,
        len: usize,
    },
    Restarted {
        command: u8,
        len: usize,
    },
}

impl<L: EscLines> Passthrough<L> {
    /// A session with nothing received and no lines.
    pub const fn new(config: SessionConfig) -> Self {
        Self {
            config,
            host: HostSide::new(),
            lines: None,
            owed: Owed::No,
            parked: false,
            connected: None,
        }
    }

    /// True while the session holds the lines (from the grant until [`Passthrough::release`]).
    pub const fn holds_lines(&self) -> bool {
        self.lines.is_some()
    }

    /// Serves the configurator until it leaves. `preload` is bytes the caller already took from
    /// the stream (the trigger request) and that are processed first. `rx` and `tx` are the host
    /// stream's halves; `rx.read` must be cancel-safe. `lender` is asked at
    /// `MSP_SET_PASSTHROUGH`; `delay` is the only clock; `battery` is called at each
    /// `MSP_BATTERY_STATE` request.
    ///
    /// The future may be dropped at any await; then call [`Passthrough::finish`].
    pub async fn run<R, W, N, D, B>(
        &mut self,
        preload: &[u8],
        rx: &mut R,
        tx: &mut W,
        lender: &mut N,
        delay: &mut D,
        mut battery: B,
    ) -> Exit<R::Error>
    where
        R: Read,
        W: Write<Error = R::Error>,
        N: Lender<Lines = L>,
        D: DelayNs,
        B: FnMut() -> msp::BatteryState,
    {
        if self.owed == Owed::Low {
            self.finish(delay).await;
        }
        let exit = match self
            .serve(preload, rx, tx, lender, delay, &mut battery)
            .await
        {
            Err(exit) => exit,
            Ok(never) => match never {},
        };
        self.finish(delay).await;
        exit
    }

    /// Ends the session: if the lines are held and the exit break is owed, does it now (all lines
    /// low for `break_ms`, then released), and brings the host side back to MSP. Idempotent. Call it
    /// after cancelling [`Passthrough::run`]; [`Passthrough::run`] calls it itself when it
    /// returns.
    ///
    /// If the line refuses `set_break`, the exit stays owed and [`Passthrough::release`] keeps
    /// refusing; call this again.
    pub async fn finish<D: DelayNs>(&mut self, delay: &mut D) {
        if let Some(client) = self.lines.as_mut()
            && self.owed != Owed::No
        {
            if self.owed == Owed::Yes && client.line_mut().set_break(true).is_ok() {
                self.owed = Owed::Low;
            }
            if self.owed == Owed::Low {
                delay.delay_ms(self.config.break_ms).await;
                if client.line_mut().set_break(false).is_ok() {
                    self.owed = Owed::No;
                }
            }
        }
        self.host.exited();
        self.parked = false;
        self.connected = None;
    }

    /// Gives the lines back.
    ///
    /// # Errors
    ///
    /// [`ReleaseError::NotHeld`] if the session holds none; [`ReleaseError::ExitOwed`] while they
    /// are held and the exit break is not done (the session keeps them).
    pub fn release(&mut self) -> Result<L, ReleaseError> {
        if self.lines.is_none() {
            return Err(ReleaseError::NotHeld);
        }
        if self.owed != Owed::No {
            return Err(ReleaseError::ExitOwed);
        }
        self.lines
            .take()
            .map(Client::into_line)
            .ok_or(ReleaseError::NotHeld)
    }

    async fn serve<R, W, N, D, B>(
        &mut self,
        preload: &[u8],
        rx: &mut R,
        tx: &mut W,
        lender: &mut N,
        delay: &mut D,
        battery: &mut B,
    ) -> Result<Infallible, Exit<R::Error>>
    where
        R: Read,
        W: Write<Error = R::Error>,
        N: Lender<Lines = L>,
        D: DelayNs,
        B: FnMut() -> msp::BatteryState,
    {
        let mut out = [0u8; fourway::MAX_REPLY_LEN];
        let mut req = [0u8; MAX_PARAMS];
        let mut input = Input::new(preload);
        loop {
            if self.lines.is_some() && self.host.mode() == Mode::FourWay && !self.parked {
                delay.delay_ms(self.config.park_ms).await;
                self.parked = true;
            }
            let timed = self.lines.is_none() && self.host.mode() == Mode::Msp;
            let idle_ms = self.config.idle_ms;
            let outcome = {
                let idle = pin!(async {
                    if timed {
                        delay.delay_ms(idle_ms).await;
                    } else {
                        pending::<()>().await;
                    }
                });
                next_outcome(
                    &mut self.host,
                    &self.config.info,
                    &mut out,
                    &mut req,
                    &mut input,
                    rx,
                    idle,
                )
                .await?
            };
            match outcome {
                Outcome::Reply(len) => {
                    let len = with_battery(&mut out, len, battery);
                    send(tx, out.get(..len).unwrap_or_default()).await?;
                }
                Outcome::Passthrough => self.passthrough(lender, tx, &mut out).await?,
                Outcome::FourWay {
                    command,
                    address,
                    len,
                } => {
                    let params = req.get(..len).unwrap_or_default();
                    let len = self.execute(command, address, params, &mut out).await;
                    send(tx, out.get(..len).unwrap_or_default()).await?;
                    if command == Command::InterfaceExit {
                        return Err(Exit::Exited);
                    }
                }
                Outcome::Restarted { command, len } => {
                    self.finish(delay).await;
                    input.inject(command, req.get(..len).unwrap_or_default());
                }
            }
        }
    }

    /// `MSP_SET_PASSTHROUGH`: ask the lender (unless lines are held), then grant or refuse.
    async fn passthrough<W: Write, N: Lender<Lines = L>>(
        &mut self,
        lender: &mut N,
        tx: &mut W,
        out: &mut [u8],
    ) -> Result<(), Exit<W::Error>> {
        if self.lines.is_none() {
            match lender.lend().await {
                Some(lines) => {
                    self.lines = Some(Client::new(lines).with_timeouts(self.config.timeouts));
                    self.owed = Owed::Yes;
                }
                None => {
                    if let Ok(len) = self.host.refuse(out) {
                        send(tx, out.get(..len).unwrap_or_default()).await?;
                    }
                    return Ok(());
                }
            }
        }
        if let Ok(len) = self.host.grant(self.config.esc_count, out) {
            self.owed = Owed::Yes;
            self.parked = false;
            self.connected = None;
            send(tx, out.get(..len).unwrap_or_default()).await?;
        }
        Ok(())
    }

    /// The client, if a controller is connected.
    fn connected_client(&mut self) -> Option<&mut Client<L>> {
        if self.connected.is_some() {
            self.lines.as_mut()
        } else {
            None
        }
    }

    /// The client with the line of controller `index` selected; `None` if the index is out of
    /// range or the line refuses.
    fn client_for(&mut self, index: u8) -> Option<&mut Client<L>> {
        let line = self.config.line_map.get(usize::from(index)).copied()?;
        let client = self.lines.as_mut()?;
        client.select(line).ok()?;
        Some(client)
    }

    /// Runs one four-way request and writes its reply at the start of `out`. Returns the length.
    async fn execute(
        &mut self,
        command: Command,
        address: u16,
        params: &[u8],
        out: &mut [u8],
    ) -> usize {
        let first = params.first().copied().unwrap_or(0);
        let request = fourway::Request {
            command,
            address,
            params,
        };
        let reply = |ack: Ack, param: u8, out: &mut [u8]| {
            fourway::encode_reply(command, address, &[param], ack, out).unwrap_or(0)
        };
        let known_channel = first < self.config.esc_count;
        match command {
            Command::InterfaceExit => reply(Ack::Ok, 0, out),
            Command::ProtocolGetVersion
            | Command::InterfaceGetName
            | Command::InterfaceGetVersion => {
                match host::answer_housekeeping(&self.config.info, &request, out) {
                    Some(Ok(len)) => len,
                    _ => reply(Ack::DGeneralError, 0, out),
                }
            }
            Command::InterfaceTestAlive => {
                let Some(client) = self.connected_client() else {
                    return reply(Ack::DGeneralError, 0, out);
                };
                if client.keep_alive().await.is_ok() {
                    reply(Ack::Ok, 0, out)
                } else {
                    self.connected = None;
                    reply(Ack::DGeneralError, 0, out)
                }
            }
            Command::DeviceInitFlash => {
                if !known_channel {
                    return reply(Ack::IInvalidChannel, first, out);
                }
                self.connected = None;
                let info = match self.client_for(first) {
                    Some(client) => client.connect().await.ok(),
                    None => None,
                };
                match info {
                    Some(info) => {
                        self.connected = Some(first);
                        let [low, high] = info.signature().to_le_bytes();
                        fourway::encode_reply(
                            command,
                            address,
                            &[low, high, info.pin_code, INTERFACE_MODE_ARM],
                            Ack::Ok,
                            out,
                        )
                        .unwrap_or(0)
                    }
                    None => reply(Ack::DGeneralError, first, out),
                }
            }
            Command::DeviceReset => {
                if !known_channel {
                    return reply(Ack::IInvalidChannel, first, out);
                }
                self.connected = None;
                let done = match self.client_for(first) {
                    Some(client) => client.run_application().await.is_ok(),
                    None => false,
                };
                if done {
                    reply(Ack::Ok, first, out)
                } else {
                    reply(Ack::DGeneralError, first, out)
                }
            }
            Command::DeviceRead => {
                let wanted = if first == 0 {
                    MAX_PARAMS
                } else {
                    usize::from(first)
                };
                let mut data = [0u8; MAX_PARAMS];
                let (Some(buf), Some(client)) = (data.get_mut(..wanted), self.connected_client())
                else {
                    return reply(Ack::DGeneralError, first, out);
                };
                match client.read(address, buf).await {
                    Ok(()) => host::encode_success(&request, buf, out).unwrap_or(0),
                    Err(_) => reply(Ack::DGeneralError, first, out),
                }
            }
            Command::DeviceWrite => {
                if params.len() < MIN_WRITE || params.len() > MAX_PARAMS {
                    return reply(Ack::IInvalidParam, 0, out);
                }
                let Some(client) = self.connected_client() else {
                    return reply(Ack::DGeneralError, 0, out);
                };
                match client.write(address, params).await {
                    Ok(()) => reply(Ack::Ok, 0, out),
                    Err(_) => reply(Ack::DGeneralError, 0, out),
                }
            }
            Command::DevicePageErase => {
                let Some(client) = self.connected_client() else {
                    return reply(Ack::DGeneralError, first, out);
                };
                let page_address = u16::from(first).wrapping_shl(PAGE_SHIFT);
                match client.erase(page_address).await {
                    Ok(()) => reply(Ack::Ok, first, out),
                    Err(_) => reply(Ack::DGeneralError, first, out),
                }
            }
            Command::InterfaceSetMode => reply(Ack::Ok, first, out),
            Command::DeviceWriteEeprom => reply(Ack::DGeneralError, 0, out),
            Command::DeviceVerify
            | Command::DeviceReadEeprom
            | Command::DeviceEraseAll
            | Command::DeviceC2ckLow
            | Command::Unknown(_) => reply(Ack::IInvalidCmd, 0, out),
        }
    }
}

/// Writes all of `bytes` and flushes. A write that takes no byte is the end of the stream.
async fn send<W: Write>(tx: &mut W, mut bytes: &[u8]) -> Result<(), Exit<W::Error>> {
    while !bytes.is_empty() {
        let written = tx.write(bytes).await.map_err(Exit::Host)?;
        if written == 0 {
            return Err(Exit::Closed);
        }
        bytes = bytes.get(written..).unwrap_or_default();
    }
    tx.flush().await.map_err(Exit::Host)
}

/// The reply to `MSP_BATTERY_STATE` carries the battery figure the firmware gives at this moment.
fn with_battery<B: FnMut() -> msp::BatteryState>(
    out: &mut [u8],
    len: usize,
    battery: &mut B,
) -> usize {
    let is_battery_reply = matches!(
        (out.first(), out.get(2), out.get(4)),
        (Some(b'$'), Some(b'>'), Some(&msp::MSP_BATTERY_STATE))
    );
    if is_battery_reply {
        msp::encode_battery_state(battery(), out).unwrap_or(len)
    } else {
        len
    }
}

/// Where the bytes to feed come from: a request put back after a restart, the caller's preload, and
/// what was read.
struct Input<'p> {
    front: [u8; msp::MAX_FRAME_LEN],
    front_pos: usize,
    front_len: usize,
    preload: &'p [u8],
    buf: [u8; READ_LEN],
    pos: usize,
    len: usize,
}

impl<'p> Input<'p> {
    const fn new(preload: &'p [u8]) -> Self {
        Self {
            front: [0; msp::MAX_FRAME_LEN],
            front_pos: 0,
            front_len: 0,
            preload,
            buf: [0; READ_LEN],
            pos: 0,
            len: 0,
        }
    }

    fn pending(&self) -> &[u8] {
        if self.front_pos < self.front_len {
            self.front.get(self.front_pos..self.front_len)
        } else if !self.preload.is_empty() {
            Some(self.preload)
        } else {
            self.buf.get(self.pos..self.len)
        }
        .unwrap_or_default()
    }

    fn consume(&mut self, used: usize) {
        if self.front_pos < self.front_len {
            self.front_pos = (self.front_pos + used).min(self.front_len);
        } else if !self.preload.is_empty() {
            self.preload = self.preload.get(used..).unwrap_or_default();
        } else {
            self.pos = (self.pos + used).min(self.len);
        }
    }

    /// Puts an MSP v1 request in front of everything else.
    fn inject(&mut self, command: u8, payload: &[u8]) {
        let len = payload.len().min(msp::MAX_PAYLOAD);
        let payload = payload.get(..len).unwrap_or_default();
        let mut checksum = len as u8 ^ command;
        for &byte in payload {
            checksum ^= byte;
        }
        let mut at = 0;
        let mut put = |byte: u8| {
            if let Some(slot) = self.front.get_mut(at) {
                *slot = byte;
                at += 1;
            }
        };
        for byte in [b'$', b'M', b'<', len as u8, command] {
            put(byte);
        }
        for &byte in payload {
            put(byte);
        }
        put(checksum);
        self.front_pos = 0;
        self.front_len = at;
    }

    async fn fill<R: Read, I: Future<Output = ()>>(
        &mut self,
        rx: &mut R,
        mut idle: Pin<&mut I>,
    ) -> Result<(), Exit<R::Error>> {
        self.pos = 0;
        self.len = 0;
        match race(rx.read(&mut self.buf), &mut idle).await {
            Either::Left(Ok(0)) => Err(Exit::Closed),
            Either::Left(Ok(got)) => {
                self.len = got.min(READ_LEN);
                Ok(())
            }
            Either::Left(Err(error)) => Err(Exit::Host(error)),
            Either::Right(()) => Err(Exit::Idle),
        }
    }
}

/// Feeds the host side until it has something to act on, reading when it has no bytes. `idle`
/// ends the wait with [`Exit::Idle`]; it is polled before the read.
async fn next_outcome<R: Read, I: Future<Output = ()>>(
    host: &mut HostSide,
    info: &HostInfo,
    out: &mut [u8],
    req: &mut [u8; MAX_PARAMS],
    input: &mut Input<'_>,
    rx: &mut R,
    mut idle: Pin<&mut I>,
) -> Result<Outcome, Exit<R::Error>> {
    loop {
        let (used, found) = {
            let (used, event) = host.feed(input.pending(), info, out);
            let found = match event {
                // `out` always holds the largest reply, so a reply that does not fit cannot happen.
                None | Some(Event::ReplyFailed(_)) => None,
                Some(Event::Reply { len }) => Some(Outcome::Reply(len)),
                Some(Event::PassthroughRequested) => Some(Outcome::Passthrough),
                Some(Event::FourWay(request)) => {
                    let len = request.params.len();
                    if let Some(slot) = req.get_mut(..len) {
                        slot.copy_from_slice(request.params);
                    }
                    Some(Outcome::FourWay {
                        command: request.command,
                        address: request.address,
                        len,
                    })
                }
                Some(Event::Restarted(request)) => {
                    let len = request.payload.len();
                    if let Some(slot) = req.get_mut(..len) {
                        slot.copy_from_slice(request.payload);
                    }
                    Some(Outcome::Restarted {
                        command: request.command,
                        len,
                    })
                }
            };
            (used, found)
        };
        input.consume(used);
        if let Some(found) = found {
            return Ok(found);
        }
        if input.pending().is_empty() {
            input.fill(rx, idle.as_mut()).await?;
        }
    }
}

enum Either<A, B> {
    Left(A),
    Right(B),
}

/// Polls `b` first, then `a`; the first to finish wins and `a` is dropped if it did not.
async fn race<A: Future, B: Future + Unpin>(a: A, b: &mut B) -> Either<A::Output, B::Output> {
    let mut a = pin!(a);
    poll_fn(|cx| {
        if let Poll::Ready(value) = Pin::new(&mut *b).poll(cx) {
            return Poll::Ready(Either::Right(value));
        }
        match a.as_mut().poll(cx) {
            Poll::Ready(value) => Poll::Ready(Either::Left(value)),
            Poll::Pending => Poll::Pending,
        }
    })
    .await
}
