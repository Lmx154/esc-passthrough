//! The host side: what the configurator's bytes ask for, and the bytes to send back.
//!
//! [`HostSide`] is sans-IO. The application feeds it the bytes that arrive from the computer
//! ([`HostSide::feed`]) and it says, one event at a time, what to do next. It holds one MSP request
//! parser, one four-way request parser and the mode. It reads and writes nothing, owns no timer and
//! knows no ESC: it never looks at an ESC index, an address or a parameter.
//!
//! # Modes
//!
//! **MSP** (the start). A complete request for `MSP_API_VERSION`, `MSP_FC_VARIANT`,
//! `MSP_BATTERY_STATE` or `MSP_MOTOR_CONFIG` is answered here, from the values in [`HostInfo`]
//! ([`Event::Reply`]). `MSP_SET_PASSTHROUGH` is not: it is reported ([`Event::PassthroughRequested`])
//! and the application answers with [`HostSide::grant`] (the reply carries the ESC count and the
//! mode becomes four-way) or [`HostSide::refuse`] (the MSP error reply; the mode stays MSP). Any
//! other command gets the MSP error reply. A malformed frame gets nothing.
//!
//! **Four-way**. Each complete request with a good CRC is handed out ([`Event::FourWay`]) with its
//! command, address and parameters, all opaque. The application answers with [`encode_success`] or
//! [`encode_failure`], which echo the request's command and address as the configurator checks
//! (`four_way.ts:400-402`), or with [`answer_housekeeping`] for the three commands that need no
//! motor controller. A request with a bad CRC is answered here and is not handed out
//! ([`Event::Reply`]): the reply is the failure reply of [`encode_failure`] built from the bytes
//! received (command, address, first parameter) with acknowledge `0x03`, `ACK_I_INVALID_CRC`. The
//! framing is ArduPilot's `blheli_send_reply` (`AP_BLHeli.cpp:610-623`), but ArduPilot itself never
//! sends this acknowledge: it drops a frame with a bad CRC without a reply (`AP_BLHeli.cpp:294-300`;
//! the code is only defined at `blheli_4way_protocol.h:160`). When the application has dealt with `InterfaceExit` (or
//! decided to end the session for any reason) it calls [`HostSide::exited`] and the mode is MSP
//! again.
//!
//! # A configurator that restarts
//!
//! A configurator that is closed and reopened starts with MSP again while the mode is still
//! four-way. Bytes go to the four-way parser first. Only bytes that arrive while that parser is
//! between frames and that are not the four-way start byte `0x2F` go to the MSP parser; a complete,
//! checksum-valid MSP request found that way is reported as [`Event::Restarted`], and the
//! application ends its session (and calls [`HostSide::exited`]). MSP errors in this mode are
//! ignored. A `$` inside a four-way frame never reaches the MSP parser. The start byte `0x2F`
//! cannot be part of an MSP request seen this way: a request with a `0x2F` in it (as length,
//! command, payload or checksum) is not recognised, and a stray `0x2F` between frames starts a
//! four-way frame that swallows what follows for as many bytes as its count says.
//!
//! # Replies
//!
//! One call to [`HostSide::feed`] stops at its first event, so the application acts before the next
//! byte is looked at, and writes at most one reply into `out`. A reply is only ever written after the
//! last byte of a complete request has been fed. [`MAX_REPLY_LEN`] bytes always suffice for the
//! replies the crate writes itself; a smaller buffer yields [`Event::ReplyFailed`] and nothing is
//! written.

use crate::EncodeError;
use crate::fourway::{self, Ack, Command, MAX_PARAMS, REQUEST_START};
use crate::msp::{self, ApiVersion, BatteryState, MotorConfig};

/// Bytes of `out` that always suffice for a reply [`HostSide::feed`] writes (the 17-byte
/// `MSP_BATTERY_STATE` reply is the longest).
pub const MAX_REPLY_LEN: usize = msp::BATTERY_STATE_PAYLOAD_LEN + msp::FRAME_OVERHEAD;

/// Default answer to `cmd_ProtocolGetVersion`: `SERIAL_4WAY_PROTOCOL_VER`, 107
/// (`blheli_4way_protocol.h:123`, answered at `AP_BLHeli.cpp:993-998`).
pub const DEFAULT_PROTOCOL_VERSION: u8 = 107;

/// Default answer to `cmd_InterfaceGetName`: the five bytes ArduPilot sends, `04` and `ARDU`
/// (`AP_BLHeli.cpp:1000-1004`).
pub const DEFAULT_INTERFACE_NAME: &[u8] = &[4, b'A', b'R', b'D', b'U'];

/// Default answer to `cmd_InterfaceGetVersion`: high and low byte of `SERIAL_4WAY_VERSION` 20005
/// divided by 100, `[200, 5]` (`blheli_4way_protocol.h:119-133`, answered at
/// `AP_BLHeli.cpp:1006-1010`).
pub const DEFAULT_INTERFACE_VERSION: [u8; 2] = [200, 5];

/// The values the host side answers with. Build it with [`HostInfo::new`] and the `with_` setters.
///
/// The defaults are ArduPilot's: MSP API version 0.1.42 (`msp_protocol.h:62-65`), variant `ARDU`
/// (`msp_protocol.h:75`), no battery reading, no motors, and the three four-way answers above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct HostInfo {
    /// Contents of the `MSP_API_VERSION` reply.
    pub api_version: ApiVersion,
    /// The four ASCII bytes of the `MSP_FC_VARIANT` reply.
    pub fc_variant: [u8; 4],
    /// Contents of the `MSP_BATTERY_STATE` reply.
    pub battery: BatteryState,
    /// Number of motors in the `MSP_MOTOR_CONFIG` reply.
    pub motor_count: u8,
    /// The one parameter byte of the `cmd_ProtocolGetVersion` answer.
    pub protocol_version: u8,
    /// The parameters of the `cmd_InterfaceGetName` answer (1 to 256 bytes).
    pub interface_name: &'static [u8],
    /// The two parameter bytes of the `cmd_InterfaceGetVersion` answer.
    pub interface_version: [u8; 2],
}

impl HostInfo {
    /// ArduPilot's values, as described on the type.
    pub const fn new() -> Self {
        Self {
            api_version: ApiVersion::new(0, 1, 42),
            fc_variant: *b"ARDU",
            battery: BatteryState::new(0),
            motor_count: 0,
            protocol_version: DEFAULT_PROTOCOL_VERSION,
            interface_name: DEFAULT_INTERFACE_NAME,
            interface_version: DEFAULT_INTERFACE_VERSION,
        }
    }

    /// Sets the `MSP_API_VERSION` triple.
    pub const fn with_api_version(mut self, api_version: ApiVersion) -> Self {
        self.api_version = api_version;
        self
    }

    /// Sets the four bytes of the `MSP_FC_VARIANT` reply.
    pub const fn with_fc_variant(mut self, fc_variant: [u8; 4]) -> Self {
        self.fc_variant = fc_variant;
        self
    }

    /// Sets the `MSP_BATTERY_STATE` values.
    pub const fn with_battery(mut self, battery: BatteryState) -> Self {
        self.battery = battery;
        self
    }

    /// Sets the motor count of the `MSP_MOTOR_CONFIG` reply.
    pub const fn with_motor_count(mut self, motor_count: u8) -> Self {
        self.motor_count = motor_count;
        self
    }

    /// Sets the `cmd_ProtocolGetVersion` answer.
    pub const fn with_protocol_version(mut self, protocol_version: u8) -> Self {
        self.protocol_version = protocol_version;
        self
    }

    /// Sets the `cmd_InterfaceGetName` answer. [`answer_housekeeping`] fails with
    /// [`EncodeError::InvalidLength`] unless it holds 1 to 256 bytes.
    pub const fn with_interface_name(mut self, interface_name: &'static [u8]) -> Self {
        self.interface_name = interface_name;
        self
    }

    /// Sets the `cmd_InterfaceGetVersion` answer.
    pub const fn with_interface_version(mut self, interface_version: [u8; 2]) -> Self {
        self.interface_version = interface_version;
        self
    }
}

impl Default for HostInfo {
    fn default() -> Self {
        Self::new()
    }
}

/// Which protocol the host side is speaking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// MSP: the start, and again after [`HostSide::exited`].
    Msp,
    /// Four-way: after [`HostSide::grant`].
    FourWay,
}

/// What [`HostSide::feed`] found. The borrowed variants are valid until the next call on the
/// [`HostSide`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event<'a> {
    /// A reply of `len` bytes (at least 1) is at the start of `out`: send it.
    Reply {
        /// Length of the reply in `out`.
        len: usize,
    },
    /// A reply was due but did not fit `out`; nothing was written and the request is consumed.
    ReplyFailed(EncodeError),
    /// A complete `MSP_SET_PASSTHROUGH` request without payload. Call [`HostSide::grant`] or
    /// [`HostSide::refuse`]. Until you do, [`HostSide::feed`] consumes nothing and reports this
    /// again.
    PassthroughRequested,
    /// A complete four-way request with a good CRC (four-way mode only). Answer it.
    FourWay(fourway::Request<'a>),
    /// A complete, checksum-valid MSP request arrived between four-way frames: the configurator
    /// started over. End the session and call [`HostSide::exited`].
    Restarted(msp::Request<'a>),
}

/// Why [`HostSide::grant`] or [`HostSide::refuse`] did nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum DecisionError {
    /// No `MSP_SET_PASSTHROUGH` request is waiting for an answer.
    NotRequested,
    /// The reply did not fit `out`; the request is still waiting and the mode is unchanged.
    Encode(EncodeError),
}

enum Found {
    Nothing,
    Reply(Result<usize, EncodeError>),
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

/// Incremental host side: MSP and four-way request parsers and the mode. See the module
/// documentation.
///
/// Storage: the two parsers (about 255 and 256 bytes), 256 bytes in which a request handed out as an
/// event is kept (the parsers' own storage cannot be lent across the loop that feeds them), and a
/// few counters. Nothing is allocated.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HostSide {
    msp: msp::RequestParser,
    four: fourway::RequestParser,
    mode: Mode,
    pending: bool,
    // The four-way parser is between frames. Kept from what it reports: it is in its start state
    // after a good frame, after a bad one that did not end in 0x2F, and when it has seen nothing but
    // bytes other than 0x2F since.
    idle: bool,
    // Bytes of the current four-way frame after the start byte, up to five: command, address,
    // count, first parameter. The parser does not keep them for a frame that fails its CRC.
    head: [u8; 5],
    seen: usize,
    held: [u8; MAX_PARAMS],
}

impl Default for HostSide {
    fn default() -> Self {
        Self::new()
    }
}

fn store(held: &mut [u8; MAX_PARAMS], source: &[u8]) {
    if let Some(target) = held.get_mut(..source.len()) {
        target.copy_from_slice(source);
    }
}

impl HostSide {
    /// A host side in MSP mode, with nothing received.
    pub const fn new() -> Self {
        Self {
            msp: msp::RequestParser::new(),
            four: fourway::RequestParser::new(),
            mode: Mode::Msp,
            pending: false,
            idle: true,
            head: [0; 5],
            seen: 0,
            held: [0; MAX_PARAMS],
        }
    }

    /// The current mode.
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// Feeds bytes until the first event. Returns how many bytes were used (the one that completed
    /// the event included) and the event. Call again with the rest of the slice;
    /// `(bytes.len(), None)` means all of it was used with nothing to report.
    ///
    /// `info` supplies the values of the MSP replies; `out` receives the reply when the event is
    /// [`Event::Reply`]. At most one reply is written per call, and only after the last byte of a
    /// complete request.
    pub fn feed(
        &mut self,
        bytes: &[u8],
        info: &HostInfo,
        out: &mut [u8],
    ) -> (usize, Option<Event<'_>>) {
        let (used, found) = if self.pending {
            (0, Found::Passthrough)
        } else {
            match self.mode {
                Mode::Msp => self.scan_msp(bytes, info, out),
                Mode::FourWay => self.scan_four_way(bytes, out),
            }
        };
        let event = match found {
            Found::Nothing => None,
            Found::Reply(Ok(len)) => Some(Event::Reply { len }),
            Found::Reply(Err(error)) => Some(Event::ReplyFailed(error)),
            Found::Passthrough => Some(Event::PassthroughRequested),
            Found::FourWay {
                command,
                address,
                len,
            } => Some(Event::FourWay(fourway::Request {
                command,
                address,
                params: self.held.get(..len).unwrap_or(&[]),
            })),
            Found::Restarted { command, len } => Some(Event::Restarted(msp::Request {
                command,
                payload: self.held.get(..len).unwrap_or(&[]),
            })),
        };
        (used, event)
    }

    fn scan_msp(&mut self, bytes: &[u8], info: &HostInfo, out: &mut [u8]) -> (usize, Found) {
        for (index, &byte) in bytes.iter().enumerate() {
            let Some(Ok(request)) = self.msp.push(byte) else {
                continue;
            };
            let command = request.command;
            let kind = request.kind();
            let no_payload = request.payload.is_empty();
            let found = match kind {
                msp::Command::ApiVersion => {
                    Found::Reply(msp::encode_api_version(info.api_version, out))
                }
                msp::Command::FcVariant => {
                    Found::Reply(msp::encode_fc_variant(info.fc_variant, out))
                }
                msp::Command::BatteryState => {
                    Found::Reply(msp::encode_battery_state(info.battery, out))
                }
                msp::Command::MotorConfig => Found::Reply(msp::encode_motor_config(
                    MotorConfig::new(info.motor_count),
                    out,
                )),
                msp::Command::SetPassthrough if no_payload => {
                    self.pending = true;
                    Found::Passthrough
                }
                _ => Found::Reply(msp::encode_error(command, out)),
            };
            return (index + 1, found);
        }
        (bytes.len(), Found::Nothing)
    }

    fn scan_four_way(&mut self, bytes: &[u8], out: &mut [u8]) -> (usize, Found) {
        for (index, &byte) in bytes.iter().enumerate() {
            let used = index + 1;
            if self.idle && byte != REQUEST_START {
                if let Some(Ok(request)) = self.msp.push(byte) {
                    let command = request.command;
                    let len = request.payload.len();
                    store(&mut self.held, request.payload);
                    return (used, Found::Restarted { command, len });
                }
                continue;
            }
            if self.idle {
                self.msp.reset();
                self.idle = false;
                self.seen = 0;
            } else if let Some(slot) = self.head.get_mut(self.seen) {
                *slot = byte;
                self.seen += 1;
            }
            match self.four.push(byte) {
                None => {}
                Some(Ok(request)) => {
                    self.idle = true;
                    let len = request.params.len();
                    store(&mut self.held, request.params);
                    return (
                        used,
                        Found::FourWay {
                            command: request.command,
                            address: request.address,
                            len,
                        },
                    );
                }
                Some(Err(_)) => {
                    let [command, address_high, address_low, _count, first] = self.head;
                    let reply = fourway::encode_reply(
                        Command::from_code(command),
                        u16::from_be_bytes([address_high, address_low]),
                        &[first],
                        Ack::IInvalidCrc,
                        out,
                    );
                    if byte == REQUEST_START {
                        self.seen = 0;
                    } else {
                        self.idle = true;
                    }
                    return (used, Found::Reply(reply));
                }
            }
        }
        (bytes.len(), Found::Nothing)
    }

    fn reset_parsers(&mut self) {
        self.msp.reset();
        self.four.reset();
        self.idle = true;
        self.seen = 0;
    }

    /// Answers the waiting `MSP_SET_PASSTHROUGH` request with the ESC count and switches to
    /// four-way mode, with both parsers reset. Returns the length of the reply written at the start
    /// of `out`; send it, then feed the rest of the stream.
    ///
    /// # Errors
    ///
    /// [`DecisionError::NotRequested`] if no such request is waiting;
    /// [`DecisionError::Encode`] if `out` is shorter than 7 bytes (the request keeps waiting).
    pub fn grant(&mut self, esc_count: u8, out: &mut [u8]) -> Result<usize, DecisionError> {
        if !self.pending {
            return Err(DecisionError::NotRequested);
        }
        let len = msp::encode_set_passthrough(esc_count, out).map_err(DecisionError::Encode)?;
        self.pending = false;
        self.mode = Mode::FourWay;
        self.reset_parsers();
        Ok(len)
    }

    /// Answers the waiting `MSP_SET_PASSTHROUGH` request with the MSP error reply. The mode stays
    /// MSP and the configurator may ask again. Returns the length of the reply (6).
    ///
    /// # Errors
    ///
    /// [`DecisionError::NotRequested`] if no such request is waiting;
    /// [`DecisionError::Encode`] if `out` is shorter than 6 bytes (the request keeps waiting).
    pub fn refuse(&mut self, out: &mut [u8]) -> Result<usize, DecisionError> {
        if !self.pending {
            return Err(DecisionError::NotRequested);
        }
        let len =
            msp::encode_error(msp::MSP_SET_PASSTHROUGH, out).map_err(DecisionError::Encode)?;
        self.pending = false;
        Ok(len)
    }

    /// Reports the four-way session over (`InterfaceExit` dealt with, or the application ended it
    /// for another reason, for example after [`Event::Restarted`]): the mode is MSP again, any
    /// request waiting for an answer is dropped and both parsers are reset.
    pub fn exited(&mut self) {
        self.mode = Mode::Msp;
        self.pending = false;
        self.reset_parsers();
    }
}

/// Answers the three four-way commands that need no motor controller: `cmd_ProtocolGetVersion`
/// (one byte), `cmd_InterfaceGetName` (the name) and `cmd_InterfaceGetVersion` (two bytes), with
/// acknowledge OK, the request's command and address, and the values in `info`. Returns `None` for
/// any other command, which this function does not answer. Otherwise returns the length written
/// at the start of `out`.
///
/// ArduPilot answers them the same way: `AP_BLHeli.cpp:993-1010`.
///
/// # Errors
///
/// The `Some` holds [`EncodeError::BufferTooSmall`] if `out` is too short, or
/// [`EncodeError::InvalidLength`] if `info.interface_name` is not 1 to 256 bytes.
pub fn answer_housekeeping(
    info: &HostInfo,
    request: &fourway::Request<'_>,
    out: &mut [u8],
) -> Option<Result<usize, EncodeError>> {
    let reply = |params: &[u8], out: &mut [u8]| {
        fourway::encode_reply(request.command, request.address, params, Ack::Ok, out)
    };
    match request.command {
        Command::ProtocolGetVersion => Some(reply(&[info.protocol_version], out)),
        Command::InterfaceGetName => Some(reply(info.interface_name, out)),
        Command::InterfaceGetVersion => Some(reply(&info.interface_version, out)),
        _ => None,
    }
}

/// Writes the success reply to `request`: its command and address, `params` (1 to 256 bytes) and
/// acknowledge OK. Returns the length written.
///
/// # Errors
///
/// [`EncodeError::InvalidLength`] unless `params` holds 1 to 256 bytes,
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than `params.len() + 8`.
pub fn encode_success(
    request: &fourway::Request<'_>,
    params: &[u8],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    fourway::encode_reply(request.command, request.address, params, Ack::Ok, out)
}

/// Writes the failure reply to `request`: its command and address, one parameter byte (the first
/// parameter of the request, as ArduPilot answers: `AP_BLHeli.cpp:1057-1060, 1080-1083`, framed by `blheli_send_reply`, `610-623`) and the
/// acknowledge `ack`, which should not be [`Ack::Ok`]. Returns the length written (9).
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] if `out` is shorter than 9 bytes.
pub fn encode_failure(
    request: &fourway::Request<'_>,
    ack: Ack,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let first = request.params.first().copied().unwrap_or(0);
    fourway::encode_reply(request.command, request.address, &[first], ack, out)
}
