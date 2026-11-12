//! The tool's conversation: the configurator's connect sequence, then for each controller the
//! init-flash and one read of the settings block, then the way out. Everything it sends is a
//! [`msp::Request`] or a [`fourway::Request`]; there is no other way to put a byte on the link.
//!
//! The sequence and its numbers are the configurator's
//! (`_plan/ref/am32-configurator`, "SerialDevice.vue" and "four_way.ts" below):
//!
//! - MSP_API_VERSION, MSP_FC_VARIANT, MSP_BATTERY_STATE, MSP_MOTOR_CONFIG, MSP_SET_PASSTHROUGH in
//!   that order (`SerialDevice.vue:618-655`), a wait of 2000 ms (`:657`), then four-way only.
//! - A first request that gets no reply: the configurator sends interface-exit, then asks again
//!   up to 30 times, 1000 ms before the first and 500 ms between (`SerialDevice.vue:618-629`,
//!   `src/communication/msp.ts:102-112`).
//! - Four-way requests are tried up to 10 times, 250 ms apart; a try fails on silence, an invalid
//!   frame, a reply to another command or address, or an acknowledge other than OK
//!   (`four_way.ts:373-428`). Init-flash waits 2000 ms per try (`:205`); a read waits
//!   `max(1500, max(2000, 120 ms per byte))` (`:40`, `:299`, `:327`).
//! - Interface-exit is sent once and its reply is not awaited (`four_way.ts:386-389`); this tool
//!   still listens briefly, to report what the board answered.
//! - An MSP request waits 1000 ms (`src/communication/serial.ts:59`).

use crate::fourway;
use crate::link::Link;
use crate::msp;
use std::io::Write;
use std::thread;
use std::time::Duration;

/// The waits and counts of the sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// Wait for the reply to an MSP request.
    pub msp_timeout: Duration,
    /// Wait after the passthrough reply, before the first four-way request.
    pub after_passthrough: Duration,
    /// Tries of a four-way request.
    pub tries: u32,
    /// Pause between tries.
    pub retry_delay: Duration,
    /// Wait for an init-flash reply, per try.
    pub init_timeout: Duration,
    /// Base wait for a read reply, per try; at least the per-byte wait applies, see
    /// [`Timing::read_timeout`].
    pub read_timeout: Duration,
    /// How long to listen for the reply to interface-exit.
    pub exit_wait: Duration,
    /// Tries of MSP_API_VERSION after a failed first one and an interface-exit.
    pub recover_tries: u32,
    /// Pause before the first of those.
    pub recover_first_delay: Duration,
    /// Pause before each later one.
    pub recover_delay: Duration,
}

impl Timing {
    /// The configurator's values.
    pub const fn configurator() -> Self {
        Self {
            msp_timeout: Duration::from_millis(1000),
            after_passthrough: Duration::from_millis(2000),
            tries: 10,
            retry_delay: Duration::from_millis(250),
            init_timeout: Duration::from_millis(2000),
            read_timeout: Duration::from_millis(1500),
            exit_wait: Duration::from_millis(1000),
            recover_tries: 30,
            recover_first_delay: Duration::from_millis(1000),
            recover_delay: Duration::from_millis(500),
        }
    }

    /// The wait for a read of `bytes` bytes: `max(base, max(2000 ms, 120 ms per byte))`
    /// (`four_way.ts:40`, `:327`).
    pub fn read_timeout(&self, bytes: usize) -> Duration {
        let per_byte =
            Duration::from_millis(120).saturating_mul(u32::try_from(bytes).unwrap_or(u32::MAX));
        self.read_timeout
            .max(Duration::from_millis(2000).max(per_byte))
    }
}

impl Default for Timing {
    fn default() -> Self {
        Self::configurator()
    }
}

/// What to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Options {
    /// Controllers to visit; `None` takes the count from the passthrough reply.
    pub escs: Option<u8>,
    /// Wire address of the block to read.
    pub address: u16,
    /// Bytes to read (1 to 256).
    pub length: u16,
    /// Expected contents, controller 0 first.
    pub compare: Vec<Vec<u8>>,
    /// Leave the board in four-way mode at the end.
    pub no_exit: bool,
    /// Waits.
    pub timing: Timing,
}

impl Options {
    /// The defaults of the command line: 184 bytes at 0x7C00, count from the board, exit at the
    /// end, the configurator's timing.
    pub fn new() -> Self {
        Self {
            escs: None,
            address: 0x7C00,
            length: 184,
            compare: Vec::new(),
            no_exit: false,
            timing: Timing::configurator(),
        }
    }
}

impl Default for Options {
    fn default() -> Self {
        Self::new()
    }
}

/// How the passthrough request ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Passthrough {
    /// Not asked (the sequence failed before it).
    NotAsked,
    /// The board answered with the MSP error reply.
    Refused,
    /// The board agreed and reported this many controllers.
    Granted(u8),
}

/// What a comparison with a file found.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Comparison {
    /// No file was given for this controller, or its block was not read.
    #[default]
    NotDone,
    /// Same bytes, same length.
    Equal,
    /// Different.
    Differs {
        /// Offsets (in the block) of bytes that differ, within the shorter of the two.
        offsets: Vec<usize>,
        /// Length of the block read.
        block_len: usize,
        /// Length of the file.
        file_len: usize,
    },
}

/// What was found about one controller.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct EscReport {
    /// Controller index, 0-based.
    pub index: usize,
    /// Signature from the init reply, high byte first as the configurator reads it (0x1F06).
    pub signature: Option<u16>,
    /// Pin code from the init reply.
    pub pin_code: Option<u8>,
    /// Interface mode from the init reply.
    pub interface_mode: Option<u8>,
    /// The block read.
    pub block: Option<Vec<u8>>,
    /// Why the controller did not answer, if it did not.
    pub error: Option<String>,
    /// The comparison with the file given for it.
    pub comparison: Comparison,
}

/// Everything a run found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// Protocol, major and minor from MSP_API_VERSION.
    pub api_version: Option<[u8; 3]>,
    /// The four ASCII bytes of MSP_FC_VARIANT, as text.
    pub variant: Option<String>,
    /// Battery voltage in tenths of a volt (byte 3 of MSP_BATTERY_STATE).
    pub battery_decivolts: Option<u8>,
    /// Motor count (byte 6 of MSP_MOTOR_CONFIG).
    pub motor_count: Option<u8>,
    /// The outcome of the passthrough request.
    pub passthrough: Passthrough,
    /// One entry per controller visited.
    pub escs: Vec<EscReport>,
    /// A failure that stopped the sequence (or a count of zero), when no controller is to blame.
    pub failure: Option<String>,
    /// Interface-exit was sent.
    pub exit_sent: bool,
    /// The board answered the interface-exit with a valid reply.
    pub exit_answered: bool,
    /// Files given with `--compare` for which there was no controller.
    pub unused_compare_files: usize,
}

impl Report {
    fn new() -> Self {
        Self {
            api_version: None,
            variant: None,
            battery_decivolts: None,
            motor_count: None,
            passthrough: Passthrough::NotAsked,
            escs: Vec::new(),
            failure: None,
            exit_sent: false,
            exit_answered: false,
            unused_compare_files: 0,
        }
    }

    /// The exit status: 3 when the passthrough was refused; else 1 when the board or a controller
    /// did not answer; else 2 when a comparison found a difference (or a file had no controller);
    /// else 0.
    pub fn status(&self) -> i32 {
        if self.passthrough == Passthrough::Refused {
            3
        } else if self.failure.is_some() || self.escs.iter().any(|esc| esc.error.is_some()) {
            1
        } else if self.unused_compare_files > 0
            || self
                .escs
                .iter()
                .any(|esc| matches!(esc.comparison, Comparison::Differs { .. }))
        {
            2
        } else {
            0
        }
    }
}

macro_rules! say {
    ($out:expr, $($arg:tt)*) => {{
        let _ = writeln!($out, $($arg)*);
    }};
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

fn pause(d: Duration) {
    if !d.is_zero() {
        thread::sleep(d);
    }
}

/// Runs the sequence on `link`, printing what happens to `out`.
pub fn run<L: Link + ?Sized>(link: &mut L, options: &Options, out: &mut dyn Write) -> Report {
    let mut report = Report::new();
    let timing = options.timing;
    let _ = link.discard_input();

    // SerialDevice.vue:618-653: the four MSP requests.
    let Some(api) = first_request(link, &timing, out) else {
        report.failure = Some("the board did not answer MSP_API_VERSION".into());
        return report;
    };
    if let [protocol, major, minor, ..] = api[..] {
        report.api_version = Some([protocol, major, minor]);
        say!(
            out,
            "board: protocol {protocol}, API version {major}.{minor}"
        );
    } else {
        say!(
            out,
            "board: MSP_API_VERSION payload has only {} bytes",
            api.len()
        );
    }
    let Some(variant) = msp_data(link, msp::Request::FcVariant, &timing, out, &mut report) else {
        return report;
    };
    let text = String::from_utf8_lossy(variant.get(..4).unwrap_or(&variant)).into_owned();
    say!(out, "board: flight controller variant {text:?}");
    report.variant = Some(text);
    let Some(battery) = msp_data(link, msp::Request::BatteryState, &timing, out, &mut report)
    else {
        return report;
    };
    match battery.get(3) {
        Some(&tenths) if battery.len() >= 8 => {
            report.battery_decivolts = Some(tenths);
            say!(out, "board: battery {:.1} V", f64::from(tenths) / 10.0);
        }
        _ => {
            report.failure = Some(format!(
                "MSP_BATTERY_STATE payload has {} bytes, the configurator needs at least 8",
                battery.len()
            ));
            say!(
                out,
                "board: {}",
                report.failure.as_deref().unwrap_or_default()
            );
            return report;
        }
    }
    let Some(motors) = msp_data(link, msp::Request::MotorConfig, &timing, out, &mut report) else {
        return report;
    };
    let Some(&motor_count) = motors.get(6) else {
        report.failure = Some(format!(
            "MSP_MOTOR_CONFIG payload has {} bytes, the motor count is at byte 6",
            motors.len()
        ));
        say!(
            out,
            "board: {}",
            report.failure.as_deref().unwrap_or_default()
        );
        return report;
    };
    report.motor_count = Some(motor_count);
    say!(out, "board: motor count {motor_count}");

    // SerialDevice.vue:655-661.
    let count = match request_msp(link, msp::Request::SetPassthrough, &timing, out) {
        Ok(msp::Reply::Refused) => {
            report.passthrough = Passthrough::Refused;
            say!(
                out,
                "passthrough: REFUSED by the board (MSP error reply); nothing was changed"
            );
            return report;
        }
        Ok(msp::Reply::Data(data)) => match data.first() {
            Some(&count) => count,
            None => {
                report.failure =
                    Some("MSP_SET_PASSTHROUGH reply carries no controller count".into());
                say!(
                    out,
                    "passthrough: {}",
                    report.failure.as_deref().unwrap_or_default()
                );
                best_effort_exit(link, options, out, &mut report);
                return report;
            }
        },
        Err(error) => {
            report.failure = Some(format!("MSP_SET_PASSTHROUGH: {error}"));
            say!(out, "passthrough: {error}");
            best_effort_exit(link, options, out, &mut report);
            return report;
        }
    };
    report.passthrough = Passthrough::Granted(count);
    say!(
        out,
        "passthrough: granted, the board reports {count} controllers"
    );
    let visit = options.escs.unwrap_or(count);
    if let Some(forced) = options.escs
        && forced != count
    {
        say!(out, "note: --escs {forced} overrides the board's {count}");
    }
    say!(
        out,
        "waiting {:.0} ms before the first four-way request (SerialDevice.vue:657)",
        ms(timing.after_passthrough)
    );
    pause(timing.after_passthrough);
    if visit == 0 {
        report.failure = Some("the board reports 0 controllers".into());
        say!(
            out,
            "passthrough: {}",
            report.failure.as_deref().unwrap_or_default()
        );
    }

    for index in 0..usize::from(visit) {
        let esc = visit_esc(link, options, index, out);
        report.escs.push(esc);
    }
    report.unused_compare_files = options.compare.len().saturating_sub(report.escs.len());
    if report.unused_compare_files > 0 {
        say!(
            out,
            "compare: {} file(s) given for controllers that were not visited",
            report.unused_compare_files
        );
    }

    if options.no_exit {
        say!(
            out,
            "exit: not sent (--no-exit); the board stays in four-way mode"
        );
    } else {
        send_exit(link, options, out, &mut report);
    }
    report
}

/// MSP_API_VERSION with the configurator's recovery (`SerialDevice.vue:618-629`).
fn first_request<L: Link + ?Sized>(
    link: &mut L,
    timing: &Timing,
    out: &mut dyn Write,
) -> Option<Vec<u8>> {
    match request_msp(link, msp::Request::ApiVersion, timing, out) {
        Ok(msp::Reply::Data(data)) => return Some(data),
        Ok(msp::Reply::Refused) => say!(
            out,
            "msp: MSP_API_VERSION was answered with the error reply"
        ),
        Err(error) => say!(out, "msp: MSP_API_VERSION: {error}"),
    }
    say!(
        out,
        "msp: trying to exit four-way and try again (SerialDevice.vue:618-629)"
    );
    let exit = fourway::Request::InterfaceExit;
    match fourway::transact(link, &exit, timing.exit_wait) {
        Ok((_, took)) => say!(out, "  cmd_InterfaceExit answered in {:.1} ms", ms(took)),
        Err(error) => say!(out, "  cmd_InterfaceExit: {error}"),
    }
    for attempt in 0..timing.recover_tries {
        pause(if attempt == 0 {
            timing.recover_first_delay
        } else {
            timing.recover_delay
        });
        match request_msp(link, msp::Request::ApiVersion, timing, out) {
            Ok(msp::Reply::Data(data)) => return Some(data),
            Ok(msp::Reply::Refused) => {}
            Err(error) => say!(
                out,
                "  retry {} of {}: {error}",
                attempt + 1,
                timing.recover_tries
            ),
        }
    }
    None
}

/// One MSP request, printed with its timing.
fn request_msp<L: Link + ?Sized>(
    link: &mut L,
    request: msp::Request,
    timing: &Timing,
    out: &mut dyn Write,
) -> Result<msp::Reply, msp::Error> {
    let (reply, took) = msp::transact(link, request, timing.msp_timeout)?;
    say!(out, "[{}] reply in {:.1} ms", request.name(), ms(took));
    Ok(reply)
}

/// An MSP request whose reply must carry data; on failure records it in `report`.
fn msp_data<L: Link + ?Sized>(
    link: &mut L,
    request: msp::Request,
    timing: &Timing,
    out: &mut dyn Write,
    report: &mut Report,
) -> Option<Vec<u8>> {
    match request_msp(link, request, timing, out) {
        Ok(msp::Reply::Data(data)) => Some(data),
        Ok(msp::Reply::Refused) => {
            report.failure = Some(format!(
                "{} was answered with the MSP error reply",
                request.name()
            ));
            say!(
                out,
                "msp: {}",
                report.failure.as_deref().unwrap_or_default()
            );
            None
        }
        Err(error) => {
            report.failure = Some(format!("{}: {error}", request.name()));
            say!(
                out,
                "msp: {}",
                report.failure.as_deref().unwrap_or_default()
            );
            None
        }
    }
}

/// Init-flash and the settings read for one controller.
fn visit_esc<L: Link + ?Sized>(
    link: &mut L,
    options: &Options,
    index: usize,
    out: &mut dyn Write,
) -> EscReport {
    let timing = &options.timing;
    let mut esc = EscReport {
        index,
        ..EscReport::default()
    };
    say!(out, "controller {}:", index + 1);
    let Ok(channel) = u8::try_from(index) else {
        esc.error = Some("controller index does not fit a byte".into());
        return esc;
    };
    let init = fourway::Request::InitFlash { esc: channel };
    let init_reply = with_retries(link, &init, timing.init_timeout, timing, out, &|reply| {
        if reply.params.len() == 4 {
            Ok(())
        } else {
            Err(format!(
                "init reply has {} parameter bytes, expected 4",
                reply.params.len()
            ))
        }
    });
    let reply = match init_reply {
        Ok(reply) => reply,
        Err(error) => {
            say!(out, "  init-flash failed: {error}");
            esc.error = Some(format!("init-flash: {error}"));
            return esc;
        }
    };
    if let [low, high, pin, mode] = reply.params[..] {
        let signature = u16::from_be_bytes([high, low]);
        esc.signature = Some(signature);
        esc.pin_code = Some(pin);
        esc.interface_mode = Some(mode);
        say!(
            out,
            "  signature 0x{signature:04X}, pin code 0x{pin:02X}, interface mode {mode} (reply bytes {low:02X} {high:02X} {pin:02X} {mode:02X})"
        );
    }

    let read = fourway::Request::DeviceRead {
        address: options.address,
        length: options.length,
    };
    let wanted = usize::from(options.length);
    let timeout = timing.read_timeout(wanted);
    let read_reply = with_retries(link, &read, timeout, timing, out, &|reply| {
        if reply.params.len() == wanted {
            Ok(())
        } else {
            Err(format!(
                "read reply has {} bytes, asked for {wanted}",
                reply.params.len()
            ))
        }
    });
    match read_reply {
        Ok(reply) => {
            say!(
                out,
                "  settings block, {} bytes at 0x{:04X}:",
                reply.params.len(),
                options.address
            );
            hex_dump(out, &reply.params);
            if let Some(expected) = options.compare.get(index) {
                esc.comparison = compare(&reply.params, expected);
                report_comparison(out, &esc.comparison);
            }
            esc.block = Some(reply.params);
        }
        Err(error) => {
            say!(out, "  read failed: {error}");
            esc.error = Some(format!("read: {error}"));
        }
    }
    esc
}

/// The configurator's retry rule (`four_way.ts:373-428`): `timing.tries` tries, `timing.retry_delay`
/// apart; every kind of failed try is retried, except a failure of the link itself.
fn with_retries<L: Link + ?Sized>(
    link: &mut L,
    request: &fourway::Request,
    timeout: Duration,
    timing: &Timing,
    out: &mut dyn Write,
    valid: &dyn Fn(&fourway::Reply) -> Result<(), String>,
) -> Result<fourway::Reply, String> {
    for attempt in 1..=timing.tries {
        let failure = match fourway::transact(link, request, timeout) {
            Ok((reply, took)) => match valid(&reply) {
                Ok(()) => {
                    say!(
                        out,
                        "  [{}] reply in {:.1} ms (try {attempt} of {})",
                        request.name(),
                        ms(took),
                        timing.tries
                    );
                    return Ok(reply);
                }
                Err(reason) => reason,
            },
            Err(fourway::Error::Exchange(
                error @ (crate::link::ExchangeError::Io(_) | crate::link::ExchangeError::Closed),
            )) => return Err(error.to_string()),
            Err(error) => error.to_string(),
        };
        say!(
            out,
            "  [{}] try {attempt} of {} failed: {failure}",
            request.name(),
            timing.tries
        );
        if attempt < timing.tries {
            pause(timing.retry_delay);
        } else {
            return Err(format!("max retries reached (last: {failure})"));
        }
    }
    Err("max retries reached".into())
}

fn send_exit<L: Link + ?Sized>(
    link: &mut L,
    options: &Options,
    out: &mut dyn Write,
    report: &mut Report,
) {
    report.exit_sent = true;
    match fourway::transact(
        link,
        &fourway::Request::InterfaceExit,
        options.timing.exit_wait,
    ) {
        Ok((_, took)) => {
            report.exit_answered = true;
            say!(
                out,
                "exit: cmd_InterfaceExit answered in {:.1} ms",
                ms(took)
            );
        }
        Err(error) => say!(
            out,
            "exit: cmd_InterfaceExit sent; no valid reply ({error}); the configurator does not wait for one"
        ),
    }
}

/// After a failure that leaves the board possibly in four-way mode.
fn best_effort_exit<L: Link + ?Sized>(
    link: &mut L,
    options: &Options,
    out: &mut dyn Write,
    report: &mut Report,
) {
    if !options.no_exit {
        send_exit(link, options, out, report);
    }
}

/// Compares `block` with `expected`.
pub fn compare(block: &[u8], expected: &[u8]) -> Comparison {
    let offsets: Vec<usize> = block
        .iter()
        .zip(expected)
        .enumerate()
        .filter_map(|(at, (a, b))| (a != b).then_some(at))
        .collect();
    if offsets.is_empty() && block.len() == expected.len() {
        Comparison::Equal
    } else {
        Comparison::Differs {
            offsets,
            block_len: block.len(),
            file_len: expected.len(),
        }
    }
}

fn report_comparison(out: &mut dyn Write, comparison: &Comparison) {
    match comparison {
        Comparison::NotDone => {}
        Comparison::Equal => say!(out, "  compare: equal"),
        Comparison::Differs {
            offsets,
            block_len,
            file_len,
        } => {
            say!(
                out,
                "  compare: DIFFERENT ({} byte(s) differ)",
                offsets.len()
            );
            if block_len != file_len {
                say!(
                    out,
                    "  compare: lengths differ, block {block_len} bytes, file {file_len} bytes"
                );
            }
            let shown: Vec<String> = offsets
                .iter()
                .take(32)
                .map(|at| format!("0x{at:02X}"))
                .collect();
            if !shown.is_empty() {
                let more = offsets.len().saturating_sub(32);
                let tail = if more > 0 {
                    format!(" and {more} more")
                } else {
                    String::new()
                };
                say!(
                    out,
                    "  compare: differing offsets {}{tail}",
                    shown.join(" ")
                );
            }
        }
    }
}

fn hex_dump(out: &mut dyn Write, bytes: &[u8]) {
    for (row, chunk) in bytes.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
        say!(out, "    {:04x}  {}", row * 16, hex.join(" "));
    }
}

/// Writes each block of `report` to `dir/escN.bin` (N from 1). Returns the paths written.
///
/// # Errors
///
/// The first I/O error.
pub fn save_blocks(
    dir: &std::path::Path,
    report: &Report,
) -> std::io::Result<Vec<std::path::PathBuf>> {
    let mut written = Vec::new();
    for esc in &report.escs {
        if let Some(block) = &esc.block {
            let path = dir.join(format!("esc{}.bin", esc.index + 1));
            std::fs::write(&path, block)?;
            written.push(path);
        }
    }
    Ok(written)
}
