//! THE ONLY WRITE-CAPABLE CODE IN THE PACKAGE. Compiled only with the cargo feature `write`, for the
//! binary `esc-passthrough-restore`; a default build contains none of this file.
//!
//! It writes one controller's block back, the way the configurator saves settings
//! (`_plan/ref/am32-configurator/src/communication/four_way.ts`): `writeSettings` (`:543-565`) asks
//! init-flash, then ONE `write` of the whole block (`:493-496`: a single `cmd_DeviceWrite` at the
//! block's address, parameters = the block), then reads it back. Its requests are those of the
//! read-only tool plus device-write: [`Request`]. There is no page-erase, no reset and no other
//! four-way command here.

pub mod cli;

use crate::fourway::{self, CheckError, crc16_xmodem, frame_len, parse_reply};
use crate::link::{Link, exchange};
use crate::tool::{self, Options, Passthrough, Report, Timing, compare, hex_dump, ms, pause};
use std::io::Write;
use std::time::Duration;

macro_rules! say {
    ($out:expr, $($arg:tt)*) => {{
        let _ = writeln!($out, $($arg)*);
    }};
}

const CMD_DEVICE_WRITE: u8 = 0x3B; // four_way.ts:97
const REQUEST_START: u8 = 0x2F;

/// Writes of the same block: the first and at most two more.
pub const MAX_WRITES: u32 = 3;
/// The byte of the settings block where newer bootloaders stamp their version.
const VERSION_OFFSET: usize = 2;

/// The requests of the restore tool: the read-only tool's, and device-write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Request {
    /// A request of the read-only tool.
    Read(fourway::Request),
    /// `cmd_DeviceWrite` of `data` (2 to 256 bytes) at the wire `address` (`four_way.ts:493-496`).
    DeviceWrite {
        /// The wire address.
        address: u16,
        /// The block.
        data: Vec<u8>,
    },
}

impl Request {
    /// The frame, or `None` when a write is not 2 to 256 bytes.
    pub fn encode(&self) -> Option<Vec<u8>> {
        match self {
            Self::Read(request) => request.encode().ok(),
            Self::DeviceWrite { address, data } => {
                let count = match data.len() {
                    256 => 0,
                    2..=255 => u8::try_from(data.len()).ok()?,
                    _ => return None,
                };
                let [high, low] = address.to_be_bytes();
                let mut frame = vec![REQUEST_START, CMD_DEVICE_WRITE, high, low, count];
                frame.extend_from_slice(data);
                frame.extend(crc16_xmodem(&frame).to_be_bytes());
                Some(frame)
            }
        }
    }
}

/// What came of one write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WriteFailure {
    /// The controller answered with this error acknowledge: the write did not happen as asked.
    AckError(u8),
    /// No usable reply: whether the controller wrote is unknown.
    Unknown(String),
}

/// Sends one device-write and waits once for the reply. Not retried here.
///
/// # Errors
///
/// [`WriteFailure`].
pub fn send_write<L: Link + ?Sized>(
    link: &mut L,
    address: u16,
    data: &[u8],
    timeout: Duration,
) -> Result<Duration, WriteFailure> {
    let request = Request::DeviceWrite {
        address,
        data: data.to_vec(),
    };
    let frame = request
        .encode()
        .ok_or_else(|| WriteFailure::Unknown("block length is not 2 to 256".into()))?;
    let (bytes, took) = exchange(link, &frame, timeout, frame_len)
        .map_err(|error| WriteFailure::Unknown(error.to_string()))?;
    let reply = parse_reply(&bytes).map_err(|error| WriteFailure::Unknown(error.to_string()))?;
    match reply.check_echo(CMD_DEVICE_WRITE, address) {
        Ok(()) => Ok(took),
        Err(CheckError::Ack(ack)) => Err(WriteFailure::AckError(ack)),
        Err(error) => Err(WriteFailure::Unknown(error.to_string())),
    }
}

/// What to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RestoreOptions {
    /// The controller, 0-based.
    pub esc: u8,
    /// The block to write; its length is the length written.
    pub file: Vec<u8>,
    /// Wire address of the block.
    pub address: u16,
    /// Accept a file that differs from the block now in the controller.
    pub allow_change: bool,
    /// Waits.
    pub timing: Timing,
}

/// How the run ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// Stopped before the end (the reason is in `base.failure`, or the passthrough was refused).
    Incomplete,
    /// FILE differs from the block in the controller and `--allow-change` was not given.
    IdenticalOnlyRefused,
    /// Written, read back and equal to FILE (`offset2_only`: except at offset 2).
    Verified {
        /// The read-back differs from FILE only at offset 2.
        offset2_only: bool,
    },
    /// Written [`MAX_WRITES`] times and the read-back still differs (or no write was acknowledged).
    NotVerified,
}

/// Everything a restore run found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RestoreReport {
    /// The MSP opening, passthrough and exit, as in the read-only tool.
    pub base: Report,
    /// The controller, 0-based.
    pub esc: u8,
    /// The block in the controller before any write.
    pub before: Option<Vec<u8>>,
    /// Offsets where FILE differs from `before`.
    pub changes: Vec<usize>,
    /// Device-write requests sent.
    pub writes_sent: u32,
    /// Of those, answered with `ACK_OK`.
    pub writes_acked: u32,
    /// The last read-back.
    pub readback: Option<Vec<u8>>,
    /// How it ended.
    pub outcome: Outcome,
}

impl RestoreReport {
    fn new(esc: u8) -> Self {
        Self {
            base: Report::new(),
            esc,
            before: None,
            changes: Vec::new(),
            writes_sent: 0,
            writes_acked: 0,
            readback: None,
            outcome: Outcome::Incomplete,
        }
    }

    /// 0 verified; 1 no answer; 2 identical-only refusal; 3 passthrough refused; 4 not verified.
    pub fn status(&self) -> i32 {
        if self.base.passthrough == Passthrough::Refused {
            return 3;
        }
        match self.outcome {
            Outcome::Verified { .. } => 0,
            Outcome::IdenticalOnlyRefused => 2,
            Outcome::NotVerified => 4,
            Outcome::Incomplete => 1,
        }
    }
}

/// Runs the restore on `link`, printing what happens to `out`. Leaves passthrough at the end
/// whenever it was entered.
pub fn run<L: Link + ?Sized>(
    link: &mut L,
    options: &RestoreOptions,
    out: &mut dyn Write,
) -> RestoreReport {
    let mut report = RestoreReport::new(options.esc);
    let base_options = Options {
        timing: options.timing,
        ..Options::new()
    };
    let Some(count) = tool::enter_passthrough(link, &base_options, out, &mut report.base) else {
        final_line(&report, options, out);
        return report;
    };
    if options.esc >= count {
        let text = format!(
            "controller {} asked for, the board reports {count}",
            u16::from(options.esc) + 1
        );
        say!(out, "error: {text}");
        report.base.failure = Some(text);
    } else {
        restore(link, options, out, &mut report);
    }
    tool::send_exit(link, &base_options, out, &mut report.base);
    final_line(&report, options, out);
    report
}

fn restore<L: Link + ?Sized>(
    link: &mut L,
    options: &RestoreOptions,
    out: &mut dyn Write,
    report: &mut RestoreReport,
) {
    let timing = &options.timing;
    let number = u16::from(options.esc) + 1;
    let length = options.file.len();
    let wire = u16::try_from(length).unwrap_or(0);
    say!(out, "controller {number}:");
    match tool::init_esc(link, timing, options.esc, out) {
        Ok([low, high, pin, mode]) => say!(
            out,
            "  signature 0x{:04X}, pin code 0x{pin:02X}, interface mode {mode}",
            u16::from_be_bytes([high, low])
        ),
        Err(error) => {
            say!(out, "  init-flash failed: {error}");
            report.base.failure = Some(format!("init-flash: {error}"));
            return;
        }
    }
    let before = match tool::read_range(link, timing, options.address, wire, out) {
        Ok(block) => block,
        Err(error) => {
            say!(out, "  read failed: {error}");
            report.base.failure = Some(format!("read: {error}"));
            return;
        }
    };
    say!(
        out,
        "  block in the controller, {length} bytes at 0x{:04X}:",
        options.address
    );
    hex_dump(out, &before);
    report.changes = match compare(&before, &options.file) {
        tool::Comparison::Equal => Vec::new(),
        tool::Comparison::Differs { offsets, .. } => offsets,
        tool::Comparison::NotDone => Vec::new(),
    };
    if report.changes.is_empty() {
        say!(
            out,
            "  FILE is identical to the block in the controller: it will be written back unchanged"
        );
    } else {
        say!(
            out,
            "  FILE differs from the block in the controller at {} offset(s) (old -> new):",
            report.changes.len()
        );
        for &at in &report.changes {
            let old = before.get(at).copied().unwrap_or_default();
            let new = options.file.get(at).copied().unwrap_or_default();
            say!(out, "    0x{at:02X}: {old:02X} -> {new:02X}");
        }
        if !options.allow_change {
            say!(
                out,
                "  refused: without --allow-change only an identical block is written; nothing was written"
            );
            report.before = Some(before);
            report.outcome = Outcome::IdenticalOnlyRefused;
            return;
        }
        say!(
            out,
            "  --allow-change given: the differences will be written"
        );
    }
    report.before = Some(before);

    let timeout = timing.read_timeout(length);
    for attempt in 1..=MAX_WRITES {
        if attempt > 1 {
            pause(timing.retry_delay);
        }
        say!(
            out,
            "write {attempt} of {MAX_WRITES}: cmd_DeviceWrite, {length} bytes at 0x{:04X}",
            options.address
        );
        report.writes_sent += 1;
        match send_write(link, options.address, &options.file, timeout) {
            Ok(took) => {
                report.writes_acked += 1;
                say!(out, "  write acknowledged in {:.1} ms", ms(took));
            }
            Err(WriteFailure::AckError(ack)) => {
                say!(
                    out,
                    "  write answered with error {}",
                    fourway::ack_name(ack)
                );
                continue;
            }
            Err(WriteFailure::Unknown(reason)) => {
                say!(out, "  no usable reply to the write: {reason}");
                report.base.failure = Some(format!(
                    "write {attempt}: {reason}; whether the controller wrote is unknown"
                ));
                return;
            }
        }
        match tool::read_range(link, timing, options.address, wire, out) {
            Ok(back) => {
                let verdict = check_readback(&back, &options.file, out);
                report.readback = Some(back);
                if let Some(offset2_only) = verdict {
                    report.outcome = Outcome::Verified { offset2_only };
                    return;
                }
            }
            Err(error) => {
                say!(out, "  read-back failed: {error}");
                report.base.failure = Some(format!(
                    "write {attempt} acknowledged, read-back failed: {error}"
                ));
                return;
            }
        }
    }
    if report.readback.is_none()
        && let Ok(back) = tool::read_range(link, timing, options.address, wire, out)
    {
        report.readback = Some(back);
    }
    say!(
        out,
        "NOT VERIFIED after {MAX_WRITES} writes. Block that was to be written (FILE):"
    );
    hex_dump(out, &options.file);
    match &report.readback {
        Some(back) => {
            say!(out, "Block read back from the controller:");
            hex_dump(out, back);
        }
        None => say!(out, "No read-back could be obtained."),
    }
    report.outcome = Outcome::NotVerified;
}

/// Compares the read-back with FILE and prints the result per byte range. `Some(false)`:
/// identical; `Some(true)`: differs only at offset 2; `None`: differs.
fn check_readback(back: &[u8], file: &[u8], out: &mut dyn Write) -> Option<bool> {
    let offsets: Vec<usize> = match compare(back, file) {
        tool::Comparison::Equal => Vec::new(),
        tool::Comparison::Differs { offsets, .. } => offsets,
        tool::Comparison::NotDone => Vec::new(),
    };
    if offsets.is_empty() && back.len() == file.len() {
        say!(
            out,
            "  read-back: 0x00-0x{:02X} identical to FILE",
            back.len().saturating_sub(1)
        );
        return Some(false);
    }
    if offsets == [VERSION_OFFSET] && back.len() == file.len() {
        say!(
            out,
            "  read-back: identical to FILE except offset 0x02 ({:02X}, FILE has {:02X}): newer bootloaders stamp their version there; treated as verified",
            back.get(VERSION_OFFSET).copied().unwrap_or_default(),
            file.get(VERSION_OFFSET).copied().unwrap_or_default()
        );
        return Some(true);
    }
    say!(out, "  read-back DIFFERS from FILE:");
    if back.len() != file.len() {
        say!(
            out,
            "    lengths differ: read-back {} bytes, FILE {}",
            back.len(),
            file.len()
        );
    }
    let mut start = 0;
    while start < offsets.len() {
        let mut end = start;
        while offsets.get(end + 1) == offsets.get(end).map(|n| n + 1).as_ref() {
            end += 1;
        }
        let (first, last) = (offsets[start], offsets[end]);
        let hex = |bytes: &[u8]| {
            bytes
                .get(first..=last)
                .unwrap_or_default()
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        say!(
            out,
            "    0x{first:02X}-0x{last:02X}: FILE {}, read back {}",
            hex(file),
            hex(back)
        );
        start = end + 1;
    }
    None
}

/// The last line: what was written to which controller.
fn final_line(report: &RestoreReport, options: &RestoreOptions, out: &mut dyn Write) {
    let number = u16::from(options.esc) + 1;
    let length = options.file.len();
    let text = match (&report.outcome, report.writes_sent) {
        (Outcome::Verified { offset2_only }, _) => format!(
            "WRITTEN controller {number}: {length} bytes at wire address 0x{:04X}, {} write(s), read back and verified{}",
            options.address,
            report.writes_sent,
            if *offset2_only {
                " (offset 0x02 differs: bootloader version stamp)"
            } else {
                ""
            }
        ),
        (Outcome::NotVerified, sent) => format!(
            "WRITTEN BUT NOT VERIFIED controller {number}: {sent} write(s) of {length} bytes at wire address 0x{:04X} sent, {} acknowledged, the read-back still differs: the controller may hold a wrong block, act on it",
            options.address, report.writes_acked
        ),
        (_, 0) => format!(
            "NOTHING WRITTEN to any controller (no write was sent){}",
            if report.outcome == Outcome::IdenticalOnlyRefused {
                ": FILE differs and --allow-change was not given"
            } else {
                ""
            }
        ),
        (_, sent) => format!(
            "A WRITE WAS SENT to controller {number} ({sent} sent, {} acknowledged) and the result is unknown: read the block back with esc-passthrough-host",
            report.writes_acked
        ),
    };
    say!(out, "FINAL: {text}");
}
