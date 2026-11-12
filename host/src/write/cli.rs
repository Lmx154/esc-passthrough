//! The command line of `esc-passthrough-restore`.

use crate::cli::{PortChoice, hex16, number};
use std::path::PathBuf;

/// A parsed command line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Args {
    /// The port.
    pub port: PortChoice,
    /// The controller, 1-based as the read-only tool prints it.
    pub esc: u8,
    /// The file holding the block to write.
    pub file: PathBuf,
    /// Wire address of the block.
    pub address: u16,
    /// Length of the block; the file must have exactly this many bytes.
    pub length: u16,
    /// Accept a file that differs from the block now in the controller.
    pub allow_change: bool,
}

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `--help`.
    Help,
    /// A run.
    Run(Args),
}

/// The usage text.
pub const USAGE: &str = "\
esc-passthrough-restore --esc N --file FILE --yes-write-to-esc [--port PATH | --vid-pid VVVV:PPPP]
                        [--address 0x7C00] [--length 184] [--allow-change]

WRITES one block to one motor controller behind a flight controller's AM32 passthrough, the way
the configurator saves settings: one device-write of the whole block, then a read-back. Without
--allow-change FILE must be identical to the block now in the controller, so no setting changes;
the run only proves that a write works. See README.md.

  --esc N               the controller, 1 to 8, numbered as esc-passthrough-host prints them
  --file FILE           the block to write (exactly --length bytes)
  --yes-write-to-esc    required: without it the tool stops before opening the port
  --port PATH           serial port (default: the port with USB ids C0DE:CAFE)
  --vid-pid V:P         USB ids, hexadecimal, to look for instead
  --address A           wire address of the block (default 0x7C00)
  --length L            block length, 2 to 256 (default 184)
  --allow-change        accept a FILE that differs from the block now in the controller

Exit status: 0 written and verified; 1 the board or controller did not answer; 2 refused by the
identical-only rule, nothing written; 3 passthrough refused; 4 written but the read-back still
differs after three writes; 64 bad command line.";

/// The message of the refusal to run without the confirmation flag.
pub const CONFIRMATION_MISSING: &str =
    "refusing to write: --yes-write-to-esc is required (nothing was opened or sent)";

/// Parses the arguments after the program name.
///
/// # Errors
///
/// A message naming the fault; [`CONFIRMATION_MISSING`] when only the confirmation is missing.
pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Command, String> {
    let mut args = args.into_iter();
    let mut port: Option<PortChoice> = None;
    let mut esc: Option<u8> = None;
    let mut file: Option<PathBuf> = None;
    let mut yes = false;
    let mut address = 0x7C00u16;
    let mut length = 184u16;
    let mut allow_change = false;
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or_else(|| format!("{name} needs a value"));
        match arg.as_str() {
            "--help" | "-h" => return Ok(Command::Help),
            "--port" | "--vid-pid" => {
                if port.is_some() {
                    return Err("--port and --vid-pid are alternatives, give one".into());
                }
                let text = value(&arg)?;
                port = Some(if arg == "--port" {
                    PortChoice::Path(text)
                } else {
                    let (vid, pid) = text
                        .split_once(':')
                        .ok_or_else(|| format!("'{text}' is not VVVV:PPPP"))?;
                    PortChoice::UsbIds(hex16(vid)?, hex16(pid)?)
                });
            }
            "--esc" => {
                esc = Some(
                    u8::try_from(number(&value("--esc")?)?)
                        .ok()
                        .filter(|n| (1..=8).contains(n))
                        .ok_or("--esc must be 1 to 8")?,
                );
            }
            "--file" => file = Some(PathBuf::from(value("--file")?)),
            "--address" => {
                address = u16::try_from(number(&value("--address")?)?)
                    .map_err(|_| "--address must fit 16 bits".to_string())?;
            }
            "--length" => {
                length = u16::try_from(number(&value("--length")?)?)
                    .ok()
                    .filter(|n| (2..=256).contains(n))
                    .ok_or("--length must be 2 to 256")?;
            }
            "--allow-change" => allow_change = true,
            "--yes-write-to-esc" => yes = true,
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    let esc = esc.ok_or("--esc N is required")?;
    let file = file.ok_or("--file FILE is required")?;
    if !yes {
        return Err(CONFIRMATION_MISSING.into());
    }
    Ok(Command::Run(Args {
        port: port.unwrap_or(PortChoice::UsbIds(
            crate::cli::DEFAULT_VID,
            crate::cli::DEFAULT_PID,
        )),
        esc,
        file,
        address,
        length,
        allow_change,
    }))
}
