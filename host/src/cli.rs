//! The command line.

use crate::tool::Dump;
use std::path::PathBuf;

/// The USB ids of the bench boards.
pub const DEFAULT_VID: u16 = 0xC0DE;
/// The USB product id of the bench boards.
pub const DEFAULT_PID: u16 = 0xCAFE;

/// Which port to use.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PortChoice {
    /// The first (only) port with these USB ids.
    UsbIds(u16, u16),
    /// This path.
    Path(String),
}

/// A parsed command line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Args {
    /// The port.
    pub port: PortChoice,
    /// `--escs`.
    pub escs: Option<u8>,
    /// `--address`.
    pub address: u16,
    /// `--length`.
    pub length: u16,
    /// `--compare` files, in order.
    pub compare: Vec<PathBuf>,
    /// `--out`.
    pub out: Option<PathBuf>,
    /// `--no-exit`.
    pub no_exit: bool,
    /// `--dump`.
    pub dump: Option<Dump>,
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
esc-passthrough-host [--port PATH | --vid-pid VVVV:PPPP] [--escs N] [--address 0x7C00]
                     [--length 184] [--compare FILE...] [--out DIR] [--no-exit]
                     [--dump START:LENGTH]

Reads the settings block of each motor controller behind a flight controller's AM32
passthrough, as the configurator does, and never writes. See README.md.

  --port PATH        serial port to open (default: the port with USB ids C0DE:CAFE)
  --vid-pid V:P      USB ids, hexadecimal, to look for instead
  --escs N           controllers to visit, 1 to 8 (default: the count in the passthrough reply)
  --address A        wire address of the block, decimal or 0x hexadecimal (default 0x7C00)
  --length L         bytes to read in one request, 1 to 256 (default 184)
  --compare FILE...  one file per controller, in order; report equal or the differing offsets
  --out DIR          save each block as DIR/esc1.bin, esc2.bin, ...
  --no-exit          do not send interface-exit at the end
  --dump S:L         after the settings block read L bytes of flash from wire address S in
                     chunks of at most 256 bytes and save escN-flash-<S>-<L>.bin under --out
                     (required); S and L decimal or 0x hexadecimal; S + L at most 0x10000

Exit status: 0 all answered and all comparisons equal; 1 the board or a controller did not
answer; 2 a comparison found a difference; 3 passthrough refused; 64 bad command line.";

pub(crate) fn number(text: &str) -> Result<u32, String> {
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|_| format!("'{text}' is not a number"))
}

pub(crate) fn hex16(text: &str) -> Result<u16, String> {
    u16::from_str_radix(text, 16).map_err(|_| format!("'{text}' is not a 4-digit hexadecimal id"))
}

/// Parses the arguments after the program name.
///
/// # Errors
///
/// A message naming the fault.
pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Command, String> {
    let mut args = args.into_iter().peekable();
    let mut port: Option<PortChoice> = None;
    let mut parsed = Args {
        port: PortChoice::UsbIds(DEFAULT_VID, DEFAULT_PID),
        escs: None,
        address: 0x7C00,
        length: 184,
        compare: Vec::new(),
        out: None,
        no_exit: false,
        dump: None,
    };
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or_else(|| format!("{name} needs a value"));
        match arg.as_str() {
            "--help" | "-h" => return Ok(Command::Help),
            "--port" => {
                if port.is_some() {
                    return Err("--port and --vid-pid are alternatives, give one".into());
                }
                port = Some(PortChoice::Path(value("--port")?));
            }
            "--vid-pid" => {
                if port.is_some() {
                    return Err("--port and --vid-pid are alternatives, give one".into());
                }
                let text = value("--vid-pid")?;
                let (vid, pid) = text
                    .split_once(':')
                    .ok_or_else(|| format!("'{text}' is not VVVV:PPPP"))?;
                port = Some(PortChoice::UsbIds(hex16(vid)?, hex16(pid)?));
            }
            "--escs" => {
                let n = number(&value("--escs")?)?;
                parsed.escs = Some(
                    u8::try_from(n)
                        .ok()
                        .filter(|n| (1..=8).contains(n))
                        .ok_or("--escs must be 1 to 8")?,
                );
            }
            "--address" => {
                parsed.address = u16::try_from(number(&value("--address")?)?)
                    .map_err(|_| "--address must fit 16 bits".to_string())?;
            }
            "--length" => {
                parsed.length = u16::try_from(number(&value("--length")?)?)
                    .ok()
                    .filter(|n| (1..=256).contains(n))
                    .ok_or("--length must be 1 to 256")?;
            }
            "--compare" => {
                let before = parsed.compare.len();
                while let Some(next) = args.next_if(|next| !next.starts_with("--")) {
                    parsed.compare.push(PathBuf::from(next));
                }
                if parsed.compare.len() == before {
                    return Err("--compare needs at least one file".into());
                }
            }
            "--out" => parsed.out = Some(PathBuf::from(value("--out")?)),
            "--no-exit" => parsed.no_exit = true,
            "--dump" => {
                let text = value("--dump")?;
                let (start, length) = text
                    .split_once(':')
                    .ok_or_else(|| format!("'{text}' is not START:LENGTH"))?;
                let start = u16::try_from(number(start)?)
                    .map_err(|_| "--dump START must fit 16 bits".to_string())?;
                let length = number(length)?;
                if length == 0 || u32::from(start) + length > 0x1_0000 {
                    return Err("--dump needs LENGTH >= 1 and START + LENGTH <= 0x10000".into());
                }
                parsed.dump = Some(Dump { start, length });
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
    }
    if parsed.dump.is_some() && parsed.out.is_none() {
        return Err("--dump needs --out DIR".into());
    }
    if let Some(port) = port {
        parsed.port = port;
    }
    Ok(Command::Run(parsed))
}
