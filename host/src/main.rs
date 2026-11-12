//! Command line of `esc-passthrough-host`; see README.md.

use esc_passthrough_host::cli::{self, Command};
use esc_passthrough_host::port;
use esc_passthrough_host::tool::{self, Options, Timing};
use std::process::ExitCode;

const USAGE_ERROR: u8 = 64;

fn main() -> ExitCode {
    let args = match cli::parse(std::env::args().skip(1)) {
        Ok(Command::Help) => {
            println!("{}", cli::USAGE);
            return ExitCode::SUCCESS;
        }
        Ok(Command::Run(args)) => args,
        Err(message) => {
            eprintln!("error: {message}\n\n{}", cli::USAGE);
            return ExitCode::from(USAGE_ERROR);
        }
    };

    let mut compare = Vec::new();
    for path in &args.compare {
        match std::fs::read(path) {
            Ok(bytes) => compare.push(bytes),
            Err(error) => {
                eprintln!("error: --compare {}: {error}", path.display());
                return ExitCode::from(USAGE_ERROR);
            }
        }
    }
    if let Some(dir) = &args.out
        && let Err(error) = std::fs::create_dir_all(dir)
    {
        eprintln!("error: --out {}: {error}", dir.display());
        return ExitCode::from(USAGE_ERROR);
    }

    let (name, mut link) = match port::open(&args.port) {
        Ok(opened) => opened,
        Err(message) => {
            eprintln!("error: {message}");
            return ExitCode::from(1);
        }
    };
    println!("port: {name}");

    let options = Options {
        escs: args.escs,
        address: args.address,
        length: args.length,
        compare,
        no_exit: args.no_exit,
        timing: Timing::configurator(),
    };
    let report = tool::run(&mut link, &options, &mut std::io::stdout());
    let mut status = report.status();

    if let Some(dir) = &args.out {
        match tool::save_blocks(dir, &report) {
            Ok(paths) => {
                for path in paths {
                    println!("saved {}", path.display());
                }
            }
            Err(error) => {
                eprintln!("error: saving into {}: {error}", dir.display());
                if status == 0 {
                    status = 1;
                }
            }
        }
    }

    let verdict = match status {
        0 => "RESULT: every controller answered and every comparison was equal",
        1 => "RESULT: the board or a controller did not answer",
        2 => "RESULT: a comparison found a difference",
        _ => "RESULT: passthrough refused",
    };
    println!("{verdict} (exit status {status})");
    ExitCode::from(u8::try_from(status).unwrap_or(1))
}
