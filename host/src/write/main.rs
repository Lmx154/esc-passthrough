//! Command line of `esc-passthrough-restore`: WRITES one block to one controller. See README.md.

use esc_passthrough_host::port;
use esc_passthrough_host::tool::Timing;
use esc_passthrough_host::write::cli::{self, CONFIRMATION_MISSING, Command};
use esc_passthrough_host::write::{self, RestoreOptions};
use std::process::ExitCode;

const USAGE_ERROR: u8 = 64;

fn main() -> ExitCode {
    let args = match cli::parse(std::env::args().skip(1)) {
        Ok(Command::Help) => {
            println!("{}", cli::USAGE);
            return ExitCode::SUCCESS;
        }
        Ok(Command::Run(args)) => args,
        Err(message) if message == CONFIRMATION_MISSING => {
            eprintln!("error: {message}");
            return ExitCode::from(USAGE_ERROR);
        }
        Err(message) => {
            eprintln!("error: {message}\n\n{}", cli::USAGE);
            return ExitCode::from(USAGE_ERROR);
        }
    };
    let file = match std::fs::read(&args.file) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("error: --file {}: {error}", args.file.display());
            return ExitCode::from(USAGE_ERROR);
        }
    };
    if file.len() != usize::from(args.length) {
        eprintln!(
            "error: {} has {} bytes, --length is {}",
            args.file.display(),
            file.len(),
            args.length
        );
        return ExitCode::from(USAGE_ERROR);
    }

    println!(
        "esc-passthrough-restore: WILL WRITE {} bytes at 0x{:04X} to controller {} ({})",
        file.len(),
        args.address,
        args.esc,
        if args.allow_change {
            "--allow-change: a different block is accepted"
        } else {
            "identical block only"
        }
    );
    let (name, mut link) = match port::open(&args.port) {
        Ok(opened) => opened,
        Err(message) => {
            eprintln!("error: {message}");
            println!("FINAL: NOTHING WRITTEN to any controller (the port was not opened)");
            return ExitCode::from(1);
        }
    };
    println!("port: {name}");
    let options = RestoreOptions {
        esc: args.esc - 1,
        file,
        address: args.address,
        allow_change: args.allow_change,
        timing: Timing::configurator(),
    };
    let report = write::run(&mut link, &options, &mut std::io::stdout());
    let status = report.status();
    println!("exit status {status}");
    ExitCode::from(u8::try_from(status).unwrap_or(1))
}
