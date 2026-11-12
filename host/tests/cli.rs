//! The command line parser.

use esc_passthrough_host::cli::{self, Args, Command, PortChoice};
use std::path::PathBuf;

fn parse(args: &[&str]) -> Result<Command, String> {
    cli::parse(args.iter().map(|s| s.to_string()))
}

fn run(args: &[&str]) -> Args {
    match parse(args) {
        Ok(Command::Run(args)) => args,
        other => panic!("{other:?}"),
    }
}

#[test]
fn parse_defaults_are_the_bench_ids_184_bytes_at_0x7c00() {
    let args = run(&[]);
    assert_eq!(args.port, PortChoice::UsbIds(0xC0DE, 0xCAFE));
    assert_eq!((args.address, args.length, args.escs), (0x7C00, 184, None));
    assert!(args.compare.is_empty() && args.out.is_none() && !args.no_exit);
}

#[test]
fn parse_reads_every_option() {
    let args = run(&[
        "--vid-pid",
        "1209:abcd",
        "--escs",
        "2",
        "--address",
        "31744",
        "--length",
        "0x20",
        "--compare",
        "a.bin",
        "b.bin",
        "--out",
        "dir",
        "--no-exit",
    ]);
    assert_eq!(args.port, PortChoice::UsbIds(0x1209, 0xABCD));
    assert_eq!((args.escs, args.address, args.length), (Some(2), 31744, 32));
    assert_eq!(
        args.compare,
        [PathBuf::from("a.bin"), PathBuf::from("b.bin")]
    );
    assert_eq!(args.out, Some(PathBuf::from("dir")));
    assert!(args.no_exit);
    assert_eq!(
        run(&["--port", "/dev/ttyACM0"]).port,
        PortChoice::Path("/dev/ttyACM0".into())
    );
}

#[test]
fn parse_compare_stops_at_the_next_option() {
    let args = run(&["--compare", "a", "b", "c", "--no-exit"]);
    assert_eq!(args.compare.len(), 3);
    assert!(args.no_exit);
}

#[test]
fn parse_rejects_bad_input() {
    for bad in [
        &["--bogus"][..],
        &["--port"],
        &["--port", "x", "--vid-pid", "1:2"],
        &["--vid-pid", "zzzz:cafe"],
        &["--vid-pid", "c0de"],
        &["--escs", "0"],
        &["--escs", "9"],
        &["--length", "0"],
        &["--length", "257"],
        &["--address", "0x10000"],
        &["--compare"],
        &["--compare", "--no-exit"],
    ] {
        assert!(parse(bad).is_err(), "{bad:?}");
    }
    assert_eq!(parse(&["--help"]), Ok(Command::Help));
}

#[test]
fn parse_dump_takes_hex_or_decimal_and_needs_out() {
    use esc_passthrough_host::tool::Dump;
    let args = run(&["--dump", "0x0:0x8000", "--out", "d"]);
    assert_eq!(
        args.dump,
        Some(Dump {
            start: 0,
            length: 0x8000
        })
    );
    assert_eq!(
        run(&["--dump", "4096:700", "--out", "d"]).dump,
        Some(Dump {
            start: 4096,
            length: 700
        })
    );
    assert_eq!(run(&[]).dump, None);
    for bad in [
        &["--dump", "0x1000:0x100"][..],
        &["--dump", "0x1000", "--out", "d"],
        &["--dump", "0x1000:0", "--out", "d"],
        &["--dump", "0x10000:1", "--out", "d"],
        &["--dump", "0xFF00:0x200", "--out", "d"],
        &["--dump", "x:1", "--out", "d"],
    ] {
        assert!(parse(bad).is_err(), "{bad:?}");
    }
}
