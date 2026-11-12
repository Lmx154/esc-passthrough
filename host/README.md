# esc-passthrough-host

A command-line tool for the computer. It talks to a flight controller over a serial port the way the
AM32 configurator does, but **read-only**: it connects, enters passthrough, identifies each motor
controller, reads its settings block, leaves, and reports. It is the bench tool that proves a board's
passthrough end to end before the real configurator is tried.

**Status: so far it has been run only against a fake board** (the `esc-passthrough` crate's session and
its simulated bootloaders, joined to the tool by in-memory pipes; `tests/fake_board.rs`). It has not
been run against a real board, and no test opens a serial port.

## Run it

From this folder (a package of its own, with its own `[workspace]`):

```
cargo run --release -- [--port PATH | --vid-pid VVVV:PPPP] [--escs N] [--address 0x7C00] \
                       [--length 184] [--compare FILE...] [--out DIR] [--no-exit]
```

| Option | Meaning |
|---|---|
| `--port PATH` | The serial port to open. Default: the one port with USB ids `C0DE:CAFE`. |
| `--vid-pid V:P` | Look for these USB ids (hexadecimal) instead. Not together with `--port`. |
| `--escs N` | Controllers to visit, 1 to 8. Default: the count in the passthrough reply. |
| `--address A` | Wire address of the block (decimal or `0x`). Default `0x7C00`, the settings block of signature `0x1F06`. |
| `--length L` | Bytes read in one request, 1 to 256. Default 184. |
| `--compare FILE...` | One file per controller, in order. Reports "equal" or the differing offsets. |
| `--out DIR` | Saves each block as `DIR/esc1.bin`, `esc2.bin`, ... (numbered from 1). |
| `--no-exit` | Does not send interface-exit at the end (the board stays in four-way mode). |

The port is opened at 115200 baud (the configurator's default; nominal for a USB CDC port).

What it does, in the configurator's order (`_plan/ref/am32-configurator`, cited in `src/tool.rs`):

1. `MSP_API_VERSION`, `MSP_FC_VARIANT`, `MSP_BATTERY_STATE`, `MSP_MOTOR_CONFIG`; prints what the board
   answered (API version, variant, battery voltage, motor count). If the first gets no answer it sends
   interface-exit and asks again, as the configurator does.
2. `MSP_SET_PASSTHROUGH`; prints the controller count. A refusal (the MSP error reply) is reported and
   ends the run with status 3.
3. Waits 2000 ms, as the configurator does.
4. For each controller: init-flash with the configurator's retry rule (up to 10 tries, 250 ms apart, 2000 ms
   per try; a try fails on silence, a bad frame, a reply to another command or address, or an
   acknowledge other than OK), prints the four reply bytes (signature, pin code, interface mode), then
   reads the block in one request (same rule; wait `max(1500, 2000, 120 ms per byte)` per try, the
   configurator's failure bound) and prints it as hex, compares, saves.
5. Interface-exit (unless `--no-exit`). The configurator sends it and does not wait; the tool listens for
   1 s and reports what came.

Every reply is validated as the configurator does (start byte, length, CRC or checksum, command and
address echo, acknowledge). Each request prints a line with the time from request to reply in ms.
Every line is plain text on standard output.

## Exit status

| Status | Meaning |
|---|---|
| 0 | Every controller answered and every requested comparison was equal. |
| 1 | The port could not be opened, the board did not answer the MSP requests, or a controller did not answer (init or read). Other controllers are still visited. |
| 2 | Every controller answered, but a comparison found a difference (or a `--compare` file had no controller). |
| 3 | The board refused passthrough. Nothing was changed. |
| 64 | Bad command line, or a `--compare` file or the `--out` folder could not be used (nothing was sent). |

When several apply, 3 beats 1 beats 2.

## The read-only rule

This project's own tools never write to a controller. The tool builds its requests only from two enums:
`msp::Request` (the five MSP requests) and `fourway::Request` (init-flash, device-read, test-alive,
interface-exit). The command number of each four-way request is a constant in `src/fourway.rs`; there is
no code path that produces device-write, page-erase, device-reset, erase-all, write-eeprom or any other
four-way command. Check it with greps (run from this folder):

```
grep -rnEi "0x3[0-9a-f]\b" src/        # exactly four lines: 0x30 test-alive, 0x34 exit, 0x37 init-flash, 0x3A read
grep -rniE "devicewrite|device_write|pageerase|page_erase|eraseall|erase_all|devicereset|device_reset|writeeeprom|write_eeprom|c2ck|setmode|set_mode" src/   # no output
grep -rn "write_all" src/              # only src/link.rs: the trait method, its one call (in `exchange`) and the serial port's implementation
```

`tests/codecs.rs` also keeps the rule: a `match` over every `fourway::Request` variant (adding a variant
stops the test compiling) and a check that each encodes to one of the four allowed command numbers; and
`tests/fake_board.rs` checks that the simulated flash images are byte-identical after a full run and that
only the init, set-address and read bursts reached the simulated controllers.

## Tests

```
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt -- --check
```

- `tests/codecs.rs`: the tool's MSP v1 and four-way codecs against the vectors of the crate's
  `tests/msp.rs` and `tests/fourway.rs` (checked against the configurator's source), the CRC against a
  bit-serial reference, the retry rule, the refusal path, exit statuses, comparison and saving.
- `tests/cli.rs`: the command line.
- `tests/fake_board.rs`: the proof on the computer. Two in-memory pipes join the tool (this thread) to a
  second thread that runs `esc_passthrough::session::Passthrough::run` (the crate under test, a path
  dependency) with a lender handing out the crate's simulated bootloaders
  (`../tests/common/sim.rs`, included by `#[path]`, not copied). Four controllers with different settings
  blocks; the park and break delays are zero.

The tool's protocol code is its own (`src/msp.rs`, `src/fourway.rs`), written independently of the crate,
which has only the flight-controller side; the crate is a dev-dependency used by the fake board only.
