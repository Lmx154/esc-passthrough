# esc-passthrough-host

Two command-line tools for the computer that talk to a flight controller over a serial port the way the
AM32 configurator does:

- **`esc-passthrough-host`** is **read-only**. It connects, enters passthrough, identifies each motor
  controller, reads its settings block (and, on request, a flash range), leaves, and reports. It is the
  bench tool that proves a board's passthrough end to end before the real configurator is tried.
- **`esc-passthrough-restore`** is the **only code in the package that can write**. It writes one
  controller's own current settings block back to it (or, only if told to, a changed one) and reads it
  back. It exists to prove that a write through the passthrough works. It is built only with the cargo
  feature `write`.

**Status:** both tools have been run against the real board with four AM32 2.20 controllers (bootloader
version 1) behind it, on 2026-11-12. `esc-passthrough-host` read the four settings blocks and dumped
0x1000 to 0x7FFF of each controller, before and after a firmware flash done with the real configurator;
the dump after the flash was compared byte for byte with the release's hex file.
`esc-passthrough-restore` wrote each controller's own block back unchanged and verified it, on all four.
**It has not been used on hardware to write a changed block** (`--allow-change`). The cargo tests open
no serial port: they join the tools by in-memory pipes to a fake board (the `esc-passthrough` crate's
session and its simulated bootloaders; `tests/restore.rs`).

## Build

From this folder (a package of its own, with its own `[workspace]`). Both binaries land in the shared
build folder, `/home/luis/Documents/MARV-V2-drivers/.target/release/` (the folder `.cargo/config.toml` of
the drivers names; elsewhere it is `host/target/release/`):

```
cargo build --release                    # esc-passthrough-host only; contains no write code
cargo build --release --features write   # also esc-passthrough-restore
```

Build the read-only tool without the feature when its guarantee matters.

## `esc-passthrough-host` (read-only)

```
cargo run --release -- [--port PATH | --vid-pid VVVV:PPPP] [--escs N] [--address 0x7C00] \
                       [--length 184] [--compare FILE...] [--out DIR] [--no-exit] \
                       [--dump START:LENGTH]
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
| `--dump S:L` | After the settings block, reads `L` bytes of flash from wire address `S` (decimal or `0x`; `S + L` at most `0x10000`) from every controller in chunks of at most 256 bytes, each with the configurator's retry rule, and saves `DIR/escN-flash-0x<S>-0x<L>.bin` (four hex digits each). Needs `--out`. A progress line every 16 chunks and the total time are printed. A chunk the controller refuses is reported with its address, no file is written for that controller, and the exit status is 1. |

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
   configurator's failure bound) and prints it as hex, compares, saves, and dumps if asked.
5. Interface-exit (unless `--no-exit`). The configurator sends it and does not wait; the tool listens for
   1 s and reports what came.

Every reply is validated as the configurator does (start byte, length, CRC or checksum, command and
address echo, acknowledge). Each request prints a line with the time from request to reply in ms.

### What it can send

Only the requests of two enums: `msp::Request` (API version, FC variant, battery state, motor config,
set-passthrough) and `fourway::Request` (init-flash `0x37`, device-read `0x3A`, test-alive `0x30`,
interface-exit `0x34`). It cannot send device-write, page-erase, device-reset, erase-all, write-eeprom or
any other four-way command. `--dump` uses device-read only.

### Exit status

| Status | Meaning |
|---|---|
| 0 | Every controller answered and every requested comparison was equal. |
| 1 | The port could not be opened, the board did not answer the MSP requests, or a controller did not answer (init, read or a dump chunk). Other controllers are still visited. |
| 2 | Every controller answered, but a comparison found a difference (or a `--compare` file had no controller). |
| 3 | The board refused passthrough. Nothing was changed. |
| 64 | Bad command line, or a `--compare` file or the `--out` folder could not be used (nothing was sent). |

When several apply, 3 beats 1 beats 2.

## `esc-passthrough-restore` (writes; `--features write`)

```
cargo run --release --features write --bin esc-passthrough-restore -- \
    --esc N --file FILE --yes-write-to-esc [--port PATH | --vid-pid VVVV:PPPP] \
    [--address 0x7C00] [--length 184] [--allow-change]
```

One controller per run; `N` is 1-based, as `esc-passthrough-host` prints the controllers. **Without
`--yes-write-to-esc` it stops before opening the port** (exit 64).

Sequence: the same MSP opening and passthrough request as the read-only tool; init-flash for controller
`N`; read the block (`--address`, `--length`; `FILE` must have exactly that many bytes) and compare it
with `FILE`.

- **Default, identical only.** If `FILE` differs from the block now in the controller, nothing is written,
  the differing offsets are printed (old -> new), passthrough is left, and the exit status is 2. In this
  mode the tool cannot change a setting; it only proves that a write works.
- With `--allow-change` a different `FILE` is accepted: the differing offsets (old and new byte) are
  printed first, then it is written.
- **The write**, as the configurator saves settings (`four_way.ts` `writeSettings` and `write`, lines
  493-565): ONE `cmd_DeviceWrite` of the whole block at the address, then a `cmd_DeviceRead` of the same
  range. No page-erase, no reset, no other four-way command: the request enum is the read-only tool's
  plus device-write (`write::Request`).
- The read-back is compared with `FILE` and the result is printed per byte range. A difference at offset
  `0x02` only is reported separately (newer bootloaders stamp their version there) and counts as
  verified. If the write is acknowledged with an error, or the read-back differs, the same block is
  written again, at most twice more (three writes in all); a write is never retried by the retry rule on
  silence, because whether it took effect is then unknown.
- Passthrough is always left at the end (interface-exit), and the last line, starting `FINAL:`, states
  plainly what was written to which controller. When every write was answered with an error it says
  `NO WRITE ACCEPTED` and whether the block read afterwards equals the block read before the writes.

### Exit status

| Status | Meaning |
|---|---|
| 0 | Written and verified. |
| 1 | The board or the controller did not answer. The `FINAL:` line says whether a write had been sent (if it had, its result is unknown: read the block back with `esc-passthrough-host`). |
| 2 | Refused by the identical-only rule; nothing was written. |
| 3 | The board refused passthrough. |
| 4 | Written, but the read-back still differs after three writes (or no write was acknowledged). Both blocks are printed in hex. This is the case a person must act on. |
| 64 | Bad command line, including a missing `--yes-write-to-esc`, a `FILE` that cannot be read, or one whose size is not `--length`. |

## The read-only rule, and how it is checked

This project's own tools never write to a controller, except `esc-passthrough-restore` for the one
purpose above. Everything that can form a device-write is in `src/write/` (the request, its command
number, the restore sequence, its command line and its `main`), which is compiled only with the cargo
feature `write`: `src/lib.rs` has `#[cfg(feature = "write")] pub mod write;` and the binary
`esc-passthrough-restore` has `required-features = ["write"]` in `Cargo.toml`. A default build contains
none of it. Check it from this folder:

```
grep -rnEi "0x3[0-9a-f]\b" src/ --exclude-dir=write   # exactly four lines, all in src/fourway.rs:
                                                      # 0x30 test-alive, 0x34 exit, 0x37 init-flash, 0x3A read
grep -rniE "devicewrite|device_write|pageerase|page_erase|eraseall|erase_all|devicereset|device_reset|writeeeprom|write_eeprom|c2ck|setmode|set_mode" src/ --exclude-dir=write   # no output
grep -rn 'feature = "write"' src/ --exclude-dir=write # exactly one line: src/lib.rs, the gate of `mod write`
grep -rn "write_all" src/ --exclude-dir=write         # only src/link.rs: the trait method, its one call (in `exchange`) and the serial port's implementation
grep -rnEi "0x3[0-9a-f]\b" src/write                  # the write module's one extra number: 0x3B, device-write
```

`cargo build --release` prints no mention of `esc-passthrough-restore`, and `strings` on the resulting
`esc-passthrough-host` finds none of `DeviceWrite`, `yes-write-to-esc` or `WRITTEN`.

`tests/codecs.rs` also keeps the rule for the read-only tool: a `match` over every `fourway::Request`
variant (adding a variant stops the test compiling) and a check that each encodes to one of the four
allowed command numbers; and `tests/fake_board.rs` checks that the simulated flash images are
byte-identical after a full run (and after a dump) and that only the init, set-address and read bursts
reached the simulated controllers.

## Tests

```
cargo test                      cargo test --features write
cargo clippy --all-targets -- -D warnings      (also with --features write)
cargo fmt -- --check
```

- `tests/codecs.rs`: the tool's MSP v1 and four-way codecs against the vectors of the crate's
  `tests/msp.rs` and `tests/fourway.rs` (checked against the configurator's source), the CRC against a
  bit-serial reference, the retry rule, the refusal path, exit statuses, comparison and saving.
- `tests/cli.rs`: the read-only tool's command line.
- `tests/fake_board.rs`: the proof on the computer for the read-only tool, including `--dump`. Two
  in-memory pipes join the tool (this thread) to a second thread that runs
  `esc_passthrough::session::Passthrough::run` (the crate under test, a path dependency) with a lender
  handing out the crate's simulated bootloaders (`../tests/common/sim.rs`, included by `#[path]`, not
  copied). Four controllers with different settings blocks; the park and break delays are zero. The
  wiring is in `tests/common/mod.rs`.
- `tests/restore.rs` (only with `--features write`): the restore tool on the same fake board with the
  simulator's 4-byte write unit: identical write-back leaves the flash image byte-identical with exactly
  one set-buffer, payload and program sequence at the block's address; the identical-only refusal writes
  nothing (exit 2); `--allow-change` changes exactly the differing bytes; a failed payload is repeated
  and ends verified; a read-back that keeps differing ends with status 4 after exactly three writes; the
  binary stops before opening the port without `--yes-write-to-esc`.

The tools' protocol code is their own (`src/msp.rs`, `src/fourway.rs`, `src/write/`), written
independently of the crate, which has only the flight-controller side; the crate is a dev-dependency used
by the fake board only.
