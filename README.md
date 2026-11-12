# esc-passthrough

Sans-IO codecs for letting the AM32 configurator on a computer reach ESCs through a flight controller, `no_std`, no allocation. The crate holds the three wire formats the flight controller needs: the MSP v1 messages the configurator opens with, the "four-way interface" it speaks afterwards, and the byte sequences of the AM32 bootloader on the ESC's signal wire. Bytes in, typed values out, and back. On top of the bootloader codecs sits the one piece that does I/O without owning any: a small async client for the exchanges on one ESC's line (connect, read, write, erase, keep-alive, run), over a line trait the board implements. On the computer side there is the sans-IO host side: a recogniser for the one MSP request that opens the configurator's conversation (`MspTrigger`) and `HostSide`, which takes the configurator's bytes, answers the MSP requests that need no decision, hands out the four-way requests and builds their replies, without executing anything on an ESC. The crate owns no timer, keeps no session on the lines and names no transport, HAL or board. It has no dependencies; `defmt` 1.0 is optional. It is written for the MARV V2 flight controller but takes everything from the caller.

Status: version 0.1.0, `publish = false`, and the API may still change. It has been tested on a host only. **Nothing has run against a real ESC or against the real configurator**: every byte sequence was written from the configurator's source, the AM32 bootloader's source and ArduPilot's behaviour, and the golden vectors were worked out by hand from them (no captured traffic exists).

## Quick start

MSP: parse the configurator's requests a byte at a time (or in slices, split anywhere) and write the reply into your own buffer.

```rust
use esc_passthrough::msp::{encode_motor_config, Command, MotorConfig, RequestParser};

let mut parser = RequestParser::new();
let mut reply = [0u8; 32];
let mut reply_len = 0;
// What the configurator sends for MSP_MOTOR_CONFIG (131).
for &byte in &[0x24, 0x4D, 0x3C, 0x00, 0x83, 0x83] {
    if let Some(Ok(request)) = parser.push(byte) {
        if request.kind() == Command::MotorConfig {
            reply_len = encode_motor_config(MotorConfig::new(4), &mut reply).unwrap();
        }
    }
}
assert_eq!(
    &reply[..reply_len],
    &[0x24, 0x4D, 0x3E, 0x0A, 0x83, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0x8D]
);
```

Four-way: parse a request, answer it. Parameters and addresses are opaque.

```rust
use esc_passthrough::fourway::{encode_reply, Ack, Command, RequestParser};

let mut parser = RequestParser::new();
// InitFlash for ESC 0, as the configurator sends it.
let request = [0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00];
let (used, event) = parser.feed(&request);
assert_eq!(used, request.len());
let request = event.unwrap().unwrap();
assert_eq!((request.command, request.address, request.params), (Command::DeviceInitFlash, 0, &[0u8][..]));

// The reply carries [signature low, signature high, pin code, interface mode].
let mut reply = [0u8; 16];
let n = encode_reply(request.command, request.address, &[0x06, 0x1F, 0x14, 0x04], Ack::Ok, &mut reply).unwrap();
assert_eq!(&reply[..n], &[0x2E, 0x37, 0, 0, 4, 0x06, 0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A]);
```

Bootloader: the bytes for "set address, then read 4 bytes", and the check of the answer.

```rust
use esc_passthrough::bootloader::{encode_read, encode_set_address, parse_ack, parse_read_reply, Ack};

let mut wire = [0u8; 16];
let n = encode_set_address(0x7C00, &mut wire).unwrap();
assert_eq!(&wire[..n], &[0xFF, 0x00, 0x7C, 0x00, 0x10, 0xD4]);
assert_eq!(parse_ack(&[0x30]), Ok(Ack::Good));

let n = encode_read(4, &mut wire).unwrap();
assert_eq!(&wire[..n], &[0x03, 0x04, 0x01, 0x33]);

// The ESC answers with the data, its CRC (low byte first) and 0x30.
let answer = [0x01, 0x02, 0x03, 0x04, 0xA1, 0x0F, 0x30];
assert_eq!(parse_read_reply(&answer, 4), Ok(&answer[..4]));
```

Bootloader client: connect to an ESC over a line. The line here is a fake that answers the init and nothing else; a board implements `EscLines` for its pins and its clock.

```rust
use core::convert::Infallible;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use esc_passthrough::client::{Client, Error, Step};
use esc_passthrough::line::EscLines;

struct FakeLine {
    to_read: &'static [u8],
    sent: usize,
}

impl EscLines for FakeLine {
    type Error = Infallible;
    fn select(&mut self, _line: u8) -> Result<(), Infallible> {
        Ok(())
    }
    async fn write(&mut self, bytes: &[u8]) -> Result<(), Infallible> {
        self.sent += bytes.len();
        Ok(())
    }
    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, Infallible> {
        // A real line waits up to `timeout_us` for a byte; this one has what is left or nothing.
        let n = if timeout_us == 0 { 0 } else { buf.len().min(self.to_read.len()) };
        buf[..n].copy_from_slice(&self.to_read[..n]);
        self.to_read = &self.to_read[n..];
        Ok(n)
    }
    fn set_break(&mut self, _on: bool) -> Result<(), Infallible> {
        Ok(())
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

// What an ESC answers to the init: "471", pin code, flash size code, 6, 6, version 3, 0x30.
let reply = &[0x34, 0x37, 0x31, 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30];
let mut client = Client::new(FakeLine { to_read: reply, sent: 0 });
client.select(0).unwrap();
let info = block_on(client.connect()).unwrap();
assert_eq!((info.pin_code, info.signature()), (0x14, 0x1F06));
assert_eq!(client.line().sent, 21);
// The fake has nothing more to say, so a keep-alive gets no answer in time.
assert_eq!(
    block_on(client.keep_alive()),
    Err(Error::Timeout { step: Step::KeepAlive })
);
```

Host side: the trigger, then the configurator's conversation. `MspTrigger` finds the 6-byte request in a stream that carries other traffic; `HostSide` answers the MSP requests, reports `MSP_SET_PASSTHROUGH` for the application to grant, and hands out the four-way requests.

```rust
use esc_passthrough::fourway::Command;
use esc_passthrough::host::{answer_housekeeping, Event, HostInfo, HostSide, Mode};
use esc_passthrough::trigger::MspTrigger;

// The trigger: 3 bytes of other traffic, then MSP_API_VERSION split across two reads.
let mut trigger = MspTrigger::new();
assert_eq!(trigger.feed(&[0x55, 0x24, 0x24, 0x4D, 0x3C]), None);
assert_eq!(trigger.feed(&[0x00, 0x01, 0x01, 0xAA]), Some(3));

let info = HostInfo::new().with_motor_count(4);
let mut host = HostSide::new();
let mut out = [0u8; 64];

// MSP_API_VERSION is answered from `info` (ArduPilot's defaults: 0.1.42).
let (used, event) = host.feed(&[0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01], &info, &mut out);
assert_eq!((used, event), (6, Some(Event::Reply { len: 9 })));
assert_eq!(&out[..9], &[0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2A, 0x29]);

// MSP_SET_PASSTHROUGH is the application's decision; here it grants with 4 ESCs.
let (_, event) = host.feed(&[0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5], &info, &mut out);
assert_eq!(event, Some(Event::PassthroughRequested));
let len = host.grant(4, &mut out).unwrap();
assert_eq!(&out[..len], &[0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0]);
assert_eq!(host.mode(), Mode::FourWay);

// A four-way request for the protocol version needs no ESC and is answered by a helper.
let request = [0x2F, 0x31, 0x00, 0x00, 0x01, 0x00, 0x65, 0x85];
let (used, event) = host.feed(&request, &info, &mut out);
assert_eq!(used, request.len());
let Some(Event::FourWay(request)) = event else { panic!("expected a request") };
assert_eq!(request.command, Command::ProtocolGetVersion);
let len = answer_housekeeping(&info, &request, &mut out).unwrap().unwrap();
assert_eq!(&out[..len], &[0x2E, 0x31, 0x00, 0x00, 0x01, 0x6B, 0x00, 0xD6, 0xB2]);

// After InterfaceExit the application reports it done and MSP is back.
host.exited();
assert_eq!(host.mode(), Mode::Msp);
```

## What the application provides

- The bytes: the host side's serial stream goes into the two request parsers; the ESC's bytes go into the reply parsers, already read from the wire. The crate reads and writes nothing itself.
- The values for the MSP replies: API version triple, the four ASCII bytes of the flight controller variant (the configurator knows `BTFL`, `KISS`, `INAV`, `ARDU` and `EMUF`), battery voltage in millivolts with optional current and charge, the number of motors, the number of ESCs.
- Output buffers. Every encoder takes `&mut [u8]`, returns the length written, and returns `EncodeError::BufferTooSmall { needed }` without writing if it does not fit. `msp::MAX_FRAME_LEN`, `fourway::MAX_REPLY_LEN` and `bootloader::MAX_DATA + 2` are the largest sizes.
- For the client: an `EscLines` implementation (select a line, send bytes back to back, read with a timeout that the implementer measures, pull all lines low) and, if the defaults do not suit, `client::Timeouts`.
- Ordering and sessions: which four-way command becomes which bootloader sequence, when to retry, when to hold the line, what to do after an error (see the rules below).

## Rules for correct use

1. A parser's request borrows its storage: use the `Request` before the next `push` or `feed`. `feed` returns after the first event with the number of bytes it used; call it again with the rest.
2. Call `reset()` on a request parser when the stream pauses in the middle of a frame (a timeout), or the next frame's bytes finish the old one.
3. The parsers skip bytes outside a frame, so the two request parsers can listen to one stream. A frame is consumed by its declared length: a corrupted length byte can swallow part of the next frame, and the checksum then fails (the configurator retries). A byte that ends a failed frame or breaks a header is looked at again as a frame start: a `$` for MSP, `0x2F` for four-way. A bad-CRC four-way frame whose last byte is `0x2F` therefore starts a new frame at that byte.
4. The four-way parameter count 0 means 256. An empty parameter list does not exist on the wire: send one `0x00`, as the configurator does. `fourway::encode_reply` refuses 0 or more than 256 parameters.
5. `bootloader::parse_read_reply` wants exactly `n + 3` bytes. A refused read is answered with one byte; shorter input is `Truncated` and the caller decides, with its timeout, whether to read that byte with `parse_ack`.
6. A bootloader acknowledge of `C1` or `C2` is a value (`Ack`), not an error, for `parse_ack`; the keep-alive expects `C1`. In replies that should be good (`parse_init_reply`, `parse_read_reply`) any other acknowledge is `ParseError::Rejected`.

7. The bootloader keeps state, and the client closes the hazard it makes. After set buffer the ESC takes the next frame it receives as the payload, until a payload with a good CRC has arrived; if a payload is refused (`C2`) or lost it keeps waiting, and it passes the next valid command as a payload, answers `30` and does not run it, so the following write's set address would be acknowledged and ignored and that write would land at the old address. Therefore `Client` tracks the line (choose it with `Client::select`, not with `line_mut().select()`; lines 0 to 31) and marks it when a `write` fails after set buffer went out and before the payload's `30` came. The next operation on a marked line, or `Client::resync()`, first sends one keep-alive and accepts `30` (the ESC was waiting and took it) or `C1` (it was not); anything else is an error, the mark stays and the operation does not start. `needs_resync()` tells. One limit: an ESC waiting for a payload of one byte answers the keep-alive `C2`, and the mark then stays (`resync` returns `Refused` at `Step::Resync`); the client has no safe way to clear that.
8. The bootloader source from v12 on answers the run command with `C1` before it jumps, and other bootloaders may send nothing; `run_application` reads and drops a `C1` and accepts silence. `Ok` does not say that the ESC left the bootloader; as the bootloader does, it may refuse to jump (the simulator models its checks: the first byte at the settings address must be 1, and the application's first two words must be in range).
9. A write the ESC refuses comes back as `Error::Refused` and is not retried or reshaped: the client passes through what it is given. In the bootloader sources a program is refused (`C1`) below the application, and on every part also for an address or length that is not a multiple of the part's flash word (2 bytes on the F051, 4 on the F421, 8 on the G071, G431 and L431), so a write of 1, 2 or 255 bytes is refused on most parts. What the bootloader does on a write at the settings address longer than 2 bytes: it replaces byte 2 with its own version, so the read-back differs.
10. Every client operation first reads the line with a timeout of 0 until nothing comes, to drop bytes an earlier failed exchange left behind. A late answer that arrives after that is not caught; the session decides how long to let the line settle after a timeout.

## What the crate does

- `msp`: `RequestParser` for MSP v1 requests (`$M<`, length, command, payload, XOR checksum) yields `Request { command, payload }` or `ParseError::{BadHeader, BadChecksum}`. Storage: 255 payload bytes inside the struct. `Request::kind()` sorts the command into the five the configurator needs (1, 2, 130, 131, 245) or `Command::Unsupported(n)`; the crate does not answer an unsupported command. Replies: `encode_api_version`, `encode_fc_variant`, `encode_battery_state` (the 11-byte ArduPilot layout; the voltage in 0.1 V at byte 3 is the one the configurator reads, rounded and saturating at 25.5 V; cell count, capacity and state are 0; current in 0.01 A, charge in mAh, voltage in 0.01 V, all rounded and saturating), `encode_motor_config` (the motor count at byte 6, the other nine bytes 0), `encode_set_passthrough` (the ESC count), `encode_error` (`$M!`, length 0, the command, checksum) and the general `encode_reply`.
- `fourway`: `RequestParser` for `0x2F`, command, address (high first), count (0 is 256), parameters, CRC16-XMODEM (high first); storage: 256 parameter bytes inside the struct. `Command` has every code of ArduPilot's protocol header (0x30 to 0x3F except 0x36, and 0x40) and `Unknown(code)`; `Ack` has the eleven codes of the configurator's table and `Unknown(code)`. `encode_reply` writes `0x2E`, command, address, count, parameters, acknowledge, CRC. `crc16_xmodem` and `crc16_xmodem_update` are public.
- `bootloader`: `INIT` and `encode_init` (21 bytes, no CRC), `encode_set_address`, `encode_set_buffer` (1 to 256, with the high byte 1 for 256), `encode_payload` (data and its CRC, 1 to 256 bytes), `encode_program`, `encode_erase` (`02 01`), `encode_read` (1 to 256, sent as `n`, 256 as 0), `encode_keep_alive`, `encode_run` (`00 00 00 00`), `crc16` (0xA001 reflected, start 0, low byte first). Reply parsers: `parse_ack`, `parse_init_reply` (`DeviceInfo`: pin code, flash size code, two bytes of 0x06, protocol version; `signature()` is the 16-bit value the configurator uses to name the MCU), `parse_read_reply` (data, CRC, acknowledge; the data is returned as a borrowed slice).
- `line`: the trait `EscLines`: `select(line)`, `async write(bytes)` (back to back, returns after the last stop bit), `async read(buf, timeout_us)` (returns at once with at least one byte, or `Ok(0)` after the timeout; the implementer owns the clock), `set_break(on)`. The line does not echo what is written. The trait uses `async fn` in a public trait (the `async_fn_in_trait` lint is allowed in that module, as in the other drivers), which needs no dependency and no executor in this crate.
- `client`: `Client<L: EscLines>` with `connect` (init, 9-byte reply into `DeviceInfo`), `read(address, buf)` and `write(address, data)` (1 to 256 bytes; write is set address, set buffer, wait for the answer that does not come, payload, program), `erase(address)`, `keep_alive` (the `C1` is success), `run_application`. Each is one exchange or a fixed sequence and returns `Error<E>`, which says the `Step` and which of: line error, nothing in time, short answer, bad CRC from the ESC, refused (`C1`, `C2`), unexpected byte; plus `InvalidLength` for an argument outside 1 to 256 (nothing is sent) and `InvalidLine` for a line above 31. `select(line)` forwards to the line and remembers it; `resync()` and `needs_resync()` belong to rule 7. `Timeouts` (`#[non_exhaustive]`, `new()` and `with_` setters) holds the waits; the defaults are ArduPilot's: 2 ms for the acknowledge of set address and keep-alive, 5 ms after set buffer, 40 ms for the payload, 500 ms for program, 3 s for erase, and for a multi-byte reply (init, read) the first byte may take 1 ms times the length of the reply and each next byte 1 ms. The client never looks inside data or buffers and has no notion of what an address means; `read` writes the caller's buffer only when the whole reply is good.
- `trigger`: `MspTrigger::feed(&[u8]) -> Option<usize>` recognises exactly `24 4D 3C 00 01 01` (MSP v1 `MSP_API_VERSION`, no payload) in a byte stream, at any split. `Some(n)` says the request completed with the `n`th byte of the slice (the rest is not looked at); a partial match is kept for the next call. It fires once per occurrence and never on a request with one byte changed, another length or another command; it knows nothing else about MSP.
- `host`: `HostSide` owns one MSP and one four-way request parser and the mode. `feed(bytes, &HostInfo, out) -> (used, Option<Event>)` stops at the first event and writes at most one reply, only after the last byte of a complete request. In MSP mode it answers commands 1, 2, 130 and 131 from `HostInfo` (`Event::Reply`; defaults are ArduPilot's), answers any other command and a `MSP_SET_PASSTHROUGH` with a payload with the MSP error reply, ignores malformed frames, and reports a plain `MSP_SET_PASSTHROUGH` as `Event::PassthroughRequested`: the application calls `grant(esc_count, out)` (four-way mode) or `refuse(out)` (error reply, stays MSP); until it does, `feed` consumes nothing and reports the request again. In four-way mode each CRC-valid request is handed out as `Event::FourWay` (command, address, parameters, all opaque) and answered by the application with `encode_success` or `encode_failure` (command and address echoed, as `four_way.ts:400-402` requires), or for protocol version, interface name and interface version by `answer_housekeeping`; a request with a bad CRC is answered by the crate (ACK `03`, the received command and address and first parameter, ArduPilot's failure-reply shape) and not handed out. The application calls `exited()` when `InterfaceExit` is dealt with and the mode is MSP again. A configurator that restarts without exiting: bytes go to the four-way parser first, and only bytes that arrive between four-way frames and are not `0x2F` go to the MSP parser; a complete valid MSP request found that way is `Event::Restarted` (the mode stays four-way until the application calls `exited()`), MSP errors there are ignored, and a `$` inside a four-way frame never reaches the MSP parser. No ESC index check, no line, no timing.
- Addresses and payloads are opaque. The crate has no settings layout, no setting names and no magic address list.
- No panic for any input and no allocation. Enums that copy a complete protocol table (`fourway::Command`, `fourway::Ack`, `bootloader::Ack`) are exhaustive; error enums and the structs that hold reply values are `#[non_exhaustive]` (build the structs with `new` and the `with_` setters).

## Not covered

- The session that executes requests on the lines: turning the four-way requests `HostSide` hands out into bootloader traffic with its retries and waits (InitFlash trying the bootloader up to three times, DeviceWrite as the client's `write`, and so on), checking ESC indexes, lending the lines to it, and the 5 s limit before the grant. `HostSide` checks nothing about an ESC; the client only resyncs a line after a failed payload. Nothing here reads a host serial port or runs an async loop over one.
- Entering and leaving the bootloader: holding the line high, the 2 s wait for the ESC to reset, the 20 ms break that sends it back to its application. `set_break` is on the trait; nothing calls it.
- Anything on a board: pins, PIO, the USB serial port, stopping the motor signal, the permission to take the lines.
- A timer: the client passes timeouts to the line and measures nothing. The bootloader's own limits (5 bit times between the bytes of a command, 250 us for a command without payload, 20 ms of line low) are the line implementation's business; neither the client nor the test simulator checks them.
- The bootloader's page erase: the `02 01` command only checks the address, and the client's `erase` does nothing more. The four-way commands for devices other than the AM32 bootloader: the codes parse and encode, nothing answers them.
- The opposite directions, for a computer-side tool: encoders for MSP and four-way requests, parsers for MSP and four-way replies, an encoder for the ESC's bootloader replies. Also MSP v2, MSP messages other than the five, and the configurator's "direct" route (one ESC, 19200 baud, its own bytes echoed).
- Reading the settings block, the file name or the device-info block that the bootloader serves at its magic addresses: `read` reads any address, it does not interpret what comes back.

## Tests

Host: `cargo test` (integration tests, proptests and the examples in this file, which are doctests). Each encoder and parser has golden vectors whose layout and source lines are cited next to them in `tests/msp.rs`, `tests/fourway.rs` and `tests/bootloader.rs`; the bytes were laid out by hand from the sources (the configurator's `four_way.ts`, `direct.ts` and `commands.queue.ts`, the npm package `@am32/serial-msp`, the bootloader's `main.c`, ArduPilot's behaviour) and their checksums computed with a separate script. The two CRCs are compared with bit-serial reference versions written in the tests and with the published check values for `"123456789"` (CRC16-XMODEM 0x31C3, CRC-16/ARC 0xBB3D). The proptests check that encoding then parsing round-trips (against independent decoders written in the tests), that parsing is the same however the stream is split, that arbitrary bytes never panic and that every frame a parser yields is one an independent whole-buffer parser also finds with a correct checksum, that a corrupted byte anywhere in a valid frame is rejected and the next frame still parses (the XOR checksum and the CRCs catch every single-byte change; a corrupted length byte is checked against the independent parser, because it can by chance make a different, correctly checksummed frame), and the reply parsers' prefixes, trailing bytes and corrupted bytes.

The client is tested against a simulated bootloader in `tests/common/sim.rs` (up to four ESCs, each with a flash image), written from the bootloader's `main.c` and not from this crate's encoders, and checked on its own in `tests/simulator.rs` with hand-worked byte sequences. It models the init match, the device-info reply, set address with the magic addresses, set buffer without answer, the payload with its CRC check (and the wait for another payload after a bad one, which the client's resync answers), program (refused below the application and for a misaligned write on parts that demand it; a plain overwrite, with no page erase), erase, read (the address is forgotten), keep-alive, run (with the jump checks), the three acknowledges, the count of bad commands and the jump after 100. Its fault switches: no answer, a late answer, a truncated answer, a corrupted CRC, a wrong acknowledge, any replacement bytes, answers split into single bytes, a damaged request, an injected line error. Time in it is virtual. The client tests also check the exact bytes and the order and timeouts of the line calls, that they do not depend on the data, that every error variant is produced and the next operation then works, and with proptests that arbitrary bytes from the line never panic or pass for other data. The simulator and the `block_on` that drives the futures are test code only.

The host side is tested in `tests/trigger.rs` and `tests/host.rs`. The trigger: the 6 bytes inserted at random positions into random bytes and into random well-formed frames of another format give exactly one firing per insertion at every split (checked against a naive search), a stream with no insertion fires only where a naive search finds the sequence, every single-byte corruption and every other length or command give none. `HostSide`: the configurator's connect conversation (four MSP requests, passthrough with grant, InitFlash, DeviceRead, InterfaceExit, MSP again) with the replies compared byte for byte against vectors laid out by hand and a separate CRC script, at every split into two and three parts and byte by byte (each event, and so each reply, comes at the last byte of its request); refusal and a second attempt; unknown command; bad four-way CRC answered and not handed out; the restart rule (a valid MSP request between frames is `Restarted`, the same six bytes inside a frame's parameters are not); mode after `exited()`; oracle proptests for both modes at random splits; arbitrary bytes never panic.

The simulator and every timing here follow the AM32 bootloader source from v12 on. Older bootloaders exist in the field (the lead's bench controllers report bootloader version 1, whose source is not available), and nothing in this crate has been run against any real bootloader.

## Measured on a board

Nothing. The crate has not been run on a board, against a real ESC or against the real configurator.

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public types.

```text
Cargo.toml   the crate
src/         msp.rs, fourway.rs, bootloader.rs, line.rs, client.rs, trigger.rs, host.rs, error.rs (EncodeError and the output writer)
tests/       one file per module (client.rs for line and client, trigger.rs, host.rs), simulator.rs, and common/ with the bit-serial reference CRCs, the simulated bootloader (sim.rs) and block_on.rs
```

MIT, see `LICENSE`.
