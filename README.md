# esc-passthrough

Sans-IO codecs for letting the AM32 configurator on a computer reach ESCs through a flight controller, `no_std`, no allocation. The crate holds the three wire formats the flight controller needs: the MSP v1 messages the configurator opens with, the "four-way interface" it speaks afterwards, and the byte sequences of the AM32 bootloader on the ESC's signal wire. Bytes in, typed values out, and back. On top of the bootloader codecs sits the one piece that does I/O without owning any: a small async client for the exchanges on one ESC's line (connect, read, write, erase, keep-alive, run), over a line trait the board implements. On the computer side there is the sans-IO host side: a recogniser for the one MSP request that opens the configurator's conversation (`MspTrigger`) and `HostSide`, which takes the configurator's bytes, answers the MSP requests that need no decision, hands out the four-way requests and builds their replies, without executing anything on an ESC. Finally the session (`session::Passthrough`) joins the two: it reads the configurator's bytes from any `embedded_io_async` stream, executes each four-way request on the lines the firmware lends it, and writes the replies back. The crate owns no timer (the caller's `embedded_hal_async::delay::DelayNs` is the only clock), owns no stream and no line, and names no transport, HAL or board. Its dependencies are `embedded-io-async` 0.7 and `embedded-hal-async` 1.0 (with the `embedded-io` and `embedded-hal` they re-export traits from); `defmt` 1.0 is optional. It is written for the MARV V2 flight controller but takes everything from the caller.

Status: version 0.1.0, `publish = false`, and the API may still change. It has been tested on a host, and on one bench (2026-11-12): through the demonstration in `hil/` on a MARV V2 board the real AM32 configurator read four ESCs, changed a setting on one and flashed one (AM32 2.20 on AT32F421, bootloader version 1; see "Measured on a board"). Every byte sequence was written from the configurator's source, the AM32 bootloader's source and ArduPilot's behaviour, and the golden vectors were worked out by hand from them; that bench is the only real traffic they have met. Not tried: any other ESC, MCU or bootloader version, and a serial port shared with another protocol.

## Works with

The crate works with any HAL and any MCU that provides `embedded-io-async` 0.7 streams and an `embedded-hal-async` 1.0 `DelayNs`. No embassy crate is needed.

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

Session: the configurator's requests arrive on a byte stream and are executed on the lines. Everything here is a fake, as in the examples above: a line that answers the init, a stream that holds the whole conversation, a delay that does not wait (it pends once, so a read that has data wins the race against the idle timer) and a lender that hands the line over once. A board provides the real ones. The firmware has already read the trigger request out of the stream and hands it to `run` as `preload`.

```rust
use core::convert::Infallible;
use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorType, Read, Write};
use esc_passthrough::line::EscLines;
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Lender, Passthrough, SessionConfig};

struct FakeLine {
    to_read: &'static [u8],
    breaks: u32,
}

impl EscLines for FakeLine {
    type Error = Infallible;
    fn select(&mut self, _line: u8) -> Result<(), Infallible> {
        Ok(())
    }
    async fn write(&mut self, _bytes: &[u8]) -> Result<(), Infallible> {
        Ok(())
    }
    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, Infallible> {
        let n = if timeout_us == 0 { 0 } else { buf.len().min(self.to_read.len()) };
        buf[..n].copy_from_slice(&self.to_read[..n]);
        self.to_read = &self.to_read[n..];
        Ok(n)
    }
    fn set_break(&mut self, on: bool) -> Result<(), Infallible> {
        self.breaks += u32::from(on);
        Ok(())
    }
}

struct FakeStream {
    input: &'static [u8],
    output: Vec<u8>,
}

impl ErrorType for FakeStream {
    type Error = Infallible;
}

impl Read for FakeStream {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        let n = buf.len().min(self.input.len());
        buf[..n].copy_from_slice(&self.input[..n]);
        self.input = &self.input[n..];
        Ok(n)
    }
}

impl Write for FakeStream {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        self.output.extend_from_slice(buf);
        Ok(buf.len())
    }
    async fn flush(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

struct FakeDelay;

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, _ns: u32) {
        let mut pended = false;
        poll_fn(|_| {
            if pended { Poll::Ready(()) } else { pended = true; Poll::Pending }
        })
        .await;
    }
}

struct FakeLender(Option<FakeLine>);

impl Lender for FakeLender {
    type Lines = FakeLine;
    async fn lend(&mut self) -> Option<FakeLine> {
        self.0.take()
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

// What the configurator sends after the trigger: MSP_SET_PASSTHROUGH, InitFlash for ESC 0,
// InterfaceExit.
let input: &[u8] = &[
    0x24, 0x4D, 0x3C, 0x00, 0xF5, 0xF5,
    0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0xA8, 0x00,
    0x2F, 0x34, 0x00, 0x00, 0x01, 0x00, 0x46, 0xD2,
];
let trigger = [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];
// The ESC's answer to the init: "471", pin code 0x14, 0x1F, 6, 6, version 3, 0x30.
let esc = FakeLine { to_read: &[0x34, 0x37, 0x31, 0x14, 0x1F, 0x06, 0x06, 0x03, 0x30], breaks: 0 };

let mut rx = FakeStream { input, output: Vec::new() };
let mut tx = FakeStream { input: &[], output: Vec::new() };
let mut lender = FakeLender(Some(esc));
let mut session = Passthrough::new(SessionConfig::new());
let exit = block_on(session.run(&trigger, &mut rx, &mut tx, &mut lender, &mut FakeDelay, || BatteryState::new(0)));
assert_eq!(exit, Exit::Exited);

// MSP_API_VERSION, the grant (4 ESCs), the init reply (signature 0x1F06, pin code 0x14, mode 4),
// and the exit reply.
assert_eq!(
    tx.output,
    [
        &[0x24, 0x4D, 0x3E, 0x03, 0x01, 0x00, 0x01, 0x2A, 0x29][..],
        &[0x24, 0x4D, 0x3E, 0x01, 0xF5, 0x04, 0xF0],
        &[0x2E, 0x37, 0, 0, 4, 0x06, 0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A],
        &[0x2E, 0x34, 0, 0, 1, 0x00, 0x00, 0x42, 0x63],
    ]
    .concat()
);
// The lines exit is done, so the lines can be given back to the firmware.
assert!(session.holds_lines());
let line = session.release().unwrap();
assert_eq!(line.breaks, 1);
```

## What the application provides

- The bytes: the host side's serial stream goes into the two request parsers; the ESC's bytes go into the reply parsers, already read from the wire. The crate reads and writes nothing itself.
- The values for the MSP replies: API version triple, the four ASCII bytes of the flight controller variant (the configurator knows `BTFL`, `KISS`, `INAV`, `ARDU` and `EMUF`), battery voltage in millivolts with optional current and charge, the number of motors, the number of ESCs.
- Output buffers. Every encoder takes `&mut [u8]`, returns the length written, and returns `EncodeError::BufferTooSmall { needed }` without writing if it does not fit. `msp::MAX_FRAME_LEN`, `fourway::MAX_REPLY_LEN` and `bootloader::MAX_DATA + 2` are the largest sizes.
- For the client: an `EscLines` implementation (select a line, send bytes back to back, read with a timeout that the implementer measures, pull all lines low) and, if the defaults do not suit, `client::Timeouts`.
- For the session: the host byte stream as `embedded_io_async::Read` and `Write` (the read must be cancel-safe: a read future that is dropped has taken no byte), a `DelayNs` (the only clock the session has), a `session::Lender` that hands the lines over when the configurator asks and returns `None` to refuse, and a closure that returns the battery figure when the configurator asks for it.

## What the firmware must do

The session takes the motor controllers' lines, so two duties belong to the firmware, not to the crate:

1. **Lend the lines only while the vehicle is disarmed.** `Lender::lend` is called at `MSP_SET_PASSTHROUGH` and nowhere else (never while the session already holds lines); return `None` to refuse and the configurator gets the MSP error reply and may ask again. The crate has no notion of armed.
2. **After cancelling `Passthrough::run` (dropping its future, at any await), call `finish()`.** Until `finish` has run the exit, the controllers stay inert in their bootloaders until they are power-cycled. `release()` refuses (`ReleaseError::ExitOwed`) while lines are held and the exit break is not done, and gives the lines back after. `run` calls `finish` itself when it returns, whatever the `Exit`.

Also: stop whatever else drives the lines before lending them, and take them back (`release()`) before driving them again. The firmware's own tap that decides when to start a session at all (it recognises the trigger with `MspTrigger` and hands the bytes it has read to `run` as `preload`) is the firmware's.

## The session

`Passthrough::run(preload, rx, tx, lender, delay, battery)` serves one configurator until it leaves and returns why: `Exit::Idle` (`idle_ms`, default 5 s, without a complete valid MSP request while no lines are held; bytes that are no request do not restart the time), `Exit::Exited` (`InterfaceExit`), `Exit::Closed` (the stream ended) or `Exit::Host(e)` (a read, write or flush failed). Whenever lines were held, the lines exit is done before `run` returns: all lines low together for `break_ms` (default 25, the bootloader jumps after 20 ms), then released.

- The lines are taken at `MSP_SET_PASSTHROUGH` only, through the lender. After the reply the session waits `park_ms` (default 3000) with the caller's delay, with the lines held high and silent, before any line operation; the host stream is not read meanwhile. After that one line is active at a time, the one of the controller the request names (`line_map` maps the index to the line number).
- Each four-way request is answered exactly once, echoing its command and address; nothing is written to the host except in reply to a complete request. The line operation of each command, the reply and the acknowledge are in the table of the `session` module documentation (init-flash is one connect, no retry; read, write 2 to 256 bytes, page erase at `page << 10` and device reset map to the client's operations; every other command gets what ArduPilot answers for an ARM bootloader, or the invalid-command acknowledge). Any client error becomes the general-error acknowledge and the session goes on; nothing is retried. The session never looks inside a payload and holds no settings layout.
- A configurator that starts over without exiting (a valid MSP request in four-way mode): the lines exit is done, the lines are kept, the request is answered in the MSP phase, and the next grant waits `park_ms` again. While lines are held, silence ends nothing.
- State lives in the `Passthrough`, not in the future of `run`, so a dropped `run` leaves `finish` and `release` everything they need. A drop during the exit break leaves the lines low; `finish` then waits the whole `break_ms` again before releasing them.
- `SessionConfig` (`#[non_exhaustive]`, `new()` and `with_` setters): `esc_count` (1 to 8, default 4), `line_map`, `park_ms`, `break_ms`, `idle_ms`, the client's `Timeouts`, and the `HostInfo` values of the MSP and housekeeping replies.

## Rules for correct use

1. A parser's request borrows its storage: use the `Request` before the next `push` or `feed`. `feed` returns after the first event with the number of bytes it used; call it again with the rest.
2. Call `reset()` on a request parser when the stream pauses in the middle of a frame (a timeout), or the next frame's bytes finish the old one.
3. The parsers skip bytes outside a frame, so the two request parsers can listen to one stream. A frame is consumed by its declared length: a corrupted length byte can swallow part of the next frame, and the checksum then fails (the configurator retries). A byte that ends a failed frame or breaks a header is looked at again as a frame start: a `$` for MSP, `0x2F` for four-way. A bad-CRC four-way frame whose last byte is `0x2F` therefore starts a new frame at that byte.
4. The four-way parameter count 0 means 256. An empty parameter list does not exist on the wire: send one `0x00`, as the configurator does. `fourway::encode_reply` refuses 0 or more than 256 parameters.
5. `bootloader::parse_read_reply` wants exactly `n + 3` bytes. A refused read is answered with one byte; shorter input is `Truncated` and the caller decides, with its timeout, whether to read that byte with `parse_ack`.
6. A bootloader acknowledge of `C1` or `C2` is a value (`Ack`), not an error, for `parse_ack`; the keep-alive expects `C1`. In replies that should be good (`parse_init_reply`, `parse_read_reply`) any other acknowledge is `ParseError::Rejected`.

7. The bootloader keeps state, and the client closes the hazard it makes. After set buffer the ESC takes the next frame it receives as the payload, until a payload with a good CRC has arrived; if a payload is refused (`C2`) or lost it keeps waiting, and it passes the next valid command as a payload, answers `30` and does not run it, so the following write's set address would be acknowledged and ignored and that write would land at the old address. Therefore `Client` tracks the line (choose it with `Client::select`, not with `line_mut().select()`; lines 0 to 31) and marks it when a `write` fails after set buffer went out and before the payload's `30` came. The next operation on a marked line, or `Client::resync()`, first sends one keep-alive and accepts `30` (the ESC was waiting and took it) or `C1` (it was not); anything else is an error, the mark stays and the operation does not start. `needs_resync()` tells. Writes are 2 to 256 bytes: a single byte is refused with `InvalidLength` before anything is sent, because no supported part programs one byte and a pending one-byte payload is the one state the keep-alive cannot clear. A limit of the protocol that this crate cannot remove: if the set-buffer command is lost on the wire, the bootloader reads the payload that follows as commands (other flight controllers have the same exposure).
8. The bootloader source from v12 on answers the run command with `C1` before it jumps, and other bootloaders may send nothing; `run_application` reads and drops whatever arrives until the line is quiet for `run_us` (a `C1`, or an application that has started) and accepts silence. `Ok` does not say that the ESC left the bootloader; as the bootloader does, it may refuse to jump (the simulator models its checks: the first byte at the settings address must be 1, and the application's first two words must be in range).
9. A write the ESC refuses comes back as `Error::Refused` and is not retried or reshaped: the client passes through what it is given. In the bootloader sources a program is refused (`C1`) below the application, and on every part also for an address or length that is not a multiple of the part's flash word (2 bytes on the F051, 4 on the F421, 8 on the G071, G431 and L431), so a write of 2 or 255 bytes is refused on most parts. What the bootloader does on a write at the settings address longer than 2 bytes: it replaces byte 2 with its own version, so the read-back differs.
10. Every client operation first reads the line with a timeout of 0 until nothing comes, to drop bytes an earlier failed exchange left behind. A late answer that arrives after that is not caught; the session decides how long to let the line settle after a timeout.

## What the crate does

- `msp`: `RequestParser` for MSP v1 requests (`$M<`, length, command, payload, XOR checksum) yields `Request { command, payload }` or `ParseError::{BadHeader, BadChecksum}`. Storage: 255 payload bytes inside the struct. `Request::kind()` sorts the command into the five the configurator needs (1, 2, 130, 131, 245) or `Command::Unsupported(n)`; the crate does not answer an unsupported command. Replies: `encode_api_version`, `encode_fc_variant`, `encode_battery_state` (the 11-byte ArduPilot layout; the voltage in 0.1 V at byte 3 is the one the configurator reads, rounded and saturating at 25.5 V; cell count, capacity and state are 0; current in 0.01 A, charge in mAh, voltage in 0.01 V, all rounded and saturating), `encode_motor_config` (the motor count at byte 6, the other nine bytes 0), `encode_set_passthrough` (the ESC count), `encode_error` (`$M!`, length 0, the command, checksum) and the general `encode_reply`.
- `fourway`: `RequestParser` for `0x2F`, command, address (high first), count (0 is 256), parameters, CRC16-XMODEM (high first); storage: 256 parameter bytes inside the struct. `Command` has every code of ArduPilot's protocol header (0x30 to 0x3F except 0x36, and 0x40) and `Unknown(code)`; `Ack` has the eleven codes of the configurator's table and `Unknown(code)`. `encode_reply` writes `0x2E`, command, address, count, parameters, acknowledge, CRC. `crc16_xmodem` and `crc16_xmodem_update` are public.
- `bootloader`: `INIT` and `encode_init` (21 bytes, no CRC), `encode_set_address`, `encode_set_buffer` (1 to 256, with the high byte 1 for 256), `encode_payload` (data and its CRC, 1 to 256 bytes), `encode_program`, `encode_erase` (`02 01`), `encode_read` (1 to 256, sent as `n`, 256 as 0), `encode_keep_alive`, `encode_run` (`00 00 00 00`), `crc16` (0xA001 reflected, start 0, low byte first). Reply parsers: `parse_ack`, `parse_init_reply` (`DeviceInfo`: pin code, flash size code, two bytes of 0x06, protocol version; `signature()` is the 16-bit value the configurator uses to name the MCU), `parse_read_reply` (data, CRC, acknowledge; the data is returned as a borrowed slice).
- `line`: the trait `EscLines`: `select(line)`, `async write(bytes)` (back to back, returns after the last stop bit), `async read(buf, timeout_us)` (returns at once with at least one byte, or `Ok(0)` after the timeout; the implementer owns the clock), `set_break(on)`. The line does not echo what is written. The trait uses `async fn` in a public trait (the `async_fn_in_trait` lint is allowed in that module, as in the other drivers), which needs no dependency and no executor in this crate.
- `client`: `Client<L: EscLines>` with `connect` (init, 9-byte reply into `DeviceInfo`), `read(address, buf)` and `write(address, data)` (2 to 256 bytes; write is set address, set buffer, wait for the answer that does not come, payload, program), `erase(address)`, `keep_alive` (the `C1` is success), `run_application`. Each is one exchange or a fixed sequence and returns `Error<E>`, which says the `Step` and which of: line error, nothing in time, short answer, bad CRC from the ESC, refused (`C1`, `C2`), unexpected byte; plus `InvalidLength` for a read outside 1 to 256 or a write outside 2 to 256 bytes (nothing is sent) and `InvalidLine` for a line above 31. `select(line)` forwards to the line and remembers it; `resync()` and `needs_resync()` belong to rule 7. `Timeouts` (`#[non_exhaustive]`, `new()` and `with_` setters) holds the waits; the defaults are ArduPilot's: 2 ms for the acknowledge of set address and keep-alive, 5 ms after set buffer, 40 ms for the payload, 500 ms for program, 3 s for erase, and for a multi-byte reply (init, read) the first byte may take 1 ms times the length of the reply and each next byte 1 ms. The client never looks inside data or buffers and has no notion of what an address means; `read` writes the caller's buffer only when the whole reply is good.
- `trigger`: `MspTrigger::feed(&[u8]) -> Option<usize>` recognises exactly `24 4D 3C 00 01 01` (MSP v1 `MSP_API_VERSION`, no payload) in a byte stream, at any split. `Some(n)` says the request completed with the `n`th byte of the slice (the rest is not looked at); a partial match is kept for the next call. It fires once per occurrence and never on a request with one byte changed, another length or another command; it knows nothing else about MSP.
- `host`: `HostSide` owns one MSP and one four-way request parser and the mode. `feed(bytes, &HostInfo, out) -> (used, Option<Event>)` stops at the first event and writes at most one reply, only after the last byte of a complete request. In MSP mode it answers commands 1, 2, 130 and 131 from `HostInfo` (`Event::Reply`; defaults are ArduPilot's), answers any other command and a `MSP_SET_PASSTHROUGH` with a payload with the MSP error reply, ignores malformed frames, and reports a plain `MSP_SET_PASSTHROUGH` as `Event::PassthroughRequested`: the application calls `grant(esc_count, out)` (four-way mode) or `refuse(out)` (error reply, stays MSP); until it does, `feed` consumes nothing and reports the request again. In four-way mode each CRC-valid request is handed out as `Event::FourWay` (command, address, parameters, all opaque) and answered by the application with `encode_success` or `encode_failure` (command and address echoed, as `four_way.ts:400-402` requires), or for protocol version, interface name and interface version by `answer_housekeeping`; a request with a bad CRC is answered by the crate (ACK `03`, the received command and address and first parameter, ArduPilot's failure-reply shape) and not handed out. The application calls `exited()` when `InterfaceExit` is dealt with and the mode is MSP again. A configurator that restarts without exiting: bytes go to the four-way parser first, and only bytes that arrive between four-way frames and are not `0x2F` go to the MSP parser; a complete valid MSP request found that way is `Event::Restarted` (the mode stays four-way until the application calls `exited()`), MSP errors there are ignored, and a `$` inside a four-way frame never reaches the MSP parser. No ESC index check, no line, no timing.
- `session`: `Passthrough<L: EscLines>` (`run`, `finish`, `release`, `holds_lines`), `SessionConfig`, the `Lender` trait, `Exit<E>` (`Idle`, `Exited`, `Closed`, `Host(E)`) and `ReleaseError` (`NotHeld`, `ExitOwed`). See "The session" and "What the firmware must do" above.
- Addresses and payloads are opaque. The crate has no settings layout, no setting names and no magic address list.
- No panic for any input and no allocation. Enums that copy a complete protocol table (`fourway::Command`, `fourway::Ack`, `bootloader::Ack`) are exhaustive; error enums and the structs that hold reply values are `#[non_exhaustive]` (build the structs with `new` and the `with_` setters).

## Not covered

- Anything on a board. The crate has no pins, PIO, USB serial port, motor-signal driver or permission logic: the `EscLines` implementation, the host byte stream, the `Lender` (and so the check that the vehicle is disarmed) and the clock are the firmware's. `hil/` holds one such firmware for one board, as a bench demonstration only (it always lends the lines and carries no other protocol on its port).
- ESCs beyond the one bench: the session's host tests run against the simulated bootloader and a scripted stream whose bytes were laid out from the configurator's source (`four_way.ts`, `SerialDevice.vue`) and ArduPilot's behaviour. On hardware it has met one ESC type, one bootloader version and the configurator as served at am32.ca on one day (see "Measured on a board").
- The firmware's own tap that decides when to start a session: recognising the trigger (`MspTrigger` is here), checking the vehicle is disarmed, handing the bytes read so far to `run` as `preload`, and ending the session when a ground station appears (by dropping `run` and calling `finish`).
- The bootloader's limits (5 bit times between the bytes of a command, 250 us for a command without payload, 20 ms of line low) are the line implementation's business; neither the client nor the test simulator checks them, and the simulator does not model the jump the exit break causes.
- Retries: the configurator retries (init-flash is one connect, not ArduPilot's three); the session retries nothing.
- ArduPilot's `cmd_DeviceVerify`, which the client has no operation for, answers the invalid-command acknowledge; the interface mode requested by `cmd_InterfaceSetMode` is acknowledged and not kept; init-flash always reports interface mode 4 (ArduPilot answers the general error for a part it does not recognise).
- Reading the settings block, the file name or the device-info block that the bootloader serves at its magic addresses: `read` reads any address, it does not interpret what comes back.

## Tests

Host: `cargo test` (integration tests, proptests and the examples in this file, which are doctests). Each encoder and parser has golden vectors whose layout and source lines are cited next to them in `tests/msp.rs`, `tests/fourway.rs` and `tests/bootloader.rs`; the bytes were laid out by hand from the sources (the configurator's `four_way.ts`, `direct.ts` and `commands.queue.ts`, the npm package `@am32/serial-msp`, the bootloader's `main.c`, ArduPilot's behaviour) and their checksums computed with a separate script. The two CRCs are compared with bit-serial reference versions written in the tests and with the published check values for `"123456789"` (CRC16-XMODEM 0x31C3, CRC-16/ARC 0xBB3D). The proptests check that encoding then parsing round-trips (against independent decoders written in the tests), that parsing is the same however the stream is split, that arbitrary bytes never panic and that every frame a parser yields is one an independent whole-buffer parser also finds with a correct checksum, that a corrupted byte anywhere in a valid frame is rejected and the next frame still parses (the XOR checksum and the CRCs catch every single-byte change; a corrupted length byte is checked against the independent parser, because it can by chance make a different, correctly checksummed frame), and the reply parsers' prefixes, trailing bytes and corrupted bytes.

The client is tested against a simulated bootloader in `tests/common/sim.rs` (up to four ESCs, each with a flash image), written from the bootloader's `main.c` and not from this crate's encoders, and checked on its own in `tests/simulator.rs` with hand-worked byte sequences. It models the init match, the device-info reply, set address with the magic addresses, set buffer without answer, the payload with its CRC check (and the wait for another payload after a bad one, which the client's resync answers), program (refused below the application and for a misaligned write on parts that demand it; a plain overwrite, with no page erase), erase, read (the address is forgotten), keep-alive, run (with the jump checks), the three acknowledges, the count of bad commands and the jump after 100. Its fault switches: no answer, a late answer, a truncated answer, a corrupted CRC, a wrong acknowledge, any replacement bytes, answers split into single bytes, a damaged request, an injected line error. Time in it is virtual. The client tests also check the exact bytes and the order and timeouts of the line calls, that they do not depend on the data, that every error variant is produced and the next operation then works, and with proptests that arbitrary bytes from the line never panic or pass for other data. The simulator and the `block_on` that drives the futures are test code only.

The host side is tested in `tests/trigger.rs` and `tests/host.rs`. The trigger: the 6 bytes inserted at random positions into random bytes and into random well-formed frames of another format give exactly one firing per insertion at every split (checked against a naive search), a stream with no insertion fires only where a naive search finds the sequence, every single-byte corruption and every other length or command give none. `HostSide`: the configurator's connect conversation (four MSP requests, passthrough with grant, InitFlash, DeviceRead, InterfaceExit, MSP again) with the replies compared byte for byte against vectors laid out by hand and a separate CRC script, at every split into two and three parts and byte by byte (each event, and so each reply, comes at the last byte of its request); refusal and a second attempt; unknown command; bad four-way CRC answered and not handed out; the restart rule (a valid MSP request between frames is `Restarted`, the same six bytes inside a frame's parameters are not); mode after `exited()`; oracle proptests for both modes at random splits; arbitrary bytes never panic.

The session is tested in `tests/session.rs`, `tests/session_cancel.rs` and `tests/session_props.rs`, with doubles in `tests/common/session_support.rs` (a virtual clock with its own executor, a scripted host stream, a delay on that clock, a lender, the simulator behind a recorder) and the conversation's vectors in `tests/common/session_talk.rs`. The configurator's conversation (the MSP requests, the passthrough request, its 2 s wait, init-flash and the three reads of `getInfo` for each of four controllers, `InterfaceExit`) is replayed with every reply byte compared against literal vectors (checksums computed with a separate script) or frames built from the layouts with the bit-serial CRC; the line log shows nothing on any line before `park_ms` after the grant, traffic only on the selected line (each simulated bootloader receives exactly its own bytes), and one exit break of at least `break_ms` on all lines after the exit reply; `release()` is refused while the exit is owed. Also: a settings-style write of 184 bytes and its read-back; each `Exit` (idle, with the idle time not restarted by noise; interface-exit; read, write and flush errors with and without lines held; end of stream; refusal then a second grant; a restart); each error path (index out of range with no line traffic, a request before any init-flash, a client timeout mid-read, the stuck payload recovered); every command of the table; the counts of invariant 5.2 (no write before a complete request, one reply per request) in every conversation. A cancellation sweep drops `run` at every await of the whole conversation (1322 of them), then `finish()` and `release()` must succeed, the log must show exactly one exit break after the last line traffic, and a fresh session on the returned lines must work. A proptest checks that for random payloads, addresses, counts and flash contents the line operations are the same and the bytes the host receives are exactly the simulator's. The tests were checked against four deliberate faults in the session (no park, no break wait, a second break on resume, a write length that depends on the first payload byte), each of which makes a test fail.

The simulator and every timing here follow the AM32 bootloader source from v12 on. The only real bootloader the crate has met is version 1 (source not available), on the bench described below; what differed there from the simulator is listed in that section.

## Measured on a board

One bench, 2026-11-12. Board: MARV V2 Rev 1 (RP2354B) running `hil/` `passthrough_demo`: a USB serial port (`usb-cdc`), this crate's session, and the four motor lines (GPIO36 to 39) through `pio-uart`'s `OneWire` as `EscLines`. ESCs: one Flywoo GOKU G55M 4-in-1, four controllers with AM32 2.20 (target `FLYWOO_GOKU_F421`, AT32F421, settings layout 3) and bootloader version 1, on a battery, no motors. Session defaults throughout: park 3000 ms, break 25 ms.

- **Entering the bootloader.** With the lines released high and silent the controllers are in their bootloaders when the park ends and answer the init (9 bytes, `34 37 31 14 1f 06 06 01 30`). Measured beforehand with a probe program: 1.5 s of silence is too short for a controller whose application is running and not armed, 2.0 s is enough. A parked bootloader stays parked (60 s tried) and answers a keep-alive afterwards. The 25 ms break starts the application.
- **Read, with the tools in `host/`.** The read-only tool read the 184-byte settings block of all four controllers and dumped 0x1000 to 0x7FFF of all four in one session (457 four-way requests, 117 268 bytes from the lines, no framing error, overrun, underrun or runt start), twice.
- **Read, with the real configurator** (am32.ca/configurator in Chrome over Web Serial; the port's USB id has to be one the configurator lists, 1209:0001 was used): the four MSP answers, the grant, and its Read of all four controllers, several times.
- **Write.** `esc-passthrough-restore` wrote each controller's own settings block back (one `cmd_DeviceWrite` of 184 bytes at 0x7C00, acknowledged in about 122 ms) and read it back identical, on all four; each whole settings page was byte for byte unchanged afterwards. Then the real configurator changed one setting on one controller, read the new value back, and changed it back.
- **Flash.** The real configurator flashed release 2.20 to one controller. By its source that is the settings write with the start flag cleared, 108 writes of 256 bytes from 0x1000, the settings write with the flag set, `cmd_DeviceReset`, and 5 s later a re-read, which needs a second connect to the same controller inside one session; all of it went through. A dump afterwards holds every byte of the release's hex file at its address; the other three controllers and all four settings blocks were unchanged; all four answered DShot afterwards.
- **Different from the simulator on this bootloader:** it erases the flash page itself when a write lands on it (the simulator models a plain overwrite; the configurator never sends a page erase), and it serves plain flash at the small addresses that newer bootloaders treat as special.

Not measured: a serial port that also carries another protocol (the demonstration's port carries nothing else, so the trigger has only been seen at the very start of a stream); any use inside a flight firmware; an interrupted flash; a different firmware or bootloader version, MCU or ESC; the commands the configurator does not send (page erase, verify); controllers with motors connected.

Known and open: one framing error was counted on the line during the flash session (188 requests, none of which failed for the configurator); it was not located. The demonstration's start-up check and idle watch report FAIL when a controller is in the half second after its own start in which it holds its line low; the check is wrong there, not the lines.

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public types.

```text
Cargo.toml   the crate
src/         msp.rs, fourway.rs, bootloader.rs, line.rs, client.rs, trigger.rs, host.rs, session.rs, error.rs (EncodeError and the output writer)
tests/       one file per module (client.rs for line and client, trigger.rs, host.rs), simulator.rs, session.rs, session_cancel.rs, session_props.rs, and common/ with the bit-serial reference CRCs, the simulated bootloader (sim.rs), block_on.rs and the session doubles
hil/         the demonstration for the MARV V2 board (passthrough_demo, passthrough_demo_check), with its own README
host/        the computer-side tools (esc-passthrough-host, esc-passthrough-restore), with their own README
tools/       run.sh: builds, flashes and runs one program of hil/ through the debug probe
```

MIT, see `LICENSE`.
