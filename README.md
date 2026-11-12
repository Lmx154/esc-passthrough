# esc-passthrough

Sans-IO codecs for letting the AM32 configurator on a computer reach ESCs through a flight controller, `no_std`, no allocation. The crate holds the three wire formats the flight controller needs: the MSP v1 messages the configurator opens with, the "four-way interface" it speaks afterwards, and the byte sequences of the AM32 bootloader on the ESC's signal wire. Bytes in, typed values out, and back. It does no I/O, owns no timer, keeps no session and names no transport, HAL or board. It has no dependencies; `defmt` 1.0 is optional. It is written for the MARV V2 flight controller but takes everything from the caller.

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

## What the application provides

- The bytes: the host side's serial stream goes into the two request parsers; the ESC's bytes go into the reply parsers, already read from the wire. The crate reads and writes nothing itself.
- The values for the MSP replies: API version triple, the four ASCII bytes of the flight controller variant (the configurator knows `BTFL`, `KISS`, `INAV`, `ARDU` and `EMUF`), battery voltage in millivolts with optional current and charge, the number of motors, the number of ESCs.
- Output buffers. Every encoder takes `&mut [u8]`, returns the length written, and returns `EncodeError::BufferTooSmall { needed }` without writing if it does not fit. `msp::MAX_FRAME_LEN`, `fourway::MAX_REPLY_LEN` and `bootloader::MAX_DATA + 2` are the largest sizes.
- Timing, ordering and sessions: which four-way command becomes which bootloader sequence, how long to wait for each reply, when to retry, when to hold the line.

## Rules for correct use

1. A parser's request borrows its storage: use the `Request` before the next `push` or `feed`. `feed` returns after the first event with the number of bytes it used; call it again with the rest.
2. Call `reset()` on a request parser when the stream pauses in the middle of a frame (a timeout), or the next frame's bytes finish the old one.
3. The parsers skip bytes outside a frame, so the two request parsers can listen to one stream. A frame is consumed by its declared length: a corrupted length byte can swallow part of the next frame, and the checksum then fails (the configurator retries). A byte that ends a failed frame or breaks a header is looked at again as a frame start: a `$` for MSP, `0x2F` for four-way. A bad-CRC four-way frame whose last byte is `0x2F` therefore starts a new frame at that byte.
4. The four-way parameter count 0 means 256. An empty parameter list does not exist on the wire: send one `0x00`, as the configurator does. `fourway::encode_reply` refuses 0 or more than 256 parameters.
5. `bootloader::parse_read_reply` wants exactly `n + 3` bytes. A refused read is answered with one byte; shorter input is `Truncated` and the caller decides, with its timeout, whether to read that byte with `parse_ack`.
6. A bootloader acknowledge of `C1` or `C2` is a value (`Ack`), not an error, for `parse_ack`; the keep-alive expects `C1`. In replies that should be good (`parse_init_reply`, `parse_read_reply`) any other acknowledge is `ParseError::Rejected`.

## What the crate does

- `msp`: `RequestParser` for MSP v1 requests (`$M<`, length, command, payload, XOR checksum) yields `Request { command, payload }` or `ParseError::{BadHeader, BadChecksum}`. Storage: 255 payload bytes inside the struct. `Request::kind()` sorts the command into the five the configurator needs (1, 2, 130, 131, 245) or `Command::Unsupported(n)`; the crate does not answer an unsupported command. Replies: `encode_api_version`, `encode_fc_variant`, `encode_battery_state` (the 11-byte ArduPilot layout; the voltage in 0.1 V at byte 3 is the one the configurator reads, rounded and saturating at 25.5 V; cell count, capacity and state are 0; current in 0.01 A, charge in mAh, voltage in 0.01 V, all rounded and saturating), `encode_motor_config` (the motor count at byte 6, the other nine bytes 0), `encode_set_passthrough` (the ESC count), `encode_error` (`$M!`, length 0, the command, checksum) and the general `encode_reply`.
- `fourway`: `RequestParser` for `0x2F`, command, address (high first), count (0 is 256), parameters, CRC16-XMODEM (high first); storage: 256 parameter bytes inside the struct. `Command` has every code of ArduPilot's protocol header (0x30 to 0x3F except 0x36, and 0x40) and `Unknown(code)`; `Ack` has the eleven codes of the configurator's table and `Unknown(code)`. `encode_reply` writes `0x2E`, command, address, count, parameters, acknowledge, CRC. `crc16_xmodem` and `crc16_xmodem_update` are public.
- `bootloader`: `INIT` and `encode_init` (21 bytes, no CRC), `encode_set_address`, `encode_set_buffer` (1 to 256, with the high byte 1 for 256), `encode_payload` (data and its CRC, 1 to 256 bytes), `encode_program`, `encode_read` (1 to 256, sent as `n`, 256 as 0), `encode_keep_alive`, `encode_run` (`00 00 00 00`), `crc16` (0xA001 reflected, start 0, low byte first). Reply parsers: `parse_ack`, `parse_init_reply` (`DeviceInfo`: pin code, flash size code, two bytes of 0x06, protocol version; `signature()` is the 16-bit value the configurator uses to name the MCU), `parse_read_reply` (data, CRC, acknowledge; the data is returned as a borrowed slice).
- Addresses and payloads are opaque. The crate has no settings layout, no setting names and no magic address list.
- No panic for any input and no allocation. Enums that copy a complete protocol table (`fourway::Command`, `fourway::Ack`, `bootloader::Ack`) are exhaustive; error enums and the structs that hold reply values are `#[non_exhaustive]` (build the structs with `new` and the `with_` setters).

## Not covered

- The session logic that turns four-way commands into bootloader traffic, with its retries and waits (InitFlash trying the bootloader up to three times, DeviceWrite as set address, set buffer, payload, program, and so on).
- The async runner over `embedded-io-async`, and the ESC-side line trait (select a line, read and write bytes, break all lines).
- Entering and leaving the bootloader: holding the line high, the 2 s wait for the ESC to reset, the 20 ms break that sends it back to its application.
- Anything on a board: pins, PIO, the USB serial port, stopping the motor signal, the permission to take the lines.
- Timing of any kind, including the bootloader's 5-bit-time limit between bytes.
- The bootloader's page erase (`02 01`) and the four-way commands for devices other than the AM32 bootloader: the codes parse and encode, nothing answers them.
- The opposite directions, for a computer-side tool: encoders for MSP and four-way requests, parsers for MSP and four-way replies, an encoder for the ESC's bootloader replies. Also MSP v2, MSP messages other than the five, and the configurator's "direct" route (one ESC, 19200 baud, its own bytes echoed).
- Reading the settings block, the file name or the device-info block that the bootloader serves at its magic addresses.

## Tests

Host: `cargo test` (integration tests, proptests and the examples in this file, which are doctests). Each encoder and parser has golden vectors whose layout and source lines are cited next to them in `tests/msp.rs`, `tests/fourway.rs` and `tests/bootloader.rs`; the bytes were laid out by hand from the sources (the configurator's `four_way.ts`, `direct.ts` and `commands.queue.ts`, the npm package `@am32/serial-msp`, the bootloader's `main.c`, ArduPilot's behaviour) and their checksums computed with a separate script. The two CRCs are compared with bit-serial reference versions written in the tests and with the published check values for `"123456789"` (CRC16-XMODEM 0x31C3, CRC-16/ARC 0xBB3D). The proptests check that encoding then parsing round-trips (against independent decoders written in the tests), that parsing is the same however the stream is split, that arbitrary bytes never panic and that every frame a parser yields is one an independent whole-buffer parser also finds with a correct checksum, that a corrupted byte anywhere in a valid frame is rejected and the next frame still parses (the XOR checksum and the CRCs catch every single-byte change; a corrupted length byte is checked against the independent parser, because it can by chance make a different, correctly checksummed frame), and the reply parsers' prefixes, trailing bytes and corrupted bytes.

## Measured on a board

Nothing. The crate has not been run on a board, against a real ESC or against the real configurator.

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public types.

```text
Cargo.toml   the crate
src/         msp.rs, fourway.rs, bootloader.rs, error.rs (EncodeError and the output writer)
tests/       one file per module, and common/ with the bit-serial reference CRCs
```

MIT, see `LICENSE`.
