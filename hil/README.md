# esc-passthrough hil: the passthrough demonstration on the MARV V2 board

**Status: built, started and enumerated on the board. No computer-side tool, and no configurator, has talked to it yet: the session path (trigger, lend, park, four-way requests, exit break) has never run on the board.** What has run: the start-up self-check (below) and the idle service.

`passthrough_demo` is a service. The board enumerates as one USB serial port (CDC-ACM through `usb-cdc`) and stays silent: nothing is ever written to the port before the trigger. When a computer sends exactly the 6 bytes `24 4D 3C 00 01 01` (`MSP_API_VERSION`, what the configurator starts with; recognised by `esc_passthrough::trigger::MspTrigger`), the board serves that conversation with `esc_passthrough::session::Passthrough` on the real motor-controller lines (`pio_uart::OneWire`, PIO2 state machine 0, GPIO36 to 39, 19200 baud). When the session ends it goes back to waiting for the trigger.

It sends **no DShot frame, ever**. Between sessions the four lines are inputs with pad pull-ups, never driven by this program, so the controllers sit in their bootloaders (the inert state). The only traffic it originates on a motor line is what the session does in answer to a host request, and the exit break (all four lines low 25 ms). A host that asks for a write gets a write: that is what a passthrough is, and nothing in this program asks for one.

## Run

```text
tools/run.sh passthrough_demo <seconds>        # the service; runs until the timeout (status 124 is normal)
tools/run.sh passthrough_demo_check 60         # the start-up self-check only; exits 0 on pass
```

`tools/run.sh` builds, takes the machine-wide probe lock, flashes through the probe and keeps the full RTT log under `hil/logs/`. Never call `probe-rs` directly. The board's own USB cable must be connected to the computer (the debug probe's cable is the other one). The serial port appears about 0.35 s after the program starts, as `/dev/serial/by-id/usb-MARV_MARV_V2_ESC_passthrough_demo_passthrough-demo-if00` (`/dev/ttyACM1` on the development computer).

To try a session: start the service with a long timeout (for example `tools/run.sh passthrough_demo 900`), wait for the line `link up; silent until the trigger request`, then open the port from the computer-side tool or configurator. When `probe-rs` is stopped, the program keeps running on the board and the port stays; it is only replaced by the next run.

## Environment variables (build time)

| Variable | Default | Meaning |
|---|---|---|
| `DEMO_USB_VID` | `C0DE` | USB vendor id, 1 to 4 hex digits (a bad value stops the build) |
| `DEMO_USB_PID` | `CAFE` | USB product id, same rule |

For example `DEMO_USB_VID=1209 DEMO_USB_PID=0001 tools/run.sh passthrough_demo 600`. Cargo rebuilds when they change; build again without them to get the defaults back. Strings: manufacturer "MARV", product "MARV V2 ESC passthrough demo", serial number "passthrough-demo". The real configurator only lists certain vendor ids; the owner has not chosen one yet, which is why this is not in the code.

## Arrangement

- `src/bin/passthrough_demo.rs` and `src/bin/passthrough_demo_check.rs` are a few lines each; everything is in `src/demo.rs` (the library `hil_common`).
- Tasks: `usb_task` runs the USB device; `link_task` owns `Tx`, `Rx`, `Link` and the PIO2 parts; `watch_task` only reads pad registers for the first 10 s.
- PIO2 parts: in the `Board` from start-up, then in the link task's lender between sessions. At `MSP_SET_PASSTHROUGH` the lender builds `OneWire` from them (always lends: the demo has no flight state) and gives it to the session as `Line` (`EscLines`, read timeout by `embassy_time::with_timeout`). After `run` returns (any `Exit`; `run` calls `finish()` itself) the task calls `release()`, reads the line statistics, takes the `OneWire` apart and puts the parts back. If the link goes down it waits for it to come up again.
- `SessionConfig`: 4 controllers; `line_map` `[3, 2, 1, 0, 4, 5, 6, 7]` so controller 0 is pad M1 (GPIO39), 1 is M2 (GPIO38), 2 is M3 (GPIO37), 3 is M4 (GPIO36); park 3000 ms, break 25 ms, idle 5000 ms (defaults).
- Battery: a fixed `BatteryState::new(0)` (0 mV). The ADC is not used.
- Trigger bytes: the session gets the 6 request bytes, plus any bytes that came in the same USB packet after them, as `preload`.
- The "four-way requests" figure in the log is counted by a second parser that watches the bytes the session reads (the session keeps no such count); it counts complete CRC-valid requests, so it can differ from what the session answered if the session restarted.

## Self-check (start-up, both programs)

`[PASS]` / `[FAIL]` lines: `clk_sys` is 150 MHz; GPIO36 to 39 read high with pull-ups and none has an output enable (pad output enable, SIO OE and, with `OneWire` built, PIO2 `DBG_PADOE`), before `OneWire` exists, while it is built and after 10 releases; `OneWire` built and released 10 times; the USB port is configured by the computer within 5 s of the pull-up. `passthrough_demo_check` then releases the USB pull-up, prints `RESULT: PASS` (or `FAIL`) and exits with the number of failures. `passthrough_demo` goes on serving whatever the result. In the service's first 10 s `watch_task` also samples the pads every 50 ms (skipping samples while a session holds the lines) and logs one `[PASS]` / `[FAIL] idle watch` line.

## What a session looks like in the log

RTT, at boundaries and errors only, never per byte:

```text
[INFO] link up; silent until the trigger request
[INFO] trigger request seen after <n> bytes read; starting a session
[INFO] session: lines lent (this demo has no flight state and always lends)
   ... (the 3 s park and the conversation: nothing is logged)
[INFO] line stats: sent <n>, received <n>, discarded <n>, framing errors <n>, overruns <n>, underruns <n>, runt starts <n>
[INFO] session ended: <Exit>, four-way requests <n>, lines were taken true
```

`<Exit>` is `Exited` (InterfaceExit), `Idle` (5 s with no valid MSP request before the grant; `lines were taken false`), `Closed`, or `host stream failed (...)` (the port went away). An error line `session: OneWire::new failed, refusing` means the configurator got the MSP error reply.
