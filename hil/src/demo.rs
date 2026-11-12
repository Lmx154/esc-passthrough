//! The demonstration of the ESC passthrough on the MARV V2 board: one USB serial port, the
//! trigger, and the `esc-passthrough` session on the four motor-controller lines (PIO2 state
//! machine 0 on GPIO36 to 39 through `pio_uart::OneWire`, 19200 baud).
//!
//! Ownership: the PIO2 parts live in `Board` until the link task takes them; from then on they
//! are in the `PioLender` between sessions, inside the session's `OneWire` (through `Line`) while
//! one runs, and back in the lender after `finish()` and `release()`. Nothing else touches PIO2.
//! The program never sends a DShot frame and never drives a motor line on its own: between
//! sessions the lines are inputs with pad pull-ups.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_rp::gpio::Pull;
use embassy_rp::interrupt::typelevel::{Binding, PIO2_IRQ_0, USBCTRL_IRQ};
use embassy_rp::pac;
use embassy_rp::peripherals::{PIO2, USB};
use embassy_rp::pio::{self, Pio};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Delay, Duration, Instant, Ticker, with_deadline, with_timeout};
use embassy_usb::{Builder, Config, UsbDevice};
use embedded_io_async::{ErrorType, Read};
use esc_passthrough::fourway;
use esc_passthrough::line::EscLines;
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Lender, Passthrough, ReleaseError, SessionConfig};
use esc_passthrough::trigger::{self, MspTrigger};
use pio_uart::{Config as WireConfig, OneWire, Parts};
use static_cell::StaticCell;
use usb_cdc::{Link, Rx, Tx, UsbCdc};

/// USB identity: build-time environment variables `DEMO_USB_VID` / `DEMO_USB_PID`, hex.
const VID: u16 = hex_env(option_env!("DEMO_USB_VID"), 0xC0DE);
const PID: u16 = hex_env(option_env!("DEMO_USB_PID"), 0xCAFE);

const CLK_HZ: u32 = 150_000_000;
const BAUD: u32 = 19_200;
const LINES: usize = 4;
/// GPIO number of the first motor line; line `i` of the `OneWire` is GPIO `FIRST_GPIO + i`.
const FIRST_GPIO: usize = 36;
const USB_LIMIT: Duration = Duration::from_secs(5);
const WATCH: Duration = Duration::from_secs(10);

type Drv = Driver<'static, USB>;
type P = Parts<'static, PIO2, 0, 4>;
type Wire = OneWire<'static, PIO2, 0, 4>;

/// True from the moment a session takes the lines until they are back in the lender.
static SESSION: AtomicBool = AtomicBool::new(false);

const fn hex_env(value: Option<&str>, default: u16) -> u16 {
    let Some(text) = value else {
        return default;
    };
    let bytes = text.as_bytes();
    assert!(
        !bytes.is_empty() && bytes.len() <= 4,
        "DEMO_USB_VID / DEMO_USB_PID: 1 to 4 hex digits"
    );
    let mut out: u16 = 0;
    let mut i = 0;
    while i < bytes.len() {
        let digit = match bytes[i] {
            b @ b'0'..=b'9' => b - b'0',
            b @ b'a'..=b'f' => b - b'a' + 10,
            b @ b'A'..=b'F' => b - b'A' + 10,
            _ => panic!("DEMO_USB_VID / DEMO_USB_PID: hex digits only"),
        };
        out = out << 4 | digit as u16;
        i += 1;
    }
    out
}

macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

/// Everything `setup` makes: the USB halves and the PIO2 parts, with the USB device running.
pub struct Board {
    tx: Tx<'static, Drv>,
    rx: Rx<'static, Drv>,
    link: Link<'static>,
    parts: Option<P>,
    started: Instant,
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, Drv>) -> ! {
    device.run().await
}

/// Builds the USB device (one CDC-ACM port) and starts its task; makes the four motor pins PIO2
/// pins with the pad pull-up on and nothing driven.
pub fn setup<I>(spawner: &Spawner, p: embassy_rp::Peripherals, irqs: I) -> Board
where
    I: Binding<USBCTRL_IRQ, InterruptHandler<USB>>
        + Binding<PIO2_IRQ_0, pio::InterruptHandler<PIO2>>
        + Copy,
{
    let mut config = Config::new(VID, PID);
    config.manufacturer = Some("MARV");
    config.product = Some("MARV V2 ESC passthrough demo");
    config.serial_number = Some("passthrough-demo");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO2, irqs);
    let mut pins = [
        common.make_pio_pin(p.PIN_36),
        common.make_pio_pin(p.PIN_37),
        common.make_pio_pin(p.PIN_38),
        common.make_pio_pin(p.PIN_39),
    ];
    for pin in &mut pins {
        pin.set_pull(Pull::Up);
    }
    let parts: P = Parts {
        common,
        sm: sm0,
        pins,
    };

    let driver = Driver::new(p.USB, irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (tx, rx, link) = cdc.split();
    let device = builder.build();
    let started = Instant::now();
    match usb_task(device) {
        Ok(token) => spawner.spawn(token),
        Err(_) => defmt::error!("[FAIL] the USB task could not be spawned"),
    }
    defmt::info!(
        "[INFO] usb identity: vid 0x{=u16:04x}, pid 0x{=u16:04x}, \"MARV\", \"MARV V2 ESC passthrough demo\"",
        VID,
        PID
    );
    Board {
        tx,
        rx,
        link,
        parts: Some(parts),
        started,
    }
}

fn wire_config() -> WireConfig {
    WireConfig::new().with_baud(BAUD).with_clk_sys_hz(CLK_HZ)
}

/// What the four motor pads are doing, read from the chip.
struct Pads {
    /// Level of GPIO36 to 39 (bit i is line i).
    high: u8,
    /// Output enable to the pad, whatever drives it.
    oe_pad: u8,
    /// PIO2 `DBG_PADOE` for the four pads; 0 and `window == false` if PIO2 cannot reach them.
    padoe: u8,
    /// SIO output enable.
    sio_oe: u8,
    /// PIO2 GPIOBASE selects pins 16 to 47 (the only window that reaches GPIO36 to 39).
    window: bool,
}

impl Pads {
    fn read() -> Pads {
        let mut oe_pad = 0u8;
        for i in 0..LINES {
            let status = pac::IO_BANK0.gpio(FIRST_GPIO + i).status().read();
            oe_pad |= u8::from(status.oetopad()) << i;
        }
        let window = pac::PIO2.gpiobase().read().gpiobase();
        let padoe = if window {
            ((pac::PIO2.dbg_padoe().read() >> (FIRST_GPIO - 16)) & 0xF) as u8
        } else {
            0
        };
        Pads {
            high: ((pac::SIO.gpio_in(1).read() >> (FIRST_GPIO - 32)) & 0xF) as u8,
            oe_pad,
            padoe,
            sio_oe: ((pac::SIO.gpio_oe(1).value().read() >> (FIRST_GPIO - 32)) & 0xF) as u8,
            window,
        }
    }

    fn undriven(&self) -> bool {
        self.oe_pad == 0 && self.padoe == 0 && self.sio_oe == 0
    }

    fn all_high(&self) -> bool {
        self.high == 0xF
    }
}

/// The start-up self-check; returns the number of failed checks. Uses the PIO2 parts and the USB
/// link, and leaves both as it found them.
pub async fn self_check(board: &mut Board) -> u32 {
    let mut failures = 0u32;
    let hz = embassy_rp::clocks::clk_sys_freq();
    check!(
        failures,
        hz == CLK_HZ,
        "{=str} clk_sys is {=u32} Hz (want 150000000)",
        hz
    );

    let Some(mut parts) = board.parts.take() else {
        check!(failures, false, "{=str} the PIO2 parts are not at hand");
        return failures;
    };

    let pads = Pads::read();
    check!(
        failures,
        pads.all_high() && pads.undriven(),
        "{=str} before OneWire: GPIO36..39 levels 0b{=u8:04b} (want 1111) pull-ups on, output enable to pad 0b{=u8:04b}, SIO OE 0b{=u8:04b} (want 0)",
        pads.high,
        pads.oe_pad,
        pads.sio_oe
    );

    let mut built = 0u32;
    let mut released = 0u32;
    let mut held = Pads::read();
    for i in 0..10 {
        match OneWire::new(parts.common, parts.sm, parts.pins, wire_config()) {
            Ok(ow) => {
                built += 1;
                if i == 0 {
                    held = Pads::read();
                }
                parts = ow.release();
                released += 1;
            }
            Err(e) => {
                defmt::error!("[INFO] OneWire::new failed: {}", e.reason);
                parts = e.parts;
                break;
            }
        }
    }
    check!(
        failures,
        built == 10 && released == 10,
        "{=str} OneWire built and released {=u32} / {=u32} times (want 10 / 10)",
        built,
        released
    );
    check!(
        failures,
        held.window && held.all_high() && held.undriven(),
        "{=str} OneWire built: PIO2 window GPIO16+ {=bool}, levels 0b{=u8:04b} (want 1111), DBG_PADOE 0b{=u8:04b}, output enable to pad 0b{=u8:04b}, SIO OE 0b{=u8:04b} (want 0)",
        held.window,
        held.high,
        held.padoe,
        held.oe_pad,
        held.sio_oe
    );
    let after = Pads::read();
    check!(
        failures,
        after.window && after.all_high() && after.undriven(),
        "{=str} after the 10 releases: levels 0b{=u8:04b} (want 1111), DBG_PADOE 0b{=u8:04b}, output enable to pad 0b{=u8:04b}, SIO OE 0b{=u8:04b} (want 0)",
        after.high,
        after.padoe,
        after.oe_pad,
        after.sio_oe
    );
    board.parts = Some(parts);

    let up = with_deadline(board.started + USB_LIMIT, board.link.wait_up())
        .await
        .is_ok();
    let up_ms = (Instant::now() - board.started).as_millis() as u32;
    check!(
        failures,
        up,
        "{=str} the USB port is configured by the computer within 5000 ms of the pull-up (checked at {=u32} ms)",
        up_ms
    );
    failures
}

/// Starts the link task and the idle watch. Call once, after `self_check` if wanted.
pub fn serve(spawner: &Spawner, board: Board) {
    let Board {
        tx,
        rx,
        link,
        parts,
        ..
    } = board;
    let Some(parts) = parts else {
        defmt::error!("[FAIL] the PIO2 parts are not at hand; nothing started");
        return;
    };
    match link_task(tx, rx, link, parts) {
        Ok(token) => spawner.spawn(token),
        Err(_) => defmt::error!("[FAIL] the link task could not be spawned"),
    }
    match watch_task() {
        Ok(token) => spawner.spawn(token),
        Err(_) => defmt::error!("[FAIL] the watch task could not be spawned"),
    }
}

/// The ESC lines for the session: `OneWire` as `EscLines`, the read timeout by `embassy_time`.
pub struct Line {
    ow: Wire,
}

impl EscLines for Line {
    type Error = pio_uart::Error;

    fn select(&mut self, line: u8) -> Result<(), Self::Error> {
        self.ow.select(usize::from(line))
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_io_async::Write::write_all(&mut self.ow, bytes).await
    }

    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, Self::Error> {
        match with_timeout(
            Duration::from_micros(u64::from(timeout_us)),
            Read::read(&mut self.ow, buf),
        )
        .await
        {
            Ok(result) => result,
            Err(_) => Ok(0),
        }
    }

    fn set_break(&mut self, on: bool) -> Result<(), Self::Error> {
        if on {
            self.ow.break_start()
        } else {
            self.ow.break_end();
            Ok(())
        }
    }
}

/// Holds the PIO2 parts between sessions. This demo has no flight state, so it always lends.
struct PioLender {
    parts: Option<P>,
}

impl Lender for PioLender {
    type Lines = Line;

    async fn lend(&mut self) -> Option<Line> {
        let parts = self.parts.take()?;
        match OneWire::new(parts.common, parts.sm, parts.pins, wire_config()) {
            Ok(ow) => {
                SESSION.store(true, Ordering::Relaxed);
                defmt::info!(
                    "[INFO] session: lines lent (this demo has no flight state and always lends)"
                );
                Some(Line { ow })
            }
            Err(e) => {
                defmt::error!(
                    "[INFO] session: OneWire::new failed, refusing: {}",
                    e.reason
                );
                self.parts = Some(e.parts);
                None
            }
        }
    }
}

/// The reader the session gets: passes every read through and counts, with a parser of its own,
/// the four-way requests that go by (the session keeps no such count).
struct Counted<'a, R> {
    inner: &'a mut R,
    parser: fourway::RequestParser,
    requests: u32,
}

impl<R: Read> ErrorType for Counted<'_, R> {
    type Error = R::Error;
}

impl<R: Read> Read for Counted<'_, R> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.inner.read(buf).await?;
        let mut rest = buf.get(..n).unwrap_or_default();
        while !rest.is_empty() {
            let (used, event) = self.parser.feed(rest);
            if matches!(event, Some(Ok(_))) {
                self.requests += 1;
            }
            rest = rest.get(used.max(1)..).unwrap_or_default();
        }
        Ok(n)
    }
}

fn session_config() -> SessionConfig {
    // Controller 0 is pad M1 (GPIO39 = line 3), 1 is M2 (GPIO38), 2 is M3 (GPIO37), 3 is M4
    // (GPIO36). The four entries after those are not reachable with 4 controllers.
    SessionConfig::new()
        .with_esc_count(4)
        .with_line_map([3, 2, 1, 0, 4, 5, 6, 7])
}

#[embassy_executor::task]
async fn link_task(
    mut tx: Tx<'static, Drv>,
    mut rx: Rx<'static, Drv>,
    link: Link<'static>,
    parts: P,
) -> ! {
    let mut lender = PioLender { parts: Some(parts) };
    let mut session: Passthrough<Line> = Passthrough::new(session_config());
    let mut delay = Delay;
    defmt::info!(
        "[INFO] link task: waiting for the computer; the battery figure is a fixed 0 mV (no ADC in this demo)"
    );
    loop {
        link.wait_up().await;
        defmt::info!("[INFO] link up; silent until the trigger request");

        let mut trigger = MspTrigger::new();
        let mut buf = [0u8; 64];
        let mut preload = [0u8; trigger::REQUEST.len() + 64];
        let mut seen = 0u32;
        let mut preload_len = 0usize;
        loop {
            let Ok(n) = rx.read(&mut buf).await else {
                break;
            };
            let chunk = buf.get(..n).unwrap_or_default();
            if let Some(used) = trigger.feed(chunk) {
                let tail = chunk.get(used..).unwrap_or_default();
                preload[..trigger::REQUEST.len()].copy_from_slice(&trigger::REQUEST);
                preload[trigger::REQUEST.len()..trigger::REQUEST.len() + tail.len()]
                    .copy_from_slice(tail);
                preload_len = trigger::REQUEST.len() + tail.len();
                seen += used as u32;
                break;
            }
            seen += n as u32;
        }
        if preload_len == 0 {
            defmt::info!(
                "[INFO] link down before the trigger request ({=u32} bytes read)",
                seen
            );
            continue;
        }
        defmt::info!(
            "[INFO] trigger request seen after {=u32} bytes read; starting a session",
            seen.saturating_sub(trigger::REQUEST.len() as u32)
        );

        let mut counted = Counted {
            inner: &mut rx,
            parser: fourway::RequestParser::new(),
            requests: 0,
        };
        let exit = session
            .run(
                preload.get(..preload_len).unwrap_or_default(),
                &mut counted,
                &mut tx,
                &mut lender,
                &mut delay,
                || BatteryState::new(0),
            )
            .await;
        let requests = counted.requests;

        let mut lines_held = true;
        loop {
            match session.release() {
                Ok(line) => {
                    let s = line.ow.stats();
                    defmt::info!(
                        "[INFO] line stats: sent {=u32}, received {=u32}, discarded {=u32}, framing errors {=u32}, overruns {=u32}, underruns {=u32}, runt starts {=u32}",
                        s.bytes_sent,
                        s.bytes_received,
                        s.bytes_discarded,
                        s.framing_errors,
                        s.overruns,
                        s.underruns,
                        s.runt_starts
                    );
                    lender.parts = Some(line.ow.release());
                    SESSION.store(false, Ordering::Relaxed);
                    break;
                }
                Err(ReleaseError::NotHeld) => {
                    lines_held = false;
                    break;
                }
                Err(_) => session.finish(&mut delay).await,
            }
        }
        match &exit {
            Exit::Host(e) => defmt::info!(
                "[INFO] session ended: host stream failed ({}), four-way requests {=u32}, lines were taken {=bool}",
                e,
                requests,
                lines_held
            ),
            other => defmt::info!(
                "[INFO] session ended: {}, four-way requests {=u32}, lines were taken {=bool}",
                other,
                requests,
                lines_held
            ),
        }
    }
}

/// For the first 10 s after the start: checks that no motor line is driven and all read high,
/// except while a session holds the lines. Logs once.
#[embassy_executor::task]
async fn watch_task() {
    let end = Instant::now() + WATCH;
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let (mut samples, mut skipped, mut driven, mut low) = (0u32, 0u32, 0u32, 0u32);
    while Instant::now() < end {
        ticker.next().await;
        if SESSION.load(Ordering::Relaxed) {
            skipped += 1;
            continue;
        }
        let pads = Pads::read();
        samples += 1;
        driven += u32::from(!pads.undriven());
        low += u32::from(!pads.all_high());
    }
    if driven == 0 && low == 0 && samples > 0 {
        defmt::info!(
            "[PASS] idle watch, first 10 s: {=u32} samples every 50 ms ({=u32} skipped while a session held the lines): driven {=u32}, not high {=u32}",
            samples,
            skipped,
            driven,
            low
        );
    } else {
        defmt::error!(
            "[FAIL] idle watch, first 10 s: {=u32} samples every 50 ms ({=u32} skipped while a session held the lines): driven {=u32}, not high {=u32}",
            samples,
            skipped,
            driven,
            low
        );
    }
}

/// Releases the D+ pull-up so the computer sees a clean disconnect (the driver has no `Drop`).
pub fn detach_usb() {
    pac::USB.sie_ctrl().modify(|w| w.set_pullup_en(false));
}
