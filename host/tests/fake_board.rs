//! The tool's library against a FAKE BOARD in the same process.
//!
//! Wiring: two in-memory byte pipes join the tool (this thread, as a [`Link`]) to a second thread
//! that runs `esc_passthrough::session::Passthrough::run` (the crate under test: the flight
//! controller side) on a tiny executor. The lender hands the session the crate's simulated
//! bootloaders (`tests/common/sim.rs` of the crate, included by path, not copied) as the
//! controllers' lines. The fake's delay returns at once for the zero-length park and break; it
//! never returns for the long idle wait, which therefore never fires.
//!
//! No serial port is opened anywhere in this file.

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Lender, Passthrough, SessionConfig};
use esc_passthrough_host::link::Link;
use esc_passthrough_host::tool::{self, Comparison, Options, Report, Timing};
use std::collections::VecDeque;
use std::future::Future;
use std::io;
use std::pin::pin;
use std::sync::{Arc, Condvar, Mutex};
use std::task::{Context, Poll, Wake, Waker};
use std::thread::{self, JoinHandle};
use std::time::Duration;

#[path = "../../tests/common/sim.rs"]
#[allow(clippy::all)]
mod sim;

use sim::{EscModel, Fault, Sim, Wire};

const BLOCK_AT: usize = 0x7C00;
const BLOCK_LEN: usize = 184;

// ---------------------------------------------------------------- the wire

#[derive(Default)]
struct State {
    bytes: VecDeque<u8>,
    closed: bool,
    waker: Option<Waker>,
}

#[derive(Default)]
struct Pipe {
    state: Mutex<State>,
    ready: Condvar,
}

impl Pipe {
    fn push(&self, bytes: &[u8]) {
        let mut state = self.state.lock().unwrap();
        state.bytes.extend(bytes);
        if let Some(waker) = state.waker.take() {
            waker.wake();
        }
        self.ready.notify_all();
    }

    fn close(&self) {
        let mut state = self.state.lock().unwrap();
        state.closed = true;
        if let Some(waker) = state.waker.take() {
            waker.wake();
        }
        self.ready.notify_all();
    }
}

/// The tool's end: writes go to the board, reads wait on the pipe with a time limit.
struct ToolEnd {
    to_board: Arc<Pipe>,
    from_board: Arc<Pipe>,
    sent: Vec<Vec<u8>>,
}

impl Link for ToolEnd {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.sent.push(bytes.to_vec());
        self.to_board.push(bytes);
        Ok(())
    }

    fn read_some(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let mut state = self.from_board.state.lock().unwrap();
        let deadline = std::time::Instant::now() + timeout;
        while state.bytes.is_empty() {
            if state.closed {
                return Ok(0);
            }
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                return Err(io::ErrorKind::TimedOut.into());
            }
            state = self.from_board.ready.wait_timeout(state, left).unwrap().0;
        }
        let n = buf.len().min(state.bytes.len());
        for slot in buf.iter_mut().take(n) {
            *slot = state.bytes.pop_front().unwrap_or_default();
        }
        Ok(n)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.from_board.state.lock().unwrap().bytes.clear();
        Ok(())
    }
}

impl Drop for ToolEnd {
    fn drop(&mut self) {
        self.to_board.close();
    }
}

/// The board's receiving half: cancel-safe (a pending read has taken nothing).
struct BoardRx(Arc<Pipe>);
/// The board's sending half.
struct BoardTx(Arc<Pipe>);

impl Drop for BoardTx {
    fn drop(&mut self) {
        self.0.close();
    }
}

impl ErrorType for BoardRx {
    type Error = ErrorKind;
}
impl ErrorType for BoardTx {
    type Error = ErrorKind;
}

impl Read for BoardRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        std::future::poll_fn(|cx| {
            let mut state = self.0.state.lock().unwrap();
            if !state.bytes.is_empty() {
                let n = buf.len().min(state.bytes.len());
                for slot in buf.iter_mut().take(n) {
                    *slot = state.bytes.pop_front().unwrap_or_default();
                }
                Poll::Ready(Ok(n))
            } else if state.closed {
                Poll::Ready(Ok(0))
            } else {
                state.waker = Some(cx.waker().clone());
                Poll::Pending
            }
        })
        .await
    }
}

impl Write for BoardTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        self.0.push(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

// ---------------------------------------------------------------- the board's thread

struct ThreadWaker(thread::Thread);

impl Wake for ThreadWaker {
    fn wake(self: Arc<Self>) {
        self.0.unpark();
    }
}

/// Runs `future` on this thread, parking while it is pending. Gives up after 60 s without end.
fn block_on<F: Future>(future: F) -> F::Output {
    let waker = Waker::from(Arc::new(ThreadWaker(thread::current())));
    let mut cx = Context::from_waker(&waker);
    let mut future = pin!(future);
    let started = std::time::Instant::now();
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
        assert!(
            started.elapsed() < Duration::from_secs(60),
            "the fake board hangs"
        );
        thread::park_timeout(Duration::from_millis(50));
    }
}

/// Returns at once for a zero delay and never for any other.
struct FakeDelay;

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        if ns > 0 {
            std::future::pending::<()>().await;
        }
    }
}

struct FakeLender {
    sim: Option<Sim>,
    grant: bool,
}

impl Lender for FakeLender {
    type Lines = Sim;

    async fn lend(&mut self) -> Option<Sim> {
        if self.grant { self.sim.take() } else { None }
    }
}

struct Finished {
    exit: Exit<ErrorKind>,
    sim: Sim,
}

struct Board {
    handle: JoinHandle<Finished>,
    tool: ToolEnd,
}

/// Starts the board with `sim` as its controllers' lines and returns the tool's end of the wire.
fn start_board(sim: Sim, grant: bool, esc_count: u8) -> Board {
    let to_board = Arc::new(Pipe::default());
    let from_board = Arc::new(Pipe::default());
    let (rx, tx) = (BoardRx(to_board.clone()), BoardTx(from_board.clone()));
    let handle = thread::spawn(move || {
        let (mut rx, mut tx) = (rx, tx);
        let config = SessionConfig::new()
            .with_esc_count(esc_count)
            .with_park_ms(0)
            .with_break_ms(0)
            .with_idle_ms(60_000);
        let mut session = Passthrough::<Sim>::new(config);
        let mut lender = FakeLender {
            sim: Some(sim),
            grant,
        };
        let exit =
            block_on(
                session.run(&[], &mut rx, &mut tx, &mut lender, &mut FakeDelay, || {
                    BatteryState::new(25_200)
                }),
            );
        let sim = session
            .release()
            .ok()
            .or(lender.sim.take())
            .expect("the sim is back");
        Finished { exit, sim }
    });
    Board {
        handle,
        tool: ToolEnd {
            to_board,
            from_board,
            sent: Vec::new(),
        },
    }
}

fn four_controllers() -> Sim {
    let mut sim = Sim::four();
    for line in 0..4 {
        let flash = sim.esc_mut(line).flash_mut();
        for (i, byte) in flash[BLOCK_AT..BLOCK_AT + BLOCK_LEN].iter_mut().enumerate() {
            *byte = match i {
                0 => 1,
                i => (i as u8).wrapping_mul(7).wrapping_add(line as u8 * 31 + 5),
            };
        }
    }
    sim
}

fn block_of(sim: &Sim, line: usize) -> Vec<u8> {
    sim.esc(line).flash()[BLOCK_AT..BLOCK_AT + BLOCK_LEN].to_vec()
}

fn quick() -> Timing {
    Timing {
        after_passthrough: Duration::ZERO,
        retry_delay: Duration::ZERO,
        ..Timing::configurator()
    }
}

fn options() -> Options {
    Options {
        timing: quick(),
        ..Options::new()
    }
}

/// Runs the tool against the board, then lets the tool's end go and collects the board.
fn session(sim: Sim, grant: bool, options: &Options) -> (Report, String, Vec<Vec<u8>>, Finished) {
    let Board { handle, mut tool } = start_board(sim, grant, 4);
    let mut out = Vec::new();
    let report = tool::run(&mut tool, options, &mut out);
    let sent = std::mem::take(&mut tool.sent);
    drop(tool);
    let finished = handle.join().expect("the board thread ended");
    (report, String::from_utf8(out).unwrap(), sent, finished)
}

fn flashes(sim: &Sim) -> Vec<Vec<u8>> {
    (0..4).map(|line| sim.esc(line).flash().to_vec()).collect()
}

// ---------------------------------------------------------------- the proof

#[test]
fn the_report_equals_what_the_simulator_holds_and_nothing_was_written() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let expected: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    assert!(
        expected.windows(2).all(|pair| pair[0] != pair[1]),
        "four different blocks"
    );

    let (report, text, sent, finished) = session(sim, true, &options());

    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(finished.exit, Exit::Exited);
    assert_eq!(report.api_version, Some([0, 1, 42]));
    assert_eq!(report.variant.as_deref(), Some("ARDU"));
    assert_eq!(report.battery_decivolts, Some(252));
    assert_eq!(report.motor_count, Some(4));
    assert_eq!(report.passthrough, tool::Passthrough::Granted(4));
    assert_eq!(report.escs.len(), 4);
    for (line, esc) in report.escs.iter().enumerate() {
        assert_eq!(esc.signature, Some(0x1F06), "{text}");
        assert_eq!(esc.pin_code, Some(0x10 + line as u8));
        assert_eq!(esc.interface_mode, Some(4));
        assert_eq!(
            esc.block.as_deref(),
            Some(&expected[line][..]),
            "controller {line}"
        );
        assert_eq!(esc.error, None);
    }
    assert!(report.exit_sent && report.exit_answered);
    assert!(text.contains("controller 4:"), "{text}");
    assert!(
        text.contains("0000  01 "),
        "the block is printed as hex\n{text}"
    );
    assert!(text.contains(" ms (try 1 of 10)"), "{text}");

    // The simulator's flash is byte-identical before and after: nothing was written.
    assert_eq!(flashes(&finished.sim), before);

    // The tool's own requests: only MSP, init-flash, read and exit.
    let commands: Vec<u8> = sent
        .iter()
        .map(|frame| if frame[0] == b'$' { frame[4] } else { frame[1] })
        .collect();
    assert_eq!(
        commands,
        [
            1, 2, 130, 131, 245, 0x37, 0x3A, 0x37, 0x3A, 0x37, 0x3A, 0x37, 0x3A, 0x34
        ]
    );

    // What reached the controllers' lines: the 21-byte init, set-address (FF) and read (03);
    // no set-buffer (FE), program (01), erase (02) or run (00 00 00 00).
    for line in 0..4 {
        let bursts = finished.sim.written(line);
        assert_eq!(bursts.len(), 3, "init, set address, read on line {line}");
        assert_eq!(bursts[0].len(), 21);
        assert_eq!(bursts[1][0], 0xFF);
        assert_eq!(bursts[2][0], 0x03);
        assert!(!finished.sim.esc(line).running());
    }
}

#[test]
fn the_line_log_shows_the_exit_break_after_interface_exit_and_nothing_after_it() {
    let (report, text, sent, finished) = session(four_controllers(), true, &options());
    assert_eq!(report.status(), 0, "{text}");
    assert_eq!(sent.last().map(|f| f[1]), Some(0x34));
    let log = finished.sim.log();
    let breaks: Vec<usize> = log
        .iter()
        .enumerate()
        .filter_map(|(at, wire)| matches!(wire, Wire::Break(_)).then_some(at))
        .collect();
    assert_eq!(
        breaks,
        [log.len() - 2, log.len() - 1],
        "one break pair, at the very end"
    );
    assert_eq!(log[log.len() - 2], Wire::Break(true));
    assert_eq!(log[log.len() - 1], Wire::Break(false));
    assert!(
        log[..log.len() - 2]
            .iter()
            .any(|w| matches!(w, Wire::Write(_)))
    );
    assert!(!finished.sim.break_on());
}

#[test]
fn no_exit_leaves_the_session_to_end_by_the_closed_link_and_sends_no_exit() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let (report, text, sent, finished) = session(
        sim,
        true,
        &Options {
            no_exit: true,
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert!(!report.exit_sent);
    assert!(sent.iter().all(|f| f[0] == b'$' || f[1] != 0x34));
    // Without an exit request the session ends when the link closes (and then breaks the lines).
    assert_eq!(finished.exit, Exit::Closed);
    assert_eq!(flashes(&finished.sim), before);
}

#[test]
fn a_refusal_is_reported_as_exit_status_three_with_no_traffic_on_the_lines() {
    let sim = four_controllers();
    let before = flashes(&sim);
    let (report, text, sent, finished) = session(sim, false, &options());
    assert_eq!(report.status(), 3, "{text}");
    assert_eq!(report.passthrough, tool::Passthrough::Refused);
    assert!(text.contains("REFUSED"), "{text}");
    assert_eq!(sent.len(), 5, "the five MSP requests only");
    assert!(finished.sim.log().is_empty(), "{:?}", finished.sim.log());
    assert_eq!(flashes(&finished.sim), before);
    assert_eq!(finished.exit, Exit::Closed);
}

#[test]
fn comparison_files_are_reported_equal_or_by_differing_offsets() {
    let sim = four_controllers();
    let mut files: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    let (report, text, _, _) = session(
        four_controllers(),
        true,
        &Options {
            compare: files.clone(),
            ..options()
        },
    );
    assert_eq!(report.status(), 0, "{text}");
    assert!(
        report
            .escs
            .iter()
            .all(|esc| esc.comparison == Comparison::Equal)
    );
    assert_eq!(text.matches("compare: equal").count(), 4, "{text}");

    files[2][10] ^= 0x01;
    files[2][183] ^= 0x80;
    let (report, text, _, _) = session(
        four_controllers(),
        true,
        &Options {
            compare: files,
            ..options()
        },
    );
    assert_eq!(report.status(), 2, "{text}");
    assert_eq!(report.escs[1].comparison, Comparison::Equal);
    assert_eq!(
        report.escs[2].comparison,
        Comparison::Differs {
            offsets: vec![10, 183],
            block_len: 184,
            file_len: 184
        }
    );
    assert!(text.contains("differing offsets 0x0A 0xB7"), "{text}");
}

#[test]
fn blocks_are_saved_as_escn_bin() {
    let sim = four_controllers();
    let expected: Vec<Vec<u8>> = (0..4).map(|line| block_of(&sim, line)).collect();
    let (report, _, _, _) = session(sim, true, &options());
    let dir = std::env::temp_dir().join(format!("esc-host-fake-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let paths = tool::save_blocks(&dir, &report).unwrap();
    assert_eq!(paths.len(), 4);
    for (n, path) in paths.iter().enumerate() {
        assert_eq!(
            path.file_name().unwrap().to_str().unwrap(),
            format!("esc{}.bin", n + 1)
        );
        assert_eq!(std::fs::read(path).unwrap(), expected[n]);
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_controller_silent_for_three_tries_is_read_and_one_silent_for_twelve_is_status_one() {
    let mut sim = four_controllers();
    let before = flashes(&sim);
    // Controller 2 stays silent for its first three init tries; controller 3 for more than ten.
    sim.faults(1, std::iter::repeat_n(Fault::NoAnswer, 3));
    sim.faults(2, std::iter::repeat_n(Fault::NoAnswer, 12));
    let (report, text, sent, finished) = session(sim, true, &options());
    assert_eq!(report.status(), 1, "{text}");
    assert!(report.escs[0].error.is_none());
    assert!(report.escs[1].error.is_none(), "{text}");
    assert!(text.contains("(try 4 of 10)"), "{text}");
    assert!(
        report.escs[2]
            .error
            .as_deref()
            .unwrap_or("")
            .contains("max retries reached")
    );
    assert!(report.escs[3].error.is_none());
    assert!(report.escs[3].block.is_some());
    let inits_for_third = sent
        .iter()
        .filter(|f| f[0] == 0x2F && f[1] == 0x37 && f[5] == 2)
        .count();
    assert_eq!(inits_for_third, 10);
    assert_eq!(flashes(&finished.sim), before);
}

#[test]
fn asking_for_more_controllers_than_the_board_has_is_status_one() {
    let (report, text, _, finished) = session(
        four_controllers(),
        true,
        &Options {
            escs: Some(5),
            ..options()
        },
    );
    assert_eq!(report.status(), 1, "{text}");
    assert_eq!(report.escs.len(), 5);
    assert!(report.escs[..4].iter().all(|esc| esc.block.is_some()));
    assert!(
        report.escs[4]
            .error
            .as_deref()
            .unwrap_or("")
            .contains("ACK_I_INVALID_CHANNEL"),
        "{text}"
    );
    assert_eq!(finished.exit, Exit::Exited);
}

#[test]
fn a_single_controller_board_is_read_with_the_default_count() {
    let sim = Sim::new(vec![EscModel::f421(0x14)]);
    let before = sim.esc(0).flash().to_vec();
    let Board { handle, mut tool } = start_board(sim, true, 1);
    let mut out = Vec::new();
    let report = tool::run(&mut tool, &options(), &mut out);
    drop(tool);
    let finished = handle.join().unwrap();
    assert_eq!(report.status(), 0, "{}", String::from_utf8_lossy(&out));
    assert_eq!(report.escs.len(), 1);
    assert_eq!(report.escs[0].pin_code, Some(0x14));
    assert_eq!(finished.sim.esc(0).flash(), before);
}
