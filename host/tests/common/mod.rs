//! The fake board shared by the integration tests: in-memory pipes, the board's thread running the
//! crate's session on a tiny executor, and the crate's simulated bootloaders as the controllers.
#![allow(dead_code)]

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use esc_passthrough::msp::BatteryState;
use esc_passthrough::session::{Exit, Lender, Passthrough, SessionConfig};
use esc_passthrough_host::link::Link;
use esc_passthrough_host::tool::Timing;
use std::collections::VecDeque;
use std::future::Future;
use std::io;
use std::pin::pin;
use std::sync::{Arc, Condvar, Mutex};
use std::task::{Context, Poll, Wake, Waker};
use std::thread::{self, JoinHandle};
use std::time::Duration;

#[path = "../../../tests/common/sim.rs"]
#[allow(clippy::all)]
pub mod sim;

use sim::{EscModel, Sim};

pub const BLOCK_AT: usize = 0x7C00;
pub const BLOCK_LEN: usize = 184;

// ---------------------------------------------------------------- the wire

#[derive(Default)]
pub struct State {
    bytes: VecDeque<u8>,
    closed: bool,
    waker: Option<Waker>,
}

#[derive(Default)]
pub struct Pipe {
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
pub struct ToolEnd {
    to_board: Arc<Pipe>,
    from_board: Arc<Pipe>,
    pub sent: Vec<Vec<u8>>,
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
pub struct BoardRx(Arc<Pipe>);
/// The board's sending half.
pub struct BoardTx(Arc<Pipe>);

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

pub struct ThreadWaker(thread::Thread);

impl Wake for ThreadWaker {
    fn wake(self: Arc<Self>) {
        self.0.unpark();
    }
}

/// Runs `future` on this thread, parking while it is pending. Gives up after 60 s without end.
pub fn block_on<F: Future>(future: F) -> F::Output {
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
pub struct FakeDelay;

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        if ns > 0 {
            std::future::pending::<()>().await;
        }
    }
}

pub struct FakeLender {
    sim: Option<Sim>,
    grant: bool,
}

impl Lender for FakeLender {
    type Lines = Sim;

    async fn lend(&mut self) -> Option<Sim> {
        if self.grant { self.sim.take() } else { None }
    }
}

pub struct Finished {
    pub exit: Exit<ErrorKind>,
    pub sim: Sim,
}

pub struct Board {
    pub handle: JoinHandle<Finished>,
    pub tool: ToolEnd,
}

/// Starts the board with `sim` as its controllers' lines and returns the tool's end of the wire.
pub fn start_board(sim: Sim, grant: bool, esc_count: u8) -> Board {
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

pub fn four_controllers() -> Sim {
    controllers(1)
}

/// Four controllers whose flash accepts writes in units of `granule` bytes.
pub fn controllers(granule: u32) -> Sim {
    let mut sim = Sim::new(
        (0..4)
            .map(|line| EscModel::f421(0x10 + line).with_write_granule(granule))
            .collect(),
    );
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

pub fn block_of(sim: &Sim, line: usize) -> Vec<u8> {
    sim.esc(line).flash()[BLOCK_AT..BLOCK_AT + BLOCK_LEN].to_vec()
}

pub fn quick() -> Timing {
    Timing {
        after_passthrough: Duration::ZERO,
        retry_delay: Duration::ZERO,
        ..Timing::configurator()
    }
}

pub fn flashes(sim: &Sim) -> Vec<Vec<u8>> {
    (0..4).map(|line| sim.esc(line).flash().to_vec()).collect()
}
