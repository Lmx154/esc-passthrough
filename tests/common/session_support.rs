//! Test doubles for the session tests: a virtual clock with its own executor, a scripted host
//! stream, a delay on that clock, a lender, the simulator behind a recording wrapper, and frame
//! builders for the configurator's requests and the expected replies (built here, from the layouts,
//! with the bit-serial CRC of `common`).
//!
//! Time is virtual and moves only when the task is pending: `World::run` polls the future and,
//! when it is pending, sets the clock to the earliest deadline some double registered.
//!
//! A gate counts the awaits of the doubles (host read, host write, flush, delay, lend, line
//! write and read). With a trip index set, the await with that index returns `Pending` once and
//! `World::run_until_trip` hands the future back to be dropped: that is how the tests drop `run` at
//! every await.
#![allow(dead_code)]

use crate::common::ref_crc_xmodem;
use crate::common::sim::{Sim, SimError, Wire};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use esc_passthrough::line::EscLines;
use esc_passthrough::session::Lender;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

pub struct World {
    now_us: Cell<u64>,
    deadlines: RefCell<Vec<u64>>,
    points: Cell<usize>,
    trip_at: Cell<Option<usize>>,
    tripped: Cell<bool>,
    log: RefCell<Vec<(u64, Ev)>>,
}

/// What happened, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Ev {
    Lend,
    /// A `write` the host stream accepted.
    HostWrite(Vec<u8>),
    Flush,
    Line(Wire),
    /// A `delay_ms`, as requested.
    Delay(u32),
}

impl World {
    pub fn new() -> Rc<Self> {
        Rc::new(Self {
            now_us: Cell::new(0),
            deadlines: RefCell::new(Vec::new()),
            points: Cell::new(0),
            trip_at: Cell::new(None),
            tripped: Cell::new(false),
            log: RefCell::new(Vec::new()),
        })
    }

    pub fn now_us(&self) -> u64 {
        self.now_us.get()
    }

    pub fn now_ms(&self) -> u64 {
        self.now_us.get() / 1000
    }

    /// Awaits counted so far.
    pub fn points(&self) -> usize {
        self.points.get()
    }

    pub fn reset_points(&self) {
        self.points.set(0);
    }

    pub fn trip_at(&self, index: Option<usize>) {
        self.trip_at.set(index);
        self.tripped.set(false);
    }

    /// One await of a double. Returns `Pending` once if this is the await with the trip index.
    pub async fn point(&self) {
        let index = self.points.get();
        self.points.set(index + 1);
        let mut first = true;
        poll_fn(|_| {
            if first {
                first = false;
                if self.trip_at.get() == Some(index) {
                    self.tripped.set(true);
                    self.trip_at.set(None);
                    return Poll::Pending;
                }
            }
            Poll::Ready(())
        })
        .await;
    }

    async fn sleep_until(&self, deadline_us: u64) {
        poll_fn(|_| {
            if self.now_us.get() >= deadline_us {
                Poll::Ready(())
            } else {
                self.deadlines.borrow_mut().push(deadline_us);
                Poll::Pending
            }
        })
        .await;
    }

    /// Runs `future` to its end; panics if it is pending with no timer to wait for.
    pub fn run<F: Future>(&self, future: F) -> F::Output {
        match self.run_until_trip(future) {
            Some(value) => value,
            None => panic!("the gate tripped in a run that had no trip index"),
        }
    }

    /// Runs `future` until it ends (`Some`) or the gate trips (`None`, and the future is dropped).
    pub fn run_until_trip<F: Future>(&self, future: F) -> Option<F::Output> {
        let mut future = pin!(future);
        let mut cx = Context::from_waker(Waker::noop());
        loop {
            self.deadlines.borrow_mut().clear();
            match future.as_mut().poll(&mut cx) {
                Poll::Ready(value) => return Some(value),
                Poll::Pending => {
                    if self.tripped.replace(false) {
                        return None;
                    }
                    let next = self.deadlines.borrow().iter().copied().min();
                    match next {
                        Some(at) => self.now_us.set(self.now_us.get().max(at)),
                        None => panic!("pending with no timer at {} us", self.now_us.get()),
                    }
                }
            }
        }
    }

    pub fn note(&self, event: Ev) {
        self.log.borrow_mut().push((self.now_us(), event));
    }

    pub fn log(&self) -> Vec<(u64, Ev)> {
        self.log.borrow().clone()
    }

    pub fn clear_log(&self) {
        self.log.borrow_mut().clear();
    }

    /// The line calls with their times.
    pub fn line_events(&self) -> Vec<(u64, Wire)> {
        self.log
            .borrow()
            .iter()
            .filter_map(|(t, e)| match e {
                Ev::Line(w) => Some((*t, w.clone())),
                _ => None,
            })
            .collect()
    }

    /// The writes the host stream accepted, with their times.
    pub fn host_writes(&self) -> Vec<(u64, Vec<u8>)> {
        self.log
            .borrow()
            .iter()
            .filter_map(|(t, e)| match e {
                Ev::HostWrite(b) => Some((*t, b.clone())),
                _ => None,
            })
            .collect()
    }

    /// The `delay_ms` calls: start time and milliseconds.
    pub fn delays(&self) -> Vec<(u64, u32)> {
        self.log
            .borrow()
            .iter()
            .filter_map(|(t, e)| match e {
                Ev::Delay(ms) => Some((*t, *ms)),
                _ => None,
            })
            .collect()
    }
}

/// The delay on the world's clock. Records each `delay_ms`.
pub struct VDelay(pub Rc<World>);

impl DelayNs for VDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.point().await;
        let start = self.0.now_us();
        self.0
            .sleep_until(start + u64::from(ns).div_ceil(1000))
            .await;
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.0.point().await;
        let start = self.0.now_us();
        self.0.note(Ev::Delay(ms));
        self.0.sleep_until(start + u64::from(ms) * 1000).await;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostError {
    Read,
    Write,
}

impl std::fmt::Display for HostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for HostError {}

impl embedded_io_async::Error for HostError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub enum Step {
    /// Bytes the stream delivers; `request` says that they complete one request that must be answered.
    Send { bytes: Vec<u8>, request: bool },
    /// The next `Send` is not delivered before this many ms after the last write to the host.
    AfterReplyMs(u64),
    /// The next `Send` is not delivered before this time (ms since the start).
    AtMs(u64),
    /// The next read fails.
    Fail,
    /// Reads return `Ok(0)` from here on.
    Eof,
    /// Reads never complete.
    Hang,
}

pub fn request(bytes: &[u8]) -> Step {
    Step::Send {
        bytes: bytes.to_vec(),
        request: true,
    }
}

pub fn noise(bytes: &[u8]) -> Step {
    Step::Send {
        bytes: bytes.to_vec(),
        request: false,
    }
}

#[derive(Default)]
pub struct HostState {
    script: VecDeque<Step>,
    current: VecDeque<u8>,
    current_is_request: bool,
    not_before_us: u64,
    last_write_us: u64,
    /// Longest slice one read returns (the whole script otherwise).
    pub max_read: usize,
    /// Complete requests delivered.
    pub complete_requests: usize,
    /// Calls to `write`.
    pub writes: usize,
    pub flushes: usize,
    /// Writes made when no unanswered complete request had been delivered.
    pub early_writes: usize,
    /// The `n`th write (0-based) fails.
    pub fail_write_at: Option<usize>,
    /// The `n`th flush fails.
    pub fail_flush_at: Option<usize>,
    /// The `n`th write accepts no byte.
    pub zero_write_at: Option<usize>,
    /// Every write accepts at most this many bytes.
    pub max_write: usize,
    pub delivered: usize,
    /// Writes accepted (replies, when every write takes a whole reply).
    pub replies: usize,
    /// A request is delivered only after the replies to all the earlier ones were written, as the
    /// configurator does. Default true.
    pub strict: bool,
}

#[derive(Clone)]
pub struct Host {
    pub state: Rc<RefCell<HostState>>,
    world: Rc<World>,
}

pub struct HostRx(Host);
pub struct HostTx(Host);

impl Host {
    pub fn new(world: &Rc<World>, script: Vec<Step>) -> Self {
        let state = HostState {
            script: script.into(),
            max_read: usize::MAX,
            max_write: usize::MAX,
            strict: true,
            ..HostState::default()
        };
        Self {
            state: Rc::new(RefCell::new(state)),
            world: world.clone(),
        }
    }

    pub fn rx(&self) -> HostRx {
        HostRx(self.clone())
    }

    pub fn tx(&self) -> HostTx {
        HostTx(self.clone())
    }

    pub fn push(&self, step: Step) {
        self.state.borrow_mut().script.push_back(step);
    }

    /// Bytes in the script that are not delivered yet.
    pub fn undelivered(&self) -> usize {
        let st = self.state.borrow();
        st.current.len()
            + st.script
                .iter()
                .map(|s| match s {
                    Step::Send { bytes, .. } => bytes.len(),
                    _ => 0,
                })
                .sum::<usize>()
    }

    pub fn complete_requests(&self) -> usize {
        self.state.borrow().complete_requests
    }

    pub fn early_writes(&self) -> usize {
        self.state.borrow().early_writes
    }

    pub fn writes(&self) -> usize {
        self.state.borrow().writes
    }

    /// Everything written to the host, in order, one `Vec` per call.
    pub fn written(&self) -> Vec<Vec<u8>> {
        self.world
            .host_writes()
            .into_iter()
            .map(|(_, b)| b)
            .collect()
    }

    pub fn written_with_time(&self) -> Vec<(u64, Vec<u8>)> {
        self.world.host_writes()
    }
}

impl ErrorType for HostRx {
    type Error = HostError;
}

impl ErrorType for HostTx {
    type Error = HostError;
}

impl Read for HostRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, HostError> {
        let host = &self.0;
        host.world.point().await;
        poll_fn(|_| {
            let mut st = host.state.borrow_mut();
            loop {
                if !st.current.is_empty() {
                    let n = buf.len().min(st.max_read).min(st.current.len());
                    for slot in buf.iter_mut().take(n) {
                        *slot = st.current.pop_front().unwrap();
                    }
                    st.delivered += n;
                    if st.current.is_empty() && st.current_is_request {
                        st.complete_requests += 1;
                    }
                    return Poll::Ready(Ok(n));
                }
                let now = host.world.now_us();
                match st.script.front() {
                    None | Some(Step::Hang) => return Poll::Pending,
                    Some(Step::Eof) => return Poll::Ready(Ok(0)),
                    Some(Step::Fail) => {
                        st.script.pop_front();
                        return Poll::Ready(Err(HostError::Read));
                    }
                    Some(Step::AtMs(ms)) => {
                        st.not_before_us = ms * 1000;
                        st.script.pop_front();
                    }
                    Some(Step::AfterReplyMs(ms)) => {
                        st.not_before_us = st.last_write_us + ms * 1000;
                        st.script.pop_front();
                    }
                    Some(Step::Send { request, .. }) => {
                        if *request && st.strict && st.replies < st.complete_requests {
                            return Poll::Pending;
                        }
                        if now < st.not_before_us {
                            let at = st.not_before_us;
                            host.world.deadlines.borrow_mut().push(at);
                            return Poll::Pending;
                        }
                        if let Some(Step::Send { bytes, request }) = st.script.pop_front() {
                            st.current = bytes.into();
                            st.current_is_request = request;
                        }
                    }
                }
            }
        })
        .await
    }
}

impl Write for HostTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, HostError> {
        let host = &self.0;
        host.world.point().await;
        let mut st = host.state.borrow_mut();
        let index = st.writes;
        st.writes += 1;
        if st.fail_write_at == Some(index) {
            return Err(HostError::Write);
        }
        if st.zero_write_at == Some(index) {
            return Ok(0);
        }
        let n = buf.len().min(st.max_write);
        if st.replies >= st.complete_requests {
            st.early_writes += 1;
        }
        st.replies += 1;
        st.last_write_us = host.world.now_us();
        host.world.note(Ev::HostWrite(buf[..n].to_vec()));
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), HostError> {
        let host = &self.0;
        host.world.point().await;
        let mut st = host.state.borrow_mut();
        let index = st.flushes;
        st.flushes += 1;
        if st.fail_flush_at == Some(index) {
            return Err(HostError::Write);
        }
        host.world.note(Ev::Flush);
        Ok(())
    }
}

/// The simulator behind a recorder: every call is logged with the world's time.
pub struct TracedLines {
    pub sim: Sim,
    world: Rc<World>,
    /// The next `n` calls of `set_break` fail.
    pub fail_breaks: usize,
}

impl TracedLines {
    pub fn new(world: &Rc<World>, sim: Sim) -> Self {
        Self {
            sim,
            world: world.clone(),
            fail_breaks: 0,
        }
    }

    fn record(&self, wire: Wire) {
        self.world.note(Ev::Line(wire));
    }
}

impl EscLines for TracedLines {
    type Error = SimError;

    fn select(&mut self, line: u8) -> Result<(), SimError> {
        self.record(Wire::Select(line));
        self.sim.select(line)
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), SimError> {
        self.world.point().await;
        self.record(Wire::Write(bytes.to_vec()));
        self.sim.write(bytes).await
    }

    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, SimError> {
        self.world.point().await;
        self.record(Wire::Read {
            cap: buf.len(),
            timeout_us,
        });
        self.sim.read(buf, timeout_us).await
    }

    fn set_break(&mut self, on: bool) -> Result<(), SimError> {
        self.record(Wire::Break(on));
        if self.fail_breaks > 0 {
            self.fail_breaks -= 1;
            return Err(SimError::Injected);
        }
        self.sim.set_break(on)
    }
}

/// Lends what it was given, one offer per call; `None` refuses.
pub struct TestLender {
    world: Rc<World>,
    pub offers: VecDeque<Option<TracedLines>>,
    pub asked: usize,
    pub asked_at_us: Vec<u64>,
}

impl TestLender {
    pub fn new(world: &Rc<World>, offers: Vec<Option<TracedLines>>) -> Self {
        Self {
            world: world.clone(),
            offers: offers.into(),
            asked: 0,
            asked_at_us: Vec::new(),
        }
    }
}

impl Lender for TestLender {
    type Lines = TracedLines;

    async fn lend(&mut self) -> Option<TracedLines> {
        self.world.point().await;
        self.asked += 1;
        self.asked_at_us.push(self.world.now_us());
        self.world.note(Ev::Lend);
        self.offers.pop_front().flatten()
    }
}

// ---------------------------------------------------------------------------------------------
// Frames, laid out by hand: MSP v1 (`$M<` / `$M>`, length, command, payload, XOR of those), the
// four-way frames (`2F` / `2E`, command, address high and low, count (0 is 256), parameters,
// [acknowledge for a reply], CRC16-XMODEM high first, over everything before it).

pub fn msp_request(command: u8, payload: &[u8]) -> Vec<u8> {
    msp_frame(b'<', command, payload)
}

pub fn msp_reply(command: u8, payload: &[u8]) -> Vec<u8> {
    msp_frame(b'>', command, payload)
}

pub fn msp_error(command: u8) -> Vec<u8> {
    msp_frame(b'!', command, &[])
}

fn msp_frame(direction: u8, command: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![b'$', b'M', direction, payload.len() as u8, command];
    frame.extend_from_slice(payload);
    let checksum = frame[3..].iter().fold(0, |acc, b| acc ^ b);
    frame.push(checksum);
    frame
}

fn four_way(start: u8, command: u8, address: u16, params: &[u8], ack: Option<u8>) -> Vec<u8> {
    let mut frame = vec![start, command, (address >> 8) as u8, address as u8];
    frame.push(if params.len() == 256 {
        0
    } else {
        params.len() as u8
    });
    frame.extend_from_slice(params);
    frame.extend(ack);
    let crc = ref_crc_xmodem(&frame);
    frame.push((crc >> 8) as u8);
    frame.push(crc as u8);
    frame
}

pub fn fw_request(command: u8, address: u16, params: &[u8]) -> Vec<u8> {
    four_way(0x2F, command, address, params, None)
}

pub fn fw_reply(command: u8, address: u16, params: &[u8], ack: u8) -> Vec<u8> {
    four_way(0x2E, command, address, params, Some(ack))
}

pub const API_VERSION: u8 = 1;
pub const FC_VARIANT: u8 = 2;
pub const BATTERY_STATE: u8 = 130;
pub const MOTOR_CONFIG: u8 = 131;
pub const SET_PASSTHROUGH: u8 = 245;

pub const CMD_TEST_ALIVE: u8 = 0x30;
pub const CMD_PROTOCOL_VERSION: u8 = 0x31;
pub const CMD_INTERFACE_NAME: u8 = 0x32;
pub const CMD_INTERFACE_VERSION: u8 = 0x33;
pub const CMD_EXIT: u8 = 0x34;
pub const CMD_RESET: u8 = 0x35;
pub const CMD_INIT_FLASH: u8 = 0x37;
pub const CMD_ERASE_ALL: u8 = 0x38;
pub const CMD_PAGE_ERASE: u8 = 0x39;
pub const CMD_READ: u8 = 0x3A;
pub const CMD_WRITE: u8 = 0x3B;
pub const CMD_C2CK_LOW: u8 = 0x3C;
pub const CMD_READ_EEPROM: u8 = 0x3D;
pub const CMD_WRITE_EEPROM: u8 = 0x3E;
pub const CMD_SET_MODE: u8 = 0x3F;
pub const CMD_VERIFY: u8 = 0x40;

pub const ACK_OK: u8 = 0x00;
pub const ACK_INVALID_CMD: u8 = 0x02;
pub const ACK_INVALID_CRC: u8 = 0x03;
pub const ACK_INVALID_CHANNEL: u8 = 0x08;
pub const ACK_INVALID_PARAM: u8 = 0x09;
pub const ACK_GENERAL_ERROR: u8 = 0x0F;

/// The settings block of the simulated F421: its first byte is 1 so the application may start; the
/// rest is a pattern the tests know.
pub fn settings_pattern(seed: u8) -> Vec<u8> {
    (0..184u32)
        .map(|i| {
            if i == 0 {
                1
            } else {
                (i as u8).wrapping_mul(7).wrapping_add(seed)
            }
        })
        .collect()
}

pub const SETTINGS_AT: usize = 0x7C00;
pub const FILE_NAME_AT: usize = 0x7BE0;
pub const DEVINFO_AT: usize = 0x0D00;

/// A four-ESC simulator whose controllers have distinct settings blocks.
pub fn four_escs() -> Sim {
    let mut sim = Sim::four();
    for line in 0..4 {
        let esc = sim.esc_mut(line);
        let settings = settings_pattern(line as u8 * 16);
        esc.flash_mut()[SETTINGS_AT..SETTINGS_AT + 184].copy_from_slice(&settings);
    }
    sim
}

/// Positions of `Break(true)` in the line events.
pub fn break_on_positions(events: &[(u64, Wire)]) -> Vec<usize> {
    events
        .iter()
        .enumerate()
        .filter(|(_, (_, w))| *w == Wire::Break(true))
        .map(|(i, _)| i)
        .collect()
}
