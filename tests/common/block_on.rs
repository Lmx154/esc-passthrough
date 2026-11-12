//! A minimal `block_on` for the host tests: polls a future with a waker that does nothing.
//! The simulated line never leaves a future pending, so a future that does is a bug: after
//! `MAX_POLLS` polls it panics instead of hanging.
#![allow(dead_code)]

use std::future::Future;
use std::pin::pin;
use std::task::{Context, Poll, Waker};

const MAX_POLLS: usize = 1_000_000;

pub fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..MAX_POLLS {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
    panic!("future still pending after {MAX_POLLS} polls");
}
