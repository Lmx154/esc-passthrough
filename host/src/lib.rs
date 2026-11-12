//! `esc-passthrough-host`: a read-only bench tool that talks to a flight controller the way the AM32
//! configurator does. It connects, enters passthrough, identifies each motor controller, reads its
//! settings block, leaves, and reports. See README.md.
//!
//! The protocol code here (MSP v1 and the four-way interface, with their checks) is written for the
//! tool and does not use the `esc-passthrough` crate, which has only the flight-controller side.
//!
//! READ-ONLY: every request the tool can send is a variant of [`msp::Request`] or
//! [`fourway::Request`]; neither has a write, erase or reset.
#![forbid(unsafe_code)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable
    )
)]

pub mod cli;
pub mod fourway;
pub mod link;
pub mod msp;
pub mod port;
pub mod tool;
