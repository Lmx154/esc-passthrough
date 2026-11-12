//! Sans-IO codecs for letting the AM32 configurator reach ESCs through a flight controller.
//!
//! The configurator on a computer speaks two protocols to a flight controller: MSP v1 first, and
//! after `MSP_SET_PASSTHROUGH` the "four-way interface". The flight controller turns four-way
//! commands into the AM32 bootloader's one-wire byte sequences. This crate holds the three wire
//! formats as pure functions on bytes; it does no I/O, owns no timer and keeps no session:
//!
//! - [`msp`]: an incremental MSP v1 request parser and the five replies the configurator needs
//!   (plus the error reply).
//! - [`fourway`]: an incremental four-way request parser and the reply encoder.
//! - [`bootloader`]: the AM32 bootloader's host-to-ESC sequences, the parsers for its replies and
//!   its CRC16.
//!
//! Payloads and addresses are opaque byte slices and `u16`s. Nothing here knows what any byte of
//! an ESC's settings means.
//!
//! No panic for any input: malformed, truncated and oversized input and too small output buffers
//! are errors ([`EncodeError`] and each module's `ParseError`). Nothing allocates.
//!
//! Examples: see README.md, whose examples are compiled by `cargo test`.
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]
#![cfg_attr(not(test), deny(clippy::float_arithmetic))]

pub mod bootloader;
mod error;
pub mod fourway;
pub mod msp;

pub use error::EncodeError;

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
