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
//! On top of the pure codecs, two modules do the one thing that needs I/O, without owning any:
//!
//! - [`mod@line`]: [`EscLines`](line::EscLines), the small trait a board implements for the ESCs'
//!   single-wire lines (select one, write, read with a time limit, pull all low).
//! - [`client`]: the async operations a flight controller performs on one ESC's line (connect,
//!   read, write, erase, keep-alive, run), generic over `EscLines`, with typed errors and
//!   configurable timeouts. It owns no timer and keeps no session.
//!
//! The host side, again without I/O:
//!
//! - [`mod@trigger`]: [`MspTrigger`](trigger::MspTrigger), which recognises the one MSP request
//!   (`24 4D 3C 00 01 01`) that opens the configurator's conversation in a stream that carries other
//!   traffic.
//! - [`host`]: [`HostSide`](host::HostSide), which takes the bytes arriving from the configurator
//!   and says what they ask for, answers the MSP requests that need no decision and builds the
//!   four-way replies. It executes nothing on an ESC.
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
pub mod client;
mod error;
pub mod fourway;
pub mod host;
pub mod line;
pub mod msp;
pub mod trigger;

pub use error::EncodeError;

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
