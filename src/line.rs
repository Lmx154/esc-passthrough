//! The ESC side of the flight controller: one single-wire serial line per ESC.
//!
//! [`EscLines`] is everything the [`client`](crate::client) asks of the hardware: choose which
//! ESC's line is active, send bytes, receive bytes with a time limit, and pull every line low.
//! It names no HAL, no pin and no protocol. The crate has no timer: the implementer owns the
//! clock and the baud rate (19200 baud, 8N1, idle high in the AM32 bootloader).
//!
//! The implementation of the trait is the board's: a few lines on top of whatever drives the
//! pins. The functions that wait are `async`; the crate needs no executor and no dependency for
//! that.
#![allow(async_fn_in_trait)]

/// Up to a few single-wire serial lines, one of them active at a time.
///
/// Rules for implementers:
///
/// - The line does not echo what is written: [`read`](Self::read) only returns bytes the ESC
///   sent.
/// - Half duplex: after [`write`](Self::write) returns, the line is listening again, and bytes
///   the ESC sends from then on are kept for [`read`](Self::read). Bytes that arrive while
///   nobody reads may be kept or dropped, but never reordered.
/// - A wait always ends: [`read`](Self::read) returns `Ok(0)` when `timeout_us` has passed.
///   `timeout_us == 0` polls: it returns what is already there, or `Ok(0)`, without waiting.
/// - Dropping a future before it completes leaves the line usable; the bytes of a half-sent
///   write may or may not have gone out.
pub trait EscLines {
    /// The implementation's own error (a bus fault, a line number it does not have, ...).
    type Error: core::fmt::Debug;

    /// Make `line` the active one (0-based; the firmware maps it to a pin).
    ///
    /// # Errors
    ///
    /// The implementation's error, for example for a line it does not have.
    fn select(&mut self, line: u8) -> Result<(), Self::Error>;

    /// Send all bytes back to back on the active line; returns when the last stop bit has ended.
    ///
    /// # Errors
    ///
    /// The implementation's error.
    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;

    /// Read into `buf` from the active line: returns as soon as at least one byte is there, or
    /// `Ok(0)` when `timeout_us` has passed with none. The implementer owns the clock.
    ///
    /// # Errors
    ///
    /// The implementation's error.
    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, Self::Error>;

    /// Drive every line low (`true`) or release them all (`false`).
    ///
    /// # Errors
    ///
    /// The implementation's error.
    fn set_break(&mut self, on: bool) -> Result<(), Self::Error>;
}
