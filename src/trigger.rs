//! A recogniser for the one MSP request that opens the configurator's conversation.
//!
//! [`MspTrigger`] looks for exactly the six bytes `24 4D 3C 00 01 01` (`$M<`, length 0, command 1
//! `MSP_API_VERSION`, checksum 1) in a byte stream that carries other traffic. It knows nothing else
//! about MSP: no other command, no length, no checksum rule beyond those six bytes.

/// The request the configurator starts and restarts with: MSP v1 `MSP_API_VERSION`, no payload.
pub const REQUEST: [u8; 6] = [0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01];

/// Finds [`REQUEST`] in a stream, whatever the split between calls.
///
/// The pattern starts with `0x24` and contains it nowhere else, so after a mismatch the longest
/// partial match left is one byte (when the mismatching byte is itself `0x24`) or none, and two
/// occurrences can never overlap. The state is one counter.
///
/// It fires once per occurrence. A corrupted request (any byte changed), a request with another
/// length or command, and a request with a payload are not occurrences.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspTrigger {
    matched: usize,
}

impl Default for MspTrigger {
    fn default() -> Self {
        Self::new()
    }
}

impl MspTrigger {
    /// A recogniser that has seen nothing.
    pub const fn new() -> Self {
        Self { matched: 0 }
    }

    /// Looks at `bytes`, the next part of the stream.
    ///
    /// Returns `Some(n)` when the request is completed inside this slice, where `n` is the number
    /// of bytes of the slice used, its last byte included (so `1 <= n <= bytes.len()`). The bytes
    /// after `n` were not looked at: call again with `&bytes[n..]` to find the next occurrence.
    /// The recogniser starts again from nothing after `Some`.
    ///
    /// Returns `None` when the request did not complete; a partial match at the end of the slice
    /// is kept for the next call.
    pub fn feed(&mut self, bytes: &[u8]) -> Option<usize> {
        for (index, &byte) in bytes.iter().enumerate() {
            if REQUEST.get(self.matched) == Some(&byte) {
                self.matched += 1;
                if self.matched == REQUEST.len() {
                    self.matched = 0;
                    return Some(index + 1);
                }
            } else {
                self.matched = usize::from(REQUEST.first() == Some(&byte));
            }
        }
        None
    }
}
