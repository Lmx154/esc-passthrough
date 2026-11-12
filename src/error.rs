/// Why an encoder refused to write a frame. Nothing is written to the caller's buffer in either
/// case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum EncodeError {
    /// The output buffer is shorter than the frame; `needed` is the frame's length in bytes.
    BufferTooSmall {
        /// Length of the frame that did not fit.
        needed: usize,
    },
    /// A length argument is outside what the format can carry (for example 0 or more than 256
    /// bytes of four-way parameters, or more than 255 bytes of MSP payload).
    InvalidLength,
}

/// Writes into a caller buffer whose size was checked up front.
pub(crate) struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> Writer<'a> {
    /// Fails with [`EncodeError::BufferTooSmall`] unless `buf` holds `needed` bytes.
    pub(crate) fn new(buf: &'a mut [u8], needed: usize) -> Result<Self, EncodeError> {
        if buf.len() < needed {
            return Err(EncodeError::BufferTooSmall { needed });
        }
        Ok(Self { buf, len: 0 })
    }

    pub(crate) fn put(&mut self, byte: u8) {
        if let Some(slot) = self.buf.get_mut(self.len) {
            *slot = byte;
            self.len += 1;
        }
    }

    pub(crate) fn put_all(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.put(byte);
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }
}
