//! The byte link to the flight controller and the one exchange the tool makes on it: drop what is
//! stale, write a request, collect one reply frame within a time limit.

use std::io;
use std::time::{Duration, Instant};

/// A serial link with a read time limit per call. The tool needs nothing else of a port.
pub trait Link {
    /// Writes all of `bytes` and sends them out.
    ///
    /// # Errors
    ///
    /// The link's I/O error.
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Reads what is there into `buf` (at least one byte), waiting at most `timeout`.
    /// `Ok(0)` means the other end is gone. A wait that ends with no byte is an error of kind
    /// [`io::ErrorKind::TimedOut`].
    ///
    /// # Errors
    ///
    /// The link's I/O error, or `TimedOut`.
    fn read_some(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;

    /// Throws away every byte received and not yet read.
    ///
    /// # Errors
    ///
    /// The link's I/O error.
    fn discard_input(&mut self) -> io::Result<()>;
}

/// Why an exchange gave no frame.
#[derive(Debug)]
pub enum ExchangeError {
    /// The link failed.
    Io(io::Error),
    /// The other end closed the link.
    Closed,
    /// The time limit passed; `got` bytes had arrived (0 for silence).
    Timeout {
        /// Bytes received before the limit.
        got: usize,
    },
}

impl std::fmt::Display for ExchangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "link error: {error}"),
            Self::Closed => write!(f, "the link was closed by the other end"),
            Self::Timeout { got: 0 } => write!(f, "no reply (silence)"),
            Self::Timeout { got } => {
                write!(f, "reply incomplete ({got} bytes) when the time ran out")
            }
        }
    }
}

/// Throws away stale input, writes `request`, and collects one frame. `frame_len` is given the
/// bytes so far and says how long the whole frame is once that is known (`None` while it is not);
/// the frame is complete when that many bytes are in. Bytes beyond it are dropped. Returns the
/// frame and the time from the start of the write to its last byte.
///
/// # Errors
///
/// [`ExchangeError`].
pub fn exchange<L: Link + ?Sized>(
    link: &mut L,
    request: &[u8],
    timeout: Duration,
    frame_len: fn(&[u8]) -> Option<usize>,
) -> Result<(Vec<u8>, Duration), ExchangeError> {
    link.discard_input().map_err(ExchangeError::Io)?;
    let start = Instant::now();
    link.write_all(request).map_err(ExchangeError::Io)?;
    let mut frame: Vec<u8> = Vec::new();
    let mut chunk = [0u8; 512];
    loop {
        if let Some(total) = frame_len(&frame)
            && frame.len() >= total
        {
            frame.truncate(total);
            return Ok((frame, start.elapsed()));
        }
        let left = timeout.saturating_sub(start.elapsed());
        if left.is_zero() {
            return Err(ExchangeError::Timeout { got: frame.len() });
        }
        match link.read_some(&mut chunk, left) {
            Ok(0) => return Err(ExchangeError::Closed),
            Ok(got) => frame.extend_from_slice(chunk.get(..got).unwrap_or_default()),
            Err(error) if error.kind() == io::ErrorKind::TimedOut => {
                return Err(ExchangeError::Timeout { got: frame.len() });
            }
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(ExchangeError::Io(error)),
        }
    }
}

/// A real serial port.
pub struct SerialLink(pub Box<dyn serialport::SerialPort>);

impl Link for SerialLink {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        io::Write::write_all(&mut self.0, bytes)?;
        io::Write::flush(&mut self.0)
    }

    fn read_some(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        self.0
            .set_timeout(timeout.max(Duration::from_millis(1)))
            .map_err(io::Error::other)?;
        io::Read::read(&mut self.0, buf)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.0
            .clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)
    }
}
