// Copy most of embedded_io_cursor here to avoid multiple embedded-io versions in dep tree

use core::cmp;
use embedded_io::{BufRead, Error, ErrorKind, ErrorType, Read, Seek, SeekFrom, Write};

#[derive(Debug, Default, Eq, PartialEq)]
pub struct Cursor<T> {
    inner: T,
    pos: usize,
}

impl<T> Cursor<T> {
    /// Creates a new cursor wrapping the provided underlying in-memory buffer.
    ///
    /// Cursor initial position is `0` even if underlying buffer (e.g., `Vec`)
    /// is not empty. So writing to cursor starts with overwriting `Vec`
    /// content, not with appending to it.
    pub const fn new(inner: T) -> Cursor<T> {
        Cursor { pos: 0, inner }
    }

    /// Consumes this cursor, returning the underlying value.
    pub fn into_inner(self) -> T {
        self.inner
    }

    /// Gets a reference to the underlying value in this cursor.
    pub const fn get_ref(&self) -> &T {
        &self.inner
    }

    /// Gets a mutable reference to the underlying value in this cursor.
    ///
    /// Care should be taken to avoid modifying the internal I/O state of the
    /// underlying value as it may corrupt this cursor's position.
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.inner
    }

    /// Returns the current position of this cursor.
    pub const fn position(&self) -> usize {
        self.pos
    }

    /// Sets the position of this cursor.
    pub fn set_position(&mut self, pos: usize) {
        self.pos = pos;
    }
}

impl<T> Cursor<T>
where
    T: AsRef<[u8]>,
{
    /// Returns the remaining slice from the current position.
    ///
    /// This method returns the portion of the underlying buffer that
    /// can still be read from the current cursor position.
    pub fn remaining_slice(&self) -> &[u8] {
        let pos = cmp::min(self.pos, self.inner.as_ref().len());
        &self.inner.as_ref()[pos..]
    }

    /// Returns `true` if there are no more bytes to read from the cursor.
    ///
    /// This is equivalent to checking if `remaining_slice().is_empty()`.
    pub fn is_empty(&self) -> bool {
        self.pos >= self.inner.as_ref().len()
    }
}

impl<T> Clone for Cursor<T>
where
    T: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Cursor {
            inner: self.inner.clone(),
            pos: self.pos,
        }
    }

    #[inline]
    fn clone_from(&mut self, other: &Self) {
        self.inner.clone_from(&other.inner);
        self.pos = other.pos;
    }
}

impl<T> ErrorType for Cursor<T> {
    type Error = ErrorKind;
}

// Read implementation for AsRef<[u8]> types
impl<T> Read for Cursor<T>
where
    T: AsRef<[u8]>,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let remaining = self.remaining_slice();
        let n = cmp::min(buf.len(), remaining.len());

        if n > 0 {
            buf[..n].copy_from_slice(&remaining[..n]);
        }

        self.pos += n;
        Ok(n)
    }
}

// BufRead implementation for AsRef<[u8]> types
impl<T> BufRead for Cursor<T>
where
    T: AsRef<[u8]>,
{
    fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        Ok(self.remaining_slice())
    }

    fn consume(&mut self, amt: usize) {
        self.pos += amt;
    }
}

// Seek implementation for AsRef<[u8]> types
impl<T> Seek for Cursor<T>
where
    T: AsRef<[u8]>,
{
    fn seek(&mut self, style: SeekFrom) -> Result<u64, Self::Error> {
        let (base_pos, offset) = match style {
            SeekFrom::Start(n) => {
                self.pos = n as usize;
                return Ok(n);
            }
            SeekFrom::End(n) => (self.inner.as_ref().len() as u64, n),
            SeekFrom::Current(n) => (self.pos as u64, n),
        };

        match base_pos.checked_add_signed(offset) {
            Some(n) => {
                self.pos = n as usize;
                Ok(self.pos as u64)
            }
            None => Err(ErrorKind::InvalidInput),
        }
    }
}

/// Helper function for writing to fixed-size slices
fn slice_write(pos_mut: &mut usize, slice: &mut [u8], buf: &[u8]) -> Result<usize, ErrorKind> {
    let pos = cmp::min(*pos_mut, slice.len()) as usize;
    let amt = (&mut slice[pos..]).write(buf).map_err(|err| err.kind())?;
    *pos_mut += amt;
    Ok(amt)
}

// Write implementation for &mut [u8]
impl Write for Cursor<&mut [u8]> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        slice_write(&mut self.pos, self.inner, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
