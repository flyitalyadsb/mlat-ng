//! Byte-level framing: the socket input buffer, the inbound zlib frame
//! decoder and the outbound zlib2 frame encoder.
//!
//! Wire format of "zlib" / "zlib2" (client -> server, and server -> client in
//! zlib2): frames of a 2-byte big-endian length and that many bytes of one
//! persistent zlib stream (zlib header, level 1), each frame ending at a
//! Z_SYNC_FLUSH whose trailing empty stored block `00 00 ff ff` is stripped.
//! The decoder appends it back before inflating, as mlat-server does.

use flate2::{Compress, Compression, Decompress, FlushCompress, FlushDecompress, Status};
use tokio::io::{AsyncRead, AsyncReadExt};

/// What a Z_SYNC_FLUSH ends with; stripped on the wire.
pub(crate) const SYNC_TRAILER: [u8; 4] = [0x00, 0x00, 0xff, 0xff];

/// Read at least this much per syscall (compacting / growing first).
const MIN_READ: usize = 8 * 1024;

/// A read buffer with a consumed-prefix cursor, so lines and frames are
/// parsed in place and the unconsumed tail is moved only when space runs
/// out. Callers bound its size (a line or frame limit), so it never holds
/// more than about one maximal unit plus one read.
pub(crate) struct InBuf {
    buf: Vec<u8>,
    start: usize,
}

impl InBuf {
    pub(crate) fn new(cap: usize) -> Self {
        InBuf { buf: Vec::with_capacity(cap), start: 0 }
    }

    /// Unconsumed bytes.
    #[inline]
    pub(crate) fn data(&self) -> &[u8] {
        &self.buf[self.start..]
    }

    #[inline]
    pub(crate) fn consume(&mut self, n: usize) {
        self.start += n;
        debug_assert!(self.start <= self.buf.len());
        if self.start >= self.buf.len() {
            self.buf.clear();
            self.start = 0;
        }
    }

    /// One read from `rd` appended to the buffer; Ok(0) is EOF. Cancel-safe:
    /// tokio's read_buf either reads into the spare capacity or not at all.
    pub(crate) async fn fill<R: AsyncRead + Unpin>(&mut self, rd: &mut R) -> std::io::Result<usize> {
        if self.buf.capacity() - self.buf.len() < MIN_READ {
            if self.start > 0 {
                self.buf.drain(..self.start);
                self.start = 0;
            }
            if self.buf.capacity() - self.buf.len() < MIN_READ {
                self.buf.reserve(MIN_READ);
            }
        }
        rd.read_buf(&mut self.buf).await
    }
}

/// Why a zlib frame could not be decoded. Every one is fatal for the
/// connection: the inflate stream cannot be resynchronised.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FrameError {
    /// Not a valid deflate stream (or not the stream we were following).
    Corrupt,
    /// Inflated past the per-frame cap.
    TooBig,
    /// The client finished the zlib stream; nothing may follow.
    StreamEnd,
}

/// The per-connection inbound inflate stream.
pub(crate) struct Inflater {
    z: Decompress,
}

impl Inflater {
    pub(crate) fn new() -> Self {
        // zlib-wrapped, like Python's zlib.decompressobj() default.
        Inflater { z: Decompress::new(true) }
    }

    /// Inflate one frame payload (as on the wire, trailer stripped), appending
    /// the output to `out`. At most `cap` bytes may come out of one frame.
    pub(crate) fn frame(&mut self, payload: &[u8], out: &mut Vec<u8>, cap: usize) -> Result<(), FrameError> {
        let start = out.len();
        self.feed(payload, out, start, cap)?;
        self.feed(&SYNC_TRAILER, out, start, cap)
    }

    fn feed(&mut self, mut input: &[u8], out: &mut Vec<u8>, start: usize, cap: usize) -> Result<(), FrameError> {
        loop {
            // decompress_vec only writes into spare capacity.
            if out.capacity() - out.len() < 4096 {
                out.reserve(16 * 1024);
            }
            let (in0, out0) = (self.z.total_in(), self.z.total_out());
            let st = self.z.decompress_vec(input, out, FlushDecompress::None).map_err(|_| FrameError::Corrupt)?;
            let used = (self.z.total_in() - in0) as usize;
            let made = (self.z.total_out() - out0) as usize;
            input = &input[used..];
            if out.len() - start > cap {
                return Err(FrameError::TooBig);
            }
            if st == Status::StreamEnd {
                return Err(FrameError::StreamEnd);
            }
            // zlib's contract: input used up and output space left over means
            // everything decodable so far has been produced.
            if input.is_empty() && out.len() < out.capacity() {
                return Ok(());
            }
            if used == 0 && made == 0 && out.len() < out.capacity() {
                // No progress with input and room available: cannot happen on
                // a sane stream (mlat-server: "Decompressor made no progress").
                return Err(FrameError::Corrupt);
            }
        }
    }
}

/// The per-connection outbound zlib2 stream: one compressor at level 1, like
/// mlat-server's `zlib.compressobj(1)`.
pub(crate) struct Deflater {
    z: Compress,
}

impl Deflater {
    pub(crate) fn new() -> Self {
        Deflater { z: Compress::new(Compression::new(1), true) }
    }

    /// Append to `out` the frames carrying `lines` (each newline-terminated),
    /// split exactly like mlat-server's `_flush_zlib_locked`: a frame is closed
    /// as soon as it holds >= 32768 bytes, the rest goes in a last frame.
    pub(crate) fn frames<'a>(&mut self, lines: impl IntoIterator<Item = &'a [u8]>, out: &mut Vec<u8>) {
        // `data` in mlat-server starts with the 2 length bytes; so does ours.
        let mut frame = out.len();
        out.extend_from_slice(&[0, 0]);
        let mut pending = false;
        for line in lines {
            self.deflate(line, out, FlushCompress::None);
            pending = true;
            if out.len() - frame >= 32768 {
                self.close_frame(out, frame);
                frame = out.len();
                out.extend_from_slice(&[0, 0]);
                pending = false;
            }
        }
        if pending {
            self.close_frame(out, frame);
        } else {
            out.truncate(frame);
        }
    }

    /// Sync-flush, strip the trailer, write the length.
    fn close_frame(&mut self, out: &mut Vec<u8>, frame: usize) {
        self.deflate(&[], out, FlushCompress::Sync);
        if out.ends_with(&SYNC_TRAILER) {
            out.truncate(out.len() - SYNC_TRAILER.len());
        }
        // Lines are capped well below 32 KiB by the writer, so a frame stays
        // under 32768 + one line; the clamp is only a backstop.
        let len = (out.len() - frame - 2).min(u16::MAX as usize) as u16;
        out[frame..frame + 2].copy_from_slice(&len.to_be_bytes());
    }

    fn deflate(&mut self, mut input: &[u8], out: &mut Vec<u8>, flush: FlushCompress) {
        loop {
            if out.capacity() - out.len() < 1024 + input.len() / 2 {
                out.reserve(1024 + input.len());
            }
            let in0 = self.z.total_in();
            if self.z.compress_vec(input, out, flush).is_err() {
                return; // only on misuse; never with these arguments
            }
            input = &input[(self.z.total_in() - in0) as usize..];
            if input.is_empty() && out.len() < out.capacity() {
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::tests::client_fill_zlib;

    /// Split a byte stream into frame payloads.
    fn payloads(mut b: &[u8]) -> Vec<&[u8]> {
        let mut v = Vec::new();
        while b.len() >= 2 {
            let n = u16::from_be_bytes([b[0], b[1]]) as usize;
            v.push(&b[2..2 + n]);
            b = &b[2 + n..];
        }
        assert!(b.is_empty());
        v
    }

    #[test]
    fn deflater_frames_inflate_back_across_frames() {
        let mut d = Deflater::new();
        let mut wire = Vec::new();
        let lines: Vec<Vec<u8>> = (0..5000u64)
            .map(|i| format!("{{\"result\":{{\"@\":{i},\"x\":\"{:016x}\"}}}}\n", i.wrapping_mul(0x9e3779b97f4a7c15)).into_bytes())
            .collect();
        d.frames(lines[..10].iter().map(|l| l.as_slice()), &mut wire);
        d.frames(lines[10..].iter().map(|l| l.as_slice()), &mut wire);
        d.frames(std::iter::empty(), &mut wire); // nothing pending: no frame
        let frames = payloads(&wire);
        assert!(frames.len() >= 3, "the big batch must split ({} frames)", frames.len());
        let mut inf = Inflater::new();
        let mut out = Vec::new();
        for f in &frames {
            assert!(f.len() < 65536);
            inf.frame(f, &mut out, 1 << 20).unwrap();
        }
        assert_eq!(out, lines.concat());
    }

    #[test]
    fn client_encoding_decodes() {
        let mut z = Compress::new(Compression::new(1), true);
        let mut wire = Vec::new();
        let a: &[u8] = b"{\"rate_report\":{}}\n";
        let b: &[u8] = b"{\"mlat\":{\"t\":123456789,\"m\":\"5d4ca1239bb7c0\"}}\n";
        client_fill_zlib(&mut z, &[a], &mut wire);
        client_fill_zlib(&mut z, &[b, b, b], &mut wire);
        let mut inf = Inflater::new();
        let mut out = Vec::new();
        for f in payloads(&wire) {
            inf.frame(f, &mut out, 1 << 20).unwrap();
        }
        assert_eq!(out, [a, b, b, b].concat());
    }

    #[test]
    fn inflate_cap_and_garbage() {
        // A highly compressible frame that inflates past the cap.
        let mut wire = Vec::new();
        let bomb = vec![b'a'; 3 << 20];
        let mut z = Compress::new(Compression::new(1), true);
        client_fill_zlib(&mut z, &[&bomb], &mut wire);
        let mut inf = Inflater::new();
        let mut out = Vec::new();
        let f = payloads(&wire)[0];
        assert_eq!(inf.frame(f, &mut out, 1 << 20), Err(FrameError::TooBig));
        // Garbage is Corrupt, never a panic.
        let mut inf = Inflater::new();
        assert_eq!(inf.frame(b"\xde\xad\xbe\xef", &mut Vec::new(), 1 << 20), Err(FrameError::Corrupt));
    }

    #[test]
    fn finished_stream_is_reported() {
        let mut z = Compress::new(Compression::new(1), true);
        let mut data = Vec::with_capacity(1024);
        z.compress_vec(b"{\"heartbeat\":{}}\n", &mut data, FlushCompress::Finish).unwrap();
        let mut inf = Inflater::new();
        let mut out = Vec::new();
        assert_eq!(inf.frame(&data, &mut out, 1 << 20), Err(FrameError::StreamEnd));
    }
}
