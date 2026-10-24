//! Incremental parser for readsb's receiver-tagged Beast output
//! (`--net-bo-port` with `--net-receiver-id`).
//!
//! Every message is `0x1A <type> <payload>`, with any 0x1A inside the payload
//! doubled. Types: `0xE3` = the receiver id (8 bytes, big endian) that applies
//! to the frames after it, `'1'` Mode A/C, `'2'` short and `'3'` long Mode S,
//! each with a 6-byte timestamp and a signal byte first. readsb emits the id
//! whenever it changes and at the start of every flush chunk, so attribution
//! is per frame. Anything else is skipped up to the next message start.

/// One Mode S or Mode A/C frame, attributed to the receiver that heard it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// Receiver id: the first 64 bits of the feeder's UUID (readsb `read_uuid`).
    pub rid: u64,
    /// 48-bit receiver timestamp, as sent (12 MHz counter for an SDR).
    pub ts: u64,
    pub sig: u8,
    /// b'1' Mode A/C (2 bytes), b'2' short (7), b'3' long (14).
    pub kind: u8,
    pub len: u8,
    pub msg: [u8; 14],
}

impl Frame {
    pub fn bytes(&self) -> &[u8] {
        &self.msg[..self.len as usize]
    }
}

#[derive(Default)]
pub struct Parser {
    buf: Vec<u8>,
    rid: u64,
    /// Message starts skipped because the type was unknown or the payload was
    /// cut by an unescaped 0x1A.
    pub desync: u64,
}

enum Step {
    /// `n` payload bytes were read; the message ended at `next`.
    Done(usize),
    /// Not enough bytes yet.
    More,
    /// An unescaped 0x1A inside the payload: a new message starts there.
    Cut(usize),
}

/// Read `out.len()` unescaped bytes starting at `i`.
fn read_payload(buf: &[u8], mut i: usize, out: &mut [u8]) -> Step {
    let mut k = 0;
    while k < out.len() {
        let Some(&b) = buf.get(i) else { return Step::More };
        if b == 0x1A {
            match buf.get(i + 1) {
                None => return Step::More,
                Some(0x1A) => {
                    out[k] = 0x1A;
                    k += 1;
                    i += 2;
                    continue;
                }
                Some(_) => return Step::Cut(i),
            }
        }
        out[k] = b;
        k += 1;
        i += 1;
    }
    Step::Done(i)
}

impl Parser {
    /// Append `data` and push every complete frame to `out`.
    pub fn push(&mut self, data: &[u8], out: &mut Vec<Frame>) {
        self.buf.extend_from_slice(data);
        let buf = std::mem::take(&mut self.buf);
        let mut i = 0;
        let mut payload = [0u8; 21];
        loop {
            let Some(off) = buf[i..].iter().position(|&b| b == 0x1A) else {
                i = buf.len();
                break;
            };
            i += off;
            let Some(&t) = buf.get(i + 1) else { break };
            let plen = match t {
                0xE3 => 8,
                b'1' => 9,
                b'2' => 14,
                b'3' => 21,
                _ => {
                    self.desync += 1;
                    i += 1;
                    continue;
                }
            };
            match read_payload(&buf, i + 2, &mut payload[..plen]) {
                Step::More => break,
                Step::Cut(at) => {
                    self.desync += 1;
                    i = at;
                }
                Step::Done(next) => {
                    if t == 0xE3 {
                        self.rid = u64::from_be_bytes(payload[..8].try_into().unwrap());
                    } else {
                        let len = plen - 7;
                        let mut msg = [0u8; 14];
                        msg[..len].copy_from_slice(&payload[7..plen]);
                        let ts = payload[..6].iter().fold(0u64, |a, &b| (a << 8) | b as u64);
                        out.push(Frame { rid: self.rid, ts, sig: payload[6], kind: t, len: len as u8, msg });
                    }
                    i = next;
                }
            }
        }
        self.buf = buf[i..].to_vec();
    }
}

/// Encode frames back to the wire format (tests and synthetic captures).
#[cfg(test)]
pub fn encode(frames: &[Frame]) -> Vec<u8> {
    fn esc(out: &mut Vec<u8>, b: &[u8]) {
        for &x in b {
            out.push(x);
            if x == 0x1A {
                out.push(0x1A);
            }
        }
    }
    let mut out = Vec::new();
    let mut rid = None;
    for f in frames {
        if rid != Some(f.rid) {
            out.extend_from_slice(&[0x1A, 0xE3]);
            esc(&mut out, &f.rid.to_be_bytes());
            rid = Some(f.rid);
        }
        out.extend_from_slice(&[0x1A, f.kind]);
        esc(&mut out, &f.ts.to_be_bytes()[2..]);
        esc(&mut out, &[f.sig]);
        esc(&mut out, f.bytes());
    }
    out
}
