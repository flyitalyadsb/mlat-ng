#!/usr/bin/env python3
"""CPython reference for mlat-ng's wire-protocol tests (src/proto).

  proto_golden.py gen OUT.bin   write the exact bytes an mlat-client 0.3.x
                                writes on its server socket after the
                                server picked zlib2: the handshake line,
                                then the zlib frames of four fill_zlib()
                                calls (one big enough to split in frames).
  proto_golden.py decode IN.bin decode a server->client zlib2 stream the
                                way mlat-client's consume_readbuf_zlib()
                                does, printing one JSON line per message.

It reimplements mlat-client's mlat/client/jsonclient.py (handshake_msg,
send_tcp_mlat, send_tcp_sync, _send_json, fill_zlib, consume_readbuf_zlib)
on CPython's zlib, the library the client really runs. The Rust test
proto::tests::cpython_golden_stream regenerates the bulk lines with the same
LCG and checks every decoded line.
"""
import json
import struct
import sys
import zlib

MASK = (1 << 64) - 1


class Lcg:
    def __init__(self, seed):
        self.x = seed

    def next32(self):
        self.x = (self.x * 6364136223846793005 + 1442695040888963407) & MASK
        return self.x >> 32


def bulk_lines(n=2000, seed=20261024):
    """send_tcp_sync / send_tcp_mlat lines with pseudo-random content."""
    r = Lcg(seed)
    out = []
    for i in range(n):
        hi = r.next32()
        t = (hi << 16) | (r.next32() & 0xFFFF)
        if i % 4 == 0:
            em = bytes(r.next32() & 0xFF for _ in range(14))
            om = bytes(r.next32() & 0xFF for _ in range(14))
            ot = t + (r.next32() & 0xFFFFFF)
            out.append('{{"sync":{{"et":{0},"em":"{1}","ot":{2},"om":"{3}"}}}}'.format(t, em.hex(), ot, om.hex()))
        else:
            m = bytes(r.next32() & 0xFF for _ in range(7 if i % 2 else 14))
            out.append('{{"mlat":{{"t":{0},"m":"{1}"}}}}'.format(t, m.hex()))
    return out


def j(o):
    return json.dumps(o, separators=(',', ':'))  # _send_json


def fill_zlib(compressor, linebuf, writebuf):
    data = bytearray()
    pending = False
    for line in linebuf:
        data.extend(compressor.compress((line + '\n').encode('ascii')))
        pending = True
        if len(data) >= 32768:
            data.extend(compressor.flush(zlib.Z_SYNC_FLUSH))
            assert len(data) < 65540
            assert data[-4:] == b'\x00\x00\xff\xff'
            del data[-4:]
            writebuf.extend(struct.pack('!H', len(data)))
            writebuf.extend(data)
            data = bytearray()
            pending = False
    if pending:
        data.extend(compressor.flush(zlib.Z_SYNC_FLUSH))
        assert len(data) < 65540
        assert data[-4:] == b'\x00\x00\xff\xff'
        del data[-4:]
        writebuf.extend(struct.pack('!H', len(data)))
        writebuf.extend(data)


def gen(path):
    hs = {'version': 3, 'client_version': '0.3.91', 'compress': ['none', 'zlib', 'zlib2'],
          'selective_traffic': True, 'heartbeat': True, 'return_results': False,
          'udp_transport': False, 'return_result_format': 'ecef',
          'uuid': '6a9423e0-7ba6-4672-81a4-81b715ac1ad2'}
    hs.update({'lat': 45.4642, 'lon': 9.19, 'alt': 120.0, 'user': 'golden-feeder',
               'clock_type': 'dump1090', 'clock_frequency': 12000000, 'clock_epoch': None,
               'privacy': False})
    wb = bytearray((json.dumps(hs, sort_keys=True) + 16 * '        ' + '\n').encode('ascii'))
    c = zlib.compressobj(1)
    calls = [
        [j({'rate_report': {}})],
        [j({'seen': ['4ca123', '3c6444', '000001']}), j({'input_connected': 'connected'})],
        bulk_lines(),
        [j({'lost': ['3c6444']}),
         j({'clock_reset': {'reason': 'Decoder mode changed to BEAST', 'frequency': 12000000, 'epoch': None, 'mode': 'BEAST'}}),
         j({'clock_jump': True}),
         j({'heartbeat': {'client_time': 1729760000.123}}),
         j({'modeac_messages_count': {'count': 42}}),
         j({'rate_report': {'{0:06X}'.format(k): round(v, 2) for k, v in {0x4ca123: 1.5, 0x3c6444: 0.254}.items()}}),
         j({'input_disconnected': 'disconnected'})],
    ]
    for lines in calls:
        fill_zlib(c, lines, wb)
    with open(path, 'wb') as f:
        f.write(wb)
    print('wrote', path, len(wb), 'bytes')


def decode(path):
    readbuf = bytearray(open(path, 'rb').read())
    d = zlib.decompressobj()
    i = 0
    while i + 2 < len(readbuf):
        hlen, = struct.unpack_from('!H', readbuf, i)
        end = i + 2 + hlen
        if end > len(readbuf):
            break
        packet = bytes(readbuf[i + 2:end]) + b'\x00\x00\xff\xff'
        for line in d.decompress(packet).split(b'\n')[:-1]:
            print(json.dumps(json.loads(line.decode('ascii'))))
        i = end
    if i != len(readbuf):
        sys.exit('trailing bytes: %d' % (len(readbuf) - i))


if __name__ == '__main__':
    {'gen': gen, 'decode': decode}[sys.argv[1]](sys.argv[2])
