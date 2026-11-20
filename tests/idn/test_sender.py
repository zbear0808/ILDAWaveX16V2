"""Tests for tools/idn_sender.py: the packets it sends, pacing, closing and management requests.

Standard library only. Run with tools/test_idn.py, or: python -m unittest discover -s tests/idn

Expected packets are written byte by byte from the IDN-Hello / IDN-Stream field tables, never
produced by the sender. Most cases are FIRST (a stream's first packet) with a few fields changed
by variant(), and every test stream is BASE plus the settings that test changes.
"""

import io
import os
import socket
import sys
import unittest
from contextlib import redirect_stderr

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))

import idn_sender as idn  # noqa: E402
from ilw_sender import LaserPoint  # noqa: E402

ADDR = ("192.0.2.50", 7255)                       # the DAC, at the IDN-Hello port
OTHER = ("192.0.2.99", 7255)                      # another host
BASE = dict(pps=30000, packet_points=20)          # 20 samples = 666.7 us per packet
SEQ = 0x1234                                      # first RT sequence number of every test stream
BLANK = LaserPoint(0, 0, 0, 0, 0, 0)

# First packet of a BASE stream of blank points (Hello 2.1; Stream 2.1, 2.2, 3.4, 6.1).
FIRST = bytes.fromhex(
    "40 00 1234"                                  # RT channel message, client group 0, sequence
    "00c0 c0 01 00000000"                         # 192 octets; config follows, channel 0; wave chunk; time 0
    "04 01 00 01"                                 # 4 config words; Routing, SDM 0; default service; continuous
    "4200 4010 4210 4010 527e 5214 51cc 5c10"     # X + precision, Y + precision, 638, 532, 460 nm, intensity
    "00 00029a") + bytes(20 * 8)                  # SCM 0, 666 us; 20 blank 8-octet samples
SECOND = bytes.fromhex("40 00 1235  00ac 80 01 0000029a  00 00029b") + bytes(20 * 8)   # no config; 667 us
CLOSE = bytes.fromhex("44 00 1236")                                                    # graceful close after them

FIELDS = {"group": (1, 1), "seq": (2, 2), "cnl": (6, 1), "ts": (8, 4),             # (offset, octets) in FIRST
          "cfl": (13, 1), "service": (14, 1), "scm": (32, 1)}


def variant(packet, **fields):
    """`packet` with some FIELDS replaced, e.g. variant(FIRST, seq=0x1236, ts=500000)."""
    out = bytearray(packet)
    for name, value in fields.items():
        offset, size = FIELDS[name]
        out[offset:offset + size] = value.to_bytes(size, "big")
    return bytes(out)


def field(packet, name):
    offset, size = FIELDS[name]
    return int.from_bytes(packet[offset:offset + size], "big")


def configured(packet):
    return bool(packet[6] & 0x40)


def samples(packet):
    header = 12 + (4 + 4 * packet[12] if configured(packet) else 0) + 4
    return (len(packet) - header) // 8


class FakeClock:
    """Seconds; sleep_until jumps to the deadline, or with `undershoot` short of it every other call."""

    def __init__(self):
        self.now, self.undershoot, self.sleeps = 100.0, 0.0, 0

    def __call__(self):
        return self.now

    def sleep_until(self, deadline):
        self.sleeps += 1
        self.now = max(self.now, deadline - (self.undershoot if self.sleeps % 2 else 0))


class FakeSocket:
    def __init__(self, clock, replies=(), fail_sends=0):
        self.clock, self.replies, self.fail_sends = clock, list(replies), fail_sends
        self.sent, self.timeout = [], None            # sent: (datagram, address, send time)

    def sendto(self, data, addr):
        if self.fail_sends:
            self.fail_sends -= 1
            raise OSError("simulated send failure")
        self.sent.append((bytes(data), addr, self.clock()))

    def settimeout(self, t):
        self.timeout = t

    def recvfrom(self, n):
        if self.replies:
            return self.replies.pop(0)
        self.clock.now += self.timeout
        raise socket.timeout()


def stream(replies=(), fail_sends=0, first_seq=SEQ, ts_base=0, lead_in=False, restart_on_late=False, **settings):
    """A stream on fake time and a fake socket: BASE plus `settings`. Returns (stream, socket, clock)."""
    clock = FakeClock()
    sock = FakeSocket(clock, replies, fail_sends)
    st = idn.IDNStream(sock, ADDR, idn.StreamConfig(**{**BASE, **settings}), first_seq=first_seq, ts_base=ts_base,
                       clock=clock, sleep_until=clock.sleep_until, lead_in=lead_in, restart_on_late=restart_on_late)
    return st, sock, clock


def sent(sock):
    return [data for data, _, _ in sock.sent]


def send(points=20, **kw):
    """The datagrams a stream sends for `points`: a list, or a number of blank points."""
    st, sock, _ = stream(**kw)
    st.stream_points([BLANK] * points if isinstance(points, int) else points, loop=False)
    return sent(sock)


class Packets(unittest.TestCase):
    def test_first_packets_and_close(self):
        st, sock, _ = stream()
        stats = st.stream_points([BLANK] * 20, seconds=0.001)    # looped for 1 ms: packets start at 0 and 666 us
        st.close()
        st.close()
        self.assertEqual(sent(sock), [FIRST, SECOND, CLOSE])
        self.assertEqual((stats["packets"], stats["samples"], stats["config_packets"]), (2, 40, 1))   # hwtest reports these

    def test_settings_change_only_their_fields(self):
        for settings, fields in [  # stream settings -> the fields that differ from FIRST
                (dict(service_id=1), dict(service=1)),
                (dict(channel=5, sdm=1), dict(cnl=0xC5, cfl=0x11, scm=0x10)),
                (dict(group=3), dict(group=3))]:
            with self.subTest(settings):
                self.assertEqual(send(**settings)[0], variant(FIRST, **fields))

    def test_sequence_numbers_and_timestamps_wrap(self):
        packets = send(60, first_seq=0xFFFF, ts_base=0xFFFFFD00)
        self.assertEqual([(field(p, "seq"), field(p, "ts")) for p in packets],
                         [(0xFFFF, 0xFFFFFD00), (0x0000, 0xFFFFFF9A), (0x0001, 0x235)])

    def test_configuration_repeats_within_200_ms(self):
        packets = send(300, pps=1000, packet_points=50)          # 50 ms per packet
        self.assertEqual([configured(p) for p in packets], [True, False, False, False, True, False])

    def test_batch_sizes(self):
        self.assertEqual([samples(p) for p in send(65, packet_points=64)], [45, 20])     # no tail under 20
        self.assertEqual([samples(p) for p in send(41)], [20, 21])                       # or merged into the last
        self.assertEqual([samples(p) for p in send(90, packet_points=64)], [64, 26])
        self.assertEqual([samples(p) for p in send(80, pps=200, packet_points=64)], [40, 40])   # at most 200 ms per packet
        largest = send(358, packet_points=179)
        self.assertEqual([samples(p) for p in largest], [179, 179])
        self.assertLessEqual(max(map(len, largest)), 1472)                               # the UDP budget
        self.assertEqual(len(send(budget=196)[0]), 196)                                  # 20 samples fit exactly
        st, sock, _ = stream()
        with self.assertRaises(ValueError):
            st.stream_points([BLANK] * 19, loop=False)
        self.assertEqual(sock.sent, [])

    def test_samples_follow_a_blanked_origin(self):
        lit = LaserPoint(1000, -1000, 0xFFFF, 0x8080, 0x0101, 0)
        packets = send([lit] * 39, lead_in=True)
        self.assertEqual([samples(p) for p in packets], [20, 20])
        self.assertEqual(packets[0][36:52], bytes(8) + bytes.fromhex("03e8 fc18 ff 80 01 00"))   # origin, then X Y R G B I


class Pacing(unittest.TestCase):
    def test_never_sends_early(self):
        st, sock, clock = stream()
        clock.undershoot = 0.0004
        start = clock.now
        st.stream_points([BLANK] * 200, loop=False)
        for data, _, t in sock.sent:
            self.assertGreaterEqual(t, start + field(data, "ts") / 1e6)   # not before its timestamp

    def test_late_sender_keeps_its_timeline_then_closes(self):
        st, sock, clock = stream()
        st.send_batch([BLANK] * 20)
        clock.now += 0.05                     # 50 ms late, under the default 100 ms limit: sent as SECOND
        st.send_batch([BLANK] * 20)
        clock.now += 0.5                      # 500 ms late: close, then refuse to send
        with self.assertRaises(idn.StreamLateError):
            st.send_batch([BLANK] * 20)
        with self.assertRaises(idn.StreamStateError):
            st.send_batch([BLANK] * 20)
        self.assertEqual(sent(sock), [FIRST, SECOND, CLOSE])

    def test_restart_on_late_opens_a_new_epoch(self):
        st, sock, clock = stream(restart_on_late=True)
        st.send_batch([BLANK] * 20)
        clock.now += 0.5
        st.send_batch([BLANK] * 20)
        # Close, then route and configure again on a timeline that follows real time.
        self.assertEqual(sent(sock), [FIRST, variant(CLOSE, seq=SEQ + 1), variant(FIRST, seq=SEQ + 2, ts=500000)])

    def test_failed_send_changes_nothing(self):
        st, sock, _ = stream(fail_sends=1)
        with self.assertRaises(OSError):
            st.send_batch([BLANK] * 20)
        st.send_batch([BLANK] * 20)
        self.assertEqual(sent(sock), [FIRST])


ACK_EB = bytes.fromhex("47 00 1236  0c eb 0000 0002 00 00 00000000")   # ACK of sequence 0x1236: 0xEB, CnlClsd


def patched(data, offset, value):
    return data[:offset] + bytes([value]) + data[offset + 1:]


class Closing(unittest.TestCase):
    def opened(self, replies=()):
        """A stream that has sent FIRST and SECOND, and its socket."""
        st, sock, _ = stream(replies)
        st.stream_points([BLANK] * 40, loop=False)
        return st, sock

    def test_abort_and_a_stream_that_never_sent(self):
        st, sock = self.opened()
        st.abort()
        self.assertEqual(sent(sock)[2:], [bytes.fromhex("46 00 1236")])
        st, sock, _ = stream()
        st.close()
        st.abort()
        self.assertEqual(sock.sent, [])

    def test_close_with_ack(self):
        st, sock = self.opened([(patched(ACK_EB, 5, 0x00), OTHER), (ACK_EB, ADDR)])   # another host's (result 0) first
        self.assertEqual(st.close(ack=True)["result_code"], 0xEB)                        # 0xEB: already closed
        self.assertEqual(sent(sock)[2:], [bytes.fromhex("45 00 1236")])
        st, sock = self.opened()               # unanswered: repeated as empty closes, never as data
        self.assertIsNone(st.close(ack=True, attempts=3, timeout=0.1))
        self.assertEqual(sent(sock)[2:], [bytes.fromhex(f"45 00 {SEQ + k:04x}") for k in (2, 3, 4)])


# Responses to scan sequence 7, service map sequence 9 and RT sequence 5 (Hello 3, 4, 6.3.3).
SCAN = (bytes.fromhex("11 00 0007  28 01 01 00  07 01 123456789abc") + bytes(8)   # 40 octets, v0.1, RT; EUI-48
        + b"ILDAWaveX16".ljust(20, b"\0"))                                       # host name
SVCMAP = bytes.fromhex("13 00 0009  04 18 01 02") + b"".join(        # 4-octet header, 24-octet entries: 1 relay, 2 services
    bytes.fromhex(head) + name.ljust(20, b"\0") for head, name in (("00 00 00 01", b"relay"),    # relay 1
                                                                   ("01 80 01 00", b"laser"),    # default, at the root
                                                                   ("02 80 00 01", b"remote")))  # behind relay 1
ACK = bytes.fromhex("47 00 0005  0c 00 0051 0401 05 00 00000000")   # 12 octets, result 0; flag bits named below


class Management(unittest.TestCase):
    def sender(self, replies=(), seq=7, **kw):
        clock = FakeClock()
        sock = FakeSocket(clock, replies)
        s = idn.IDNSender(ADDR[0], sock=sock, clock=clock, sleep_until=clock.sleep_until, **kw)
        s.mgmt_seq = seq
        return s, sock

    def test_requests_and_ping(self):
        s, sock = self.sender()
        s.scan(timeout=0.1)
        s.svcmap(timeout=0.1)
        self.assertEqual(sent(sock), [bytes.fromhex("10 00 0007"), bytes.fromhex("12 00 0008")])
        echo = bytes.fromhex("09 03 1234 deadbeef")
        s, sock = self.sender([(echo, ADDR)], group=3, seq=0x1234)
        self.assertIsNotNone(s.ping(bytes.fromhex("deadbeef"), timeout=1.0))
        self.assertEqual(sent(sock), [bytes.fromhex("08 03 1234 deadbeef")])
        s, _ = self.sender([(echo[:-1], ADDR)], group=3, seq=0x1234)
        with self.assertRaises(idn.IDNResponseError):
            s.ping(bytes.fromhex("deadbeef"), timeout=1.0)
        self.assertIsNotNone(self.sender([(bytes.fromhex("09 00 0007"), ADDR)])[0].ping(b"", timeout=1.0))
        self.assertIsNone(self.sender()[0].ping(b"", timeout=0.2))

    def test_responses(self):
        self.assertEqual(self.sender([(SCAN, ADDR)])[0].scan(), [{
            "struct_size": 40, "protocol_version": "0.1", "status": 1, "realtime": True, "malfunction": False,
            "offline": False, "excluded": False, "occupied": False, "unit_id": "01-123456789ABC",
            "host_name": "ILDAWaveX16", "source": "192.0.2.50:7255"}])
        self.assertEqual(self.sender([(SVCMAP, ADDR)], seq=9)[0].svcmap(), {
            "relays": [{"relay_number": 1, "name": "relay"}],
            "services": [{"service_id": 1, "service_type": 0x80, "default": True, "relay_number": 0, "name": "laser"},
                         {"service_id": 2, "service_type": 0x80, "default": False, "relay_number": 1, "name": "remote"}]})
        self.assertEqual(idn.parse_ack(ACK, 0, 5), {
            "struct_size": 12, "result_code": 0, "input_events": ["SeqErr:missing", "SeqErr:not-monotonic", "New"],
            "pipeline_events": ["CTYErr", "CnlRted"], "status": ["SNMsg", "SOCnl"], "link_quality": 0, "latency_us": 0})
        for parse, data, seq, reason in [
                (idn.parse_scan_response, patched(SCAN, 4, 32), 7, "struct-size"),       # struct shorter than 40
                (idn.parse_scan_response, SCAN[:30], 7, "truncated"),
                (idn.parse_svcmap_response, patched(SVCMAP, 7, 3), 9, "truncated"),     # 3 services, 2 sent
                (idn.parse_svcmap_response, patched(SVCMAP, 59, 2), 9, "relay-number"),  # relay 2 does not exist
                (idn.parse_ack, patched(ACK, 4, 8), 5, "struct-size"),
                (idn.parse_ack, ACK, 4, "sequence")]:
            with self.subTest(reason):
                with self.assertRaises(idn.IDNResponseError) as cm:
                    parse(data, 0, seq)
                self.assertEqual(cm.exception.reason, reason)

    def test_scan_ignores_unrelated_datagrams(self):
        s, _ = self.sender([(SCAN, OTHER),                              # another host
                            (patched(SCAN, 3, 8), ADDR),                # another sequence
                            (patched(SCAN, 1, 2), ADDR),                # another client group
                            (bytes.fromhex("09 00 0007"), ADDR),        # another command
                            (SCAN, ADDR)])
        self.assertEqual([u["source"] for u in s.scan(timeout=1.0)], ["192.0.2.50:7255"])
        s, _ = self.sender([(SCAN, OTHER), (SCAN, ADDR)], broadcast=True)        # a broadcast scan: every unit
        self.assertEqual([u["source"] for u in s.scan(timeout=1.0)], ["192.0.2.99:7255", "192.0.2.50:7255"])

    def test_management_keeps_its_own_sequence(self):
        s, sock = self.sender()
        st = s.open_stream(**BASE, first_seq=SEQ, ts_base=0, lead_in=False)
        st.send_batch([BLANK] * 20)
        s.ping(b"", timeout=0.05)
        st.send_batch([BLANK] * 20)
        self.assertEqual(sent(sock), [FIRST, bytes.fromhex("08 00 0007"), SECOND])


class Settings(unittest.TestCase):
    def test_bad_settings_are_rejected_not_masked(self):
        for bad in (dict(channel=64), dict(channel=-1), dict(group=16), dict(sdm=4), dict(service_id=256),
                    dict(pps=0), dict(packet_points=19), dict(packet_points=180), dict(budget=195),
                    dict(service_mode=2), dict(config_interval_us=0), dict(config_interval_us=300000),
                    dict(max_late_s=0), dict(pps=50)):
            with self.subTest(bad), self.assertRaises(ValueError):
                idn.StreamConfig(**{**BASE, **bad})
        for build in (lambda: idn.build_hello_header(0x40, 16, 0), lambda: idn.build_channel_message(64, 1, 0),
                      lambda: idn.build_wave_chunk(0, 0, bytes(160)),
                      lambda: idn.build_config_header(4, routing=False, service_id=1),
                      lambda: idn.IDNSender(ADDR[0], group=16, sock=FakeSocket(FakeClock()))):
            with self.assertRaises(ValueError):
                build()

    def test_builders_for_void_and_channel_close(self):
        void = idn.build_hello_header(0x40, 0, 1) + idn.build_channel_message(
            0, idn.CHUNK_VOID, 0, idn.build_config_header(4, service_id=1) + idn.xyrgbi_dictionary())
        self.assertEqual(void, bytes.fromhex("40 00 0001  001c c0 00 00000000  04 01 01 01") + FIRST[16:32])
        close = idn.build_hello_header(0x40, 0, 0x1236) + idn.build_channel_message(
            0, idn.CHUNK_VOID, 1333, idn.build_config_header(0, close=True, routing=False, service_mode=0))
        self.assertEqual(close, bytes.fromhex("40 00 1236  000c c0 00 00000535  00 02 00 00"))

    def test_cli(self):
        def parse(*argv):
            return idn.build_arg_parser().parse_args(["--host", "x", *argv])
        for argv in (["square"], ["circle"], ["sine"], ["ild", "show.ild"], ["scan"], ["svcmap"], ["ping"]):
            self.assertEqual(parse(*argv).cmd, argv[0])
        args = parse("--channel", "63", "--group", "0x0f", "circle")
        self.assertEqual((args.channel, args.group), (63, 15))
        for argv in (["--channel", "64", "circle"], ["--group", "16", "scan"], ["--packet-points", "19", "circle"],
                     ["stop"]):
            with self.subTest(argv), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse(*argv)


if __name__ == "__main__":
    unittest.main()
