"""Sender and response-parser tests against the literal fixtures in tests/idn/fixtures.

Standard library only. Expected bytes always come from the fixture files, never
from the sender itself. Run through tools/test_idn.py, or directly:

  python -m unittest discover -s tests/idn
"""

import glob
import io
import json
import os
import socket
import sys
import unittest
from contextlib import redirect_stderr

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "tools"))

import idn_sender as idn  # noqa: E402
from ilw_sender import LaserPoint  # noqa: E402

ADDR = ("192.0.2.50", idn.IDN_PORT)


def _load():
    by_id = {}
    for path in glob.glob(os.path.join(HERE, "fixtures", "*.json")):
        with open(path, encoding="utf-8") as f:
            for fx in json.load(f)["fixtures"]:
                by_id[fx["id"]] = fx
    return by_id


FX = _load()
P = FX["sender-xyrgbi-40-at-30k"]["sender"]   # the stream most tests start from
SEQ = P["first_seq"]


def literal(obj) -> bytes:
    return bytes.fromhex("".join(s["hex"] for s in obj["segments"]))


def fx_bytes(fid: str, seq=None, ts=None) -> bytes:
    """A fixture datagram, optionally with another Hello sequence number or RT timestamp."""
    data = bytearray(literal(FX[fid]))
    if seq is not None:
        data[2:4] = (seq & 0xFFFF).to_bytes(2, "big")
    if ts is not None:
        data[8:12] = (ts & 0xFFFFFFFF).to_bytes(4, "big")
    return bytes(data)


def wave_samples(datagram: bytes) -> int:
    """Samples in an RT wave packet, read from its own headers (Hello 2.1, Stream 2.1, 2.2, 6.1)."""
    n = 4 + 8                      # Hello header, channel message header
    if datagram[6] & 0x40:         # CCLF: configuration header and SCWC words
        n += 4 + 4 * datagram[n]
    return (len(datagram) - n - 4) // 8


class FakeClock:
    """Monotonic seconds; sleep_until jumps to the deadline (or short of it)."""

    def __init__(self, start=100.0, undershoot=0.0):
        self.now = start
        self.undershoot = undershoot
        self.sleeps = 0

    def __call__(self):
        return self.now

    def sleep_until(self, deadline):
        self.sleeps += 1
        target = deadline - self.undershoot if self.undershoot and self.sleeps % 2 else deadline
        self.now = max(self.now, target)


class FakeSocket:
    def __init__(self, clock=None, replies=None, fail_sends=0):
        self.sent = []
        self.clock = clock
        self.replies = list(replies or [])
        self.fail_sends = fail_sends
        self.timeouts = []

    def sendto(self, data, addr):
        if self.fail_sends:
            self.fail_sends -= 1
            raise OSError("simulated send failure")
        self.sent.append((bytes(data), addr, self.clock() if self.clock else None))
        return len(data)

    def settimeout(self, t):
        self.timeouts.append(t)

    def recvfrom(self, n):
        if self.replies:
            return self.replies.pop(0)
        if self.clock:
            self.clock.now += self.timeouts[-1] if self.timeouts else 10.0
        raise socket.timeout()

    def close(self):
        pass


def make_stream(params=P, clock=None, sock=None, **kw):
    clock = clock or FakeClock()
    sock = sock or FakeSocket(clock)
    cfg = idn.StreamConfig(pps=params["pps"], packet_points=params["packet_points"], channel=params["channel"],
                           service_id=params["service_id"], sdm=params["sdm"], group=params["group"],
                           budget=params["budget"])
    st = idn.IDNStream(sock, ADDR, cfg, first_seq=params["first_seq"], ts_base=params["ts_base"],
                       clock=clock, sleep_until=clock.sleep_until, lead_in=params["lead_in"], **kw)
    return st, sock, clock


def sender_input(params=P):
    if "input_points" in params:
        return [LaserPoint(*p) for p in params["input_points"]]
    return [LaserPoint(0, 0, 0, 0, 0, 0) for _ in range(params["input_blank_count"])]


class SenderBytes(unittest.TestCase):
    """Whole-packet and header-prefix comparisons with literal fixtures."""

    def test_sender_sequence_fixtures(self):
        for fid, f in FX.items():
            if f["kind"] != "sender-sequence":
                continue
            with self.subTest(fixture=fid):
                p = f["sender"]
                st, sock, clock = make_stream(p)
                origin = clock.now
                st.stream_points(sender_input(p), loop=False)
                if p.get("close", True):
                    st.close()
                self.assertEqual(len(sock.sent), len(f["packets"]), "packet count")
                for k, (want, (got, addr, t)) in enumerate(zip(f["packets"], sock.sent)):
                    self.assertEqual(addr, ADDR)
                    if "ref" in want:
                        self.assertEqual(got.hex(), fx_bytes(want["ref"]).hex(), f"packet {k} ({want['ref']})")
                    else:
                        prefix = literal(want)
                        self.assertEqual(len(got), want["datagram_length"], f"packet {k} length")
                        self.assertEqual(got[:len(prefix)].hex(), prefix.hex(), f"packet {k} header")
                    if want["send_at_s"] is not None:
                        self.assertGreaterEqual(t - origin, want["send_at_s"] - 1e-12, f"packet {k} sent early")
                        self.assertAlmostEqual(t - origin, want["send_at_s"], places=9, msg=f"packet {k} send time")

    def test_lead_in_is_blanked_origin_and_counted(self):
        st, sock, _ = make_stream(dict(P, lead_in=True))
        stats = st.stream_points(sender_input()[1:], loop=False)  # the lead-in replaces the fixture's origin sample
        self.assertEqual(sock.sent[0][0], fx_bytes("rt-first-config-wave-xyrgbi-20"))
        self.assertEqual(stats["samples"], 40)


class Batching(unittest.TestCase):
    def test_plan_batches_table(self):
        for c in FX["batching-plan"]["cases"]:
            with self.subTest(c=c):
                if "expect_error" in c:
                    with self.assertRaises(ValueError):
                        idn.plan_batches(c["total"], c["preferred"], c["capacity"], c["minimum"])
                else:
                    got = idn.plan_batches(c["total"], c["preferred"], c["capacity"], c["minimum"])
                    self.assertEqual(got, c["expect"])

    def test_capacity_table(self):
        for c in FX["packet-capacity"]["cases"]:
            with self.subTest(c=c):
                if "expect_error" in c:
                    with self.assertRaises(ValueError):
                        idn.packet_capacity(c["budget"], c["sample_bytes"], c["dictionary_bytes"])
                    continue
                self.assertEqual(idn.packet_capacity(c["budget"], c["sample_bytes"], c["dictionary_bytes"], True),
                                 c["expect_with_config"])
                self.assertEqual(idn.packet_capacity(c["budget"], c["sample_bytes"], c["dictionary_bytes"], False),
                                 c["expect_without_config"])

    def test_cumulative_timing_table(self):
        for c in FX["cumulative-timing"]["cases"]:
            with self.subTest(c=c):
                n, offs, durs = 0, [], []
                for b in c["batches"]:
                    offs.append(idn.boundary_us(n, c["pps"]))
                    durs.append(idn.boundary_us(n + b, c["pps"]) - offs[-1])
                    n += b
                self.assertEqual(offs, c["expect_offsets_us"])
                self.assertEqual(durs, c["expect_durations_us"])

    def test_max_size_packet_fits_budget(self):
        st, sock, _ = make_stream(dict(P, packet_points=179))
        st.stream_points([LaserPoint(0, 0)] * 358, loop=False)
        sent = [d for d, _, _ in sock.sent]
        self.assertEqual([wave_samples(d) for d in sent], [179, 179])
        self.assertLessEqual(max(map(len, sent)), P["budget"])
        with self.assertRaises(ValueError):
            make_stream(dict(P, packet_points=180))

    def test_ilda_frame_ends_do_not_make_short_packets(self):
        st, sock, _ = make_stream(dict(P, packet_points=64))
        frames = [[LaserPoint(0, 0)] * 30] * 3  # 90 points
        st.stream(pt for fr in frames for pt in fr)
        self.assertEqual([wave_samples(d) for d, _, _ in sock.sent], [64, 26])

    def test_below_minimum_finite_input_is_an_error(self):
        st, sock, _ = make_stream()
        with self.assertRaises(ValueError):
            st.stream_points([LaserPoint(0, 0)] * 19, loop=False)
        self.assertEqual(sock.sent, [])


class Pacing(unittest.TestCase):
    def test_never_sends_early_even_if_sleep_returns_early(self):
        clock = FakeClock(undershoot=0.0004)
        st, sock, _ = make_stream(clock=clock)
        origin = clock.now
        st.stream_points([LaserPoint(0, 0)] * 200, loop=False)
        for k, (_, _, t) in enumerate(sock.sent):
            self.assertGreaterEqual(t, origin + idn.boundary_us(k * P["packet_points"], P["pps"]) / 1e6)

    def test_small_lateness_keeps_the_authored_timeline(self):
        st, sock, clock = make_stream()
        pts = sender_input()
        st.send_batch(pts[:20])
        clock.now += 0.05  # 50 ms late, under the 100 ms limit
        st.send_batch(pts[20:40])
        self.assertEqual(sock.sent[1][0], fx_bytes("rt-second-wave-xyrgbi-20"))

    def test_too_late_closes_and_raises(self):
        st, sock, clock = make_stream()
        pts = sender_input()
        st.send_batch(pts[:20])
        clock.now += 0.5
        with self.assertRaises(idn.StreamLateError):
            st.send_batch(pts[20:40])
        self.assertEqual(sock.sent[-1][0], fx_bytes("rt-connection-close-empty", seq=SEQ + 1))
        with self.assertRaises(idn.StreamStateError):
            st.send_batch(pts[20:40])

    def test_too_late_restart_opens_a_fresh_epoch(self):
        st, sock, clock = make_stream(restart_on_late=True)
        pts = sender_input()
        st.send_batch(pts[:20])
        clock.now += 0.5
        st.send_batch(pts[:20])
        self.assertEqual(sock.sent[1][0], fx_bytes("rt-connection-close-empty", seq=SEQ + 1))
        # The new epoch routes and configures again; its timestamps follow real time (0 + 500 ms).
        self.assertEqual(sock.sent[2][0], fx_bytes("rt-first-config-wave-xyrgbi-20", seq=SEQ + 2, ts=500000))
        self.assertEqual(st.stats["epochs"], 2)

    def test_failed_send_does_not_advance_state(self):
        clock = FakeClock()
        st, sock, _ = make_stream(clock=clock, sock=FakeSocket(clock, fail_sends=1))
        pts = sender_input()
        with self.assertRaises(OSError):
            st.send_batch(pts[:20])
        st.send_batch(pts[:20])
        self.assertEqual([d for d, _, _ in sock.sent], [fx_bytes("rt-first-config-wave-xyrgbi-20")])


class CloseAbort(unittest.TestCase):
    def opened(self, replies=None):
        """A stream that has sent the two data packets of sender-xyrgbi-40-at-30k."""
        clock = FakeClock()
        st, sock, _ = make_stream(clock=clock, sock=FakeSocket(clock, replies))
        pts = sender_input()
        st.send_batch(pts[:20])
        st.send_batch(pts[20:40])
        return st, sock

    def test_close_on_same_socket(self):
        st, sock = self.opened()
        st.close()
        st.close()
        self.assertEqual(sock.sent[-1][0], fx_bytes("rt-connection-close-empty"))
        self.assertEqual(len(sock.sent), 3, "closing twice sends one close")

    def test_abort(self):
        st, sock = self.opened()
        st.abort()
        self.assertEqual(sock.sent[-1][0], fx_bytes("rt-abort"))
        self.assertEqual(st.state, st.CLOSED)

    def test_never_sent_stream_sends_no_close(self):
        st, sock, _ = make_stream()
        st.close()
        st.abort()
        self.assertEqual(sock.sent, [])

    def test_close_with_ack_accepts_0xeb(self):
        eb = fx_bytes("ack-eb-close-without-connection", seq=SEQ + 2)
        st, sock = self.opened(replies=[(eb, ("192.0.2.99", idn.IDN_PORT)), (eb, ADDR)])  # a stray copy first
        ack = st.close(ack=True)
        self.assertEqual(sock.sent[-1][0], fx_bytes("rt-connection-close-ack-empty"))
        self.assertEqual(ack["result_code"], 0xEB)
        self.assertEqual(st.state, st.CLOSED)

    def test_unanswered_close_repeats_without_data(self):
        st, sock = self.opened()
        self.assertIsNone(st.close(ack=True, attempts=3, timeout=0.1))
        self.assertEqual([d for d, _, _ in sock.sent[2:]],
                         [fx_bytes("rt-connection-close-ack-empty", seq=SEQ + 2 + k) for k in range(3)])


class Management(unittest.TestCase):
    PARSERS = {"scan-req": idn.parse_scan_response, "svcmap-req": idn.parse_svcmap_response}

    def sender(self, replies=(), **kw):
        clock = FakeClock()
        sock = FakeSocket(clock, list(replies))
        return idn.IDNSender(ADDR[0], sock=sock, clock=clock, sleep_until=clock.sleep_until, **kw), sock

    def parse(self, f):
        """Parse a server-to-client fixture against the request it answers."""
        exp, data = f["expect"], literal(f)
        if "request" not in exp:
            return idn.parse_ack(data, 0, exp["request_sequence"])
        req = fx_bytes(exp["request"])
        group, seq = req[1] & 0x0F, int.from_bytes(req[2:4], "big")
        if exp["request"].startswith("ping"):
            return idn.parse_ping_response(data, req[4:], group, seq)
        return self.PARSERS[exp["request"]](data, group, seq)

    def test_response_fixtures(self):
        for fid, f in FX.items():
            if f["kind"] != "datagram" or f["direction"] != "server-to-client":
                continue
            exp = f["expect"]
            with self.subTest(fixture=fid):
                if exp["outcome"] == "parse_ok":
                    got = self.parse(f)
                    if "parsed" in exp:
                        self.assertEqual(got, exp["parsed"])
                else:
                    with self.assertRaises(idn.IDNResponseError) as cm:
                        self.parse(f)
                    self.assertEqual(cm.exception.reason, exp["reason"])

    def test_requests_match_fixture_bytes(self):
        s, sock = self.sender()
        s.mgmt_seq = 0x0007
        s.scan(timeout=0.1)
        self.assertEqual(sock.sent[0][0], fx_bytes("scan-req"))
        s.mgmt_seq = 0x0009
        s.svcmap(timeout=0.1)
        self.assertEqual(sock.sent[1][0], fx_bytes("svcmap-req"))
        s.group, s.mgmt_seq = 3, 0x1234
        s.ping(bytes.fromhex("deadbeef"), timeout=0.1)
        self.assertEqual(sock.sent[2][0], fx_bytes("ping-req-payload"))

    def test_transaction_ignores_unrelated_datagrams(self):
        good = fx_bytes("scan-resp-valid")
        replies = [(good, ("192.0.2.99", 7255)),             # other host
                   (fx_bytes("scan-resp-wrong-sequence"), ADDR),
                   (fx_bytes("scan-resp-wrong-group"), ADDR),
                   (fx_bytes("ping-resp-empty"), ADDR),      # other command
                   (good, ADDR)]
        s, _ = self.sender(replies)
        s.mgmt_seq = 0x0007
        units = s.scan(timeout=1.0)
        self.assertEqual(len(units), 1)
        self.assertEqual(units[0]["unit_id"], "01-123456789ABC")
        self.assertEqual(units[0]["source"], f"{ADDR[0]}:{ADDR[1]}")

    def test_ping_exact_echo_and_timeout(self):
        s, _ = self.sender([(fx_bytes("ping-resp-payload"), ADDR)], group=3)
        s.mgmt_seq = 0x1234
        self.assertIsNotNone(s.ping(bytes.fromhex("deadbeef"), timeout=1.0))
        s, _ = self.sender([(fx_bytes("ping-resp-short-echo"), ADDR)], group=3)
        s.mgmt_seq = 0x1234
        with self.assertRaises(idn.IDNResponseError):
            s.ping(bytes.fromhex("deadbeef"), timeout=1.0)
        s, _ = self.sender([])
        self.assertIsNone(s.ping(b"", timeout=0.2))

    def test_management_does_not_touch_rt_sequence(self):
        s, sock = self.sender()
        st = s.open_stream(P["pps"], P["packet_points"], first_seq=SEQ, ts_base=0, lead_in=False)
        pts = sender_input()
        st.send_batch(pts[:20])
        s.ping(b"", timeout=0.05)
        st.send_batch(pts[20:40])
        rt = [d for d, _, _ in sock.sent if d[0] == idn.IDNCMD_RT_CHANNEL]
        self.assertEqual(rt, [fx_bytes("rt-first-config-wave-xyrgbi-20"), fx_bytes("rt-second-wave-xyrgbi-20")])


class Validation(unittest.TestCase):
    def test_rejects_instead_of_masking(self):
        bad = [dict(channel=64), dict(channel=-1), dict(group=16), dict(sdm=4), dict(service_id=256),
               dict(pps=0), dict(pps=-1), dict(packet_points=19), dict(packet_points=180), dict(budget=100),
               dict(service_mode=2), dict(config_interval_us=0), dict(config_interval_us=300000),
               dict(max_late_s=0), dict(pps=50)]
        for kw in bad:
            with self.subTest(kw=kw):
                with self.assertRaises(ValueError):
                    idn.StreamConfig(**{"pps": 30000, "packet_points": 64, **kw})
        with self.assertRaises(ValueError):
            idn.build_hello_header(0x40, 16, 0)
        with self.assertRaises(ValueError):
            idn.build_channel_message(64, 1, 0)
        with self.assertRaises(ValueError):
            idn.build_wave_chunk(0, 0, bytes(160))
        with self.assertRaises(ValueError):
            idn.build_config_header(4, routing=False, service_id=1)
        with self.assertRaises(ValueError):
            idn.IDNSender(ADDR[0], group=16, sock=FakeSocket())

    def test_low_rate_caps_batch_by_config_cadence(self):
        cfg = idn.StreamConfig(pps=200, packet_points=64)
        self.assertEqual(cfg.effective_packet_points, 40)  # 200 ms at 200 pps

    def test_builders_reproduce_routing_void_and_channel_close(self):
        tags = idn.xyrgbi_dictionary()
        void = idn.build_hello_header(0x40, 0, 1) + idn.build_channel_message(
            0, idn.CHUNK_VOID, 0, idn.build_config_header(4, service_id=1) + tags)
        self.assertEqual(void, fx_bytes("rt-routing-void-xyrgbi"))
        close = idn.build_hello_header(0x40, 0, 0x1236) + idn.build_channel_message(
            0, idn.CHUNK_VOID, 1333, idn.build_config_header(0, close=True, routing=False, service_mode=0))
        self.assertEqual(close, fx_bytes("rt-close-channel-void"))


class Cli(unittest.TestCase):
    def parse(self, *argv):
        return idn.build_arg_parser().parse_args(["--host", "x", *argv])

    def test_cli_accepts_real_commands(self):
        for argv in (["square"], ["circle"], ["sine"], ["ild", "show.ild"], ["scan"], ["svcmap"], ["ping"]):
            with self.subTest(argv=argv):
                self.assertEqual(self.parse(*argv).cmd, argv[0])
        args = self.parse("--channel", "63", "--group", "15", "circle")
        self.assertEqual((args.channel, args.group), (63, 15))

    def test_cli_rejects_out_of_range_ids_and_stop(self):
        for argv in (["--channel", "64", "circle"], ["--group", "16", "scan"], ["--packet-points", "19", "circle"],
                     ["stop"]):
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    self.parse(*argv)


if __name__ == "__main__":
    unittest.main()
