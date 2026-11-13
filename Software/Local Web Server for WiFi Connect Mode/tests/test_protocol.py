# SPDX-License-Identifier: MIT
"""relay-pc/dsentry/protocol.py (generated) against the spec, the contract and the docs."""

import re
import struct
import unittest

import _support as S
from dsentry import protocol as P

SPEC = S.load_spec()
SIZES = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "u32": 4, "i32": 4}


def payload_len(packet):
    total = 0
    for f in packet["fields"]:
        t = f["type"]
        total += SIZES[t] if t in SIZES else int(t.rstrip("]").split("[")[1])
    return total


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(P.crc8(b"123456789"), 0xF4)
        self.assertEqual(P.DS_CRC8_CHECK, 0xF4)

    def test_known_vectors(self):
        self.assertEqual(P.crc8(b""), 0x00)
        self.assertEqual(P.crc8(b"\x00"), 0x00)
        self.assertEqual(P.crc8(b"\x01"), 0x07)     # poly 0x07, no reflection, no xor-out
        self.assertEqual(P.crc8(b"\x80"), 0x89)


class TestLayout(unittest.TestCase):
    def test_contract_sizes(self):
        # architecture.md 4.1: ControlPacket payload 10, StatusPacket payload 7
        self.assertEqual((P.ControlPacket.START, P.ControlPacket.LEN, P.ControlPacket.FRAME_LEN), (0xA5, 10, 13))
        self.assertEqual((P.StatusPacket.START, P.StatusPacket.LEN, P.StatusPacket.FRAME_LEN), (0x5A, 7, 10))
        self.assertEqual(P.TelemetryPacket.START, 0xD5)
        self.assertEqual(P.BeaconPacket.START, 0xD6)
        self.assertEqual(P.BeaconPacket.LEN, 8)
        self.assertEqual(P.DS_MAX_FRAME_LEN, P.TelemetryPacket.FRAME_LEN)

    def test_matches_spec(self):
        self.assertEqual([c.__name__ for c in P.PACKET_CLASSES], [p["name"] for p in SPEC.PACKETS])
        for p in SPEC.PACKETS:
            cls = getattr(P, p["name"])
            self.assertEqual(cls.START, p["start"])
            self.assertEqual(cls.LEN, payload_len(p))
            self.assertEqual(cls.FRAME_LEN, payload_len(p) + 3)
            self.assertEqual(cls._STRUCT.size, cls.LEN)
            self.assertEqual([f.name for f in P.dataclasses.fields(cls)], [f["name"] for f in p["fields"]])
        for name, value, _doc in SPEC.CONSTANTS:
            self.assertEqual(getattr(P, name), value, name)
        self.assertEqual(P.DS_ANGLE_SCALE, 100)
        self.assertEqual(P.DS_TELEMETRY_VERSION, 2)
        self.assertEqual(P.DS_PROTO_VERSION, 1)

    def test_telemetry_v2_appends_the_controller_block(self):
        # version 1 ended with lcd at payload offset 59 (91 bytes); version 2 appends 16 bytes
        names = [f.name for f in P.dataclasses.fields(P.TelemetryPacket)]
        self.assertEqual(names[names.index("lcd") + 1:],
                         ["padState", "padBattery", "padLx", "padLy", "padRx", "padRy", "padLt", "padRt",
                          "padButtons"])
        self.assertEqual((P.DS_TELEMETRY_V1_LEN, P.DS_TELEMETRY_V1_FRAME_LEN), (91, 94))
        self.assertEqual((P.TelemetryPacket.LEN, P.TelemetryPacket.FRAME_LEN), (107, 110))
        self.assertEqual(P.DS_AT_DST_LINE_LEN, 229)
        self.assertEqual(P.TelemetryPacket.LEGACY, {91: 1})
        self.assertEqual(P.TelemetryPacket.layout_version(107), 2)
        self.assertEqual(P.TelemetryPacket.layout_version(91), 1)
        self.assertIsNone(P.TelemetryPacket.layout_version(90))
        self.assertEqual(P.ControlPacket.LEGACY, {})
        t = P.TelemetryPacket()
        self.assertEqual((t.padState, t.padBattery), (P.PadState.NONE, P.DS_PAD_BATTERY_UNKNOWN))
        self.assertEqual((P.PadState.NONE, P.DS_PAD_BATTERY_UNKNOWN), (255, 255))
        # PadState is the fork's xbox_r4_conn_state, value for value
        self.assertEqual([(m.name, m.value) for m in P.PadState][:6],
                         [("CONNECTED", 0), ("WAITING_FIRST_REPORT", 1), ("FOUND", 2), ("SCANNING", 3),
                          ("OFF", 4), ("BLOCKED_BY_HCI", 5)])
        self.assertEqual([(k, m) for k, m in P.BIT_KEYS["Buttons"]][12:],
                         [("lsClick", 1 << 12), ("rsClick", 1 << 13), ("xbox", 1 << 14), ("share", 1 << 15)])


def v1_frame(**fields) -> bytes:
    """A version 1 TelemetryPacket as an older RA4M1 build sends it: the v2 layout's first 91
    payload bytes with version 1, framed with its own len and CRC."""
    payload = P.TelemetryPacket(**dict(P.EXAMPLES["TelemetryPacket"], version=1, **fields)).payload()
    return P._frame(P.DS_TELEMETRY_START, payload[:P.DS_TELEMETRY_V1_LEN])


class TestLegacyTelemetry(unittest.TestCase):
    def test_v1_frame_decodes_only_when_asked(self):
        frame = v1_frame()
        self.assertEqual((len(frame), frame[1]), (94, 91))
        with self.assertRaises(ValueError):
            P.TelemetryPacket.decode(frame)
        pkt = P.TelemetryPacket.decode(frame, accept_legacy=True)
        ex = P.EXAMPLES["TelemetryPacket"]
        self.assertEqual(pkt.version, 1)
        self.assertEqual((pkt.panCdeg, pkt.lcd, pkt.ledG), (ex["panCdeg"], ex["lcd"], ex["ledG"]))
        # the fields version 1 lacks keep their defaults: no source, unknown, neutral
        self.assertEqual((pkt.padState, pkt.padBattery), (P.DS_PAD_STATE_NONE, P.DS_PAD_BATTERY_UNKNOWN))
        self.assertEqual((pkt.padLx, pkt.padRx, pkt.padRt, pkt.padButtons), (0, 0, 0, 0))
        # a v2 frame cut to 91 bytes but with a bad CRC is still refused
        bad = bytearray(frame)
        bad[-1] ^= 1
        with self.assertRaises(ValueError):
            P.TelemetryPacket.decode(bytes(bad), accept_legacy=True)

    def test_deframer_reads_v1_only_with_accept_legacy(self):
        v2 = P.TelemetryPacket(**P.EXAMPLES["TelemetryPacket"]).encode()
        stream = b"[2] boot\r\n" + v1_frame(seq=1) + b"text\n" + v2 + v1_frame(seq=3)
        strict = P.Deframer()
        self.assertEqual([f.raw for f in strict.feed(stream)], [v2])
        self.assertGreater(strict.len_errors, 0)
        legacy = P.Deframer(accept_legacy=True)
        got = []
        for i in range(0, len(stream), 7):              # any split of the stream
            got += legacy.feed(stream[i:i + 7])
        self.assertEqual([(f.packet.version, f.packet.seq) for f in got], [(1, 1), (2, 200), (1, 3)])
        self.assertEqual(legacy.len_errors, 0)
        self.assertEqual(legacy.crc_errors, 0)

    def test_enums_and_bits_match_spec(self):
        for e in SPEC.ENUMS:
            cls = getattr(P, e["name"])
            self.assertEqual([(m.name, m.value) for m in cls], [(n, v) for n, v, _d in e["values"]])
            for n, v, _d in e["values"]:
                self.assertEqual(getattr(P, e["prefix"] + n), v)
        for b in SPEC.BITS:
            self.assertEqual(P.BIT_KEYS[b["name"]], tuple((k, 1 << bit) for k, bit, _d in b["bits"]))
        self.assertEqual(P.DS_BTN_DPAD_LEFT, 1 << 8)
        self.assertEqual(P.DS_TELEM_FLAG_ESP_ALIVE, 1 << 14)
        self.assertEqual(P.TurretState.FAULT, 6)
        self.assertEqual(P.EspMode.UNKNOWN, 255)

    def test_at_commands(self):
        self.assertEqual(P.DS_AT_DSCAPS, "+DSCAPS")
        self.assertEqual([c["request"] for c in P.AT_COMMANDS],
                         ["AT+DSCAPS?", "AT+DSS=<wantMode>,<rumbleSeq>,<strong>,<weak>,<rumbleMs>,<state>",
                          "AT+DST=<hex>"])
        self.assertEqual(P.DS_AT_DSS_ARGC, 6)
        self.assertEqual(P.DS_AT_DSS_ARG_RUMBLE_MS, 4)
        self.assertEqual(P.DS_AT_DST_LINE_LEN, len("AT+DST=") + 2 * P.TelemetryPacket.FRAME_LEN + 2)


class TestCodec(unittest.TestCase):
    def test_control_example_matches_research_prototype(self):
        # research/pc/udp_probe_codec.py, an independent implementation, produced this frame
        frame = P.ControlPacket(**P.EXAMPLES["ControlPacket"]).encode()
        self.assertEqual(frame, bytes.fromhex("a50a0300feff01ff03110807ac"))

    def test_examples_equal_spec(self):
        for p in SPEC.PACKETS:
            ex = dict(p["example"])
            self.assertEqual(P.EXAMPLES[p["name"]], {k: (tuple(v) if isinstance(v, (tuple, list)) else v)
                                                     for k, v in ex.items()})

    def test_round_trip_examples_and_extremes(self):
        rng = S.seeded()
        for cls in P.PACKET_CLASSES:
            cases = [P.EXAMPLES[cls.__name__]] + [S.random_values(SPEC, cls.__name__, rng, extreme=e)
                                                  for e in ("lo", "hi")]
            for values in cases:
                pkt = cls(**values)
                frame = pkt.encode()
                self.assertEqual(len(frame), cls.FRAME_LEN)
                self.assertEqual(frame[0], cls.START)
                self.assertEqual(frame[1], cls.LEN)
                self.assertEqual(frame[-1], P.crc8(frame[1:-1]))
                self.assertEqual(cls.decode(frame), pkt)
                self.assertEqual(P.decode_frame(frame), pkt)

    def test_round_trip_random(self):
        rng = S.seeded(7)
        for _ in range(300):
            cls = rng.choice(P.PACKET_CLASSES)
            pkt = cls(**S.random_values(SPEC, cls.__name__, rng))
            self.assertEqual(cls.decode(pkt.encode()), pkt)

    def test_little_endian(self):
        frame = P.ControlPacket(rx=0x0102, ry=-2, rt=0x0304, buttons=0x0506).encode()
        self.assertEqual(frame[3:5], b"\x02\x01")
        self.assertEqual(frame[5:7], b"\xfe\xff")
        self.assertEqual(frame[7:9], b"\x04\x03")
        self.assertEqual(frame[9:11], b"\x06\x05")

    def test_out_of_range_refused(self):
        with self.assertRaises(struct.error):
            P.ControlPacket(rx=40000).encode()
        with self.assertRaises(ValueError):
            P.BeaconPacket(ip=(1, 2, 3)).encode()

    def test_lcd_text(self):
        t = P.TelemetryPacket(lcd="HI")
        frame = t.encode()
        lcd_off = 2 + P.DS_TELEMETRY_V1_LEN - P.DS_TELEMETRY_LCD_LEN    # lcd: the last field of v1
        self.assertEqual(frame[lcd_off:lcd_off + P.DS_TELEMETRY_LCD_LEN], b"HI" + b" " * 30)
        self.assertEqual(P.TelemetryPacket.decode(frame).lcd, "HI" + " " * 30)
        self.assertEqual(P.TelemetryPacket().version, P.DS_TELEMETRY_VERSION)
        self.assertEqual(P.TelemetryPacket().lcd, " " * 32)
        with self.assertRaises(ValueError):
            P.TelemetryPacket(lcd="x" * 33).encode()

    def test_decode_rejects(self):
        good = P.StatusPacket(**P.EXAMPLES["StatusPacket"]).encode()
        with self.assertRaisesRegex(ValueError, "bytes"):
            P.StatusPacket.decode(good[:-1])
        with self.assertRaisesRegex(ValueError, "bytes"):
            P.StatusPacket.decode(good + b"\x00")
        with self.assertRaisesRegex(ValueError, "CRC"):
            P.StatusPacket.decode(good[:-1] + bytes([good[-1] ^ 1]))
        wrong_start = bytes([P.DS_CONTROL_START]) + good[1:]
        with self.assertRaisesRegex(ValueError, "start"):
            P.StatusPacket.decode(wrong_start)
        body = bytes([good[1] + 1]) + good[2:-1]
        wrong_len = bytes([good[0]]) + body + bytes([P.crc8(body)])   # CRC fixed: only len is wrong
        with self.assertRaisesRegex(ValueError, "len"):
            P.StatusPacket.decode(wrong_len)
        with self.assertRaises(ValueError):
            P.decode_frame(b"\x00" * 10)
        with self.assertRaises(ValueError):
            P.decode_frame(b"")

    def test_bits_to_dict(self):
        d = P.bits_to_dict("Buttons", P.DS_BTN_A | P.DS_BTN_BACK)
        self.assertTrue(d["a"] and d["back"])
        self.assertFalse(d["b"] or d["dpadLeft"])
        self.assertEqual(list(d), [k for k, _bit, _d in next(b for b in SPEC.BITS if b["name"] == "Buttons")["bits"]])


class TestDeframer(unittest.TestCase):
    def frames(self):
        return {cls.__name__: cls(**P.EXAMPLES[cls.__name__]).encode() for cls in P.PACKET_CLASSES}

    def test_text_with_z_and_split_reads(self):
        f = self.frames()
        stream = (b"[2] STATE PAIRING -> DISARMED Zap Zz\r\n" + f["StatusPacket"] + b"Z\x07junk\r\n"
                  + f["TelemetryPacket"] + b"ZZZ\r\n" + f["ControlPacket"] + f["BeaconPacket"] + b"done\n")
        expected = [f["StatusPacket"], f["TelemetryPacket"], f["ControlPacket"], f["BeaconPacket"]]
        for chunk in (1, 2, 3, 7, 13, 64, len(stream)):
            d = P.Deframer()
            got = []
            for i in range(0, len(stream), chunk):
                got += d.feed(stream[i:i + chunk])
            self.assertEqual([g.raw for g in got], expected, "chunk %d" % chunk)
            self.assertEqual([type(g.packet).__name__ for g in got],
                             ["StatusPacket", "TelemetryPacket", "ControlPacket", "BeaconPacket"])
            self.assertEqual(d.held, 0)
            self.assertEqual(d.junk_bytes + sum(len(e) for e in expected), len(stream))

    def test_text_comes_back_in_order(self):
        f = self.frames()
        d = P.Deframer()
        items = d.feed_ordered(b"hello Z\r\n" + f["ControlPacket"] + b"bye\n")
        self.assertEqual(items[0], b"hello Z\r\n")
        self.assertIsInstance(items[1], P.Frame)
        self.assertEqual(items[2], b"bye\n")

    def test_rejects_wrong_len(self):
        f = bytearray(self.frames()["ControlPacket"])
        f[1] = 11
        f[-1] = P.crc8(f[1:-1])        # a correct CRC for the wrong length
        d = P.Deframer()
        self.assertEqual(d.feed(bytes(f) + b"\n" * 20), [])
        self.assertGreaterEqual(d.len_errors, 1)
        self.assertEqual(d.crc_errors, 0)

    def test_rejects_bad_crc(self):
        f = bytearray(self.frames()["TelemetryPacket"])
        f[30] ^= 0x10
        d = P.Deframer()
        self.assertEqual(d.feed(bytes(f) + b"\n" * 100), [])
        self.assertEqual(d.crc_errors, 1)
        self.assertEqual(d.frames, 0)

    def test_frame_inside_rejected_candidate_is_found(self):
        f = self.frames()
        tel = f["TelemetryPacket"]
        # a telemetry frame missing a byte swallows the next frame's start byte
        stream = tel[:40] + tel[41:] + f["StatusPacket"] + f["ControlPacket"]
        got = P.Deframer().feed(stream)
        self.assertEqual([g.raw for g in got], [f["StatusPacket"], f["ControlPacket"]])

    def test_printable_text_never_fakes_a_frame(self):
        rng = S.seeded(1)
        alphabet = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 :.,=-_[]()\r\n"
        text = bytes(rng.choice(alphabet) for _ in range(200_000))
        d = P.Deframer()
        self.assertEqual(sum(len(d.feed(text[i:i + 4096])) for i in range(0, len(text), 4096)), 0)
        self.assertEqual(d.crc_errors, 0)      # no candidate even reaches the CRC check

    def test_chunking_does_not_matter(self):
        rng = S.seeded(99)
        stream = S.mixed_stream(P, SPEC, rng, pieces=400)
        ref = P.Deframer()
        ref_frames = [g.raw for g in ref.feed(stream)]
        self.assertGreater(len(ref_frames), 100)
        self.assertGreater(ref.crc_errors, 10)
        self.assertGreater(ref.len_errors, 10)
        for _ in range(5):
            d = P.Deframer()
            got, i = [], 0
            while i < len(stream):
                n = rng.randrange(1, 200)
                got += [g.raw for g in d.feed(stream[i:i + n])]
                i += n
            self.assertEqual(got, ref_frames)
            self.assertEqual((d.crc_errors, d.len_errors, d.junk_bytes, d.held),
                             (ref.crc_errors, ref.len_errors, ref.junk_bytes, ref.held))
        self.assertEqual(ref.junk_bytes + sum(map(len, ref_frames)) + ref.held, len(stream))
        for raw in ref_frames:
            P.decode_frame(raw)       # every accepted frame decodes


class TestDocs(unittest.TestCase):
    def test_example_frames_in_docs(self):
        text = S.DOC.read_text(encoding="utf-8")
        found = dict(re.findall(r"<!-- example (\w+) -->\n```text\n(.*?)```", text, re.S))
        self.assertEqual(sorted(found), sorted(c.__name__ for c in P.PACKET_CLASSES))
        for cls in P.PACKET_CLASSES:
            doc_bytes = bytes.fromhex(found[cls.__name__].replace("\n", " "))
            self.assertEqual(doc_bytes, cls(**P.EXAMPLES[cls.__name__]).encode(), cls.__name__)

    def test_docs_and_headers_are_generated(self):
        self.assertIn("GENERATED by", S.DOC.read_text(encoding="utf-8").splitlines()[0])
        self.assertIn("GENERATED by", S.HEADER.read_text(encoding="utf-8"))
        self.assertEqual(S.HEADER.read_bytes(), S.ESP32_HEADER.read_bytes())


if __name__ == "__main__":
    unittest.main()
