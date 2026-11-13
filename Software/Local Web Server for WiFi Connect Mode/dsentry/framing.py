# SPDX-License-Identifier: MIT
"""Received bytes -> D-Sentry frames and text lines.

The USB port carries the RA4M1's binary Status/Telemetry frames and its ASCII debug text on one
byte stream (architecture.md 4.3). protocol.Deframer finds the frames (exact len + CRC, so text can
never pass for one) and hands back everything else as junk runs in stream order; LinkReader joins
the junk into text lines. A UDP datagram is one whole frame and goes through the same path.
"""

from typing import List, NamedTuple, Union

from . import protocol as P

# A run of text this long without a newline is emitted as a line of its own (binary noise, or a
# board printing without newlines, can never grow the buffer without bound).
TEXT_LINE_CAP = 1024


class TextLine(NamedTuple):
    text: str


Item = Union[P.Frame, TextLine]


def clean_text(raw: bytes) -> str:
    """Printable ASCII copy of a text line: CR dropped, other control or non-ASCII bytes -> '?'."""
    out = []
    for b in raw:
        if b == 0x0D:
            continue
        out.append(chr(b) if 0x20 <= b < 0x7F or b == 0x09 else "?")
    return "".join(out).rstrip()


class LinkReader:
    """Stateful: feed() any split of the stream and get frames and complete text lines in order."""

    def __init__(self) -> None:
        # accept_legacy: an older RA4M1 build's TelemetryPacket (version 1, no controller block)
        # still reaches the hub, which shows it and marks the controller as not reported.
        self.deframer = P.Deframer(accept_legacy=True)
        self._text = bytearray()
        self.lines = 0

    @property
    def crc_errors(self) -> int:
        return self.deframer.crc_errors

    @property
    def frames(self) -> int:
        return self.deframer.frames

    def reset(self) -> None:
        """After a reconnect: a half-received frame or line must not join the new stream."""
        self.deframer.reset()
        self._text.clear()

    def feed(self, data: bytes) -> List[Item]:
        items: List[Item] = []
        for piece in self.deframer.feed_ordered(data):
            if isinstance(piece, P.Frame):
                items.append(piece)
            else:
                self._take_text(piece, items)
        return items

    def _take_text(self, raw: bytes, items: List[Item]) -> None:
        self._text += raw
        while True:
            end = self._text.find(b"\n")
            if end < 0:
                break
            line = clean_text(bytes(self._text[:end]))
            del self._text[:end + 1]
            if line:
                self.lines += 1
                items.append(TextLine(line))
        if len(self._text) >= TEXT_LINE_CAP:
            line = clean_text(bytes(self._text))
            self._text.clear()
            if line:
                self.lines += 1
                items.append(TextLine(line))
