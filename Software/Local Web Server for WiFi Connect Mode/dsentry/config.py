# SPDX-License-Identifier: MIT
"""Reads D-SentryCode/config.h at run time, so the relay, the bench console and the tests use the
owner's tunables instead of copies (architecture.md rule 1: Python reads tunables from config.h).

    from dsentry import config
    cfg = config.load()                  # D-SentryCode/config.h, or $DSENTRY_CONFIG_H
    cfg["CONTROL_PACKET_HZ"]             # 100            (int)
    cfg.JOY_DEADZONE                     # 0.12           (float; the f suffix is dropped)
    cfg.PAN_STEPS_PER_DEG                # 22.222...      (arithmetic over other defines)
    cfg.TOGGLE_ACTIVE_LOW                # True           (bool)
    cfg.WIFI_SSID                        # 'CHANGE_ME'    (str, escapes decoded)
    cfg.LED_COLOR_EMPTY_MAG              # 0x8000FF       (int; cfg.rgb(name) -> (128, 0, 255))
    cfg.PIN_TOGGLE_POWER                 # RawToken('A0') (a str: Arduino names stay text)
    cfg.comment("LINK_TIMEOUT_MS")       # the // comment beside the define, continuation joined
    cfg.section("LINK_TIMEOUT_MS")       # '10. CONTROLLER / LINK'

Value rules:
- integers in decimal, hex, octal or binary, with u/l suffixes and ' digit separators -> int;
- floating literals (a '.', an exponent or an f suffix) -> float;
- true / false -> bool; "string" literals (adjacent ones joined) -> str;
- expressions over numbers and other defines (+ - * / % << >> & | ^ ~ ! comparisons && || ?:,
  parentheses and casts such as (uint32_t) or (float)) are evaluated with C's rules where they
  differ from Python's: int / int truncates toward zero, % takes the dividend's sign, a cast to an
  unsigned fixed-width type wraps. Not modelled: float32 rounding (results are doubles) and
  unsigned wrap-around outside a cast;
- anything else (NEO_GRB, A0, Serial2, an expression using them, a function-like macro) stays the
  raw text as a RawToken, a str subclass, so it compares equal to the text and can be told apart
  from a real string with isinstance(value, RawToken).
Conditional directives (#if, #ifdef) are not evaluated: every #define is read, the last one wins,
and repeats are listed in Config.duplicates.
"""

import math
import os
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Dict, Iterator, List, NamedTuple, Optional, Tuple

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config.h"   # D-SentryCode/config.h
ENV_VAR = "DSENTRY_CONFIG_H"

# "// 10. CONTROLLER / LINK" directly under a "// ----" rule line
SECTION_RULE_RE = re.compile(r"^\s*//\s*-{10,}\s*$")
SECTION_HEADER_RE = re.compile(r"^\s*//\s*(\d+)\.\s+(\S.*?)\s*$")
DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)(\([^)]*\))?(.*)$")
UNDEF_RE = re.compile(r"^\s*#\s*undef\s+([A-Za-z_]\w*)")
CONDITIONAL_RE = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b")
CONTINUATION_COMMENT_RE = re.compile(r"^\s+//(.*)$")

_TOKEN_RE = re.compile(r"""
    (?P<ws>\s+)
  | (?P<str>(?:u8|u|U|L)?"(?:[^"\\\n]|\\.)*")
  | (?P<chr>(?:u8|u|U|L)?'(?:[^'\\\n]|\\.)+')
  | (?P<num>(?:0[xX][0-9a-fA-F']*\.?[0-9a-fA-F']*(?:[pP][+-]?[0-9]+)?
              |[0-9][0-9']*\.?[0-9']*(?:[eE][+-]?[0-9]+)?
              |\.[0-9][0-9']*(?:[eE][+-]?[0-9]+)?)[A-Za-z0-9_]*)
  | (?P<id>[A-Za-z_]\w*)
  | (?P<op><<|>>|<=|>=|==|!=|&&|\|\||[-+*/%&|^~!<>?:(),])
""", re.X)

# Casts the evaluator understands: name -> (kind, bits, signed)
_CAST_TYPES = {
    "uint8_t": ("int", 8, False), "int8_t": ("int", 8, True),
    "uint16_t": ("int", 16, False), "int16_t": ("int", 16, True),
    "uint32_t": ("int", 32, False), "int32_t": ("int", 32, True),
    "uint64_t": ("int", 64, False), "int64_t": ("int", 64, True),
    "size_t": ("int", 32, False),          # both targets are 32-bit
    "float": ("float", 0, True), "double": ("float", 0, True), "bool": ("bool", 0, False),
}
_CAST_WORDS = {"unsigned", "signed", "int", "long", "short", "char", "const"}

_ESCAPES = {"n": "\n", "t": "\t", "r": "\r", "0": "\0", "\\": "\\", '"': '"', "'": "'",
            "a": "\a", "b": "\b", "f": "\f", "v": "\v", "?": "?"}


class RawToken(str):
    """A define whose value could not be resolved to a number, bool or string: the raw text."""

    def __repr__(self) -> str:
        return "RawToken(%s)" % str.__repr__(self)


class Define(NamedTuple):
    name: str
    raw: str                 # value text, comments removed, whitespace collapsed
    line: int                # 1-based line of the #define
    section: str             # e.g. "10. CONTROLLER / LINK", or "" before the first section
    comment: str             # trailing // comment plus its indented continuation lines
    params: Optional[str]    # "(a, b)" for a function-like macro, else None


class _Unresolved(Exception):
    pass


# ---------------------------------------------------------------------------------------------
# Lexing
# ---------------------------------------------------------------------------------------------
def _is_digit_separator(text: str, i: int) -> bool:
    """True when the ' at text[i] is a C++14 digit separator (1'000), not a char literal."""
    j = i
    while j > 0 and (text[j - 1].isalnum() or text[j - 1] == "_"):
        j -= 1
    word = text[j:i]
    return bool(word) and word[0].isdigit()


def _split_comments(text: str) -> Tuple[str, Dict[int, str]]:
    """Returns (code with every comment replaced by spaces, newlines kept;
    {line number: comment text on that line})."""
    out: List[str] = []
    comments: Dict[int, List[str]] = {}
    i, n, line = 0, len(text), 1
    while i < n:
        c = text[i]
        if c == "\n":
            out.append(c)
            line += 1
            i += 1
        elif c == '"' or (c == "'" and not _is_digit_separator(text, i)):
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(text[i:j])
            i = j
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            comments.setdefault(line, []).append(text[i + 2:j].strip())
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            body = text[i + 2:j - 2] if text.endswith("*/", 0, j) else text[i + 2:j]
            comments.setdefault(line, []).append(" ".join(body.split()))
            chunk = text[i:j]
            out.append("".join("\n" if ch == "\n" else " " for ch in chunk))
            line += chunk.count("\n")
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out), {k: " ".join(v) for k, v in comments.items()}


def _tokens(raw: str) -> List[Tuple[str, str]]:
    toks = []
    pos = 0
    while pos < len(raw):
        m = _TOKEN_RE.match(raw, pos)
        if not m:
            raise _Unresolved("cannot read %r" % raw[pos:])
        pos = m.end()
        kind = m.lastgroup
        if kind != "ws":
            toks.append((kind, m.group()))
    return toks


def parse_number(text: str):
    """A C integer or floating literal -> int or float. ValueError if it is not one."""
    t = text.replace("'", "")
    low = t.lower()
    if low.startswith("0x"):
        body = low[2:]
        if "." in body or "p" in body:
            m = re.fullmatch(r"([0-9a-f]*\.?[0-9a-f]*p[+-]?[0-9]+)([fl]?)", body)
            if not m:
                raise ValueError(text)
            return float.fromhex("0x" + m.group(1))
        m = re.fullmatch(r"([0-9a-f]+)(u|l|ul|lu|ll|ull|llu|z|uz|zu)?", body)
        if not m:
            raise ValueError(text)
        return int(m.group(1), 16)
    if low.startswith("0b"):
        m = re.fullmatch(r"([01]+)(u|l|ul|lu|ll|ull|llu|z|uz|zu)?", low[2:])
        if not m:
            raise ValueError(text)
        return int(m.group(1), 2)
    m = re.fullmatch(r"([0-9]+)(u|l|ul|lu|ll|ull|llu|z|uz|zu)?", low)
    if m:
        digits = m.group(1)
        if len(digits) > 1 and digits[0] == "0":
            return int(digits, 8)          # raises ValueError on 8 or 9, as C would refuse it
        return int(digits)
    m = re.fullmatch(r"([0-9]*\.[0-9]*|[0-9]+)(e[+-]?[0-9]+)?([fl]?)", low)
    if m and m.group(1) not in (".", "") and (("." in m.group(1)) or m.group(2) or m.group(3)):
        return float(m.group(1) + (m.group(2) or ""))
    raise ValueError(text)


def _decode_string(tok: str) -> str:
    body = tok[tok.index('"') + 1:-1]
    out, i = [], 0
    while i < len(body):
        c = body[i]
        if c != "\\":
            out.append(c)
            i += 1
            continue
        nxt = body[i + 1] if i + 1 < len(body) else ""
        if nxt == "x":
            m = re.match(r"[0-9a-fA-F]+", body[i + 2:])
            if not m:
                raise _Unresolved("bad \\x escape")
            out.append(chr(int(m.group(), 16) & 0xFF))
            i += 2 + m.end()
        elif nxt in "01234567" and nxt:
            m = re.match(r"[0-7]{1,3}", body[i + 1:])
            out.append(chr(int(m.group(), 8)))
            i += 1 + m.end()
        elif nxt in _ESCAPES:
            out.append(_ESCAPES[nxt])
            i += 2
        else:
            raise _Unresolved("unknown escape")
    return "".join(out)


# ---------------------------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------------------------
def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, str)


def _as_int(v) -> int:
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, int):
        return v
    raise _Unresolved("integer operand needed")


def _arith(v):
    if isinstance(v, bool):
        return int(v)
    if _is_num(v):
        return v
    raise _Unresolved("numeric operand needed")


def _truthy(v) -> bool:
    if _is_num(v):
        return v != 0
    raise _Unresolved("numeric condition needed")


def _c_div(a, b):
    a, b = _arith(a), _arith(b)
    if b == 0:
        raise _Unresolved("division by zero")
    if isinstance(a, int) and isinstance(b, int):
        q = abs(a) // abs(b)
        return q if (a >= 0) == (b >= 0) else -q
    return a / b


def _c_mod(a, b):
    a, b = _as_int(a), _as_int(b)
    if b == 0:
        raise _Unresolved("modulo by zero")
    return a - b * _c_div(a, b)


_BINARY_LEVELS = [
    ("||",), ("&&",), ("|",), ("^",), ("&",), ("==", "!="), ("<", ">", "<=", ">="),
    ("<<", ">>"), ("+", "-"), ("*", "/", "%"),
]


def _apply(op, a, b):
    if op == "||":
        return int(_truthy(a) or _truthy(b))
    if op == "&&":
        return int(_truthy(a) and _truthy(b))
    if op in ("|", "^", "&", "<<", ">>"):
        a, b = _as_int(a), _as_int(b)
        if op == "|":
            return a | b
        if op == "^":
            return a ^ b
        if op == "&":
            return a & b
        if b < 0:
            raise _Unresolved("negative shift")
        return a << b if op == "<<" else a >> b
    if op in ("==", "!=", "<", ">", "<=", ">="):
        if isinstance(a, str) and isinstance(b, str) and op in ("==", "!="):
            raise _Unresolved("string comparison")
        a, b = _arith(a), _arith(b)
        return int({"==": a == b, "!=": a != b, "<": a < b, ">": a > b,
                    "<=": a <= b, ">=": a >= b}[op])
    if op == "+":
        return _arith(a) + _arith(b)
    if op == "-":
        return _arith(a) - _arith(b)
    if op == "*":
        return _arith(a) * _arith(b)
    if op == "/":
        return _c_div(a, b)
    if op == "%":
        return _c_mod(a, b)
    raise _Unresolved(op)


def _cast(words: List[str], v):
    named = [w for w in words if w in _CAST_TYPES]
    if named:
        kind, bits, signed = _CAST_TYPES[named[0]]
    elif "float" in words or "double" in words:
        kind, bits, signed = "float", 0, True
    else:   # int, unsigned, long, short, char combinations: both targets have 32-bit int/long
        bits = 8 if "char" in words else 16 if "short" in words else 64 if words.count("long") > 1 else 32
        kind, signed = "int", "unsigned" not in words
    if kind == "float":
        return float(_arith(v))
    if kind == "bool":
        return _truthy(v)
    x = _arith(v)
    x = int(x) if isinstance(x, float) and math.isfinite(x) else _as_int(x)
    x &= (1 << bits) - 1
    if signed and x >= 1 << (bits - 1):
        x -= 1 << bits
    return x


class _Parser:
    def __init__(self, toks, lookup):
        self.toks = toks
        self.i = 0
        self.lookup = lookup

    def peek(self, k=0):
        j = self.i + k
        return self.toks[j] if j < len(self.toks) else (None, None)

    def take(self):
        tok = self.peek()
        self.i += 1
        return tok

    def expect(self, text):
        kind, val = self.take()
        if val != text:
            raise _Unresolved("expected %s" % text)

    def parse(self):
        if not self.toks:
            raise _Unresolved("empty")
        v = self.ternary()
        if self.i != len(self.toks):
            raise _Unresolved("trailing tokens")
        return v

    def ternary(self):
        cond = self.binary(0)
        if self.peek()[1] == "?":
            self.take()
            a = self.ternary()
            self.expect(":")
            b = self.ternary()
            return a if _truthy(cond) else b
        return cond

    def binary(self, level):
        if level == len(_BINARY_LEVELS):
            return self.unary()
        left = self.binary(level + 1)
        while self.peek()[0] == "op" and self.peek()[1] in _BINARY_LEVELS[level]:
            op = self.take()[1]
            right = self.binary(level + 1)
            left = _apply(op, left, right)
        return left

    def cast_words(self):
        """Type words of a cast starting at '(' (the current token), or None."""
        if self.peek()[1] != "(":
            return None
        words, k = [], 1
        while self.peek(k)[0] == "id" and (self.peek(k)[1] in _CAST_TYPES or self.peek(k)[1] in _CAST_WORDS
                                           or self.peek(k)[1] in ("float", "double")):
            words.append(self.peek(k)[1])
            k += 1
        if words and self.peek(k)[1] == ")":
            return words, k + 1
        return None

    def unary(self):
        kind, val = self.peek()
        if kind == "op" and val in ("-", "+", "!", "~"):
            self.take()
            v = self.unary()
            if val == "-":
                return -_arith(v)
            if val == "+":
                return _arith(v)
            if val == "!":
                return int(not _truthy(v))
            return ~_as_int(v)
        cast = self.cast_words()
        if cast:
            words, skip = cast
            self.i += skip
            return _cast(words, self.unary())
        return self.primary()

    def primary(self):
        kind, val = self.take()
        if kind == "num":
            try:
                return parse_number(val)
            except ValueError:
                raise _Unresolved("bad number %s" % val)
        if kind == "str":
            parts = [_decode_string(val)]
            while self.peek()[0] == "str":
                parts.append(_decode_string(self.take()[1]))
            return "".join(parts)
        if kind == "chr":
            body = _decode_string('"' + val[val.index("'") + 1:-1] + '"')
            if len(body) != 1:
                raise _Unresolved("multi-character constant")
            return ord(body)
        if kind == "id":
            if val == "true":
                return True
            if val == "false":
                return False
            return self.lookup(val)
        if val == "(":
            v = self.ternary()
            self.expect(")")
            return v
        raise _Unresolved("unexpected %r" % (val,))


# ---------------------------------------------------------------------------------------------
class Config(Mapping):
    """The #defines of a config.h as a read-only mapping name -> typed value (file order)."""

    def __init__(self, text: str, path: Optional[Path] = None) -> None:
        self.path = path
        self._defines: Dict[str, Define] = {}
        self.duplicates: List[str] = []
        self.has_conditionals = False
        self._parse(text)
        self._values: Dict[str, object] = {}
        self._resolving: set = set()
        for name in self._defines:
            self._value(name)

    # -- parsing --
    def _parse(self, text: str) -> None:
        code, comments = _split_comments(text)
        code_lines = code.split("\n")
        src_lines = text.split("\n")
        section = ""
        i = 0
        while i < len(code_lines):
            start = i
            line = code_lines[i]
            while line.rstrip().endswith("\\") and i + 1 < len(code_lines):
                i += 1
                line = line.rstrip()[:-1] + " " + code_lines[i]
            i += 1
            m = SECTION_HEADER_RE.match(src_lines[start])
            if m and start > 0 and SECTION_RULE_RE.match(src_lines[start - 1]):
                section = "%s. %s" % (m.group(1), m.group(2))
                continue
            if CONDITIONAL_RE.match(line):
                self.has_conditionals = True
                continue
            m = UNDEF_RE.match(line)
            if m:
                self._defines.pop(m.group(1), None)
                continue
            m = DEFINE_RE.match(line)
            if not m:
                continue
            name, params, raw = m.group(1), m.group(2), " ".join(m.group(3).split())
            comment = [comments.get(start + 1, "")]
            j = i
            while j < len(src_lines):           # indented "//" lines continue the comment
                cm = CONTINUATION_COMMENT_RE.match(src_lines[j])
                if not cm or DEFINE_RE.match(src_lines[j]):
                    break
                comment.append(cm.group(1).strip())
                j += 1
            if name in self._defines:
                self.duplicates.append(name)
                del self._defines[name]       # re-insert so file order follows the winner
            self._defines[name] = Define(name, raw, start + 1, section,
                                         " ".join(c for c in comment if c), params)

    # -- evaluation --
    def _lookup(self, name: str):
        if name not in self._defines:
            raise _Unresolved("unknown identifier %s" % name)
        return self._value(name)

    def _value(self, name: str):
        if name in self._values:
            return self._values[name]
        d = self._defines[name]
        if d.params is not None:
            value = RawToken(d.raw)
        elif name in self._resolving:
            raise _Unresolved("%s refers to itself" % name)
        else:
            self._resolving.add(name)
            try:
                value = _Parser(_tokens(d.raw), self._lookup).parse()
            except _Unresolved:
                value = RawToken(d.raw)
            finally:
                self._resolving.discard(name)
        self._values[name] = value
        return value

    # -- Mapping --
    def __getitem__(self, name: str):
        return self._values[name]

    def __iter__(self) -> Iterator[str]:
        return iter(self._defines)

    def __len__(self) -> int:
        return len(self._defines)

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._values[name]
        except KeyError:
            raise AttributeError("config.h has no #define %s" % name) from None

    def __repr__(self) -> str:
        return "Config(%s, %d defines)" % (self.path, len(self))

    # -- extras --
    def define(self, name: str) -> Define:
        return self._defines[name]

    def raw(self, name: str) -> str:
        return self._defines[name].raw

    def comment(self, name: str) -> str:
        return self._defines[name].comment

    def section(self, name: str) -> str:
        return self._defines[name].section

    def names_in_section(self, number: int) -> List[str]:
        prefix = "%d. " % number
        return [n for n, d in self._defines.items() if d.section.startswith(prefix)]

    @property
    def unresolved(self) -> List[str]:
        """Names whose value stayed raw text (RawToken)."""
        return [n for n, v in self._values.items() if isinstance(v, RawToken)]

    def rgb(self, name: str) -> Tuple[int, int, int]:
        """A 0xRRGGBB colour define as (r, g, b)."""
        v = self[name]
        if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 0xFFFFFF:
            raise ValueError("%s is not a 0xRRGGBB colour" % name)
        return (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF


def default_path() -> Path:
    env = os.environ.get(ENV_VAR)
    return Path(env) if env else DEFAULT_PATH


def load(path=None) -> Config:
    """Parse config.h: path, else $DSENTRY_CONFIG_H, else D-SentryCode/config.h."""
    p = Path(path) if path is not None else default_path()
    return Config(p.read_text(encoding="utf-8"), p)


def from_text(text: str) -> Config:
    return Config(text)
