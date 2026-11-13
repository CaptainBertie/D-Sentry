#!/usr/bin/env python3
# check_literals.py -- acceptance item 20 (spec 11, architecture.md rule 1): no pin, speed,
# angle, colour, timeout, threshold, size, rate or unit factor is written as a number in a
# D-Sentry C/C++ source; they come from config.h or dsentry_protocol.h.
#
# How it reads a file: a small C/C++ tokeniser drops // and /* */ comments (including a //
# comment continued by a trailing backslash), string and character literals (with u8/u/U/L
# prefixes and R"delim(...)delim" raw strings) and the rest of #include / #error / #warning
# lines, then finds every numeric literal as the preprocessor sees it (a pp-number: decimal,
# hex, binary, octal, floating with exponents and hex floats, any suffix such as u UL f, and
# C++14 digit separators such as 1'000).
#
# Allowed, everything else is reported as path:line:col with the literal:
#   - a literal whose value is 0 or 1, in any spelling: 0 1 0u 1UL 0.0f 1.0f 0x0 1. 1e0;
#   - a literal that is the whole content of a pair of square brackets, i.e. `[` literal `]`
#     with nothing else inside, when it is an array INDEX (buf[3], m[2][3] in an expression).
#     An array DIMENSION is a size and is reported (architecture.md rule 1: sizes come from
#     config.h): uint8_t buf[16], int m[2][3], static const char* names[4], new uint8_t[5],
#     typedef uint8_t mac_t[6], uint8_t Foo::buf_[8], void f(const uint8_t data[2]).
#     Heuristic (tokens only, no parser): walk back over any earlier [..] groups and a
#     qualified name (A::b) to the name being bracketed; it is a declaration when the token
#     before that name is a type-like identifier (not a keyword such as return, case, else,
#     sizeof, delete, throw), or a `*` / `&` run that follows a type-like identifier at the start
#     of a declaration (after ; { } or a specifier such as const / static). Everything else
#     (=, (, ,, ;, ), operators, ., ->) makes it an index. buf[i + 2] or buf[2 * n] are
#     reported whatever they are;
#   - ESP32 scope only (--esp32, or --lit-ok): a line ending in a `// lit-ok: <reason>` comment.
#     The reason is required; a bare `// lit-ok:` is itself reported. The RA4M1 has no such
#     escape hatch: its numbers go to config.h;
#   - with --allow-pending: lines between `// CONFIG-PENDING begin` and `// CONFIG-PENDING end`
#     (the temporary blocks parallel agents use until the integration stage moves them into
#     config.h). Without the flag those literals are reported and marked as pending.
#
# Scopes:
#   --ra4m1 (default)  D-SentryCode/firmware-ra4m1/dsentry/**/*.ino|.cpp|.h|.hpp|.c, except
#                      config.h, dsentry_protocol.h and xbox_r4_protocol.h (symlinks to the
#                      tunables file and the generated protocol headers)
#   --esp32            files in firmware/UNOR4USBBridge whose names contain "dsentry"
#                      (cmds_dsentry.h, dsentry_*.cpp ...), except the generated
#                      dsentry_protocol.h and dsentry_config.h
#   --paths F...       these files, RA4M1 rules unless --lit-ok
# Scopes combine. A scope that matches no file is an error (exit 2) unless --allow-empty, so a
# wrong path can never pass silently.
#
# Usage: "<repo>/.venv/bin/python" D-SentryCode/tools/check_literals.py [--ra4m1] [--esp32]
#        [--paths F ...] [--lit-ok] [--allow-pending] [--allow-empty] [--selftest]
# Exit status: 0 clean, 1 literals found (or a self-test failed), 2 usage error / empty scope.
#
# SPDX-License-Identifier: MIT

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DSC = HERE.parent
REPO = DSC.parent
RA4M1_ROOT = DSC / "firmware-ra4m1" / "dsentry"
RA4M1_EXCLUDE = {"config.h", "dsentry_protocol.h", "xbox_r4_protocol.h"}
ESP32_ROOT = REPO / "firmware" / "UNOR4USBBridge"
ESP32_GENERATED = {"dsentry_protocol.h", "dsentry_config.h"}
SOURCE_SUFFIXES = {".ino", ".cpp", ".cc", ".c", ".h", ".hpp"}

STRING_PREFIXES = {"L", "u", "U", "u8"}
RAW_PREFIXES = {"R", "LR", "uR", "UR", "u8R"}
SKIP_LINE_DIRECTIVES = {"include", "include_next", "import", "error", "warning"}
LIT_OK_RE = re.compile(r"^\s*lit-ok:(.*)$")
PENDING_BEGIN_RE = re.compile(r"^\s*CONFIG-PENDING\s+begin\b")
PENDING_END_RE = re.compile(r"^\s*CONFIG-PENDING\s+end\b")
DIRECTIVE_RE = re.compile(r"#[ \t]*([A-Za-z_]\w*)")
INT_SUFFIX = r"(u|l|ul|lu|ll|ull|llu|z|uz|zu)?"


def is_ident_char(c):
    return c.isalnum() or c in "_$" or ord(c) >= 0x80


class Token:
    __slots__ = ("kind", "text", "line", "col")

    def __init__(self, kind, text, line, col):
        self.kind = kind     # num, id, str, punct
        self.text = text
        self.line = line
        self.col = col


def tokenize(text):
    """-> (tokens, comments {line: [comment text]}). Comments, strings and character literals
    never produce num tokens; string/char literals become one 'str' token each."""
    toks = []
    comments = {}
    i, n = 0, len(text)
    line, line_start = 1, 0
    at_line_start = True          # only whitespace so far on this logical line

    def newline(pos):
        nonlocal line, line_start
        line += 1
        line_start = pos + 1

    def skip_quoted(j, quote):
        """j: index after the opening quote. Returns the index after the closing quote (or the
        newline that ends an unterminated literal)."""
        while j < n:
            c = text[j]
            if c == "\\" and j + 1 < n:
                if text[j + 1] == "\n":
                    newline(j + 1)
                j += 2
                continue
            if c == quote:
                return j + 1
            if c == "\n":
                return j
            j += 1
        return n

    while i < n:
        c = text[i]
        col = i - line_start + 1
        if c == "\n":
            newline(i)
            at_line_start = True
            i += 1
            continue
        if c == "\\" and i + 1 < n and text[i + 1] == "\n":     # line splice
            newline(i + 1)
            i += 2
            continue
        if c in " \t\r\f\v":
            i += 1
            continue
        if text.startswith("//", i):
            start_line = line
            j = i + 2
            while True:                # a backslash at the end continues the comment
                k = text.find("\n", j)
                k = n if k < 0 else k
                if text[j:k].rstrip("\r").endswith("\\") and k < n:
                    newline(k)
                    j = k + 1
                    continue
                break
            comments.setdefault(start_line, []).append(text[i + 2:k])
            i = k
            continue
        if text.startswith("/*", i):
            k = text.find("*/", i + 2)
            k = n if k < 0 else k + 2
            comments.setdefault(line, []).append(text[i + 2:k - 2])
            for p in range(i, k):
                if text[p] == "\n":
                    newline(p)
            i = k
            continue
        if c == "#" and at_line_start:
            m = DIRECTIVE_RE.match(text, i)
            if m and m.group(1) in SKIP_LINE_DIRECTIVES:
                k = text.find("\n", i)
                k = n if k < 0 else k
                # keep a trailing // comment of the skipped line visible (lit-ok, pending)
                rest = text[m.end():k]
                pos = rest.find("//")
                if pos >= 0:
                    comments.setdefault(line, []).append(rest[pos + 2:])
                i = k
                continue
            toks.append(Token("punct", "#", line, col))
            at_line_start = False
            i += 1
            continue
        at_line_start = False
        if c == '"' or c == "'":
            j = skip_quoted(i + 1, c)
            toks.append(Token("str", text[i:j], line, col))
            i = j
            continue
        if c.isdigit() or (c == "." and i + 1 < n and text[i + 1].isdigit()):
            j = i + (2 if c == "." else 1)
            while j < n:
                d = text[j]
                if d in "eEpP" and j + 1 < n and text[j + 1] in "+-":
                    j += 2
                elif d.isalnum() or d in "_.":
                    j += 1
                elif d == "'" and j + 1 < n and (text[j + 1].isalnum() or text[j + 1] == "_"):
                    j += 2
                else:
                    break
            toks.append(Token("num", text[i:j], line, col))
            i = j
            continue
        if is_ident_char(c):
            j = i + 1
            while j < n and is_ident_char(text[j]):
                j += 1
            word = text[i:j]
            if j < n and text[j] == '"' and word in RAW_PREFIXES:
                m = re.compile(r'"([^()\\\s]{0,16})\(').match(text, j)
                if m:
                    end = text.find(")" + m.group(1) + '"', m.end())
                    end = n if end < 0 else end + len(m.group(1)) + 2
                    for p in range(j, end):
                        if text[p] == "\n":
                            newline(p)
                    toks.append(Token("str", text[i:end], line, col))
                    i = end
                    continue
            if j < n and text[j] in "\"'" and word in STRING_PREFIXES:
                k = skip_quoted(j + 1, text[j])
                toks.append(Token("str", text[i:k], line, col))
                i = k
                continue
            toks.append(Token("id", word, line, col))
            i = j
            continue
        toks.append(Token("punct", c, line, col))
        i += 1
    return toks, comments


def literal_value(token):
    """Numeric value of a C/C++ literal, or None when it is not a plain number (e.g. 10ms)."""
    t = token.replace("'", "").lower()
    if t.startswith("0x"):
        body = t[2:]
        if "." in body or "p" in body:
            m = re.fullmatch(r"([0-9a-f]*\.?[0-9a-f]*p[+-]?[0-9]+)[fl]?", body)
            return float.fromhex("0x" + m.group(1)) if m else None
        m = re.fullmatch(r"([0-9a-f]+)" + INT_SUFFIX, body)
        return int(m.group(1), 16) if m else None
    if t.startswith("0b"):
        m = re.fullmatch(r"([01]+)" + INT_SUFFIX, t[2:])
        return int(m.group(1), 2) if m else None
    m = re.fullmatch(r"([0-9]+)" + INT_SUFFIX, t)
    if m:
        digits = m.group(1)
        if len(digits) > 1 and digits[0] == "0":
            return int(digits, 8) if re.fullmatch(r"[0-7]+", digits) else None
        return int(digits)
    m = re.fullmatch(r"([0-9]*\.[0-9]*|[0-9]+)(e[+-]?[0-9]+)?[fl]?", t)
    if m and m.group(1) not in (".", ""):
        return float(m.group(1) + (m.group(2) or ""))
    return None


# Identifiers that can sit right before a bracketed name in an EXPRESSION (so the brackets are
# an index), unlike a type name, which makes the brackets a declaration's dimension.
EXPRESSION_KEYWORDS = {
    "return", "case", "else", "do", "goto", "throw", "sizeof", "alignof", "typeid", "delete",
    "co_return", "co_yield", "co_await", "and", "or", "not", "xor", "bitand", "bitor", "compl",
    "and_eq", "or_eq", "xor_eq", "not_eq", "default",
}
# Tokens that start a declaration statement, before its type: `T* name[N]` after one of them
# is a declaration; after anything else (a call argument, an operand) `a * b[N]` is an index.
DECL_START = {";", "{", "}", "const", "static", "constexpr", "extern", "volatile", "struct",
              "class", "unsigned", "signed", "inline", "thread_local", "mutable"}


def is_type_like(tok):
    return tok is not None and tok.kind == "id" and tok.text not in EXPRESSION_KEYWORDS


def is_dimension(toks, open_idx):
    """toks[open_idx] is the `[` of `[ literal ]`. True when those brackets are an array
    dimension in a declaration (see the header comment for the heuristic)."""
    p = open_idx - 1
    # earlier bracket groups of the same name: m[2][3], name[N][M]
    while p >= 0 and toks[p].text == "]":
        depth = 0
        while p >= 0:
            if toks[p].text == "]":
                depth += 1
            elif toks[p].text == "[":
                depth -= 1
                if depth == 0:
                    break
            p -= 1
        p -= 1
    if p < 0 or toks[p].kind != "id" or toks[p].text in EXPRESSION_KEYWORDS:
        return False      # )[3], "abc"[2], return[...]: an index
    # a qualified name: Foo::buf_[8] (the tokeniser gives ':' ':' as two punct tokens)
    while p >= 3 and toks[p - 1].text == ":" and toks[p - 2].text == ":" and toks[p - 3].kind == "id":
        p -= 3
    q = p - 1
    if q < 0:
        return False
    before = toks[q]
    if is_type_like(before):
        return True       # uint8_t buf[16], new uint8_t[5], typedef uint8_t mac_t[6]
    if before.text in ("*", "&"):
        r = q
        while r >= 0 and toks[r].text in ("*", "&", "const", "volatile"):
            r -= 1
        if r < 0 or not is_type_like(toks[r]):
            return False
        s = r - 1
        return s < 0 or toks[s].text in DECL_START
    return False


class Finding:
    def __init__(self, path, line, col, token, note=""):
        self.path, self.line, self.col, self.token, self.note = path, line, col, token, note

    def key(self):
        return (self.line, self.col, self.token)

    def __str__(self):
        return "%s:%d:%d: %s%s" % (self.path, self.line, self.col, self.token,
                                   "  (%s)" % self.note if self.note else "")


def check_text(text, path="<text>", lit_ok=False, allow_pending=False):
    toks, comments = tokenize(text)
    exempt, bad_lit_ok, has_lit_ok, pending = set(), [], set(), set()
    in_pending = False
    for ln in sorted(comments):
        for body in comments[ln]:
            m = LIT_OK_RE.match(body)
            if m:
                has_lit_ok.add(ln)
                if m.group(1).strip():
                    exempt.add(ln)
                else:
                    bad_lit_ok.append(ln)
    # CONFIG-PENDING spans: begin comment line .. end comment line
    pending_marks = []
    for ln in sorted(comments):
        for body in comments[ln]:
            if PENDING_BEGIN_RE.match(body):
                pending_marks.append((ln, "begin"))
            elif PENDING_END_RE.match(body):
                pending_marks.append((ln, "end"))
    start = None
    for ln, what in pending_marks:
        if what == "begin" and start is None:
            start = ln
        elif what == "end" and start is not None:
            pending.update(range(start, ln + 1))
            start = None
    if start is not None:
        pending.update(range(start, text.count("\n") + 2))
        in_pending = True

    findings = []
    for k, t in enumerate(toks):
        if t.kind != "num":
            continue
        value = literal_value(t.text)
        if value is not None and value in (0, 1):
            continue
        prev_t = toks[k - 1].text if k > 0 else ""
        next_t = toks[k + 1].text if k + 1 < len(toks) else ""
        dimension = False
        if prev_t == "[" and next_t == "]":
            if not is_dimension(toks, k - 1):
                continue
            dimension = True
        if lit_ok and t.line in exempt:
            continue
        if allow_pending and t.line in pending:
            continue
        note = "array dimension: name the size in config.h" if dimension else ""
        if t.line in pending:
            note = "inside CONFIG-PENDING: move it to config.h"
        elif t.line in has_lit_ok and not lit_ok:
            note = "lit-ok is not honoured in this scope"
        findings.append(Finding(path, t.line, t.col, t.text, note))
    if lit_ok:
        for ln in bad_lit_ok:
            findings.append(Finding(path, ln, 1, "lit-ok:", "a lit-ok comment needs a reason"))
    if in_pending:
        findings.append(Finding(path, max(pending), 1, "CONFIG-PENDING",
                                "begin without end"))
    findings.sort(key=lambda f: (f.line, f.col))
    return findings


# ---------------------------------------------------------------------------------------------
def ra4m1_files():
    if not RA4M1_ROOT.is_dir():
        return None
    return sorted(p for p in RA4M1_ROOT.rglob("*")
                  if p.suffix in SOURCE_SUFFIXES and p.name not in RA4M1_EXCLUDE and p.is_file())


def esp32_files():
    if not ESP32_ROOT.is_dir():
        return None
    return sorted(p for p in ESP32_ROOT.iterdir()
                  if p.is_file() and p.suffix in SOURCE_SUFFIXES and "dsentry" in p.name.lower()
                  and p.name not in ESP32_GENERATED)


def display(path):
    try:
        return str(Path(path).resolve().relative_to(REPO))
    except ValueError:
        return str(path)


# ---------------------------------------------------------------------------------------------
SELFTESTS = [
    # (name, source, lit_ok, allow_pending, expected [(line, col, token)])
    ("comments, strings and characters hide numbers",
     '// 42 in a line comment\n'
     '/* 43 in a block\n'
     '   comment 44 */\n'
     'const char* s = "45 \\" 46";\n'
     "char c = '7';\n"
     "char d = '\\'';\n"
     "char e = '\\x38';\n"
     'int ok = 0, one = 1;\n'
     'int bad = 2;\n',
     False, False, [(9, 11, "2")]),
    ("every spelling of 0 and 1 is allowed",
     "a = 0x0; b = 0x1u; c = 0b1; d = 1.0f; e = 0.0F; f = 1.; g = .0; h = 0UL; i = 1ULL;\n"
     "j = 00; k = 1e0; l = 0x1p0; m = 1'0'0 - 99; n = 1.0L;\n",
     False, False, [(2, 33, "1'0'0"), (2, 41, "99")]),
    ("number forms are found whole",
     "k = 0x10; l = 0b10; m = 1e1; n = 2.5e-3f; o = 1'000'000; p = 0x1.8p1; q = 10ms;\n"
     "r = 017; s = 0x1e+5; t = .5f; u = 08; v = 10ULL;\n",
     False, False, [(1, 5, "0x10"), (1, 15, "0b10"), (1, 25, "1e1"), (1, 34, "2.5e-3f"),
                    (1, 47, "1'000'000"), (1, 62, "0x1.8p1"), (1, 75, "10ms"),
                    (2, 5, "017"), (2, 14, "0x1e+5"), (2, 26, ".5f"), (2, 35, "08"),
                    (2, 43, "10ULL")]),
    ("identifiers with digits are not numbers",
     "uint8_t x; int16_t y; Serial2.begin(); A0; GPT0; x2 = y3; R_PORT1->PCNTR3 = 0;\n",
     False, False, []),
    ("[N] alone is an index; arithmetic inside brackets is not",
     "x = buf[3];\n"
     "y = buf[ 4 ];\n"
     "z = buf[i + 2];\n"
     "w = buf[2 * i];\n"
     "std::array<int, 4> arr;\n",
     False, False, [(3, 13, "2"), (4, 9, "2"), (5, 17, "4")]),
    ("array dimensions are sizes: reported",
     "uint8_t buf[16];\n"
     "int m[2][3];\n"
     "static const char* names[4];\n"
     "uint8_t Foo::buf_[8] = {};\n"
     "p = new uint8_t[5];\n"
     "typedef uint8_t mac_t[6];\n"
     "void f(const uint8_t data[7]);\n"
     "struct S { char line[9][10]; };\n"
     "uint8_t *ptrs[11];\n",
     False, False, [(1, 13, "16"), (2, 7, "2"), (2, 10, "3"), (3, 26, "4"), (4, 19, "8"),
                    (5, 17, "5"), (6, 23, "6"), (7, 27, "7"), (8, 22, "9"), (8, 25, "10"),
                    (9, 15, "11")]),
    ("array indices in every expression position are allowed",
     "x = buf[3];\n"
     "buf[4] = 0;\n"
     "return buf[5];\n"
     "f(buf[6], m[2][3]);\n"
     "y = a.b[2] + p->q[3];\n"
     "if (c) buf[2] = 1;\n"
     "else buf[2] = 1;\n"
     "n = sizeof buf[2];\n"
     "z = a * buf[2];\n"
     "w = (int)v[4] & mask[5];\n"
     "uint8_t k = table[7];\n"
     "case 1: buf[8] = 0;\n",
     False, False, []),
    ("raw strings and prefixed literals",
     'auto r = R"x(12 )" 13)x";\n'
     'auto u = u8"14";\n'
     "auto l = L'5';\n"
     'auto p = LR"(16\n17)";\n'
     "int after = 18;\n",
     False, False, [(6, 13, "18")]),
    ("preprocessor lines",
     "#include <stdint.h>\n"
     '#include "file2.h"\n'
     "#  include <bits/c++17.h>\n"
     "#if DEBUG_LEVEL > 2\n"
     "#error need version 3\n"
     "#endif\n"
     "#define LOCAL_THING 5\n",
     False, False, [(4, 19, "2"), (7, 21, "5")]),
    ("a // comment continued by a backslash",
     "// comment continues \\\n"
     "   99 still comment\n"
     "int z = 98;\n",
     False, False, [(3, 9, "98")]),
    ("lit-ok honoured in the ESP32 scope, reason required",
     "delay(50); // lit-ok: AT task yield, ESP32 only\n"
     "delay(60); // lit-ok:\n"
     "delay(70); // something else\n",
     True, False, [(2, 1, "lit-ok:"), (2, 7, "60"), (3, 7, "70")]),
    ("lit-ok ignored in the RA4M1 scope",
     "delay(50); // lit-ok: AT task yield\n",
     False, False, [(1, 7, "50")]),
    ("CONFIG-PENDING blocks are reported unless allowed",
     "// CONFIG-PENDING begin\n"
     "#define NEW_THING_MS 25\n"
     "// CONFIG-PENDING end\n"
     "int q = 26;\n",
     False, False, [(2, 22, "25"), (4, 9, "26")]),
    ("CONFIG-PENDING blocks with --allow-pending",
     "// CONFIG-PENDING begin\n"
     "#define NEW_THING_MS 25\n"
     "// CONFIG-PENDING end\n"
     "int q = 26;\n",
     False, True, [(4, 9, "26")]),
    ("columns, negatives, unterminated literals",
     "  foo(3);\n"
     "t = -1; u = 1.0f / 255;\n"
     'const char* bad = "abc\n'
     "int k = 9;\n",
     False, False, [(1, 7, "3"), (2, 20, "255"), (4, 9, "9")]),
    ("line splices and member access",
     "int a = obj.v2 + \\\n  7;\n"
     "x = p->m3[0];\n",
     False, False, [(2, 3, "7")]),
]


def selftest():
    failures = 0
    for name, src, lit_ok, pending, expected in SELFTESTS:
        got = [f.key() for f in check_text(src, lit_ok=lit_ok, allow_pending=pending)]
        if got == expected:
            print("PASS  %s" % name)
        else:
            failures += 1
            print("FAIL  %s\n      expected %s\n      got      %s" % (name, expected, got))
    # The tokeniser must get through real code without raising: the fork, the generated
    # protocol header and the RA4M1 sources when present.
    real = sorted(ESP32_ROOT.glob("*.h")) + sorted(ESP32_ROOT.glob("*.cpp")) + sorted(ESP32_ROOT.glob("*.ino"))
    real += [DSC / "protocol" / "dsentry_protocol.h", DSC / "tests" / "unit" / "test_protocol_c.cpp"]
    real += ra4m1_files() or []
    count = 0
    for p in real:
        if p.is_file():
            try:
                check_text(p.read_text(encoding="utf-8", errors="replace"), str(p))
                count += 1
            except Exception as e:  # report and fail, do not crash the run
                failures += 1
                print("FAIL  tokenising %s: %r" % (display(p), e))
    if count == len([p for p in real if p.is_file()]):
        print("PASS  tokenised %d real source files without error" % count)
    print("check_literals --selftest: %d case(s), %d failure(s)" % (len(SELFTESTS) + 1, failures))
    return 1 if failures else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Report numeric literals in D-Sentry C/C++ sources (acceptance item 20).")
    ap.add_argument("--ra4m1", action="store_true", help="check firmware-ra4m1/dsentry (the default)")
    ap.add_argument("--esp32", action="store_true", help="check the D-Sentry files of the ESP32 fork")
    ap.add_argument("--paths", nargs="+", type=Path, metavar="FILE", help="check these files")
    ap.add_argument("--lit-ok", action="store_true", help="honour // lit-ok: <reason> in --paths files")
    ap.add_argument("--allow-pending", action="store_true",
                    help="skip lines inside // CONFIG-PENDING begin ... end blocks")
    ap.add_argument("--allow-empty", action="store_true", help="a scope with no files is not an error")
    ap.add_argument("--selftest", action="store_true", help="run the built-in tricky-input tests")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()

    jobs = []     # (scope name, files or None, lit_ok)
    if args.esp32:
        jobs.append(("--esp32 (%s)" % display(ESP32_ROOT), esp32_files(), True))
    if args.paths:
        jobs.append(("--paths", list(args.paths), args.lit_ok))
    if args.ra4m1 or not jobs:
        jobs.append(("--ra4m1 (%s)" % display(RA4M1_ROOT), ra4m1_files(), False))

    total_files, findings = 0, []
    for scope, files, lit_ok in jobs:
        if not files:
            msg = "check_literals: %s: no source files%s" % (scope, "" if files is not None else " (directory missing)")
            if args.allow_empty:
                print(msg, file=sys.stderr)
                continue
            print(msg + "; pass --allow-empty if that is expected", file=sys.stderr)
            return 2
        for p in files:
            try:
                text = Path(p).read_text(encoding="utf-8")
            except OSError as e:
                print("check_literals: %s: %s" % (p, e), file=sys.stderr)
                return 2
            total_files += 1
            findings += check_text(text, display(p), lit_ok=lit_ok, allow_pending=args.allow_pending)
    for f in findings:
        print(f)
    files_hit = len({f.path for f in findings})
    if findings:
        print("check_literals: %d literal(s) in %d of %d file(s)" % (len(findings), files_hit, total_files),
              file=sys.stderr)
        return 1
    print("check_literals: %d file(s) clean" % total_files, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
