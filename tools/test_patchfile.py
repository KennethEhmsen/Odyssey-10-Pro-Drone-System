#!/usr/bin/env python3
"""
Odyssey-10 Pro -- tests for the in-place edit helpers.

The first test in `test_the_bug_that_caused_this` reproduces, exactly, the failure that
this module was written to prevent: a patch script split an LF file on "\\r\\n", got the
whole file back as a single "line", matched it, and replaced a 1244-line module with one
line. Nothing objected.

    python3 tools/test_patchfile.py
"""

import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from patchfile import (PatchError, TextFile, detect_eol, read_text, write_text)

_pass = 0
_fail = 0


def check(cond, what):
    global _pass, _fail
    if cond:
        _pass += 1
        print(f"  PASS  {what}")
    else:
        _fail += 1
        print(f"  FAIL  {what}")


def raises(fn, what):
    try:
        fn()
    except PatchError:
        check(True, what)
        return
    except Exception as exc:               # noqa: BLE001 - a wrong exception is a fail
        check(False, f"{what}  (raised {type(exc).__name__}, not PatchError)")
        return
    check(False, f"{what}  (nothing raised)")


def section(title):
    print(f"\n{title}\n{'-' * len(title)}")


class tmpfile:
    """A temporary file with exact bytes, cleaned up afterwards."""

    def __init__(self, data: bytes, suffix=".txt"):
        self.data = data
        self.suffix = suffix

    def __enter__(self):
        fd, name = tempfile.mkstemp(suffix=self.suffix)
        os.close(fd)
        self.path = Path(name)
        self.path.write_bytes(self.data)
        return self.path

    def __exit__(self, *a):
        try:
            self.path.unlink()
        except OSError:
            pass
        return False


# =====================================================================================
def test_detect_eol():
    section("Line endings are detected by counting, not by membership")

    check(detect_eol(b"a\r\nb\r\nc\r\n") == "\r\n", "an all-CRLF file reports CRLF")
    check(detect_eol(b"a\nb\nc\n") == "\n", "an all-LF file reports LF")

    # The case `"\r\n" in raw` gets wrong. One stray CRLF must not condemn the file to
    # a whole-file rewrite.
    mostly_lf = b"\n".join([b"x"] * 500) + b"\r\n"
    check(detect_eol(mostly_lf) == "\n",
          "one stray CRLF in a 500-line LF file still reports LF")

    mostly_crlf = b"\r\n".join([b"x"] * 500) + b"\n"
    check(detect_eol(mostly_crlf) == "\r\n",
          "one stray LF in a 500-line CRLF file still reports CRLF")

    check(detect_eol(b"no newline at all") == os.linesep,
          "a file with no newline falls back to the platform default")


def test_round_trip_is_byte_identical():
    section("An unchanged file is rewritten byte for byte")

    for data in (b"a\r\nb\r\nc\r\n", b"a\nb\nc\n", b"a\r\nb\r\nc",
                 b"\r\n\r\n", b"single line no newline"):
        with tmpfile(data) as p:
            text, eol = read_text(p)
            write_text(p, text, eol)
            check(p.read_bytes() == data,
                  f"round trip preserves {data[:14]!r}...")


def test_edits_preserve_endings():
    section("An edit changes only what it was asked to change")

    original = b"alpha\r\nbeta\r\ngamma\r\n"
    with tmpfile(original) as p:
        with TextFile(p) as f:
            f.replace_once("beta", "BETA")
        after = p.read_bytes()
        check(after == b"alpha\r\nBETA\r\ngamma\r\n",
              "a CRLF file stays CRLF through an edit")
        check(after.count(b"\r\n") == 3, "no line endings were added or lost")

    original = b"alpha\nbeta\ngamma\n"
    with tmpfile(original) as p:
        with TextFile(p) as f:
            f.replace_once("beta", "BETA")
        check(p.read_bytes() == b"alpha\nBETA\ngamma\n",
              "an LF file stays LF through an edit")

    # eol=lf files -- shell scripts and git hooks -- must never gain a CR, or /bin/sh
    # rejects them with "bad interpreter".
    hook = b"#!/bin/sh\nset -e\necho hi\n"
    with tmpfile(hook, suffix="") as p:
        with TextFile(p) as f:
            f.replace_once("echo hi", "echo bye")
        check(b"\r" not in p.read_bytes(),
              "a shell script never gains a CR, which would break its interpreter line")


def test_the_bug_that_caused_this():
    section("The whole-file anchor is refused")

    # Verbatim reproduction. The script did raw.split('\r\n') on an LF file, took the
    # single element that came back, and replaced it. patchfile refuses that shape.
    body = "\n".join(f"line {i}" for i in range(1244)) + "\n"
    with tmpfile(body.encode()) as p:
        text, _ = read_text(p)
        lines = text.split("\r\n")           # the original mistake
        check(len(lines) == 1,
              "splitting an LF file on CRLF still yields one giant 'line'")

        f = TextFile(p)
        raises(lambda: f.replace_once(lines[0], "    m = re.search(...)"),
               "replacing that 'line' is refused as a whole-file anchor")
        check(p.read_bytes() == body.encode(),
              "and the file on disk is untouched")

    # The other half of the guard: even a legitimate-looking edit that would gut the
    # file is refused.
    with tmpfile(body.encode()) as p:
        raises(lambda: write_text(p, "just one line\n", "\n"),
               "writing a result 99% smaller than the original is refused")
        check(p.read_bytes() == body.encode(), "and again the file is untouched")
        # ...unless it is genuinely what was meant.
        write_text(p, "just one line\n", "\n", allow_shrink=True)
        check(p.read_bytes() == b"just one line\n",
              "allow_shrink=True permits it explicitly")


def test_bad_anchors():
    section("Anchors that cannot be resolved are refused, not guessed")

    data = b"one\ntwo\ntwo\nthree\n"
    with tmpfile(data) as p:
        f = TextFile(p)
        raises(lambda: f.replace_once("missing", "x"),
               "an anchor that is not present is refused")
        raises(lambda: f.replace_once("two", "TWO"),
               "an anchor appearing twice is refused as ambiguous")
        raises(lambda: f.replace_once("", "x"),
               "an empty anchor is refused")
        raises(lambda: f.require("nope"), "require() fails on a missing string")
        check(p.read_bytes() == data, "none of those touched the file")

        # replace_all handles the ambiguous case, when that is what was meant.
        with TextFile(p) as g:
            g.replace_all("two", "TWO", expect=2)
        check(p.read_bytes() == b"one\nTWO\nTWO\nthree\n",
              "replace_all with the right expected count succeeds")

    with tmpfile(data) as p:
        f = TextFile(p)
        raises(lambda: f.replace_all("two", "TWO", expect=5),
               "replace_all with the wrong expected count is refused")


def test_python_must_still_parse():
    section("A Python file that no longer parses is not written")

    src = b"def f():\n    return 1\n\n\ndef g():\n    return 2\n"
    with tmpfile(src, suffix=".py") as p:
        f = TextFile(p)
        f.replace_once("return 1", "return (((")
        raises(f.save, "a syntax error is caught before the file is written")
        check(p.read_bytes() == src, "and the working version survives")

    with tmpfile(src, suffix=".py") as p:
        with TextFile(p) as f:
            f.replace_once("return 1", "return 42")
        check(b"return 42" in p.read_bytes(), "a valid edit still writes")


def test_nothing_written_on_failure():
    section("A failed patch leaves nothing half-applied")

    data = b"aaa\nbbb\nccc\n"
    with tmpfile(data) as p:
        try:
            with TextFile(p) as f:
                f.replace_once("aaa", "AAA")      # succeeds
                f.replace_once("zzz", "ZZZ")      # fails
        except PatchError:
            pass
        check(p.read_bytes() == data,
              "the earlier successful edit is not written when a later one fails")

    with tmpfile(data) as p:
        with TextFile(p) as f:
            f.replace_once("aaa", "AAA")
            f.replace_once("ccc", "CCC")
        check(p.read_bytes() == b"AAA\nbbb\nCCC\n",
              "when every edit succeeds, all of them are written")

    with tmpfile(data) as p:
        before = p.stat().st_mtime_ns
        with TextFile(p) as f:
            f.require("aaa")
        check(p.stat().st_mtime_ns == before,
              "a no-op patch does not rewrite the file at all")


def test_insert_helpers():
    section("Insertion helpers")

    with tmpfile(b"start\nEND\n") as p:
        with TextFile(p) as f:
            f.insert_before("END", "middle\n")
        check(p.read_bytes() == b"start\nmiddle\nEND\n", "insert_before places a block")

    with tmpfile(b"start\nEND\n") as p:
        with TextFile(p) as f:
            f.insert_after("start\n", "middle\n")
        check(p.read_bytes() == b"start\nmiddle\nEND\n", "insert_after places a block")


def main():
    print("=" * 69)
    print(" Odyssey-10 Pro -- in-place edit helper tests")
    print("=" * 69)
    test_detect_eol()
    test_round_trip_is_byte_identical()
    test_edits_preserve_endings()
    test_the_bug_that_caused_this()
    test_bad_anchors()
    test_python_must_still_parse()
    test_nothing_written_on_failure()
    test_insert_helpers()
    print("\n" + "=" * 69)
    print(f" {_pass} passed, {_fail} failed")
    print("=" * 69)
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
