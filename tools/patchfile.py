#!/usr/bin/env python3
"""
Odyssey-10 Pro -- surgical in-place edits to source files.

WHY THIS EXISTS

Several tools in this repository rewrite files in place: `check_consistency.py --fix`
applies corrections, and ad-hoc patch scripts apply larger edits. Every one of them
needs the same two things, and getting either wrong is expensive:

  1. PRESERVE LINE ENDINGS. This repository is developed on Windows, where the working
     tree is CRLF for most files (`* text=auto`) but LF for shell scripts and git hooks
     (`eol=lf`). A tool that normalises endings turns a four-character fix into a
     whole-file diff, which buries the real change and makes review worthless.

  2. FAIL LOUDLY ON A BAD ANCHOR. A replace() whose anchor does not match does nothing
     and reports success. A replace() whose anchor accidentally matches the ENTIRE FILE
     replaces the entire file. Both are silent.

The second one is not hypothetical. A patch script in this project did:

    for line in raw.split('\\r\\n'):        # assumes CRLF
        if 'BlackBoxRecord' in line: ...

On an LF file, `split('\\r\\n')` returns a single element containing the whole file. That
"line" matched, and `raw.replace(target, replacement, 1)` reduced a 1244-line module to
one line. It was restored from git, but nothing in the script noticed or objected.

So the primitives here refuse the ambiguous cases rather than guessing:

    with TextFile("tools/check_consistency.py") as f:
        f.replace_once("old text", "new text")
    # written on exit, endings preserved, syntax verified

Nothing is written if any replacement fails, so a partially-applied patch is not a
state this module can produce.

WRITE THESE SCRIPTS TO A FILE. DO NOT PIPE THEM THROUGH A SHELL HEREDOC.

A script delivered to python via a shell heredoc loses one level of backslash escaping
before the shell ever sees it, and quoting does not help -- a `<<'EOF'` heredoc, which
the shell guarantees is literal, still arrives mangled, because the damage happens
upstream of the shell. Demonstrated:

    printed into a quoted heredoc:   \\d     arrives as:  \d
    echo '\\d' in single quotes:       \\d     prints as:   \d

The consequences are silent and specific:

    "\\n"  intended as backslash-n   ->  "\n"  ->  Python reads a NEWLINE
    "\\b"  intended as backslash-b   ->  "\b"  ->  Python reads a BACKSPACE (0x08)
    r"\\d"  intended as a regex       ->  r"\d"  ->  a different pattern entirely

The third is merely wrong. The second is worse: it writes a raw control character into
a source file, where it is invisible in every editor and breaks the file for everyone.
That happened to tools/check_consistency.py and cost a restore from git.

The reliable method is to write the patch script to a file and run it by path, so the
script's bytes never pass through shell escaping at all. The guards below exist because
"be careful" is not a mechanism.
"""

from __future__ import annotations

import ast
import os
from pathlib import Path

__all__ = ["detect_eol", "read_text", "write_text", "TextFile", "PatchError"]


class PatchError(Exception):
    """A patch could not be applied safely. Nothing was written."""


#  Control characters that should never appear in this project's source. Tab and
#  newline are legitimate; carriage return is handled as a line ending. Everything else
#  in the C0 range is either a mangled escape sequence or a corrupted file, and both are
#  invisible in an editor -- which is exactly why they need catching mechanically.
_FORBIDDEN_CONTROLS = {c for c in range(0x20)} - {0x09, 0x0A, 0x0D}
_FORBIDDEN_CONTROLS.add(0x7F)

_CONTROL_NAMES = {0x00: "NUL", 0x07: "BEL", 0x08: "BACKSPACE", 0x0B: "VT",
                  0x0C: "FORM FEED", 0x1B: "ESC", 0x7F: "DEL"}


def _reject_controls(text, what):
    """
    Raises if `text` holds a control character.

    A BACKSPACE here almost always means a "\\b" in a patch script lost a backslash on
    its way through a shell and became Python's \x08. The character is invisible, the
    file still looks right, and the next tool to read it fails somewhere unrelated.
    """
    for i, ch in enumerate(text):
        o = ord(ch)
        if o in _FORBIDDEN_CONTROLS:
            name = _CONTROL_NAMES.get(o, f"0x{o:02X}")
            ctx = repr(text[max(0, i - 30):i + 30])
            raise PatchError(
                f"{what} contains a {name} control character at offset {i}. This is "
                f"almost always a backslash escape that was mangled on its way through "
                f"a shell -- write the patch script to a file and run it by path "
                f"instead of piping it through a heredoc. Context: {ctx}")


# -------------------------------------------------------------------------------------
#  Line endings
# -------------------------------------------------------------------------------------
def detect_eol(raw: bytes) -> str:
    """
    The file's dominant line ending, decided by COUNTING rather than by membership.

    `"\\r\\n" in raw` is the tempting version and it is wrong for a mixed file: one stray
    CRLF in a 2000-line LF file would make every line get rewritten. Counting picks the
    ending the file actually uses and leaves a stray alone.

    A file with no newline at all reports the platform default, because there is nothing
    to preserve and something has to be chosen.
    """
    crlf = raw.count(b"\r\n")
    lf = raw.count(b"\n") - crlf
    if crlf == 0 and lf == 0:
        return os.linesep
    return "\r\n" if crlf > lf else "\n"


def read_text(path) -> tuple[str, str]:
    """Returns (LF-normalised text, the file's dominant line ending)."""
    raw = Path(path).read_bytes()
    return raw.decode("utf-8").replace("\r\n", "\n"), detect_eol(raw)


def write_text(path, text: str, eol: str, *, allow_shrink: bool = False) -> None:
    """
    Writes LF-normalised text back with the given line ending.

    Two guards, both learned the hard way:

      * A result dramatically smaller than the original is refused. Every legitimate
        edit in this project's history has been a small fraction of the file; losing
        most of it means an anchor matched something it should not have.
      * A .py file that no longer parses is refused, because the next thing to run it
        would be CI or a git hook, and a syntax error there is a worse place to find out.
    """
    path = Path(path)
    if eol not in ("\n", "\r\n"):
        raise PatchError(f"refusing to write an unknown line ending {eol!r}")

    if path.exists() and not allow_shrink:
        before = len(path.read_bytes())
        after = len(text.encode("utf-8"))
        # 200 bytes of slack so genuinely tiny files are not fussed over.
        if before > 200 and after < before * 0.5:
            raise PatchError(
                f"refusing to write {path}: it would shrink from {before} to {after} "
                f"bytes ({100 * after / before:.0f}%). An anchor almost certainly "
                f"matched more than it should have. Pass allow_shrink=True if this "
                f"really is intended."
            )

    _reject_controls(text, f"the text being written to {path}")

    if path.suffix == ".py":
        try:
            ast.parse(text)
        except SyntaxError as exc:
            raise PatchError(
                f"refusing to write {path}: the result is not valid Python "
                f"({exc.msg} at line {exc.lineno})"
            ) from exc

    out = text.replace("\n", "\r\n") if eol == "\r\n" else text
    path.write_bytes(out.encode("utf-8"))


# -------------------------------------------------------------------------------------
#  Editing
# -------------------------------------------------------------------------------------
class TextFile:
    """
    A file opened for surgical edits, with its line endings remembered.

    Used as a context manager it writes on a clean exit and writes nothing if an
    exception propagates, so a failed patch leaves the file untouched rather than
    half-applied.
    """

    def __init__(self, path, *, allow_shrink: bool = False):
        self.path = Path(path)
        self.text, self.eol = read_text(self.path)
        self._original = self.text
        self.allow_shrink = allow_shrink
        self.edits = 0

    # -- replacements ------------------------------------------------------------------
    def replace_once(self, old: str, new: str) -> "TextFile":
        """
        Replaces exactly one occurrence of `old`, or raises.

        Refuses four cases that a bare str.replace() accepts silently:
          * `old` is absent      -- the anchor moved, and the edit would be a no-op
          * `old` appears twice  -- which one was meant is not knowable
          * `old` is the whole file -- the bug this module was written for
          * `old` is empty       -- replace() would splice `new` between every character
        """
        if not old:
            raise PatchError(f"{self.path}: empty anchor")
        _reject_controls(old, f"{self.path}: the anchor")
        _reject_controls(new, f"{self.path}: the replacement")
        if old == self.text:
            raise PatchError(
                f"{self.path}: the anchor matches the ENTIRE file. This is what a "
                f"line-splitting bug looks like -- the whole file became one 'line'. "
                f"Refusing."
            )
        n = self.text.count(old)
        if n == 0:
            raise PatchError(
                f"{self.path}: anchor not found: {_excerpt(old)}"
            )
        if n > 1:
            raise PatchError(
                f"{self.path}: anchor appears {n} times, so the target is ambiguous: "
                f"{_excerpt(old)}"
            )
        self.text = self.text.replace(old, new, 1)
        self.edits += 1
        return self

    def replace_all(self, old: str, new: str, *, expect: int | None = None) -> "TextFile":
        """Replaces every occurrence. `expect` asserts how many there should be."""
        if not old:
            raise PatchError(f"{self.path}: empty anchor")
        _reject_controls(old, f"{self.path}: the anchor")
        _reject_controls(new, f"{self.path}: the replacement")
        n = self.text.count(old)
        if n == 0:
            raise PatchError(f"{self.path}: anchor not found: {_excerpt(old)}")
        if expect is not None and n != expect:
            raise PatchError(
                f"{self.path}: expected {expect} occurrences of {_excerpt(old)}, "
                f"found {n}"
            )
        self.text = self.text.replace(old, new)
        self.edits += n
        return self

    def insert_before(self, anchor: str, block: str) -> "TextFile":
        """Inserts `block` immediately before a unique anchor."""
        return self.replace_once(anchor, block + anchor)

    def insert_after(self, anchor: str, block: str) -> "TextFile":
        """Inserts `block` immediately after a unique anchor."""
        return self.replace_once(anchor, anchor + block)

    def require(self, needle: str) -> "TextFile":
        """Asserts a string is present, without changing anything."""
        if needle not in self.text:
            raise PatchError(f"{self.path}: expected to find {_excerpt(needle)}")
        return self

    # -- writing -----------------------------------------------------------------------
    @property
    def changed(self) -> bool:
        return self.text != self._original

    def save(self) -> bool:
        """Writes if anything changed. Returns whether it wrote."""
        if not self.changed:
            return False
        write_text(self.path, self.text, self.eol, allow_shrink=self.allow_shrink)
        self._original = self.text
        return True

    def __enter__(self) -> "TextFile":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is None:
            self.save()
        return False        # never swallow the exception


def _excerpt(s: str, n: int = 60) -> str:
    one_line = " ".join(s.split())
    return repr(one_line[:n] + ("..." if len(one_line) > n else ""))
