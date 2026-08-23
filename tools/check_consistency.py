#!/usr/bin/env python3
"""
Odyssey-10 Pro -- project consistency checker.

Almost every defect found in this project's review was a DISAGREEMENT rather than a
logic error: the specification said 115200 baud and the firmware opened 9600; the motor
diagram said clockwise and the pinout said counter-clockwise; the BOM total did not
match its own line items; the code declared a sensor the firmware never read. Each was
individually obvious and collectively invisible, because nothing checked that the parts
still agreed with each other.

This tool checks. Where a disagreement has one mechanically correct resolution, it can
also fix it.

    python tools/check_consistency.py            # report only, exit 1 if problems
    python tools/check_consistency.py --fix       # apply the safe corrections
    python tools/check_consistency.py --list      # show every check and what it does

DESIGN RULE: the CODE is the source of truth for anything that has a code
representation, and --fix rewrites the DOCUMENTATION to match. It never edits firmware
to satisfy a document, because a document cannot be compiled or tested.

Checks that cannot be resolved mechanically are reported and never auto-fixed.

Coverage note: `spec-constants` handles values in table cells, whose shape is
predictable. `prose-constants` handles values asserted in sentences and formula blocks,
which is where the CRUISE_CURRENT_A error hid for three revisions -- the constant moved
and the prose kept quoting the old figure. Those patterns are explicit rather than
generic, so the check reports how many claims it verified; if that number stops growing
while the document does, the check is falling behind.
"""

import argparse
import csv
import os
import re
import struct

import patchfile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

SPEC = ROOT / "docs" / "Odyssey-10-Pro-Drone-System.md"
RESOLUTION = ROOT / "docs" / "review-findings-resolution.md"
BOM = ROOT / "hardware" / "bom.csv"
CONFIG_H = ROOT / "firmware" / "flight-controller" / "include" / "config.h"
TYPES_H = ROOT / "firmware" / "flight-controller" / "include" / "types.h"
DECODER = ROOT / "tools" / "blackbox_decode.py"
LINK_H = ROOT / "shared" / "odyssey_link.h"
GITATTR = ROOT / ".gitattributes"

BS = chr(92)   # backslash, written this way so it survives shell heredocs

SOURCE_GLOBS = ["firmware/**/*.cpp", "firmware/**/*.h", "shared/*.h",
                "tools/host_tests/*.cpp", "tools/host_tests/*.h"]


# =====================================================================================
#  Reporting
# =====================================================================================
class Report:
    def __init__(self):
        self.problems = []   # (severity, check, message, fixable)
        self.fixed = []
        self.checks_run = 0

    def problem(self, check, message, fixable=False, severity="error"):
        self.problems.append((severity, check, message, fixable))

    def fix(self, check, message):
        self.fixed.append((check, message))

    def summary(self):
        errors = [p for p in self.problems if p[0] == "error"]
        warns = [p for p in self.problems if p[0] == "warn"]

        if self.fixed:
            print("\nFIXED")
            print("-----")
            for check, msg in self.fixed:
                print(f"  [{check}] {msg}")

        if self.problems:
            print("\nPROBLEMS")
            print("--------")
            for sev, check, msg, fixable in self.problems:
                tag = "ERROR" if sev == "error" else "warn "
                hint = "  (fixable with --fix)" if fixable else ""
                print(f"  {tag} [{check}] {msg}{hint}")

        if getattr(self, "note", None):
            print()
            print("NOTES")
            print("-----")
            for n in self.note:
                print(f"  {n}")

        print()
        print("=" * 78)
        print(f"  {self.checks_run} checks | {len(self.fixed)} fixed | "
              f"{len(errors)} errors | {len(warns)} warnings")
        print("=" * 78)
        return 1 if errors else 0


# =====================================================================================
#  Helpers
# =====================================================================================
# Line endings must survive a --fix untouched. On Windows these files are checked out
# with CRLF, and a fixer that normalised them to LF would turn a one-character
# correction into a whole-file diff -- burying the actual change and making the tool
# something nobody wants to run.
_EOL = {}


def read(path):
    """Reads a file, remembering its line ending, and returns LF-normalised text."""
    text, eol = patchfile.read_text(path)
    _EOL[path] = eol
    return text


def write(path, text):
    """
    Writes text back using whatever line ending the file already had.

    Delegates to patchfile, which refuses a result dramatically smaller than the
    original and refuses a .py file that no longer parses. --fix rewrites source in
    place, so a bad anchor here would silently damage a file rather than report an
    inconsistency.
    """
    patchfile.write_text(path, text, _EOL.get(path, os.linesep))


def config_defines(text):
    """
    Extracts #define NAME VALUE pairs by regex.

    This is the FALLBACK. It cannot see through #if / #elif, so once config.h grew the
    PROP_BLADES switch it would happily return the 3-blade values for a 2-blade build --
    both branches are present in the text and the last one wins. Use resolved_defines()
    instead wherever correctness matters.
    """
    out = {}
    for m in re.finditer(r"^\s*#define\s+([A-Z0-9_]+)\s+([^\n/]+)", text, re.M):
        out[m.group(1)] = m.group(2).strip()
    return out


# Arithmetic-only evaluator for macro expansions like "(3.30f * 6)". Restricted to
# characters that cannot express anything but a number, so there is nothing to escape.
_ARITH_OK = set("0123456789.+-*/() \t")


def _eval_numeric(expr):
    expr = re.sub(r"(?<=[\d.])[fFuUlL]+", "", expr).strip()
    if not expr or not set(expr) <= _ARITH_OK:
        return None
    try:
        return float(eval(expr, {"__builtins__": {}}, {}))
    except Exception:
        return None


_RESOLVED_CACHE = {}


def resolved_defines(extra_flags=()):
    """
    Returns config.h constants as NUMBERS, resolved by the real C preprocessor.

    config.h selects most of the propulsion constants through a #if on PROP_BLADES, and
    routes two of them through an #ifndef indirection so they can be overridden. Neither
    is visible to a regex, and guessing wrong would mean the checker validates the
    documentation against constants the firmware is not actually compiled with.

    So the preprocessor does the work: a probe file is expanded with the same include
    path the firmware uses, and the expansions are evaluated as arithmetic. Passing
    extra_flags lets a caller ask what a different build would produce, for example
    ("-DPROP_BLADES=3",).
    """
    key = tuple(extra_flags)
    if key in _RESOLVED_CACHE:
        return _RESOLVED_CACHE[key]

    import shutil, subprocess, tempfile

    names = sorted(set(config_defines(read(CONFIG_H)).keys()))
    cc = shutil.which("gcc") or shutil.which("g++") or shutil.which("clang")
    if not cc or not names:
        return {}

    lines = ['#include "config.h"']
    for n in names:
        # The label is QUOTED so the preprocessor leaves it alone -- an unquoted label
        # gets expanded too, and the output comes back as "ODYVAL 2 2" with no way to
        # tell which constant it belonged to.
        lines.append(f'ODYVAL "{n}" {n}')
    probe = "\n".join(lines) + "\n"

    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "probe.cpp"
        src.write_text(probe, encoding="utf-8")
        try:
            r = subprocess.run(
                [cc, "-E", "-x", "c++", str(src),
                 "-I", str(CONFIG_H.parent), "-I", str(ROOT / "shared"), *extra_flags],
                capture_output=True, text=True, timeout=60)
        except Exception:
            return {}

    out = {}
    for line in r.stdout.splitlines():
        if not line.startswith("ODYVAL "):
            continue
        m = re.match(r'ODYVAL\s+"([A-Z0-9_]+)"\s+(.*)$', line)
        if not m:
            continue
        name, expansion = m.group(1), m.group(2)
        if expansion.strip() == name:       # macro did not expand to anything numeric
            continue
        v = _eval_numeric(expansion)
        if v is not None:
            out[name] = v

    _RESOLVED_CACHE[key] = out
    return out


def strip_c_comments(src):
    """Character state machine -- handles strings, chars, escapes exactly."""
    out, i, n, state = [], 0, len(src), "code"
    while i < n:
        c, nxt = src[i], src[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line"; i += 2; continue
            if c == "/" and nxt == "*":
                state = "block"; i += 2; continue
            if c == '"':
                state = "str"; i += 1; continue
            if c == "'":
                state = "chr"; i += 1; continue
            out.append(c); i += 1; continue
        if state == "line":
            if c == "\n":
                state = "code"; out.append(c)
            i += 1; continue
        if state == "block":
            if c == "*" and nxt == "/":
                state = "code"; i += 2; continue
            i += 1; continue
        if state in ("str", "chr"):
            if c == BS:
                i += 2; continue
            if (state == "str" and c == '"') or (state == "chr" and c == "'"):
                state = "code"
            i += 1; continue
    return "".join(out)


#  Directories holding code this project did not write and must not modify.
#
#  Until the ESP-IDF build existed there was no third-party source in the tree, so
#  source_files() could glob freely. The first `--fix` run after it appeared rewrote
#  whitespace in 255 files belonging to arduino-esp32, esp-modem and esp-dl, and
#  reported brace imbalances in their examples as though they were defects here.
#
#  Nothing was committed -- both directories are gitignored -- but a tool that edits
#  vendored code on its own initiative is a tool that will eventually do so somewhere
#  that matters.
VENDORED_DIRS = ("managed_components", "components", "build", ".git")


def _is_vendored(path):
    return any(part in VENDORED_DIRS for part in path.parts)


def source_files():
    files = []
    for g in SOURCE_GLOBS:
        files.extend(p for p in sorted(ROOT.glob(g)) if not _is_vendored(p))
    return files


# =====================================================================================
#  CHECK: comment line continuations
# =====================================================================================
def check_comment_backslash(rep, fix):
    """A // comment ending in a backslash silently swallows the following line."""
    rep.checks_run += 1
    for f in source_files():
        for i, line in enumerate(read(f).split("\n"), 1):
            s = line.rstrip("\r")
            if s.lstrip().startswith("//") and s.endswith(BS):
                rel = f.relative_to(ROOT).as_posix()
                rep.problem(
                    "comment-continuation",
                    f"{rel}:{i} a // comment ends in a backslash, so the next line is "
                    f"silently swallowed. Convert the block to /* */ or reword.",
                )


# =====================================================================================
#  CHECK: bracket balance
# =====================================================================================
def check_brackets(rep, fix):
    rep.checks_run += 1
    for f in source_files():
        src = read(f)
        s = strip_c_comments(src)
        rel = f.relative_to(ROOT).as_posix()
        for o, c, name in (("{", "}", "brace"), ("(", ")", "paren"), ("[", "]", "bracket")):
            d = s.count(o) - s.count(c)
            if d:
                rep.problem("brackets", f"{rel}: {name} imbalance {d:+d}")


# =====================================================================================
#  CHECK: trailing whitespace and final newline  (auto-fixable)
# =====================================================================================
def check_whitespace(rep, fix):
    rep.checks_run += 1
    targets = source_files() + [SPEC, RESOLUTION, ROOT / "README.md"]
    for f in targets:
        if not f.exists():
            continue
        src = read(f)
        rel = f.relative_to(ROOT).as_posix()
        lines = src.split("\n")
        trailing = [i for i, l in enumerate(lines, 1) if l != l.rstrip()]
        needs_nl = src and not src.endswith("\n")

        if not trailing and not needs_nl:
            continue

        if fix:
            new = "\n".join(l.rstrip() for l in lines)
            if not new.endswith("\n"):
                new += "\n"
            write(f, new)
            bits = []
            if trailing:
                bits.append(f"{len(trailing)} lines of trailing whitespace")
            if needs_nl:
                bits.append("missing final newline")
            rep.fix("whitespace", f"{rel}: {', '.join(bits)}")
        else:
            if trailing:
                rep.problem("whitespace",
                            f"{rel}: trailing whitespace on {len(trailing)} line(s)",
                            fixable=True, severity="warn")
            if needs_nl:
                rep.problem("whitespace", f"{rel}: no final newline",
                            fixable=True, severity="warn")


# =====================================================================================
#  CHECK: BOM arithmetic and the total quoted in the specification  (auto-fixable)
# =====================================================================================
def bom_total():
    rows = list(csv.DictReader(open(BOM, encoding="utf-8")))
    return rows, sum(float(r["Total USD"]) for r in rows)


def check_bom(rep, fix):
    rep.checks_run += 1
    if not BOM.exists():
        rep.problem("bom", "hardware/bom.csv is missing")
        return

    rows, total = bom_total()

    # Per-line qty x unit = total, where qty is a plain integer.
    for r in rows:
        q = r["Qty"].strip()
        if q.isdigit():
            want = int(q) * float(r["Unit Price USD"])
            if abs(want - float(r["Total USD"])) > 0.005:
                rep.problem("bom",
                            f"{r['Component']}: {q} x {r['Unit Price USD']} = {want:.2f}"
                            f" but the row says {r['Total USD']}")

    # The specification must quote the same master total.
    spec = read(SPEC)
    m = re.search(r"\*\*Master total\*\*[^|]*\|[^|]*\|[^|]*\|[^|]*\|[^|]*\|\s*"
                  r"\*\*\$([0-9,]+\.\d\d)\*\*", spec)
    if not m:
        rep.problem("bom", "could not find the master total row in the specification",
                    severity="warn")
        return

    stated = float(m.group(1).replace(",", ""))
    if abs(stated - total) > 0.005:
        if fix:
            spec = spec.replace(f"**${m.group(1)}**", f"**${total:.2f}**")
            write(SPEC, spec)
            rep.fix("bom", f"specification master total ${stated:.2f} -> ${total:.2f}")
        else:
            rep.problem("bom",
                        f"specification says ${stated:.2f}, bom.csv sums to ${total:.2f}",
                        fixable=True)


# =====================================================================================
#  CHECK: constants shared between config.h and the specification  (auto-fixable)
# =====================================================================================
# Each entry: (label, how to derive the expected value from config.h, regex over the
# spec whose group(1) must equal it).
def spec_constants(defines):
    resolved = resolved_defines()
    cells = int(resolved.get("CELL_COUNT", defines["CELL_COUNT"]))

    def volts(key):
        cell = key.replace("PACK", "CELL")
        if cell in resolved:
            return resolved[cell] * cells
        return float(defines[cell].rstrip("f")) * cells

    return [
        # The pack column of the threshold table must be cell voltage x CELL_COUNT.
        ("full pack voltage", f"{volts('PACK_FULL_V'):.2f}",
         r"\|\s*Full\s*\|\s*4\.20 V\s*\|\s*([\d.]+) V"),
        ("nominal pack voltage", f"{volts('PACK_NOMINAL_V'):.2f}",
         r"\|\s*Nominal\s*\|\s*3\.70 V\s*\|\s*([\d.]+) V"),
        ("critical cutoff", f"{volts('PACK_CRITICAL_V'):.2f}",
         r"\|\s*Critical\s*\|\s*3\.30 V\s*\|\s*([\d.]+) V"),
        ("warning threshold", f"{volts('PACK_WARN_V'):.2f}",
         r"\|\s*Warning\s*\|\s*3\.40 V\s*\|\s*([\d.]+) V"),
        ("launch minimum", f"{volts('PACK_LAUNCH_MIN_V'):.2f}",
         r"\|\s*Launch minimum\s*\|\s*3\.85 V\s*\|\s*([\d.]+) V"),
        ("all-up weight", str(int(_config_value(defines, "AIRFRAME_AUW_G"))),
         r"\*\*All-up weight \(AUW\)\*\*\s*\|\s*\*\*(\d+) g\*\*"),
    ]


def check_spec_constants(rep, fix):
    rep.checks_run += 1
    defines = config_defines(read(CONFIG_H))
    spec = read(SPEC)
    changed = False

    for label, expected, pattern in spec_constants(defines):
        m = re.search(pattern, spec)
        if not m:
            rep.problem("spec-constants",
                        f"{label}: could not locate it in the specification",
                        severity="warn")
            continue
        if m.group(1) != expected:
            if fix:
                start, end = m.span(1)
                spec = spec[:start] + expected + spec[end:]
                changed = True
                rep.fix("spec-constants",
                        f"{label}: specification said {m.group(1)}, config.h says {expected}")
            else:
                rep.problem("spec-constants",
                            f"{label}: specification says {m.group(1)}, "
                            f"config.h says {expected}",
                            fixable=True)

    if changed:
        write(SPEC, spec)

    # Thrust-to-weight must follow from the two masses rather than being asserted.
    auw = _config_value(defines, "AIRFRAME_AUW_G")
    thrust = _config_value(defines, "MOTOR_MAX_THRUST_G") * 4.0
    twr = thrust / auw
    m = re.search(r"thrust-to-weight ratio of ([\d.]+):1", spec)
    if m and abs(float(m.group(1)) - twr) > 0.02:
        if fix:
            spec = spec.replace(f"thrust-to-weight ratio of {m.group(1)}:1",
                                f"thrust-to-weight ratio of {twr:.2f}:1")
            write(SPEC, spec)
            rep.fix("spec-constants", f"TWR {m.group(1)} -> {twr:.2f}")
        else:
            rep.problem("spec-constants",
                        f"TWR: specification says {m.group(1)}:1, "
                        f"{thrust:.0f} g / {auw:.0f} g = {twr:.2f}:1",
                        fixable=True)


# =====================================================================================
#  CHECK: BlackBox record layout agrees between firmware and decoder
# =====================================================================================
def _record_body():
    """
    The declaration body of BlackBoxRecord, and whether it is declared packed.

    Located by NAME rather than by matching the whole declaration line. Matching the
    full text meant that dropping the packed attribute made the struct simply not be
    found, and the check died with an IndexError instead of reporting the one thing
    that had actually gone wrong.
    """
    types = read(TYPES_H)
    m = re.search(r"struct\s+([^{]*?)BlackBoxRecord\s*\{", types)
    if not m:
        return None, False
    return types[m.end():].split("};")[0], ("packed" in m.group(1))


def _record_field_names():
    """Field names of BlackBoxRecord, in declaration order."""
    body, _ = _record_body()
    if body is None:
        return []
    names = []
    for line in body.splitlines():
        line = line.split("//")[0].strip().rstrip(";")
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        for n in parts[1].split(","):
            n = n.strip()
            if n:
                names.append(n)
    return names


def _expand_fmt(fmt):
    """'<I3h H' -> ['I','h','h','h','H'], so per-field offsets can be computed."""
    codes, count = [], ""
    for ch in fmt:
        if ch in "<>=!@ ":
            continue
        if ch.isdigit():
            count += ch
            continue
        codes.extend([ch] * (int(count) if count else 1))
        count = ""
    return codes


def check_blackbox_layout(rep, fix):
    """
    Verifies the C struct and the Python decoder agree, FIELD BY FIELD.

    The previous version summed a hand-written table of type sizes. That catches a
    changed field but not a changed LAYOUT -- drop the packed attribute and the sum is
    unchanged while every offset past the first misaligned member moves. A log decoded
    against wrong offsets does not fail loudly; it produces plausible numbers that are
    silently wrong, which is the worst possible outcome for a flight recorder. So the
    real compiler is asked for the real offsets.
    """
    rep.checks_run += 1
    import shutil, subprocess, tempfile, os

    types = read(TYPES_H)
    dec = read(DECODER)

    # ---- the decoder's declared formats --------------------------------------------
    fmts = {}
    for m in re.finditer(r"_V(\d+)_FMT\s*=\s*(.+)", dec):
        ver, expr = int(m.group(1)), m.group(2).strip()
        lits = re.findall(r'"([^"]*)"', expr)
        if not lits:
            continue
        base = re.search(r"_V(\d+)_FMT\s*\+", expr)
        prefix = fmts.get(int(base.group(1)), "") if base else ""
        fmts[ver] = prefix + "".join(lits)
    if not fmts:
        rep.problem("blackbox", "could not find any _V<n>_FMT in blackbox_decode.py")
        return

    # ---- the version gate must accept what the firmware writes ---------------------
    fw = re.search(r"#define\s+BLACKBOX_VERSION\s+(\d+)", types)
    if not fw:
        rep.problem("blackbox", "could not find BLACKBOX_VERSION in types.h")
        return
    fw_ver = int(fw.group(1))
    if fw_ver not in fmts:
        rep.problem("blackbox",
                    f"firmware writes format v{fw_ver} but the decoder defines formats "
                    f"for {sorted(fmts)} -- logs from this build would not decode")
        return

    # Older formats must stay readable. A flight recorder whose decoder only reads the
    # newest format makes every previous flight unreadable on the next field change.
    if fw_ver > 2 and len(fmts) < 2:
        rep.problem("blackbox",
                    f"the decoder handles only v{fw_ver}; earlier logs became unreadable "
                    f"when the format was bumped", severity="warn")

    body, packed = _record_body()
    if body is None:
        rep.problem("blackbox", "could not find struct BlackBoxRecord in types.h")
        return
    if not packed:
        # The offset comparison below would catch this too, but naming the cause is
        # more use than a byte count.
        rep.problem("blackbox",
                    "BlackBoxRecord is no longer declared __attribute__((packed)) -- the "
                    "compiler may insert padding the decoder knows nothing about")
        return

    names = _record_field_names()
    codes = _expand_fmt(fmts[fw_ver])
    if len(names) != len(codes):
        which = "decoder format" if len(codes) < len(names) else "struct"
        rep.problem("blackbox",
                    f"BlackBoxRecord has {len(names)} fields but the v{fw_ver} decoder "
                    f"format has {len(codes)} -- the {which} is missing "
                    f"{abs(len(names) - len(codes))}")
        return

    # ---- ask the compiler where the fields actually are ----------------------------
    cc = shutil.which("g++") or shutil.which("clang++")
    if not cc:
        rep.problem("blackbox", "no C++ compiler available to verify the record layout",
                    severity="warn")
        return

    probe = ["#include <cstddef>", "#include <cstdio>", '#include "types.h"',
             "int main(){", '  printf("SIZE %zu\\n", sizeof(BlackBoxRecord));']
    for n in names:
        probe.append(f'  printf("OFF {n} %zu\\n", offsetof(BlackBoxRecord, {n}));')
    probe += ["  return 0;", "}"]

    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, "probe.cpp")
        exe = os.path.join(td, "probe.exe")
        with open(src, "w", encoding="utf-8") as fh:
            fh.write("\n".join(probe) + "\n")
        cmd = [cc, "-std=c++17"]
        for inc in (os.path.join(ROOT, "firmware", "flight-controller", "include"),
                    os.path.join(ROOT, "tools", "host_tests"),
                    os.path.join(ROOT, "shared")):
            cmd += ["-I", str(inc)]
        cmd += [src, "-o", exe]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            detail = " / ".join(l.strip() for l in r.stderr.strip().splitlines()
                                if "error" in l.lower())[:200]
            rep.problem("blackbox",
                        f"the record-layout probe did not compile: "
                        f"{detail or 'unknown error'}", severity="warn")
            return
        r = subprocess.run([exe], capture_output=True, text=True)
        if r.returncode != 0:
            rep.problem("blackbox", "the record-layout probe did not run", severity="warn")
            return

    c_size, c_off = None, {}
    for line in r.stdout.splitlines():
        p = line.split()
        if len(p) == 2 and p[0] == "SIZE":
            c_size = int(p[1])
        elif len(p) == 3 and p[0] == "OFF":
            c_off[p[1]] = int(p[2])

    py_size = struct.calcsize(fmts[fw_ver])
    if c_size != py_size:
        rep.problem("blackbox",
                    f"BlackBoxRecord is {c_size} bytes to the compiler but the v{fw_ver} "
                    f"decoder format is {py_size} -- logs would decode as garbage")
        return

    off = 0
    for name, code in zip(names, codes):
        if c_off.get(name) != off:
            rep.problem("blackbox",
                        f"field '{name}' is at byte {c_off.get(name)} in the C struct but "
                        f"byte {off} in the decoder format -- that field and every one "
                        f"after it would decode as the wrong value")
            return
        off += struct.calcsize("<" + code)

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"blackbox: v{fw_ver} record verified field-by-field against the "
                    f"compiler ({len(names)} fields, {c_size} bytes)")



# =====================================================================================
#  CHECK: review findings appear in both documents
# =====================================================================================
def check_findings(rep, fix):
    rep.checks_run += 1
    spec, res = read(SPEC), read(RESOLUTION)
    for n in range(1, 19):
        in_spec = re.search(rf"FINDING {n}\b|Finding {n}\b|\|\s*{n}\s*\|", spec)
        in_res = re.search(rf"^## Finding {n} ", res, re.M)
        if not in_spec:
            rep.problem("findings", f"finding {n} is not referenced in the specification")
        if not in_res:
            rep.problem("findings", f"finding {n} has no section in the resolution record")


# =====================================================================================
#  CHECK: documentation cross-references resolve
# =====================================================================================
def check_doc_refs(rep, fix):
    rep.checks_run += 1
    spec = read(SPEC)
    sections = set()
    for m in re.finditer(r"^#{2,4}\s+(\d+(?:\.\d+)?)[.\s]", spec, re.M):
        sections.add(m.group(1))
    # A top-level "## 5. Title" also implies section 5 exists for "section 5" references.
    tops = {s.split(".")[0] for s in sections}
    sections |= tops

    for f in source_files() + [ROOT / "README.md"]:
        if not f.exists():
            continue
        rel = f.relative_to(ROOT).as_posix()
        for m in re.finditer(r"docs section (\d+(?:\.\d+)?)", read(f), re.I):
            if m.group(1) not in sections:
                rep.problem("doc-refs",
                            f"{rel}: refers to 'docs section {m.group(1)}', "
                            f"which does not exist in the specification",
                            severity="warn")


# =====================================================================================
#  CHECK: identity placeholders are structurally invalid on purpose
# =====================================================================================
def check_identity_placeholders(rep, fix):
    rep.checks_run += 1
    rid = ROOT / "firmware" / "remote-id" / "src" / "main.cpp"
    if not rid.exists():
        return
    src = read(rid)

    # A committed operator ID would publish the three secret characters.
    m = re.search(r'#define\s+OPERATOR_ID\s+"([^"]*)"', src)
    if m:
        rep.problem("identity",
                    "remote-id/main.cpp has a compiled-in OPERATOR_ID. The three "
                    "characters after the hyphen are secret and must not be committed; "
                    "provision it into NVS at runtime instead.")

    # Any string that looks like a full operator ID anywhere in the tree.
    #
    # Two deliberate exemptions:
    #   * the host tests, which must exercise the split/validate paths on a full value;
    #   * EASA's own published example, which is documentation, not anybody's secret.
    EASA_EXAMPLE = "FIN87astrdge12k8"
    for f in source_files():
        rel = f.relative_to(ROOT).as_posix()
        if rel.startswith("tools/host_tests/"):
            continue
        for m in re.finditer(r'"([A-Z]{3}[A-Za-z0-9]{13})-([A-Za-z0-9]{3})"', read(f)):
            if m.group(1) == EASA_EXAMPLE:
                continue
            rep.problem("identity",
                        f"{rel}: a full operator ID including its secret suffix "
                        f"appears to be committed")

    # The shipped CTA serial must be a placeholder, not a plausible-looking invalid one.
    m = re.search(r'#define\s+UAS_SERIAL_NUMBER\s+"([^"]*)"', src)
    if m and m.group(1) not in ("SET-YOUR-CTA-SERIAL",):
        ok = validate_cta(m.group(1))
        if ok is not True:
            rep.problem("identity",
                        f"UAS_SERIAL_NUMBER '{m.group(1)}' is not a valid CTA-2063-A "
                        f"serial ({ok}) and is not the recognised placeholder either")


def validate_cta(s):
    """Mirror of odyValidateCtaSerial() in identity.cpp."""
    allowed = set("0123456789ABCDEFGHJKLMNPQRSTUVWXYZ")   # no I, no O
    if not 6 <= len(s) <= 20:
        return "wrong length"
    if any(c not in allowed for c in s[:4]):
        return "manufacturer code malformed"
    code = s[4]
    if "1" <= code <= "9":
        declared = int(code)
    elif "A" <= code <= "F":
        declared = 10 + ord(code) - ord("A")
    else:
        return "length code is not 1-9 or A-F"
    if declared != len(s) - 5:
        return "declared length does not match the serial"
    if any(c not in allowed for c in s[5:]):
        return "character outside the permitted alphabet"
    return True


# =====================================================================================
#  CHECK: .gitattributes covers what it needs to
# =====================================================================================
def check_gitattributes(rep, fix):
    rep.checks_run += 1
    if not GITATTR.exists():
        rep.problem("gitattributes", ".gitattributes is missing -- .docx would be "
                                     "corrupted by text normalisation")
        return
    attrs = read(GITATTR)
    for pattern, why in (
        ("*.docx", "generated Word documents would be corrupted by text normalisation"),
        ("*.sh", "shell scripts need LF to stay runnable on Unix"),
        ("tools/git-hooks/*", "hooks have no extension and need LF"),
    ):
        if pattern not in attrs:
            rep.problem("gitattributes", f"no rule for {pattern} -- {why}")


# =====================================================================================
#  CHECK: every sensor the arm gate requires is actually reported
# =====================================================================================
def check_arm_gate(rep, fix):
    rep.checks_run += 1
    link = read(LINK_H)
    main = read(ROOT / "firmware" / "flight-controller" / "src" / "main.cpp")

    m = re.search(r"#define ODY_ARM_REQUIRED_SENSORS\s*\(([^)]*(?:\)[^)]*)*?)\)\s*\n",
                  link, re.S)
    if not m:
        rep.problem("arm-gate", "could not parse ODY_ARM_REQUIRED_SENSORS")
        return

    required = set(re.findall(r"ODY_SENS_[A-Z_]+", m.group(1)))

    # Remote ID is opt-in via REQUIRE_REMOTE_ID_TO_ARM, so include it in the required
    # set only when the operator has actually opted in.
    cfg = read(CONFIG_H)
    opt = re.search(r"#define\s+REQUIRE_REMOTE_ID_TO_ARM\s+(\d+)", cfg)
    if opt and opt.group(1) != "0":
        required.add("ODY_SENS_REMOTE_ID")
    sensors_cpp = read(ROOT / "firmware" / "flight-controller" / "src" / "sensors.cpp")
    settable = set(re.findall(r"ODY_SENS_[A-Z_]+", sensors_cpp)) | \
               set(re.findall(r"ODY_SENS_[A-Z_]+", main))

    missing = sorted(required - settable)
    if missing:
        rep.problem("arm-gate",
                    f"the arm gate requires {', '.join(missing)} but nothing ever sets "
                    f"those bits -- arming would be permanently blocked")


# =====================================================================================
#  CHECK: the CI workflow stays cheap
#
#  This was originally a `grep` step inside the workflow itself, and it failed on its
#  first real run by matching ITS OWN PATTERN -- the line containing
#  'runs-on:.*(windows|macos)' does, unavoidably, contain that text.
#
#  Parsing the YAML structure instead of scanning the text cannot make that mistake,
#  and putting the guard here means it also runs in the pre-push hook rather than only
#  after a push has already happened.
# =====================================================================================
WORKFLOW = ROOT / ".github" / "workflows" / "host-tests.yml"


def check_workflow_cost(rep, fix):
    rep.checks_run += 1
    if not WORKFLOW.exists():
        return
    try:
        import yaml
    except ImportError:
        rep.problem("workflow-cost", "PyYAML not installed, cannot verify the workflow",
                    severity="warn")
        return

    doc = yaml.safe_load(read(WORKFLOW))

    # PyYAML parses the bare key `on:` as the boolean True.
    triggers = doc.get(True, doc.get("on", {})) or {}
    if "schedule" in triggers:
        rep.problem("workflow-cost",
                    "the workflow has a schedule trigger, which consumes runner time "
                    "even when nothing has changed")

    for job_name, job in (doc.get("jobs") or {}).items():
        runs_on = str(job.get("runs-on", "")).lower()
        if "windows" in runs_on:
            rep.problem("workflow-cost",
                        f"job '{job_name}' runs on Windows, billed at 2x the Linux rate")
        if "macos" in runs_on:
            rep.problem("workflow-cost",
                        f"job '{job_name}' runs on macOS, billed at 10x the Linux rate")
        if "strategy" in job:
            rep.problem("workflow-cost",
                        f"job '{job_name}' has a build matrix, multiplying the cost by "
                        f"the number of combinations")
        if "timeout-minutes" not in job:
            rep.problem("workflow-cost",
                        f"job '{job_name}' has no timeout-minutes, so a hung job runs "
                        f"until GitHub's 6-hour default",
                        severity="warn")


# =====================================================================================
#  CHECK: numbers asserted in PROSE match the constants they claim to quote
#
#  The spec-constants check covers values that sit in tables with a predictable shape.
#  It does not cover a sentence like "CRUISE_CURRENT_A in config.h is set to 9.7 A", or a
#  formula block that writes "I_cruise = 9.7 A" -- and both of those went stale during
#  the airframe rework, one of them for three revisions.
#
#  That mattered: the stale one hid a real defect. CRUISE_CURRENT_A was set from hover
#  power rather than cruise power, so the return-to-home energy reserve was optimistic,
#  and the prose kept quoting the old figure while the constant moved underneath it.
#
#  Each entry below is an EXPLICIT pattern rather than a generic scan. A generic scan
#  over prose produces false positives, and a check nobody trusts is a check nobody
#  runs. The cost is that coverage has to be extended by hand, so the check reports how
#  many claims it verified -- if that number stops growing while the document does, the
#  check is falling behind.
# =====================================================================================

#  (where it lives, regex with ONE capture group, config.h constant, tolerance)
PROSE_CLAIMS = [
    # 8.3.1 quotes the dynamic notch's own parameters. These are exactly the kind of
    # figure that goes stale the moment the constant behind it is retuned.
    ("8.3.1 notch bin count",
     r"Sliding DFT, ([\d]+) bins",                          "DYN_NOTCH_BINS",   0.5),
    ("8.3.1 notch update rate",
     r"\| Update rate \| ([\d.]+) Hz \|",                    "DYN_NOTCH_UPDATE_HZ", 0.5),
    ("8.3.1 notch slew rate",
     r"centre \*\*slews\*\* at ([\d.]+) Hz/s",                "DYN_NOTCH_SLEW_HZ_PER_S", 0.5),
    ("8.3.1 notch band floor",
     r"clamped\*\* to ([\d.]+)×",                            "DYN_NOTCH_BAND_LOW", 0.01),
    ("5.2 energy formula",
     r"I_cruise\s*=\s*([\d.]+)\s*A",                       "CRUISE_CURRENT_A", 0.05),
    ("3.3 cruise current",
     r"`CRUISE_CURRENT_A`[\s\S]{0,80}?set to \*{0,2}([\d.]+)\*{0,2}\s*A",
                                                            "CRUISE_CURRENT_A", 0.05),
    ("5.2 critical cutoff",
     r"V_req\s*=\s*V_critical[^\n]*\n\s*=\s*([\d.]+) V",   "PACK_CRITICAL_V",  0.01),
    ("8.3 notch centre",
     r"The notch centre is \*\*([\d.]+) Hz\*\*",            "NOTCH_CENTER_HZ",  0.01),
    ("5.9 free-fall threshold",
     r"acceleration magnitude below \*\*([\d.]+) m/s",      "FREEFALL_ACCEL_MPS2", 0.01),
    ("5.9 free-fall hold",
     r"held\s*\ncontinuously for \*\*(\d+) ms\*\*",         "FREEFALL_HOLD_MS", 0.5),
    ("5.9 parachute minimum altitude",
     r"\*\*Minimum deployment altitude, (\d+) m AGL",       "PARACHUTE_MIN_AGL_M", 0.01),
    ("5.7 touchdown ToF threshold",
     r"AGL_tof\s*<=\s*([\d.]+) m",                          "TOUCHDOWN_TOF_M",  0.001),
    ("5.7 touchdown veto altitude",
     r"AGL_any\s*>\s*([\d.]+) m",                           "TOUCHDOWN_VETO_AGL_M", 0.001),
    ("5.8 arm throttle threshold",
     r"Throttle stick at minimum \(≤ (\d+) µs\)",           "ARM_THROTTLE_MAX_US", 0.5),
    ("5.4 CRSF timeout",
     r"CRSF gap of more than (\d+) ms",                     "CRSF_TIMEOUT_MS",  0.5),
    ("6.2 obstacle brake distance",
     r"d <= ([\d.]+) m\s+hard brake",                       "OBSTACLE_STOP_CM", 0.01),
    ("6.3 flare engage altitude",
     r"Active below ([\d.]+) m AGL",                        "FLARE_ENGAGE_AGL_M", 0.01),
]

# Constants stored in different units from how the prose states them.
PROSE_SCALE = {
    "OBSTACLE_STOP_CM": 0.01,      # centimetres in code, metres in prose
}


def _config_value(defines, name):
    """Resolves a config.h constant to a float, preferring the preprocessed value."""
    resolved = resolved_defines()
    if name in resolved:
        return resolved[name]
    if name in defines:
        # Strip the C literal suffixes: 1.5f, 400u, 500UL and so on.
        raw = defines[name].strip().rstrip("fFuUlL")
        try:
            return float(raw)
        except ValueError:
            pass
    # PACK_* values are products of a per-cell constant and CELL_COUNT.
    if name.startswith("PACK_") and name.endswith("_V"):
        cell = name.replace("PACK_", "CELL_")
        if cell in defines and "CELL_COUNT" in defines:
            return float(defines[cell].rstrip("f")) * int(defines["CELL_COUNT"])
    return None


def check_prose_constants(rep, fix):
    rep.checks_run += 1
    defines = config_defines(read(CONFIG_H))
    spec = read(SPEC)
    verified = 0

    for where, pattern, const, tol in PROSE_CLAIMS:
        expected = _config_value(defines, const)
        if expected is None:
            rep.problem("prose-constants",
                        f"{where}: cannot resolve {const} from config.h", severity="warn")
            continue

        expected *= PROSE_SCALE.get(const, 1.0)

        m = re.search(pattern, spec)
        if not m:
            # A claim that has vanished is as much a drift signal as one gone stale --
            # the prose may have been reworded around it.
            rep.problem("prose-constants",
                        f"{where}: the sentence quoting {const} is no longer present. "
                        f"Either it was reworded (update the pattern) or the figure was "
                        f"dropped.", severity="warn")
            continue

        try:
            stated = float(m.group(1))
        except ValueError:
            rep.problem("prose-constants", f"{where}: could not parse '{m.group(1)}'")
            continue

        if abs(stated - expected) > tol:
            rep.problem("prose-constants",
                        f"{where}: prose says {m.group(1)} but {const} is "
                        f"{expected:g}")
        else:
            verified += 1

    if verified:
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"prose-constants: {verified}/{len(PROSE_CLAIMS)} claims verified")


# =====================================================================================
#  CHECK: README badges and revision agree with reality
#
#  Badges that carry numbers rot. A badge claiming "135 assertions" after the suite has
#  grown to 150 is worse than no badge, because it is confidently wrong. The counts are
#  therefore derived and compared, not trusted -- and --fix rewrites them.
# =====================================================================================
README = ROOT / "README.md"


def _run_suite_count():
    """Builds and runs the host suite, returning its assertion count, or None."""
    import shutil, subprocess
    if not shutil.which("g++"):
        return None
    script = ROOT / "tools" / "host_tests" / "run_tests.sh"
    if not script.exists():
        return None
    try:
        r = subprocess.run(["sh", str(script)], capture_output=True, text=True,
                           timeout=180, cwd=str(ROOT))
    except Exception:
        return None
    m = re.search(r"(\d+)\s+passed,\s+(\d+)\s+failed", r.stdout)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2))


def _run_decoder_count():
    """Builds and runs the Remote ID decoder tests, returning the assertion count."""
    import shutil, subprocess
    if not shutil.which("javac"):
        return None
    script = ROOT / "tools" / "run_android_parser_tests.sh"
    if not script.exists():
        return None
    try:
        r = subprocess.run(["sh", str(script)], capture_output=True, text=True,
                           timeout=180, cwd=str(ROOT))
    except Exception:
        return None
    m = re.search(r"(\d+)\s+passed,\s+(\d+)\s+failed", r.stdout)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2))


def check_readme(rep, fix):
    rep.checks_run += 1
    if not README.exists():
        return
    text = read(README)
    changed = False

    # --- revision must match the specification -------------------------------------
    spec = read(SPEC)
    ms = re.search(r"\*\*Document revision:\*\*\s*([\d.]+)", spec)
    mr = re.search(r"\*\*Status:\*\*\s*revision\s*([\d.]+)", text)
    if ms and mr and ms.group(1) != mr.group(1):
        if fix:
            start, end = mr.span(1)
            text = text[:start] + ms.group(1) + text[end:]
            changed = True
            rep.fix("readme", f"revision {mr.group(1)} -> {ms.group(1)}")
        else:
            rep.problem("readme",
                        f"README says revision {mr.group(1)}, the specification says "
                        f"{ms.group(1)}", fixable=True)

    # --- the same count, written out in prose ---------------------------------------
    # The badge was guarded from the start; this sentence was not, and it sat at 14
    # while the suite had grown to 16. A number in prose drifts exactly as easily as a
    # number in a badge -- it just does not look like data, so nobody checks it.
    mp = re.search(r"(\d+) checks\. `--fix` repairs", text)
    if mp:
        actual = len(CHECKS)
        if int(mp.group(1)) != actual:
            if fix:
                start, end = mp.span(1)
                text = text[:start] + str(actual) + text[end:]
                changed = True
                rep.fix("readme", f"prose check count {mp.group(1)} -> {actual}")
            else:
                rep.problem("readme",
                            f"README prose says {mp.group(1)} checks, there are "
                            f"{actual}", fixable=True)

    # --- consistency-check count ----------------------------------------------------
    m = re.search(r"consistency_checks-(\d+)-", text)
    if m:
        actual = len(CHECKS)
        if int(m.group(1)) != actual:
            if fix:
                text = text.replace(f"consistency_checks-{m.group(1)}-",
                                    f"consistency_checks-{actual}-")
                changed = True
                rep.fix("readme", f"consistency badge {m.group(1)} -> {actual}")
            else:
                rep.problem("readme",
                            f"consistency badge says {m.group(1)}, there are {actual} "
                            f"checks", fixable=True)

    # --- host assertion count -------------------------------------------------------
    m = re.search(r"host_assertions-(\d+)-", text)
    if m:
        actual = _run_suite_count()
        if actual is None:
            rep.problem("readme",
                        "cannot verify the assertion badge (no compiler, or the suite "
                        "did not run)", severity="warn")
        elif int(m.group(1)) != actual:
            if fix:
                text = text.replace(f"host_assertions-{m.group(1)}-",
                                    f"host_assertions-{actual}-")
                changed = True
                rep.fix("readme", f"assertion badge {m.group(1)} -> {actual}")
            else:
                rep.problem("readme",
                            f"assertion badge says {m.group(1)}, the suite runs {actual}",
                            fixable=True)

    # --- Remote ID decoder assertion count ------------------------------------------
    m = re.search(r"RID_decoder_tests-(\d+)-", text)
    if m:
        actual = _run_decoder_count()
        if actual is None:
            rep.problem("readme",
                        "cannot verify the decoder badge (no JDK, or the tests did not "
                        "run)", severity="warn")
        elif int(m.group(1)) != actual:
            if fix:
                text = text.replace(f"RID_decoder_tests-{m.group(1)}-",
                                    f"RID_decoder_tests-{actual}-")
                changed = True
                rep.fix("readme", f"decoder badge {m.group(1)} -> {actual}")
            else:
                rep.problem("readme",
                            f"decoder badge says {m.group(1)}, the suite runs {actual}",
                            fixable=True)

    # --- the CI badge must point at this repository's workflow ----------------------
    if "actions/workflows/host-tests.yml/badge.svg" not in text:
        rep.problem("readme", "no CI status badge", severity="warn")

    if changed:
        write(README, text)


# =====================================================================================
#  CHECK: both PROP_BLADES configurations stay coherent
#
#  The switch exists so five coupled constants move together. This verifies they
#  actually do -- that the 3-blade build is not silently identical to the 2-blade one,
#  which is what a mistyped #elif would produce, and that neither drifts into nonsense.
# =====================================================================================
#  Frame -> motors that are characterised for it. A 2807 on a 10-inch is under-stator'd
#  and a 3115 on a 7-inch is dead weight, so those pairings are not offered at all.
FRAME_MOTORS = {
    7:  ["MOTOR_2807"],
    9:  ["MOTOR_2810", "MOTOR_3110"],
    10: ["MOTOR_3110", "MOTOR_3115"],
}


def check_prop_configs(rep, fix):
    rep.checks_run += 1

    builds = {}
    for frame, motors in FRAME_MOTORS.items():
        for mc in motors:
            for pb in (2, 3):
                flags = [f"-DFRAME_SIZE_IN={frame}", f"-DMOTOR_CLASS={mc}",
                         f"-DPROP_BLADES={pb}", "-DACCEPT_CONNECTOR_OVER_RATING=1"]
                d = resolved_defines(tuple(flags))
                if not d:
                    rep.problem("build-configs",
                                "no C preprocessor available to resolve config.h",
                                severity="warn")
                    return
                builds[(frame, mc, pb)] = d

    # ---- the default must stay the characterised airframe ---------------------------
    default = resolved_defines()
    if default.get("FRAME_SIZE_IN") != 9:
        rep.problem("build-configs",
                    f"the default frame is not 9-inch "
                    f"(FRAME_SIZE_IN={default.get('FRAME_SIZE_IN')})")
    if default.get("PROP_BLADES") != 2:
        rep.problem("build-configs", "the default build is not 2-blade")
    if abs(default.get("AIRFRAME_AUW_G", 0) - 1584.0) > 0.5:
        rep.problem("build-configs",
                    f"the default AUW is {default.get('AIRFRAME_AUW_G')} g, not the "
                    f"1584 g the specification is written around")

    # ---- every build must be internally sane ----------------------------------------
    over_connector = []
    for (frame, mc, pb), d in builds.items():
        label = f'{frame}" {mc[6:]} {pb}b'
        auw, thrust = d.get("AIRFRAME_AUW_G"), d.get("MOTOR_MAX_THRUST_G")
        if not auw or not thrust:
            rep.problem("build-configs", f"{label}: AUW or thrust missing")
            continue

        twr = 4 * thrust / auw
        if not (2.0 < twr < 12.0):
            rep.problem("build-configs", f"{label}: thrust-to-weight {twr:.2f}:1 is "
                                         f"outside any sane band")

        # The assertion that excluded the 5-inch: a notch above Nyquist chases an alias.
        notch, loop = d.get("PROP_NOTCH_DEFAULT_HZ", 0), d.get("FLIGHT_LOOP_HZ", 0)
        if notch >= loop / 2:
            rep.problem("build-configs",
                        f"{label}: notch {notch:.0f} Hz is at or above Nyquist for a "
                        f"{loop:.0f} Hz loop")

        # The DLPF has to pass the peak the notch is aimed at.
        dlpf = d.get("IMU_DLPF_HZ", 0)
        if dlpf < notch * 1.3:
            rep.problem("build-configs",
                        f"{label}: IMU DLPF {dlpf:.0f} Hz is not 30% clear of the "
                        f"{notch:.0f} Hz notch -- it would attenuate that peak itself "
                        f"and leave its phase lag in the control band")

        # The dynamic notch searches a band around the static centre. If that band does
        # not straddle the centre, or runs past the DLPF corner, the tracker is either
        # unable to reach the real peak or hunting in the filter's own roll-off. This is
        # the same coupled-constant trap that produced the DLPF defect, one level up.
        lo = notch * d.get("DYN_NOTCH_BAND_LOW", 0)
        hi = notch * d.get("DYN_NOTCH_BAND_HIGH", 0)
        if not (lo < notch < hi):
            rep.problem("build-configs",
                        f"{label}: the dynamic notch band {lo:.0f}-{hi:.0f} Hz does not "
                        f"straddle the {notch:.0f} Hz static centre")
        if min(hi, dlpf) <= lo:
            rep.problem("build-configs",
                        f"{label}: the dynamic notch band collapses once capped at the "
                        f"{dlpf:.0f} Hz DLPF corner -- nothing left to search")
        if min(hi, dlpf) >= loop / 2:
            rep.problem("build-configs",
                        f"{label}: the dynamic notch could track up to "
                        f"{min(hi, dlpf):.0f} Hz, at or above Nyquist for a "
                        f"{loop:.0f} Hz loop")

        peak, packA = d.get("PROP_PEAK_PACK_A", 0), d.get("PACK_MAX_DISCHARGE_A", 0)
        if peak > packA:
            rep.problem("build-configs",
                        f"{label}: peak {peak:.0f} A exceeds the {packA:.0f} A pack")
        if peak > d.get("CONNECTOR_RATING_A", 90):
            over_connector.append(f"{label} ({peak:.0f} A)")

        cA = d.get("CRUISE_CURRENT_A", 0)
        if not (4.0 < cA < 30.0):
            rep.problem("build-configs", f"{label}: cruise current {cA:.1f} A is "
                                         f"implausible")

    if over_connector:
        rep.problem("build-configs",
                    "these exceed the XT90-S 90 A continuous rating on a full-throttle "
                    "burst and need -DACCEPT_CONNECTOR_OVER_RATING=1 or a bigger "
                    "connector: " + ", ".join(sorted(over_connector)),
                    severity="warn")

    # ---- the switches must actually switch -------------------------------------------
    def differs(axis, name, pairs):
        for a, b in pairs:
            if abs(builds[a].get(name, 0) - builds[b].get(name, 0)) < 1e-9:
                rep.problem("build-configs",
                            f"{name} is identical across {axis} -- that switch is not "
                            f"driving it")

    def direction(name, pairs, expect, why):
        for a, b in pairs:
            va, vb = builds[a].get(name), builds[b].get(name)
            if va is None or vb is None:
                continue
            ok = (vb > va) if expect == "greater" else (vb < va)
            if not ok:
                rep.problem("build-configs",
                            f"{name}: {b[0]}\"/{b[1][6:]}/{b[2]}b should be {expect} "
                            f"than {a[0]}\"/{a[1][6:]}/{a[2]}b ({why}), got "
                            f"{va:g} and {vb:g}")

    # Blade axis, within each frame and motor.
    blade_pairs = [((f, m, 2), (f, m, 3))
                   for f, ms in FRAME_MOTORS.items() for m in ms]
    for name in ("PROP_MASS_G_EACH", "PROP_BASE_THRUST_G", "PROP_NOTCH_DEFAULT_HZ",
                 "PROP_POWER_LOADING_GW", "AIRFRAME_AUW_G", "MOTOR_MAX_THRUST_G",
                 "CRUISE_CURRENT_A"):
        differs("PROP_BLADES", name, blade_pairs)
    direction("PROP_MASS_G_EACH",      blade_pairs, "greater", "an extra blade")
    direction("MOTOR_MAX_THRUST_G",    blade_pairs, "greater", "more blade area")
    direction("PROP_NOTCH_DEFAULT_HZ", blade_pairs, "less",
              "three blades spin slower for the same thrust")
    direction("PROP_POWER_LOADING_GW", blade_pairs, "less", "three blades are less efficient")

    # Motor axis, where a frame offers two.
    motor_pairs = [((f, ms[0], b), (f, ms[1], b))
                   for f, ms in FRAME_MOTORS.items() if len(ms) == 2 for b in (2, 3)]
    for name in ("MOTOR_MASS_G_EACH", "MOTOR_THRUST_FACTOR", "AIRFRAME_AUW_G",
                 "MOTOR_MAX_THRUST_G"):
        differs("MOTOR_CLASS", name, motor_pairs)
    direction("MOTOR_MASS_G_EACH",  motor_pairs, "greater", "a larger stator")
    direction("MOTOR_MAX_THRUST_G", motor_pairs, "greater",
              "more stator holds RPM better under load")

    # A larger stator must not be modelled as a big thrust win -- these propellers are
    # prop-limited, and a large gain would mean the revision 2.3 error had returned.
    for a, b in motor_pairs:
        va, vb = builds[a]["MOTOR_MAX_THRUST_G"], builds[b]["MOTOR_MAX_THRUST_G"]
        if vb / va > 1.10:
            rep.problem("build-configs",
                        f"the larger stator is modelled at {100*(vb/va-1):.0f}% more "
                        f"thrust. These propellers are prop-limited; a gain this large "
                        f"means the stator model is wrong.")

    # Frame axis: bigger frame, bigger everything except notch frequency.
    frame_pairs = [((7, "MOTOR_2807", b), (9, "MOTOR_2810", b)) for b in (2, 3)] + \
                  [((9, "MOTOR_3110", b), (10, "MOTOR_3110", b)) for b in (2, 3)]
    direction("AIRFRAME_AUW_G",        frame_pairs, "greater", "a bigger airframe")
    direction("PROP_MASS_G_EACH",      frame_pairs, "greater", "a bigger propeller")
    direction("PROP_NOTCH_DEFAULT_HZ", frame_pairs, "less",
              "a bigger disc turns slower for the same thrust")
    direction("FRAME_GAIN_SCALE",      frame_pairs, "less",
              "a bigger airframe has more rotational inertia")

    # ---- the 1000 Hz option, where it exists ----------------------------------------
    #
    # FLIGHT_LOOP_HZ is a switch now, and a faster loop is only worth having if the
    # constants that must move with it actually did. Checking the default build alone
    # would miss exactly the coupled-constant drift these switches exist to prevent.
    fast = {}
    for frame, motors in FRAME_MOTORS.items():
        if frame == 7:
            continue          # already 1000 Hz by default
        for mc in motors:
            for pb in (2, 3):
                d = resolved_defines((f"-DFRAME_SIZE_IN={frame}", f"-DMOTOR_CLASS={mc}",
                                      f"-DPROP_BLADES={pb}", "-DFLIGHT_LOOP_HZ=1000",
                                      "-DACCEPT_CONNECTOR_OVER_RATING=1"))
                if d:
                    fast[(frame, mc, pb)] = d

    gained = 0
    for key, d in sorted(fast.items()):
        frame, mc, pb = key
        label = f'{frame}"/{mc[6:]}/{pb}b @1kHz'
        slow = builds.get(key, {})

        if d.get("FLIGHT_LOOP_HZ") != 1000:
            rep.problem("build-configs", f"{label}: the loop override did not take")
            continue
        # The anti-alias corner is the whole reason to go faster. If it stayed put, the
        # extra loop rate bought nothing but CPU.
        if d.get("IMU_DLPF_HZ", 0) <= slow.get("IMU_DLPF_HZ", 0):
            rep.problem("build-configs",
                        f"{label}: the loop doubled but the anti-alias corner did not "
                        f"move, so the harmonic is still hidden and the change is pure "
                        f"cost")
        # Resolution is loop/bins, so the bin count must follow or it halves.
        res_slow = slow.get("FLIGHT_LOOP_HZ", 1) / max(slow.get("DYN_NOTCH_BINS", 1), 1)
        res_fast = d["FLIGHT_LOOP_HZ"] / max(d.get("DYN_NOTCH_BINS", 1), 1)
        if res_fast > res_slow * 1.01:
            rep.problem("build-configs",
                        f"{label}: SDFT resolution got WORSE ({res_slow:.1f} -> "
                        f"{res_fast:.1f} Hz) -- DYN_NOTCH_BINS did not follow the loop")
        if d.get("IMU_DLPF_HZ", 0) >= d["FLIGHT_LOOP_HZ"] / 2:
            rep.problem("build-configs",
                        f"{label}: the anti-alias corner is at or above Nyquist")

        f0 = d.get("PROP_NOTCH_DEFAULT_HZ", 0)
        ceil_fast = min(d.get("IMU_DLPF_HZ", 0),
                        d["FLIGHT_LOOP_HZ"] * 0.5 * d.get("DYN_NOTCH_H2_NYQUIST_FRAC", 0.8))
        ceil_slow = min(slow.get("IMU_DLPF_HZ", 0),
                        slow.get("FLIGHT_LOOP_HZ", 1) * 0.5
                        * slow.get("DYN_NOTCH_H2_NYQUIST_FRAC", 0.8))
        if 2 * f0 <= ceil_fast and 2 * f0 > ceil_slow:
            gained += 1

    if fast:
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"loop-rate: {len(fast)} builds also verified at 1000 Hz; "
                        f"{gained} of them gain an observable second harmonic that the "
                        f"500 Hz build cannot see")

    # ---- can each build see its own second harmonic? --------------------------------
    #
    # Reported as a note rather than a problem: an unobservable harmonic is a property
    # of the MPU-6050's anti-alias corner, not a defect. But it must be VISIBLE, because
    # the natural assumption is that a harmonic notch works everywhere, and on this
    # aircraft it mostly does not. Silence here would let that assumption stand.
    h2_yes, h2_no = [], []
    for (frame, mc, pb), d in sorted(builds.items()):
        f0 = d.get("PROP_NOTCH_DEFAULT_HZ", 0)
        dlpf = d.get("IMU_DLPF_HZ", 0)
        loop = d.get("FLIGHT_LOOP_HZ", 0)
        mult = d.get("DYN_NOTCH_H2_MULTIPLE", 2.0)
        frac = d.get("DYN_NOTCH_H2_NYQUIST_FRAC", 0.8)
        if not (f0 and dlpf and loop):
            continue
        h2 = f0 * mult
        ceiling = min(dlpf, loop * 0.5 * frac)
        label = f'{frame}"/{mc[6:]}/{pb}b'
        if h2 <= ceiling:
            h2_yes.append(f"{label} @{h2:.0f}Hz")
        else:
            why = "DLPF" if h2 > dlpf else "Nyquist"
            h2_no.append(f"{label} ({h2:.0f}>{ceiling:.0f}Hz {why})")

        # A harmonic notch must never be placed above the ceiling, in any build.
        if h2 <= ceiling and h2 >= loop * 0.5:
            rep.problem("build-configs",
                        f"{label}: the harmonic at {h2:.0f} Hz is at or above Nyquist "
                        f"for a {loop:.0f} Hz loop")

    rep.note = getattr(rep, "note", [])
    if h2_yes:
        rep.note.append(f"harmonic: observable in {len(h2_yes)}/{len(builds)} builds -- "
                        + ", ".join(h2_yes))
    if h2_no:
        rep.note.append(f"harmonic: NOT observable in {len(h2_no)}/{len(builds)} builds, "
                        f"2*f0 is above the IMU anti-alias corner -- "
                        + ", ".join(h2_no))

    for (frame, mc, pb), d in builds.items():
        loop = d.get("FLIGHT_LOOP_HZ", 0)
        bins = d.get("DYN_NOTCH_BINS", 0)
        if bins and loop:
            resolution = loop / bins
            if resolution > 12.0:
                rep.problem("build-configs",
                            f'{frame}" {mc[6:]} {pb}b: {bins} bins at {loop:.0f} Hz '
                            f"gives {resolution:.1f} Hz notch resolution, too coarse to "
                            f"be worth tracking")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"build-configs: {len(builds)} frame x motor x propeller "
                    f"combinations coherent, distinct and correctly ordered")


def check_notch_observability(rep, fix):
    """
    Guards the finding in section 8.3: the motor peak is NOT observable in the BlackBox
    gyro trace, so the documentation must not tell anyone to look for it there.

    This exists because the instruction survived every revision from 1.0 to 2.9. It
    reads like standard practice -- it IS standard practice on a controller that logs
    raw gyro at 2-8 kHz -- and nothing compared the log rate against the notch frequency.
    A prose check is the only thing that catches a plausible-sounding sentence.
    """
    rep.checks_run += 1
    spec = read(SPEC)

    d = resolved_defines()
    if not d:
        rep.problem("notch-observability",
                    "no C preprocessor available to resolve the log rate", severity="warn")
        return

    log_hz = d.get("BLACKBOX_LOG_HZ", 0)
    nyquist = log_hz / 2.0
    notch = d.get("PROP_NOTCH_DEFAULT_HZ", 0)
    if not log_hz or not notch:
        rep.problem("notch-observability", "could not resolve the log rate or the notch")
        return

    observable = notch < nyquist

    # The prose must not promise a measurement the sample rate cannot deliver.
    bad_phrases = [
        r"[Tt]ake a BlackBox gyro trace[^.]*find the actual peak",
        r"find the (?:actual )?peak[^.]*(?:in|from) the (?:BlackBox|gyro) (?:log|trace)",
        r"FFT[^.]*(?:BlackBox|flight) log[^.]*peak",
    ]
    for pat in bad_phrases:
        m = re.search(pat, spec)
        if m and not observable:
            rep.problem("notch-observability",
                        f"the specification says \"{m.group(0)[:70]}...\" but the "
                        f"{log_hz:.0f} Hz log cannot show a {notch:.0f} Hz peak "
                        f"(Nyquist {nyquist:.0f} Hz) -- it aliases to "
                        f"{abs(((notch + nyquist) % log_hz) - nyquist):.0f} Hz. "
                        f"This is the defect recorded in section 8.3.")

    # If it is not observable, the tracker's verdict is the only route to the number,
    # so the record must actually carry it.
    if not observable:
        types = read(TYPES_H)
        for field in ("notchCentreDeciHz", "notchFlags"):
            if field not in types:
                rep.problem("notch-observability",
                            f"the motor peak is not observable in the log, so the "
                            f"tracker's verdict is the only way to recover it -- but "
                            f"BlackBoxRecord has no '{field}'")

        if "aliases" not in spec and "alias" not in spec:
            rep.problem("notch-observability",
                        "the specification does not explain that the motor peak aliases "
                        "in the flight log. Without that, the impossible measurement "
                        "procedure reads as reasonable and comes back.", severity="warn")

    # The logged gyro is post-notch. If that ever changes the finding's premise changes
    # with it, so say so rather than letting the documentation drift out from under it.
    main = read(ROOT / "firmware" / "flight-controller" / "src" / "main.cpp")
    m = re.search(r"rec\.gyroX\s*=\s*\(int16_t\)\(\s*(\w+)", main)
    if m and m.group(1).endswith("Raw"):
        rep.problem("notch-observability",
                    "the BlackBox now logs the RAW gyro. That changes the premise of the "
                    "finding in section 8.3 -- the trace is no longer post-notch, though "
                    f"the {log_hz:.0f} Hz sample rate still cannot show the peak. Update "
                    f"the finding.", severity="warn")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"notch-observability: {notch:.0f} Hz peak vs a {log_hz:.0f} Hz log "
                    f"(Nyquist {nyquist:.0f} Hz) -- not observable, verdict logged instead")


def check_line_endings(rep, fix):
    """
    Every tracked text file's line endings must match what .gitattributes declares.

    Shell scripts and git hooks are `eol=lf` for a concrete reason: /bin/sh rejects a
    script whose shebang line ends in CR with "bad interpreter", which is a confusing
    way to discover a line-ending problem. Everything else is `text=auto`, checked out
    native, and must be internally consistent -- a file that has become half CRLF and
    half LF produces a whole-file diff on the next edit, burying the real change.

    This also catches a file mangled by a tool that guessed the ending wrongly, which
    is why tools/patchfile.py exists.
    """
    rep.checks_run += 1
    import subprocess

    try:
        rows = subprocess.run(["git", "ls-files", "--eol"], cwd=str(ROOT),
                              capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        rep.problem("line-endings", "git is not available to list tracked files",
                    severity="warn")
        return
    if rows.returncode != 0:
        rep.problem("line-endings", "git ls-files --eol failed", severity="warn")
        return

    checked = 0
    for row in rows.stdout.splitlines():
        parts = row.split("\t")
        if len(parts) != 2:
            continue
        meta, rel = parts[0], parts[1].strip()
        if "w/-text" in meta or "w/none" in meta:
            continue            # binary, or empty
        path = ROOT / rel
        if not path.exists():
            continue

        raw = path.read_bytes()
        crlf = raw.count(b"\r\n")
        lf = raw.count(b"\n") - crlf
        lone_cr = raw.count(b"\r") - crlf
        checked += 1

        if crlf and lf:
            rep.problem("line-endings",
                        f"{rel} has MIXED line endings ({crlf} CRLF, {lf} LF). The next "
                        f"edit will rewrite whichever kind loses, producing a whole-file "
                        f"diff that buries the real change.")
        if lone_cr:
            rep.problem("line-endings",
                        f"{rel} contains {lone_cr} bare CR characters")

        # Any other C0 control character is either a mangled escape sequence or a
        # corrupted file. Both are INVISIBLE in an editor, which is why they need
        # catching mechanically: a stray 0x08 in check_consistency.py once passed
        # review, passed a diff, and only surfaced as an unrelated regex failing.
        for offset, byte in enumerate(raw):
            if byte < 0x20 and byte not in (0x09, 0x0A, 0x0D) or byte == 0x7F:
                name = {0x00: "NUL", 0x07: "BEL", 0x08: "BACKSPACE", 0x0B: "VT",
                        0x0C: "FORM FEED", 0x1B: "ESC",
                        0x7F: "DEL"}.get(byte, f"0x{byte:02X}")
                line = raw[:offset].count(b"\n") + 1
                rep.problem("line-endings",
                            f"{rel} line {line} contains a {name} control character. "
                            f"It is invisible in an editor and is almost always a "
                            f"backslash escape mangled on its way through a shell -- "
                            f"see the note at the top of tools/patchfile.py")
                break     # one report per file is enough to act on

        if "eol=lf" in meta and crlf:
            rep.problem("line-endings",
                        f"{rel} is declared eol=lf in .gitattributes but has {crlf} CRLF "
                        f"line endings on disk. If this is a shell script or a git hook, "
                        f"/bin/sh will reject it with 'bad interpreter'.")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"line-endings: {checked} tracked text files consistent with "
                    f".gitattributes")


def check_hook_coverage(rep, fix):
    """
    Every test suite the pre-push hook runs behind its change gate must also appear in
    the hook's WATCHED list.

    The hook skips the expensive suites unless a WATCHED path changed. A suite that is
    run but not watched therefore skips itself on exactly the change most likely to
    break it -- a change to the suite, or to the module it covers.

    This was not hypothetical: tools/patchfile.py and its tests were added to the hook
    and to WATCHED in the same edit, a git checkout during unrelated testing reverted
    both, and the next push ran only the consistency check without anything noticing.

    Only the GATED section is examined. check_consistency.py runs unconditionally above
    the gate, by design, so it neither needs nor gets a WATCHED entry.
    """
    rep.checks_run += 1
    hook = ROOT / "tools" / "git-hooks" / "pre-push"
    if not hook.exists():
        rep.problem("hook-coverage", "tools/git-hooks/pre-push is missing", severity="warn")
        return

    text = read(hook)
    m = re.search(r'WATCHED="([^"]*)"', text)
    if not m:
        rep.problem("hook-coverage", "could not find WATCHED in the pre-push hook")
        return
    watched = m.group(1).split()

    # Everything after the gate is conditional on a WATCHED path having changed.
    gate = re.search(r'if \[ "\$should_run" = no \]; then\s*\n\s*exit 0\s*\n\s*fi', text)
    if not gate:
        rep.problem("hook-coverage",
                    "could not find the hook's change gate -- if it was removed, every "
                    "push now runs the full suite", severity="warn")
        return
    gated = text[gate.end():]

    def covered(path):
        return any(path == w or path.startswith(w.rstrip("/") + "/") for w in watched)

    run = set(f"tools/{n}.py" for n in re.findall(r'tools/([\w.]+)\.py', gated)
              if "$" not in n)
    # `for t in a b c; do ... tools/$t.py ...` runs each name in the list.
    for names in re.findall(r"for t in ([\w \t]+); do", gated):
        run.update(f"tools/{n}.py" for n in names.split())

    missing = sorted(p for p in run if not covered(p))
    for p in missing:
        rep.problem("hook-coverage",
                    f"the pre-push hook runs {p} behind its change gate but does not "
                    f"watch it, so a change to that file would push untested")

    if not missing:
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"hook-coverage: all {len(run)} gated suites are in WATCHED")


def check_unverified_defaults(rep, fix):
    """
    Anything that has never run on hardware must be OFF by default.

    dshot_rmt.cpp drives motors and has not been compiled, let alone flown. A frame that
    misses a bit boundary does not fail politely -- it is a throttle value the ESC acts
    on. The difference between that being opt-in and being the default is the difference
    between a considered experiment and an accident, and the default is exactly the kind
    of thing that gets flipped during debugging and never flipped back.

    This check exists so flipping it is a deliberate, reviewable change rather than a
    line nobody notices.
    """
    rep.checks_run += 1
    d = resolved_defines()
    if not d:
        rep.problem("unverified-defaults", "no C preprocessor available", severity="warn")
        return

    UNVERIFIED = {
        "DSHOT_ENABLE": (
            "drives the motors through an RMT path that has never been compiled or "
            "scoped. Section 4.3.1 has the bring-up procedure; until it has been "
            "followed this must stay opt-in."),
    }

    for name, why in UNVERIFIED.items():
        value = d.get(name)
        if value is None:
            rep.problem("unverified-defaults",
                        f"{name} is no longer defined -- if the feature was removed, "
                        f"remove it from this check too")
        elif value != 0:
            rep.problem("unverified-defaults",
                        f"{name} defaults to {value:g}, but it {why}")

    # The safety argument only holds if the guarded code really is inert when off.
    src = ROOT / "firmware" / "flight-controller" / "src" / "dshot_rmt.cpp"
    if src.exists():
        text = read(src)
        # Match the DIRECTIVE at the start of a line, not the string. The file's own
        # header comment quotes "#if DSHOT_ENABLE" while explaining the guard, and
        # splitting on the bare string cut the search region off above every include --
        # so this check passed while an unguarded include sat right there.
        m = re.search(r"^#if DSHOT_ENABLE\s*$", text, re.M)
        if not m:
            rep.problem("unverified-defaults",
                        "dshot_rmt.cpp is not guarded by #if DSHOT_ENABLE, so disabling "
                        "the feature no longer removes the code")
        else:
            before = text[:m.start()]
            for line in before.splitlines():
                s = line.split("//")[0].strip()
                if s.startswith("#include") and "config.h" not in s:
                    rep.problem("unverified-defaults",
                                f"dshot_rmt.cpp includes {s} OUTSIDE its guard, so a "
                                f"build with DSHOT_ENABLE=0 still needs the ESP-IDF RMT "
                                f"headers")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"unverified-defaults: {len(UNVERIFIED)} unproven feature(s) "
                    f"confirmed off by default")


#  Constants the documentation, the comments or an assertion present as build switches.
#  Each must be #ifndef-guarded, or a -D override is silently discarded -- or, with
#  -Werror, breaks the build outright.
BUILD_SWITCHES = [
    "FRAME_SIZE_IN", "MOTOR_CLASS", "PROP_BLADES", "FLIGHT_LOOP_HZ",
    "NOTCH_CENTER_HZ", "AIRFRAME_AUW_G", "CRUISE_CURRENT_A",
    "IMU_DLPF_HZ", "IMU_ACCEL_READ_HZ",
    "DYN_NOTCH_ENABLE", "DYN_NOTCH_BINS", "DYN_NOTCH_HARMONIC",
    "DSHOT_ENABLE", "DSHOT_BIDIRECTIONAL", "DSHOT_BITRATE_KHZ",
    "DSHOT_TELEM_TIMEOUT_US", "DSHOT_RMT_RESOLUTION_HZ",
    "MOTOR_POLE_PAIRS", "REQUIRE_REMOTE_ID_TO_ARM",
]


def check_build_switches(rep, fix):
    """
    A constant that is documented as a build switch must actually be one.

    An unguarded `#define X 300` looks configurable and is not: a -D override is either
    silently discarded, or -- because CI builds with -Werror -- fails the build with a
    redefinition error that reads like a toolchain problem rather than a missing #ifndef.

    This is the third time this project has hit it. NOTCH_CENTER_HZ was unguarded while
    section 8.3 told the reader to measure and override it, and DSHOT_BITRATE_KHZ was
    unguarded while an assertion right beneath it enumerated the four legal alternatives.
    Both read as configurable. Neither was.
    """
    rep.checks_run += 1
    text = read(CONFIG_H)
    rmt = ROOT / "firmware" / "flight-controller" / "include" / "dshot_rmt.h"
    if rmt.exists():
        text += "\n" + read(rmt)

    missing = []
    for name in BUILD_SWITCHES:
        if not re.search(rf"^\s*#\s*define\s+{name}\b", text, re.M):
            rep.problem("build-switches",
                        f"{name} is listed as a build switch but is not defined anywhere",
                        severity="warn")
            continue
        if not re.search(rf"^\s*#\s*ifndef\s+{name}\s*$", text, re.M):
            missing.append(name)

    for name in missing:
        rep.problem("build-switches",
                    f"{name} is presented as a build switch but is defined without an "
                    f"#ifndef guard, so -D{name}=... is discarded or -- under CI's "
                    f"-Werror -- breaks the build with a redefinition error")

    if not missing:
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"build-switches: all {len(BUILD_SWITCHES)} documented switches "
                        f"are genuinely overridable")


def check_repo_layout(rep, fix):
    """
    Section 10.1's repository layout must name every source file that exists.

    It had drifted badly: dynamic_notch.h, all three DShot files, the entire android/
    tree, six of the eight tools and the CI workflow were all missing, and it still
    quoted 98 host assertions against 387. A reader using it to find their way around
    would have concluded half the system did not exist.

    Nothing compares a prose file listing against the filesystem unless something is
    written to do it, which is the same reason every other check here exists.

    Directories are accepted in place of the files beneath them -- the listing is meant
    to orient a reader, not to mirror `git ls-files`. What it may NOT do is omit a
    subsystem entirely.
    """
    rep.checks_run += 1
    import subprocess

    spec = read(SPEC)
    if "Odyssey-10-Pro-Drone-System/" not in spec:
        rep.problem("repo-layout", "section 10.1's layout block is missing")
        return
    block = spec.split("Odyssey-10-Pro-Drone-System/", 1)[1].split("```", 1)[0]

    try:
        out = subprocess.run(["git", "ls-files"], cwd=str(ROOT),
                             capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        rep.problem("repo-layout", "git is not available", severity="warn")
        return
    if out.returncode != 0:
        rep.problem("repo-layout", "git ls-files failed", severity="warn")
        return

    SOURCE_SUFFIXES = (".h", ".cpp", ".py", ".sh", ".yml")

    #  Subsystems the listing deliberately summarises rather than enumerating. The
    #  PREFIX itself must still appear, so a whole subsystem cannot vanish -- but the
    #  files beneath it need not be named one by one.
    #
    #  This list is short and explicit on purpose. The first version of this check
    #  excused any file whose parent directory appeared anywhere in the listing, which
    #  meant "include/" being present excused every header inside it -- and the check
    #  passed while dynamic_notch.h and patchfile.py were both missing.
    SUMMARISED = ("android/", "firmware/beacon-node/", "firmware/ground-station/",
                  "firmware/remote-id/")

    tracked = [r.strip() for r in out.stdout.split() if r.strip()]

    #  Summarised subsystems are tested HERE, against every tracked file rather than
    #  only those with a source suffix. android/ holds Java, which this check does not
    #  scan -- so folding the subsystem test into the suffix-filtered loop below meant
    #  the whole android/ tree could vanish from the listing and nothing would notice.
    for prefix in SUMMARISED:
        if not any(r.startswith(prefix) for r in tracked):
            continue
        leaf = prefix.rstrip("/").rsplit("/", 1)[-1]
        if not re.search(r"\+--\s+" + re.escape(leaf) + r"/", block):
            rep.problem("repo-layout",
                        f"the {prefix} subsystem exists but is missing from section "
                        f"10.1's layout entirely")

    missing = []
    for rel in tracked:
        if not rel.endswith(SOURCE_SUFFIXES):
            continue
        if any(rel.startswith(p) for p in SUMMARISED):
            continue        # covered by the subsystem test above

        # Bounded, not substring. "patchfile.py" is a substring of "test_patchfile.py",
        # so a plain `in` check reported the listing as complete while patchfile.py was
        # missing from it -- the exact failure this check exists to catch.
        name = rel.rsplit("/", 1)[-1]
        if not re.search(rf"(?<![\w.]){re.escape(name)}(?![\w])", block):
            missing.append(rel)

    for rel in sorted(missing):
        rep.problem("repo-layout",
                    f"{rel} exists but section 10.1's layout does not mention it or any "
                    f"directory containing it")

    # The counts quoted inside the listing drift exactly as readily as the files do.
    m = re.search(r"check_consistency\.py[^\n]*?\[(\d+) checks\]", block)
    if m and int(m.group(1)) != len(CHECKS):
        if fix:
            spec = spec.replace(f"[{m.group(1)} checks]", f"[{len(CHECKS)} checks]", 1)
            write(SPEC, spec)
            rep.fix("repo-layout", f"layout check count {m.group(1)} -> {len(CHECKS)}")
        else:
            rep.problem("repo-layout",
                        f"section 10.1 says {m.group(1)} consistency checks, there are "
                        f"{len(CHECKS)}", fixable=True)

    if not missing:
        rep.note = getattr(rep, "note", [])
        rep.note.append("repo-layout: section 10.1 accounts for every source file")


BOM_VARIANTS = ROOT / "hardware" / "bom-variants.csv"

#  Mass that is bought but does not fly. The BOM buys two SX1278 radios and only one
#  leaves the ground, so a naive sum of "Mass g" overstates the airframe by 6 g. The
#  distinction used to live in a prose note inside a CSV cell, where nothing could
#  check it; it is now the "Airborne Mass g" column, and this reconciles it.
BOM_SHARED_AVIONICS_G = 308.0


def check_bom_mass(rep, fix):
    """
    The parts list and the firmware's mass model must agree, for every build.

    check_bom() has always verified BOM arithmetic and the price quoted in the
    specification. Nothing verified MASS against config.h -- so AIRFRAME_AUW_G, which
    sizes thrust-to-weight, hover power and the whole energy budget, was free to drift
    away from the parts that produce it.

    And until now only one of the ten builds had a parts list at all.
    """
    rep.checks_run += 1
    if not BOM.exists():
        return

    import csv, io

    # ---- the reference build ---------------------------------------------------------
    rows = list(csv.DictReader(io.StringIO(read(BOM))))
    if "Airborne Mass g" not in (rows[0].keys() if rows else {}):
        rep.problem("bom-mass",
                    "hardware/bom.csv has no 'Airborne Mass g' column, so mass that is "
                    "bought but does not fly cannot be told apart from mass that does")
        return

    by_cat = {}
    for r in rows:
        by_cat[r["Category"]] = by_cat.get(r["Category"], 0.0) + float(r["Airborne Mass g"])

    d = resolved_defines()
    if not d:
        rep.problem("bom-mass", "no C preprocessor available", severity="warn")
        return

    shared = sum(v for k, v in by_cat.items()
                 if k not in ("Airframe", "Propulsion", "Propellers", "Main Battery"))
    if abs(shared - BOM_SHARED_AVIONICS_G) > 1.0:
        rep.problem("bom-mass",
                    f"the shared avionics mass is {shared:.0f} g, but the variant BOM is "
                    f"written around {BOM_SHARED_AVIONICS_G:.0f} g -- every frame's dry "
                    f"mass is that figure plus its own airframe")

    checks = [
        ("frame + avionics", by_cat.get("Airframe", 0) + shared, d["FRAME_BASE_DRY_G"]),
        ("motors",           by_cat.get("Propulsion", 0), 4 * d["MOTOR_MASS_G_EACH"]),
        ("propellers",       by_cat.get("Propellers", 0), 4 * d["PROP_MASS_G_EACH"]),
        ("battery",          by_cat.get("Main Battery", 0), d["BATTERY_MASS_G"]),
    ]
    for label, bom_g, cfg_g in checks:
        if abs(bom_g - cfg_g) > 1.0:
            rep.problem("bom-mass",
                        f"reference build {label}: bom.csv says {bom_g:.0f} g, config.h "
                        f"says {cfg_g:.0f} g")

    total = sum(by_cat.values()) + d["PAYLOAD_RESERVE_G"]
    if abs(total - d["AIRFRAME_AUW_G"]) > 1.5:
        rep.problem("bom-mass",
                    f"reference build AUW: the parts plus the {d['PAYLOAD_RESERVE_G']:.0f} g "
                    f"payload reserve come to {total:.0f} g, config.h says "
                    f"{d['AIRFRAME_AUW_G']:.0f} g")

    # ---- the other nine --------------------------------------------------------------
    if not BOM_VARIANTS.exists():
        rep.problem("bom-variants",
                    "hardware/bom-variants.csv is missing -- nine of the ten build "
                    "combinations would have no parts list")
        return

    vrows = list(csv.DictReader(io.StringIO(read(BOM_VARIANTS))))

    def pick(frame, motor, blades, category):
        """The variant row for one build and category, matching 'any' wildcards."""
        for r in vrows:
            if r["Category"] != category:
                continue
            if r["Frame in"] not in (str(frame), "any"):
                continue
            if r["Motor"] not in (motor, "any"):
                continue
            if r["Blades"] not in (str(blades), "any"):
                continue
            return r
        return None

    covered = 0
    for frame, motors in FRAME_MOTORS.items():
        for mc in motors:
            for pb in (2, 3):
                motor = mc[6:]
                label = f'{frame}"/{motor}/{pb}b'
                cfg = resolved_defines((f"-DFRAME_SIZE_IN={frame}",
                                        f"-DMOTOR_CLASS={mc}", f"-DPROP_BLADES={pb}",
                                        "-DACCEPT_CONNECTOR_OVER_RATING=1"))
                if not cfg:
                    continue

                parts = {c: pick(frame, motor, pb, c)
                         for c in ("Airframe", "Propulsion", "Propellers",
                                   "Main Battery", "Drive")}
                missing = [c for c, r in parts.items() if r is None]
                if missing:
                    rep.problem("bom-variants",
                                f"{label}: no parts listed for {', '.join(missing)}")
                    continue
                covered += 1

                frame_g = float(parts["Airframe"]["Airborne Mass g"])
                if abs(frame_g + shared - cfg["FRAME_BASE_DRY_G"]) > 1.0:
                    rep.problem("bom-variants",
                                f"{label}: airframe {frame_g:.0f} g plus {shared:.0f} g "
                                f"of avionics is {frame_g + shared:.0f} g, but "
                                f"FRAME_BASE_DRY_G is {cfg['FRAME_BASE_DRY_G']:.0f} g")

                for cat, cfg_key, mult in (("Propulsion", "MOTOR_MASS_G_EACH", 4),
                                           ("Propellers", "PROP_MASS_G_EACH", 4),
                                           ("Main Battery", "BATTERY_MASS_G", 1)):
                    got = float(parts[cat]["Airborne Mass g"])
                    want = mult * cfg[cfg_key]
                    if abs(got - want) > 1.0:
                        rep.problem("bom-variants",
                                    f"{label}: {cat} mass {got:.0f} g in the BOM, "
                                    f"{want:.0f} g in config.h")

                auw = (frame_g + shared + float(parts["Propulsion"]["Airborne Mass g"])
                       + float(parts["Propellers"]["Airborne Mass g"])
                       + float(parts["Main Battery"]["Airborne Mass g"])
                       + cfg["PAYLOAD_RESERVE_G"])
                if abs(auw - cfg["AIRFRAME_AUW_G"]) > 1.5:
                    rep.problem("bom-variants",
                                f"{label}: the parts come to {auw:.0f} g AUW, config.h "
                                f"says {cfg['AIRFRAME_AUW_G']:.0f} g")

                # The ESC has to survive the current this build actually draws.
                spec = parts["Drive"]["Key Specifications"]
                m = re.search(r"(\d+)\s*A\s*cont", spec)
                if not m:
                    rep.problem("bom-variants",
                                f"{label}: cannot read a continuous rating from the ESC "
                                f"specification")
                    continue
                rating = float(m.group(1))
                per_motor = cfg["PROP_PEAK_PACK_A"] / 4.0
                margin = rating / per_motor if per_motor else 0.0
                if margin < 2.0:
                    rep.problem("bom-variants",
                                f"{label}: the listed {rating:.0f} A/channel ESC gives "
                                f"only {margin:.1f}x on a {per_motor:.1f} A per-motor "
                                f"peak. Section 4.3 argues for 2x on a hover-heavy "
                                f"profile, because published ESC ratings assume forced "
                                f"airflow")

                # And the connector, where the XT90-S is not enough.
                if cfg["PROP_PEAK_PACK_A"] > cfg.get("CONNECTOR_RATING_A", 90):
                    if pick(frame, motor, pb, "Wiring/Passives") is None:
                        rep.problem("bom-variants",
                                    f"{label}: peaks at "
                                    f"{cfg['PROP_PEAK_PACK_A']:.0f} A, above the "
                                    f"{cfg.get('CONNECTOR_RATING_A', 90):.0f} A "
                                    f"XT90-S rating, but no replacement connector is "
                                    f"listed")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"bom-mass: parts and config.h agree on mass for {covered} of "
                    f"{sum(len(m) * 2 for m in FRAME_MOTORS.values())} builds")


#  Every generated Word document, and the subtitle it is generated with.
DOCX_TARGETS = {
    "Odyssey-10-Pro-Drone-System": "Engineering Master Specification",
    "review-findings-resolution":  "Resolution Record",
    "remote-id-regulatory-notes":  "Regulatory Notes",
}


def check_docx_current(rep, fix):
    """
    Each .docx must be what its .md currently produces.

    The Word documents are committed, so they are what most people actually read, and
    they have twice drifted days behind the Markdown without anyone noticing -- once for
    two full revisions. A timestamp comparison would not have caught it either, since
    regenerating one document touches its mtime whether or not the content moved.

    So this regenerates into a temporary file and compares bytes. That is only possible
    because md2docx is byte-reproducible; before that, every run produced a different
    file and there was nothing to compare against.

    --fix regenerates. The check needs python-docx, which CI does not install, so it
    downgrades to a warning there rather than failing a build over a missing optional
    dependency.
    """
    rep.checks_run += 1
    import subprocess, tempfile, os, hashlib

    md2docx = ROOT / "tools" / "md2docx.py"
    if not md2docx.exists():
        rep.problem("docx", "tools/md2docx.py is missing", severity="warn")
        return

    probe = subprocess.run([sys.executable, "-c", "import docx"],
                           capture_output=True, text=True)
    if probe.returncode != 0:
        rep.problem("docx",
                    "python-docx is not installed, so the Word documents cannot be "
                    "verified against their Markdown here", severity="warn")
        return

    stale, checked = [], 0
    with tempfile.TemporaryDirectory() as td:
        for stem, subtitle in DOCX_TARGETS.items():
            md = ROOT / "docs" / f"{stem}.md"
            docx = ROOT / "docs" / f"{stem}.docx"
            if not md.exists():
                rep.problem("docx", f"docs/{stem}.md is missing")
                continue
            if not docx.exists():
                if not fix:
                    rep.problem("docx", f"docs/{stem}.docx has never been generated",
                                fixable=True)
                stale.append((stem, subtitle))
                continue

            out = os.path.join(td, f"{stem}.docx")
            r = subprocess.run([sys.executable, str(md2docx), str(md), out,
                                "--subtitle", subtitle],
                               capture_output=True, text=True)
            if r.returncode != 0:
                rep.problem("docx",
                            f"regenerating {stem}.docx failed: "
                            f"{r.stderr.strip().splitlines()[-1] if r.stderr.strip() else '?'}",
                            severity="warn")
                continue

            checked += 1
            want = hashlib.sha256(open(out, "rb").read()).hexdigest()
            have = hashlib.sha256(docx.read_bytes()).hexdigest()
            if want != have:
                stale.append((stem, subtitle))
                if not fix:
                    rep.problem("docx",
                                f"docs/{stem}.docx does not match docs/{stem}.md -- the "
                                f"Word document people read is not the document that "
                                f"was written", fixable=True)

    if fix and stale:
        for stem, subtitle in stale:
            md = ROOT / "docs" / f"{stem}.md"
            docx = ROOT / "docs" / f"{stem}.docx"
            r = subprocess.run([sys.executable, str(md2docx), str(md), str(docx),
                                "--subtitle", subtitle], capture_output=True, text=True)
            if r.returncode == 0:
                rep.fix("docx", f"regenerated {stem}.docx from its Markdown")

    if checked and not stale:
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"docx: all {checked} Word documents match their Markdown "
                        f"byte for byte")


def check_quoted_totals(rep, fix):
    """
    Any dollar figure a document CALLS a total must equal the computed one.

    Finding 18 was the BOM summing to one number while the specification stated
    another. The fix computed §2's master total from bom.csv and checked it -- and then
    the resolution record describing that fix went on quoting a frozen $540.50 in two
    places, which drifted the moment a part was right-sized. The check covered exactly
    one sentence in one file.

    This scans every document for a figure described as a total and compares it. Prices
    of individual parts are not described that way, so the pattern is narrow enough to
    stay quiet.
    """
    rep.checks_run += 1
    if not BOM.exists():
        return
    _, total = bom_total()

    pattern = re.compile(
        r"(?i)\btotal\b[^.\n$]{0,60}?\*{0,2}\$([\d,]+\.\d\d)\*{0,2}")

    checked = 0
    for md in sorted((ROOT / "docs").glob("*.md")):
        if md.name.endswith(".ORIGINAL.md"):
            continue          # a historical artefact, deliberately frozen
        text = read(md)
        for m in pattern.finditer(text):
            stated = float(m.group(1).replace(",", ""))
            # Historical figures are fine when the prose says they are historical.
            window = text[max(0, m.start() - 120):m.end() + 60].lower()
            if any(w in window for w in ("was ", "moved up from", "previously",
                                         "revision 2.0", "up from", "against a stated")):
                continue
            checked += 1
            if abs(stated - total) > 0.005:
                line = text[:m.start()].count("\n") + 1
                rep.problem("quoted-totals",
                            f"{md.name} line {line} states a total of ${stated:.2f}, but "
                            f"hardware/bom.csv sums to ${total:.2f}")

    rep.note = getattr(rep, "note", [])
    rep.note.append(f"quoted-totals: {checked} stated total(s) across the documents "
                    f"agree with bom.csv")


#  Pins the ESP32-P4-Function-EV-Board wires to its on-board ESP32-C6 over SDIO.
#  Documented at
#  https://github.com/espressif/esp-hosted-mcu/blob/main/docs/esp32_p4_function_ev_board.md
EV_BOARD_RESERVED = {
    14: "SDIO D0", 15: "SDIO D1", 16: "SDIO D2", 17: "SDIO D3",
    18: "SDIO CLK", 19: "SDIO CMD", 54: "C6 reset",
}

#  Pins the ESP32-P4 SILICON reserves, on any board. Unlike the SDIO list above these
#  are not a development-board constraint: using them costs the finished aircraft its
#  USB flashing port and its console.
#
#  Finding 41. Revisions up to 4.6 assigned all four USB data lines, and a single
#  pinMode() on GPIO 25 boot-looped the aircraft with CHIP_USB_UART_RESET.
#  Pins that can reach an ADC on the ESP32-P4: ADC1 on 16-23, ADC2 on 49-54.
#
#  analogReadMilliVolts() on anything else does not return a bad reading -- it takes a
#  Load access fault inside the Arduino HAL. PIN_BATT_ADC sat on GPIO 1 until the first
#  DShot-enabled run crashed in the telemetry task, and pack voltage is what drives the
#  return-to-home and landing decisions.
ADC_CAPABLE = set(range(16, 24)) | set(range(49, 55))

#  Pins the LP-UART can attach to. SOC_RTCIO_PIN_COUNT is 16, so LP-IO is GPIO 0-15.
LP_IO_CAPABLE = set(range(0, 16))

CHIP_RESERVED = {
    24: "USB Serial/JTAG D-", 25: "USB Serial/JTAG D+",
    26: "USB Serial/JTAG D-", 27: "USB Serial/JTAG D+",
    37: "UART0 TX (ROM console)", 38: "UART0 RX (ROM console)",
}


def check_devboard_pins(rep, fix):
    """
    Reports which project pins are unusable on the development board.

    A WARNING, not an error. The aircraft's flight controller is a P4 module on a
    carrier with no C6 on SDIO, so the pinout in section 9 is correct for the thing
    being built. It is wrong for the board the bring-up actually happens on, and
    discovering that with a scope attached and nothing on the wire wastes an evening.

    The list is worth keeping current as other boards get used: the failure it prevents
    is silent, because a pin driven by two peripherals produces a signal that looks
    plausible rather than absent.
    """
    rep.checks_run += 1
    text = read(CONFIG_H)

    assigned = {}
    for m in re.finditer(r"^\s*#define\s+(PIN_[A-Z0-9_]+|MOTOR[0-9]_PIN)\s+(\d+)\b",
                         text, re.M):
        assigned.setdefault(int(m.group(2)), []).append(m.group(1))

    #  The silicon's own reservations are an ERROR, not a warning: they break the real
    #  aircraft, not just the bench.
    for gpio, why in sorted(CHIP_RESERVED.items()):
        for name in assigned.get(gpio, []):
            rep.problem("devboard-pins",
                        f"{name} is on GPIO {gpio}, which the ESP32-P4 uses for "
                        f"{why}. This is silicon, not a board choice -- it costs the "
                        f"aircraft its USB console and flashing port. See finding 41.")

    clashes = []
    for gpio, why in sorted(EV_BOARD_RESERVED.items()):
        for name in assigned.get(gpio, []):
            clashes.append(f"{name} (GPIO {gpio}) vs {why}")

    if clashes:
        rep.problem("devboard-pins",
                    "on an ESP32-P4-Function-EV-Board these pins are taken by the "
                    "on-board C6: " + "; ".join(clashes) +
                    ". Fine on the aircraft, which has no C6 on SDIO -- see section "
                    "4.3.2 before wiring the bench",
                    severity="warn")
    else:
        rep.note = getattr(rep, "note", [])
        rep.note.append("devboard-pins: no project pin collides with the EV board's C6")

    #  A pin has to be capable of what it is asked to do. Neither of these fails
    #  politely: the ADC one faults, and the LP-UART one refuses to start the driver.
    for name, gpio in ((n, g) for g, names in assigned.items() for n in names):
        if name == "PIN_BATT_ADC" and gpio not in ADC_CAPABLE:
            rep.problem("devboard-pins",
                        f"PIN_BATT_ADC is GPIO {gpio}, which has no ADC on the ESP32-P4. "
                        f"ADC1 is 16-23 and ADC2 is 49-54; anything else takes a Load "
                        f"access fault in analogReadMilliVolts(), in the task that reads "
                        f"pack voltage. See finding 42.")

    #  The LP-UART constraint only applies if something is actually on the LP-UART.
    main_cpp = ROOT / "firmware" / "flight-controller" / "src" / "main.cpp"
    if main_cpp.exists():
        mtext = read(main_cpp)
        m = re.search(r"HardwareSerial\s+AuxSerial\((\d+)\)", mtext)
        if m and int(m.group(1)) == 5:
            gpio = next((g for g, names in assigned.items()
                         if "PIN_AUX_BUS_TX" in names), None)
            if gpio is not None and gpio not in LP_IO_CAPABLE:
                rep.problem("devboard-pins",
                            f"AuxSerial is on the LP-UART (UART5) but PIN_AUX_BUS_TX is "
                            f"GPIO {gpio}. The LP-UART can only attach to LP-IO pins, "
                            f"which are GPIO 0-15 on the P4 -- the driver refuses to "
                            f"start otherwise.")

    # A pin driven twice is a defect on ANY board, so that stays an error.
    for gpio, names in sorted(assigned.items()):
        if len(names) > 1:
            rep.problem("devboard-pins",
                        f"GPIO {gpio} is assigned to {len(names)} functions: "
                        + ", ".join(names))


# =====================================================================================
#  Registry
# =====================================================================================
def check_gpio_map(rep, fix):
    """Section 9.2's pinout versus config.h, pin by pin.

    This is the diagram somebody solders from. When finding 42 moved PIN_BATT_ADC off
    GPIO 1 -- which has no ADC on the P4 -- config.h was corrected and section 9.2 was
    not, so for three commits the document told you to wire the battery divider to a pin
    the firmware no longer reads. Nothing caught it: prose-constants checks numeric
    claims in prose, and devboard-pins checks config.h against the silicon. Neither
    compares the two documents' idea of the pinout with each other.
    """
    cfg = read(CONFIG_H)
    doc = read(SPEC)

    defined = {}
    for m in re.finditer(r"^#define\s+(PIN_[A-Z0-9_]+|MOTOR[0-9]_PIN)\s+(\d+)", cfg, re.M):
        defined.setdefault(int(m.group(2)), []).append(m.group(1))

    sec = re.search(r"### 9\.2 GPIO map(.*?)^### 9\.3", doc, re.S | re.M)
    if not sec:
        rep.problem("gpio-map", "section 9.2's GPIO map is missing entirely")
        return
    mapped = {int(m.group(1)): m.group(2).strip()
              for m in re.finditer(r"^GPIO (\d+)\s+-->\s+(.+)$", sec.group(1), re.M)}

    for gpio in sorted(set(defined) - set(mapped)):
        rep.problem("gpio-map",
                    f"config.h puts {', '.join(defined[gpio])} on GPIO {gpio}, but "
                    f"section 9.2's map does not list that pin -- the document somebody "
                    f"wires from is missing a connection the firmware drives")
    for gpio in sorted(set(mapped) - set(defined)):
        rep.problem("gpio-map",
                    f"section 9.2 maps GPIO {gpio} to \"{mapped[gpio][:60]}\", but no "
                    f"PIN_* in config.h is on that GPIO -- wiring it would connect a "
                    f"peripheral to a pin the firmware never touches")

    if not (set(defined) ^ set(mapped)):
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"gpio-map: section 9.2 and config.h agree on all "
                        f"{len(defined)} assigned GPIOs")


def check_uart_allocation(rep, fix):
    """Every HardwareSerial(n) in the firmware, against itself and against section 9.1.

    Two HardwareSerial objects on one port is not a compile error and not a runtime
    error. The second begin() silently re-points the peripheral, so the loser simply
    stops working -- no message, no failed init, nothing on the console. Finding 44 was
    exactly this: AuxSerial and GnssSerial both took UART1, and because sensors.cpp
    calls GnssSerial.begin() after setup() calls auxBus.begin(), the AUX broadcast to
    the beacon and the Remote ID module went dead.

    A pin check cannot see it. Both objects have their own perfectly valid pins; it is
    the peripheral behind them that collides.
    """
    main = read(ROOT / "firmware" / "flight-controller" / "src" / "main.cpp")
    doc  = read(SPEC)

    #  object name -> port number
    objs = {m.group(1): int(m.group(2))
            for m in re.finditer(r"^HardwareSerial\s+(\w+)\s*\(\s*(\d+)\s*\)\s*;",
                                 main, re.M)}
    if not objs:
        rep.problem("uart-allocation", "no HardwareSerial objects found in main.cpp")
        return

    by_port = {}
    for name, port in objs.items():
        by_port.setdefault(port, []).append(name)

    for port, names in sorted(by_port.items()):
        if len(names) > 1:
            rep.problem("uart-allocation",
                        f"{' and '.join(sorted(names))} all claim HardwareSerial({port}). "
                        f"One peripheral wins silently -- whichever calls begin() last "
                        f"re-points the port onto its own pins and the others stop "
                        f"working with nothing on the console. See finding 44.")

    #  Section 9.1 claims a peripheral per port; the firmware must agree on which ports
    #  are in use, even though the table names peripherals rather than C++ objects.
    sec = re.search(r"### 9\.1 UART allocation(.*?)^### 9\.2", doc, re.S | re.M)
    if not sec:
        rep.problem("uart-allocation", "section 9.1's UART table is missing")
        return

    documented = set()
    for m in re.finditer(r"^\|\s*UART(\d)\s*\|\s*([^|]+?)\s*\|", sec.group(1), re.M):
        if "unused" not in m.group(2).lower():
            documented.add(int(m.group(1)))

    used = set(by_port)
    for port in sorted(used - documented):
        rep.problem("uart-allocation",
                    f"the firmware opens UART{port} ({', '.join(by_port[port])}) but "
                    f"section 9.1 does not list it as used")
    for port in sorted(documented - used):
        rep.problem("uart-allocation",
                    f"section 9.1 lists UART{port} as in use, but no HardwareSerial "
                    f"in the firmware claims it")

    if not (used ^ documented) and all(len(v) == 1 for v in by_port.values()):
        rep.note = getattr(rep, "note", [])
        rep.note.append(f"uart-allocation: {len(objs)} ports, one peripheral each, "
                        f"matching section 9.1")


CHECKS = [
    ("comment-continuation", "// comments ending in a backslash swallow the next line",
     check_comment_backslash),
    ("brackets", "brace, paren and bracket balance in every source file", check_brackets),
    ("whitespace", "trailing whitespace and missing final newlines", check_whitespace),
    ("bom", "BOM per-line arithmetic and the total quoted in the specification", check_bom),
    ("spec-constants", "battery thresholds, AUW and TWR agree with config.h",
     check_spec_constants),
    ("quoted-totals",
     "every figure the documents call a total matches bom.csv",
     check_quoted_totals),
    ("devboard-pins",
     "project pins versus what the development board has already claimed",
     check_devboard_pins),
    ("gpio-map",
     "section 9.2's pinout and config.h name the same GPIOs",
     check_gpio_map),
    ("uart-allocation",
     "no two peripherals share a UART, and section 9.1 agrees with the firmware",
     check_uart_allocation),
    ("bom-mass",
     "the parts list and config.h agree on mass, ESC margin and connector, for every build",
     check_bom_mass),
    ("repo-layout",
     "section 10.1 lists every source file that exists",
     check_repo_layout),
    ("build-switches",
     "every documented build switch is actually overridable",
     check_build_switches),
    ("unverified-defaults",
     "features that have never run on hardware are off by default",
     check_unverified_defaults),
    ("hook-coverage",
     "every suite the pre-push hook runs behind its gate is also watched by it",
     check_hook_coverage),
    ("line-endings",
     "every tracked text file matches the line endings .gitattributes declares",
     check_line_endings),
    ("blackbox", "BlackBox record layout agrees between firmware and decoder",
     check_blackbox_layout),
    ("findings", "all 18 review findings appear in both documents", check_findings),
    ("doc-refs", "'docs section N' references resolve to a real section", check_doc_refs),
    ("identity", "no operator secret is committed; CTA serial is well formed",
     check_identity_placeholders),
    ("gitattributes", "binary and LF rules are present", check_gitattributes),
    ("arm-gate", "every sensor the arm gate requires is actually reported",
     check_arm_gate),
    ("workflow-cost", "the CI workflow stays on Linux, unscheduled and unmatrixed",
     check_workflow_cost),
    ("readme", "README badge counts and revision match reality", check_readme),
    ("prose-constants", "numbers stated in prose match the constants they quote",
     check_prose_constants),
    ("notch-observability",
     "the documentation does not promise a measurement the log rate cannot deliver",
     check_notch_observability),
    ("build-configs",
     "every frame x motor x propeller build is coherent, distinct and correctly ordered",
     check_prop_configs),
    #  LAST, deliberately. readme and repo-layout rewrite the
    #  specification Markdown when they fix their counts, so a Word
    #  document generated before them would be stale on arrival.
    ("docx",
     "every generated Word document matches the Markdown it came from",
     check_docx_current),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true",
                    help="apply the mechanically safe corrections")
    ap.add_argument("--list", action="store_true", help="list the checks and exit")
    ap.add_argument("--only", metavar="NAME", help="run a single check by name")
    args = ap.parse_args()

    if args.list:
        print("Checks:\n")
        for name, desc, _ in CHECKS:
            print(f"  {name:<22} {desc}")
        print("\n--fix can correct: whitespace, bom, spec-constants.")
        print("Everything else is reported only, because there is no single")
        print("mechanically correct resolution.")
        return 0

    print("=" * 78)
    print(" Odyssey-10 Pro -- consistency check" + ("  [--fix]" if args.fix else ""))
    print("=" * 78)

    rep = Report()
    for name, _desc, fn in CHECKS:
        if args.only and args.only != name:
            continue
        try:
            fn(rep, args.fix)
        except Exception as exc:                      # a broken check must not hide others
            rep.problem(name, f"check itself failed: {exc.__class__.__name__}: {exc}")

    rc = rep.summary()
    if rc == 0 and not rep.problems:
        print("  Everything agrees.")
    elif not args.fix and any(p[3] for p in rep.problems):
        print("\n  Re-run with --fix to apply the corrections marked fixable.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
