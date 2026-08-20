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
"""

import argparse
import csv
import os
import re
import struct
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
    """Reads a file, remembering its dominant line ending, and returns LF-normalised text."""
    raw = path.read_bytes()
    crlf = raw.count(b"\r\n")
    lf = raw.count(b"\n") - crlf
    _EOL[path] = "\r\n" if crlf > lf else "\n"
    return raw.decode("utf-8").replace("\r\n", "\n")


def write(path, text):
    """Writes text back using whatever line ending the file already had."""
    eol = _EOL.get(path, os.linesep)
    if eol == "\r\n":
        text = text.replace("\n", "\r\n")
    path.write_bytes(text.encode("utf-8"))


def config_defines(text):
    """Extracts #define NAME VALUE pairs, ignoring function-like macros."""
    out = {}
    for m in re.finditer(r"^#define\s+([A-Z0-9_]+)\s+([^\n/]+)", text, re.M):
        out[m.group(1)] = m.group(2).strip()
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


def source_files():
    files = []
    for g in SOURCE_GLOBS:
        files.extend(sorted(ROOT.glob(g)))
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
    cells = int(defines["CELL_COUNT"])

    def volts(key):
        return float(defines[key.replace("PACK", "CELL")].rstrip("f")) * cells

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
        ("all-up weight", str(int(float(defines["AIRFRAME_AUW_G"].rstrip("f")))),
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
    auw = float(defines["AIRFRAME_AUW_G"].rstrip("f"))
    thrust = float(defines["MOTOR_MAX_THRUST_G"].rstrip("f")) * 4.0
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
def check_blackbox_layout(rep, fix):
    rep.checks_run += 1
    types = read(TYPES_H)
    dec = read(DECODER)

    body = types.split("struct __attribute__((packed)) BlackBoxRecord {")[1].split("};")[0]
    sizes = {"uint32_t": 4, "int32_t": 4, "uint16_t": 2, "int16_t": 2,
             "uint8_t": 1, "int8_t": 1, "float": 4}
    total = 0
    for line in body.splitlines():
        line = line.split("//")[0].strip().rstrip(";")
        if not line:
            continue
        ty, names = line.split(None, 1)
        if ty not in sizes:
            rep.problem("blackbox", f"unhandled type '{ty}' in BlackBoxRecord")
            return
        total += sizes[ty] * len([x for x in names.split(",") if x.strip()])

    m = re.search(r'RECORD_FMT\s*=\s*"([^"]+)"', dec)
    if not m:
        rep.problem("blackbox", "could not find RECORD_FMT in blackbox_decode.py")
        return
    py = struct.calcsize(m.group(1))

    if py != total:
        rep.problem("blackbox",
                    f"BlackBoxRecord is {total} bytes in types.h but the decoder's "
                    f"RECORD_FMT is {py} bytes -- logs would decode as garbage")

    # The decoder's version gate must accept what the firmware writes.
    fw_ver = re.search(r"#define\s+BLACKBOX_VERSION\s+(\d+)", types)
    dec_ver = re.search(r"SUPPORTED_VERSIONS\s*=\s*\(([^)]*)\)", dec)
    if fw_ver and dec_ver:
        supported = [int(x) for x in re.findall(r"\d+", dec_ver.group(1))]
        if int(fw_ver.group(1)) not in supported:
            rep.problem("blackbox",
                        f"firmware writes format v{fw_ver.group(1)} but the decoder "
                        f"only accepts {supported}")


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
#  Registry
# =====================================================================================
CHECKS = [
    ("comment-continuation", "// comments ending in a backslash swallow the next line",
     check_comment_backslash),
    ("brackets", "brace, paren and bracket balance in every source file", check_brackets),
    ("whitespace", "trailing whitespace and missing final newlines", check_whitespace),
    ("bom", "BOM per-line arithmetic and the total quoted in the specification", check_bom),
    ("spec-constants", "battery thresholds, AUW and TWR agree with config.h",
     check_spec_constants),
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
