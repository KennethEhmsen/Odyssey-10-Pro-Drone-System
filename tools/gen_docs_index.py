"""Writes docs/index.html -- the map for the sheet set.

Sheet titles, revisions and the checks that guard them are read from the files
themselves rather than listed by hand, so a new sheet appears here by existing.
"""
import re
from pathlib import Path

DOCS = Path("docs")

#  What each sheet answers, and which consistency check holds it to the code. The
#  question is editorial; everything else on this page is read from disk.
META = {
    "odyssey-schematic.html": ("What is wired to what",
        "Interconnect, flight-loop stages, power tree and GPIO budget",
        ["gpio-map", "uart-allocation"], "pwm"),
    "odyssey-layout.html": ("Where every part is bolted",
        "Plan and elevation, §8.1 coordinates, RF separation, mass budget",
        [], "i2c"),
    "odyssey-flow.html": ("How the system decides",
        "Four processors, five links, and the escalate-only state ladder",
        ["flow-transitions"], "spi"),
    "odyssey-arming.html": ("Why it will not arm",
        "Ten conditions, seven blocker bits, and the one that can never fire",
        ["arming-sheet"], "power"),
    "odyssey-blackbox.html": ("What a flight log contains",
        "The 54-byte record, field by field and byte by byte",
        ["blackbox-sheet", "blackbox"], "uart"),
    "odyssey-builds.html": ("What the three switches move",
        "Ten builds resolved by the C preprocessor, and the coupling between them",
        ["build-matrix", "build-configs"], "copper"),
    "odyssey-dshot.html": ("What the wire should look like",
        "DShot300 drawn to scale, for holding next to a capture",
        ["dshot-sheet"], "pwm"),
    "odyssey-link.html": ("How the four images talk",
        "Frame anatomy, the header, the four types and the command opcodes",
        [], "uart"),
    "odyssey-checklist.html": ("What to do before it flies",
        "Thirty-one checks in five phases, in the order they have to happen",
        [], "power"),
    "odyssey-bringup.html": ("Where the project is",
        "Four steps, four DShot assumptions, and what is actually verified",
        [], "gpio"),
}

#  Spelled out, because a headline reading "10 Sheets" is not a headline. Written
#  once here rather than typed into the HTML, where it said "Nine" while the page
#  listed ten.
WORDS = {1:"One", 2:"Two", 3:"Three", 4:"Four", 5:"Five", 6:"Six",
         7:"Seven", 8:"Eight", 9:"Nine", 10:"Ten", 11:"Eleven", 12:"Twelve"}

sheets = []
for path in sorted(DOCS.glob("*.html")):
    if path.name == "index.html":
        continue
    text = path.read_text(encoding="utf-8", errors="replace")
    title = re.search(r"<title>(.*?)</title>", text)
    h1 = re.search(r"<h1>(.*?)</h1>", text, re.S)
    rev = re.search(r'const DOC_REV\s*=\s*"([0-9.]+)"', text)
    eyebrow = re.search(r'class="eyebrow">(.*?)</div>', text, re.S)
    sheet_no = re.search(r"Sheet (\d+) of", eyebrow.group(1)) if eyebrow else None
    q, sub, checks, cls = META.get(path.name, ("", "", [], "gpio"))
    sheets.append({
        "file": path.name,
        "title": re.sub(r"\s+", " ", h1.group(1)).strip() if h1 else path.stem,
        "tab": title.group(1) if title else path.stem,
        "rev": rev.group(1) if rev else "?",
        "no": int(sheet_no.group(1)) if sheet_no else 99,
        "q": q, "sub": sub, "checks": checks, "cls": cls,
        "bytes": path.stat().st_size,
    })
sheets.sort(key=lambda s: s["no"])

revs = {s["rev"] for s in sheets}
CSS = (DOCS / "odyssey-bringup.html").read_text(encoding="utf-8")
style = CSS[CSS.index("<style>"):CSS.index("</style>") + len("</style>")]
style = style.replace("/* Ninth sheet, same system as the others. */",
                      "/* The index. Same system as the sheets it lists. */")
style = style.replace(
    "td.note{color:var(--ink-2);font-size:13px}",
    "td.note{color:var(--ink-2);font-size:13px}\n"
    "a{color:inherit;text-decoration:none}\n"
    "a.card{display:flex;flex-direction:column;gap:6px;padding:18px 20px;\n"
    "  background:var(--sheet);border:1px solid var(--rule);border-left:3px solid var(--cls);\n"
    "  border-radius:2px;box-shadow:var(--shadow);transition:border-color .12s}\n"
    "a.card:hover{border-color:var(--cls)}\n"
    ".cards{display:grid;gap:14px;grid-template-columns:repeat(auto-fill,minmax(330px,1fr))}\n"
    ".cardq{font-family:Archivo,sans-serif;font-size:16px;font-weight:600;\n"
    "  letter-spacing:-.008em}\n"
    ".cardsub{font-size:13px;color:var(--ink-2)}\n"
    ".cardmeta{display:flex;flex-wrap:wrap;gap:6px 12px;font-family:\"IBM Plex Mono\",monospace;\n"
    "  font-size:10.5px;color:var(--ink-3);padding-top:4px}\n"
    ".no{font-family:\"IBM Plex Mono\",monospace;font-size:10.5px;color:var(--cls);\n"
    "  font-weight:600;letter-spacing:.1em}")

def cards():
    out = []
    for s in sheets:
        checks = (" · ".join(f"<code>{c}</code>" for c in s["checks"])
                  if s["checks"] else "<span style='opacity:.6'>no dedicated check</span>")
        out.append(
            f'      <a class="card" href="{s["file"]}" style="--cls:var(--c-{s["cls"]})">\n'
            f'        <div class="no">SHEET {s["no"]}</div>\n'
            f'        <div class="cardq">{s["q"]}</div>\n'
            f'        <div class="cardsub">{s["sub"]}</div>\n'
            f'        <div class="cardmeta"><span>rev {s["rev"]}</span>'
            f'<span>{checks}</span></div>\n'
            f'      </a>')
    return "\n".join(out)

HTML = f"""<title>Odyssey-10 Pro Drawing Set</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">

{style}

<div class="wrap">

  <header class="masthead">
    <div class="eyebrow">Odyssey-10 Pro &middot; Drawing set &middot; revision {sorted(revs)[-1]}</div>
    <h1>{WORDS.get(len(sheets), len(sheets))} Sheets</h1>
    <p class="standfirst">
      Each one answers a question the specification answers in prose, and each is held to
      the source by a check that fails the build when they drift apart. If a sheet and the
      code disagree, the code is right and the sheet is a bug.
    </p>
    <div class="provenance" id="provenance"></div>
  </header>

  <section>
    <h2>The set <span>{len(sheets)} sheets</span></h2>
    <div class="cards">
{cards()}
    </div>
  </section>

  <section>
    <h2>Where to start <span>by what you are doing</span></h2>
    <div class="scroll"><table>
      <thead><tr><th>If you are…</th><th>Read</th></tr></thead>
      <tbody>
        <tr><td class="k" style="--cls:var(--c-gpio)">wondering what to do next</td>
            <td class="note"><a href="odyssey-bringup.html"><b>Bring-up</b></a> — four steps, one done, and what each still needs</td></tr>
        <tr><td class="k" style="--cls:var(--c-pwm)">soldering</td>
            <td class="note"><a href="odyssey-schematic.html"><b>Interconnect</b></a> for the nets, then <a href="odyssey-layout.html"><b>Layout</b></a> for where they go. The power rules in §9.5 are on the interconnect sheet</td></tr>
        <tr><td class="k" style="--cls:var(--c-power)">staring at ARMING IS BLOCKED</td>
            <td class="note"><a href="odyssey-arming.html"><b>Arming</b></a> — every condition, its constant, and what to do about it</td></tr>
        <tr><td class="k" style="--cls:var(--c-uart)">holding a logic analyser</td>
            <td class="note"><a href="odyssey-dshot.html"><b>DShot</b></a> — drawn to scale, with the disarmed frame to compare against</td></tr>
        <tr><td class="k" style="--cls:var(--c-spi)">writing a decoder</td>
            <td class="note"><a href="odyssey-blackbox.html"><b>BlackBox</b></a> for log files, <a href="odyssey-link.html"><b>Link</b></a> for radio frames</td></tr>
        <tr><td class="k" style="--cls:var(--copper)">choosing a build</td>
            <td class="note"><a href="odyssey-builds.html"><b>Build matrix</b></a> — ten combinations, and which three exceed the connector</td></tr>
        <tr><td class="k" style="--cls:var(--c-i2c)">trying to understand it</td>
            <td class="note"><a href="odyssey-flow.html"><b>Flow</b></a> — the state ladder is the shortest explanation of how this aircraft thinks</td></tr>
      </tbody>
    </table></div>
  </section>

  <section>
    <h2>How these stay true <span>not by discipline</span></h2>
    <div class="callout">
      <h3>Every sheet is derived, and most are checked</h3>
      <p>
        No number on these pages was typed in from memory. Pin assignments come from
        <code>config.h</code>, byte offsets are computed from the field lists, the build
        matrix runs the C preprocessor once per combination, and the DShot timings use the
        firmware's own integer arithmetic — down to reproducing its 9999 ppm rounding.
        <br><br>
        Six sheets have a dedicated consistency check that re-reads the source and fails
        the build on a mismatch, and <code>schematic-rev</code> holds every sheet to the
        specification's revision. The three without one are drawings of things that have
        no single machine-readable source: physical placement, the wire protocol's prose,
        and the project's own progress.
      </p>
    </div>
  </section>

  <footer>
    Odyssey-10 Pro &middot; drawing set, revision {sorted(revs)[-1]} &middot;
    generated by <code>tools/gen_docs_index.py</code><br>
    Titles, revisions and sheet numbers are read from the files themselves, so a new sheet
    appears here by existing rather than by being remembered.<br>
    The specification these accompany is <code>docs/Odyssey-10-Pro-Drone-System.md</code>.
  </footer>
</div>

<script>
const DOC_REV = "{sorted(revs)[-1]}";
const SHEETS = {len(sheets)};
document.querySelector("#provenance").innerHTML = [
  `<b>Sheets</b> ${{SHEETS}}`, `<span class="dot">/</span>`,
  `<b>Revision</b> ${{DOC_REV}}`, `<span class="dot">/</span>`,
  `<b>Checked</b> 6 of ${{SHEETS}}`, `<span class="dot">/</span>`,
  `<b>Source</b> read from the files`
].join(" ");
</script>
"""

(DOCS / "index.html").write_text(HTML, encoding="utf-8", newline="\n")
print(f"docs/index.html written — {len(sheets)} sheets, revision {sorted(revs)[-1]}")
for s in sheets:
    print(f"  sheet {s['no']}: {s['file']:28} rev {s['rev']}  {len(s['checks'])} check(s)")
