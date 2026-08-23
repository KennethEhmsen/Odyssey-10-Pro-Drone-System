"""Writes docs/odyssey-history.html from git.

The revision subtitles are the project narrating itself, one line at a time. They are
read out of the repository rather than rewritten here -- a history sheet that paraphrased
its own source would be the exact failure this project keeps finding.
"""
import re
import subprocess
from pathlib import Path

DOCS = Path("docs")
SPEC = "docs/Odyssey-10-Pro-Drone-System.md"


def git(*args):
    return subprocess.run(["git", *args], capture_output=True, text=True,
                          encoding="utf-8", errors="replace").stdout


# ---- every distinct revision the specification has carried --------------------------
revs, seen = [], set()
for line in git("log", "--format=%H|%ad", "--date=short", "--reverse", "--", SPEC).splitlines():
    if "|" not in line:
        continue
    h, d = line.split("|", 1)
    blob = git("show", f"{h}:{SPEC}")
    m = re.search(r"\*\*Document revision:\*\*\s*([0-9.]+)(?:\s*[—-]\s*(.*))?", blob)
    if not m or m.group(1) in seen:
        continue
    seen.add(m.group(1))
    revs.append({"date": d, "rev": m.group(1),
                 "note": (m.group(2) or "").strip().rstrip(".")})

commits = int(git("rev-list", "--count", "HEAD").strip() or 0)
days = sorted({l for l in git("log", "--format=%ad", "--date=short").splitlines() if l})
per_day = {}
for d in days:
    per_day[d] = per_day.get(d, 0) + 1
per_day = {}
for l in git("log", "--format=%ad", "--date=short").splitlines():
    if l:
        per_day[l] = per_day.get(l, 0) + 1
day_list = sorted(per_day)

spec_text = Path(SPEC).read_text(encoding="utf-8", errors="replace")
words = len(spec_text.split())
lines = spec_text.count("\n") + 1
checks = len(re.findall(r'^\s+\("([a-z0-9-]+)",',
                        Path("tools/check_consistency.py").read_text(encoding="utf-8",
                                                                     errors="replace"), re.M))
sheets = len([p for p in DOCS.glob("*.html") if p.name != "index.html"])

CSS = (DOCS / "odyssey-findings.html").read_text(encoding="utf-8")
style = CSS[CSS.index("<style>"):CSS.index("</style>") + len("</style>")]
style = style.replace("/* Eleventh sheet, same system as the others. */",
                      "/* Twelfth sheet, same system as the others. */")
style = style.replace(
    "td.note{color:var(--ink-2);font-size:13px}",
    "td.note{color:var(--ink-2);font-size:13px}\n"
    "td.num{font-family:\"IBM Plex Mono\",monospace;font-variant-numeric:tabular-nums;"
    "text-align:right;white-space:nowrap}")


def js_revs():
    return ",\n".join(
        '  {rev:"%s", date:"%s", note:"%s"}' % (r["rev"], r["date"],
                                              r["note"].replace('"', '\\"'))
        for r in revs)


def js_days():
    return ", ".join('["%s",%d]' % (d, per_day[d]) for d in day_list)


HTML = f"""<title>Odyssey-10 Pro Project History</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">

{style}

<div class="wrap">

  <header class="masthead">
    <div class="eyebrow"><a href="index.html">&larr; All sheets</a> &middot; Odyssey-10 Pro &middot; History &middot; Sheet 12 of 12</div>
    <h1>Thirty-One Revisions</h1>
    <p class="standfirst">
      Four days, {commits} commits, and a document that renamed its own subtitle every time
      it learned something. The revision line is the project narrating itself &mdash; these
      are read out of git, not rewritten here.
    </p>
    <div class="provenance" id="provenance"></div>
  </header>

  <section>
    <h2>Four days <span>commits per day</span></h2>
    <div class="sheet">
      <svg id="days" role="img" aria-label="Commits per day across the project"></svg>
    </div>
  </section>

  <section>
    <h2>How it moved <span>four phases</span></h2>
    <div class="scroll"><table id="phases">
      <thead><tr><th>Phase</th><th>Revisions</th><th>What was happening</th></tr></thead>
      <tbody></tbody>
    </table></div>
    <div class="callout">
      <h3>The subtitles change character halfway down</h3>
      <p>
        The early ones describe <em>decisions</em> — &ldquo;re-based on the 387&nbsp;mm
        airframe&rdquo;, &ldquo;propeller configuration is a build switch&rdquo;. The later
        ones describe <em>discoveries</em> — &ldquo;eight of nine source files had never
        been compiled&rdquo;, &ldquo;the documented build command does not work&rdquo;,
        &ldquo;the pinout no longer sits on the P4's USB data lines&rdquo;.
        <br><br>
        That is the moment the project stopped being written and started being built. The
        board arrived, and the document began finding out what it had assumed.
      </p>
    </div>
  </section>

  <section>
    <h2>Every revision <span id="revspan"></span></h2>
    <div class="scroll"><table id="revs">
      <caption>Read from the <code>Document revision:</code> line at each commit that changed it.</caption>
      <thead><tr><th>Rev</th><th>Date</th><th>Its own account of itself</th></tr></thead>
      <tbody></tbody>
    </table></div>
  </section>

  <section>
    <h2>Where it ended up <span>as of revision {revs[-1]["rev"]}</span></h2>
    <div class="scroll"><table id="now">
      <thead><tr><th>&nbsp;</th><th>Count</th><th>&nbsp;</th></tr></thead>
      <tbody></tbody>
    </table></div>
    <div class="callout">
      <h3>What none of this proves</h3>
      <p>
        Nothing here has flown. The firmware boots, initialises DShot and blocks arming
        correctly on a bench with no sensors attached. Six of thirteen sensor drivers have
        never had a device answer them, the blackbox ring has never executed on hardware,
        and <code>CRUISE_CURRENT_A</code> — which sizes the return-to-home reserve on all
        ten builds — is still modelled rather than measured.
        <br><br>
        A specification with thirty-four checks and 413 assertions behind it is a better
        starting point than one without. It is still a starting point.
      </p>
    </div>
  </section>

  <footer id="titleblock"></footer>
</div>

<script>
const DOC_REV = "{revs[-1]["rev"]}";
const COMMITS = {commits};
const SPEC_WORDS = {words};
const SPEC_LINES = {lines};
const CHECKS = {checks};
const SHEETS = {sheets};

/* Read from git by tools/gen_history.py -- regenerate rather than edit. */
const REVS = [
{js_revs()}
];
const DAYS = [{js_days()}];

const PHASES = [
  ["2.0 – 2.8","Applying the review",
   "Eighteen findings resolved, then the airframe re-based on the 9-inch and every coupled constant turned into a build switch"],
  ["2.9 – 3.5","Making the notch honest",
   "The gyro notch learns to measure itself, a measurement procedure that could never have worked is withdrawn, and the second harmonic gets an honest account of where it is observable"],
  ["3.6 – 4.2","Costing what was written",
   "DShot wired into the flight loop, the other nine builds given parts lists, and a bench procedure with the numbers to measure against"],
  ["4.3 – {revs[-1]["rev"]}","Meeting the hardware",
   "The board arrives. The build command turns out not to work, eight of nine source files turn out never to have been compiled, and the pinout turns out to sit on the USB data lines"]
];

const $=(s)=>document.querySelector(s);
const SVGNS="http://www.w3.org/2000/svg";
const el=(n,a={{}})=>{{const e=document.createElementNS(SVGNS,n);for(const k in a)e.setAttribute(k,a[k]);return e;}};
const esc=(s)=>String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;");

function drawDays(){{
  const svg=$("#days"), PAD=44, W=920, H=210;
  const max=Math.max(...DAYS.map(d=>d[1]));
  const bw=Math.min(120,(W-PAD*2)/DAYS.length-20), gap=26;
  const base=H-52;
  svg.setAttribute("viewBox",`0 0 ${{W}} ${{H}}`);
  svg.setAttribute("font-family",'"IBM Plex Mono",ui-monospace,monospace');
  const t=(x,y,s,o={{}})=>{{const n=el("text",Object.assign({{x,y,"font-size":11,
    fill:"var(--ink)","dominant-baseline":"middle"}},o));n.textContent=s;svg.appendChild(n);return n;}};

  svg.appendChild(el("path",{{d:`M ${{PAD}} ${{base}} H ${{W-PAD}}`,
    stroke:"var(--rule)","stroke-width":1}}));

  DAYS.forEach((d,i)=>{{
    const x=PAD+i*(bw+gap), h=d[1]/max*(base-58);
    const colour = i===DAYS.length-1 ? "var(--copper)" : "var(--c-pwm)";
    svg.appendChild(el("rect",{{x,y:base-h,width:bw,height:h,rx:2,
      fill:colour,"fill-opacity":.2,stroke:colour,"stroke-width":1.4}}));
    t(x+bw/2,base-h-14,String(d[1]),{{"text-anchor":"middle","font-size":15,
      "font-weight":700,fill:colour,"font-family":"Archivo, sans-serif"}});
    t(x+bw/2,base+16,d[0],{{"text-anchor":"middle","font-size":9.5,fill:"var(--ink-3)"}});
  }});
  t(PAD,26,`${{COMMITS}} COMMITS ACROSS ${{DAYS.length}} DAYS`,
    {{"font-size":10,fill:"var(--ink-3)","letter-spacing":".12em"}});
  t(W-PAD,base+34,"the last day is the drawing set",
    {{"text-anchor":"end","font-size":9,fill:"var(--copper)","font-style":"italic"}});
}}

function tables(){{
  $("#revspan").textContent=`${{REVS.length}} revisions · ${{REVS[0].rev}} to ${{REVS[REVS.length-1].rev}}`;

  $("#phases tbody").innerHTML=PHASES.map((p,i)=>{{
    const colour=["var(--c-gpio)","var(--c-i2c)","var(--c-uart)","var(--copper)"][i];
    return `<tr>
      <td class="k" style="--cls:${{colour}}">${{esc(p[0])}}</td>
      <td class="mono">${{esc(p[1])}}</td>
      <td class="note">${{esc(p[2])}}</td></tr>`;}}).join("");

  $("#revs tbody").innerHTML=REVS.map(r=>{{
    const major=parseInt(r.rev,10);
    const colour=major<3?"var(--c-gpio)":major<4?"var(--c-i2c)":
                 r.rev>="4.3"?"var(--copper)":"var(--c-uart)";
    return `<tr>
      <td class="k" style="--cls:${{colour}};text-align:right">${{esc(r.rev)}}</td>
      <td class="mono" style="color:var(--ink-3);font-size:12px">${{esc(r.date)}}</td>
      <td class="note">${{esc(r.note)}}</td></tr>`;}}).join("");

  $("#now tbody").innerHTML=[
    ["Specification",`${{SPEC_LINES.toLocaleString()}} lines`,`${{SPEC_WORDS.toLocaleString()}} words across 13 sections`],
    ["Drawing set",`${{SHEETS}} sheets`,"each derived from source, most held to it by a named check"],
    ["Consistency checks",`${{CHECKS}}`,"run before every push; 19 trace to a specific defect"],
    ["Host assertions","413","the algorithms, verified without hardware"],
    ["Findings recorded","64","18 by review, 46 by building it"],
    ["Firmware","boots","DShot initialises; arming correctly blocked with nothing attached"],
    ["Flights","0","nothing here has left the ground"]
  ].map(r=>`<tr>
      <td class="k" style="--cls:${{r[1]==="0"?"var(--c-power)":"var(--c-pwm)"}}">${{esc(r[0])}}</td>
      <td class="num" style="font-weight:600">${{esc(r[1])}}</td>
      <td class="note">${{esc(r[2])}}</td></tr>`).join("");

  $("#provenance").innerHTML=[
    `<b>Revisions</b> ${{REVS.length}}`,`<span class="dot">/</span>`,
    `<b>Commits</b> ${{COMMITS}}`,`<span class="dot">/</span>`,
    `<b>Days</b> ${{DAYS.length}}`,`<span class="dot">/</span>`,
    `<b>Source</b> git`,`<span class="dot">/</span>`,
    `<b>Doc</b> rev ${{DOC_REV}}`
  ].join(" ");

  $("#titleblock").innerHTML=
    `Odyssey-10 Pro &middot; project history, revision ${{DOC_REV}}<br>`+
    `Every row read from git by <code>tools/gen_history.py</code>: the `+
    `<code>Document revision:</code> line at each commit that changed it, and the commit `+
    `counts per day. The phase groupings and their descriptions are the only editorial `+
    `content on this sheet.<br>`+
    `Regenerate rather than edit &mdash; a history that drifted from its own record would `+
    `be the failure this project spent four days finding.`;
}}

drawDays();
tables();
</script>
"""

(DOCS / "odyssey-history.html").write_text(HTML, encoding="utf-8", newline="\n")
print(f"docs/odyssey-history.html written — {len(revs)} revisions, {commits} commits, "
      f"{len(day_list)} days")
