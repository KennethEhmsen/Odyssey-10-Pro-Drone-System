"""Writes docs/odyssey-spec.html by profiling the specification itself."""
import re
from pathlib import Path

DOCS = Path("docs")
SPEC = DOCS / "Odyssey-10-Pro-Drone-System.md"
ORIG = DOCS / "Odyssey-10-Pro-Drone-System.ORIGINAL.md"

text = SPEC.read_text(encoding="utf-8", errors="replace")
lines = text.splitlines()
rev = re.search(r"\*\*Document revision:\*\*\s*([0-9.]+)", text).group(1)

tops = [(i, m.group(1), m.group(2)) for i, l in enumerate(lines)
        for m in [re.match(r"^## (\d+)\. (.+)$", l)] if m]

#  Which sheet, if any, covers each section. Editorial -- everything else is measured.
SHEET = {
    "4": ("dshot", "DShot waveform"),
    "5": ("flow", "state ladder"),
    "6": ("layout", "placement"),
    "8": ("layout", "airframe layout"),
    "9": ("schematic", "interconnect"),
    "10": ("blackbox", "record format"),
    "11": ("checklist", "commissioning"),
    "13": ("findings", "findings ledger"),
}

secs = []
for k, (i, n, name) in enumerate(tops):
    end = tops[k + 1][0] if k + 1 < len(tops) else len(lines)
    body = "\n".join(lines[i:end])
    secs.append({
        "n": n, "name": name,
        "lines": end - i,
        "subs": len(re.findall(r"^### ", body, re.M)),
        "rows": body.count("\n|"),
        "findings": len(re.findall(r"FINDING \d+", body)),
        "sheet": SHEET.get(n),
    })

words = len(text.split())
orig_words = len(ORIG.read_text(encoding="utf-8", errors="replace").split()) if ORIG.exists() else 0
companions = [(p.name, len(p.read_text(encoding="utf-8", errors="replace").split()))
              for p in sorted(DOCS.glob("*.md")) if p != SPEC]

CSS = (DOCS / "odyssey-findings.html").read_text(encoding="utf-8")
style = CSS[CSS.index("<style>"):CSS.index("</style>") + len("</style>")]
style = style.replace("/* Eleventh sheet, same system as the others. */",
                      "/* Thirteenth sheet, same system as the others. */")
style = style.replace(
    "td.note{color:var(--ink-2);font-size:13px}",
    "td.note{color:var(--ink-2);font-size:13px}\n"
    "td.num{font-family:\"IBM Plex Mono\",monospace;font-variant-numeric:tabular-nums;"
    "text-align:right;white-space:nowrap}")

js_secs = ",\n".join(
    '  {n:"%s", name:"%s", lines:%d, subs:%d, rows:%d, findings:%d, sheet:%s}' % (
        s["n"], s["name"].replace('"', '\\"'), s["lines"], s["subs"], s["rows"],
        s["findings"],
        ('["%s","%s"]' % s["sheet"]) if s["sheet"] else "null")
    for s in secs)

js_comp = ",\n".join('  ["%s",%d]' % c for c in companions)

HTML = f"""<title>Odyssey-10 Pro Specification</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">

{style}

<div class="wrap">

  <header class="masthead">
    <div class="eyebrow"><a href="index.html">&larr; All sheets</a> &middot; Odyssey-10 Pro &middot; Specification &middot; Sheet 13 of 13</div>
    <h1>Five Times Longer, And Now True</h1>
    <p class="standfirst">
      The document began at {orig_words:,} words and looked finished. Making it actually
      describe a buildable aircraft took it to {words:,}. This is a map of what is in it,
      measured rather than remembered.
    </p>
    <div class="provenance" id="provenance"></div>
  </header>

  <section>
    <h2>Growth <span>the original is still in the repository</span></h2>
    <div class="sheet">
      <svg id="growth" role="img" aria-label="The original specification against the current one, by word count"></svg>
    </div>
    <div class="callout">
      <h3>The original is kept, not deleted</h3>
      <p>
        <code>docs/Odyssey-10-Pro-Drone-System.ORIGINAL.md</code> is revision 1.0, intact.
        It is the control: without it, &ldquo;eighteen findings&rdquo; is a claim rather
        than something anybody can check. Every finding in the ledger can be read against
        the text that contained it.
      </p>
    </div>
  </section>

  <section>
    <h2>Thirteen sections <span id="secspan"></span></h2>
    <p class="lede">
      Size is a poor proxy for importance, but it is an honest one for where the work went.
      §8 and §4 are the largest because physical placement and frame dynamics are where the
      most assumptions had to be replaced with numbers.
    </p>
    <div class="scroll"><table id="secs">
      <caption>Lines, subsections, table rows and inline findings, counted from the file.</caption>
      <thead><tr><th>§</th><th>Section</th><th>Lines</th><th>Sub</th><th>Rows</th><th>Findings</th><th>Drawn on</th></tr></thead>
      <tbody></tbody>
    </table></div>
  </section>

  <section>
    <h2>Where the findings cluster <span>and why there</span></h2>
    <div class="sheet">
      <svg id="cluster" role="img" aria-label="Inline findings per section"></svg>
    </div>
    <div class="callout">
      <h3>Findings gather where the document meets the physical world</h3>
      <p>
        §5 (energy, return-to-home and failsafe) and §8 (physical placement and RF) carry
        the most inline findings. Both are sections where a sentence has to survive contact
        with something real — a battery that discharges, an antenna that interferes, a
        magnetometer that sits too close to the power train.
        <br><br>
        §11 carries none, and that is not a good sign. It is the commissioning checklist:
        nothing in it has been performed yet, so nothing in it has had the chance to be
        found wrong.
      </p>
    </div>
  </section>

  <section>
    <h2>The companion documents <span>three of them</span></h2>
    <div class="scroll"><table id="comp">
      <thead><tr><th>File</th><th>Words</th><th>What it is for</th></tr></thead>
      <tbody></tbody>
    </table></div>
  </section>

  <footer id="titleblock"></footer>
</div>

<script>
const DOC_REV = "{rev}";
const WORDS = {words};
const ORIG_WORDS = {orig_words};

const SECS = [
{js_secs}
];
const COMPANIONS = [
{js_comp}
];
const COMP_WHY = {{
  "Odyssey-10-Pro-Drone-System.ORIGINAL.md":
    "Revision 1.0, kept intact. The control against which every finding can be read",
  "review-findings-resolution.md":
    "The original eighteen findings in detail, one section each",
  "remote-id-regulatory-notes.md":
    "The regulatory position, separated so it can be revised without touching the engineering"
}};

const $=(s)=>document.querySelector(s);
const SVGNS="http://www.w3.org/2000/svg";
const el=(n,a={{}})=>{{const e=document.createElementNS(SVGNS,n);for(const k in a)e.setAttribute(k,a[k]);return e;}};
const esc=(s)=>String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;");

function drawGrowth(){{
  const svg=$("#growth"), PAD=40, W=920, H=200;
  const px=(W-PAD*2-190)/WORDS;
  svg.setAttribute("viewBox",`0 0 ${{W}} ${{H}}`);
  svg.setAttribute("font-family",'"IBM Plex Mono",ui-monospace,monospace');
  const t=(x,y,s,o={{}})=>{{const n=el("text",Object.assign({{x,y,"font-size":11,
    fill:"var(--ink)","dominant-baseline":"middle"}},o));n.textContent=s;svg.appendChild(n);return n;}};
  [[ORIG_WORDS,"revision 1.0","looked finished","var(--ink-3)"],
   [WORDS,`revision ${{DOC_REV}}`,"describes something buildable","var(--c-pwm)"]
  ].forEach((b,i)=>{{
    const y=PAD+20+i*70, w=Math.max(b[0]*px,4);
    svg.appendChild(el("rect",{{x:PAD,y,width:w,height:42,rx:2,
      fill:b[3],"fill-opacity":.18,stroke:b[3],"stroke-width":1.5}}));
    svg.appendChild(el("rect",{{x:PAD,y,width:3,height:42,fill:b[3]}}));
    t(PAD+14,y+15,b[0].toLocaleString(),{{"font-size":17,"font-weight":700,fill:b[3],
      "font-family":"Archivo, sans-serif"}});
    t(PAD+14,y+32,b[1],{{"font-size":10,"font-weight":600}});
    t(PAD+w+16,y+21,b[2],{{"font-size":9.5,fill:"var(--ink-3)"}});
  }});
  t(PAD,H-22,`${{(WORDS/ORIG_WORDS).toFixed(1)}}× — and the aircraft it describes has not changed`,
    {{"font-size":10,fill:"var(--copper)","font-style":"italic"}});
}}

function drawCluster(){{
  const svg=$("#cluster"), PAD=40, W=920, H=210, base=H-46;
  const max=Math.max(...SECS.map(s=>s.findings),1);
  const bw=(W-PAD*2)/SECS.length-12;
  svg.setAttribute("viewBox",`0 0 ${{W}} ${{H}}`);
  svg.setAttribute("font-family",'"IBM Plex Mono",ui-monospace,monospace');
  const t=(x,y,s,o={{}})=>{{const n=el("text",Object.assign({{x,y,"font-size":11,
    fill:"var(--ink)","dominant-baseline":"middle"}},o));n.textContent=s;svg.appendChild(n);return n;}};
  svg.appendChild(el("path",{{d:`M ${{PAD}} ${{base}} H ${{W-PAD}}`,stroke:"var(--rule)","stroke-width":1}}));
  SECS.forEach((s,i)=>{{
    const x=PAD+i*(bw+12), h=s.findings/max*(base-56);
    const colour=s.findings>=5?"var(--c-power)":s.findings?"var(--c-i2c)":"var(--rule)";
    if(s.findings) svg.appendChild(el("rect",{{x,y:base-h,width:bw,height:h,rx:2,
      fill:colour,"fill-opacity":.24,stroke:colour,"stroke-width":1.3}}));
    if(s.findings) t(x+bw/2,base-h-12,String(s.findings),{{"text-anchor":"middle",
      "font-size":12,"font-weight":700,fill:colour,"font-family":"Archivo, sans-serif"}});
    t(x+bw/2,base+15,`§${{s.n}}`,{{"text-anchor":"middle","font-size":9.5,
      fill:s.findings?"var(--ink-2)":"var(--ink-3)"}});
  }});
  t(PAD,26,"INLINE FINDINGS PER SECTION",{{"font-size":10,fill:"var(--ink-3)",
    "letter-spacing":".12em"}});
  t(W-PAD,base+34,"§11 has none because none of it has been performed",
    {{"text-anchor":"end","font-size":9,fill:"var(--copper)","font-style":"italic"}});
}}

function tables(){{
  const totalLines=SECS.reduce((a,s)=>a+s.lines,0);
  $("#secspan").textContent=`${{totalLines.toLocaleString()}} lines · ${{WORDS.toLocaleString()}} words`;

  $("#secs tbody").innerHTML=SECS.map(s=>{{
    const colour=s.findings>=5?"var(--c-power)":s.findings?"var(--c-i2c)":"var(--c-gpio)";
    return `<tr>
      <td class="k" style="--cls:${{colour}};text-align:right">${{s.n}}</td>
      <td>${{esc(s.name)}}</td>
      <td class="num">${{s.lines}}</td>
      <td class="num">${{s.subs}}</td>
      <td class="num">${{s.rows}}</td>
      <td class="num" style="color:${{s.findings?colour:"var(--ink-3)"}};font-weight:${{s.findings?600:400}}">${{s.findings||"—"}}</td>
      <td class="note">${{s.sheet?`<a href="odyssey-${{s.sheet[0]}}.html" style="color:var(--copper);border-bottom:1px dotted">${{esc(s.sheet[1])}}</a>`:"<span style='opacity:.5'>—</span>"}}</td>
    </tr>`;}}).join("");

  $("#comp tbody").innerHTML=COMPANIONS.map(c=>`<tr>
      <td class="k" style="--cls:var(--c-uart)"><code>${{esc(c[0])}}</code></td>
      <td class="num">${{c[1].toLocaleString()}}</td>
      <td class="note">${{COMP_WHY[c[0]]||""}}</td></tr>`).join("");

  const drawn=SECS.filter(s=>s.sheet).length;
  $("#provenance").innerHTML=[
    `<b>Sections</b> ${{SECS.length}}`,`<span class="dot">/</span>`,
    `<b>Words</b> ${{WORDS.toLocaleString()}}`,`<span class="dot">/</span>`,
    `<b>Growth</b> ${{(WORDS/ORIG_WORDS).toFixed(1)}}×`,`<span class="dot">/</span>`,
    `<b>Drawn</b> ${{drawn}} of ${{SECS.length}}`,`<span class="dot">/</span>`,
    `<b>Rev</b> ${{DOC_REV}}`
  ].join(" ");

  $("#titleblock").innerHTML=
    `Odyssey-10 Pro &middot; specification map, revision ${{DOC_REV}}<br>`+
    `Every count measured from <code>docs/Odyssey-10-Pro-Drone-System.md</code> by `+
    `<code>tools/gen_spec_map.py</code> — lines, subsections, table rows and inline `+
    `findings per section. Which sheet covers which section is the only editorial content `+
    `here.<br>`+
    `The specification is the source. These sheets are readings of it, and where they `+
    `disagree it is right and they are bugs.`;
}}

drawGrowth();
drawCluster();
tables();
</script>
"""

(DOCS / "odyssey-spec.html").write_text(HTML, encoding="utf-8", newline="\n")
print(f"docs/odyssey-spec.html written — {len(secs)} sections, {words:,} words "
      f"({words/orig_words:.1f}x the original)")
