"""Writes docs/odyssey-builds.html with the ten builds resolved by the real C
preprocessor -- not transcribed. Re-run this after changing any coupled constant."""
import sys
from pathlib import Path

#  Resolve every combination through the same preprocessor the checker uses,
#  rather than reading a cached dump. The sheet is only worth anything if its
#  numbers are the compiler's.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_consistency as C

NYQ, RATING = 0.80, 90.0
rows = []
for frame, motors in C.FRAME_MOTORS.items():
    for mc in motors:
        for pb in (2, 3):
            d = C.resolved_defines((f"-DFRAME_SIZE_IN={frame}",
                                    f"-DMOTOR_CLASS={mc}",
                                    f"-DPROP_BLADES={pb}",
                                    "-DACCEPT_CONNECTOR_OVER_RATING=1"))
            if not d:
                raise SystemExit("no C preprocessor available")
            r = {"frame": frame, "motor": mc.replace("MOTOR_", ""), "blades": pb}
            r.update(d)
            ceiling = min(d["IMU_DLPF_HZ"], d["FLIGHT_LOOP_HZ"] * 0.5 * NYQ)
            r["ceilingHz"] = ceiling
            r["h2Hz"] = 2 * d["NOTCH_CENTER_HZ"]
            r["h2Visible"] = r["h2Hz"] <= ceiling
            r["overRating"] = d["PROP_PEAK_PACK_A"] > RATING
            rows.append(r)

NUM_RULE = (
    'td.num{font-family:"IBM Plex Mono",monospace;'
    'font-variant-numeric:tabular-nums;text-align:right;white-space:nowrap}'
)

def js_rows():
    out = []
    for r in rows:
        out.append(
            "  {frame:%d, motor:\"%s\", blades:%d, wb:%d, prop:%d, kv:%d, loop:%d, "
            "dlpf:%d, notch:%.0f, bins:%d, auw:%.0f, peak:%.1f, cruise:%.2f, "
            "mah:%.0f, h2:%.0f, ceiling:%.0f, h2vis:%s, over:%s}" % (
                r["frame"], r["motor"], r["blades"], r["FRAME_WHEELBASE_MM"],
                r["PROP_DIAMETER_IN"], r["MOTOR_KV"], r["FLIGHT_LOOP_HZ"],
                r["IMU_DLPF_HZ"], r["NOTCH_CENTER_HZ"], r["DYN_NOTCH_BINS"],
                r["AIRFRAME_AUW_G"], r["PROP_PEAK_PACK_A"], r["CRUISE_CURRENT_A"],
                r["PACK_CAPACITY_MAH"], r["h2Hz"], r["ceilingHz"],
                "true" if r["h2Visible"] else "false",
                "true" if r["overRating"] else "false"))
    return ",\n".join(out)

CSS = Path("docs/odyssey-blackbox.html").read_text(encoding="utf-8")
style = CSS[CSS.index("<style>"):CSS.index("</style>") + len("</style>")]
style = style.replace("/* Fifth sheet, same system as the others. */",
                      "/* Sixth sheet, same system as the others. */")

#  The borrowed stylesheet has no rule for the numeric columns this sheet uses.
#  A matrix is read down its columns, so the digits have to line up.
style = style.replace(
    "td.note{color:var(--ink-2);font-size:13px}",
    "td.note{color:var(--ink-2);font-size:13px}" + chr(10) + NUM_RULE)

HTML = """<title>Odyssey-10 Pro Build Matrix</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">

""" + style + """

<div class="wrap">

  <header class="masthead">
    <div class="eyebrow">Odyssey-10 Pro &middot; Build matrix &middot; Sheet 6 of 6</div>
    <h1>Three Switches, Ten Aircraft</h1>
    <p class="standfirst">
      Frame size, motor class and blade count. Everything else &mdash; loop rate, filter
      corner, notch centre, mass, thrust, peak current &mdash; follows from those three,
      which is why they are compile-time switches rather than sets of edits you make by
      hand and hope you made consistently.
    </p>
    <div class="provenance" id="provenance"></div>
  </header>

  <section>
    <h2>What one switch moves <span>the coupling, not the values</span></h2>
    <p class="lede">
      Set <code>FRAME_SIZE_IN</code> and eight other constants move with it. That is the
      whole argument for the switch: these numbers are not independent, and a build where
      the notch matches the old propeller is not a build that flies badly &mdash; it is a
      build that flies badly for a reason nobody can see.
    </p>
    <div class="sheet">
      <svg id="cascade" role="img" aria-label="How the three build switches cascade into the derived constants"></svg>
    </div>
  </section>

  <section>
    <h2>The ten builds <span id="mspan"></span></h2>
    <p class="lede">
      Resolved by running the C preprocessor over <code>config.h</code> once per
      combination &mdash; the same way <code>tools/check_consistency.py</code> does it, so
      these are the constants the compiler would actually see.
    </p>
    <div class="scroll"><table id="matrix">
      <caption>A 2807 on a 10-inch is under-stator&rsquo;d and a 3115 on a 7-inch is dead weight, so neither pairing is offered.</caption>
      <thead><tr>
        <th>Build</th><th>Wheel&shy;base</th><th>Loop</th><th>DLPF</th><th>Notch</th>
        <th>Bins</th><th>AUW</th><th>Cruise</th><th>Peak</th><th>2&middot;f0</th>
      </tr></thead>
      <tbody></tbody>
    </table></div>
    <div class="legend" id="legend"></div>
  </section>

  <section>
    <h2>Three builds exceed the connector <span>XT90-S, 90 A continuous</span></h2>
    <div class="scroll"><table id="over">
      <thead><tr><th>Build</th><th>Peak pack current</th><th>Over by</th><th>Required</th></tr></thead>
      <tbody></tbody>
    </table></div>
    <div class="callout">
      <h3>A burst rating, not a cruise rating</h3>
      <p>
        <code>PROP_PEAK_PACK_A</code> is a full-throttle burst, not a number the aircraft
        sits at &mdash; cruise is a third of it. But the XT90-S is the single connector
        every amp passes through, and a burst is exactly when you find out. These builds
        need <code>-DACCEPT_CONNECTOR_OVER_RATING=1</code>, which is a deliberate
        acknowledgement rather than a silencer, or a bigger connector.
        <br><br>
        All three are <b>3-blade</b>. A third blade buys thrust and costs current, and
        this is where that bill arrives.
      </p>
    </div>
  </section>

  <section>
    <h2>Only two builds can see their harmonic <span>2 of 10</span></h2>
    <div class="scroll"><table id="harm">
      <thead><tr><th>Build</th><th>2&middot;f0</th><th>Ceiling</th><th>Set by</th><th>Visible?</th></tr></thead>
      <tbody></tbody>
    </table></div>
    <div class="callout">
      <h3>The ceiling is the IMU&rsquo;s anti-alias corner, not the loop rate</h3>
      <p>
        The harmonic tracker searches up to <code>min(IMU_DLPF_HZ, 0.8 &times; Nyquist)</code>.
        On the 500&nbsp;Hz builds Nyquist gives 200&nbsp;Hz but the DLPF corner is
        184&nbsp;Hz, so <b>the filter decides, not the loop</b>. Only the 10-inch 3-blade
        builds put 2&middot;f0 at 180&nbsp;Hz &mdash; just under it.
        <br><br>
        This is why the tracker reports observability as its own flag rather than
        inferring it: on eight of ten builds a harmonic search would be looking above the
        corner, where there is nothing but the filter&rsquo;s own roll-off. Notching an
        artefact costs real phase lag.
      </p>
    </div>
  </section>

  <footer id="titleblock"></footer>
</div>

<script>
const DOC_REV = "5.1";
const CONNECTOR_RATING_A = 90.0;   /* XT90-S continuous, from config.h */

/* Resolved by the C preprocessor, one pass per combination. Regenerate with the
   script in tools/ rather than editing by hand. */
const BUILDS = [
""" + js_rows() + """
];

const CLASSES = {
  f7:{label:"7-inch", colour:"var(--c-uart)"},
  f9:{label:"9-inch", colour:"var(--c-pwm)"},
  f10:{label:"10-inch", colour:"var(--copper)"}
};
const col = (b)=>CLASSES["f"+b.frame].colour;
const name = (b)=>`${b.frame}" ${b.motor} ${b.blades}b`;

const $=(s)=>document.querySelector(s);
const SVGNS="http://www.w3.org/2000/svg";
const el=(n,a={})=>{const e=document.createElementNS(SVGNS,n);for(const k in a)e.setAttribute(k,a[k]);return e;};
const esc=(s)=>String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;");

/* ---- the cascade ------------------------------------------------------
   Three switches on the left, what each one moves on the right. Drawn as a
   fan rather than a tree, because several constants are moved by more than
   one switch and a tree would have to pick one parent and lie.            */
const SWITCHES = [
  {n:"FRAME_SIZE_IN", v:"7 · 9 · 10", cls:"f9", moves:[
    "FRAME_WHEELBASE_MM","PROP_DIAMETER_IN","FRAME_BASE_DRY_G","BATTERY_MASS_G",
    "PACK_CAPACITY_MAH","FLIGHT_LOOP_HZ","IMU_DLPF_HZ","DYN_NOTCH_BINS"]},
  {n:"MOTOR_CLASS", v:"2807 · 2810 · 3110 · 3115", cls:"f10", moves:[
    "MOTOR_KV","MOTOR_MASS_G_EACH","MOTOR_THRUST_G_EACH","MOTOR_STATOR_MM"]},
  {n:"PROP_BLADES", v:"2 · 3", cls:"f7", moves:[
    "PROP_NOTCH_DEFAULT_HZ","PROP_MASS_G_EACH","PROP_BASE_PEAK_A","CRUISE_CURRENT_A"]}
];

function drawCascade(){
  const svg=$("#cascade");
  const PAD=32,SW=232,SH=62,GAP=30,CX=PAD+SW+150,CW=214;
  const rowsMax=Math.max(...SWITCHES.map(s=>s.moves.length));
  const W=CX+CW+PAD, H=PAD*2+SWITCHES.length*(SH+GAP*3.6);
  svg.setAttribute("viewBox",`0 0 ${W} ${H}`);
  svg.setAttribute("font-family",'"IBM Plex Mono",ui-monospace,monospace');
  const t=(x,y,s,o={})=>{const n=el("text",Object.assign({x,y,"font-size":11,
    fill:"var(--ink)","dominant-baseline":"middle"},o));n.textContent=s;svg.appendChild(n);return n;};

  SWITCHES.forEach((s,i)=>{
    const y=PAD+i*(SH+GAP*3.6), cy=y+SH/2, colour=CLASSES[s.cls].colour;
    svg.appendChild(el("rect",{x:PAD,y,width:SW,height:SH,rx:2,
      fill:"var(--paper)",stroke:colour,"stroke-width":1.8}));
    svg.appendChild(el("rect",{x:PAD,y,width:3,height:SH,fill:colour}));
    t(PAD+14,y+24,s.n,{"font-size":12.5,"font-weight":600,
      "font-family":"Archivo, sans-serif"});
    t(PAD+14,y+42,s.v,{"font-size":9.5,fill:"var(--ink-3)"});

    /* one leader per moved constant, fanning to a stacked column */
    s.moves.forEach((m,j)=>{
      const my=y+14+j*18;
      svg.appendChild(el("path",{
        d:`M ${PAD+SW} ${cy} C ${PAD+SW+60} ${cy}, ${CX-70} ${my}, ${CX-10} ${my}`,
        stroke:colour,"stroke-width":1,fill:"none",opacity:.42}));
      svg.appendChild(el("circle",{cx:CX-6,cy:my,r:2.6,fill:colour,opacity:.8}));
      t(CX+6,my,m,{"font-size":9.5,fill:"var(--ink-2)"});
    });
    t(CX+6,y+14+s.moves.length*18+4,`${s.moves.length} constants`,
      {"font-size":8.5,fill:colour,opacity:.85});
  });
}

function tables(){
  $("#mspan").textContent=`${BUILDS.length} combinations · resolved by cpp`;
  $("#legend").innerHTML=Object.values(CLASSES).map(c=>
    `<span><i style="background:${c.colour}"></i>${esc(c.label)}</span>`).join("");

  $("#matrix tbody").innerHTML=BUILDS.map(b=>`<tr>
      <td class="off" style="--cls:${col(b)};text-align:left">${esc(name(b))}</td>
      <td class="num">${b.wb} mm</td>
      <td class="num">${b.loop} Hz</td>
      <td class="num">${b.dlpf} Hz</td>
      <td class="num" style="color:${col(b)};font-weight:600">${b.notch} Hz</td>
      <td class="num">${b.bins}</td>
      <td class="num">${b.auw} g</td>
      <td class="num">${b.cruise.toFixed(1)} A</td>
      <td class="num" style="color:${b.over?"var(--c-power)":"inherit"};font-weight:${b.over?600:400}">${b.peak.toFixed(0)} A</td>
      <td class="num" style="color:${b.h2vis?"var(--c-pwm)":"var(--ink-3)"}">${b.h2vis?b.h2+" Hz":"—"}</td>
    </tr>`).join("");

  const over=BUILDS.filter(b=>b.over);
  $("#over tbody").innerHTML=over.map(b=>`<tr>
      <td class="off" style="--cls:var(--c-power);text-align:left">${esc(name(b))}</td>
      <td class="num" style="color:var(--c-power);font-weight:600">${b.peak.toFixed(1)} A</td>
      <td class="num">+${(b.peak-CONNECTOR_RATING_A).toFixed(1)} A</td>
      <td class="note"><code>-DACCEPT_CONNECTOR_OVER_RATING=1</code>, or a bigger connector</td>
    </tr>`).join("");

  $("#harm tbody").innerHTML=BUILDS.map(b=>`<tr>
      <td class="off" style="--cls:${b.h2vis?"var(--c-pwm)":"var(--ink-3)"};text-align:left">${esc(name(b))}</td>
      <td class="num">${b.h2} Hz</td>
      <td class="num">${b.ceiling} Hz</td>
      <td class="note">${b.dlpf<=b.loop*0.4?"IMU anti-alias corner":"Nyquist margin"}</td>
      <td><span class="pill" style="color:${b.h2vis?"var(--c-pwm)":"var(--ink-3)"}">${b.h2vis?"visible":"above the ceiling"}</span></td>
    </tr>`).join("");

  const vis=BUILDS.filter(b=>b.h2vis).length;
  $("#provenance").innerHTML=[
    `<b>Builds</b> ${BUILDS.length}`,`<span class="dot">/</span>`,
    `<b>Source</b> cpp over config.h`,`<span class="dot">/</span>`,
    `<b>Over connector</b> ${over.length}`,`<span class="dot">/</span>`,
    `<b>Harmonic visible</b> ${vis}`,`<span class="dot">/</span>`,
    `<b>Doc</b> rev ${DOC_REV}`
  ].join(" ");

  $("#titleblock").innerHTML=
    `Odyssey-10 Pro &middot; build matrix, revision ${DOC_REV}<br>`+
    `Every number resolved by running the C preprocessor over <code>config.h</code> once `+
    `per combination, the same way <code>tools/check_consistency.py</code> resolves them `+
    `&mdash; so these are the constants the compiler sees, not a transcription of them.<br>`+
    `<code>CRUISE_CURRENT_A</code> is still modelled rather than measured, on every row. `+
    `It sizes the return-to-home reserve, and the thrust stand in §4.3.2 step 3 is what settles it.`;
}

drawCascade();
tables();
</script>
"""

Path("docs/odyssey-builds.html").write_text(HTML, encoding="utf-8", newline="\n")
print(f"docs/odyssey-builds.html written with {len(rows)} preprocessor-resolved builds")
