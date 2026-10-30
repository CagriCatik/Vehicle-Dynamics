// Vehicle-Dynamics workbench front-end . Plain JS, no build step.
// All configuration logic stays in the Python experiment engine: the form only assembles the YAML mapping of a
// controller comparison; validation, saving and running go through the backend (the CLI's own parser).
"use strict";

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const PALETTE = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#8a63d2", "#7d8590"];
const WHEELS = ["FL", "FR", "RL", "RR"];
const DEFAULTS = { mu: 1.0, mu_left: 1.0, mu_right: 0.3, speed_kph: 80, steer_deg: 2, duration_s: 3, amplitude_deg: 6 };
const LABELS = { mu: "µ", mu_left: "µ left", mu_right: "µ right", speed_kph: "initial speed [km/h]", steer_deg: "steer [deg]",
  duration_s: "duration [s]", amplitude_deg: "amplitude [deg]" };
let OPT = null, current = null, polling = null;

async function api(path, opts = {}) {
  const r = await fetch(path, { headers: { "Content-Type": "application/json" }, ...opts });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.detail || r.statusText);
  return j;
}
const post = (p, body) => api(p, { method: "POST", body: JSON.stringify(body) });
const fmt = (x) => (x === null || x === undefined || x === "") ? "–" : (typeof x === "number" ? (Number.isInteger(x) ? String(x) : x.toPrecision(4)) : String(x));
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
function status(el, text, ok) { el.textContent = text; el.className = "status " + (ok === undefined ? "" : ok ? "ok" : "bad"); }

// ------------------------------------------------------------------ tabs
$$("#tabs button").forEach((b) => b.onclick = () => showTab(b.dataset.tab));
function showTab(t) {
  $$("#tabs button").forEach((b) => b.classList.toggle("active", b.dataset.tab === t));
  $$(".tab").forEach((s) => s.classList.toggle("active", s.id === t));
  if (t === "runs") refreshJobs();
  if (t === "results") loadResults();
  if (t === "compare") loadCompareList();
}

// ------------------------------------------------------------------ configure
function fillSelect(sel, values, keep) {
  const cur = keep ? [...sel.options].filter((o) => o.value === "") : [];
  sel.innerHTML = "";
  cur.forEach((o) => sel.appendChild(o));
  values.forEach((v) => { const o = document.createElement("option"); o.value = o.textContent = v; sel.appendChild(o); });
}
function paramInputs(div, keys, values = {}) {
  div.innerHTML = keys.map((k) => `<label>${LABELS[k] || k}<input data-key="${k}" type="number" step="any" value="${values[k] ?? DEFAULTS[k] ?? ""}"></label>`).join("");
  div.oninput = buildYaml;
}
function ctrlRow(c = { law: "pi", inputs: "tv", feedback: "ideal", actuators: "ideal" }) {
  const d = document.createElement("div");
  d.className = "ctrl-row";
  const sel = (name, vals, v) => `<label>${name}<select data-k="${name}">${vals.map((x) => `<option ${x === v ? "selected" : ""}>${x}</option>`).join("")}</select></label>`;
  d.innerHTML = sel("law", OPT.laws, c.law) + sel("inputs", OPT.inputs, c.inputs || "tv") + sel("feedback", OPT.feedback, c.feedback || "ideal") + sel("actuators", OPT.actuators, c.actuators || "ideal")
    + `<button type="button" class="small">remove</button>`;
  d.querySelector("button").onclick = () => { d.remove(); buildYaml(); };
  d.oninput = buildYaml;
  $("#controllers").appendChild(d);
}
function formData() {
  const f = $("#form");
  const v = (n) => f.elements[n].value;
  const nums = (div) => Object.fromEntries($$("input", div).map((i) => [i.dataset.key, Number(i.value)]));
  const d = { type: "controller_comparison", name: v("name"), vehicle: v("vehicle"),
    tyres: { front: v("tyre_front"), rear: v("tyre_rear") },
    road: { type: v("road_type"), ...nums($("#road-params")) },
    manoeuvre: { type: v("man_type"), ...nums($("#man-params")) },
    controllers: $$(".ctrl-row").map((r) => Object.fromEntries($$("select", r).map((s) => [s.dataset.k, s.value]))),
    dt: Number(v("dt")), seed: Number(v("seed")),
    outputs: { formats: $$("#formats input:checked").map((i) => i.value) } };
  if (v("description")) d.description = v("description");
  if (v("kc")) d.kc = v("kc");
  d.controllers.forEach((c) => { if (c.inputs === "tv") delete c.inputs; if (c.feedback === "ideal") delete c.feedback;
    if (c.actuators === "ideal") delete c.actuators; });
  if (v("sweep_param") && v("sweep_values").trim())
    d.sweep = { parameter: v("sweep_param"), values: v("sweep_values").split(",").map((x) => Number(x.trim())).filter((x) => !Number.isNaN(x)) };
  return d;
}
let yamlTimer = null;
function buildYaml() {
  if ($("#raw-mode").checked) return;
  clearTimeout(yamlTimer);
  yamlTimer = setTimeout(async () => {
    const r = await post("/api/validate", formData());
    $("#yaml").value = r.yaml || JSON.stringify(formData(), null, 2);
    status($("#status"), r.valid ? `valid: ${r.description}` : r.error, r.valid);
  }, 250);
}
function setForm(d) {
  const f = $("#form");
  f.elements.name.value = d.name || "experiment";
  f.elements.description.value = d.description || "";
  f.elements.vehicle.value = d.vehicle;
  f.elements.tyre_front.value = (d.tyres || {}).front || OPT.tyres[0];
  f.elements.tyre_rear.value = (d.tyres || {}).rear || (d.tyres || {}).front || OPT.tyres[0];
  f.elements.kc.value = d.kc || "";
  f.elements.road_type.value = d.road.type;
  paramInputs($("#road-params"), OPT.roads[d.road.type], d.road);
  f.elements.man_type.value = d.manoeuvre.type;
  paramInputs($("#man-params"), OPT.manoeuvres[d.manoeuvre.type], d.manoeuvre);
  $("#controllers").innerHTML = "";
  (d.controllers || []).forEach(ctrlRow);
  f.elements.sweep_param.value = d.sweep ? d.sweep.parameter : "";
  f.elements.sweep_values.value = d.sweep ? d.sweep.values.join(", ") : "";
  f.elements.dt.value = d.dt ?? 0.005;
  f.elements.seed.value = d.seed ?? 0;
  const fm = (d.outputs || {}).formats || OPT.default_formats;
  $$("#formats input").forEach((i) => i.checked = fm.includes(i.value));
  buildYaml();
}
async function initConfigure() {
  OPT = await api("/api/options");
  const f = $("#form");
  fillSelect(f.elements.vehicle, OPT.vehicles);
  f.elements.vehicle.value = OPT.vehicles.find((v) => v.includes("seven_dof")) || OPT.vehicles[0];
  fillSelect(f.elements.tyre_front, OPT.tyres); fillSelect(f.elements.tyre_rear, OPT.tyres);
  const mf = OPT.tyres.find((t) => t.startsWith("magic_formula")) || OPT.tyres[0];
  f.elements.tyre_front.value = f.elements.tyre_rear.value = mf;
  fillSelect(f.elements.kc, OPT.kc, true);
  fillSelect(f.elements.road_type, Object.keys(OPT.roads));
  fillSelect(f.elements.man_type, Object.keys(OPT.manoeuvres));
  fillSelect(f.elements.sweep_param, OPT.sweep_parameters, true);
  $("#formats").innerHTML = OPT.formats.map((x) => `<label class="inline"><input type="checkbox" value="${x}" ${OPT.default_formats.includes(x) ? "checked" : ""}>${x}</label>`).join("");
  f.elements.road_type.onchange = () => { paramInputs($("#road-params"), OPT.roads[f.elements.road_type.value]); buildYaml(); };
  f.elements.man_type.onchange = () => { paramInputs($("#man-params"), OPT.manoeuvres[f.elements.man_type.value]); buildYaml(); };
  f.oninput = buildYaml;
  $("#add-ctrl").onclick = () => { ctrlRow(); buildYaml(); };
  setForm({ name: "workbench_experiment", vehicle: f.elements.vehicle.value, tyres: { front: mf, rear: mf },
    road: { type: "uniform", mu: 1.0 }, manoeuvre: { type: "sine_with_dwell", speed_kph: 80, amplitude_deg: 6 },
    controllers: [{ law: "passive" }, { law: "pi" }], dt: 0.005, seed: 0 });
  await loadConfigList();
  $("#cfg-load").onclick = loadConfig;
  $("#raw-mode").onchange = () => { $("#yaml").readOnly = !$("#raw-mode").checked; $("#form").style.opacity = $("#raw-mode").checked ? 0.4 : 1; if (!$("#raw-mode").checked) buildYaml(); };
  $("#yaml").oninput = () => { if ($("#raw-mode").checked) { clearTimeout(yamlTimer); yamlTimer = setTimeout(validateRaw, 400); } };
  $("#validate").onclick = () => $("#raw-mode").checked ? validateRaw() : buildYaml();
  $("#save").onclick = () => save(false);
  $("#save-run").onclick = () => save(true);
}
async function validateRaw() {
  const r = await post("/api/validate_yaml", { text: $("#yaml").value });
  status($("#status"), r.valid ? `valid: [${r.kind}] ${r.description}` : r.error, r.valid);
}
async function loadConfigList() {
  const list = await api("/api/configs");
  const sel = $("#cfg-list");
  sel.innerHTML = `<option value="">– new controller comparison –</option>` + list.map((c) =>
    `<option value="${esc(c.path)}">${c.saved ? "★ " : ""}${esc(c.path)} [${esc(c.kind)}]</option>`).join("");
}
async function loadConfig() {
  const path = $("#cfg-list").value;
  if (!path) return;
  const c = await api("/api/config?path=" + encodeURIComponent(path));
  const kind = c.data.type || "controller_comparison";
  if (kind === "controller_comparison") {
    $("#raw-mode").checked = false; $("#raw-mode").onchange(); setForm(c.data);
  } else {
    $("#raw-mode").checked = true; $("#raw-mode").onchange(); $("#yaml").value = c.yaml; validateRaw();
  }
}
async function save(run) {
  try {
    const r = $("#raw-mode").checked ? await post("/api/configs_yaml", { text: $("#yaml").value }) : await post("/api/configs", formData());
    status($("#status"), `saved ${r.path}`, true);
    await loadConfigList();
    if (run) {
      const j = await post("/api/jobs", { config: r.path });
      status($("#status"), `saved ${r.path}; started run ${j.id}\n${j.command_line}`, true);
      showTab("runs");
    }
  } catch (e) { status($("#status"), e.message, false); }
}

// ------------------------------------------------------------------ runs
let logJob = null;
async function refreshJobs() {
  const jobs = await api("/api/jobs");
  const running = jobs.filter((j) => j.status === "running").length;
  $("#running-badge").textContent = running ? `(${running})` : "";
  $("#jobs tbody").innerHTML = jobs.map((j) => {
    const p = j.progress, pct = p && p.total ? Math.round(100 * p.done / p.total) : null;
    return `<tr data-id="${esc(j.id)}"><td>${esc(j.id)}</td><td>${esc(j.kind)}</td><td><span class="pill ${j.status}">${j.status}</span></td>
      <td>${pct === null ? (j.status === "running" ? "no progress reported" : "") : `<span class="bar"><i style="width:${pct}%"></i></span> ${p.done}/${p.total} ${esc(p.label || "")}`}</td>
      <td class="num">${j.elapsed_s} s</td><td><code>${esc(j.command_line)}</code></td>
      <td>${j.status === "running" ? `<button class="small" data-cancel="${esc(j.id)}">cancel</button>` : ""}
          ${j.has_results ? `<button class="small" data-open="${esc(j.out)}">open</button>` : ""}
          <button class="small" data-log="${esc(j.id)}">log</button></td></tr>`;
  }).join("");
  $$("[data-cancel]").forEach((b) => b.onclick = async () => { await post(`/api/jobs/${b.dataset.cancel}/cancel`, {}); refreshJobs(); });
  $$("[data-open]").forEach((b) => b.onclick = () => { showTab("results"); openResult(b.dataset.open); });
  $$("[data-log]").forEach((b) => b.onclick = () => { logJob = b.dataset.log; showLog(); });
  if (!logJob && jobs.length) logJob = jobs[0].id;
  showLog(jobs);
  clearTimeout(polling);
  if (running) polling = setTimeout(refreshJobs, 1200);
}
async function showLog(jobs) {
  if (!logJob) return;
  const j = (jobs || []).find((x) => x.id === logJob) || await api(`/api/jobs/${logJob}`);
  $("#log-id").textContent = j.id;
  $("#log").textContent = j.log_tail || "(empty)";
}

// ------------------------------------------------------------------ results
async function loadResults() {
  const list = await api("/api/results");
  $("#result-list").innerHTML = list.map((r) => `<li data-id="${esc(r.id)}"><b>${esc(r.name)}</b><small>${esc(r.kind)} · ${esc(r.source)} · ${esc(r.created)}</small></li>`).join("");
  $$("#result-list li").forEach((li) => li.onclick = () => openResult(li.dataset.id));
}
async function openResult(id) {
  $$("#result-list li").forEach((li) => li.classList.toggle("sel", li.dataset.id === id));
  const r = await api("/api/result?id=" + encodeURIComponent(id));
  current = r;
  const v = $("#result-view");
  const ver = r.verification.length ? `${r.verification.filter((c) => c.passed).length}/${r.verification.length} model checks passed` : "";
  const views = [["overview", "Overview"]];
  if (r.histories.length) views.push(["vd", "Vehicle dynamics"], ["mc", "Motion control"]);
  if (r.figures.length) views.push(["figures", figuresLabel(r.experiment.type)]);
  if (r.tables.length) views.push(["tables", "Tables"]);
  v.innerHTML = `<h3>${esc(r.experiment.name || id)} <span class="muted">[${esc(r.experiment.type || "controller_comparison")}]</span></h3>
    <div class="muted">commit ${esc(r.provenance.git_commit || "?")} · ${esc(r.provenance.created_utc || "")} ${ver ? "· " + ver : ""}</div>
    <div class="muted">reproduce: <code>${esc(r.reproduce)}</code></div>
    <div class="subtabs">${views.map(([k, l], i) => `<button data-view="${k}" class="${i ? "" : "active"}">${l}</button>`).join("")}</div>
    <div id="view"></div>`;
  $$(".subtabs button", v).forEach((b) => b.onclick = () => { $$(".subtabs button", v).forEach((x) => x.classList.toggle("active", x === b)); renderView(b.dataset.view); });
  renderView("overview");
}
function csvLine(line) {                     // one CSV record with quoted fields (Python csv module output)
  const out = []; let cur = "", q = false;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (q) { if (c === '"' && line[i + 1] === '"') { cur += '"'; i++; } else if (c === '"') q = false; else cur += c; }
    else if (c === '"') q = true; else if (c === ",") { out.push(cur); cur = ""; } else cur += c;
  }
  out.push(cur);
  return out;
}
function figuresLabel(kind) {
  return { stability_map: "Nonlinear stability", equilibrium_analysis: "Nonlinear stability", design_optimization: "Optimization",
    critical_scenario_search: "Critical scenarios" }[kind] || "Figures";
}
function kpiTable(rows, keys) {
  if (!rows.length) return "<p class='muted'>No KPI rows for this experiment type (see the report and figures).</p>";
  keys = keys || [...new Set(rows.flatMap((r) => Object.keys(r)))].filter((k) => !["point", "controller", "feedback", "result", "name"].includes(k));
  return `<div style="overflow:auto"><table><thead><tr><th>point</th><th>controller</th>${keys.map((k) => `<th>${esc(k)}</th>`).join("")}</tr></thead><tbody>
    ${rows.map((r) => `<tr><td>${esc(r.point || "")}</td><td>${esc(r.controller || "")}${r.feedback ? " · " + esc(r.feedback) : ""}</td>${keys.map((k) => `<td class="num">${fmt(r[k])}</td>`).join("")}</tr>`).join("")}</tbody></table></div>`;
}
async function renderView(view) {
  const el = $("#view"), r = current;
  if (view === "overview") {
    el.innerHTML = kpiTable(r.kpis) + (r.report ? `<h3>Report</h3><pre>${esc(r.report)}</pre>` : "");
  } else if (view === "figures") {
    el.innerHTML = `<div class="gallery">${r.figures.map((f) => `<figure><img loading="lazy" src="/files/${f}"><figcaption>${esc(f)}</figcaption></figure>`).join("")}</div>`;
  } else if (view === "tables") {
    el.innerHTML = r.tables.map((t, i) => `<h3><a href="/files/${t}" download>${esc(t.split("/").pop())}</a></h3><div id="tbl${i}" class="muted">loading…</div>`).join("");
    r.tables.forEach(async (t, i) => {
      const rows = (await (await fetch("/files/" + t)).text()).trim().split(/\r?\n/).map(csvLine);
      const shown = rows.slice(1, 41);
      $("#tbl" + i).outerHTML = `<div style="overflow:auto;max-height:420px"><table><thead><tr>${rows[0].map((h) => `<th>${esc(h)}</th>`).join("")}</tr></thead><tbody>${shown.map((r_) => `<tr>${r_.map((c) => `<td class="num">${esc(isNaN(+c) || c === "" ? c : fmt(+c))}</td>`).join("")}</tr>`).join("")}</tbody></table></div>${rows.length > 41 ? `<p class="muted">first 40 of ${rows.length - 1} rows</p>` : ""}`;
    });
  } else {
    el.innerHTML = `<label>Run <select id="hist">${r.histories.map((h) => `<option value="${esc(h)}">${esc(h.split("/").pop())}</option>`).join("")}</select></label><div class="grid" id="charts"></div>`;
    const draw = async () => {
      const h = (await api("/api/history?path=" + encodeURIComponent($("#hist").value))).columns;
      $("#charts").innerHTML = "";
      (view === "vd" ? vehicleCharts(h) : controlCharts(h)).forEach((c) => $("#charts").appendChild(c));
    };
    $("#hist").onchange = draw;
    draw();
  }
}
const deg = (a) => a.map((x) => x * 180 / Math.PI);
function vehicleCharts(h) {
  const t = h.t, out = [];
  out.push(chart("Trajectory (CoG path)", h.X, [{ name: "path", y: h.Y }], "X [m]", "Y [m]", { equal: true }));
  out.push(chart("Yaw rate", t, [{ name: "r", y: deg(h.r) }, { name: "r reference", y: deg(h.r_ref), dash: true }], "t [s]", "deg/s"));
  out.push(chart("Sideslip", t, [{ name: "β", y: deg(h.beta) }], "t [s]", "deg"));
  if (h.ay) out.push(chart("Accelerations (from tyre forces)", t, [{ name: "a_x", y: h.ax }, { name: "a_y", y: h.ay }], "t [s]", "m/s²"));
  out.push(chart("Steer angles", t, [{ name: "δ front", y: deg(h.steer_f) }, { name: "δ rear", y: deg(h.steer_r) }], "t [s]", "deg"));
  if (h.Fy_FL) out.push(chart("Lateral tyre forces", t, WHEELS.map((w) => ({ name: w, y: h["Fy_" + w].map((x) => x / 1e3) })), "t [s]", "kN"));
  if (h.Fz_FL) out.push(chart("Wheel loads", t, WHEELS.map((w) => ({ name: w, y: h["Fz_" + w].map((x) => x / 1e3) })), "t [s]", "kN"));
  return out;
}
function controlCharts(h) {
  const t = h.t, out = [];
  out.push(chart("Yaw moment: demand vs allocated", t, [{ name: "demand", y: h.Mz_demand.map((x) => x / 1e3) }, { name: "allocated", y: h.Mz_allocated.map((x) => x / 1e3) }], "t [s]", "kN m"));
  out.push(chart("Yaw rate: reference vs actual", t, [{ name: "reference", y: deg(h.r_ref) }, { name: "actual", y: deg(h.r) }], "t [s]", "deg/s"));
  if (h.tyre_utilisation) out.push(chart("Tyre utilisation (max over wheels)", t, [{ name: "utilisation", y: h.tyre_utilisation }], "t [s]", "–"));
  if (h.max_abs_slip) out.push(chart("Max |slip| (ABS/TCS)", t, [{ name: "|κ| max", y: h.max_abs_slip }], "t [s]", "–"));
  if (h.T_brake_total) out.push(chart("Total brake torque", t, [{ name: "brake", y: h.T_brake_total }], "t [s]", "N m"));
  if (h.Fx_FL) out.push(chart("Longitudinal tyre forces (allocation)", t, WHEELS.map((w) => ({ name: w, y: h["Fx_" + w].map((x) => x / 1e3) })), "t [s]", "kN"));
  if (h.status) out.push(statusStrip(t, h.status));
  return out;
}
function statusStrip(t, st) {
  const kinds = [...new Set(st)];
  return chart("Controller status (1 = active state)", t, kinds.map((k) => ({ name: k, y: st.map((s) => s === k ? 1 : 0) })), "t [s]", "state");
}

// ------------------------------------------------------------------ SVG line chart with crosshair tooltip
function chart(title, x, series, xl, yl, o = {}) {
  const box = document.createElement("div");
  box.className = "chart";
  const W = 520, H = 230, L = 48, R = 10, T = 8, B = 30;
  const ys = series.flatMap((s) => s.y).filter(Number.isFinite), xs = x.filter(Number.isFinite);
  let [x0, x1] = [Math.min(...xs), Math.max(...xs)], [y0, y1] = [Math.min(...ys), Math.max(...ys)];
  if (y0 === y1) { y0 -= 1; y1 += 1; }
  const py = (y1 - y0) * 0.06; y0 -= py; y1 += py;
  if (o.equal) {                                     // equal scale for the path
    const sx = (x1 - x0) / (W - L - R), sy = (y1 - y0) / (H - T - B), s = Math.max(sx, sy);
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    x0 = cx - s * (W - L - R) / 2; x1 = cx + s * (W - L - R) / 2; y0 = cy - s * (H - T - B) / 2; y1 = cy + s * (H - T - B) / 2;
  }
  const X = (v) => L + (v - x0) / (x1 - x0) * (W - L - R), Y = (v) => T + (1 - (v - y0) / (y1 - y0)) * (H - T - B);
  const ticks = (a, b, n) => { const st = niceStep((b - a) / n); const out = []; for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toPrecision(10)); return out; };
  let svg = `<svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none">`;
  ticks(y0, y1, 4).forEach((v) => svg += `<line x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}" stroke="#1d2a40" stroke-width="1"/><text x="${L - 6}" y="${Y(v) + 4}" fill="#8b98ad" font-size="10" text-anchor="end">${fmtTick(v)}</text>`);
  ticks(x0, x1, 6).forEach((v) => svg += `<text x="${X(v)}" y="${H - 12}" fill="#8b98ad" font-size="10" text-anchor="middle">${fmtTick(v)}</text>`);
  svg += `<text x="${W - R}" y="${H - 1}" fill="#8b98ad" font-size="10" text-anchor="end">${esc(xl)}</text><text x="4" y="${T + 8}" fill="#8b98ad" font-size="10">${esc(yl)}</text>`;
  series.forEach((s, i) => {
    let d = "";
    s.y.forEach((v, k) => { if (!Number.isFinite(v) || !Number.isFinite(x[k])) return; d += (d ? "L" : "M") + `${X(x[k]).toFixed(1)},${Y(v).toFixed(1)}`; });
    svg += `<path d="${d}" fill="none" stroke="${s.color || PALETTE[i % PALETTE.length]}" stroke-width="2" ${s.dash ? 'stroke-dasharray="5 4"' : ""} stroke-linejoin="round"/>`;
  });
  svg += `<line class="cross" x1="0" x2="0" y1="${T}" y2="${H - B}" stroke="#e6edf6" stroke-width="1" visibility="hidden"/><rect x="${L}" y="${T}" width="${W - L - R}" height="${H - T - B}" fill="transparent"/></svg>`;
  box.innerHTML = `<h4>${esc(title)}</h4>${svg}` + (series.length > 1 ? `<div class="legend">${series.map((s, i) => `<span><i style="background:${s.color || PALETTE[i % PALETTE.length]}"></i>${esc(s.name)}</span>`).join("")}</div>` : "");
  const el = box.querySelector("svg"), cross = box.querySelector(".cross"), tip = $("#tooltip");
  el.onmousemove = (e) => {
    const r = el.getBoundingClientRect(), vx = x0 + ((e.clientX - r.left) / r.width * W - L) / (W - L - R) * (x1 - x0);
    let k = 0, best = Infinity;
    x.forEach((v, j) => { const dd = Math.abs(v - vx); if (dd < best) { best = dd; k = j; } });
    cross.setAttribute("x1", X(x[k])); cross.setAttribute("x2", X(x[k])); cross.setAttribute("visibility", "visible");
    tip.hidden = false; tip.style.left = e.clientX + 14 + "px"; tip.style.top = e.clientY + 10 + "px";
    tip.textContent = `${xl}: ${fmt(x[k])}\n` + series.map((s) => `${s.name}: ${fmt(s.y[k])} ${yl}`).join("\n");
  };
  el.onmouseleave = () => { cross.setAttribute("visibility", "hidden"); tip.hidden = true; };
  return box;
}
function niceStep(raw) { const p = Math.pow(10, Math.floor(Math.log10(raw || 1))); const f = raw / p; return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p; }
function fmtTick(v) { return Math.abs(v) >= 1000 ? v.toFixed(0) : +v.toPrecision(3) + ""; }

// ------------------------------------------------------------------ compare
async function loadCompareList() {
  const list = await api("/api/results");
  $("#compare-list").innerHTML = list.map((r) => `<li><label class="inline"><input type="checkbox" value="${esc(r.id)}"> <span><b>${esc(r.name)}</b><small>${esc(r.kind)} · ${esc(r.created)}</small></span></label></li>`).join("");
  $("#compare-go").onclick = compare;
  $("#export").onclick = exportReport;
}
const selected = () => $$("#compare-list input:checked").map((i) => i.value);
async function compare() {
  const ids = selected(), v = $("#compare-view");
  if (ids.length < 1) { v.innerHTML = "<p class='muted'>Select results.</p>"; return; }
  const c = await post("/api/compare", { ids });
  const results = await Promise.all(ids.map((id) => api("/api/result?id=" + encodeURIComponent(id))));
  const hist = results.flatMap((r) => r.histories.map((h) => ({ label: `${r.experiment.name}: ${h.split("/").pop().replace(".csv", "")}`, path: h })));
  const keys = c.kpis.filter((k) => !k.startsWith("computation"));
  const base = c.rows[0] || {};
  v.innerHTML = `<h3>KPI comparison <span class="muted">(Δ relative to the first row)</span></h3>
    <div style="overflow:auto"><table><thead><tr><th>result</th><th>point</th><th>controller</th>${keys.map((k) => `<th>${esc(k)}</th>`).join("")}</tr></thead><tbody>
    ${c.rows.map((r, i) => `<tr><td>${esc(r.name)}</td><td>${esc(r.point || "")}</td><td>${esc(r.controller || "")}</td>${keys.map((k) => {
      const d = i && base[k] ? (r[k] / base[k] - 1) * 100 : null;
      return `<td class="num">${fmt(r[k])}${d === null || !Number.isFinite(d) ? "" : ` <small class="${d > 0 ? "worse" : "better"}">${d > 0 ? "+" : ""}${d.toFixed(0)} %</small>`}</td>`;
    }).join("")}</tr>`).join("")}</tbody></table></div>
    ${hist.length ? `<h3>Time histories</h3>
    <div class="row"><label>Channel <select id="cmp-ch">${["r", "beta", "ay", "steer_f", "steer_r", "Mz_allocated", "Y", "speed", "tyre_utilisation"].map((x) => `<option>${x}</option>`).join("")}</select></label>
    <label>Runs (Ctrl/Cmd for several) <select id="cmp-runs" multiple size="5">${hist.map((h, i) => `<option value="${esc(h.path)}" ${i < 2 ? "selected" : ""}>${esc(h.label)}</option>`).join("")}</select></label>
    <button id="cmp-draw">Draw</button></div><div class="grid" id="cmp-charts"></div>
    <h3>Animated vehicle motion</h3><div class="anim-controls"><button id="anim-play">play</button><input type="range" id="anim-t" min="0" max="1000" value="0"><span id="anim-time" class="muted"></span></div>
    <svg id="anim" viewBox="0 0 900 360" style="width:100%;height:360px;background:#0d1626;border:1px solid #22304a;border-radius:6px"></svg>` : "<p class='muted'>The selected results have no time histories (only KPI and figure outputs).</p>"}`;
  if (hist.length) { $("#cmp-draw").onclick = drawCompare; drawCompare(); }
}
async function drawCompare() {
  const paths = [...$("#cmp-runs").selectedOptions].map((o) => o.value), ch = $("#cmp-ch").value;
  const labels = [...$("#cmp-runs").selectedOptions].map((o) => o.textContent);
  const H = await Promise.all(paths.map((p) => api("/api/history?path=" + encodeURIComponent(p)).then((r) => r.columns)));
  const conv = (k, a) => ["r", "beta", "steer_f", "steer_r", "r_ref"].includes(k) ? deg(a) : k === "Mz_allocated" ? a.map((x) => x / 1e3) : a;
  const unit = { r: "deg/s", beta: "deg", ay: "m/s²", steer_f: "deg", steer_r: "deg", Mz_allocated: "kN m", Y: "m", speed: "m/s", tyre_utilisation: "–" }[ch];
  const box = $("#cmp-charts");
  box.innerHTML = "";
  const t0 = H[0].t;
  box.appendChild(chart(`Overlay: ${ch}`, t0, H.map((h, i) => ({ name: labels[i], y: interp(h.t, conv(ch, h[ch]), t0) })), "t [s]", unit));
  if (H.length >= 2) box.appendChild(chart(`Difference: ${labels[1]} − ${labels[0]}`, t0,
    [{ name: "difference", y: interp(H[1].t, conv(ch, H[1][ch]), t0).map((v, k) => v - conv(ch, H[0][ch])[k]) }], "t [s]", unit));
  animate(H, labels);
}
function interp(t, y, tq) {
  let j = 0;
  return tq.map((q) => { while (j < t.length - 2 && t[j + 1] < q) j++; if (q < t[0] || q > t[t.length - 1]) return NaN;
    const a = t[j], b = t[j + 1], w = b > a ? (q - a) / (b - a) : 0; return y[j] + w * (y[j + 1] - y[j]); });
}
let animTimer = null;
function animate(H, labels) {
  const svg = $("#anim"), W = 900, Hh = 360, pad = 20;
  const xs = H.flatMap((h) => h.X), ys = H.flatMap((h) => h.Y);
  let [x0, x1, y0, y1] = [Math.min(...xs) - 5, Math.max(...xs) + 5, Math.min(...ys) - 5, Math.max(...ys) + 5];
  const s = Math.min((W - 2 * pad) / (x1 - x0), (Hh - 2 * pad) / (y1 - y0));
  const X = (v) => pad + (v - x0) * s, Y = (v) => Hh - pad - (v - y0) * s;
  const tmax = Math.max(...H.map((h) => h.t[h.t.length - 1]));
  const car = (h, i, k) => {
    const x = h.X[k], y = h.Y[k], p = h.psi ? h.psi[k] : 0, Lc = 4.5, Wc = 1.8, c = Math.cos(p), sn = Math.sin(p);
    const pts = [[Lc / 2, Wc / 2], [Lc / 2, -Wc / 2], [-Lc / 2, -Wc / 2], [-Lc / 2, Wc / 2]].map(([a, b]) => `${X(x + a * c - b * sn)},${Y(y + a * sn + b * c)}`).join(" ");
    return `<polygon points="${pts}" fill="${PALETTE[i % PALETTE.length]}" fill-opacity="0.75" stroke="#0d1626" stroke-width="1"/>`;
  };
  const draw = (frac) => {
    const tq = frac * tmax;
    let g = H.map((h, i) => `<polyline points="${h.X.map((v, k) => `${X(v)},${Y(h.Y[k])}`).join(" ")}" fill="none" stroke="${PALETTE[i % PALETTE.length]}" stroke-width="1.5" stroke-opacity="0.45"/>`).join("");
    H.forEach((h, i) => { let k = h.t.findIndex((v) => v >= tq); if (k < 0) k = h.t.length - 1; g += car(h, i, k); });
    g += labels.map((l, i) => `<text x="${pad}" y="${pad + 14 * i}" fill="${PALETTE[i % PALETTE.length]}" font-size="12">${esc(l)}</text>`).join("");
    svg.innerHTML = g;
    $("#anim-time").textContent = `t = ${tq.toFixed(2)} s (recorded samples; true scale)`;
  };
  const slider = $("#anim-t");
  slider.oninput = () => draw(slider.value / 1000);
  $("#anim-play").onclick = () => {
    clearInterval(animTimer);
    animTimer = setInterval(() => { slider.value = (+slider.value + 1000 / (tmax * 25)) % 1000; draw(slider.value / 1000); }, 40);
    $("#anim-play").textContent = "restart";
  };
  draw(0);
}
async function exportReport() {
  const ids = selected();
  try {
    const r = await post("/api/export", { ids, title: $("#report-title").value });
    $("#export-status").innerHTML = `report ready: <a href="/files/${r.path}" download>${esc(r.path)}</a>`;
    $("#export-status").className = "status ok";
  } catch (e) { status($("#export-status"), e.message, false); }
}

initConfigure().catch((e) => status($("#status"), "could not load options: " + e.message, false));
