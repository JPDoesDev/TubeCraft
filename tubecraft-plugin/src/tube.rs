//! The `tube` entry: makes one tube along the picked curves and copes it against the tubes
//! already in the design. Every host call goes through [`Host`], so the call sequence is
//! tested natively too.
//!
//! Earlier tubes are found in the timeline: every pipe feature that makes a new body is a
//! tube (made by TubeCraft or by hand), and a pipe that joins onto one is its run-on.

use crate::geom::{Curve, V3};
use crate::perp::{self, Cuts};
use crate::plan::{self, chain, inches, CopeStyle, Existing, Options, Plan, Seg, SharpJoints};
use crate::profiles::{self, MM_PER_IN};
use crate::sketch::{curves_of, CurveError};
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// The plugin's view of SolidCraft: run one command, get its JSON reply.
pub trait Host {
    type Err;
    fn run(&mut self, command: &str, args: Value) -> Result<Value, Self::Err>;
    /// An error about the entry's input, named by its field.
    fn input_error(&self, field: &str, reason: String) -> Self::Err;
}

/// Host calls one run may make (SolidCraft allows 256 for an entry; a few are spare).
pub const CALL_BUDGET: usize = 250;

/// A host that counts the calls made through it, to keep the run inside [`CALL_BUDGET`].
struct Counted<'a, H: Host> {
    host: &'a mut H,
    calls: usize,
}

impl<H: Host> Host for Counted<'_, H> {
    type Err = H::Err;
    fn run(&mut self, command: &str, args: Value) -> Result<Value, H::Err> {
        self.calls += 1;
        self.host.run(command, args)
    }
    fn input_error(&self, field: &str, reason: String) -> H::Err {
        self.host.input_error(field, reason)
    }
}

/// Names of the features TubeCraft adds besides tubes: skipped when looking for tubes.
const CUT_PREFIXES: [&str; 2] = ["Cope ", "Miter "];

fn pt(v: V3) -> Value {
    json!(v.to_array())
}

fn strings(v: &Value) -> Vec<String> {
    v.as_array().map(|a| a.iter().filter_map(|s| s.as_str().map(str::to_string)).collect()).unwrap_or_default()
}

fn mm(x: f64) -> String {
    format!("{x:.6} mm")
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

/// A length expression as a pipe feature stores it ("38.100000 mm", "1.5 in", "38.1"), in mm.
/// `None` for an expression that uses parameters.
pub fn length_mm(expr: &str, design_unit: &str) -> Option<f64> {
    let s = expr.trim();
    let split = s.find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')).unwrap_or(s.len());
    let (num, unit) = s.split_at(split);
    let x: f64 = num.trim().parse().ok()?;
    let unit = unit.trim();
    let unit = if unit.is_empty() { design_unit } else { unit };
    let k = match unit {
        "mm" => 1.0,
        "cm" => 10.0,
        "m" => 1000.0,
        "in" | "\"" => MM_PER_IN,
        "ft" | "'" => 12.0 * MM_PER_IN,
        _ => return None,
    };
    Some(x * k)
}

struct Input {
    sketch: i64,
    ids: Vec<String>,
    size: profiles::TubeSize,
    options: Options,
    prefix: String,
}

fn parse<H: Host>(h: &H, params: &Value) -> Result<Input, H::Err> {
    let bad = |field: &str, reason: &str| h.input_error(field, reason.to_string());
    let paths = params.get("paths").and_then(Value::as_array).ok_or_else(|| bad("paths", "pick the sketch lines and arcs of one tube"))?;
    if paths.is_empty() {
        return Err(bad("paths", "pick the sketch lines and arcs of one tube"));
    }
    let mut sketch = None;
    let mut ids = Vec::new();
    for item in paths {
        let kind = item.get("type").and_then(Value::as_str).unwrap_or("");
        if !kind.ends_with("sketch_curve") {
            return Err(bad("paths", "pick sketch lines and arcs, not model edges or faces"));
        }
        let s = item.get("sketch").and_then(Value::as_i64).ok_or_else(|| bad("paths", "a pick names no sketch"))?;
        if sketch.is_some_and(|k| k != s) {
            return Err(bad("paths", "the curves of one tube must be in one sketch"));
        }
        sketch = Some(s);
        let id = item.get("id").and_then(Value::as_str).ok_or_else(|| bad("paths", "a pick names no curve"))?;
        ids.push(id.to_string());
    }
    let size_id = params.get("size").and_then(Value::as_str).unwrap_or("hrew-1.750x0.120");
    let size = *profiles::find(size_id).ok_or_else(|| bad("size", &format!("no tube size `{size_id}`")))?;
    let num = |key: &str, default: f64| params.get(key).and_then(Value::as_f64).unwrap_or(default);
    let gap_in = num("gap_in", 0.0);
    if !(0.0..=0.25).contains(&gap_in) {
        return Err(bad("gap_in", "a weld gap from 0 to 0.25 in"));
    }
    let clr_in = num("clr_in", 5.5);
    if clr_in <= 0.0 {
        return Err(bad("clr_in", "a die CLR above 0"));
    }
    let sharp = match params.get("sharp_joints").and_then(Value::as_str).unwrap_or("miter") {
        "miter" => SharpJoints::Miter,
        "butt" => SharpJoints::Butt,
        other => return Err(bad("sharp_joints", &format!("`{other}`: use miter or butt"))),
    };
    let cope = match params.get("cope").and_then(Value::as_str).unwrap_or("flush") {
        "flush" => CopeStyle::Flush,
        "perpendicular" => CopeStyle::Perpendicular,
        other => return Err(bad("cope", &format!("`{other}`: use flush or perpendicular"))),
    };
    let prefix = params.get("name").and_then(Value::as_str).unwrap_or("Tube").trim().to_string();
    if prefix.is_empty() || prefix.chars().count() > 40 {
        return Err(bad("name", "a name of 1 to 40 characters"));
    }
    // "<name> NN" must not read as one of TubeCraft's own cuts, which later runs skip.
    if let Some(p) = CUT_PREFIXES.iter().find(|p| format!("{prefix} ").starts_with(**p)) {
        return Err(bad("name", &format!("a name that does not start with `{}`: TubeCraft names its own cuts that way", p.trim())));
    }
    Ok(Input {
        sketch: sketch.unwrap(),
        ids,
        size,
        options: Options {
            od_mm: size.od_mm(),
            wall_mm: size.wall_mm(),
            gap_mm: gap_in * MM_PER_IN,
            clr_mm: clr_in * MM_PER_IN,
            sharp,
            cope,
        },
        prefix,
    })
}

/// "<prefix> NN" with the first number above those already used, by bodies or features.
fn next_name(taken: &[String], prefix: &str) -> String {
    let mut max = 0;
    for name in taken {
        if let Some(rest) = name.strip_prefix(prefix).and_then(|r| r.strip_prefix(' ')) {
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            if let Ok(n) = digits.parse::<usize>() {
                max = max.max(n);
            }
        }
    }
    format!("{prefix} {:02}", max + 1)
}

type SketchCurves = BTreeMap<String, Result<Curve, CurveError>>;

fn read_sketch<H: Host>(h: &mut H, cache: &mut BTreeMap<i64, SketchCurves>, sketch: i64) -> Result<(), H::Err> {
    if let std::collections::btree_map::Entry::Vacant(slot) = cache.entry(sketch) {
        let reply = h.run("sketch.inspect", json!({"sketch": sketch}))?;
        slot.insert(curves_of(&reply));
    }
    Ok(())
}

/// A pipe feature's definition, as `timeline.inspect` gives it.
struct PipeDef {
    name: String,
    op: String,
    /// `None` for a pipe whose path is not sketch curves.
    sketch: Option<i64>,
    path: Vec<String>,
    od_mm: Option<f64>,
    wall_mm: Option<f64>,
    body_names: Vec<String>,
    targets: Vec<String>,
}

/// The earlier tubes, read from the timeline.
fn earlier_tubes<H: Host>(
    h: &mut Counted<H>,
    doc: &Value,
    cache: &mut BTreeMap<i64, SketchCurves>,
    warnings: &mut Vec<String>,
) -> Result<Vec<Existing>, H::Err> {
    let unit = doc.get("units").and_then(Value::as_str).unwrap_or("mm").to_string();
    let bodies = strings(doc.get("bodies").unwrap_or(&Value::Null));
    let mut pipes = Vec::new();
    for f in doc.get("timeline").and_then(Value::as_array).into_iter().flatten() {
        let ty = f.get("type").and_then(Value::as_str).unwrap_or("");
        let name = f.get("name").and_then(Value::as_str).unwrap_or("").to_string();
        let off = |k: &str| f.get(k).is_some_and(|v| v.as_bool() == Some(true) || (v.is_string()));
        if !(ty == "PipeFeature" || ty == "pipe") || off("suppressed") || off("rolled_back") || off("error") {
            continue;
        }
        if CUT_PREFIXES.iter().any(|p| name.starts_with(p)) {
            continue;
        }
        if let Some(id) = f.get("id").cloned() {
            pipes.push((id, name));
        }
    }
    // Each pipe is one call to read, and the tube itself needs a few more.
    if h.calls + pipes.len() + 3 > CALL_BUDGET {
        return Err(h.input_error(
            "paths",
            format!(
                "this design has {} pipe features to read and one run may make {CALL_BUDGET} SolidCraft steps: it is too large for TubeCraft",
                pipes.len()
            ),
        ));
    }
    let mut defs = Vec::new();
    for (id, name) in pipes {
        let reply = h.run("timeline.inspect", json!({"feature": id}))?;
        let d = &reply["definition"];
        let op = d.get("operation").and_then(Value::as_str).unwrap_or("new_body").to_string();
        let len = |k: &str| d.get(k).and_then(Value::as_str).and_then(|s| length_mm(s, &unit));
        defs.push(PipeDef {
            name: reply.get("name").and_then(Value::as_str).map(str::to_string).unwrap_or(name),
            op,
            sketch: d.get("path_sketch").and_then(Value::as_i64),
            path: strings(&d["path"]),
            od_mm: len("diameter").or_else(|| d.get("diameter").and_then(Value::as_f64)),
            wall_mm: len("wall").or_else(|| d.get("wall").and_then(Value::as_f64)),
            body_names: strings(&d["body_names"]),
            targets: strings(&d["targets"]),
        });
    }

    let mut out: Vec<Existing> = Vec::new();
    for d in defs.iter().filter(|d| d.op == "new_body" || d.op == "new") {
        let Some(od) = d.od_mm else {
            warnings.push(format!("{}: its diameter is not a plain length, so nothing is coped to it", d.name));
            continue;
        };
        let names = if d.body_names.is_empty() { vec![d.name.clone()] } else { d.body_names.clone() };
        let found: Vec<String> = bodies
            .iter()
            .filter(|b| names.iter().any(|n| *b == n || b.starts_with(&format!("{n} ("))))
            .cloned()
            .collect();
        if found.is_empty() {
            continue; // Removed or combined away.
        }
        let unread = format!("{}: its path is not lines and arcs TubeCraft can read, so nothing is coped to it", d.name);
        let Some(sketch) = d.sketch else {
            warnings.push(unread);
            continue;
        };
        read_sketch(h, cache, sketch)?;
        let curves: Option<Vec<(String, Curve)>> =
            d.path.iter().map(|id| cache[&sketch].get(id).and_then(|c| c.as_ref().ok()).map(|c| (id.clone(), *c))).collect();
        let Some(curves) = curves.filter(|c| !c.is_empty()) else {
            warnings.push(unread);
            continue;
        };
        let ch = chain(&curves).ok();
        let polyline = match &ch {
            Some(c) => c.polyline(),
            None => curves.iter().flat_map(|(_, c)| c.polyline()).collect(),
        };
        out.push(Existing {
            name: d.name.clone(),
            bodies: found,
            od_mm: od,
            wall_mm: d.wall_mm,
            sketch,
            chain: ch,
            polyline,
            run_ons: Vec::new(),
            ids: d.path.clone(),
        });
    }
    // Run-ons: pipes joined onto a tube's body.
    for d in defs.iter().filter(|d| d.op == "join") {
        let Some(k) = out.iter().position(|e| d.targets.iter().any(|t| e.bodies.contains(t))) else { continue };
        let Some(sketch) = d.sketch else { continue };
        read_sketch(h, cache, sketch)?;
        let poly: Vec<V3> = d
            .path
            .iter()
            .filter_map(|id| cache[&sketch].get(id).and_then(|c| c.as_ref().ok()))
            .flat_map(Curve::polyline)
            .collect();
        if !poly.is_empty() {
            out[k].run_ons.push((sketch, d.path.clone(), poly));
        }
    }
    Ok(out)
}

pub fn run<H: Host>(host: &mut H, params: &Value) -> Result<Value, H::Err> {
    let input = parse(host, params)?;
    let h = &mut Counted { host, calls: 0 };
    let mut cache: BTreeMap<i64, SketchCurves> = BTreeMap::new();
    read_sketch(h, &mut cache, input.sketch)?;
    let mut curves = Vec::new();
    for id in &input.ids {
        match cache[&input.sketch].get(id) {
            Some(Ok(c)) => curves.push((id.clone(), *c)),
            Some(Err(CurveError::Unsupported(why))) => {
                return Err(h.input_error("paths", format!("curve {id} of sketch {} is {why}", input.sketch)))
            }
            Some(Err(CurveError::Missing)) | None => {
                return Err(h.input_error("paths", format!("sketch {} has no curve {id}", input.sketch)))
            }
        }
    }

    let doc = h.run("document.inspect", json!({"timeline": "summary", "bodies": "names"}))?;
    let mut warnings = Vec::new();
    let existing = earlier_tubes(h, &doc, &mut cache, &mut warnings)?;
    let mut taken = strings(doc.get("bodies").unwrap_or(&Value::Null));
    taken.extend(doc.get("timeline").and_then(Value::as_array).into_iter().flatten().filter_map(|f| f["name"].as_str().map(str::to_string)));
    let name = next_name(&taken, &input.prefix);

    let mut plan = plan::build(&name, input.sketch, &curves, &existing, &input.options).map_err(|e| h.input_error(e.field, e.reason))?;
    plan.warnings.splice(0..0, warnings);
    let cuts = perp::cuts(&name, &plan, &existing, &input.options);
    plan.warnings.extend(cuts.warnings.iter().cloned());
    let calls = h.calls + plan.host_calls(&existing, &cuts.flush_new, cuts.flush_old.len()) + cuts.host_calls();
    if calls > CALL_BUDGET {
        return Err(h.input_error(
            "paths",
            format!("this tube needs about {calls} SolidCraft steps and one run may make {CALL_BUDGET}"),
        ));
    }
    build(h, &name, &input, &plan, &cuts, &existing)
}

/// Cuts one perpendicular cope on `body`: draws the cope's radial lines in a 3D sketch, lofts
/// the cut face through them, splits the body with it and removes the off-cut. Returns the
/// name of the body that is left: the larger piece (the off-cut is always the shorter one).
/// The lines' sketch is added to `sketches`, to be hidden when the tube is done.
#[allow(clippy::too_many_arguments)]
fn perp_cut<H: Host>(
    h: &mut H,
    features: &mut Vec<Value>,
    sketches: &mut Vec<Value>,
    label: &str,
    body: &str,
    end: V3,
    into: V3,
    cope: &perp::Cope,
) -> Result<String, H::Err> {
    let reply = h.run("sketch.create", json!({"plane": "XY", "name": format!("{label} lines")}))?;
    let sketch = reply.get("sketch").cloned().unwrap_or(Value::Null);
    h.run("sketch.options", json!({"sketch_3d": true}))?;
    let points: Vec<Value> = cope.zigzag().into_iter().map(pt).collect();
    let reply = h.run("sketch.line3d", json!({"points": points}))?;
    let wires = strings(&reply["created"]["wires"]);
    h.run("sketch.finish", json!({}))?;
    features.push(sketch.clone());
    // The zigzag's first, third, ... segments are the radial lines.
    let sections: Vec<Value> = wires.iter().step_by(2).map(|w| json!({"sketch": sketch, "curves": [w]})).collect();
    if sections.len() != cope.lines.len() {
        return Err(h.input_error("cope", format!("{label}: the cut lines could not be drawn")));
    }
    let surface = format!("{label} face");
    let reply = h.run("surface.loft", json!({"sections": sections, "name": format!("{label} face"), "body_name": surface}))?;
    features.extend(reply.get("feature").filter(|f| !f.is_null()).cloned());
    let surface = strings(&reply["created_bodies"]).into_iter().next().unwrap_or(surface);

    let reply = h.run("solid.split_body", json!({"body": body, "tool": {"body": surface, "point": pt(cope.pick)}, "name": label}))?;
    features.extend(reply.get("feature").filter(|f| !f.is_null()).cloned());
    let mut pieces = vec![body.to_string()];
    pieces.extend(strings(&reply["created_bodies"]).into_iter().filter(|b| b != body && *b != surface));

    // The tube is the larger piece: the cope is never longer than the straight end it is cut
    // from, and a tube of one line shares that line between its two ends. (The piece whose
    // middle is furthest along the tube would be wrong for a tube bent back past its own end.)
    // Without volumes, the piece whose middle is furthest from the end.
    let mut keep = pieces[0].clone();
    if pieces.len() > 1 {
        let reply = h.run("inspect.measure", json!({"bodies": pieces}))?;
        let measured: Vec<&Value> = reply["bodies"].as_array().into_iter().flatten().collect();
        let by_volume = measured.iter().all(|b| b["volume_mm3"].is_number());
        let along = |b: &Value| -> Option<f64> {
            let c: Vec<f64> = b["center_of_mass"].as_array()?.iter().filter_map(Value::as_f64).collect();
            (c.len() == 3).then(|| (V3 { x: c[0], y: c[1], z: c[2] } - end).dot(into))
        };
        let mut best = f64::NEG_INFINITY;
        for b in measured {
            let value = if by_volume { b["volume_mm3"].as_f64() } else { along(b) };
            if let (Some(n), Some(x)) = (b["name"].as_str(), value) {
                if x > best {
                    best = x;
                    keep = n.to_string();
                }
            }
        }
    }
    let mut gone: Vec<String> = pieces.into_iter().filter(|b| *b != keep).collect();
    gone.push(surface);
    let reply = h.run("solid.remove", json!({"bodies": gone}))?;
    features.extend(reply.get("feature").filter(|f| !f.is_null()).cloned());
    sketches.push(sketch);
    Ok(keep)
}

fn build<H: Host>(h: &mut H, name: &str, input: &Input, plan: &Plan, cuts: &Cuts, existing: &[Existing]) -> Result<Value, H::Err> {
    let opt = &input.options;
    let od = mm(opt.od_mm);
    let wall = mm(opt.wall_mm);
    let mut features: Vec<Value> = Vec::new();
    let mut sketches: Vec<Value> = Vec::new();
    let add = |reply: &Value, features: &mut Vec<Value>| {
        if let Some(f) = reply.get("feature").filter(|f| !f.is_null()) {
            features.push(f.clone());
        }
    };

    // The tube.
    let reply = h.run(
        "solid.pipe",
        json!({"path_sketch": input.sketch, "path": plan.chain.ids(), "diameter": od, "wall": wall,
               "name": name, "body_name": name}),
    )?;
    add(&reply, &mut features);
    let mut bodies: Vec<String> = strings(&reply["created_bodies"]);
    if bodies.is_empty() {
        bodies.push(name.to_string());
    }

    // Helper lines for miters and butts, in one 3D sketch.
    let mut segs: Vec<Seg> = Vec::new();
    for m in &plan.miters {
        segs.extend([m.run_on_old, m.run_on_new, m.cut_old, m.cut_new]);
    }
    for b in &plan.butts {
        segs.push(b.run_on_old);
    }
    let mut wires: Vec<String> = Vec::new();
    let mut helper = Value::Null;
    if !segs.is_empty() {
        let reply = h.run("sketch.create", json!({"plane": "XY", "name": format!("{name} corners")}))?;
        helper = reply.get("sketch").cloned().unwrap_or(Value::Null);
        h.run("sketch.options", json!({"sketch_3d": true}))?;
        for s in &segs {
            let reply = h.run("sketch.line3d", json!({"points": [pt(s.from), pt(s.to)]}))?;
            wires.push(reply["created"]["wires"][0].as_str().unwrap_or("w1").to_string());
        }
        h.run("sketch.finish", json!({}))?;
        features.push(helper.clone());
    }
    let mut wire = wires.iter();

    let mut mitred_to = Vec::new();
    for m in &plan.miters {
        let e = &existing[m.with];
        let (w_old, w_new, c_old, c_new) = (wire.next().unwrap(), wire.next().unwrap(), wire.next().unwrap(), wire.next().unwrap());
        let mut old_pipe = json!({"path_sketch": helper, "path": [w_old], "diameter": mm(e.od_mm), "operation": "join",
                                  "targets": e.bodies, "name": format!("{} run-on", e.name)});
        if let Some(w) = e.wall_mm {
            old_pipe["wall"] = json!(mm(w));
        }
        add(&h.run("solid.pipe", old_pipe)?, &mut features);
        add(
            &h.run(
                "solid.pipe",
                json!({"path_sketch": helper, "path": [w_new], "diameter": od, "wall": wall, "operation": "join",
                       "targets": bodies, "name": format!("{name} run-on")}),
            )?,
            &mut features,
        );
        let cutters = [
            (c_old, &e.bodies, format!("Miter {} to {name}", e.name)),
            (c_new, &bodies, format!("Miter {name} to {}", e.name)),
        ];
        for (w, targets, label) in cutters {
            add(
                &h.run(
                    "solid.pipe",
                    json!({"path_sketch": helper, "path": [w], "diameter": mm(m.cutter_dia), "operation": "cut",
                           "targets": targets, "name": label}),
                )?,
                &mut features,
            );
        }
        mitred_to.push(e.name.clone());
    }

    let mut butt_wires: Vec<(usize, String)> = Vec::new();
    for b in &plan.butts {
        let e = &existing[b.with];
        let w = wire.next().unwrap().clone();
        let mut pipe = json!({"path_sketch": helper, "path": [w], "diameter": mm(e.od_mm), "operation": "join",
                              "targets": e.bodies, "name": format!("{} run-on", e.name)});
        if let Some(wall) = e.wall_mm {
            pipe["wall"] = json!(mm(wall));
        }
        add(&h.run("solid.pipe", pipe)?, &mut features);
        butt_wires.push((b.with, w));
    }

    // Perpendicular copes of the new tube's ends, while it is still one whole body.
    let mut coped_to: Vec<String> = Vec::new();
    let mut perp_copes: Vec<Value> = Vec::new();
    // Feature names are unique in a design: a tube's second cope to the same tube says so.
    let mut labels: Vec<String> = Vec::new();
    let mut unique = |label: String| {
        let n = labels.iter().filter(|l| **l == label).count();
        labels.push(label.clone());
        if n == 0 { label } else { format!("{label}, end {}", n + 1) }
    };
    for c in &cuts.new_ends {
        let to: Vec<String> = c.to.iter().map(|&k| existing[k].name.clone()).collect();
        let label = unique(format!("Cope {name} to {}", to.join(" + ")));
        let kept = perp_cut(h, &mut features, &mut sketches, &label, &bodies[0], c.end, c.into, &c.cope)?;
        bodies[0] = kept;
        perp_copes.push(json!({"tube": name, "to": to, "end_mm": pt(c.end), "trim_long_point_in": round3(c.cope.trim_min / MM_PER_IN),
                               "trim_short_point_in": round3(c.cope.trim_max / MM_PER_IN),
                               "clearance_in": round3(c.cope.clearance / MM_PER_IN)}));
        for n in to {
            if !coped_to.contains(&n) {
                coped_to.push(n);
            }
        }
    }

    // Flush copes of the new tube: cut by each earlier tube's outside along its paths.
    for &k in &cuts.flush_new {
        let e = &existing[k];
        let mut paths: Vec<(Value, Value)> = vec![(json!(e.sketch), json!(e.ids))];
        paths.extend(e.run_ons.iter().map(|(s, ids, _)| (json!(s), json!(ids))));
        paths.extend(butt_wires.iter().filter(|(j, _)| *j == k).map(|(_, w)| (helper.clone(), json!([w]))));
        for (sketch, path) in paths {
            let reply = h.run(
                "solid.pipe",
                json!({"path_sketch": sketch, "path": path, "diameter": mm(e.od_mm + 2.0 * opt.gap_mm), "operation": "cut",
                       "targets": bodies, "name": unique(format!("Cope {name} to {}", e.name))}),
            )?;
            add(&reply, &mut features);
            for b in strings(&reply["created_bodies"]) {
                if !bodies.contains(&b) {
                    bodies.push(b);
                }
            }
        }
        if !coped_to.contains(&e.name) {
            coped_to.push(e.name.clone());
        }
    }

    // Rule B: an earlier tube that ends on the new one, which runs through, is coped to it.
    let mut coped_ends: Vec<String> = Vec::new();
    let mut old_body: BTreeMap<usize, String> = BTreeMap::new();
    for c in &cuts.old_ends {
        let e = &existing[c.tube];
        let label = unique(format!("Cope {} to {name}", e.name));
        let body = old_body.get(&c.tube).cloned().unwrap_or_else(|| e.bodies[0].clone());
        let kept = perp_cut(h, &mut features, &mut sketches, &label, &body, c.end, c.into, &c.cope)?;
        old_body.insert(c.tube, kept);
        perp_copes.push(json!({"tube": e.name, "to": [name], "end_mm": pt(c.end), "trim_long_point_in": round3(c.cope.trim_min / MM_PER_IN),
                               "trim_short_point_in": round3(c.cope.trim_max / MM_PER_IN),
                               "clearance_in": round3(c.cope.clearance / MM_PER_IN)}));
        if !coped_ends.contains(&e.name) {
            coped_ends.push(e.name.clone());
        }
    }
    for &k in &cuts.flush_old {
        let e = &existing[k];
        let reply = h.run(
            "solid.pipe",
            json!({"path_sketch": input.sketch, "path": plan.chain.ids(), "diameter": mm(opt.od_mm + 2.0 * opt.gap_mm),
                   "operation": "cut", "targets": e.bodies, "name": unique(format!("Cope {} to {name}", e.name))}),
        )?;
        add(&reply, &mut features);
        coped_ends.push(e.name.clone());
    }

    h.run("timeline.group", json!({"name": name, "features": features}))?;
    // The helper sketches stay in the timeline (the cuts are built on them) but out of sight.
    // SolidCraft 0.48.1 to 0.49.1 show them again when the run ends; see docs/DESIGN.md.
    if !helper.is_null() {
        sketches.push(helper.clone());
    }
    if !sketches.is_empty() {
        h.run("browser.visibility", json!({"visible": false, "items": sketches}))?;
    }

    let run_on: Vec<Value> = plan
        .miters
        .iter()
        .map(|m| (m.with, m.run_on_old.len()))
        .chain(plan.butts.iter().map(|b| (b.with, b.run_on_old.len())))
        .map(|(k, l)| json!({"tube": existing[k].name, "run_on_in": round3(l / MM_PER_IN)}))
        .collect();
    // The stock to cut: the centreline, on to each miter's long point (the run-on is 1 mm more),
    // less what each cope takes off at its long point.
    let own_run_on: f64 = plan.miters.iter().map(|m| m.run_on_new.len() - plan::MITER_OVER_MM).sum();
    let own_trim: f64 = cuts.new_ends.iter().map(|c| c.cope.trim_min).sum::<f64>() + cuts.flush_trim;
    let cut_mm = plan.chain.length() + own_run_on - own_trim;
    let style = match input.options.cope {
        CopeStyle::Flush => "flush",
        CopeStyle::Perpendicular => "perpendicular",
    };
    Ok(json!({
        "tube": {
            "name": name,
            "bodies": bodies,
            "size": input.size.title,
            "sketch": input.sketch,
            "curves": plan.chain.ids(),
            "centreline_in": round3(plan.chain.length() / MM_PER_IN),
            "cut_length_in": round3(cut_mm / MM_PER_IN),
            "cope": style,
            "bends": plan.bends.iter().map(|b| json!({"curve": b.curve, "angle_deg": round3(b.angle_deg), "clr_in": round3(b.clr_mm / MM_PER_IN)})).collect::<Vec<_>>(),
            "coped_to": coped_to,
            "mitred_to": mitred_to,
        },
        "perpendicular_copes": perp_copes,
        "earlier_tubes_coped": coped_ends,
        "earlier_tubes_run_on": run_on,
        "earlier_tubes_found": existing.len(),
        "warnings": plan.warnings,
        "summary": summary(name, input, plan, cuts, cut_mm, &coped_to, &mitred_to, &coped_ends),
    }))
}

#[allow(clippy::too_many_arguments)]
fn summary(name: &str, input: &Input, plan: &Plan, cuts: &Cuts, cut_mm: f64, coped_to: &[String], mitred: &[String], coped_ends: &[String]) -> String {
    let mut s = format!("{name}: {}, {} centreline, {} to cut", input.size.title, inches(plan.chain.length()), inches(cut_mm));
    if !plan.bends.is_empty() {
        s += &format!(", {} bend(s)", plan.bends.len());
    }
    if !coped_to.is_empty() {
        s += &format!("; coped to {}", coped_to.join(", "));
    }
    for c in &cuts.new_ends {
        s += &format!("; perpendicular cope trims {}", perp::trim_text(&c.cope));
    }
    if !cuts.new_ends.is_empty() || !cuts.old_ends.is_empty() {
        s += "; a perpendicular cope does not follow the sketch: make the tube again after moving its lines";
    }
    if !mitred.is_empty() {
        s += &format!("; mitred to {}", mitred.join(", "));
    }
    if !coped_ends.is_empty() {
        s += &format!("; coped the end of {} to it", coped_ends.join(", "));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers from canned replies by command, and records every call.
    struct Fake {
        calls: Vec<(String, Value)>,
        /// The design's pipe features, as `timeline.inspect` definitions, by feature id.
        pipes: Vec<(i64, &'static str, Value)>,
        bodies: Vec<&'static str>,
        units: &'static str,
        /// Where `inspect.measure` puts the middle of a split's "(1)" piece and of the other.
        offcut_at: [f64; 3],
        tube_at: [f64; 3],
        /// The volumes `inspect.measure` gives the "(1)" piece and the other, when it gives any.
        volumes: Option<[f64; 2]>,
    }

    impl Fake {
        fn new(pipes: Vec<(i64, &'static str, Value)>, bodies: Vec<&'static str>, units: &'static str) -> Fake {
            Fake { calls: vec![], pipes, bodies, units, offcut_at: [0.0; 3], tube_at: [0.0; 3], volumes: None }
        }
    }

    impl Host for Fake {
        type Err = String;
        fn run(&mut self, command: &str, args: Value) -> Result<Value, String> {
            self.calls.push((command.to_string(), args.clone()));
            let n = self.calls.len();
            Ok(match command {
                "sketch.inspect" => example_sketch(),
                "document.inspect" => {
                    let mut tl = vec![json!({"id": 1, "name": "Sketch1", "type": "Sketch"})];
                    tl.extend(self.pipes.iter().map(|(id, name, _)| json!({"id": id, "name": name, "type": "PipeFeature"})));
                    json!({"units": self.units, "bodies": self.bodies, "timeline": tl})
                }
                "timeline.inspect" => {
                    let (_, name, def) = self.pipes.iter().find(|(id, _, _)| json!(id) == args["feature"]).unwrap();
                    json!({"name": name, "definition": def})
                }
                "solid.pipe" => {
                    let created = if args.get("operation").is_none() { vec![args["body_name"].clone()] } else { vec![] };
                    json!({"feature": 100 + n, "created_bodies": created})
                }
                "sketch.create" => json!({"sketch": 9}),
                "sketch.line3d" => {
                    let segments = args["points"].as_array().map_or(1, |p| p.len() - 1);
                    json!({"created": {"wires": (0..segments).map(|i| format!("w{n}_{i}")).collect::<Vec<_>>()}})
                }
                "surface.loft" => json!({"feature": 100 + n, "created_bodies": [args["body_name"].clone()]}),
                "solid.split_body" => json!({"feature": 100 + n, "created_bodies": [format!("{} (1)", args["body"].as_str().unwrap())]}),
                "inspect.measure" => {
                    // The off-cut "(1)" lies at the end; the tube's middle is far along it.
                    let bodies: Vec<Value> = args["bodies"].as_array().unwrap().iter().map(|b| {
                        let name = b.as_str().unwrap();
                        let first = name.ends_with("(1)");
                        let mut body = json!({"name": name, "center_of_mass": if first { self.offcut_at } else { self.tube_at }});
                        if let Some(v) = self.volumes {
                            body["volume_mm3"] = json!(v[usize::from(!first)]);
                        }
                        body
                    }).collect();
                    json!({"bodies": bodies})
                }
                "solid.remove" => json!({"feature": 100 + n}),
                _ => json!({}),
            })
        }
        fn input_error(&self, field: &str, reason: String) -> String {
            format!("{field}: {reason}")
        }
    }

    fn example_sketch() -> Value {
        json!({
            "world_plane": {"origin": [0.0, 0.0, 0.0], "x_dir": [1.0, 0.0, 0.0], "y_dir": [0.0, 1.0, 0.0]},
            "curves": [
                {"id": "l1", "type": "line", "start_at": [-609.6, -457.2], "end_at": [469.9, -457.2]},
                {"id": "a1", "type": "arc", "center_at": [469.9, -317.5], "radius": 139.7, "start_at": [469.9, -457.2], "end_at": [609.6, -317.5]},
                {"id": "l2", "type": "line", "start_at": [609.6, -317.5], "end_at": [609.6, 317.5]},
                {"id": "a2", "type": "arc", "center_at": [469.9, 317.5], "radius": 139.7, "start_at": [609.6, 317.5], "end_at": [469.9, 457.2]},
                {"id": "l3", "type": "line", "start_at": [469.9, 457.2], "end_at": [-609.6, 457.2]},
                {"id": "l7", "type": "line", "start_at": [562.637989461379, -421.97849209603436], "end_at": [-69.85, 457.2]},
                {"id": "l8", "type": "line", "start_at": [-69.85, 457.2], "end_at": [-69.85, -457.2]},
                {"id": "l9", "type": "line", "start_at": [-609.6, -457.2], "end_at": [-609.6, 457.2]}
            ]
        })
    }

    fn picks(ids: &[&str]) -> Value {
        json!(ids.iter().map(|id| json!({"type": "external_sketch_curve", "sketch": 1, "id": id})).collect::<Vec<_>>())
    }

    fn hoop_def() -> Value {
        json!({"path_sketch": 1, "path": ["l1", "a1", "l2", "a2", "l3"], "diameter": "1.75 in", "wall": "0.12 in",
               "operation": "new_body", "body_names": ["Main 01"]})
    }

    fn cmds(h: &Fake) -> Vec<&str> {
        h.calls.iter().map(|(c, _)| c.as_str()).collect()
    }

    #[test]
    fn lengths_in_any_unit() {
        let close = |x: Option<f64>| x.is_some_and(|x| (x - 38.1).abs() < 1e-9);
        assert!(close(length_mm("38.100000 mm", "in")));
        assert!(close(length_mm("1.5 in", "mm")));
        assert!(close(length_mm("1.5", "in")));
        assert_eq!(length_mm("d8", "mm"), None);
    }

    #[test]
    fn first_tube_in_an_empty_design() {
        let mut h = Fake::new(vec![], vec![], "in");
        let out = run(&mut h, &json!({"paths": picks(&["l3", "a2", "l2", "a1", "l1"]), "size": "hrew-1.750x0.120"})).unwrap();
        assert_eq!(cmds(&h), ["sketch.inspect", "document.inspect", "solid.pipe", "timeline.group"]);
        assert_eq!(out["tube"]["name"], "Tube 01");
        assert_eq!(out["tube"]["bends"].as_array().unwrap().len(), 2);
        // The pipe path is in chain order, whatever the pick order.
        assert_eq!(h.calls[2].1["path"].as_array().unwrap().len(), 5);
        assert_eq!(h.calls[2].1["diameter"], "44.450000 mm");
    }

    #[test]
    fn brace_after_hoop_is_coped_to_it_in_its_own_size() {
        let mut h = Fake::new(vec![(2, "Main 01", hoop_def())], vec!["Main 01"], "in");
        let out = run(&mut h, &json!({"paths": picks(&["l8"]), "size": "hrew-1.500x0.120", "name": "Brace"})).unwrap();
        assert_eq!(cmds(&h), ["sketch.inspect", "document.inspect", "timeline.inspect", "solid.pipe", "solid.pipe", "timeline.group"]);
        let cope = &h.calls[4].1;
        assert_eq!(cope["operation"], "cut");
        assert_eq!(cope["targets"], json!(["Brace 01"]));
        assert_eq!(cope["path"], json!(["l1", "a1", "l2", "a2", "l3"]));
        // Cut by the hoop's own outside: 1.75 in, not the brace's 1.5 in.
        assert_eq!(cope["diameter"], "44.450000 mm");
        assert_eq!(out["tube"]["coped_to"], json!(["Main 01"]));
        assert_eq!(h.calls[5].1["name"], "Brace 01");
    }

    const PERP_CUT: [&str; 8] = [
        "sketch.create", "sketch.options", "sketch.line3d", "sketch.finish", "surface.loft", "solid.split_body",
        "inspect.measure", "solid.remove",
    ];

    #[test]
    fn perpendicular_brace_is_split_by_a_lofted_face_at_each_end() {
        let mut h = Fake::new(vec![(2, "Main 01", hoop_def())], vec!["Main 01"], "in");
        let params = json!({"paths": picks(&["l8"]), "size": "hrew-1.500x0.120", "name": "Brace", "cope": "perpendicular"});
        let out = run(&mut h, &params).unwrap();
        let mut want = vec!["sketch.inspect", "document.inspect", "timeline.inspect", "solid.pipe"];
        want.extend(PERP_CUT);
        want.extend(PERP_CUT);
        want.extend(["timeline.group", "browser.visibility"]);
        assert_eq!(cmds(&h), want);
        // The run makes as many calls as it counted before it started.
        assert_eq!(h.calls.len(), 3 + 1 + 2 * perp::CALLS_PER_COPE + 2);
        assert_eq!(h.calls.last().unwrap().1["items"].as_array().unwrap().len(), 2);
        // No cut by the hoop's outside: the only pipe is the brace itself.
        assert_eq!(h.calls.iter().filter(|(c, _)| c == "solid.pipe").count(), 1);
        // One zigzag draws every radial line; the loft goes through each of them once.
        let lines = h.calls[6].1["points"].as_array().unwrap().len() / 2;
        assert_eq!(lines, perp::STATIONS + 1);
        let sections = h.calls[8].1["sections"].as_array().unwrap();
        assert_eq!(sections.len(), lines);
        assert_eq!(sections[1]["curves"], json!(["w7_2"]));
        // The tube is split by that face; the off-cut and the face go.
        assert_eq!(h.calls[9].1["body"], "Brace 01");
        assert_eq!(h.calls[9].1["tool"]["body"], "Cope Brace 01 to Main 01 face");
        assert_eq!(h.calls[11].1["bodies"], json!(["Brace 01 (1)", "Cope Brace 01 to Main 01 face"]));
        // The second end's features have names of their own.
        assert_eq!(h.calls[12].1["name"], "Cope Brace 01 to Main 01, end 2 lines");
        assert_eq!(out["tube"]["cope"], "perpendicular");
        assert_eq!(out["tube"]["bodies"], json!(["Brace 01"]));
        assert_eq!(out["tube"]["coped_to"], json!(["Main 01"]));
        let copes = out["perpendicular_copes"].as_array().unwrap();
        assert_eq!(copes.len(), 2);
        // A square 1.5 on 1.75 in tee: the short point is trimmed by the hoop's radius.
        assert!((copes[0]["trim_short_point_in"].as_f64().unwrap() - 0.875).abs() < 0.006);
        assert!(out["tube"]["cut_length_in"].as_f64().unwrap() < out["tube"]["centreline_in"].as_f64().unwrap());
        // Every step is in the tube's folder.
        assert_eq!(h.calls[h.calls.len() - 2].1["features"].as_array().unwrap().len(), 1 + 2 * 4);
    }

    #[test]
    fn perpendicular_cope_keeps_the_piece_away_from_the_end_whatever_its_name() {
        let mut h = Fake::new(vec![(2, "Main 01", hoop_def())], vec!["Main 01"], "in");
        // The split leaves the tube as the new "(1)" body and the off-cut under the old name.
        h.offcut_at = [-69.85, 0.0, 0.0];
        h.tube_at = [-69.85, 450.0, 0.0];
        let params = json!({"paths": picks(&["l8"]), "size": "hrew-1.500x0.120", "name": "Brace", "cope": "perpendicular"});
        let out = run(&mut h, &params).unwrap();
        assert_eq!(h.calls[11].1["bodies"], json!(["Brace 01", "Cope Brace 01 to Main 01 face"]));
        assert_eq!(h.calls[17].1["body"], "Brace 01 (1)");
        assert_eq!(out["tube"]["bodies"], json!(["Brace 01 (1)"]));
    }

    #[test]
    fn perpendicular_cope_keeps_the_larger_piece_wherever_its_middle_is() {
        let mut h = Fake::new(vec![(2, "Main 01", hoop_def())], vec!["Main 01"], "in");
        // As for a tube bent back past its own end: the tube's middle lies behind the end, the
        // off-cut's just ahead of it. The tube is the "(1)" piece here.
        h.offcut_at = [-69.85, -300.0, 0.0];
        h.tube_at = [-69.85, 450.0, 0.0];
        h.volumes = Some([290000.0, 900.0]);
        let params = json!({"paths": picks(&["l8"]), "size": "hrew-1.500x0.120", "name": "Brace", "cope": "perpendicular"});
        let out = run(&mut h, &params).unwrap();
        assert_eq!(h.calls[11].1["bodies"], json!(["Brace 01", "Cope Brace 01 to Main 01 face"]));
        assert_eq!(out["tube"]["bodies"], json!(["Brace 01 (1)"]));
    }

    #[test]
    fn cut_length_is_the_stock_to_cut() {
        // A 36 in brace, 1.5 in OD, coped flush at both ends to a 1.75 in hoop: each cope's long
        // point is sqrt(22.225² - 19.05²) = 11.447 mm short of the hoop's centreline.
        let mut h = Fake::new(vec![(2, "Main 01", hoop_def())], vec!["Main 01"], "in");
        let out = run(&mut h, &json!({"paths": picks(&["l8"]), "size": "hrew-1.500x0.120"})).unwrap();
        assert_eq!(out["tube"]["centreline_in"], 36.0);
        let want = (914.4 - 2.0 * (22.225f64.powi(2) - 19.05f64.powi(2)).sqrt()) / 25.4;
        assert!((out["tube"]["cut_length_in"].as_f64().unwrap() - want).abs() < 0.002, "{}", out["tube"]);
        // A mitred tube is cut to its long point, one radius past a square corner.
        let left = json!({"path_sketch": 1, "path": ["l9"], "diameter": "44.45 mm", "wall": "3.048 mm",
                          "operation": "new_body", "body_names": ["Tube 01"]});
        let mut h = Fake::new(vec![(2, "Tube 01", left)], vec!["Tube 01"], "mm");
        let out = run(&mut h, &json!({"paths": picks(&["l1"]), "sharp_joints": "miter"})).unwrap();
        assert_eq!(out["tube"]["centreline_in"], 42.5);
        assert_eq!(out["tube"]["cut_length_in"], round3((1079.5 + 22.225) / 25.4));
        let names: Vec<&str> = h.calls.iter().filter(|(c, a)| c == "solid.pipe" && a["operation"] == "cut").map(|(_, a)| a["name"].as_str().unwrap()).collect();
        assert_eq!(names, ["Miter Tube 01 to Tube 02", "Miter Tube 02 to Tube 01"]);
    }

    #[test]
    fn names_that_read_as_cuts_are_refused() {
        let mut h = Fake::new(vec![], vec![], "mm");
        for name in ["Cope", "Miter", "Cope rail"] {
            assert!(run(&mut h, &json!({"paths": picks(&["l1"]), "name": name})).unwrap_err().starts_with("name:"), "{name}");
        }
        // Forty characters, more than forty bytes.
        let out = run(&mut h, &json!({"paths": picks(&["l1"]), "name": "é".repeat(40)})).unwrap();
        assert!(out["tube"]["name"].as_str().unwrap().ends_with(" 01"));
        assert!(run(&mut h, &json!({"paths": picks(&["l1"]), "name": "é".repeat(41)})).unwrap_err().starts_with("name:"));
        assert_eq!(run(&mut h, &json!({"paths": picks(&["l1"]), "name": "Coper"})).unwrap()["tube"]["name"], "Coper 01");
    }

    #[test]
    fn a_design_too_large_to_read_is_refused_before_it_is_read() {
        let pipes: Vec<(i64, &'static str, Value)> = (0..260).map(|i| (10 + i, "Rail", hoop_def())).collect();
        let mut h = Fake::new(pipes, vec!["Main 01"], "in");
        let e = run(&mut h, &json!({"paths": picks(&["l8"])})).unwrap_err();
        assert!(e.contains("260 pipe features"), "{e}");
        assert_eq!(cmds(&h), ["sketch.inspect", "document.inspect"]);
    }

    #[test]
    fn every_pipe_read_counts_towards_the_budget() {
        // 240 run-ons on one tube: each is a call, though only one tube is found.
        let run_on = json!({"path_sketch": 1, "path": ["l9"], "diameter": "44.45 mm", "operation": "join", "targets": ["Main 01"]});
        let mut pipes = vec![(2, "Main 01", hoop_def())];
        pipes.extend((0..240).map(|i| (10 + i, "Main 01 run-on", run_on.clone())));
        let mut h = Fake::new(pipes, vec!["Main 01"], "in");
        let e = run(&mut h, &json!({"paths": picks(&["l8"])})).unwrap_err();
        assert!(e.contains("SolidCraft steps"), "{e}");
        assert!(!cmds(&h).contains(&"solid.pipe"));
    }

    #[test]
    fn a_pipe_with_no_sketch_path_is_a_warning() {
        let edge = json!({"path": [], "diameter": "44.45 mm", "operation": "new_body", "body_names": ["Main 01"]});
        let mut h = Fake::new(vec![(2, "Main 01", edge)], vec!["Main 01"], "mm");
        let out = run(&mut h, &json!({"paths": picks(&["l8"])})).unwrap();
        assert_eq!(out["earlier_tubes_found"], 0);
        assert!(out["warnings"][0].as_str().unwrap().contains("Main 01"));
        assert_eq!(h.calls.iter().filter(|(c, _)| c == "sketch.inspect").count(), 1);
    }

    #[test]
    fn perpendicular_hoop_after_brace_trims_the_brace() {
        let brace = json!({"path_sketch": 1, "path": ["l8"], "diameter": "38.1 mm", "wall": "3.048 mm",
                           "operation": "new_body", "body_names": ["Tube 01"]});
        let mut h = Fake::new(vec![(2, "Tube 01", brace)], vec!["Tube 01"], "mm");
        let params = json!({"paths": picks(&["l1", "a1", "l2", "a2", "l3"]), "size": "hrew-1.750x0.120", "cope": "perpendicular"});
        let out = run(&mut h, &params).unwrap();
        assert_eq!(h.calls.iter().filter(|(c, _)| c == "solid.split_body").count(), 2);
        assert!(h.calls.iter().filter(|(c, _)| c == "solid.split_body").all(|(_, a)| a["body"] == "Tube 01"));
        assert_eq!(h.calls.iter().filter(|(c, a)| c == "solid.pipe" && a["operation"] == "cut").count(), 0);
        assert_eq!(out["earlier_tubes_coped"], json!(["Tube 01"]));
        assert_eq!(out["perpendicular_copes"][0]["tube"], "Tube 01");
    }

    #[test]
    fn hoop_after_brace_copes_the_brace_end() {
        let brace = json!({"path_sketch": 1, "path": ["l8"], "diameter": "38.1 mm", "wall": "3.048 mm",
                           "operation": "new_body", "body_names": ["Tube 01"]});
        let mut h = Fake::new(vec![(2, "Tube 01", brace)], vec!["Tube 01"], "mm");
        let out = run(&mut h, &json!({"paths": picks(&["l1", "a1", "l2", "a2", "l3"]), "size": "hrew-1.750x0.120"})).unwrap();
        assert_eq!(out["tube"]["name"], "Tube 02");
        let cope = &h.calls[4].1;
        assert_eq!(cope["targets"], json!(["Tube 01"]));
        assert_eq!(cope["path"].as_array().unwrap().len(), 5);
        assert_eq!(out["earlier_tubes_coped"], json!(["Tube 01"]));
        assert!(out["tube"]["coped_to"].as_array().unwrap().is_empty());
    }

    #[test]
    fn free_corner_miter_draws_helpers_and_cuts_both() {
        let left = json!({"path_sketch": 1, "path": ["l9"], "diameter": "44.45 mm", "wall": "3.048 mm",
                          "operation": "new_body", "body_names": ["Tube 01"]});
        let mut h = Fake::new(vec![(2, "Tube 01", left)], vec!["Tube 01"], "mm");
        let out = run(&mut h, &json!({"paths": picks(&["l1"]), "sharp_joints": "miter"})).unwrap();
        assert_eq!(
            cmds(&h),
            ["sketch.inspect", "document.inspect", "timeline.inspect", "solid.pipe", "sketch.create", "sketch.options",
             "sketch.line3d", "sketch.line3d", "sketch.line3d", "sketch.line3d", "sketch.finish",
             "solid.pipe", "solid.pipe", "solid.pipe", "solid.pipe", "timeline.group", "browser.visibility"]
        );
        assert_eq!(h.calls[11].1["operation"], "join");
        assert_eq!(h.calls[11].1["targets"], json!(["Tube 01"]));
        assert_eq!(h.calls[14].1["targets"], json!(["Tube 02"]));
        assert_eq!(out["tube"]["mitred_to"], json!(["Tube 01"]));
    }

    #[test]
    fn earlier_run_on_is_part_of_the_tube() {
        let left = json!({"path_sketch": 1, "path": ["l9"], "diameter": "44.45 mm", "wall": "3.048 mm",
                          "operation": "new_body", "body_names": ["Tube 01"]});
        let run_on = json!({"path_sketch": 1, "path": ["l8"], "diameter": "44.45 mm", "operation": "join", "targets": ["Tube 01"]});
        let mut h = Fake::new(vec![(2, "Tube 01", left), (3, "Tube 01 run-on", run_on)], vec!["Tube 01"], "mm");
        // The diagonal ends on l8, which here is Tube 01's run-on, not a tube of its own.
        let out = run(&mut h, &json!({"paths": picks(&["l7"])})).unwrap();
        let cuts: Vec<&Value> = h.calls.iter().filter(|(c, a)| c == "solid.pipe" && a["operation"] == "cut").map(|(_, a)| a).collect();
        assert_eq!(cuts.len(), 2);
        assert_eq!(cuts[1]["path"], json!(["l8"]));
        assert_eq!(out["earlier_tubes_found"], 1);
    }

    #[test]
    fn bad_input_names_its_field() {
        let mut h = Fake::new(vec![], vec![], "mm");
        assert!(run(&mut h, &json!({"paths": picks(&["l1"]), "size": "nope"})).unwrap_err().starts_with("size:"));
        assert!(run(&mut h, &json!({"paths": picks(&["l1"]), "cope": "laser"})).unwrap_err().starts_with("cope:"));
        assert!(run(&mut h, &json!({"paths": picks(&["zz"])})).unwrap_err().starts_with("paths:"));
        assert!(run(&mut h, &json!({"paths": [{"type": "edge", "body": "B", "index": 1}]})).unwrap_err().starts_with("paths:"));
        assert!(run(&mut h, &json!({"paths": picks(&["l9", "l1"])})).unwrap_err().contains("sharp corner"));
        let two = json!([{"type": "external_sketch_curve", "sketch": 1, "id": "l1"}, {"type": "external_sketch_curve", "sketch": 2, "id": "l2"}]);
        assert!(run(&mut h, &json!({"paths": two})).unwrap_err().contains("one sketch"));
    }
}
