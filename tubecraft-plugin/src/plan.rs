//! Plans one tube: the picked curves as one bent tube, and how it meets the tubes already in
//! the design. No host calls happen here, so every rule is unit-tested natively.
//!
//! Rules:
//! - One run makes **one physical tube**. The picked curves must join end to end and run on
//!   tangent (lines and their bends). A sharp corner means two tubes: make them in two runs.
//! - **A tube end is coped; a tube that runs through a joint is never cut.** Usually the new
//!   tube ends on the earlier one and the new tube is coped. When an earlier tube ends on the
//!   new tube and the new tube runs through, the earlier tube's end is coped instead.
//! - Two tubes that cross (neither ends there): the new tube is cut, in two pieces.
//! - The new tube's end on an earlier tube's end, with no other tube there, is a **free
//!   corner**: a miter when both are the same OD and the dialog asks for one, otherwise a butt
//!   (the earlier tube runs on past the corner and the new tube is coped to it).

use crate::geom::{angle_between, point_poly, poly_poly, Curve, V3};
use crate::profiles::MM_PER_IN;

/// Tangent within this angle (degrees) continues a bent tube.
pub const TANGENT_TOL_DEG: f64 = 0.5;
/// Ends closer than this (mm) meet.
pub const JOIN_TOL_MM: f64 = 0.01;
/// A miter's run-on reaches this far (mm) past its long point, so the cut has material to cut.
pub const MITER_OVER_MM: f64 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SharpJoints {
    Miter,
    Butt,
}

/// How a tube end is cut where it lands on another tube.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CopeStyle {
    /// Cut by the other tube's own outside (a hole-saw cope): the whole wall touches.
    Flush,
    /// Cut square to the tube's own wall, as a tube laser cuts, and trimmed until no part of
    /// the wall is inside the other tube.
    Perpendicular,
}

/// Curves joined end to end, each oriented along the chain.
#[derive(Debug, Clone, PartialEq)]
pub struct Chain {
    pub curves: Vec<(String, Curve)>,
}

impl Chain {
    pub fn start(&self) -> V3 {
        self.curves[0].1.start()
    }
    pub fn end(&self) -> V3 {
        self.curves.last().unwrap().1.end()
    }
    pub fn closed(&self) -> bool {
        self.curves.len() > 1 && self.start().dist(self.end()) < JOIN_TOL_MM
    }
    pub fn length(&self) -> f64 {
        self.curves.iter().map(|(_, c)| c.length()).sum()
    }
    pub fn ids(&self) -> Vec<String> {
        self.curves.iter().map(|(id, _)| id.clone()).collect()
    }
    pub fn polyline(&self) -> Vec<V3> {
        let mut out: Vec<V3> = Vec::new();
        for (_, c) in &self.curves {
            let skip = usize::from(!out.is_empty());
            out.extend(c.polyline().into_iter().skip(skip));
        }
        out
    }
    /// The open ends: each point with the direction that leads from it into the tube.
    pub fn ends(&self) -> Vec<(V3, V3)> {
        if self.closed() {
            return Vec::new();
        }
        let last = &self.curves.last().unwrap().1;
        vec![(self.start(), self.curves[0].1.tangent(0.0)), (self.end(), -last.tangent(1.0))]
    }
    /// Sharp joints between consecutive curves, as pairs of curve ids. A closed chain's last
    /// curve is followed by its first.
    pub fn sharp_joints(&self) -> Vec<(String, String)> {
        let n = self.curves.len();
        let pairs = if self.closed() { n } else { n - 1 };
        (0..pairs)
            .map(|i| (&self.curves[i], &self.curves[(i + 1) % n]))
            .filter(|(a, b)| angle_between(a.1.tangent(1.0), b.1.tangent(0.0)).to_degrees() > TANGENT_TOL_DEG)
            .map(|(a, b)| (a.0.clone(), b.0.clone()))
            .collect()
    }
    /// Points along the chain no more than `step` mm apart, from its start to its end.
    pub fn samples(&self, step: f64) -> Vec<V3> {
        let mut out = vec![self.start()];
        for (_, c) in &self.curves {
            let k = (c.length() / step).ceil().max(1.0) as usize;
            out.extend((1..=k).map(|i| c.point(i as f64 / k as f64)));
        }
        out
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ChainError {
    /// This curve meets neither end of the others.
    Apart(String),
}

/// Joins curves end to end, in any pick order and drawn either way.
pub fn chain(curves: &[(String, Curve)]) -> Result<Chain, ChainError> {
    let mut rest: Vec<(String, Curve)> = curves.to_vec();
    let mut out = vec![rest.remove(0)];
    let near = |a: V3, b: V3| a.dist(b) < JOIN_TOL_MM;
    while !rest.is_empty() {
        let head = out[0].1.start();
        let tail = out.last().unwrap().1.end();
        let found = rest.iter().position(|(_, c)| {
            near(c.start(), tail) || near(c.end(), tail) || near(c.start(), head) || near(c.end(), head)
        });
        let Some(i) = found else { return Err(ChainError::Apart(rest[0].0.clone())) };
        let (id, c) = rest.remove(i);
        if near(c.start(), tail) {
            out.push((id, c));
        } else if near(c.end(), tail) {
            out.push((id, c.reversed()));
        } else if near(c.end(), head) {
            out.insert(0, (id, c));
        } else {
            out.insert(0, (id, c.reversed()));
        }
    }
    Ok(Chain { curves: out })
}

/// A tube that is already in the design.
#[derive(Debug, Clone, PartialEq)]
pub struct Existing {
    /// Its pipe feature's name, as the user sees it.
    pub name: String,
    /// Its bodies (a tube cut through by a crossing is in two).
    pub bodies: Vec<String>,
    pub od_mm: f64,
    /// `None` for a solid bar.
    pub wall_mm: Option<f64>,
    pub sketch: i64,
    /// Its centreline; `None` when the path is not one chain (its ends are then unknown).
    pub chain: Option<Chain>,
    pub polyline: Vec<V3>,
    /// Run-ons joined to it by earlier runs: (sketch, curve ids, polyline).
    pub run_ons: Vec<(i64, Vec<String>, Vec<V3>)>,
    /// The curves its pipe follows, to refuse picking them again.
    pub ids: Vec<String>,
}

impl Existing {
    pub(crate) fn r(&self) -> f64 {
        self.od_mm / 2.0
    }
    fn near(&self, poly: &[V3]) -> f64 {
        self.run_ons.iter().map(|(_, _, p)| poly_poly(p, poly)).fold(poly_poly(&self.polyline, poly), f64::min)
    }
    pub(crate) fn point_near(&self, p: V3) -> f64 {
        self.run_ons.iter().map(|(_, _, q)| point_poly(p, q)).fold(point_poly(p, &self.polyline), f64::min)
    }
    /// Its open ends, with the direction from each end into the tube.
    pub(crate) fn ends(&self) -> Vec<(V3, V3)> {
        self.chain.as_ref().map(Chain::ends).unwrap_or_default()
    }
}

#[derive(Debug, Clone)]
pub struct Options {
    pub od_mm: f64,
    pub wall_mm: f64,
    /// Weld gap left at every cope, mm.
    pub gap_mm: f64,
    /// The bender die's centreline radius, mm; bends of another radius are reported.
    pub clr_mm: f64,
    pub sharp: SharpJoints,
    pub cope: CopeStyle,
}

/// A straight helper line, in world space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seg {
    pub from: V3,
    pub to: V3,
}

impl Seg {
    pub fn len(&self) -> f64 {
        self.from.dist(self.to)
    }
}

/// A miter with an earlier tube of the same OD at a free corner.
#[derive(Debug, Clone, PartialEq)]
pub struct Miter {
    pub with: usize,
    /// The earlier tube runs on past the corner, and the new one starts before it, so the
    /// miter's long point has material.
    pub run_on_old: Seg,
    pub run_on_new: Seg,
    /// Cylinders whose flat end is the miter plane; each cuts its own tube's far side off.
    pub cut_old: Seg,
    pub cut_new: Seg,
    pub cutter_dia: f64,
}

/// A butt with an earlier tube at a free corner: the earlier tube runs on, the new tube is
/// coped to it.
#[derive(Debug, Clone, PartialEq)]
pub struct Butt {
    pub with: usize,
    pub run_on_old: Seg,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Bend {
    pub curve: String,
    pub angle_deg: f64,
    pub clr_mm: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub chain: Chain,
    pub bends: Vec<Bend>,
    /// Earlier tubes the new tube is coped to (its ends land on them, or it crosses them).
    pub cope_new_to: Vec<usize>,
    /// Those of `cope_new_to` the new tube crosses away from its ends: it is cut in two there.
    pub crossings: Vec<usize>,
    /// Earlier tubes whose ends are coped to the new tube, which runs through.
    pub cope_old_to_new: Vec<usize>,
    pub miters: Vec<Miter>,
    pub butts: Vec<Butt>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanError {
    pub field: &'static str,
    pub reason: String,
}

fn err(field: &'static str, reason: impl Into<String>) -> PlanError {
    PlanError { field, reason: reason.into() }
}

pub fn inches(mm: f64) -> String {
    format!("{:.3} in", mm / MM_PER_IN)
}

/// Plans the new tube `name` along `curves` (all from one sketch) against `existing`.
pub fn build(name: &str, sketch: i64, curves: &[(String, Curve)], existing: &[Existing], opt: &Options) -> Result<Plan, PlanError> {
    if curves.is_empty() {
        return Err(err("paths", "pick the sketch lines and arcs of one tube"));
    }
    for (i, (id, _)) in curves.iter().enumerate() {
        if curves[..i].iter().any(|(j, _)| j == id) {
            return Err(err("paths", format!("curve {id} is picked twice")));
        }
        if let Some(e) = existing.iter().find(|e| e.sketch == sketch && e.ids.contains(id)) {
            return Err(err("paths", format!("curve {id} is already the path of {}", e.name)));
        }
    }
    let chain = chain(curves).map_err(|ChainError::Apart(id)| {
        err("paths", format!("curve {id} does not join the other curves end to end: pick the curves of one tube"))
    })?;
    if let Some((a, b)) = chain.sharp_joints().first() {
        return Err(err(
            "paths",
            format!("{a} and {b} meet at a sharp corner: one run makes one bent tube, so make each side of the corner as its own tube"),
        ));
    }

    let r = opt.od_mm / 2.0;
    let mut warnings = Vec::new();
    let mut bends = Vec::new();
    for (id, c) in &chain.curves {
        if let Curve::Arc { r: rad, sweep, .. } = *c {
            if rad <= r * 1.0001 {
                return Err(err("paths", format!("the bend {id} has a radius of {}, not more than the tube's own radius", inches(rad))));
            }
            if (rad - opt.clr_mm).abs() > opt.clr_mm * 0.005 {
                warnings.push(format!("bend {id} has a CLR of {}, the die is {}", inches(rad), inches(opt.clr_mm)));
            }
            bends.push(Bend { curve: id.clone(), angle_deg: sweep.to_degrees(), clr_mm: rad });
        }
    }

    let poly = chain.polyline();
    let ends = chain.ends();
    let mut plan = Plan {
        chain: chain.clone(),
        bends,
        cope_new_to: Vec::new(),
        crossings: Vec::new(),
        cope_old_to_new: Vec::new(),
        miters: Vec::new(),
        butts: Vec::new(),
        warnings,
    };

    for (k, e) in existing.iter().enumerate() {
        let reach = r + e.r() - 0.05;
        if e.near(&poly) >= reach {
            continue;
        }
        // A free corner: an end of the new tube on an end of this one, and no third tube there.
        let corner = ends.iter().find_map(|&(p, b)| {
            e.ends().into_iter().find(|&(q, _)| q.dist(p) < 0.5).map(|(_, into_old)| (p, b, -into_old))
        });
        if let Some((p, b, a)) = corner {
            let node = existing.iter().enumerate().any(|(j, o)| j != k && o.point_near(p) < o.r().max(r));
            if !node {
                corner_joint(&mut plan, name, k, e, p, a, b, opt);
                continue;
            }
        }
        let new_ends_on_old = ends.iter().any(|&(p, _)| e.point_near(p) < reach);
        let old_ends_on_new = e.ends().iter().any(|&(q, _)| point_poly(q, &poly) < reach);
        if !new_ends_on_old && old_ends_on_new {
            plan.cope_old_to_new.push(k);
            if e.od_mm > opt.od_mm + 0.01 {
                plan.warnings.push(format!("{} ({} OD) is coped to {name}, which is smaller", e.name, inches(e.od_mm)));
            }
            continue;
        }
        if !new_ends_on_old || crosses_away_from_ends(&chain, e, reach) {
            plan.warnings.push(format!("{name} crosses {}, so {name} is cut in two", e.name));
            plan.crossings.push(k);
        }
        plan.cope_new_to.push(k);
        if opt.od_mm > e.od_mm + 0.01 {
            plan.warnings.push(format!("{name} ({} OD) is coped to {}, which is smaller", inches(opt.od_mm), e.name));
        }
    }
    Ok(plan)
}

/// Whether the tube along `chain` meets `e` (centrelines closer than `reach`) somewhere other
/// than in the stretch that runs from each of its ends: a crossing besides an end that lands.
fn crosses_away_from_ends(chain: &Chain, e: &Existing, reach: f64) -> bool {
    let within: Vec<bool> = chain.samples(2.0).into_iter().map(|p| e.point_near(p) < reach).collect();
    if chain.closed() {
        return within.contains(&true);
    }
    let head = within.iter().take_while(|w| **w).count();
    let tail = within.iter().rev().take_while(|w| **w).count();
    head + tail < within.len() && within[head..within.len() - tail].contains(&true)
}

/// A miter or butt where the new tube's end `p` (leading into it along `b`) meets the end of
/// earlier tube `k`, which arrives at `p` along `a`.
#[allow(clippy::too_many_arguments)]
fn corner_joint(plan: &mut Plan, name: &str, k: usize, e: &Existing, p: V3, a: V3, b: V3, opt: &Options) {
    let turn = angle_between(a, b);
    if turn.to_degrees() <= TANGENT_TOL_DEG {
        plan.warnings.push(format!("{name} continues {} in a straight line: the joint is left square", e.name));
        return;
    }
    if turn.to_degrees() > 179.0 {
        plan.warnings.push(format!("{name} turns back on {}: no joint is made", e.name));
        return;
    }
    let same = (e.od_mm - opt.od_mm).abs() < 0.01;
    let mut style = opt.sharp;
    if style == SharpJoints::Miter && !same {
        plan.warnings.push(format!("{name} and {} are not the same OD, so their corner is butted, not mitred", e.name));
        style = SharpJoints::Butt;
    }
    match style {
        SharpJoints::Miter => {
            let r = opt.od_mm / 2.0;
            if turn.to_degrees() > 120.0 {
                plan.warnings.push(format!(
                    "the miter between {} and {name} turns {:.0}°: its long point is long and thin",
                    e.name,
                    turn.to_degrees()
                ));
            }
            let long = r * (turn / 2.0).tan() + MITER_OVER_MM;
            let n = (a + b).norm();
            let reach = long + r + 2.0;
            plan.miters.push(Miter {
                with: k,
                run_on_old: Seg { from: p, to: p + a * long },
                run_on_new: Seg { from: p - b * long, to: p },
                cut_old: Seg { from: p, to: p + n * reach },
                cut_new: Seg { from: p, to: p - n * reach },
                cutter_dia: 2.0 * reach,
            });
        }
        SharpJoints::Butt => {
            let run = butt_run_on(a, b, e.r(), opt.od_mm / 2.0);
            if turn.to_degrees() > 150.0 {
                plan.warnings.push(format!(
                    "{name} meets {} at {:.0}°: the butt needs {} of run-on",
                    e.name,
                    180.0 - turn.to_degrees(),
                    inches(run)
                ));
            }
            if opt.od_mm > e.od_mm + 0.01 {
                plan.warnings.push(format!("{name} ({} OD) is butted to {}, which is smaller", inches(opt.od_mm), e.name));
            }
            plan.butts.push(Butt { with: k, run_on_old: Seg { from: p, to: p + a * run } });
            plan.cope_new_to.push(k);
        }
    }
}

/// How far a through tube (radius `r_through`, running on along `a`) must run past a corner to
/// cover the end of a tube of radius `r_other` that leaves the corner along `b`: the furthest
/// reach, along `a`, of the other tube's outside inside the through tube.
pub fn butt_run_on(a: V3, b: V3, r_through: f64, r_other: f64) -> f64 {
    let a = a.norm();
    let b = b.norm();
    let p = b.any_perp();
    let q = b.cross(p);
    let mut best: f64 = 0.0;
    let reach = 6.0 * r_through.max(r_other) / (angle_between(a, b).sin().max(0.1));
    for i in 0..=400 {
        let s = reach * i as f64 / 400.0;
        for k in 0..72 {
            let phi = std::f64::consts::TAU * k as f64 / 72.0;
            let x = b * s + (p * phi.cos() + q * phi.sin()) * r_other;
            let along = x.dot(a);
            let off = (x - a * along).len();
            if off <= r_through {
                best = best.max(along);
            }
        }
    }
    best
}

impl Plan {
    /// The helper lines this plan draws.
    pub fn helper_lines(&self) -> usize {
        self.miters.len() * 4 + self.butts.len()
    }

    /// Host calls the build makes, given the earlier tubes: `flush_new` are those the new tube is
    /// cut to by their own outside, `flush_old` how many earlier tubes are cut by the new one's.
    pub fn host_calls(&self, existing: &[Existing], flush_new: &[usize], flush_old: usize) -> usize {
        let helpers = if self.helper_lines() == 0 { 0 } else { 3 + self.helper_lines() };
        let cope_new: usize = flush_new
            .iter()
            .map(|&k| 1 + existing[k].run_ons.len() + self.butts.iter().filter(|b| b.with == k).count())
            .sum();
        // Pipe, helpers, miters (two run-ons, two cuts), butt run-ons, copes, folder, and one
        // call to hide the helper sketches (counted whether or not there are any).
        1 + helpers + self.miters.len() * 4 + self.butts.len() + cope_new + flush_old + 2
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::geom::v3;

    const R175: f64 = 22.225;

    pub(crate) fn opts(od_mm: f64, sharp: SharpJoints) -> Options {
        Options { od_mm, wall_mm: 3.048, gap_mm: 0.0, clr_mm: 139.7, sharp, cope: CopeStyle::Flush }
    }

    pub(crate) fn line(id: &str, a: [f64; 2], b: [f64; 2]) -> (String, Curve) {
        (id.into(), Curve::Line { a: v3(a[0], a[1], 0.0), b: v3(b[0], b[1], 0.0) })
    }

    pub(crate) fn arc(id: &str, c: [f64; 2], r: f64, start_deg: f64, sweep_deg: f64) -> (String, Curve) {
        let t = start_deg.to_radians();
        let cc = v3(c[0], c[1], 0.0);
        (id.into(), Curve::Arc { c: cc, r, n: v3(0.0, 0.0, 1.0), a: cc + v3(t.cos(), t.sin(), 0.0) * r, sweep: sweep_deg.to_radians() })
    }

    /// The example sketch (examples/example-wireframe.sc): an open hoop with two R5.5" bends, a
    /// vertical brace and a diagonal from the lower bend to the brace's top.
    pub(crate) fn hoop() -> Vec<(String, Curve)> {
        vec![
            line("l1", [-609.6, -457.2], [469.9, -457.2]),
            arc("a1", [469.9, -317.5], 139.7, -90.0, 90.0),
            line("l2", [609.6, -317.5], [609.6, 317.5]),
            arc("a2", [469.9, 317.5], 139.7, 0.0, 90.0),
            line("l3", [469.9, 457.2], [-609.6, 457.2]),
        ]
    }
    pub(crate) fn brace() -> Vec<(String, Curve)> {
        vec![line("l8", [-69.85, 457.2], [-69.85, -457.2])]
    }
    pub(crate) fn diagonal() -> Vec<(String, Curve)> {
        vec![line("l7", [562.637989461379, -421.97849209603436], [-69.85, 457.2])]
    }

    pub(crate) fn existing(name: &str, od: f64, curves: &[(String, Curve)]) -> Existing {
        let chain = chain(curves).ok();
        let polyline = chain.as_ref().map(Chain::polyline).unwrap();
        Existing {
            name: name.into(),
            bodies: vec![name.into()],
            od_mm: od,
            wall_mm: Some(3.048),
            sketch: 1,
            chain,
            polyline,
            run_ons: Vec::new(),
            ids: curves.iter().map(|(id, _)| id.clone()).collect(),
        }
    }

    #[test]
    fn hoop_alone_is_one_bent_tube() {
        let plan = build("Tube 01", 1, &hoop(), &[], &opts(44.45, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.chain.ids(), ["l1", "a1", "l2", "a2", "l3"]);
        assert_eq!(plan.bends.len(), 2);
        assert!((plan.bends[0].angle_deg - 90.0).abs() < 1e-9);
        assert!(plan.cope_new_to.is_empty() && plan.warnings.is_empty());
    }

    #[test]
    fn picks_in_any_order_and_direction_make_one_chain() {
        let mut h = hoop();
        h.swap(0, 3);
        h[1].1 = h[1].1.reversed();
        let plan = build("Tube 01", 1, &h, &[], &opts(44.45, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.chain.curves.len(), 5);
        assert!(plan.chain.sharp_joints().is_empty());
        assert!((plan.chain.length() - chain(&hoop()).unwrap().length()).abs() < 1e-9);
    }

    #[test]
    fn mixed_sizes_brace_then_diagonal_are_coped_to_what_came_before() {
        let old = vec![existing("Hoop", 44.45, &hoop())];
        let plan = build("Brace", 1, &brace(), &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.cope_new_to, vec![0]);
        assert!(plan.cope_old_to_new.is_empty());
        assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);

        let old = vec![existing("Hoop", 44.45, &hoop()), existing("Brace", 38.1, &brace())];
        let plan = build("Diagonal", 1, &diagonal(), &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        // Both its ends land on the hoop; its top is at the brace's top too: a node, so no miter.
        assert_eq!(plan.cope_new_to, vec![0, 1]);
        assert!(plan.miters.is_empty() && plan.butts.is_empty());
    }

    #[test]
    fn through_tube_made_last_copes_the_earlier_end() {
        // Rule B: the brace exists, the hoop runs through its end and stays whole.
        let old = vec![existing("Brace", 38.1, &brace())];
        let plan = build("Hoop", 1, &hoop(), &old, &opts(44.45, SharpJoints::Miter)).unwrap();
        assert!(plan.cope_new_to.is_empty());
        assert_eq!(plan.cope_old_to_new, vec![0]);
        assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
    }

    #[test]
    fn crossing_cuts_the_new_tube_with_a_warning() {
        let old = vec![existing("A", 38.1, &[line("l1", [-300.0, 0.0], [300.0, 0.0])])];
        let plan = build("B", 1, &[line("l2", [0.0, -300.0], [0.0, 300.0])], &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.cope_new_to, vec![0]);
        assert!(plan.warnings[0].contains("crosses"));
    }

    #[test]
    fn an_end_that_lands_and_a_crossing_further_on_are_both_found() {
        // B starts on rail A1 and runs up through rail A2 and past it.
        let rails = [line("l1", [-300.0, 0.0], [300.0, 0.0]), line("l3", [-300.0, 400.0], [300.0, 400.0])];
        let old = vec![existing("A1", 38.1, &rails[..1]), existing("A2", 38.1, &rails[1..])];
        let plan = build("B", 1, &[line("l2", [0.0, 0.0], [0.0, 600.0])], &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.cope_new_to, vec![0, 1]);
        assert_eq!(plan.crossings, vec![1]);
        // One rail bent round so that B lands on it and crosses it too.
        let u = [line("l1", [-300.0, 0.0], [300.0, 0.0]), arc("a1", [300.0, 200.0], 200.0, -90.0, 180.0), line("l3", [300.0, 400.0], [-300.0, 400.0])];
        let old = vec![existing("U", 38.1, &u)];
        let plan = build("B", 1, &[line("l2", [0.0, 0.0], [0.0, 600.0])], &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.crossings, vec![0]);
        assert!(plan.warnings.iter().any(|w| w.contains("crosses")));
        // An end that only lands is not a crossing.
        let plan = build("B", 1, &[line("l2", [0.0, 0.0], [0.0, 300.0])], &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert!(plan.crossings.is_empty() && plan.warnings.is_empty());
    }

    #[test]
    fn free_corner_miter_same_od() {
        let old = vec![existing("A", 44.45, &[line("l1", [-300.0, 0.0], [0.0, 0.0])])];
        let plan = build("B", 1, &[line("l2", [0.0, 300.0], [0.0, 0.0])], &old, &opts(44.45, SharpJoints::Miter)).unwrap();
        assert_eq!(plan.miters.len(), 1);
        assert!(plan.cope_new_to.is_empty());
        let m = &plan.miters[0];
        // 90° turn: the long point is r past the corner (plus 1 mm).
        assert!((m.run_on_old.len() - (R175 + 1.0)).abs() < 1e-9);
        assert!(m.run_on_old.to.near(v3(R175 + 1.0, 0.0, 0.0)));
        assert!(m.run_on_new.from.near(v3(0.0, -(R175 + 1.0), 0.0)));
        // The cutters leave the corner normal to the miter plane, each away from its tube.
        let n = (m.cut_old.to - m.cut_old.from).norm();
        assert!(n.near(v3(1.0, 1.0, 0.0).norm()), "{n:?}");
        assert!((m.cut_new.to - m.cut_new.from).norm().near(-n));
    }

    #[test]
    fn free_corner_butt_and_mixed_od_falls_back_to_butt() {
        let old = vec![existing("A", 44.45, &[line("l1", [-300.0, 0.0], [0.0, 0.0])])];
        let new = [line("l2", [0.0, 0.0], [0.0, 300.0])];
        let plan = build("B", 1, &new, &old, &opts(44.45, SharpJoints::Butt)).unwrap();
        assert_eq!(plan.butts.len(), 1);
        assert_eq!(plan.cope_new_to, vec![0]);
        // A square butt runs the through tube on by its radius.
        assert!((plan.butts[0].run_on_old.to.x - R175).abs() < 0.05);

        let plan = build("B", 1, &new, &old, &opts(38.1, SharpJoints::Miter)).unwrap();
        assert!(plan.miters.is_empty());
        assert_eq!(plan.butts.len(), 1);
        assert!(plan.warnings[0].contains("not the same OD"));
        // The smaller tube's outside lands inside the larger tube's run-on.
        assert!((plan.butts[0].run_on_old.to.x - 19.05).abs() < 0.05);
    }

    #[test]
    fn refusals() {
        let o = opts(44.45, SharpJoints::Miter);
        // A sharp corner is two tubes.
        let e = build("T", 1, &[line("l1", [0.0, 0.0], [100.0, 0.0]), line("l2", [100.0, 0.0], [100.0, 100.0])], &[], &o).unwrap_err();
        assert!(e.reason.contains("sharp corner"), "{e:?}");
        // Curves that do not touch.
        let e = build("T", 1, &[line("l1", [0.0, 0.0], [100.0, 0.0]), line("l2", [200.0, 0.0], [300.0, 0.0])], &[], &o).unwrap_err();
        assert!(e.reason.contains("does not join"));
        // Picked twice.
        let e = build("T", 1, &[line("l1", [0.0, 0.0], [100.0, 0.0]), line("l1", [0.0, 0.0], [100.0, 0.0])], &[], &o).unwrap_err();
        assert!(e.reason.contains("picked twice"));
        // Already a tube.
        let old = vec![existing("Brace", 38.1, &brace())];
        let e = build("T", 1, &brace(), &old, &o).unwrap_err();
        assert!(e.reason.contains("already the path of Brace"));
        // A closed loop with a sharp corner where its last curve meets its first.
        let (t, tip) = (100.0 * (0.5f64).sqrt(), -100.0 * (2.0f64).sqrt());
        let drop = [line("l1", [0.0, tip], [t, -t]), arc("a1", [0.0, 0.0], 100.0, -45.0, 270.0), line("l2", [-t, -t], [0.0, tip])];
        let e = build("T", 1, &drop, &[], &o).unwrap_err();
        assert!(e.reason.contains("sharp corner"), "{e:?}");
        // A bend tighter than the tube.
        let e = build("T", 1, &[line("l1", [0.0, 0.0], [100.0, 0.0]), arc("a1", [100.0, 20.0], 20.0, -90.0, 90.0)], &[], &o).unwrap_err();
        assert!(e.reason.contains("radius"));
    }

    #[test]
    fn other_clr_is_a_warning() {
        let plan = build(
            "T",
            1,
            &[line("l1", [0.0, 0.0], [100.0, 0.0]), arc("a1", [100.0, 100.0], 100.0, -90.0, 90.0)],
            &[],
            &opts(44.45, SharpJoints::Miter),
        )
        .unwrap();
        assert_eq!(plan.warnings.len(), 1);
        assert!(plan.warnings[0].contains("CLR"));
    }

    #[test]
    fn butt_run_on_grows_for_shallow_angles() {
        let a = v3(1.0, 0.0, 0.0);
        let square = butt_run_on(a, v3(0.0, 1.0, 0.0), 10.0, 10.0);
        assert!((square - 10.0).abs() < 0.05);
        let t = (60.0f64).to_radians();
        assert!(butt_run_on(a, v3(t.cos(), t.sin(), 0.0), 10.0, 10.0) > square);
    }
}
