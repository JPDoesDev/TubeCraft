//! The perpendicular cope: a tube end cut square to its own wall, as a CNC tube laser cuts it,
//! and trimmed back until no part of the wall is inside the tubes it lands on.
//!
//! A laser's beam always points at the tube's axis, so the cut face is made of radial lines:
//! at each angle round the tube the wall is cut at one distance from the end. That distance is
//! the furthest any fibre of the wall (inside, outside or between) reaches into another tube,
//! so the trimmed end touches the other tube along one edge and clears it everywhere else.
//!
//! The cut is built as a surface lofted through those radial lines (SolidCraft refuses a loft
//! through closed 3D curves, and a ruled strip that starts on the body's own edge; a loft
//! through open lines that overshoot the wall on both sides is accepted and splits the tube).
//!
//! The loft takes at most 50 sections and is a smooth surface through them, so the trim is
//! worked out at many more angles than there are lines. The lines are spread evenly along the
//! cut's own length (closer in angle where the cut is steep, as at the heel of a shallow cope);
//! SolidCraft's kernel refuses the surface when lines are bunched. Where a peak falls between two
//! lines, or the surface would swing below a crease (two tubes of a node taking over from each
//! other), only the lines beside it are lifted. So the trimmed end clears the other tubes
//! everywhere, by a few hundredths of a millimetre on a plain joint and a few tenths at the
//! point of a crease.
//! No host calls happen here.

use crate::geom::{point_poly, Curve, V3};
use crate::plan::{inches, Chain, CopeStyle, Existing, Options, Plan, Seg};

/// Radial lines round the tube. The loft takes 50 sections; the last repeats the first.
pub const STATIONS: usize = 48;
/// Angles round the tube the trim is worked out at; the lines are chosen among them.
const FINE: usize = 720;
/// Clearance added to the trim on top of what the lines' spacing misses, mm.
const CLEAR_MM: f64 = 0.02;
/// The loft is a smooth surface through the lines, so beside a crease it swings past them, by
/// about this share of the crease's step from one line to the next (measured on SolidCraft
/// 0.48.1: 0.066; a cubic through a corner gives the same). The lines there are lifted by it.
const RING: f64 = 0.12;
/// A tube known only as a polyline is this much larger than it reads, mm (the chord error).
const CHORD_MM: f64 = 0.05;
/// Host calls one perpendicular cope makes: sketch, 3D mode, lines, finish, loft, split,
/// measure, remove.
pub const CALLS_PER_COPE: usize = 8;
/// The radial lines overshoot the wall by this much on each side, mm.
const OVER_MM: f64 = 1.5;
/// Where a station's wall never enters another tube, the cut lies this far beyond the end.
const MISS_MM: f64 = -2.0;
/// Fibres checked across the wall, inside to outside.
const FIBRES: usize = 5;
/// The search step along a fibre, mm (then bisected).
const STEP_MM: f64 = 1.0;

/// A tube the end lands on: its centrelines (exact curves, polylines or both), and its outside
/// radius with the weld gap added.
pub struct Other<'a> {
    pub curves: Vec<Curve>,
    pub polys: Vec<&'a [V3]>,
    pub r: f64,
}

impl Other<'_> {
    fn dist(&self, p: V3) -> f64 {
        let d = self.curves.iter().map(|c| c.dist(p)).fold(f64::INFINITY, f64::min);
        self.polys.iter().map(|q| point_poly(p, q)).fold(d, f64::min)
    }
}

/// The steps from angle `a` to angle `b`, going round.
fn span(n: usize, a: usize, b: usize) -> usize {
    if b > a {
        b - a
    } else {
        b + n - a
    }
}

/// `STATIONS` of the `f.len()` angles, going round once from the seam: evenly spread along the
/// cut's length, on a tube of mean radius `r`. The seam, where the surface's two ends meet, is
/// put where the cut runs straightest.
fn stations(f: &[f64], r: f64) -> Vec<usize> {
    let n = f.len();
    let step = std::f64::consts::TAU * r / n as f64;
    let reach = n / STATIONS;
    let bend = |j: usize| (f[(j + reach) % n] - 2.0 * f[j] + f[(j + n - reach) % n]).abs();
    let seam = (1..n).fold(0, |best, j| if bend(j) < bend(best) { j } else { best });
    // The length along the cut from the seam to each angle.
    let mut along = vec![0.0; n + 1];
    for s in 0..n {
        let (a, b) = (f[(seam + s) % n], f[(seam + s + 1) % n]);
        along[s + 1] = along[s] + step.hypot(b - a);
    }
    let mut out: Vec<usize> = Vec::with_capacity(STATIONS);
    let mut s = 0;
    for i in 0..STATIONS {
        let want = along[n] * i as f64 / STATIONS as f64;
        while s + 1 < n && along[s + 1] <= want {
            s += 1;
        }
        // Never the same angle twice, and room left for the lines still to come.
        s = s.max(i).min(n - (STATIONS - i));
        out.push((seam + s) % n);
        s += 1;
    }
    out
}

/// For each chosen line, the most `f` rises above the straight runs to its two neighbours: a
/// peak between lines that the surface would cut through.
fn peak_lift(f: &[f64], chosen: &[usize]) -> Vec<f64> {
    let (n, m) = (f.len(), chosen.len());
    let over: Vec<f64> = (0..m)
        .map(|w| {
            let (a, b) = (chosen[w], chosen[(w + 1) % m]);
            let len = span(n, a, b);
            (1..len).fold(0.0f64, |worst, s| worst.max(f[(a + s) % n] - (f[a] + (f[b] - f[a]) * s as f64 / len as f64)))
        })
        .collect();
    (0..m).map(|i| over[i].max(over[(i + m - 1) % m])).collect()
}

/// How far each line is lifted so the smooth surface through the lines (at heights `g`) does
/// not swing below them beside a crease: `RING` of the sharpest turn nearby, halving with each
/// line away.
fn ring_lift(n: usize, g: &[f64], chosen: &[usize]) -> Vec<f64> {
    let m = chosen.len();
    let turn: Vec<f64> = (0..m)
        .map(|i| {
            let (a, c, b) = (chosen[(i + m - 1) % m], chosen[i], chosen[(i + 1) % m]);
            let (da, db) = (span(n, a, c) as f64, span(n, c, b) as f64);
            let (ga, gc, gb) = (g[(i + m - 1) % m], g[i], g[(i + 1) % m]);
            ((gb - gc) / db - (gc - ga) / da).abs() * da.min(db)
        })
        .collect();
    (0..m)
        .map(|i| {
            // A crease between two lines shows as half a turn at each: count a line with its
            // sharper neighbour.
            let pair = |k: usize| turn[k] + turn[(k + 1) % m].max(turn[(k + m - 1) % m]);
            (0..=3usize).fold(0.0f64, |best, d| {
                let near = pair((i + d) % m).max(pair((i + m - d) % m));
                best.max(RING * near * 0.5f64.powi(d as i32))
            })
        })
        .collect()
}

/// One perpendicular cope: the radial lines of its cut face.
#[derive(Debug, Clone, PartialEq)]
pub struct Cope {
    /// `STATIONS + 1` lines from inside the bore to outside the tube; the last repeats the first.
    pub lines: Vec<Seg>,
    /// A point on the cut face inside the wall, to pick the face by.
    pub pick: V3,
    /// The least and the most trimmed off the end, along the tube, mm: the cope's long and
    /// short points.
    pub trim_min: f64,
    pub trim_max: f64,
    /// The most of that which is clearance (it is less away from creases): what the spacing of
    /// the lines and the smooth surface through them could leave inside the other tube, mm.
    pub clearance: f64,
}

impl Cope {
    /// The lines as one zigzag polyline (in, out, out, in, ...): its odd segments (first,
    /// third, ...) are the radial lines, so one `sketch.line3d` call draws them all.
    pub fn zigzag(&self) -> Vec<V3> {
        let mut out = Vec::with_capacity(self.lines.len() * 2);
        for (i, l) in self.lines.iter().enumerate() {
            if i % 2 == 0 {
                out.extend([l.from, l.to]);
            } else {
                out.extend([l.to, l.from]);
            }
        }
        out
    }
}

/// Why an end gets no perpendicular cope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// The wall never enters another tube: nothing to cut.
    Clear,
    /// The cope would be longer than the straight end allows (a shallow angle, or a short end).
    Long,
}

/// The perpendicular cope of a tube end at `end`, whose straight end piece leads into the tube
/// along `into` for `straight` mm, against `others`. `ro` and `ri` are the tube's radii.
pub fn cope(end: V3, into: V3, ro: f64, ri: f64, straight: f64, others: &[Other]) -> Result<Cope, Refused> {
    let b = into.norm();
    let u = b.any_perp();
    let v = b.cross(u);
    let r_max = others.iter().map(|o| o.r).fold(0.0, f64::max);
    let limit = straight.min(8.0 * (r_max + ro));

    let Some(reach) = reach_along(end, b, ro, limit, others) else { return Err(Refused::Clear) };
    if reach + STEP_MM > limit {
        return Err(Refused::Long);
    }
    let top = reach + STEP_MM;

    let radial = |j: usize| {
        let th = std::f64::consts::TAU * j as f64 / FINE as f64;
        u * th.cos() + v * th.sin()
    };
    let mut fine = Vec::with_capacity(FINE);
    for j in 0..FINE {
        let rad = radial(j);
        let mut trim = MISS_MM;
        for j in 0..FIBRES {
            let rho = ri + (ro - ri) * j as f64 / (FIBRES - 1) as f64;
            if let Some(t) = fibre_exit(end + rad * rho, b, top, others) {
                trim = trim.max(t);
            }
        }
        fine.push(trim);
    }
    if fine.iter().all(|t| *t <= 0.0) {
        return Err(Refused::Clear);
    }
    let chosen = stations(&fine, 0.5 * (ro + ri));
    let peak = peak_lift(&fine, &chosen);
    let raised: Vec<f64> = chosen.iter().zip(&peak).map(|(&j, p)| fine[j] + p).collect();
    let ring = ring_lift(FINE, &raised, &chosen);
    let lift: Vec<f64> = peak.iter().zip(&ring).map(|(p, r)| p + r + CLEAR_MM).collect();
    let extra = lift.iter().copied().fold(0.0, f64::max);
    let trims: Vec<(V3, f64)> = chosen.iter().zip(&lift).map(|(&j, l)| (radial(j), fine[j] + l)).collect();
    let trim_max = trims.iter().map(|t| t.1).fold(f64::NEG_INFINITY, f64::max);
    let trim_min = trims.iter().map(|t| t.1).fold(f64::INFINITY, f64::min);
    if trim_max + STEP_MM > straight {
        return Err(Refused::Long);
    }

    let r_in = ri - OVER_MM.min(ri * 0.5);
    let r_out = ro + OVER_MM;
    let at = |i: usize, rho: f64| {
        let (rad, t) = trims[i % trims.len()];
        end + rad * rho + b * t
    };
    let lines = (0..=trims.len()).map(|i| Seg { from: at(i, r_in), to: at(i, r_out) }).collect();
    Ok(Cope { lines, pick: at(1, 0.5 * (ro + ri)), trim_min: trim_min.max(0.0), trim_max, clearance: extra })
}

/// How far along the axis from `end` the other tubes can still reach the wall: the end of the
/// stretch, starting at the end, where the axis is within a radius `ro` of their outsides. A
/// second meeting further along (the tube crossing one of them) is not part of this end.
fn reach_along(end: V3, b: V3, ro: f64, limit: f64, others: &[Other]) -> Option<f64> {
    let mut reach = None;
    let steps = (limit / STEP_MM).floor() as usize;
    for i in 0..=steps {
        let t = i as f64 * STEP_MM;
        if others.iter().any(|o| o.dist(end + b * t) < o.r + ro) {
            reach = Some(t);
        } else if reach.is_some() {
            break;
        }
    }
    reach
}

/// Where the fibre from `base` along `b` leaves the other tubes for the last time before `top`:
/// the distance from `base`, or `None` when it is never inside one.
fn fibre_exit(base: V3, b: V3, top: f64, others: &[Other]) -> Option<f64> {
    let inside = |p: V3| others.iter().any(|o| o.dist(p) < o.r);
    // Searched from the far side.
    let mut t = top;
    let mut lo = loop {
        if t <= 0.0 {
            return None;
        }
        t = (t - STEP_MM).max(0.0);
        if inside(base + b * t) {
            break t;
        }
    };
    let mut hi = lo + STEP_MM;
    for _ in 0..18 {
        let mid = 0.5 * (lo + hi);
        if inside(base + b * mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(hi)
}

/// What a flush cope takes off the end at its long point, mm: the least any fibre of the wall
/// is cut back by the other tubes' outsides. 0 where some fibre is not cut at all.
pub fn flush_long_point(end: V3, into: V3, ro: f64, ri: f64, straight: f64, others: &[Other]) -> f64 {
    let b = into.norm();
    let u = b.any_perp();
    let v = b.cross(u);
    let r_max = others.iter().map(|o| o.r).fold(0.0, f64::max);
    let limit = straight.min(8.0 * (r_max + ro));
    let Some(reach) = reach_along(end, b, ro, limit, others) else { return 0.0 };
    let top = (reach + STEP_MM).min(straight);
    let mut least = f64::INFINITY;
    for j in 0..FINE / 2 {
        let th = std::f64::consts::TAU * j as f64 / (FINE / 2) as f64;
        let rad = u * th.cos() + v * th.sin();
        for rho in [ri, ro] {
            match fibre_exit(end + rad * rho, b, top, others) {
                Some(t) => least = least.min(t),
                None => return 0.0,
            }
        }
    }
    least
}

/// A perpendicular cope of one end of the new tube.
#[derive(Debug, Clone, PartialEq)]
pub struct NewEnd {
    pub end: V3,
    pub into: V3,
    pub cope: Cope,
    /// The earlier tubes it is trimmed to.
    pub to: Vec<usize>,
}

/// A perpendicular cope of an earlier tube's end, to the new tube that runs through it.
#[derive(Debug, Clone, PartialEq)]
pub struct OldEnd {
    pub tube: usize,
    pub end: V3,
    pub into: V3,
    pub cope: Cope,
}

/// How each cope of a plan is cut.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Cuts {
    pub new_ends: Vec<NewEnd>,
    pub old_ends: Vec<OldEnd>,
    /// Earlier tubes the new tube is cut to by their own outside (flush).
    pub flush_new: Vec<usize>,
    /// Earlier tubes cut by the new tube's outside (flush).
    pub flush_old: Vec<usize>,
    /// What the flush copes of the new tube's ends take off at their long points, mm, in all.
    pub flush_trim: f64,
    pub warnings: Vec<String>,
}

impl Cuts {
    pub fn host_calls(&self) -> usize {
        (self.new_ends.len() + self.old_ends.len()) * CALLS_PER_COPE
    }
}

/// A chain's open ends: the point, the direction into the tube, and how long the straight piece
/// there is (`None` when the end is in a bend). A tube of one line shares it between its ends.
fn straight_ends(chain: &Chain) -> Vec<(V3, V3, Option<f64>)> {
    let share = if chain.curves.len() == 1 { 0.5 } else { 1.0 };
    let len = |c: &Curve| match c {
        Curve::Line { .. } => Some(c.length() * share),
        Curve::Arc { .. } => None,
    };
    let (first, last) = (&chain.curves[0].1, &chain.curves.last().unwrap().1);
    chain.ends().into_iter().zip([len(first), len(last)]).map(|((p, b), l)| (p, b, l)).collect()
}

/// Decides, for each cope of `plan`, whether it is cut flush or perpendicular.
pub fn cuts(name: &str, plan: &Plan, existing: &[Existing], opt: &Options) -> Cuts {
    let mut out = Cuts::default();
    let ro = opt.od_mm / 2.0;
    let ri = ro - opt.wall_mm;
    let poly = plan.chain.polyline();
    let butts: Vec<(usize, [V3; 2])> = plan.butts.iter().map(|b| (b.with, [b.run_on_old.from, b.run_on_old.to])).collect();
    let perpendicular = opt.cope == CopeStyle::Perpendicular;

    // The new tube's ends.
    let mut flush: Vec<usize> = Vec::new();
    let mut seen: Vec<usize> = Vec::new();
    for (p, b, straight) in straight_ends(&plan.chain) {
        let near: Vec<usize> =
            plan.cope_new_to.iter().copied().filter(|&k| existing[k].point_near(p) < ro + existing[k].r() - 0.05).collect();
        if near.is_empty() {
            continue;
        }
        seen.extend(&near);
        let names = near.iter().map(|&k| existing[k].name.as_str()).collect::<Vec<_>>().join(" and ");
        let Some(straight) = straight else {
            if perpendicular {
                out.warnings.push(format!("{name} ends on {names} in a bend, so that end is coped flush, not perpendicular"));
            }
            flush.extend(&near);
            continue;
        };
        let mut others: Vec<Other> = Vec::new();
        for &k in &near {
            let e = &existing[k];
            let r = e.r() + opt.gap_mm;
            // Its own path, exact when it is one chain; run-ons are straight lines.
            let mut polys: Vec<&[V3]> = e.run_ons.iter().map(|(_, _, q)| q.as_slice()).collect();
            polys.extend(butts.iter().filter(|(j, _)| *j == k).map(|(_, s)| s.as_slice()));
            match &e.chain {
                Some(c) => others.push(Other { curves: c.curves.iter().map(|(_, c)| *c).collect(), polys, r }),
                None => {
                    others.push(Other { curves: Vec::new(), polys: vec![&e.polyline], r: r + CHORD_MM });
                    others.push(Other { curves: Vec::new(), polys, r });
                }
            }
        }
        if !perpendicular {
            out.flush_trim += flush_long_point(p, b, ro, ri, straight, &others);
            continue;
        }
        match cope(p, b, ro, ri, straight, &others) {
            Ok(cope) => out.new_ends.push(NewEnd { end: p, into: b, cope, to: near }),
            Err(Refused::Clear) => {}
            Err(Refused::Long) => {
                out.warnings.push(format!(
                    "{name} meets {names} too shallow or too close to its own bend for a perpendicular cope, so that end is coped flush"
                ));
                out.flush_trim += flush_long_point(p, b, ro, ri, straight, &others);
                flush.extend(&near);
            }
        }
    }
    if !perpendicular {
        out.flush_new = plan.cope_new_to.clone();
        out.flush_old = plan.cope_old_to_new.clone();
        return out;
    }
    // Crossings (also of a tube an end lands on) and refused ends are cut flush.
    out.flush_new = plan
        .cope_new_to
        .iter()
        .copied()
        .filter(|k| flush.contains(k) || !seen.contains(k) || plan.crossings.contains(k))
        .collect();

    // Earlier tubes whose ends land on the new tube.
    let other = [Other { curves: plan.chain.curves.iter().map(|(_, c)| *c).collect(), polys: Vec::new(), r: ro + opt.gap_mm }];
    for &k in &plan.cope_old_to_new {
        let e = &existing[k];
        let why = match (&e.chain, e.wall_mm) {
            _ if e.bodies.len() != 1 => Some("is in several pieces"),
            (None, _) => Some("has a path that is not one chain"),
            (_, None) => Some("is a solid bar"),
            (_, Some(w)) if w >= e.r() - 0.05 => Some("is a solid bar"),
            _ => None,
        };
        if let Some(why) = why {
            out.warnings.push(format!("{} {why}, so its end is coped flush to {name}, not perpendicular", e.name));
            out.flush_old.push(k);
            continue;
        }
        let (chain, wall) = (e.chain.as_ref().unwrap(), e.wall_mm.unwrap());
        let mut made = Vec::new();
        let mut refused = false;
        for (q, into, straight) in straight_ends(chain) {
            if point_poly(q, &poly) >= ro + e.r() - 0.05 {
                continue;
            }
            match straight.map(|s| cope(q, into, e.r(), e.r() - wall, s, &other)) {
                Some(Ok(cope)) => made.push(OldEnd { tube: k, end: q, into, cope }),
                Some(Err(Refused::Clear)) => {}
                Some(Err(Refused::Long)) | None => refused = true,
            }
        }
        if refused {
            out.warnings.push(format!(
                "{} ends on {name} in a bend or too shallow for a perpendicular cope, so its end is coped flush",
                e.name
            ));
            out.flush_old.push(k);
        } else {
            out.old_ends.extend(made);
        }
    }
    out
}

/// "0.607 to 0.875 in": what a cope trims off the end, long point to short point.
pub fn trim_text(c: &Cope) -> String {
    format!("{:.3} to {}", c.trim_min / crate::profiles::MM_PER_IN, inches(c.trim_max))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::v3;
    use crate::plan::tests::{brace, diagonal, existing, hoop, line, opts};
    use crate::plan::{build, SharpJoints};

    const R: f64 = 22.225;
    const RO: f64 = 19.05;
    const RI: f64 = 16.002;

    fn perp(od: f64) -> Options {
        Options { cope: CopeStyle::Perpendicular, ..opts(od, SharpJoints::Miter) }
    }

    /// A square tee: the brace along -Y from the origin, the run tube along X.
    fn tee() -> Cope {
        let run = [v3(-300.0, 0.0, 0.0), v3(300.0, 0.0, 0.0)];
        cope(v3(0.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), RO, RI, 400.0, &[Other { curves: vec![], polys: vec![&run], r: R }]).unwrap()
    }

    #[test]
    fn square_tee_is_trimmed_to_the_inside_wall() {
        let c = tee();
        // Along the run tube's axis the wall reaches its full radius; across it, the inside of
        // the wall is what touches: sqrt(R² - ri²).
        // On top comes the clearance, and what the spacing of the lines would leave inside.
        assert!(c.trim_max >= R && c.trim_max - R < 0.15, "{}", c.trim_max);
        assert!(c.clearance >= CLEAR_MM && c.clearance < 0.15, "{}", c.clearance);
        let across = (R * R - RI * RI).sqrt();
        assert!(c.trim_min >= across && c.trim_min - across < 0.15, "{}", c.trim_min);
        assert_eq!(c.lines.len(), STATIONS + 1);
        assert!(c.lines[0].from.near(c.lines[STATIONS].from) && c.lines[0].to.near(c.lines[STATIONS].to));
    }

    #[test]
    fn every_line_is_radial_and_clears_the_other_tube() {
        let c = tee();
        for l in &c.lines {
            // Square to the tube's axis (Y), and pointing away from it.
            let d = l.to - l.from;
            assert!(d.y.abs() < 1e-9);
            assert!(((l.to - v3(0.0, l.to.y, 0.0)).len() - (RO + OVER_MM)).abs() < 1e-9);
            assert!(d.cross(v3(l.from.x, 0.0, l.from.z)).len() < 1e-9);
            // No point of the wall on the kept side is inside the run tube (axis X).
            for j in 0..=10 {
                let rho = RI + (RO - RI) * j as f64 / 10.0;
                let dir = v3(l.to.x, 0.0, l.to.z).norm();
                let p = dir * rho + v3(0.0, l.to.y - 1e-6, 0.0);
                assert!((p.y * p.y + p.z * p.z).sqrt() >= R - 1e-4);
            }
        }
    }

    #[test]
    fn lines_are_spread_along_the_cut_not_round_the_tube() {
        // Flat for half a turn, then a steep V: the V is as long again as the flat, so it
        // gets about as many lines, though it spans the same angle.
        let r = 17.5;
        let step = std::f64::consts::TAU * r / FINE as f64;
        let f: Vec<f64> = (0..FINE).map(|j| if j < 360 { 0.0 } else { 3.0 * step * (180.0 - (j as f64 - 540.0).abs()) }).collect();
        let s = stations(&f, r);
        assert_eq!(s.len(), STATIONS);
        // Going round once from the seam, no angle twice.
        let turned: usize = (0..STATIONS).map(|i| span(FINE, s[i], s[(i + 1) % STATIONS])).sum();
        assert_eq!(turned, FINE);
        assert!((0..STATIONS).all(|i| s[i] != s[(i + 1) % STATIONS]));
        let steep = s.iter().filter(|&&j| j >= 360).count();
        assert!(steep > STATIONS * 2 / 3, "{steep}");
        // The seam is on the flat.
        assert!(s[0] < 360);
    }

    #[test]
    fn a_peak_between_lines_lifts_only_its_neighbours() {
        let mut f = vec![0.0; FINE];
        f[7] = 1.0;
        let chosen: Vec<usize> = (0..STATIONS).map(|i| i * FINE / STATIONS).collect();
        let lift = peak_lift(&f, &chosen);
        assert_eq!(lift[0], 1.0);
        assert_eq!(lift[1], 1.0);
        assert!(lift[2..].iter().all(|l| *l == 0.0));
    }

    #[test]
    fn lines_beside_a_crease_are_lifted() {
        let chosen: Vec<usize> = (0..STATIONS).map(|i| i * FINE / STATIONS).collect();
        // A V at the tenth line, turning by 2 per line.
        let g: Vec<f64> = (0..STATIONS).map(|i| (i as f64 - 10.0).abs().min(4.0)).collect();
        let lift = ring_lift(FINE, &g, &chosen);
        assert!(lift[10] >= RING * 2.0 - 1e-9, "{}", lift[10]);
        assert!(lift[11] < lift[10] + 1e-9 && lift[11] > 0.0);
        // Far from every crease nothing is lifted.
        assert!(lift[30] < 1e-9 && lift[40] < 1e-9);
    }

    #[test]
    fn zigzag_alternates_so_odd_segments_are_the_lines() {
        let c = tee();
        let z = c.zigzag();
        assert_eq!(z.len(), 2 * (STATIONS + 1));
        for (i, l) in c.lines.iter().enumerate() {
            let (a, b) = (z[2 * i], z[2 * i + 1]);
            assert!((a.near(l.from) && b.near(l.to)) || (a.near(l.to) && b.near(l.from)));
        }
        // Consecutive lines share the side they join on.
        assert!(z[1].dist(z[2]) < 5.0 && z[3].dist(z[4]) < 5.0);
    }

    #[test]
    fn a_weld_gap_trims_further() {
        let run = [v3(-300.0, 0.0, 0.0), v3(300.0, 0.0, 0.0)];
        let c = cope(v3(0.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), RO, RI, 400.0, &[Other { curves: vec![], polys: vec![&run], r: R + 1.0 }]).unwrap();
        assert!(c.trim_max >= R + 1.0 && c.trim_max - (R + 1.0) < 0.15);
    }

    #[test]
    fn clear_and_too_long_are_refused() {
        let far = [v3(-300.0, 100.0, 0.0), v3(300.0, 100.0, 0.0)];
        let e = cope(v3(0.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), RO, RI, 400.0, &[Other { curves: vec![], polys: vec![&far], r: R }]);
        assert_eq!(e, Err(Refused::Clear));
        // Along the other tube: the wall never leaves it.
        let along = [v3(0.0, 300.0, 0.0), v3(0.0, -300.0, 0.0)];
        let e = cope(v3(0.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), RO, RI, 400.0, &[Other { curves: vec![], polys: vec![&along], r: R }]);
        assert_eq!(e, Err(Refused::Long));
        // A straight end shorter than the cope.
        let run = [v3(-300.0, 0.0, 0.0), v3(300.0, 0.0, 0.0)];
        let e = cope(v3(0.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), RO, RI, 15.0, &[Other { curves: vec![], polys: vec![&run], r: R }]);
        assert_eq!(e, Err(Refused::Long));
    }

    #[test]
    fn flush_style_keeps_every_cope_flush() {
        let old = vec![existing("Hoop", 44.45, &hoop())];
        let o = opts(38.1, SharpJoints::Miter);
        let plan = build("Brace", 1, &brace(), &old, &o).unwrap();
        let c = cuts("Brace", &plan, &old, &o);
        assert_eq!(c.flush_new, vec![0]);
        assert!(c.new_ends.is_empty() && c.host_calls() == 0);
    }

    #[test]
    fn brace_on_hoop_is_trimmed_at_both_ends() {
        let old = vec![existing("Hoop", 44.45, &hoop())];
        let plan = build("Brace", 1, &brace(), &old, &perp(38.1)).unwrap();
        let c = cuts("Brace", &plan, &old, &perp(38.1));
        assert_eq!(c.new_ends.len(), 2);
        assert!(c.flush_new.is_empty() && c.warnings.is_empty(), "{:?}", c.warnings);
        for e in &c.new_ends {
            assert_eq!(e.to, vec![0]);
            assert!((e.cope.trim_max - R).abs() < 0.15);
        }
        assert_eq!(c.host_calls(), 2 * CALLS_PER_COPE);
    }

    #[test]
    fn diagonal_at_a_node_is_trimmed_to_both_tubes_in_one_cut() {
        let old = vec![existing("Hoop", 44.45, &hoop()), existing("Brace", 38.1, &brace())];
        let plan = build("Diagonal", 1, &diagonal(), &old, &perp(38.1)).unwrap();
        let c = cuts("Diagonal", &plan, &old, &perp(38.1));
        assert_eq!(c.new_ends.len(), 2);
        assert!(c.flush_new.is_empty());
        let node = c.new_ends.iter().find(|e| e.to.len() == 2).expect("the top end meets hoop and brace");
        let single = c.new_ends.iter().find(|e| e.to.len() == 1).unwrap();
        assert!(node.cope.trim_max > single.cope.trim_max);
    }

    #[test]
    fn hoop_made_last_trims_the_brace_ends() {
        let old = vec![existing("Brace", 38.1, &brace())];
        let plan = build("Hoop", 1, &hoop(), &old, &perp(44.45)).unwrap();
        let c = cuts("Hoop", &plan, &old, &perp(44.45));
        assert_eq!(c.old_ends.len(), 2);
        assert!(c.flush_old.is_empty() && c.new_ends.is_empty());
        assert!((c.old_ends[0].cope.trim_max - R).abs() < 0.15);
    }

    #[test]
    fn crossings_and_solid_bars_stay_flush() {
        let old = vec![existing("A", 38.1, &[line("l1", [-300.0, 0.0], [300.0, 0.0])])];
        let new = [line("l2", [0.0, -300.0], [0.0, 300.0])];
        let plan = build("B", 1, &new, &old, &perp(38.1)).unwrap();
        let c = cuts("B", &plan, &old, &perp(38.1));
        assert_eq!(c.flush_new, vec![0]);
        assert!(c.new_ends.is_empty());

        let mut bar = existing("Bar", 38.1, &brace());
        bar.wall_mm = None;
        let old = vec![bar];
        let plan = build("Hoop", 1, &hoop(), &old, &perp(44.45)).unwrap();
        let c = cuts("Hoop", &plan, &old, &perp(44.45));
        assert_eq!(c.flush_old, vec![0]);
        assert!(c.warnings[0].contains("solid bar"));
    }

    #[test]
    fn an_end_on_a_tube_it_also_crosses_is_trimmed_at_the_end_and_cut_at_the_crossing() {
        // A rail bent round into a U; the brace starts on its lower leg and runs up through
        // the upper one, 200 mm away, well inside the stretch a cope searches.
        let u = [line("l1", [-300.0, 0.0], [300.0, 0.0]), crate::plan::tests::arc("a1", [300.0, 100.0], 100.0, -90.0, 180.0), line("l3", [300.0, 200.0], [-300.0, 200.0])];
        let old = vec![existing("U", 44.45, &u)];
        let new = [line("l2", [0.0, 0.0], [0.0, 900.0])];
        let plan = build("B", 1, &new, &old, &perp(38.1)).unwrap();
        let c = cuts("B", &plan, &old, &perp(38.1));
        // The end's cope is that of a plain tee: the crossing does not lengthen it.
        assert_eq!(c.new_ends.len(), 1);
        assert!((c.new_ends[0].cope.trim_max - R).abs() < 0.15, "{}", c.new_ends[0].cope.trim_max);
        // And the crossing is cut by the rail's outside.
        assert_eq!(c.flush_new, vec![0]);
    }

    #[test]
    fn a_flush_cope_is_measured_to_its_long_point() {
        let old = vec![existing("Hoop", 44.45, &hoop())];
        let o = opts(38.1, SharpJoints::Miter);
        let plan = build("Brace", 1, &brace(), &old, &o).unwrap();
        let c = cuts("Brace", &plan, &old, &o);
        let each = (R * R - RO * RO).sqrt();
        assert!((c.flush_trim - 2.0 * each).abs() < 0.01, "{}", c.flush_trim);
        // The same size all round: the cope's ears reach the other tube's centreline.
        let o = opts(44.45, SharpJoints::Miter);
        let plan = build("Brace", 1, &brace(), &old, &o).unwrap();
        assert!(cuts("Brace", &plan, &old, &o).flush_trim < 0.01);
    }
}
