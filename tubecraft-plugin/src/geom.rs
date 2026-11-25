//! World-space geometry of tube centrelines: lines and circular arcs, as a bender makes them.
//! All lengths are millimetres, all angles radians.

use std::f64::consts::PI;
use std::ops::{Add, Mul, Neg, Sub};

/// Points closer than this are the same point.
pub const POINT_TOL: f64 = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: V3) -> V3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn norm(self) -> V3 {
        let l = self.len();
        if l == 0.0 {
            self
        } else {
            self * (1.0 / l)
        }
    }
    pub fn dist(self, o: V3) -> f64 {
        (self - o).len()
    }
    pub fn near(self, o: V3) -> bool {
        self.dist(o) <= POINT_TOL
    }
    /// Any unit vector square to `self` (which must not be zero).
    pub fn any_perp(self) -> V3 {
        let n = self.norm();
        let pick = if n.x.abs() < 0.9 { v3(1.0, 0.0, 0.0) } else { v3(0.0, 1.0, 0.0) };
        n.cross(pick).norm()
    }
    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, k: f64) -> V3 {
        v3(self.x * k, self.y * k, self.z * k)
    }
}
impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

/// A tube centreline piece.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Curve {
    Line { a: V3, b: V3 },
    /// Counter-clockwise about the unit normal `n`, from `a` through `sweep` radians.
    Arc { c: V3, r: f64, n: V3, a: V3, sweep: f64 },
}

impl Curve {
    pub fn start(&self) -> V3 {
        match *self {
            Curve::Line { a, .. } => a,
            Curve::Arc { a, .. } => a,
        }
    }
    pub fn end(&self) -> V3 {
        self.point(1.0)
    }
    /// The point at `s` in 0..=1 along the curve.
    pub fn point(&self, s: f64) -> V3 {
        match *self {
            Curve::Line { a, b } => a + (b - a) * s,
            Curve::Arc { c, r, n, a, sweep } => {
                let u = (a - c).norm();
                let v = n.cross(u);
                let t = s * sweep;
                c + (u * t.cos() + v * t.sin()) * r
            }
        }
    }
    /// The unit direction of travel at `s`.
    pub fn tangent(&self, s: f64) -> V3 {
        match *self {
            Curve::Line { a, b } => (b - a).norm(),
            Curve::Arc { c, n, a, sweep, .. } => {
                let u = (a - c).norm();
                let v = n.cross(u);
                let t = s * sweep;
                (u * -t.sin() + v * t.cos()).norm()
            }
        }
    }
    pub fn length(&self) -> f64 {
        match *self {
            Curve::Line { a, b } => a.dist(b),
            Curve::Arc { r, sweep, .. } => r * sweep,
        }
    }
    /// The closest distance from `p` to the curve, exact for arcs too.
    pub fn dist(&self, p: V3) -> f64 {
        match *self {
            Curve::Line { a, b } => point_seg(p, a, b),
            Curve::Arc { c, r, n, a, sweep } => {
                let u = (a - c).norm();
                let v = n.cross(u);
                let q = p - c;
                let (x, y, z) = (q.dot(u), q.dot(v), q.dot(n));
                let mut ang = y.atan2(x);
                if ang < 0.0 {
                    ang += 2.0 * PI;
                }
                if ang <= sweep {
                    ((x.hypot(y) - r).powi(2) + z * z).sqrt()
                } else {
                    p.dist(a).min(p.dist(self.end()))
                }
            }
        }
    }
    pub fn reversed(&self) -> Curve {
        match *self {
            Curve::Line { a, b } => Curve::Line { a: b, b: a },
            Curve::Arc { c, r, n, sweep, .. } => Curve::Arc { c, r, n: -n, a: self.end(), sweep },
        }
    }
    /// The curve as a polyline, fine enough for clearance checks on tube-sized gaps.
    pub fn polyline(&self) -> Vec<V3> {
        match *self {
            Curve::Line { a, b } => vec![a, b],
            Curve::Arc { r, sweep, .. } => {
                // Chord error r(1-cos(d/2)) below 0.05 mm, at least 8 segments per quarter turn.
                let max_step = (2.0 * (1.0 - 0.05 / r.max(0.05)).acos()).max(1e-3);
                let k = ((sweep / max_step).ceil() as usize).max((sweep / (PI / 16.0)).ceil() as usize).max(2);
                (0..=k).map(|i| self.point(i as f64 / k as f64)).collect()
            }
        }
    }
}

/// The arc's counter-clockwise sweep from angle `t0` to `t1`, in (0, 2π].
pub fn ccw_sweep(t0: f64, t1: f64) -> f64 {
    let mut d = (t1 - t0) % (2.0 * PI);
    if d <= 1e-12 {
        d += 2.0 * PI;
    }
    d
}

/// The closest distance between segments p0-p1 and q0-q1.
pub fn seg_seg(p0: V3, p1: V3, q0: V3, q1: V3) -> f64 {
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (s, t);
    if a <= 1e-18 && e <= 1e-18 {
        return p0.dist(q0);
    }
    if a <= 1e-18 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= 1e-18 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > 1e-18 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    (p0 + d1 * s).dist(q0 + d2 * t)
}

/// The closest distance from point `p` to segment a-b.
pub fn point_seg(p: V3, a: V3, b: V3) -> f64 {
    seg_seg(p, p, a, b)
}

/// Closest distance between two polylines.
pub fn poly_poly(p: &[V3], q: &[V3]) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..p.len().saturating_sub(1) {
        for j in 0..q.len().saturating_sub(1) {
            best = best.min(seg_seg(p[i], p[i + 1], q[j], q[j + 1]));
        }
    }
    best
}

/// Closest distance from a point to a polyline.
pub fn point_poly(p: V3, q: &[V3]) -> f64 {
    let mut best = f64::INFINITY;
    for j in 0..q.len().saturating_sub(1) {
        best = best.min(point_seg(p, q[j], q[j + 1]));
    }
    best
}

/// The angle between two directions, radians.
pub fn angle_between(a: V3, b: V3) -> f64 {
    a.norm().dot(b.norm()).clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    fn arc_ends_tangents_and_reverse() {
        // Quarter circle in XY from +X to +Y about the origin.
        let arc = Curve::Arc { c: v3(0., 0., 0.), r: 10.0, n: v3(0., 0., 1.), a: v3(10., 0., 0.), sweep: PI / 2.0 };
        assert!(arc.end().near(v3(0., 10., 0.)));
        assert!(arc.tangent(0.0).near(v3(0., 1., 0.)));
        assert!(arc.tangent(1.0).near(v3(-1., 0., 0.)));
        close(arc.length(), 5.0 * PI);
        let rev = arc.reversed();
        assert!(rev.start().near(arc.end()));
        assert!(rev.end().near(arc.start()));
        assert!(rev.tangent(0.0).near(v3(1., 0., 0.)));
    }

    #[test]
    fn arc_distance_is_exact() {
        let arc = Curve::Arc { c: v3(0., 0., 0.), r: 10.0, n: v3(0., 0., 1.), a: v3(10., 0., 0.), sweep: PI / 2.0 };
        // Beside the arc, above it, and past each end.
        close(arc.dist(v3(12.0 * (0.3f64).cos(), 12.0 * (0.3f64).sin(), 0.0)), 2.0);
        close(arc.dist(v3(10.0 * (1.0f64).cos(), 10.0 * (1.0f64).sin(), 3.0)), 3.0);
        close(arc.dist(v3(10.0, -4.0, 0.0)), 4.0);
        close(arc.dist(v3(-3.0, 10.0, 0.0)), 3.0);
    }

    #[test]
    fn sweep_wraps() {
        close(ccw_sweep(-PI / 2.0, 0.0), PI / 2.0);
        close(ccw_sweep(0.0, -PI / 2.0), 1.5 * PI);
        close(ccw_sweep(1.0, 1.0), 2.0 * PI);
    }

    #[test]
    fn segment_distances() {
        close(seg_seg(v3(0., 0., 0.), v3(10., 0., 0.), v3(5., 3., 4.), v3(5., 3., -4.)), 3.0);
        close(seg_seg(v3(0., 0., 0.), v3(1., 0., 0.), v3(3., 0., 0.), v3(4., 0., 0.)), 2.0);
        close(point_seg(v3(0., 5., 0.), v3(-1., 0., 0.), v3(1., 0., 0.)), 5.0);
    }
}
