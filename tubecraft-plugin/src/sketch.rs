//! Reads tube centrelines out of a `sketch.inspect` reply: planar lines and arcs (in the
//! sketch's own x, y, placed by `world_plane`) and 3D sketch wires (already in world space).

use crate::geom::{ccw_sweep, v3, Curve, V3};
use serde_json::Value;
use std::collections::BTreeMap;

/// Why a picked curve cannot be a tube centreline.
#[derive(Debug, Clone, PartialEq)]
pub enum CurveError {
    Missing,
    Unsupported(String),
}

fn arr3(v: &Value) -> Option<V3> {
    let a = v.as_array()?;
    if a.len() == 3 {
        Some(v3(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?))
    } else {
        None
    }
}

fn arr2(v: &Value) -> Option<(f64, f64)> {
    let a = v.as_array()?;
    if a.len() == 2 {
        Some((a[0].as_f64()?, a[1].as_f64()?))
    } else {
        None
    }
}

struct Plane {
    o: V3,
    x: V3,
    y: V3,
}

impl Plane {
    fn world(&self, (u, w): (f64, f64)) -> V3 {
        self.o + self.x * u + self.y * w
    }
    fn normal(&self) -> V3 {
        self.x.cross(self.y).norm()
    }
}

/// Every curve and wire of one sketch, by id, or why it cannot be used.
pub fn curves_of(inspect: &Value) -> BTreeMap<String, Result<Curve, CurveError>> {
    let mut out = BTreeMap::new();
    let wp = inspect.get("world_plane").or_else(|| inspect.get("plane"));
    let plane = wp.and_then(|p| {
        Some(Plane { o: arr3(p.get("origin")?)?, x: arr3(p.get("x_dir")?)?.norm(), y: arr3(p.get("y_dir")?)?.norm() })
    });

    if let (Some(plane), Some(curves)) = (plane, inspect.get("curves").and_then(Value::as_array)) {
        for c in curves {
            let Some(id) = c.get("id").and_then(Value::as_str) else { continue };
            out.insert(id.to_string(), planar_curve(&plane, c));
        }
    }
    if let Some(wires) = inspect.get("wires").and_then(Value::as_array) {
        for w in wires {
            let Some(id) = w.get("id").and_then(Value::as_str) else { continue };
            out.insert(id.to_string(), wire_curve(w));
        }
    }
    out
}

fn planar_curve(plane: &Plane, c: &Value) -> Result<Curve, CurveError> {
    let kind = c.get("type").and_then(Value::as_str).unwrap_or("");
    let bad = || CurveError::Unsupported(format!("a sketch {kind} could not be read"));
    match kind {
        "line" => {
            let a = plane.world(arr2(c.get("start_at").ok_or_else(bad)?).ok_or_else(bad)?);
            let b = plane.world(arr2(c.get("end_at").ok_or_else(bad)?).ok_or_else(bad)?);
            if a.near(b) {
                return Err(CurveError::Unsupported("a line of no length".into()));
            }
            Ok(Curve::Line { a, b })
        }
        "arc" => {
            let cen = arr2(c.get("center_at").ok_or_else(bad)?).ok_or_else(bad)?;
            let s = arr2(c.get("start_at").ok_or_else(bad)?).ok_or_else(bad)?;
            let e = arr2(c.get("end_at").ok_or_else(bad)?).ok_or_else(bad)?;
            let r = c.get("radius").and_then(Value::as_f64).unwrap_or(((s.0 - cen.0).powi(2) + (s.1 - cen.1).powi(2)).sqrt());
            // Sketch arcs run counter-clockwise from start to end about the sketch normal.
            let t0 = (s.1 - cen.1).atan2(s.0 - cen.0);
            let t1 = (e.1 - cen.1).atan2(e.0 - cen.0);
            Ok(Curve::Arc { c: plane.world(cen), r, n: plane.normal(), a: plane.world(s), sweep: ccw_sweep(t0, t1) })
        }
        other => Err(CurveError::Unsupported(format!(
            "a sketch {other}: tube paths are lines and arcs, as a bender makes them"
        ))),
    }
}

fn wire_curve(w: &Value) -> Result<Curve, CurveError> {
    let bad = || CurveError::Unsupported("a 3D sketch wire could not be read".into());
    if let Some(arc) = w.get("arc").filter(|a| !a.is_null()) {
        let c = arr3(arc.get("center").ok_or_else(bad)?).ok_or_else(bad)?;
        let s = arr3(arc.get("start").ok_or_else(bad)?).ok_or_else(bad)?;
        let e = arr3(arc.get("end").ok_or_else(bad)?).ok_or_else(bad)?;
        let r = arc.get("radius").and_then(Value::as_f64).unwrap_or(s.dist(c));
        let sweep_deg = arc.get("sweep_deg").and_then(Value::as_f64).ok_or_else(bad)?;
        let cross = (s - c).cross(e - c);
        if (sweep_deg - 180.0).abs() < 1e-6 || cross.len() < 1e-9 || sweep_deg >= 360.0 - 1e-6 {
            return Err(CurveError::Unsupported(
                "a 3D arc of a half or whole turn: split it into two arcs".into(),
            ));
        }
        let n = if sweep_deg < 180.0 { cross.norm() } else { -cross.norm() };
        return Ok(Curve::Arc { c, r, n, a: s, sweep: sweep_deg.to_radians() });
    }
    let points = w.get("points").and_then(Value::as_u64).unwrap_or(2);
    if points > 2 {
        return Err(CurveError::Unsupported(
            "a 3D spline: tube paths are lines and arcs, as a bender makes them".into(),
        ));
    }
    let a = arr3(w.get("start").ok_or_else(bad)?).ok_or_else(bad)?;
    let b = arr3(w.get("end").ok_or_else(bad)?).ok_or_else(bad)?;
    Ok(Curve::Line { a, b })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::f64::consts::PI;

    #[test]
    fn planar_arc_on_yz_plane() {
        let inspect = json!({
            "world_plane": {"origin": [0.0, 0.0, 0.0], "x_dir": [0.0, 1.0, 0.0], "y_dir": [0.0, 0.0, 1.0]},
            "curves": [
                {"id": "a1", "type": "arc", "center_at": [469.9, -317.5], "radius": 139.7,
                 "start_at": [469.9, -457.2], "end_at": [609.6, -317.5]},
                {"id": "l1", "type": "line", "start_at": [0.0, 0.0], "end_at": [10.0, 0.0]},
                {"id": "c1", "type": "circle", "center_at": [0.0, 0.0], "radius": 3.0}
            ]
        });
        let m = curves_of(&inspect);
        let a1 = m["a1"].clone().unwrap();
        assert!((a1.length() - 139.7 * PI / 2.0).abs() < 1e-9);
        assert!(a1.start().near(v3(0.0, 469.9, -457.2)));
        assert!(a1.end().near(v3(0.0, 609.6, -317.5)));
        assert!(m["l1"].clone().unwrap().end().near(v3(0.0, 10.0, 0.0)));
        assert!(matches!(m["c1"], Err(CurveError::Unsupported(_))));
    }

    #[test]
    fn wire_arc_from_solidcraft() {
        // As `sketch.inspect` reports an arc3d.
        let w = json!({"arc": {"center": [139.98275862068965, 0.0, 400.01724137931035], "end": [140.0, 0.0, 540.0],
            "radius": 139.98275968248169, "start": [0.0, 0.0, 400.0], "sweep_deg": 90.01411399906968},
            "end": [140.0, 0.0, 540.0], "id": "w2", "points": 209, "start": [0.0, 0.0, 400.0]});
        let c = wire_curve(&w).unwrap();
        assert!(c.end().dist(v3(140.0, 0.0, 540.0)) < 1e-6);
        // It leaves the line w1 (along +Z) going up.
        assert!(c.tangent(0.0).dot(v3(0.0, 0.0, 1.0)) > 0.999);
    }
}
