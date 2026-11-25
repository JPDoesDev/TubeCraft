//! Tube sizes. Each entry is data, so new materials (DOM, 4130) and sizes are one line each.
//!
//! The `id` is what the plugin receives from the dialog; keep ids stable once released,
//! because saved scripts and agent calls name them.

pub const MM_PER_IN: f64 = 25.4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TubeSize {
    pub id: &'static str,
    pub title: &'static str,
    /// Outside diameter, inches.
    pub od_in: f64,
    /// Wall thickness, inches.
    pub wall_in: f64,
}

impl TubeSize {
    pub fn od_mm(&self) -> f64 {
        self.od_in * MM_PER_IN
    }
    pub fn wall_mm(&self) -> f64 {
        self.wall_in * MM_PER_IN
    }
    pub fn radius_mm(&self) -> f64 {
        self.od_mm() / 2.0
    }
}

macro_rules! hrew {
    ($id:literal, $title:literal, $od:expr, $wall:expr) => {
        TubeSize { id: $id, title: $title, od_in: $od, wall_in: $wall }
    };
}

/// HREW (hot-rolled electric-welded) round tube.
pub const SIZES: &[TubeSize] = &[
    hrew!("hrew-1.250x0.065", "1.250 x 0.065 HREW", 1.25, 0.065),
    hrew!("hrew-1.250x0.095", "1.250 x 0.095 HREW", 1.25, 0.095),
    hrew!("hrew-1.250x0.120", "1.250 x 0.120 HREW", 1.25, 0.120),
    hrew!("hrew-1.500x0.065", "1.500 x 0.065 HREW", 1.5, 0.065),
    hrew!("hrew-1.500x0.095", "1.500 x 0.095 HREW", 1.5, 0.095),
    hrew!("hrew-1.500x0.120", "1.500 x 0.120 HREW", 1.5, 0.120),
    hrew!("hrew-1.750x0.065", "1.750 x 0.065 HREW", 1.75, 0.065),
    hrew!("hrew-1.750x0.095", "1.750 x 0.095 HREW", 1.75, 0.095),
    hrew!("hrew-1.750x0.120", "1.750 x 0.120 HREW", 1.75, 0.120),
    hrew!("hrew-2.000x0.065", "2.000 x 0.065 HREW", 2.0, 0.065),
    hrew!("hrew-2.000x0.095", "2.000 x 0.095 HREW", 2.0, 0.095),
    hrew!("hrew-2.000x0.120", "2.000 x 0.120 HREW", 2.0, 0.120),
];

pub fn find(id: &str) -> Option<&'static TubeSize> {
    SIZES.iter().find(|s| s.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_and_walls_fit() {
        for (i, a) in SIZES.iter().enumerate() {
            assert!(a.wall_in * 2.0 < a.od_in, "{}", a.id);
            for b in &SIZES[i + 1..] {
                assert_ne!(a.id, b.id);
            }
        }
        assert_eq!(SIZES.len(), 12);
        assert!((find("hrew-1.750x0.120").unwrap().od_mm() - 44.45).abs() < 1e-9);
    }
}
