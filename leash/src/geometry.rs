//! Rectangles against points and segments.

use contracts::{Point, Rect};

pub fn inflate(r: Rect, c: f32) -> Rect {
    Rect {
        x0: r.x0 - c,
        x1: r.x1 + c,
        y0: r.y0 - c,
        y1: r.y1 + c,
    }
}

pub fn contains(r: Rect, p: Point) -> bool {
    p.x >= r.x0 && p.x <= r.x1 && p.y >= r.y0 && p.y <= r.y1
}

/// Box distance from `p`: positive outside, negative inside. `p` lies inside
/// `inflate(r, c)` exactly when this is `<= c`.
pub fn box_distance(r: Rect, p: Point) -> f32 {
    (r.x0 - p.x).max(p.x - r.x1).max(r.y0 - p.y).max(p.y - r.y1)
}

/// Liang-Barsky: does the closed segment `a`-`b` touch `r`?
pub fn hits_segment(r: Rect, a: Point, b: Point) -> bool {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for (p, q) in [
        (-dx, a.x - r.x0),
        (dx, r.x1 - a.x),
        (-dy, a.y - r.y0),
        (dy, r.y1 - a.y),
    ] {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}
