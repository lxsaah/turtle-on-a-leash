//! Turn toward the target, drive once roughly facing it.

use contracts::Point;
use micromath::F32Ext;

const PI: f32 = core::f32::consts::PI;
const MAX_ANGULAR: f32 = 2.0;
const ANGULAR_GAIN: f32 = 4.0;
const LINEAR_GAIN: f32 = 1.5;

/// `(linear, angular)` from pose `(x, y, theta)` toward `target`.
///
/// Math goes through `F32Ext` explicitly, so host tests run the same
/// approximations as the board rather than `std`'s.
pub fn step(target: Point, (x, y, theta): (f32, f32, f32), max_linear: f32) -> (f32, f32) {
    let (dx, dy) = (target.x - x, target.y - y);
    let distance = F32Ext::sqrt(dx * dx + dy * dy);
    let mut error = F32Ext::atan2(dy, dx) - theta;
    while error > PI {
        error -= 2.0 * PI;
    }
    while error <= -PI {
        error += 2.0 * PI;
    }
    let angular = (ANGULAR_GAIN * error).clamp(-MAX_ANGULAR, MAX_ANGULAR);
    let linear = if error.abs() < PI / 4.0 {
        (LINEAR_GAIN * distance).min(max_linear) * F32Ext::cos(error)
    } else {
        0.0
    };
    (linear, angular)
}
