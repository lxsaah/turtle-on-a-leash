//! The demo's fixture: the map and the two motion policies.
//!
//! The numbers are measured on the real rig. The start, at `(1.25, 5.5)`, gives
//! about 2 s from the first motion at full speed to the Restricted boundary.

use alloc::vec;

use contracts::{Mode, Point, Policy, Rect, WorldMap};

pub const START: Point = Point { x: 1.25, y: 5.5 };
pub const DESTINATION: Point = Point { x: 9.0, y: 5.5 };

/// The line a pose trace is judged against under Restricted.
pub const RESTRICTED_BOUNDARY: f32 = 0.25;
/// The boundary plus measured corner-cutting (0.023) and some margin.
const RESTRICTED_ADMISSION: f32 = 0.30;
const RESTRICTED_LIVE: f32 = 0.15;
const OPEN_SPEED: f32 = 2.0;
const RESTRICTED_SPEED: f32 = 0.8;
const SOLID_MARGIN: f32 = 0.1;
const MAX_WAYPOINTS: u8 = 16;
const ARRIVAL_TOLERANCE: f32 = 0.1;

pub fn map() -> WorldMap {
    WorldMap {
        bounds: Rect {
            x0: 1.0,
            x1: 10.0,
            y0: 1.0,
            y1: 10.0,
        },
        solid_obstacles: vec![Rect {
            x0: 5.5,
            x1: 6.5,
            y0: 2.0,
            y1: 3.5,
        }],
        work_area: Rect {
            x0: 5.5,
            x1: 7.5,
            y0: 3.5,
            y1: 7.0,
        },
    }
}

pub fn open(revision: u32) -> Policy {
    Policy {
        mode: Mode::Open,
        revision,
        speed_limit: OPEN_SPEED,
        work_area_excluded: false,
        admission_clearance: 0.0,
        live_clearance: 0.0,
        solid_margin: SOLID_MARGIN,
        max_waypoints: MAX_WAYPOINTS,
        arrival_tolerance: ARRIVAL_TOLERANCE,
    }
}

pub fn restricted(revision: u32) -> Policy {
    Policy {
        mode: Mode::Restricted,
        revision,
        speed_limit: RESTRICTED_SPEED,
        work_area_excluded: true,
        admission_clearance: RESTRICTED_ADMISSION,
        live_clearance: RESTRICTED_LIVE,
        solid_margin: SOLID_MARGIN,
        max_waypoints: MAX_WAYPOINTS,
        arrival_tolerance: ARRIVAL_TOLERANCE,
    }
}

/// The policy after one button press.
pub fn toggled(policy: &Policy) -> Policy {
    match policy.mode {
        Mode::Open => restricted(policy.revision + 1),
        Mode::Restricted => open(policy.revision + 1),
    }
}
