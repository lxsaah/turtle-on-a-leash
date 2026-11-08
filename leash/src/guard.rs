//! The stateless part of route admission. Mission, sequence, busy and
//! freshness checks need executor state and live in the state machine.

use contracts::{Point, Policy, Reason, WorldMap};
use micromath::F32Ext;

use crate::geometry::{box_distance, contains, hits_segment, inflate};

/// Keeps the live pose strictly outside the shrunk keep-out box.
const POSE_EPSILON: f32 = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rejection {
    pub reason: Reason,
    /// The failing segment: 0 runs from the live pose to the first waypoint.
    pub segment: Option<u8>,
}

impl From<Reason> for Rejection {
    fn from(reason: Reason) -> Self {
        Rejection {
            reason,
            segment: None,
        }
    }
}

/// Checks `from` -> `wps[0]` -> ... against the map and policy, keeping
/// `work_clearance` from the work area when the policy excludes it.
///
/// The first segment starts at the live pose, which can legitimately be
/// inside a keep-out band after a stop. It is checked at the smaller of the
/// clearance and the pose's own distance, so a segment leaving the band
/// passes and one going deeper fails. Without this a turtle stopped inside
/// the band could never be given a route again.
pub fn check_path(
    from: Point,
    wps: &[Point],
    map: &WorldMap,
    policy: &Policy,
    work_clearance: f32,
) -> Result<(), Rejection> {
    let mut a = from;
    for (i, &b) in wps.iter().enumerate() {
        let reject = |reason| Rejection {
            reason,
            segment: Some(i as u8),
        };
        let keep_out = |distance: f32, clearance: f32| {
            if i == 0 {
                clearance.min(distance - POSE_EPSILON)
            } else {
                clearance
            }
        };
        for &solid in &map.solid_obstacles {
            let c = keep_out(box_distance(solid, a), policy.solid_margin);
            if hits_segment(inflate(solid, c), a, b) {
                return Err(reject(Reason::CrossesSolid));
            }
        }
        if policy.work_area_excluded {
            let c = keep_out(box_distance(map.work_area, a), work_clearance);
            if hits_segment(inflate(map.work_area, c), a, b) {
                return Err(reject(Reason::CrossesWorkArea));
            }
        }
        a = b;
    }
    Ok(())
}

/// Admission checks that depend only on the proposal, the pose and the rules.
pub fn check_route(
    pose: Point,
    wps: &[Point],
    destination: Point,
    map: &WorldMap,
    policy: &Policy,
) -> Result<(), Rejection> {
    if wps.is_empty() || wps.len() > usize::from(policy.max_waypoints) {
        return Err(Reason::BadLength.into());
    }
    for p in wps {
        // Unreachable through the wire contract, but this is the last check
        // before a value drives the turtle.
        if !(p.x.is_finite() && p.y.is_finite()) {
            return Err(Reason::InvalidCoordinate.into());
        }
        if !contains(map.bounds, *p) {
            return Err(Reason::OutOfBounds.into());
        }
    }
    let last = wps[wps.len() - 1];
    if F32Ext::hypot(last.x - destination.x, last.y - destination.y) > policy.arrival_tolerance {
        return Err(Reason::WrongDestination.into());
    }
    check_path(pose, wps, map, policy, policy.admission_clearance)
}
