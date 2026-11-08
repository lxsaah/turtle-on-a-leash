//! The records the board, the gateway and the agent agree on.
//!
//! Shapes only: what the rules do with these values lives in `leash`.
#![no_std]
extern crate alloc;

use alloc::vec::Vec;

use aimdb_data_contracts::SchemaType;
use serde::de::{self, Deserializer, SeqAccess, Visitor};
use serde::{Deserialize, Serialize};

pub mod keys;

/// Longest route accepted at the write interface. Between the policy's
/// `max_waypoints` and this, the board answers `bad_length`; above it the
/// write itself fails, so a proposal can neither outgrow a serial frame and
/// vanish without a decision nor exhaust the board's heap while parsing.
pub const MAX_WIRE_WAYPOINTS: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point {
    #[serde(deserialize_with = "finite")]
    pub x: f32,
    #[serde(deserialize_with = "finite")]
    pub y: f32,
}

/// JSON has no infinity: `1e40` would parse to `inf`, leave the gateway as
/// `null`, and be dropped by the board with no decision. Refusing it here
/// makes the write itself fail.
fn finite<'de, D: Deserializer<'de>>(d: D) -> Result<f32, D::Error> {
    let value = f32::deserialize(d)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(de::Error::custom("coordinate is not finite"))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Rect {
    pub x0: f32,
    pub x1: f32,
    pub y0: f32,
    pub y1: f32,
}

/// `delivery.mission`: written by the trusted setup path only.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Mission {
    pub mission_id: u32,
    pub destination: Point,
}

/// `world.map`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldMap {
    pub bounds: Rect,
    pub solid_obstacles: Vec<Rect>,
    pub work_area: Rect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Open,
    Restricted,
}

/// `motion.policy`: every rule a route is judged by, so an agent can plan
/// from this record alone.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Policy {
    pub mode: Mode,
    pub revision: u32,
    pub speed_limit: f32,
    pub work_area_excluded: bool,
    /// Distance a route must keep from the work area to be admitted.
    pub admission_clearance: f32,
    /// Distance enforced while driving; smaller than the admission clearance
    /// so normal tracking error does not cancel a route planned along an edge.
    pub live_clearance: f32,
    /// Distance every segment must keep from a solid obstacle.
    pub solid_margin: f32,
    pub max_waypoints: u8,
    /// How close the last waypoint must be to the destination.
    pub arrival_tolerance: f32,
}

/// `delivery.route_proposal`: the agent's only writable record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteProposal {
    pub mission_id: u32,
    pub seq: u32,
    pub observed_policy_revision: u32,
    #[serde(deserialize_with = "bounded_waypoints")]
    pub waypoints: Vec<Point>,
}

fn bounded_waypoints<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<Point>, D::Error> {
    struct Bounded;

    impl<'de> Visitor<'de> for Bounded {
        type Value = Vec<Point>;

        fn expecting(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
            f.write_str("a bounded list of waypoints")
        }

        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
            let mut out = Vec::new();
            while let Some(p) = seq.next_element()? {
                if out.len() == MAX_WIRE_WAYPOINTS {
                    return Err(de::Error::custom("too many waypoints"));
                }
                out.push(p);
            }
            Ok(out)
        }
    }

    d.deserialize_seq(Bounded)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    NotArmed,
    WrongMission,
    StaleSequence,
    RouteBusy,
    PolicyChanged,
    BadLength,
    InvalidCoordinate,
    OutOfBounds,
    WrongDestination,
    StalePose,
    CrossesSolid,
    CrossesWorkArea,
    RouteInvalidated,
    PoseLost,
}

/// `delivery.route_decision`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RouteDecision {
    /// Counts every decision. A repeated proposal has the same mission and
    /// sequence as the original, so only this tells the two answers apart.
    pub decision_no: u32,
    pub mission_id: u32,
    pub seq: u32,
    pub applied_policy_revision: u32,
    pub accepted: bool,
    pub reason: Option<Reason>,
    /// The failing segment: 0 runs from the live pose to the first waypoint.
    pub segment: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Idle,
    Armed,
    Executing,
    AwaitingRoute,
    Reached,
}

/// `delivery.status`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Status {
    pub mission_id: u32,
    pub route_seq: u32,
    pub applied_policy_revision: u32,
    pub state: State,
    pub reason: Option<Reason>,
    pub pose: Option<Point>,
    /// The waypoint being driven to; `None` whenever no route is active.
    pub next_waypoint: Option<Point>,
    pub elapsed_ms: u32,
}

/// `turtle.pose`, as turtlesim publishes it on Jazzy.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "ros2", derive(aimdb_data_contracts::RosMessage))]
#[cfg_attr(
    feature = "ros2",
    ros(
        type = "turtlesim/msg/Pose",
        hash = "RIHS01_739beba26bcba6920404ba722b7b8321348512f92ea5be235c47251940dd8aa9"
    )
)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "ros2", derive(aimdb_data_contracts::RosMessage))]
#[cfg_attr(
    feature = "ros2",
    ros(
        type = "geometry_msgs/msg/Vector3",
        hash = "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d"
    )
)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// `turtle.cmd_vel`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "ros2", derive(aimdb_data_contracts::RosMessage))]
#[cfg_attr(
    feature = "ros2",
    ros(
        type = "geometry_msgs/msg/Twist",
        hash = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
    )
)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

macro_rules! schema {
    ($($t:ty => $name:literal @ $version:literal),+ $(,)?) => {$(
        impl SchemaType for $t {
            const NAME: &'static str = $name;
            const VERSION: u32 = $version;
        }

        #[cfg(feature = "linkable")]
        impl aimdb_data_contracts::Linkable for $t {
            fn from_bytes(data: &[u8]) -> Result<Self, alloc::string::String> {
                serde_json::from_slice(data).map_err(|e| alloc::string::ToString::to_string(&e))
            }

            fn to_bytes(&self) -> Result<Vec<u8>, alloc::string::String> {
                serde_json::to_vec(self).map_err(|e| alloc::string::ToString::to_string(&e))
            }
        }
    )+};
}

schema! {
    Mission => "leash.mission" @ 1,
    WorldMap => "leash.world_map" @ 1,
    Policy => "leash.motion_policy" @ 1,
    RouteProposal => "leash.route_proposal" @ 1,
    RouteDecision => "leash.route_decision" @ 1,
    Status => "leash.delivery_status" @ 1,
}

// With `ros2` the derive supplies these, named by their ROS type, so a type's
// identity does not depend on the feature.
#[cfg(not(feature = "ros2"))]
impl SchemaType for Pose {
    const NAME: &'static str = "turtlesim/msg/Pose";
}

#[cfg(not(feature = "ros2"))]
impl SchemaType for Twist {
    const NAME: &'static str = "geometry_msgs/msg/Twist";
}

// The same `NAME@VERSION` always has the same wire shape: `cargo test` checks
// these against `aimdb.lock`, and a changed shape needs a `VERSION` bump.
aimdb_data_contracts::catalog! {
    contracts: [Mission, WorldMap, Policy, RouteProposal, RouteDecision, Status, Pose, Twist],
    helpers: [Mode, Reason, State],
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use alloc::vec;
    use serde_json::json;

    fn proposal(n: usize) -> serde_json::Value {
        let wps: Vec<_> = (0..n).map(|_| json!({"x": 9.0, "y": 5.5})).collect();
        json!({"mission_id": 1, "seq": 1, "observed_policy_revision": 1, "waypoints": wps})
    }

    #[test]
    fn non_finite_coordinate_is_refused_on_write() {
        assert!(serde_json::from_str::<Point>(r#"{"x":1e40,"y":5.5}"#).is_err());
        assert!(serde_json::from_str::<Point>(r#"{"x":9.0,"y":5.5}"#).is_ok());
    }

    #[test]
    fn wire_length_cap() {
        assert!(serde_json::from_value::<RouteProposal>(proposal(MAX_WIRE_WAYPOINTS)).is_ok());
        assert!(serde_json::from_value::<RouteProposal>(proposal(MAX_WIRE_WAYPOINTS + 1)).is_err());
    }

    #[test]
    fn longest_allowed_proposal_fits_a_serial_frame() {
        let worst = Point {
            x: -1.123_456_7e-10,
            y: -1.123_456_7e-10,
        };
        let longest = RouteProposal {
            mission_id: u32::MAX,
            seq: u32::MAX,
            observed_policy_revision: u32::MAX,
            waypoints: vec![worst; MAX_WIRE_WAYPOINTS],
        };
        // The serial link's frame limit is 8 KiB, envelope included.
        assert!(serde_json::to_vec(&longest).unwrap().len() < 7000);
    }

    #[test]
    fn enums_use_their_wire_names() {
        assert_eq!(
            serde_json::to_string(&Mode::Restricted).unwrap(),
            "\"restricted\""
        );
        assert_eq!(
            serde_json::to_string(&State::AwaitingRoute).unwrap(),
            "\"awaiting_route\""
        );
        let r: Reason = serde_json::from_str("\"route_busy\"").unwrap();
        assert_eq!(r, Reason::RouteBusy);
    }
}
