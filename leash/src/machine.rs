//! One mission on one board. Handling one event at a time is what serializes
//! admission, policy updates and command generation.

use alloc::vec::Vec;

use contracts::{
    Mission, Point, Policy, Pose, Reason, RouteDecision, RouteProposal, State, Status, Twist,
    Vector3, WorldMap,
};
use micromath::F32Ext;

use crate::controller::step;
use crate::guard::{Rejection, check_path, check_route};
use crate::rules;

/// A pose older than this stops the turtle.
const POSE_STALE_MS: u64 = 300;
/// Intermediate waypoints only need to be passed nearby.
const WAYPOINT_TOLERANCE: f32 = 0.15;

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Arm(Mission),
    Proposal(RouteProposal),
    ButtonPressed,
    Pose(Pose),
    /// Time passing with nothing else happening; drives pose-loss detection.
    Tick,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Effect {
    Drive(Twist),
    Decision(RouteDecision),
    Status(Status),
    Policy(Policy),
}

struct Route {
    waypoints: Vec<Point>,
    next: usize,
}

pub struct Leash {
    map: WorldMap,
    policy: Policy,
    mission: Option<Mission>,
    armed_at_ms: u64,
    state: State,
    reason: Option<Reason>,
    route_seq: u32,
    last_seq: u32,
    decisions: u32,
    route: Option<Route>,
    pose: Option<(f32, f32, f32)>,
    pose_at_ms: u64,
}

impl Leash {
    pub fn new(map: WorldMap) -> Self {
        Self {
            map,
            policy: rules::open(1),
            mission: None,
            armed_at_ms: 0,
            state: State::Idle,
            reason: None,
            route_seq: 0,
            last_seq: 0,
            decisions: 0,
            route: None,
            pose: None,
            pose_at_ms: 0,
        }
    }

    /// Publishes the initial policy and status.
    pub fn start(&self, now_ms: u64, out: &mut impl FnMut(Effect)) {
        out(Effect::Policy(self.policy));
        self.publish_status(now_ms, out);
    }

    pub fn handle(&mut self, event: Event, now_ms: u64, out: &mut impl FnMut(Effect)) {
        match event {
            Event::Arm(mission) => self.arm(mission, now_ms, out),
            Event::Proposal(proposal) => self.propose(proposal, now_ms, out),
            Event::ButtonPressed => self.toggle_policy(now_ms, out),
            Event::Pose(pose) => {
                self.pose = Some((pose.x, pose.y, pose.theta));
                self.pose_at_ms = now_ms;
                self.drive(now_ms, out);
            }
            Event::Tick => {}
        }
        if self.route.is_some() && !self.pose_is_fresh(now_ms) {
            self.abort(Reason::PoseLost, now_ms, out);
        }
    }

    fn pose_point(&self) -> Option<Point> {
        self.pose.map(|(x, y, _)| Point { x, y })
    }

    fn pose_is_fresh(&self, now_ms: u64) -> bool {
        self.pose.is_some() && now_ms.saturating_sub(self.pose_at_ms) <= POSE_STALE_MS
    }

    fn publish_status(&self, now_ms: u64, out: &mut impl FnMut(Effect)) {
        out(Effect::Status(Status {
            mission_id: self.mission.map_or(0, |m| m.mission_id),
            route_seq: self.route_seq,
            applied_policy_revision: self.policy.revision,
            state: self.state,
            reason: self.reason,
            pose: self.pose_point(),
            next_waypoint: self.route.as_ref().map(|r| r.waypoints[r.next]),
            elapsed_ms: match self.mission {
                Some(_) => now_ms.saturating_sub(self.armed_at_ms) as u32,
                None => 0,
            },
        }));
    }

    fn enter(
        &mut self,
        state: State,
        reason: Option<Reason>,
        now_ms: u64,
        out: &mut impl FnMut(Effect),
    ) {
        self.state = state;
        self.reason = reason;
        self.publish_status(now_ms, out);
    }

    fn stop_motors(out: &mut impl FnMut(Effect)) {
        out(Effect::Drive(Twist::default()));
    }

    fn abort(&mut self, reason: Reason, now_ms: u64, out: &mut impl FnMut(Effect)) {
        self.route = None;
        Self::stop_motors(out);
        self.enter(State::AwaitingRoute, Some(reason), now_ms, out);
    }

    fn arm(&mut self, mission: Mission, now_ms: u64, out: &mut impl FnMut(Effect)) {
        if self
            .mission
            .is_some_and(|m| m.mission_id == mission.mission_id)
        {
            return;
        }
        self.mission = Some(mission);
        self.armed_at_ms = now_ms;
        self.last_seq = 0;
        self.route_seq = 0;
        if self.route.take().is_some() {
            Self::stop_motors(out);
        }
        self.enter(State::Armed, None, now_ms, out);
    }

    fn admit(&mut self, proposal: &RouteProposal, now_ms: u64) -> Result<(), Rejection> {
        let mission = self.mission.ok_or(Reason::NotArmed)?;
        if proposal.mission_id != mission.mission_id {
            return Err(Reason::WrongMission.into());
        }
        if proposal.seq <= self.last_seq {
            return Err(Reason::StaleSequence.into());
        }
        if self.route.is_some() {
            return Err(Reason::RouteBusy.into());
        }
        // A newer sequence is used up from here on, even if it is rejected.
        self.last_seq = proposal.seq;
        if proposal.observed_policy_revision != self.policy.revision {
            return Err(Reason::PolicyChanged.into());
        }
        if !self.pose_is_fresh(now_ms) {
            return Err(Reason::StalePose.into());
        }
        let pose = self.pose_point().ok_or(Reason::StalePose)?;
        check_route(
            pose,
            &proposal.waypoints,
            mission.destination,
            &self.map,
            &self.policy,
        )
    }

    fn propose(&mut self, proposal: RouteProposal, now_ms: u64, out: &mut impl FnMut(Effect)) {
        let result = self.admit(&proposal, now_ms);
        self.decisions += 1;
        out(Effect::Decision(RouteDecision {
            decision_no: self.decisions,
            mission_id: proposal.mission_id,
            seq: proposal.seq,
            applied_policy_revision: self.policy.revision,
            accepted: result.is_ok(),
            reason: result.err().map(|r| r.reason),
            segment: result.err().and_then(|r| r.segment),
        }));
        if result.is_err() {
            return;
        }
        self.route_seq = proposal.seq;
        self.route = Some(Route {
            waypoints: proposal.waypoints,
            next: 0,
        });
        self.enter(State::Executing, None, now_ms, out);
        self.drive(now_ms, out);
    }

    fn remaining_route_is_allowed(&self) -> bool {
        let (Some(route), Some(pose)) = (&self.route, self.pose_point()) else {
            return true;
        };
        check_path(
            pose,
            &route.waypoints[route.next..],
            &self.map,
            &self.policy,
            self.policy.live_clearance,
        )
        .is_ok()
    }

    fn toggle_policy(&mut self, now_ms: u64, out: &mut impl FnMut(Effect)) {
        self.policy = rules::toggled(&self.policy);
        out(Effect::Policy(self.policy));
        if self.route.is_some() && !self.remaining_route_is_allowed() {
            self.abort(Reason::RouteInvalidated, now_ms, out);
        } else {
            self.publish_status(now_ms, out);
        }
    }

    fn drive(&mut self, now_ms: u64, out: &mut impl FnMut(Effect)) {
        if self.route.is_none() {
            return;
        }
        if !self.remaining_route_is_allowed() {
            self.abort(Reason::RouteInvalidated, now_ms, out);
            return;
        }
        let Some((x, y, theta)) = self.pose else {
            return;
        };
        let arrival_tolerance = self.policy.arrival_tolerance;
        let target = loop {
            let Some(route) = self.route.as_mut() else {
                return;
            };
            let target = route.waypoints[route.next];
            let last = route.next + 1 == route.waypoints.len();
            let tolerance = if last {
                arrival_tolerance
            } else {
                WAYPOINT_TOLERANCE
            };
            if F32Ext::hypot(target.x - x, target.y - y) >= tolerance {
                break target;
            }
            if last {
                self.route = None;
                Self::stop_motors(out);
                self.enter(State::Reached, None, now_ms, out);
                return;
            }
            route.next += 1;
            self.publish_status(now_ms, out);
        };
        let (linear, angular) = step(target, (x, y, theta), self.policy.speed_limit);
        out(Effect::Drive(Twist {
            linear: Vector3 {
                x: linear as f64,
                ..Vector3::default()
            },
            angular: Vector3 {
                z: angular as f64,
                ..Vector3::default()
            },
        }));
    }
}
