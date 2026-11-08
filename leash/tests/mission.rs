//! Whole-mission behaviour, driven through events the way the board does.

use contracts::{
    Mission, Mode, Point, Policy, Pose, Reason, RouteDecision, RouteProposal, State, Status, Twist,
};
use leash::rules::{self, DESTINATION as DEST, START};
use leash::{Effect, Event, Leash};

const MISSION: u32 = 7;
const FRAME_MS: u64 = 16;

struct Rig {
    leash: Leash,
    now: u64,
    effects: Vec<Effect>,
}

fn p(x: f32, y: f32) -> Point {
    Point { x, y }
}

fn pose(at: Point) -> Pose {
    Pose {
        x: at.x,
        y: at.y,
        ..Pose::default()
    }
}

impl Rig {
    fn new() -> Self {
        Rig {
            leash: Leash::new(rules::map()),
            now: 1000,
            effects: Vec::new(),
        }
    }

    /// A mission armed at `at`, with a fresh pose.
    fn armed(at: Point) -> Self {
        let mut rig = Rig::new();
        rig.send(Event::Pose(pose(at)));
        rig.send(Event::Arm(Mission {
            mission_id: MISSION,
            destination: DEST,
        }));
        rig.effects.clear();
        rig
    }

    fn send(&mut self, event: Event) {
        let effects = &mut self.effects;
        self.leash.handle(event, self.now, &mut |e| effects.push(e));
    }

    fn pose_at(&mut self, at: Point) {
        self.now += FRAME_MS;
        self.send(Event::Pose(pose(at)));
    }

    fn propose(&mut self, seq: u32, revision: u32, waypoints: &[Point]) -> RouteDecision {
        self.effects.clear();
        self.send(Event::Proposal(RouteProposal {
            mission_id: MISSION,
            seq,
            observed_policy_revision: revision,
            waypoints: waypoints.to_vec(),
        }));
        self.decisions().pop().expect("a decision")
    }

    fn press(&mut self) {
        self.effects.clear();
        self.send(Event::ButtonPressed);
    }

    fn decisions(&self) -> Vec<RouteDecision> {
        self.effects
            .iter()
            .filter_map(|e| {
                if let Effect::Decision(d) = e {
                    Some(*d)
                } else {
                    None
                }
            })
            .collect()
    }

    fn statuses(&self) -> Vec<Status> {
        self.effects
            .iter()
            .filter_map(|e| {
                if let Effect::Status(s) = e {
                    Some(*s)
                } else {
                    None
                }
            })
            .collect()
    }

    fn last_status(&self) -> Status {
        *self.statuses().last().expect("a status")
    }

    fn policy(&self) -> Policy {
        self.effects
            .iter()
            .rev()
            .find_map(|e| {
                if let Effect::Policy(p) = e {
                    Some(*p)
                } else {
                    None
                }
            })
            .expect("a policy")
    }

    fn drives(&self) -> Vec<Twist> {
        self.effects
            .iter()
            .filter_map(|e| {
                if let Effect::Drive(t) = e {
                    Some(*t)
                } else {
                    None
                }
            })
            .collect()
    }

    fn stopped(&self) -> bool {
        self.drives().last() == Some(&Twist::default())
    }
}

fn detour() -> [Point; 3] {
    [p(5.0, 7.6), p(8.0, 7.6), DEST]
}

#[test]
fn start_publishes_policy_and_idle_status() {
    let leash = Leash::new(rules::map());
    let mut effects = Vec::new();
    leash.start(0, &mut |e| effects.push(e));
    assert!(matches!(effects[0], Effect::Policy(p) if p.mode == Mode::Open && p.revision == 1));
    assert!(matches!(effects[1], Effect::Status(s) if s.state == State::Idle));
}

#[test]
fn full_mission_with_a_press_and_a_replan() {
    let mut rig = Rig::armed(START);

    let first = rig.propose(1, 1, &[DEST]);
    assert!(first.accepted);
    assert_eq!(rig.last_status().state, State::Executing);

    rig.pose_at(p(2.4, 5.5));
    assert!(rig.drives().last().unwrap().linear.x > 0.0);

    rig.press();
    assert_eq!(rig.policy().mode, Mode::Restricted);
    assert!(rig.stopped());
    let stopped = rig.last_status();
    assert_eq!(stopped.state, State::AwaitingRoute);
    assert_eq!(stopped.reason, Some(Reason::RouteInvalidated));
    assert_eq!(stopped.applied_policy_revision, 2);
    assert_eq!(stopped.mission_id, MISSION);
    assert_eq!(stopped.pose, Some(p(2.4, 5.5)));

    let stale = rig.propose(2, 1, &detour());
    assert_eq!(stale.reason, Some(Reason::PolicyChanged));

    assert!(rig.propose(3, 2, &detour()).accepted);

    rig.effects.clear();
    for at in [p(5.0, 7.6), p(8.0, 7.6)] {
        rig.pose_at(at);
    }
    assert_eq!(rig.last_status().next_waypoint, Some(DEST));
    rig.pose_at(DEST);
    assert!(rig.stopped());
    let done = rig.last_status();
    assert_eq!(done.state, State::Reached);
    assert_eq!(done.route_seq, 3);
    assert_eq!(done.mission_id, MISSION);
}

#[test]
fn status_tracks_the_next_waypoint() {
    let mut rig = Rig::armed(START);
    let route = detour();
    assert!(rig.propose(1, 1, &route).accepted);
    assert_eq!(rig.last_status().next_waypoint, Some(route[0]));

    rig.effects.clear();
    rig.pose_at(route[0]);
    assert_eq!(rig.last_status().next_waypoint, Some(route[1]));
    assert_eq!(rig.statuses().len(), 1);

    rig.effects.clear();
    rig.now += FRAME_MS;
    rig.send(Event::Pose(pose(p(5.5, 7.6))));
    assert!(rig.statuses().is_empty(), "no status between waypoints");

    rig.now += 301;
    rig.send(Event::Tick);
    assert_eq!(rig.last_status().state, State::AwaitingRoute);
    assert_eq!(rig.last_status().next_waypoint, None);
}

#[test]
fn every_nonzero_command_respects_the_current_limit() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &detour()).accepted);
    rig.pose_at(p(1.3, 5.6));
    let open = rig.drives().iter().map(|t| t.linear.x).fold(0.0, f64::max);
    assert!(open <= 2.0);

    rig.press();
    assert_eq!(rig.last_status().state, State::Executing);
    assert_eq!(rig.last_status().applied_policy_revision, 2);

    rig.effects.clear();
    for _ in 0..20 {
        rig.pose_at(p(1.3, 5.6));
    }
    for t in rig.drives() {
        assert!(t.linear.x <= 0.8 + 1e-6, "{}", t.linear.x);
    }
}

#[test]
fn proposals_while_executing_are_busy_and_leave_status_alone() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &[DEST]).accepted);
    let busy = rig.propose(2, 1, &[DEST]);
    assert_eq!(busy.reason, Some(Reason::RouteBusy));
    assert!(rig.statuses().is_empty());
    assert!(rig.drives().is_empty());
}

#[test]
fn repeats_are_rejected_without_motion_and_can_be_told_apart() {
    let mut rig = Rig::armed(START);
    let first = rig.propose(5, 1, &[DEST]);
    assert!(first.accepted);
    rig.pose_at(DEST);
    assert_eq!(rig.last_status().state, State::Reached);

    for seq in [5, 3] {
        let d = rig.propose(seq, 1, &[p(1.5, 5.5), DEST]);
        assert_eq!(d.reason, Some(Reason::StaleSequence));
        assert!(d.decision_no > first.decision_no);
        assert!(rig.drives().is_empty());
        assert!(rig.statuses().is_empty());
    }
}

#[test]
fn a_press_with_no_active_route_keeps_the_state() {
    let mut rig = Rig::armed(START);
    rig.press();
    assert_eq!(rig.last_status().state, State::Armed);
    assert_eq!(rig.last_status().applied_policy_revision, 2);

    rig.press();
    assert!(rig.propose(1, 3, &[DEST]).accepted);
    rig.pose_at(DEST);
    rig.press();
    assert_eq!(rig.last_status().state, State::Reached);
    assert_eq!(rig.last_status().reason, None);
}

#[test]
fn losing_the_pose_stops_and_a_returning_pose_does_not_resume() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &[DEST]).accepted);
    rig.effects.clear();

    rig.now += 301;
    rig.send(Event::Tick);
    assert!(rig.stopped());
    assert_eq!(rig.last_status().reason, Some(Reason::PoseLost));

    rig.effects.clear();
    let stale = rig.propose(2, 1, &[DEST]);
    assert_eq!(stale.reason, Some(Reason::StalePose));

    rig.effects.clear();
    rig.pose_at(p(2.0, 5.5));
    assert!(rig.drives().is_empty());
    assert!(rig.propose(3, 1, &[DEST]).accepted);
}

#[test]
fn a_pose_at_the_limit_of_freshness_keeps_driving() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &[DEST]).accepted);
    rig.effects.clear();
    rig.now += 300;
    rig.send(Event::Tick);
    assert!(rig.statuses().is_empty());
}

#[test]
fn a_fresh_board_refuses_routes_until_armed() {
    let mut rig = Rig::new();
    rig.send(Event::Pose(pose(START)));
    let d = rig.propose(1, 1, &[DEST]);
    assert_eq!(d.reason, Some(Reason::NotArmed));
    assert!(rig.drives().is_empty());
}

#[test]
fn wrong_mission_does_not_use_up_a_sequence() {
    let mut rig = Rig::armed(START);
    rig.send(Event::Proposal(RouteProposal {
        mission_id: MISSION + 1,
        seq: 1,
        observed_policy_revision: 1,
        waypoints: vec![DEST],
    }));
    assert_eq!(rig.decisions()[0].reason, Some(Reason::WrongMission));
    assert!(rig.propose(1, 1, &[DEST]).accepted);
}

#[test]
fn rearming_the_same_mission_is_ignored_and_a_new_one_stops_the_route() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &[DEST]).accepted);
    rig.effects.clear();

    rig.send(Event::Arm(Mission {
        mission_id: MISSION,
        destination: DEST,
    }));
    assert!(rig.effects.is_empty());

    rig.send(Event::Arm(Mission {
        mission_id: MISSION + 1,
        destination: DEST,
    }));
    assert!(rig.stopped());
    assert_eq!(rig.last_status().state, State::Armed);
    assert_eq!(rig.last_status().mission_id, MISSION + 1);
    assert_eq!(rig.last_status().elapsed_ms, 0);
}

#[test]
fn drifting_into_the_live_clearance_cancels_the_route() {
    let mut rig = Rig::armed(START);
    rig.press();
    assert!(rig.propose(1, 2, &detour()).accepted);
    rig.effects.clear();
    // From here the line to the next waypoint crosses the work area.
    rig.pose_at(p(8.5, 5.5));
    assert!(rig.stopped());
    assert_eq!(rig.last_status().reason, Some(Reason::RouteInvalidated));
}

#[test]
fn a_late_press_can_back_out_and_finish() {
    let mut rig = Rig::armed(START);
    assert!(rig.propose(1, 1, &[DEST]).accepted);
    rig.pose_at(p(5.35, 5.5));
    rig.press();
    assert_eq!(rig.last_status().state, State::AwaitingRoute);

    let straight = rig.propose(2, 2, &[DEST]);
    assert_eq!(straight.reason, Some(Reason::CrossesWorkArea));
    assert_eq!(straight.segment, Some(0));

    let back_out = [p(4.5, 5.5), p(4.5, 7.6), p(8.0, 7.6), DEST];
    assert!(rig.propose(3, 2, &back_out).accepted);
}
