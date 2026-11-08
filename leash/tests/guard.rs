use contracts::{Point, Reason, WorldMap};
use leash::guard::{Rejection, check_route};
use leash::rules::{self, DESTINATION as DEST, START};

fn p(x: f32, y: f32) -> Point {
    Point { x, y }
}

fn map() -> WorldMap {
    rules::map()
}

fn reason(r: Result<(), Rejection>) -> Option<Reason> {
    r.err().map(|e| e.reason)
}

#[test]
fn straight_shortcut_passes_open_fails_restricted() {
    let m = map();
    assert_eq!(
        check_route(START, &[DEST], DEST, &m, &rules::open(1)),
        Ok(())
    );
    assert_eq!(
        reason(check_route(START, &[DEST], DEST, &m, &rules::restricted(2))),
        Some(Reason::CrossesWorkArea)
    );
}

#[test]
fn north_detour_passes_restricted() {
    let route = [p(5.0, 7.6), p(8.0, 7.6), DEST];
    assert_eq!(
        check_route(START, &route, DEST, &map(), &rules::restricted(2)),
        Ok(())
    );
}

#[test]
fn admission_clearance_is_wider_than_the_boundary() {
    let m = map();
    let c = rules::restricted(2);
    let inside = [p(5.22, 7.28), p(7.78, 7.28), DEST];
    assert_eq!(
        reason(check_route(START, &inside, DEST, &m, &c)),
        Some(Reason::CrossesWorkArea)
    );
    let outside = [p(5.18, 7.32), p(7.82, 7.32), DEST];
    assert_eq!(check_route(START, &outside, DEST, &m, &c), Ok(()));
}

#[test]
fn crossing_a_solid_with_both_endpoints_outside_names_the_segment() {
    let route = [p(5.0, 2.75), p(7.0, 2.75), DEST];
    assert_eq!(
        check_route(p(4.0, 2.75), &route, DEST, &map(), &rules::open(1)),
        Err(Rejection {
            reason: Reason::CrossesSolid,
            segment: Some(1)
        })
    );
}

#[test]
fn rejects_bad_coordinates_and_lengths() {
    let m = map();
    let f = rules::open(1);
    let check = |route: &[Point]| reason(check_route(START, route, DEST, &m, &f));
    assert_eq!(
        check(&[p(f32::INFINITY, 5.5), DEST]),
        Some(Reason::InvalidCoordinate)
    );
    assert_eq!(
        check(&[p(f32::NAN, 5.5), DEST]),
        Some(Reason::InvalidCoordinate)
    );
    assert_eq!(check(&[p(0.5, 5.5), DEST]), Some(Reason::OutOfBounds));
    assert_eq!(check(&[p(8.0, 5.5)]), Some(Reason::WrongDestination));
    assert_eq!(check(&[]), Some(Reason::BadLength));
    assert_eq!(check(&[DEST; 17]), Some(Reason::BadLength));
    assert_eq!(check(&[DEST; 16]), None);
}

#[test]
fn pose_stopped_in_the_band_can_leave_but_not_go_deeper() {
    let m = map();
    let c = rules::restricted(2);
    let in_band = p(5.35, 5.5);
    let out = [p(4.5, 5.5), p(4.5, 7.6), p(8.0, 7.6), DEST];
    assert_eq!(check_route(in_band, &out, DEST, &m, &c), Ok(()));
    assert_eq!(
        check_route(in_band, &[DEST], DEST, &m, &c),
        Err(Rejection {
            reason: Reason::CrossesWorkArea,
            segment: Some(0)
        })
    );
}

#[test]
fn pose_inside_the_area_can_leave() {
    let out = [p(4.5, 5.5), p(4.5, 7.6), p(8.0, 7.6), DEST];
    assert_eq!(
        check_route(p(5.8, 5.5), &out, DEST, &map(), &rules::restricted(2)),
        Ok(())
    );
}
