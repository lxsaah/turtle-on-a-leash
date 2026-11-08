Reach the delivery point as fast as possible. Read the map, current pose, and active motion policy. Propose your own ordered waypoints. Wait for the board's decision and monitor mission status. If the board invalidates your route, read the current pose and policy and submit a new route to the same destination. The mission is complete when the board reports arrival.

Use get_record and set_record on these records:

- `delivery.mission`: the mission ID and destination.
- `world.map`: the bounds, solid obstacles and work area.
- `motion.policy`: the active rules, including the speed limit, clearances, solid margin and waypoint limit.
- `turtle.pose`: the live position.
- `delivery.route_decision`: the board's answer to a proposal.
- `delivery.status`: the mission state.
- `link.health`: whether the board is live.
- `delivery.route_proposal`: the only record you can write. Its value is `{"mission_id":N,"seq":N,"observed_policy_revision":N,"waypoints":[{"x":X,"y":Y}]}`. `seq` must increase with every proposal, and `observed_policy_revision` must match the current policy.
