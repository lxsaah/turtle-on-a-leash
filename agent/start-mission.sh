#!/bin/sh
# The trusted setup path: puts the turtle at the start and arms a new mission.
# Run it while the gateway is up. Needs the `aimdb` CLI on PATH, or in $AIMDB.
#
# The positions repeat leash/src/rules.rs.
AIMDB=${AIMDB:-aimdb}
ADMIN_SOCKET=${AIMX_ADMIN_SOCKET:-/tmp/aimdb-turtle-admin.sock}
START_X=1.25
START_Y=5.5
DESTINATION='{"x":9.0,"y":5.5}'

cd "$(dirname "$0")/.." || exit 1

# `off` is quoted because YAML reads the bare word as a boolean.
ros() {
    docker compose -f docker/docker-compose.yml exec -T turtlesim \
        bash -c "source /opt/ros/jazzy/setup.bash; ros2 service call $1" >/dev/null || exit 1
}

ros "/turtle1/set_pen turtlesim/srv/SetPen '{\"off\": 1}'"
ros "/turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute '{x: $START_X, y: $START_Y, theta: 0.0}'"
ros "/turtle1/set_pen turtlesim/srv/SetPen '{r: 179, g: 184, b: 255, width: 3, \"off\": 0}'"
ros "/clear std_srvs/srv/Empty"

mission_id=$(date +%s)
"$AIMDB" --connect "unix://$ADMIN_SOCKET" record set delivery.mission \
    "{\"mission_id\":$mission_id,\"destination\":$DESTINATION}" >/dev/null || exit 1
echo "armed mission $mission_id"
