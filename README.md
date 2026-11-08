# Turtle on a Leash

An AI agent plans a delivery route. A microcontroller decides whether the turtle may drive it.

The agent gets one task: reach the delivery point as fast as possible. It reads the map, the live pose and the motion policy, and proposes its own waypoints. An STM32 running [AimDB](https://github.com/aimdb-dev/aimdb) checks every route and drives the turtle itself. Pressing the blue button on the board changes the rules mid-mission. If the current route no longer passes, the board stops the turtle and asks for a new one, and the agent replans from where it stopped.

The demo answers one question: **can an operator change how a robot is allowed to move while an agent keeps pursuing the same goal?**

This is a demonstration in a simulator, not a certified safety system.

## How it fits together

```text
Agent ──MCP──▶ aimdb-mcp ──▶ ┌── gateway (on the host) ──┐ ◀──ROS 2 / Zenoh──▶ turtlesim
Setup script ──────────────▶ │  bridges every record     │                     (Docker)
Browser viewer ◀───────────  └─────────────┬─────────────┘
                                 AimX over the ST-LINK UART
                             ┌─────────────┴─────────────┐
       blue button ────────▶ │ NUCLEO-H563ZI             │
                             │ route guard + motion      │
                             └───────────────────────────┘
```

The board is the only source of velocity commands, route decisions, mission status and policy. The agent's one writable record is its route proposal.

## What you need

- A [NUCLEO-H563ZI](https://www.st.com/en/evaluation-tools/nucleo-h563zi.html) on its ST-LINK USB port, with no other wiring
- [Docker](https://docs.docker.com/get-docker/)
- [Rust](https://rustup.rs/) (the toolchain and target are installed from `rust-toolchain.toml`)
- [probe-rs](https://probe.rs/docs/getting-started/installation/)
- The AimDB tools, at the revision this repo pins:

  ```sh
  cargo install --git https://github.com/aimdb-dev/aimdb \
      --rev 3d04efb10b3e2b5a835fade6cc9f6c2a6bfa864b --locked aimdb-cli aimdb-mcp
  ```

- [Claude Code](https://claude.com/claude-code), to run the agent. Each mission uses your Claude usage.

It has been run on macOS with Docker Desktop. On Linux, the board's serial port is usually `/dev/ttyACM0`.

## Run it

**1. Start the Zenoh router and turtlesim:**

```sh
docker compose -f docker/docker-compose.yml up -d --build
```

**2. Flash the board:**

```sh
(cd firmware && cargo build --release)
probe-rs download --chip STM32H563ZITx \
    firmware/target/thumbv8m.main-none-eabihf/release/leash-firmware
```

**3. Start the gateway** in its own terminal, using your board's port (`ls /dev/cu.usbmodem*` on macOS):

```sh
SERIAL_PORT=/dev/cu.usbmodem14503 gateway/run.sh
```

Always start it this way. `run.sh` resets the board first, and restarts the gateway if the board is lost.

**4. Open the viewer** at <http://127.0.0.1:8080/>.

**5. Arm a mission and start the agent:**

```sh
agent/start-mission.sh
agent/run.sh
```

**6. Press the blue button** once the turtle starts moving. You have about 2 seconds before it reaches the work area. The turtle stops, the agent plans a detour, and the turtle drives it under the stricter policy.

Stop everything with Ctrl-C in the gateway terminal, then `docker compose -f docker/docker-compose.yml down`.

### Without an agent

You can play the agent yourself with the `aimdb` CLI:

```sh
agent/start-mission.sh
export AIMDB_CONNECT=unix:///tmp/aimdb-turtle.sock
aimdb record get delivery.mission     # the mission_id
aimdb record get motion.policy        # the revision
aimdb record set delivery.route_proposal \
    '{"mission_id":MISSION_ID,"seq":1,"observed_policy_revision":REVISION,"waypoints":[{"x":9.0,"y":5.5}]}'
aimdb record get delivery.route_decision
```

`seq` must increase with every proposal.

## The rules

| Rule | Open | Restricted |
|---|---|---|
| Speed limit | 2.0 units/s | 0.8 units/s |
| Work area | Allowed | Closed |
| Clearance to admit a route | None | 0.30 |
| Clearance while driving | None | 0.15 |

In both modes, routes must stay inside the map, keep 0.1 from solid obstacles, use at most 16 waypoints, and end within 0.1 of the destination. Each button press toggles the policy.

The board checks every segment of a route, starting from the turtle's live pose. If a press invalidates the rest of a route, or the pose goes stale for 300 ms, the board stops the turtle and waits for a new proposal. A repeated or older proposal can never restart motion.

The numbers live in [`leash/src/rules.rs`](leash/src/rules.rs). All of them are also published in `motion.policy`, so an agent can read the rules it is held to.

## Records

| Record | Written by | Content |
|---|---|---|
| `delivery.mission` | Setup script, through its own socket | Mission ID and destination |
| `world.map` | Board | Bounds, solid obstacles, work area |
| `motion.policy` | Board | The active rules |
| `turtle.pose` | ROS 2, through the gateway | Position and heading |
| `delivery.route_proposal` | Agent | Mission, sequence, observed policy revision, waypoints |
| `delivery.route_decision` | Board | Accepted, or the reason and failing segment |
| `delivery.status` | Board | Mission state, reason, pose, next waypoint, elapsed time |
| `turtle.cmd_vel` | Board | Velocity sent to ROS 2 |
| `board.button.b1` | Board | Each button press |
| `mcu.diag` | Board | Uptime, heap and timing |
| `link.health` | Gateway | Whether the board is still live |

Writes are gated twice, at the gateway and again on the board. The velocity, decision, status, policy and button records also have an AimDB transform or source, so AimDB refuses remote writes to them whatever the write policy says.

## Repository layout

| Path | What it is |
|---|---|
| [`contracts/`](contracts) | Record types and keys, with wire shapes locked in `aimdb.lock` |
| [`leash/`](leash) | Route admission and the mission state machine, with no AimDB or board types |
| [`firmware/`](firmware) | The STM32 application |
| [`gateway/`](gateway) | The bridge between ROS 2, the board, the agent and the viewer |
| [`viewer/`](viewer) | The browser page, served by the gateway |
| [`agent/`](agent) | The prompt, the MCP configuration and the mission setup |
| [`docker/`](docker) | The Zenoh router and turtlesim |

## Development

```sh
cargo test --workspace
```

Mission behaviour is tested on the host through `leash`, without a board.

A change to a record's wire shape fails the tests until its version in `contracts/src/lib.rs` is bumped. To accept a new contract or version:

```sh
AIMDB_UPDATE_LOCK=1 cargo test -p contracts
```

AimDB is pinned to one revision in both the root `Cargo.toml` and `firmware/Cargo.toml`, because the firmware is its own workspace. Keep the two equal. The pin is ahead of AimDB 3.0.0 because the board publishes `world.map` before the gateway connects, which needs late-join snapshots.

## Known issues

- **The gateway must start through `run.sh`.** A board that outlives its gateway refuses the next one's subscriptions, so records from the board silently stay empty ([aimdb#443](https://github.com/aimdb-dev/aimdb/issues/443)).
- **`run.sh` fails if another `probe-rs` session holds the probe,** such as a `cargo run` streaming logs.
- **Reading a record with no value through MCP takes 3 seconds** ([aimdb#444](https://github.com/aimdb-dev/aimdb/issues/444)). An agent that checks `delivery.route_decision` before its first proposal pays this on the mission clock.
- **If the viewer shows no policy or status after a start,** the gateway did not receive data from the board. This was seen once, the cause is unknown, and it did not recur on later starts. Stop `run.sh` and start it again.

## License

[Apache-2.0](LICENSE)
