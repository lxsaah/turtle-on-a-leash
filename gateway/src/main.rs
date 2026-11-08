//! Bridges ROS 2 and the board, and serves records to two local sockets:
//! the agent's, which can write only route proposals, and the setup path's,
//! which can write only missions. It also serves the viewer and a read-only
//! feed for it, on loopback.
//!
//! Start it with `run.sh`, which resets the board first. A board that outlives
//! its gateway refuses the next one's subscriptions, so the records mirrored
//! from it would silently stay dead (aimdb-dev/aimdb#443).

use std::env;
use std::sync::Arc;
use std::time::Duration;

use aimdb_core::buffer::BufferCfg;
use aimdb_core::remote::{AimxConfig, SecurityPolicy};
use aimdb_core::transform::JoinEventRx;
use aimdb_core::{AimDbBuilder, Producer};
use aimdb_data_contracts::link_codecs::Json;
use aimdb_data_contracts::{LinkCodecRegistrarExt, LinkableRegistrarExt};
use aimdb_serial_connector::SerialClient;
use aimdb_tokio_adapter::{TokioAdapter, TokioRecordRegistrarExt};
use aimdb_uds_connector::UdsServer;
use aimdb_websocket_connector::WebSocketConnector;
use aimdb_zenoh_connector::{Ros2Connector, Ros2Node};
use axum::Router;
use axum::response::Html;
use axum::routing::get;
use contracts::{
    BoardDiag, ButtonPress, LinkHealth, Mission, Policy, Pose, RouteDecision, RouteProposal,
    Status, Twist, WorldMap, keys,
};
use tokio::time::{Instant, timeout};

/// Exit status `run.sh` restarts on: mirrors do not survive a board reset.
const EXIT_BOARD_LOST: i32 = 3;
const BOARD_STALE: Duration = Duration::from_millis(1500);
const HEALTH_PERIOD: Duration = Duration::from_millis(250);
const POSE_TOPIC: &str = "ros2://turtle1/pose";
const CMD_VEL_TOPIC: &str = "ros2://turtle1/cmd_vel";
const VIEWER_PAGE: &str = include_str!("../../viewer/index.html");

fn setting(name: &str, default: &str) -> String {
    env::var(name).unwrap_or_else(|_| default.into())
}

fn serial_link(key: &str) -> String {
    format!("serial://{key}")
}

/// Viewer feed. Nothing links from `ws://`, so browsers can only read.
fn viewer_link(key: &str) -> String {
    format!("ws://{key}")
}

fn local_socket(path: &str, writable: &str) -> UdsServer {
    let _ = std::fs::remove_file(path);
    UdsServer::from_config(
        AimxConfig::uds_default()
            .socket_path(path)
            .security_policy(SecurityPolicy::read_write().with_writable_key(writable)),
    )
}

/// Turns `mcu.diag` into `link.health`, and exits when the board is lost. A
/// join rather than a plain transform, because staleness is what happens when
/// no input arrives, so this has to run on a timeout too.
async fn watch_board(mut diags: JoinEventRx, health: Producer<LinkHealth>) {
    let mut last_seen = Instant::now();
    let mut board_uptime_ms = None;
    loop {
        match timeout(HEALTH_PERIOD, diags.recv()).await {
            Ok(Ok(trigger)) => {
                if let Some(diag) = trigger.as_input::<BoardDiag>() {
                    last_seen = Instant::now();
                    board_uptime_ms = Some(diag.uptime_ms);
                }
            }
            Ok(Err(_)) => return,
            Err(_) => {}
        }
        let age = last_seen.elapsed();
        let board_fresh = age < BOARD_STALE;
        health.produce(LinkHealth {
            board_fresh,
            ms_since_board: age.as_millis() as u64,
            board_uptime_ms: board_uptime_ms.unwrap_or(0),
        });
        // A board never seen is not a board lost: keep waiting for it.
        if !board_fresh && board_uptime_ms.is_some() {
            eprintln!("gateway: no data from the board for {age:?}, exiting");
            std::process::exit(EXIT_BOARD_LOST);
        }
    }
}

#[tokio::main]
async fn main() {
    let router = setting("ZENOH_ENDPOINT", "tcp/127.0.0.1:7447");
    let agent_socket = setting("AIMX_SOCKET", "/tmp/aimdb-turtle.sock");
    let admin_socket = setting("AIMX_ADMIN_SOCKET", "/tmp/aimdb-turtle-admin.sock");
    let port = env::var("SERIAL_PORT").expect("set SERIAL_PORT, e.g. /dev/cu.usbmodem14503");
    let viewer_addr = setting("VIEWER_ADDR", "127.0.0.1:8080");
    let baud: u32 = setting("SERIAL_BAUD", "921600")
        .parse()
        .expect("SERIAL_BAUD");

    let ros2 = Ros2Connector::new(
        router.as_str(),
        Ros2Node::new("turtle_gateway").namespace("/aimdb"),
    )
    .register::<Pose>()
    .register::<Twist>();

    let mut builder = AimDbBuilder::new()
        .runtime(Arc::new(TokioAdapter::new().expect("tokio adapter")))
        .with_connector(ros2)
        .with_connector(local_socket(&agent_socket, keys::ROUTE_PROPOSAL))
        .with_connector(local_socket(&admin_socket, keys::MISSION).scheme("aimx-admin"))
        .with_connector(SerialClient::over_port(port.as_str(), baud))
        .with_connector(
            WebSocketConnector::new()
                .bind(viewer_addr.as_str())
                .path("/ws")
                .with_late_join(true)
                .with_additional_routes(
                    Router::new().route("/", get(|| async { Html(VIEWER_PAGE) })),
                ),
        );

    // Written here, forwarded to the board.
    builder.configure::<RouteProposal>(keys::ROUTE_PROPOSAL, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_to(&serial_link(keys::ROUTE_PROPOSAL))
            .linked_to(&viewer_link(keys::ROUTE_PROPOSAL));
    });
    builder.configure::<Mission>(keys::MISSION, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_to(&serial_link(keys::MISSION))
            .linked_to(&viewer_link(keys::MISSION));
    });

    // Produced by the board, mirrored here for readers.
    builder.configure::<WorldMap>(keys::WORLD_MAP, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::WORLD_MAP))
            .linked_to(&viewer_link(keys::WORLD_MAP));
    });
    builder.configure::<Policy>(keys::POLICY, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::POLICY))
            .linked_to(&viewer_link(keys::POLICY));
    });
    builder.configure::<RouteDecision>(keys::ROUTE_DECISION, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::ROUTE_DECISION))
            .linked_to(&viewer_link(keys::ROUTE_DECISION));
    });
    builder.configure::<Status>(keys::STATUS, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::STATUS))
            .linked_to(&viewer_link(keys::STATUS));
    });
    builder.configure::<ButtonPress>(keys::BUTTON, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::BUTTON));
    });
    builder.configure::<BoardDiag>(keys::BOARD_DIAG, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(&serial_link(keys::BOARD_DIAG));
    });

    // The gateway's own view of the board.
    builder.configure::<LinkHealth>(keys::LINK_HEALTH, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .transform_join(|join| {
                join.input::<BoardDiag>(keys::BOARD_DIAG)
                    .on_triggers(watch_board)
            })
            .linked_to(&viewer_link(keys::LINK_HEALTH));
    });

    // Pose and velocity are CDR toward ROS and JSON toward the board.
    builder.configure::<Pose>(keys::POSE, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from(POSE_TOPIC)
            .linked_to_with(&serial_link(keys::POSE), Json)
            .linked_to_with(&viewer_link(keys::POSE), Json);
    });
    builder.configure::<Twist>(keys::CMD_VEL, |r| {
        r.buffer(BufferCfg::SingleLatest)
            .with_remote_access()
            .linked_from_with(&serial_link(keys::CMD_VEL), Json)
            .linked_to(CMD_VEL_TOPIC);
    });

    let (_db, runner) = builder.build().await.expect("build");

    println!("gateway: router {router}, board {port}@{baud}");
    println!("gateway: agent unix://{agent_socket}, admin unix://{admin_socket}");
    println!("gateway: viewer http://{viewer_addr}/");
    runner.run().await;
}
