#![no_std]
#![no_main]

//! The board side of the demo: records in, `leash` decides, records out.
//!
//! - `board.rs` owns the hardware: heap, clocks, USART3, the LED, and the user
//!   button B1 as an owned input, plus the `PushButton` input that turns it
//!   into `ButtonPress` records.
//! - `main.rs` is the application: the records and how they connect. One
//!   `transform_join` owns the state machine and writes every result to
//!   `leash.effects`; a transform on each output record picks out its own.

extern crate alloc;

mod board;

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use aimdb_core::remote::SecurityPolicy;
use aimdb_core::transform::{JoinEventRx, JoinTrigger};
use aimdb_core::{AimDb, AimDbBuilder, Producer};
use aimdb_embassy_adapter::io::EmbassyUart;
use aimdb_embassy_adapter::{EmbassyAdapter, EmbassyBufferType, EmbassyRecordRegistrarExtCustom};
use aimdb_serial_connector::SerialServer;
use contracts::{
    BoardDiag, ButtonPress, Mission, Policy, Pose, RouteDecision, RouteProposal, Status, Twist,
    WorldMap, keys,
};
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use leash::{Effect, Event, Leash, rules};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use board::PushButton;

/// Upper bound on how late a lost pose is noticed, beyond its staleness limit.
const TICK_MS: u64 = 50;
const DIAG_MS: u64 = 500;
/// Internal to the board: not exposed over AimX.
const EFFECTS: &str = "leash.effects";
/// Room for several events' effects while the output transforms catch up.
const EFFECT_SLOTS: usize = 32;

static HANDLE_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
/// Cycle counter at a policy change that is about to stop the turtle.
static POLICY_AT: AtomicU32 = AtomicU32::new(0);
static POLICY_TO_ZERO_CYCLES: AtomicU32 = AtomicU32::new(0);
static PRESS_TO_ZERO_CYCLES: AtomicU32 = AtomicU32::new(0);

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

#[embassy_executor::task]
async fn diag_task(diag: Producer<BoardDiag>) {
    loop {
        diag.produce(BoardDiag {
            uptime_ms: now_ms(),
            heap_used: board::heap_used(),
            heap_peak: board::heap_peak(),
            presses: board::PRESSES.load(Relaxed),
            physical_presses: board::PHYSICAL_PRESSES.load(Relaxed),
            handle_cycles_max: HANDLE_CYCLES_MAX.load(Relaxed),
            policy_to_zero_cycles: POLICY_TO_ZERO_CYCLES.load(Relaxed),
            press_to_zero_cycles: PRESS_TO_ZERO_CYCLES.load(Relaxed),
        });
        Timer::after(Duration::from_millis(DIAG_MS)).await;
    }
}

/// Waits for the output transforms to subscribe, which they do when first
/// run. Otherwise the initial policy and status would be published to nobody.
const STARTUP_MS: u64 = 50;

/// The record type of each join input, turned into the state machine's event.
fn event_from(trigger: &JoinTrigger) -> Option<Event> {
    if trigger.as_input::<ButtonPress>().is_some() {
        Some(Event::ButtonPressed)
    } else if let Some(mission) = trigger.as_input::<Mission>() {
        Some(Event::Arm(*mission))
    } else if let Some(proposal) = trigger.as_input::<RouteProposal>() {
        Some(Event::Proposal(proposal.clone()))
    } else {
        trigger.as_input::<Pose>().map(|pose| Event::Pose(*pose))
    }
}

/// Owns the state machine, which is what serializes route admission, policy
/// updates and command generation. Every result goes out as one `Effect`.
async fn run_leash(mut triggers: JoinEventRx, effects: Producer<Effect>, map: WorldMap) {
    let mut leash = Leash::new(map);
    Timer::after(Duration::from_millis(STARTUP_MS)).await;
    leash.start(now_ms(), &mut |effect| effects.produce(effect));

    loop {
        let event = match select(
            triggers.recv(),
            Timer::after(Duration::from_millis(TICK_MS)),
        )
        .await
        {
            Either::First(Ok(trigger)) => match event_from(&trigger) {
                Some(event) => event,
                None => continue,
            },
            Either::First(Err(_)) => return,
            Either::Second(()) => Event::Tick,
        };

        let started = board::cycles();
        let is_press = event == Event::ButtonPressed;
        let mut stopped = false;
        leash.handle(event, now_ms(), &mut |effect| {
            if is_press && matches!(effect, Effect::Policy(_)) {
                POLICY_AT.store(board::cycles(), Relaxed);
            }
            stopped |= is_stop(&effect);
            effects.produce(effect);
        });
        if is_press && !stopped {
            POLICY_AT.store(0, Relaxed);
        }
        HANDLE_CYCLES_MAX.fetch_max(board::cycles().wrapping_sub(started), Relaxed);
    }
}

fn is_stop(effect: &Effect) -> bool {
    matches!(effect, Effect::Drive(twist) if *twist == Twist::default())
}

fn cmd_vel_from(effect: &Effect) -> Option<Twist> {
    let Effect::Drive(twist) = effect else {
        return None;
    };
    // The first zero after a policy change is what a press is timed against.
    let policy_at = POLICY_AT.swap(0, Relaxed);
    if policy_at != 0 && is_stop(effect) {
        let now = board::cycles();
        POLICY_TO_ZERO_CYCLES.store(now.wrapping_sub(policy_at), Relaxed);
        PRESS_TO_ZERO_CYCLES.store(now.wrapping_sub(board::PRESS_AT.load(Relaxed)), Relaxed);
    }
    Some(*twist)
}

fn decision_from(effect: &Effect) -> Option<RouteDecision> {
    match effect {
        Effect::Decision(decision) => Some(*decision),
        _ => None,
    }
}

fn status_from(effect: &Effect) -> Option<Status> {
    match effect {
        Effect::Status(status) => Some(*status),
        _ => None,
    }
}

fn policy_from(effect: &Effect) -> Option<Policy> {
    match effect {
        Effect::Policy(policy) => Some(*policy),
        _ => None,
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board = board::init();
    let mut led = board.led;
    let board::NucleoInputs { b1 } = board.inputs;

    // The board's own write gate: the gateway can write these and nothing else.
    let mut security = SecurityPolicy::read_write();
    security.allow_write_key(keys::POSE);
    security.allow_write_key(keys::ROUTE_PROPOSAL);
    security.allow_write_key(keys::MISSION);

    let mut builder = AimDbBuilder::new()
        .runtime(alloc::sync::Arc::new(EmbassyAdapter::new()))
        .with_connector(
            SerialServer::new(EmbassyUart::new(board.serial_rx, board.serial_tx))
                .security_policy(security),
        );

    // Four subscriber slots: the leash task, the gateway's mirror, and room
    // for remote readers.
    builder.configure::<Pose>(keys::POSE, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access();
    });
    builder.configure::<Mission>(keys::MISSION, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access();
    });
    builder.configure::<WorldMap>(keys::WORLD_MAP, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access();
    });
    builder.configure::<RouteProposal>(keys::ROUTE_PROPOSAL, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access();
    });
    builder.configure::<BoardDiag>(keys::BOARD_DIAG, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access();
    });

    // A ring, so that no press is lost. Having a source also makes AimDB refuse
    // remote writes to this record, whatever the policy says.
    builder.configure::<ButtonPress>(keys::BUTTON, |reg| {
        reg.buffer_sized::<8, 3>(EmbassyBufferType::SpmcRing)
            .with_remote_access()
            .input(PushButton::on(b1));
    });

    // The state machine: every input in, one stream of effects out.
    let map = rules::map();
    let leash_map = map.clone();
    builder.configure::<Effect>(EFFECTS, |reg| {
        reg.buffer_sized::<EFFECT_SLOTS, 4>(EmbassyBufferType::SpmcRing)
            .transform_join(|join| {
                join.input::<ButtonPress>(keys::BUTTON)
                    .input::<Mission>(keys::MISSION)
                    .input::<RouteProposal>(keys::ROUTE_PROPOSAL)
                    .input::<Pose>(keys::POSE)
                    .on_triggers(move |triggers, effects| run_leash(triggers, effects, leash_map))
            });
    });

    // Each output record takes its own effects. Having a transform also makes
    // AimDB refuse remote writes to it, whatever the policy says.
    builder.configure::<Twist>(keys::CMD_VEL, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access()
            .transform::<Effect, _>(EFFECTS, |t| t.map(cmd_vel_from));
    });
    builder.configure::<RouteDecision>(keys::ROUTE_DECISION, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access()
            .transform::<Effect, _>(EFFECTS, |t| t.map(decision_from));
    });
    builder.configure::<Status>(keys::STATUS, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access()
            .transform::<Effect, _>(EFFECTS, |t| t.map(status_from));
    });
    builder.configure::<Policy>(keys::POLICY, |reg| {
        reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
            .with_remote_access()
            .transform::<Effect, _>(EFFECTS, |t| t.map(policy_from));
    });

    static DB_CELL: StaticCell<AimDb> = StaticCell::new();
    let (db, db_runner) = builder.build().await.expect("Failed to build database");
    let db = DB_CELL.init(db);

    db.producer::<WorldMap>(keys::WORLD_MAP)
        .expect("map")
        .produce(map);

    let diag = db.producer(keys::BOARD_DIAG).expect("diag");

    spawner.spawn(unwrap!(diag_task(diag)));

    info!("up");

    // Drive the AimDB runner and the LED heartbeat concurrently.
    embassy_futures::join::join(db_runner.run(), async {
        loop {
            led.set_high();
            Timer::after(Duration::from_millis(100)).await;
            led.set_low();
            Timer::after(Duration::from_millis(900)).await;
        }
    })
    .await;
}
