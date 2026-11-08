#![no_std]
#![no_main]

//! The board side of the demo: records in, `leash` decides, records out.
//!
//! - `board.rs` owns the hardware: heap, clocks, USART3, the LED, and the user
//!   button B1 as an owned input, plus the `PushButton` input that turns it
//!   into `ButtonPress` records.
//! - `main.rs` is the application: the records, and the one task that owns the
//!   state machine, which is what serializes route admission, policy updates
//!   and command generation.

extern crate alloc;

mod board;

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use aimdb_core::buffer::Reader;
use aimdb_core::remote::SecurityPolicy;
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
use embassy_futures::select::{Either5, select5};
use embassy_time::{Duration, Instant, Timer};
use leash::{Effect, Event, Leash, rules};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use board::PushButton;

/// Upper bound on how late a lost pose is noticed, beyond its staleness limit.
const TICK_MS: u64 = 50;
const DIAG_MS: u64 = 500;

static HANDLE_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
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

struct Inputs {
    presses: Reader<ButtonPress>,
    missions: Reader<Mission>,
    proposals: Reader<RouteProposal>,
    poses: Reader<Pose>,
}

struct Outputs {
    cmd_vel: Producer<Twist>,
    decision: Producer<RouteDecision>,
    status: Producer<Status>,
    policy: Producer<Policy>,
}

impl Outputs {
    fn apply(&self, effect: Effect) {
        match effect {
            Effect::Drive(twist) => self.cmd_vel.produce(twist),
            Effect::Decision(decision) => self.decision.produce(decision),
            Effect::Status(status) => self.status.produce(status),
            Effect::Policy(policy) => self.policy.produce(policy),
        }
    }
}

#[embassy_executor::task]
async fn leash_task(mut leash: Leash, mut inputs: Inputs, outputs: Outputs) {
    leash.start(now_ms(), &mut |effect| outputs.apply(effect));
    loop {
        // Polled in order: a press is consumed before the next pose.
        let event = match select5(
            inputs.presses.recv(),
            inputs.missions.recv(),
            inputs.proposals.recv(),
            inputs.poses.recv(),
            Timer::after(Duration::from_millis(TICK_MS)),
        )
        .await
        {
            Either5::First(Ok(_)) => Event::ButtonPressed,
            Either5::Second(Ok(mission)) => Event::Arm(mission.value),
            Either5::Third(Ok(proposal)) => Event::Proposal(proposal.value),
            Either5::Fourth(Ok(pose)) => Event::Pose(pose.value),
            Either5::Fifth(()) => Event::Tick,
            _ => continue,
        };

        let started = board::cycles();
        let measure_stop = event == Event::ButtonPressed;
        let mut policy_at = None;
        leash.handle(event, now_ms(), &mut |effect| {
            let is_zero = matches!(&effect, Effect::Drive(t) if *t == Twist::default());
            if matches!(effect, Effect::Policy(_)) {
                policy_at = Some(board::cycles());
            }
            outputs.apply(effect);
            if let (true, true, Some(policy_at)) = (measure_stop, is_zero, policy_at) {
                let now = board::cycles();
                POLICY_TO_ZERO_CYCLES.store(now.wrapping_sub(policy_at), Relaxed);
                PRESS_TO_ZERO_CYCLES
                    .store(now.wrapping_sub(board::PRESS_AT.load(Relaxed)), Relaxed);
            }
        });
        HANDLE_CYCLES_MAX.fetch_max(board::cycles().wrapping_sub(started), Relaxed);
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
    macro_rules! latest {
        ($($ty:ty => $key:expr),+ $(,)?) => {$(
            builder.configure::<$ty>($key, |reg| {
                reg.buffer_sized::<1, 4>(EmbassyBufferType::SingleLatest)
                    .with_remote_access();
            });
        )+};
    }
    latest! {
        Pose => keys::POSE,
        Twist => keys::CMD_VEL,
        Mission => keys::MISSION,
        WorldMap => keys::WORLD_MAP,
        Policy => keys::POLICY,
        RouteProposal => keys::ROUTE_PROPOSAL,
        RouteDecision => keys::ROUTE_DECISION,
        Status => keys::STATUS,
        BoardDiag => keys::BOARD_DIAG,
    }
    // A ring, so that no press is lost. Having a source also makes AimDB refuse
    // remote writes to this record, whatever the policy says.
    builder.configure::<ButtonPress>(keys::BUTTON, |reg| {
        reg.buffer_sized::<8, 3>(EmbassyBufferType::SpmcRing)
            .with_remote_access()
            .input(PushButton::on(b1));
    });

    static DB_CELL: StaticCell<AimDb> = StaticCell::new();
    let (db, db_runner) = builder.build().await.expect("Failed to build database");
    let db = DB_CELL.init(db);

    let map = rules::map();
    db.producer::<WorldMap>(keys::WORLD_MAP)
        .expect("map")
        .produce(map.clone());

    let inputs = Inputs {
        presses: db.subscribe(keys::BUTTON).expect("presses"),
        missions: db.subscribe(keys::MISSION).expect("missions"),
        proposals: db.subscribe(keys::ROUTE_PROPOSAL).expect("proposals"),
        poses: db.subscribe(keys::POSE).expect("poses"),
    };
    let outputs = Outputs {
        cmd_vel: db.producer(keys::CMD_VEL).expect("cmd_vel"),
        decision: db.producer(keys::ROUTE_DECISION).expect("decision"),
        status: db.producer(keys::STATUS).expect("status"),
        policy: db.producer(keys::POLICY).expect("policy"),
    };
    let diag = db.producer(keys::BOARD_DIAG).expect("diag");

    spawner.spawn(unwrap!(leash_task(Leash::new(map), inputs, outputs)));
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
