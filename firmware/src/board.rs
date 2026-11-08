//! The board: everything that touches the STM32H563ZI Nucleo's hardware.
//!
//! [`init`] brings up the heap, clocks, USART3, the LED and the user button,
//! and hands the application only what it needs. Pins never leave this module:
//! inputs come out as owned values in [`NucleoInputs`], so wiring one into two
//! records is a compile error (`use of moved value`).

use core::alloc::{GlobalAlloc, Layout};
use core::fmt::Debug;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use aimdb_core::{Input, RecordRegistrar};
use contracts::ButtonPress;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_stm32::{Config, bind_interrupts, interrupt, peripherals, usart};
use static_cell::StaticCell;

const HEAP_SIZE: usize = 131_072;
const BAUD: u32 = 921_600;
const UART_BUFFER: usize = 2048;
/// A bounce shorter than this is no press.
const DEBOUNCE_MS: u64 = 50;

static ALLOCATOR: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();
static HEAP_PEAK: AtomicU32 = AtomicU32::new(0);

/// Every press, physical or injected.
pub static PRESSES: AtomicU32 = AtomicU32::new(0);
pub static PHYSICAL_PRESSES: AtomicU32 = AtomicU32::new(0);
/// Cycle counter at the most recent press.
pub static PRESS_AT: AtomicU32 = AtomicU32::new(0);

/// A debugger write here acts as a button press. Not reachable over AimX.
#[cfg(feature = "test-hooks")]
#[unsafe(no_mangle)]
pub static TEST_PRESS: AtomicU32 = AtomicU32::new(0);

struct TrackingAllocator;

unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { ALLOCATOR.alloc(layout) };
        HEAP_PEAK.fetch_max(ALLOCATOR.used() as u32, Relaxed);
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { ALLOCATOR.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: TrackingAllocator = TrackingAllocator;

bind_interrupts!(struct Irqs {
    EXTI13 => exti::InterruptHandler<interrupt::typelevel::EXTI13>;
    USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
});

/// What the application gets from the board.
pub struct Board {
    /// The green user LED (LD1, PB0).
    pub led: Output<'static>,
    /// USART3 (ST-LINK VCP, PD8 = TX / PD9 = RX).
    pub serial_tx: BufferedUartTx<'static>,
    /// See [`serial_tx`](Self::serial_tx).
    pub serial_rx: BufferedUartRx<'static>,
    /// The board's inputs, each an owned value.
    pub inputs: NucleoInputs,
}

/// The Nucleo's inputs.
pub struct NucleoInputs {
    /// The blue user button B1.
    pub b1: UserButton,
}

/// The blue user button B1 (PC13, high while pressed). Neither `Clone` nor
/// `Copy`: [`PushButton::on`] takes it by value.
pub struct UserButton(ExtiInput<'static, Async>);

pub fn heap_used() -> u32 {
    ALLOCATOR.used() as u32
}

pub fn heap_peak() -> u32 {
    HEAP_PEAK.load(Relaxed)
}

pub fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

fn clock_config() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        use embassy_stm32::time::Hertz;

        config.rcc.hsi = None;
        config.rcc.hsi48 = Some(Default::default());
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::BypassDigital,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL125,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb3_pre = APBPrescaler::DIV1;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.voltage_scale = VoltageScale::Scale0;
    }
    config
}

/// Brings up the board.
pub fn init() -> Board {
    {
        use core::mem::MaybeUninit;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe {
            let heap_ptr = core::ptr::addr_of_mut!(HEAP);
            ALLOCATOR.init((*heap_ptr).as_ptr() as usize, HEAP_SIZE)
        }
    }

    let p = embassy_stm32::init(clock_config());

    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();
    core_peripherals.DCB.enable_trace();
    core_peripherals.DWT.enable_cycle_counter();

    static TX_BUF: StaticCell<[u8; UART_BUFFER]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; UART_BUFFER]> = StaticCell::new();
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = BAUD;
    let uart = BufferedUart::new(
        p.USART3,
        p.PD9, // RX
        p.PD8, // TX
        TX_BUF.init([0; UART_BUFFER]),
        RX_BUF.init([0; UART_BUFFER]),
        Irqs,
        uart_config,
    )
    .unwrap();
    let (serial_tx, serial_rx) = uart.split();

    Board {
        led: Output::new(p.PB0, Level::Low, Speed::Low),
        serial_tx,
        serial_rx,
        inputs: NucleoInputs {
            b1: UserButton(ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs)),
        },
    }
}

/// Resolves when a debugger has written to [`TEST_PRESS`].
#[cfg(feature = "test-hooks")]
async fn debugger_press() {
    use embassy_time::{Duration, Timer};

    while TEST_PRESS.swap(0, Relaxed) == 0 {
        Timer::after(Duration::from_millis(2)).await;
    }
}

/// A push button input: one [`ButtonPress`] per press, debounced.
pub struct PushButton {
    button: UserButton,
}

impl PushButton {
    pub fn on(button: UserButton) -> Self {
        Self { button }
    }
}

impl Input<ButtonPress> for PushButton
where
    ButtonPress: Send + Sync + 'static + Debug + Clone,
{
    fn wire(self, reg: &mut RecordRegistrar<'_, ButtonPress>) {
        let UserButton(mut pin) = self.button;
        reg.source(move |ctx, producer| async move {
            let time = ctx.time();
            let mut press = 0;
            let report = |press| {
                PRESS_AT.store(cycles(), Relaxed);
                PRESSES.fetch_add(1, Relaxed);
                producer.produce(ButtonPress { press });
            };
            loop {
                #[cfg(feature = "test-hooks")]
                {
                    use embassy_futures::select::{Either, select};
                    if let Either::Second(()) =
                        select(pin.wait_for_rising_edge(), debugger_press()).await
                    {
                        press += 1;
                        report(press);
                        continue;
                    }
                }
                #[cfg(not(feature = "test-hooks"))]
                pin.wait_for_rising_edge().await;

                time.sleep_millis(DEBOUNCE_MS).await;
                if pin.is_low() {
                    continue;
                }
                press += 1;
                PHYSICAL_PRESSES.fetch_add(1, Relaxed);
                report(press);
                pin.wait_for_falling_edge().await;
                time.sleep_millis(DEBOUNCE_MS).await;
            }
        });
    }
}
