#![no_std]
#![no_main]

use panic_halt as _;

//Grab the HAL and our associated drivers
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::clocks::Clock;

//Frequency of our crystal (this is currently set to the PICO default)
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//Spinlocks for communication between cores
#[allow(unused_imports)]
use rp2040_hal::sio::Spinlock0; //Debug Spinlock
use rp2040_hal::sio::Spinlock1; //Telemetry spinlock

//Enums that handle some data transfer

//Container mode
#[allow(dead_code)]
enum ContainerMode {
    Flight,
    Simulation,
}

#[allow(dead_code)]

enum DeploymentState {
    Deployed,
    Undeployed,
}
#[allow(dead_code)]

enum FlightState {
    PreFlight,
    Flight,
    PostFlight,
}

//Spinlocked global telemetry fields
#[allow(dead_code)]
static mut TEAM_ID: u32 = 0;
#[allow(dead_code)]
static mut MISSION_TIME_HOURS: u32 = 0;
#[allow(dead_code)]
static mut MISSION_TIME_MINUTES: u32 = 0;
#[allow(dead_code)]
static mut MISSION_TIME_SECONDS: f32 = 0.0;
#[allow(dead_code)]
static mut PACKET_COUNT: u32 = 0;
#[allow(dead_code)]
static mut CONTAINER_MODE: ContainerMode = ContainerMode::Flight;
#[allow(dead_code)]
static mut MISSION_STATE: FlightState = FlightState::PreFlight;
#[allow(dead_code)]
static mut ALTITUDE: f32 = 0.0;
#[allow(dead_code)]
static mut HEAT_SHIELD_DEPLOYMENT_STATE: DeploymentState = DeploymentState::Undeployed;
#[allow(dead_code)]
static mut TEMPERATURE: f32 = 0.0;
#[allow(dead_code)]
static mut VOLTAGE: f32 = 0.0;
#[allow(dead_code)]
static mut GPS_TIME_HOURS: u32 = 0;
#[allow(dead_code)]
static mut GPS_TIME_MINUTES: u32 = 0;
#[allow(dead_code)]
static mut GPS_TIME_SECONDS: f32 = 0.0;
#[allow(dead_code)]
static mut GPS_LATITUDE: f32 = 0.0;
#[allow(dead_code)]
static mut GPS_LONGITUDE: f32 = 0.0;
#[allow(dead_code)]
static mut GPS_SAT_COUNT: u32 = 0;
#[allow(dead_code)]
static mut CMD_ECHO: [u32; 32] = [0; 32];

//Function to update the LED state
fn blink_led() {
    //Claim the spinlock
    let _lock = Spinlock1::claim();

    //Now this code is inside an unsafe block to avoid,
    //the compiler from yelling at us for modifying this
    //from multiple cores
    unsafe {
        if TEAM_ID == 0 {
            TEAM_ID = 1;
        }
        else {
            TEAM_ID = 0;
        }
    }
}

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

fn core1_task () -> ! {
    //Yes, it gets annoyed if we have this unused, but I'm sure we'll use it later
    let mut pac = unsafe { pac::Peripherals::steal() };
    let _core = unsafe { cortex_m::Peripherals::steal()};

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        //Claim the spinlock
        let _lock = Spinlock1::claim();
        unsafe {
            if TEAM_ID == 0 {
                led_pin.set_high().unwrap();
            }
            else {
                led_pin.set_low().unwrap();
            }
        }
    }
}

static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

//THIS LINE IS VERY IMPORTANT. IT TELLS THE COMILER TO USE THIS FUNCTION AS ENTRY POINT
#[rp2040_hal::entry]
fn main () -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = hal::Sio::new(pac.SIO);
    let core = cortex_m::Peripherals::take().unwrap();

    let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    //Set up our watchdog timer, though we won't be using it
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    //Configure our clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    //Delay object lets us control timing
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let _run = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

    loop {
        blink_led();
        delay.delay_ms(500);
    }
}