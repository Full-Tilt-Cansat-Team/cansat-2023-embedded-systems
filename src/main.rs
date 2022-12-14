#![no_std]
#![no_main]

//Make sure to halt on panic
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

//Spinlocked structure to consolidate all of our telemetry fields
#[allow(dead_code)]
struct Telemetry {
    team_id: u32,
    mission_time_hours: u32,
    mission_time_minutes: u32,
    mission_time_seconds: f32,
    packet_count: u32,
    container_mode: ContainerMode,
    mission_state: FlightState,
    altitude: f32,
    heat_shield_deployment_state: DeploymentState,
    parachute_deployment_state: DeploymentState,
    mast_deployment_state: DeploymentState,
    temperature: f32,
    voltage: f32,
    gps_time_hours: u32,
    gps_time_minutes: u32,
    gps_time_seconds: f32,
    gps_latitude: f32,
    gps_longitude: f32,
    gps_sat_count: u32,
    cmd_echo: [u32; 32],
}

static mut TELEMETRY: Telemetry = Telemetry {
    team_id: 0,
    mission_time_hours: 0,
    mission_time_minutes: 0,
    mission_time_seconds: 0.0,
    packet_count: 0,
    container_mode: ContainerMode::Flight,
    mission_state: FlightState::PreFlight,
    altitude: 0.0,
    heat_shield_deployment_state: DeploymentState::Undeployed,
    parachute_deployment_state: DeploymentState::Undeployed,
    mast_deployment_state: DeploymentState::Undeployed,
    temperature: 0.0,
    voltage: 0.0,
    gps_time_hours: 0,
    gps_time_minutes: 0,
    gps_time_seconds: 0.0,
    gps_latitude: 0.0,
    gps_longitude: 0.0,
    gps_sat_count: 0,
    cmd_echo: [0; 32],
};

impl Telemetry {
    //Function to hang until the spinlock is available
    fn claim (&mut self) {
        let _lock = Spinlock1::claim();
    }
    //Function to reset the telemetry to hardcoded initial values
    fn reset(&mut self) {
        self.claim();
        self.team_id = 0;
        self.mission_time_hours = 0;
        self.mission_time_minutes = 0;
        self.mission_time_seconds = 0.0;
        self.packet_count = 0;
        self.container_mode = ContainerMode::Flight;
        self.mission_state = FlightState::PreFlight;
        self.altitude = 0.0;
        self.heat_shield_deployment_state = DeploymentState::Undeployed;
        self.parachute_deployment_state = DeploymentState::Undeployed;
        self.mast_deployment_state = DeploymentState::Undeployed;
        self.temperature = 0.0;
        self.voltage = 0.0;
        self.gps_time_hours = 0;
        self.gps_time_minutes = 0;
        self.gps_time_seconds = 0.0;
        self.gps_latitude = 0.0;
        self.gps_longitude = 0.0;
        self.gps_sat_count = 0;
        self.cmd_echo = [0; 32];
    } 
}

//Function to update the LED state
fn blink_led() {
    //Claim the spinlock
    let _lock = Spinlock1::claim();

    //Now this code is inside an unsafe block to avoid,
    //the compiler from yelling at us for modifying this
    //from multiple cores
    unsafe {
        if TELEMETRY.team_id == 0 {
            TELEMETRY.team_id = 1;
        }
        else {
            TELEMETRY.team_id = 0;
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
            if TELEMETRY.team_id == 0 {
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

    //Reset the telemetry
    unsafe {TELEMETRY.reset();}

    let _run = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

    loop {
        blink_led();
        delay.delay_ms(500);
    }
}