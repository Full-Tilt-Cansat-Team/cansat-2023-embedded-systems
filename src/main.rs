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
use rp2040_hal::sio::Spinlock0;
use rp2040_hal::sio::Spinlock1; //Telemetry spinlock

//LED Container
static mut LED_ON: u32 = 0;

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

//Struct to contain telemetry data for passing between cores
struct TelemetryStructure {
    //Team ID
    team_id: u16,

    //Mission time in hours, minutes, seconds
    mission_time_hours: u8,
    mssion_time_minutes: u8,
    mission_time_seconds: f32,

    //Packet count
    packet_count: u16,

    //Mode (encoded as a u8)
    mode: ContainerMode,

    //State
    state: FlightState,

    //Altitude in meters (pressure-derived)
    altitude: f32,

    //Heat shield deployment state
    heat_shield_deployment_state: DeploymentState,

    //Parachute deployment state
    parachute_deployment_state: DeploymentState,

    //Mast deployment state
    mast_deployment_state: DeploymentState,

    //Temperature in degrees Celsius
    temperature: f32,

    //Voltage in volts
    voltage: f32,

    //GPS Time in hours, minutes, seconds
    gps_time_hours: u8,
    gps_time_minutes: u8,
    gps_time_seconds: f32,

    //GPS Latitude in degrees
    gps_latitude: f32,

    //GPS Longitude in degrees
    gps_longitude: f32,

    //GPS Altitude in meters
    gps_altitude: f32,

    //GPS Sat Count
    gps_sat_count: u8,

    //Last command received (as a fixed-length u8 array)
    last_command_received: [u8; 32],
}

static mut TELEMETRY: TelemetryStructure = TelemetryStructure {team_id: 0,
    mission_time_hours: 0,
    mssion_time_minutes: 0,
    mission_time_seconds: 0.0,
    packet_count: 0,
    mode: ContainerMode::Flight,
    state: FlightState::PreFlight,
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
    gps_altitude: 0.0,
    gps_sat_count: 0,
    last_command_received: [0; 32],
};   

//Function to update the LED state
fn blink_led() {
    //Claim the spinlock
    let _lock = Spinlock0::claim();

    //Now this code is inside an unsafe block to avoid,
    //the compiler from yelling at us for modifying this
    //from multiple cores
    unsafe {
        //If the LED is on, turn it off
        if LED_ON == 1 {
            LED_ON = 0;
        }
        //Otherwise, turn it on
        else {
            LED_ON = 1;
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

    //Claim the telemetry spinlock, and initialize the telemetry structure to 0 for all fields
    let _lock = Spinlock1::claim();

    unsafe {
        TELEMETRY.team_id = 0;
        TELEMETRY.mission_time_hours = 0;
        TELEMETRY.mssion_time_minutes = 0;
        TELEMETRY.mission_time_seconds = 0.0;
        TELEMETRY.packet_count = 0;
        TELEMETRY.mode = ContainerMode::Flight;
        TELEMETRY.state = FlightState::PreFlight;
        TELEMETRY.altitude = 0.0;
        TELEMETRY.heat_shield_deployment_state = DeploymentState::Undeployed;
        TELEMETRY.parachute_deployment_state = DeploymentState::Undeployed;
        TELEMETRY.mast_deployment_state = DeploymentState::Undeployed;
        TELEMETRY.temperature = 0.0;
        TELEMETRY.voltage = 0.0;
        TELEMETRY.gps_time_hours = 0;
        TELEMETRY.gps_time_minutes = 0;
        TELEMETRY.gps_time_seconds = 0.0;
        TELEMETRY.gps_latitude = 0.0;
        TELEMETRY.gps_longitude = 0.0;
        TELEMETRY.gps_altitude = 0.0;
        TELEMETRY.gps_sat_count = 0;
        TELEMETRY.last_command_received = [0; 32];
    }

    loop {
        //Claim the spinlock
        let _lock = Spinlock0::claim();
        unsafe {
            if LED_ON == 1 {
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

    let _run = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

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

    loop {
        blink_led();
        delay.delay_ms(500);
    }
}