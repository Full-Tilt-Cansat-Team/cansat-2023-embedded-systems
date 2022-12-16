#![no_std]
#![no_main]

//Make sure to halt on panic
use panic_halt as _;

//Grab the HAL and our associated drivers
use rp2040_hal as hal;
use hal::{pac, rtc::RealTimeClock};

use embedded_hal::digital::v2::OutputPin;

//Frequency of our crystal (this is currently set to the PICO default)
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//Spinlocks for communication between cores
#[allow(unused_imports)]
use rp2040_hal::sio::Spinlock0; //Debug Spinlock
use rp2040_hal::sio::Spinlock1; //Telemetry spinlock

//Enums that handle some data transfer

//Container mode
#[allow(dead_code)]
#[derive(Copy, Clone)]
enum ContainerMode {
    Flight,
    Simulation,
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum DeploymentState {
    Deployed,
    Undeployed,
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum FlightState {
    PreFlight,
    Flight,
    PostFlight,
}

//Spinlocked structure to consolidate all of our telemetry fields
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
    gps_altitude: f32,
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
    gps_altitude: 0.0,
    gps_latitude: 0.0,
    gps_longitude: 0.0,
    gps_sat_count: 0,
    cmd_echo: [0; 32],
};

//General functions for the telemetry
impl Telemetry {
    //Function to hang until the spinlock is available
    fn claim (&mut self) {
        let _lock = Spinlock1::claim();
    }

    //Function to release the spinlock, to prevent deadlocks
    fn release (&mut self) {
        unsafe {let _lock = Spinlock1::release();}
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
        self.gps_altitude = 0.0;
        self.gps_latitude = 0.0;
        self.gps_longitude = 0.0;
        self.gps_sat_count = 0;
        self.cmd_echo = [0; 32];
        self.release();
    }
}

//Getters and setters for the telemetry
#[allow(dead_code)]
impl Telemetry {
    //Deep getter
    fn get(&mut self) -> Telemetry {
        self.claim();
        let telemetry = Telemetry {
            team_id: self.team_id,
            mission_time_hours: self.mission_time_hours,
            mission_time_minutes: self.mission_time_minutes,
            mission_time_seconds: self.mission_time_seconds,
            packet_count: self.packet_count,
            container_mode: self.container_mode,
            mission_state: self.mission_state,
            altitude: self.altitude,
            heat_shield_deployment_state: self.heat_shield_deployment_state,
            parachute_deployment_state: self.parachute_deployment_state,
            mast_deployment_state: self.mast_deployment_state,
            temperature: self.temperature,
            voltage: self.voltage,
            gps_time_hours: self.gps_time_hours,
            gps_time_minutes: self.gps_time_minutes,
            gps_time_seconds: self.gps_time_seconds,
            gps_altitude: self.gps_altitude,
            gps_latitude: self.gps_latitude,
            gps_longitude: self.gps_longitude,
            gps_sat_count: self.gps_sat_count,
            cmd_echo: self.cmd_echo,
        };
        self.release();
        telemetry
    }

    //Deep setter
    fn set(&mut self, telemetry: &Telemetry) {
        self.claim();
        self.team_id = telemetry.team_id;
        self.mission_time_hours = telemetry.mission_time_hours;
        self.mission_time_minutes = telemetry.mission_time_minutes;
        self.mission_time_seconds = telemetry.mission_time_seconds;
        self.packet_count = telemetry.packet_count;
        self.container_mode = telemetry.container_mode;
        self.mission_state = telemetry.mission_state;
        self.altitude = telemetry.altitude;
        self.heat_shield_deployment_state = telemetry.heat_shield_deployment_state;
        self.parachute_deployment_state = telemetry.parachute_deployment_state;
        self.mast_deployment_state = telemetry.mast_deployment_state;
        self.temperature = telemetry.temperature;
        self.voltage = telemetry.voltage;
        self.gps_time_hours = telemetry.gps_time_hours;
        self.gps_time_minutes = telemetry.gps_time_minutes;
        self.gps_time_seconds = telemetry.gps_time_seconds;
        self.gps_altitude = telemetry.gps_altitude;
        self.gps_latitude = telemetry.gps_latitude;
        self.gps_longitude = telemetry.gps_longitude;
        self.gps_sat_count = telemetry.gps_sat_count;
        self.cmd_echo = telemetry.cmd_echo;
        self.release();
    }
}

//Set up our linker
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

//Manually set up our stack
//This is not allowed to be dead code as a little warning
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

//Function to run on core 1
#[allow(dead_code)]
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
        //Safe copy the TELEMETRY global variable into our local telemetry
        let telemetry = unsafe {TELEMETRY.get()};

        //Update the LED state
        if telemetry.team_id == 0 {
            led_pin.set_low().unwrap();
        }
        else {
            led_pin.set_high().unwrap();
        }
    }
}

//THIS LINE IS VERY IMPORTANT. IT TELLS THE COMILER TO USE THIS FUNCTION AS ENTRY POINT
#[allow(unused_variables)]
#[rp2040_hal::entry]
fn main () -> ! {

    //Set up our peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = hal::Sio::new(pac.SIO);
    let core = cortex_m::Peripherals::take().unwrap();
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //Multicore setup
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

    //Initial date
    let date = hal::rtc::DateTime {
        year: 2021,
        month: 1,
        day_of_week: hal::rtc::DayOfWeek::Monday,
        day: 1,
        hour: 0,
        minute: 0,
        second: 0,
    };

    let clock = RealTimeClock::new(pac.RTC, clocks.rtc_clock, &mut pac.RESETS, date)
        .unwrap();

    let mut led_pin = pins.gpio25.into_push_pull_output();

    //Reset the telemetry
    unsafe {TELEMETRY.reset();}

    let mut led_state = 1;

    let mut start_moment = RealTimeClock::now(&clock).unwrap();

    loop {
        let current_moment = RealTimeClock::now(&clock).unwrap();
        let difference = datetime_difference(&start_moment, &current_moment);

        if difference >= 0.1 {
            start_moment = current_moment;
            if led_state == 1 {
                led_pin.set_high().unwrap();
                led_state = 0;
            }
            else {
                led_pin.set_low().unwrap();
                led_state = 1;
            }
        }
    }
}

fn datetime_difference (start: &hal::rtc::DateTime, end: &hal::rtc::DateTime) -> f32 {
    let mut seconds = 0.0;
    seconds += (end.year - start.year) as f32 * 365.0 * 24.0 * 60.0 * 60.0;
    seconds += (end.month - start.month) as f32 * 30.0 * 24.0 * 60.0 * 60.0;
    seconds += (end.day - start.day) as f32 * 24.0 * 60.0 * 60.0;
    seconds += (end.hour - start.hour) as f32 * 60.0 * 60.0;
    seconds += (end.minute - start.minute) as f32 * 60.0;
    seconds += (end.second - start.second) as f32;
    seconds
}