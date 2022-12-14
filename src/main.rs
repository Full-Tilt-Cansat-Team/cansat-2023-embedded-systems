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

impl Telemetry {
    //Function to hang until the spinlock is available
    fn claim (&mut self) {
        let _lock = Spinlock1::claim();
    }

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
    //Getters and setters
    fn get_team_id(&mut self) -> u32 {
        self.claim();
        let team_id = self.team_id;
        self.release();
        team_id
    }

    fn set_team_id(&mut self, team_id: u32) {
        self.claim();
        self.team_id = team_id;
        self.release();
    }

    fn get_mission_time_hours(&mut self) -> u32 {
        self.claim();
        let mission_time_hours = self.mission_time_hours;
        self.release();
        mission_time_hours
    }

    fn set_mission_time_hours(&mut self, mission_time_hours: u32) {
        self.claim();
        self.mission_time_hours = mission_time_hours;
        self.release();
    }

    fn get_mission_time_minutes(&mut self) -> u32 {
        self.claim();
        let mission_time_minutes = self.mission_time_minutes;
        self.release();
        mission_time_minutes
    }

    fn set_mission_time_minutes(&mut self, mission_time_minutes: u32) {
        self.claim();
        self.mission_time_minutes = mission_time_minutes;
        self.release();
    }

    fn get_mission_time_seconds(&mut self) -> f32 {
        self.claim();
        let mission_time_seconds = self.mission_time_seconds;
        self.release();
        mission_time_seconds
    }

    fn set_mission_time_seconds(&mut self, mission_time_seconds: f32) {
        self.claim();
        self.mission_time_seconds = mission_time_seconds;
        self.release();
    }

    fn get_packet_count(&mut self) -> u32 {
        self.claim();
        let packet_count = self.packet_count;
        self.release();
        packet_count
    }

    fn set_packet_count(&mut self, packet_count: u32) {
        self.claim();
        self.packet_count = packet_count;
        self.release();
    }

    fn get_container_mode(&mut self) -> ContainerMode {
        self.claim();
        let container_mode = self.container_mode;
        self.release();
        container_mode
    }

    fn set_container_mode(&mut self, container_mode: ContainerMode) {
        self.claim();
        self.container_mode = container_mode;
        self.release();
    }

    fn get_mission_state(&mut self) -> FlightState {
        self.claim();
        let mission_state = self.mission_state;
        self.release();
        mission_state
    }

    fn set_mission_state(&mut self, mission_state: FlightState) {
        self.claim();
        self.mission_state = mission_state;
        self.release();
    }

    fn get_altitude(&mut self) -> f32 {
        self.claim();
        let altitude = self.altitude;
        self.release();
        altitude
    }

    fn set_altitude(&mut self, altitude: f32) {
        self.claim();
        self.altitude = altitude;
        self.release();
    }

    fn get_heat_shield_deployment_state(&mut self) -> DeploymentState {
        self.claim();
        let heat_shield_deployment_state = self.heat_shield_deployment_state;
        self.release();
        heat_shield_deployment_state
    }

    fn set_heat_shield_deployment_state(&mut self, heat_shield_deployment_state: DeploymentState) {
        self.claim();
        self.heat_shield_deployment_state = heat_shield_deployment_state;
        self.release();
    }

    fn get_parachute_deployment_state(&mut self) -> DeploymentState {
        self.claim();
        let parachute_deployment_state = self.parachute_deployment_state;
        self.release();
        parachute_deployment_state
    }

    fn set_parachute_deployment_state(&mut self, parachute_deployment_state: DeploymentState) {
        self.claim();
        self.parachute_deployment_state = parachute_deployment_state;
        self.release();
    }

    fn get_mast_deployment_state(&mut self) -> DeploymentState {
        self.claim();
        let mast_deployment_state = self.mast_deployment_state;
        self.release();
        mast_deployment_state
    }

    fn set_mast_deployment_state(&mut self, mast_deployment_state: DeploymentState) {
        self.claim();
        self.mast_deployment_state = mast_deployment_state;
        self.release();
    }

    fn get_gps_latitude(&mut self) -> f32 {
        self.claim();
        let gps_latitude = self.gps_latitude;
        self.release();
        gps_latitude
    }

    fn set_gps_latitude(&mut self, gps_latitude: f32) {
        self.claim();
        self.gps_latitude = gps_latitude;
        self.release();
    }

    fn get_gps_longitude(&mut self) -> f32 {
        self.claim();
        let gps_longitude = self.gps_longitude;
        self.release();
        gps_longitude
    }

    fn set_gps_longitude(&mut self, gps_longitude: f32) {
        self.claim();
        self.gps_longitude = gps_longitude;
        self.release();
    }

    fn get_gps_altitude(&mut self) -> f32 {
        self.claim();
        let gps_altitude = self.gps_altitude;
        self.release();
        gps_altitude
    }

    fn set_gps_altitude(&mut self, gps_altitude: f32) {
        self.claim();
        self.gps_altitude = gps_altitude;
        self.release();
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