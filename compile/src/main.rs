#![no_std]
#![no_main]

use panic_halt as _;

use rp2040_hal as hal;

use hal::{pac, rtc::RealTimeClock};

use hal::sio::Spinlock0; //Debug-reserved spinlock
use hal::sio::Spinlock1; //Telemetry structure spinlock

use embedded_hal::digital::v2::OutputPin;


#[allow(dead_code)]
#[derive(Copy, Clone)]
enum FlightState {
	PreFlight,
	Flight,
	PostFlight,
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum DeploymentState {
	Deployed,
	Undeployed,
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum ContainerMode {
	Flight,
	Simulation,
}

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

static mut TELEMETRY: Telemetry = Telemetry {
	team_id: 0,
	mission_time_hours: 0,
	mission_time_minutes: 0,
	mission_time_seconds: 0.0,
	packet_count: 0,
	container_mode: ContainerMode::Flight,
	mission_state: FlightState::PreFlight,
	altitude: 0.0,
	heat_shield_DeploymentState: DeploymentState::Undeployed,
	parachute_DeploymentState: DeploymentState::Undeployed,
	mast_DeploymentState: DeploymentState::Undeployed,
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

#[link_section = ".boot"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();


struct Telemetry {
	team_id: u32,
	mission_time_hours: u32,
	mission_time_minutes: u32,
	mission_time_seconds: f32,
	packet_count: u32,
	container_mode: ContainerMode,
	mission_state: FlightState,
	altitude: f32,
	heat_shield_DeploymentState: DeploymentState,
	parachute_DeploymentState: DeploymentState,
	mast_DeploymentState: DeploymentState,
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


impl Telemetry {
fn claim (&mut self) {
	let _lock = Spinlock1::claim();
}
}
impl Telemetry {
fn release (&mut self) {
	unsafe {let _lock = Spinlock1::release();}
}
}
impl Telemetry {
fn new() -> Self {
	Self {
		team_id: 0,
		mission_time_hours: 0,
		mission_time_minutes: 0,
		mission_time_seconds: 0.0,
		packet_count: 0,
		container_mode: ContainerMode::Flight,
		mission_state: FlightState::PreFlight,
		altitude: 0.0,
		heat_shield_DeploymentState: DeploymentState::Undeployed,
		parachute_DeploymentState: DeploymentState::Undeployed,
		mast_DeploymentState: DeploymentState::Undeployed,
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
	}
}
}
impl Telemetry {
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
		heat_shield_DeploymentState: self.heat_shield_DeploymentState,
		parachute_DeploymentState: self.parachute_DeploymentState,
		mast_DeploymentState: self.mast_DeploymentState,
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
}
impl Telemetry {
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
	self.heat_shield_DeploymentState = telemetry.heat_shield_DeploymentState;
	self.parachute_DeploymentState = telemetry.parachute_DeploymentState;
	self.mast_DeploymentState = telemetry.mast_DeploymentState;
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
fn datetime_difference(start: &hal::rtc::DateTime, end: &hal::rtc::DateTime) -> f32 {
	let mut seconds = 0.0;
	seconds += (end.year - start.year) as f32 * 365.0 * 24.0 * 60.0 * 60.0;
	seconds += (end.month - start.month) as f32 * 30.0 * 24.0 * 60.0 * 60.0;
	seconds += (end.day - start.day) as f32 * 24.0 * 60.0 * 60.0;
	seconds += (end.hour - start.hour) as f32 * 60.0 * 60.0;
	seconds += (end.minute - start.minute) as f32 * 60.0;
	seconds += (end.second - start.second) as f32;
	seconds 
}
#[hal::entry]
fn main () -> ! {
	
let mut pac = pac::Peripherals::take().unwrap();
let mut sio = hal::Sio::new(pac.SIO);
let core = cortex_m::Peripherals::take().unwrap();

let pins = hal::gpio::Pins::new(
	pac.IO_BANK0,
	pac.PADS_BANK0,
	sio.gpio_bank0,
	&mut pac.RESETS,
);
let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
let clocks = hal::clocks::init_clocks_and_plls(
	XTAL_FREQ_HZ,
	pac.XOSC,
	pac.CLOCKS,
	pac.PLL_SYS,
	pac.PLL_USB,
	&mut pac.RESETS,
	&mut watchdog,).ok().unwrap();

let date = hal::rtc::DateTime {
	year: 2022,
	month: 1,
	day_of_week: hal::rtc::DayOfWeek::Saturday,
	day: 21,
	hour: 16,
	minute: 25,
	second: 0,
};

let clock = RealTimeClock::new(
	pac.RTC,
	clocks.rtc_clock,
	&mut pac.RESETS,
	date
).unwrap();
let mut led_pin = pins.gpio25.into_push_pull_output();
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
		} else {
			led_pin.set_low().unwrap();
			led_state = 1;
		}
	}
}
}

