#![no_std]
#![no_main]

use panic_halt as _;

use rp2040_hal as hal;

use hal::{pac, rtc::RealTimeClock};

use rp2040_hal::sio::Spinlock0; //Debug-reserved spinlock
use rp2040_hal::sio::Spinlock1; //Telemetry structure spinlock


enum FlightState {
	PreFlight,
	Flight,
	PostFlight,
}

enum DeploymentState {
	Deployed,
	Undeployed,
}

enum ContainerMode {
	Flight,
	Simulation,
}

const XTAL_FREQ_HZ: u32 = 12_000_000u32;


struct Telemetry {
	team_id: u32,
	mission_time_hours: u32,
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


impl Telemetry {
fn claim (&mut self) {
	let _lock = Spinlock1::claim();
}
}

