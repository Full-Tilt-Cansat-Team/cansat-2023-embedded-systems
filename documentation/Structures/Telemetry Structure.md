The telemetry structure holds much of our data for transport as part of our [[Multicore System]].

### Definition
```rust
//struct
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
```

### Functions
The claim and release functions grabs our dedicated [[Spinlock]] so that we can be safe with our [[Multicore System]].

```rust
//func-impl Telemetry
fn claim (&mut self) {
	let _lock = Spinlock1::claim();
}
```

```rust
//func-impl Telemetry
fn release (&mut self) {
	unsafe {let _lock = Spinlock1::release();}
}
```

Creating this many variables is super tedious, so we can do so here.

```rust
//func-impl Telemetry
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
```

We also have getters and setters to modify these values safely.

```rust
//func-impl Telemetry
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
```

Don't send an unsafe (global) [[Telemetry Structure]] to this function.
```rust
//func-impl Telemetry
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
```