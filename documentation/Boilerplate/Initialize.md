Here, we set up our freqency for our [[Clock]]

```rust
//init
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
```

Initialize a main [[Telemetry Structure]] for our [[Multicore System]].

```rust
//init
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
```

Give our linker information on how to configure our system

```rust
//init
#[link_section = ".boot"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
```

Finally, set up our stack manually for [[Core One]]

```rust
//init
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();
```