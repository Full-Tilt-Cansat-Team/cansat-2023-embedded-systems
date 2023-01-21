The telemetry structure holds much of our data for transport as part of our [[Multicore System]].

```rust
//struct
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
```