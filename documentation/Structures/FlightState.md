The Flgiht state helps the [[Telemetry Structure]] enumerate between several different possible flightstates without issues.

```rust
//init
#[allow(dead_code)]
#[derive(Copy, Clone)]
enum FlightState {
	PreFlight,
	Flight,
	PostFlight,
}
```