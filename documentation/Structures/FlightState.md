The Flgiht state helps the [[Telemetry Structure]] enumerate between several different possible flightstates without issues.

```rust
//init
enum FlightState {
	PreFlight,
	Flight,
	PostFlight,
}
```