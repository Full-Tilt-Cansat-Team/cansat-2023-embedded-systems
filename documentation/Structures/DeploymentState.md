The Deployement state enum helps the [[Telemetry Structure]] keep track of most of handling of states between the [[Heat Shield]], the [[Parachute]] and the [[Mast]] deployment.

```rust
//init
#[allow(dead_code)]
#[derive(Copy, Clone)]
enum DeploymentState {
	Deployed,
	Undeployed,
}
```