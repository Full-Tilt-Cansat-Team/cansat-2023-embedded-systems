The Deployement state enum helps the [[Telemetry Structure]] keep track of most of handling of states between the [[Heat Shield]], the [[Parachute]] and the [[Mast]] deployment.

```rust
//init
enum DeploymentState {
	Deployed,
	Undeployed,
}
```