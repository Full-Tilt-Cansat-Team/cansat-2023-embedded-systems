
The containerstate helps the [[Telemetry Structure]] maintain multiple modes without making things terrible. It stores whether or not the container is in [[Simulation Mode]].

```rust
//init
enum ContainerMode {
	Flight,
	Simulation,
}
```