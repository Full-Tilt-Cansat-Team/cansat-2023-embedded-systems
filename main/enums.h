// Holds flight state
enum FlightState {
  LowPower,
  PreLaunch,
  Launch,
  Peak,
  Deployment,
  Parachute,
  Landed
};

// Holds deployment states
enum DeploymentState {
  NotDeployed,
  Deployed
};

// Holds toggle states
enum OptionalToggleState {
  ToggleOn,
  ToggleOff
};

// Holds computer mode
enum FlightMode {
  Flight,
  Simulation
};