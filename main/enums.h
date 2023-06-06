#ifndef ENUMS
#define ENUMS
// Holds flight state
enum FlightState {
  LowPower,
  PreLaunch,
  Launch,
  Peak,
  Deployment,
  Parachute,
  Landed,
  ABORT
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

// Holds information regarding single and double-faults
enum FaultDetected {
  NoFault,
  SingleFault,
  DoubleFault
};

#endif
