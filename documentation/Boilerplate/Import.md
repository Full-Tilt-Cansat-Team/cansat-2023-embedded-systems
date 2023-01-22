
First, we tell our compiler not to use the Standard Template Libary or the standard main function.

```rust
//import
#![no_std]
#![no_main]
```

Let our program know that we will simply halt if we run into a panic.

```rust
//import
use panic_halt as _;
```

Then, we grab our HAL

```rust
//import
use rp2040_hal as hal;
```

We also need our [[Clock]] for keeping track of time

```rust
//import
use hal::{pac, rtc::RealTimeClock};
```

Grab our [[Spinlock]]s. These help us communicate between [[Core One]] and [[Core Two]] safely. 

```rust
//import
use hal::sio::Spinlock0; //Debug-reserved spinlock
use hal::sio::Spinlock1; //Telemetry structure spinlock
```

Set up our embedded hal to use imput

```rust
//import
use embedded_hal::digital::v2::OutputPin;
```