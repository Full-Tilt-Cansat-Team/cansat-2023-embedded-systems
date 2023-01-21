
First, we tell our compiler not to use the Standard Template Libary or the standard main function.

```rust
//import
#![no_std]
#![no_main]
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
use rp2040_hal::sio::Spinlock0; //Debug-reserved spinlock
use rp2040_hal::sio::Spinlock1; //Telemetry structure spinlock
```

