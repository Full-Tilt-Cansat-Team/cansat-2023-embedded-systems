The first core has to start doing several things before it can enter its main loop. First, specify some metadat, including our entry point.

```rust
//func-def main
#[hal::entry]
fn main () -> ! {
	
}
```

Next, we'll set up our [[Peripherals]].

```rust
//func main
let mut pac = pac::Peripherals::take().unwrap();
let mut sio = hal::Sio::new(pac.SIO);
let core = cortex_m::Peripherals::take().unwrap();

let pins = hal::gpio::Pins::new(
	pac.IO_BANK0,
	pac.PADS_BANK0,
	sio.gpio_bank0,
	&mut pac.RESETS,
);
```

Now, we'll set up a watchdog timer. At the moment, this isn't utilized, but it will be annoyed if we don't.

```rust
//func main
let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
```

Clock configuration, alongside setting the current date

```rust
//func main
let clocks = hal::clocks::init_clocks_and_plls(
	XTAL_FREQ_HZ,
	pac.XOSC,
	pac.CLOCKS,
	pac.PLL_SYS,
	pac.PLL_USB,
	&mut pac.RESETS,
	&mut watchdog,).ok().unwrap();

let date = hal::rtc::DateTime {
	year: 2022,
	moth: 1,
	day_of_week: hal::rtc::DayOfWeek::Saturday,
	day: 21,
	hour: 16,
	minute: 25,
	second: 0,
};

let clock = RealTimeClock::new(
	pac.RTC,
	clocks.rtc_clock,
	&mut pac.RESETS,
	date
).unwrap();
```

As part of our decleration, this function may never exit, so we'll need to add a terminating infinite loop.

```rust
//func-close main
loop {}
```