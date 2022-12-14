#![no_std]
#![no_main]

use panic_halt as _;

//Grab the HAL and our associated drivers
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::clocks::Clock;

//Frequency of our crystal (this is currently set to the PICO default)
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//Spinlock for communication between cores
use rp2040_hal::sio::Spinlock0;
static mut LED_ON: u32 = 0;

//Function to update the LED state
fn blink_led() {
    //Claim the spinlock
    let _lock = Spinlock0::claim();

    //Now this code is inside an unsafe block to avoid,
    //the compiler from yelling at us for modifying this
    //from multiple cores
    unsafe {
        //If the LED is on, turn it off
        if LED_ON == 1 {
            LED_ON = 0;
        }
        //Otherwise, turn it on
        else {
            LED_ON = 1;
        }
    }
}

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

fn core1_task () -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { cortex_m::Peripherals::steal()};
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    //Set up our watchdog timer, though we won't be using it
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    //Configure our clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    //Delay object lets us control timing
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        //Claim the spinlock
        let _lock = Spinlock0::claim();
        unsafe {
            if LED_ON == 1 {
                led_pin.set_high().unwrap();
            }
            else {
                led_pin.set_low().unwrap();
            }
        }
    }
}

static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

//THIS LINE IS VERY IMPORTANT. IT TELLS THE COMILER TO USE THIS FUNCTION AS ENTRY POINT
#[rp2040_hal::entry]
fn main () -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = hal::Sio::new(pac.SIO);
    let core = cortex_m::Peripherals::take().unwrap();

    let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _run = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

    //Set up our watchdog timer, though we won't be using it
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    //Configure our clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    //Delay object lets us control timing
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        blink_led();
        delay.delay_ms(500);
    }
}