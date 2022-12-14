#![no_std]
#![no_main]

use panic_halt as _;

use rp2040_hal as hal;
use hal::pac;
use embedded_hal::digital::v2::OutputPin;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;


fn core1_task () -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    led_pin.set_high().unwrap();

    loop {}
}

static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

//THIS LINE IS VERY IMPORTANT. IT TELLS THE COMILER TO USE THIS FUNCTION AS ENTRY POINT
#[rp2040_hal::entry]
fn main () -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = hal::Sio::new(pac.SIO);

    let mut mc = hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _run = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

    loop {}
}