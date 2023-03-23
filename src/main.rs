//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use hal::Clock;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use fugit::RateExtU32;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::fmt::Write;
use heapless::String;

//I2C
use embedded_hal::blocking::i2c::Read as I2CRead;
use embedded_hal::blocking::i2c::Write as I2CWrite;
use embedded_hal::blocking::i2c::WriteRead;

//BNO055
static BNO_I2C_ADDR: u8 = 0x28;
static BNO_ACC_START: u8 = 0x08;
static BNO_MAG_START: u8 = 0x0E;
static BNO_GYRO_START: u8 = 0x14;
static BNO_EUL_START: u8 = 0x1C;
static BNO_TEMP_LOC: u8 = 0x34;

// Structure to hold our BNO055 Data
#[derive(Debug)]
struct BNO055Packet {
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
    mag_x: f32,
    mag_y: f32,
    mag_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    euler_x: f32,
    euler_y: f32,
    euler_z: f32,
    temp: f32,
}

impl BNO055Packet {
    fn new() -> Self {
        BNO055Packet {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 0.0,
            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            euler_x: 0.0,
            euler_y: 0.0,
            euler_z: 0.0,
            temp: 0.0,
        }
    }
}

// Funtion to read BNO055 Data
#[allow(non_snake_case)]
fn read_BNO(
    i2c: &mut hal::I2C<
        pac::I2C0,
        (
            hal::gpio::pin::Pin<
                hal::gpio::pin::bank0::Gpio0,
                hal::gpio::pin::Function<hal::gpio::I2C>,
            >,
            hal::gpio::pin::Pin<
                hal::gpio::pin::bank0::Gpio1,
                hal::gpio::pin::Function<hal::gpio::I2C>,
            >,
        ),
    >,
) -> BNO055Packet {
    let mut bno055_packet = BNO055Packet::new();

    // Read Accelerometer Data
    let mut accel_data: [u8; 6] = [0; 6];
    i2c.write_read(BNO_I2C_ADDR, &[BNO_ACC_START], &mut accel_data)
        .unwrap();
    bno055_packet.accel_y = ((accel_data[3] as i16) << 8 | accel_data[2] as i16) as f32 / 100.00;
    bno055_packet.accel_x = ((accel_data[1] as i16) << 8 | accel_data[0] as i16) as f32 / 100.00;
    bno055_packet.accel_z = ((accel_data[5] as i16) << 8 | accel_data[4] as i16) as f32 / 100.00;

    // Read Magnetometer Data
    let mut mag_data: [u8; 6] = [0; 6];
    i2c.write_read(BNO_I2C_ADDR, &[BNO_MAG_START], &mut mag_data)
        .unwrap();
    bno055_packet.mag_x = ((mag_data[1] as i16) << 8 | mag_data[0] as i16) as f32 / 100.00;
    bno055_packet.mag_y = ((mag_data[3] as i16) << 8 | mag_data[2] as i16) as f32 / 100.00;
    bno055_packet.mag_z = ((mag_data[5] as i16) << 8 | mag_data[4] as i16) as f32 / 100.00;

    // Read Gyroscope Data
    let mut gyro_data: [u8; 6] = [0; 6];
    i2c.write_read(BNO_I2C_ADDR, &[BNO_GYRO_START], &mut gyro_data)
        .unwrap();
    bno055_packet.gyro_x = ((gyro_data[1] as i16) << 8 | gyro_data[0] as i16) as f32 / 100.00;
    bno055_packet.gyro_y = ((gyro_data[3] as i16) << 8 | gyro_data[2] as i16) as f32 / 100.00;
    bno055_packet.gyro_z = ((gyro_data[5] as i16) << 8 | gyro_data[4] as i16) as f32 / 100.00;

    // Read Euler Data
    let mut euler_data: [u8; 6] = [0; 6];
    i2c.write_read(BNO_I2C_ADDR, &[BNO_EUL_START], &mut euler_data)
        .unwrap();
    bno055_packet.euler_x = ((euler_data[1] as i16) << 8 | euler_data[0] as i16) as f32 / 100.00;
    bno055_packet.euler_y = ((euler_data[3] as i16) << 8 | euler_data[2] as i16) as f32 / 100.00;
    bno055_packet.euler_z = ((euler_data[5] as i16) << 8 | euler_data[4] as i16) as f32 / 100.00;

    // Read Temperature Data
    let mut temp_data: [u8; 1] = [0; 1];
    i2c.write_read(BNO_I2C_ADDR, &[BNO_TEMP_LOC], &mut temp_data)
        .unwrap();
    bno055_packet.temp = temp_data[0] as f32;

    bno055_packet
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set our pins to act as I2C
    let sda_pin = pins.gpio0.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<hal::gpio::FunctionI2C>();

    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;
    loop {
        //Welcome message
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            let _ = serial.write(b"Hello World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            let mut buffer = [0u8; 7];
            // 0x00 is the address of the chip ID register
            match i2c.write_read(BNO_I2C_ADDR, &[0x00], &mut buffer) {
                Ok(_) => {
                    writeln!(
                        &mut text,
                        "Current timer ticks: {}\r\nBNO055 Validation Adress: {:x?}\r\n",
                        time, buffer[0]
                    )
                    .unwrap();

                    // This is a bit of a hack, but it works
                    let _ = serial.write(text.as_bytes());

                    // Write internal oscilator
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x3F, 0x40]);

                    // Reset all interrupt status bits: 0x3F 0x01;
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x3F, 0x01]);

                    // Configure the power mode 0x3E 0x00
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x3E, 0x00]);

                    // Default axis 0x41 0x24
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x41, 0x24]);

                    // Default axis sign 0x42 0x24
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x42, 0x24]);

                    // Units to m/s^2 0x3B 0b0001000
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x3B, 0b0001000]);

                    // Acceleration only 0x3D 0x0C
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x3D, 0x0C]);

                    // Configure for high G 0x08 00010111b
                    let _ = i2c.write(BNO_I2C_ADDR, &[0x08, 0b00010111]);

                    let _ = serial.write(b"BNO055 Configuration Complete\r\n");
                }
                Err(_) => {
                    let _ = serial.write(b"Issue reading from BNO055");
                }
            }

            said_hello = true;

            let _ = serial.write(b"Checks Complete, System Booting...\r\n");
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }

        if said_hello {
            break;
        }
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        // Read BNO data
        let packet: BNO055Packet = read_BNO(&mut i2c);

        // Display BNO Acceleration Data
        let mut text: String<64> = String::new();
        let _ = write!(&mut text, "X: {} ", packet.accel_x);
        let _ = write!(&mut text, "Y: {} ", packet.accel_y);
        let _ = writeln!(&mut text, "Z: {}  \n\r ", packet.accel_z);
        let _ = serial.write(text.as_bytes());

        // Display BNO Gyro Data
        let mut text: String<64> = String::new();
        let _ = write!(&mut text, "X: {} ", packet.gyro_x);
        let _ = write!(&mut text, "Y: {} ", packet.gyro_y);
        let _ = writeln!(&mut text, "Z: {}  \n\r ", packet.gyro_z);
        let _ = serial.write(text.as_bytes());

        // Display BNO Euler Data
        let mut text: String<64> = String::new();
        let _ = write!(&mut text, "X: {} ", packet.euler_x);
        let _ = write!(&mut text, "Y: {} ", packet.euler_y);
        let _ = writeln!(&mut text, "Z: {}  \n\r ", packet.euler_z);
        let _ = serial.write(text.as_bytes());

        // Display Temperature
        let mut text: String<64> = String::new();
        let _ = writeln!(&mut text, "Temperature: {}  \n\r ", packet.temp);

        let _ = serial.write(text.as_bytes());
        delay.delay_ms(1000);
    }
}
