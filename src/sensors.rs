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

// Struct to hold BNO055 Data (temperature, pressure)
#[derive(Debug, Clone, Copy)]
struct BME680Packet {
    temp: f32,
    pressure: f32,
}

// Default constructor for BME680Packet
impl Default for BME680Packet {
    fn default() -> Self {
        Self {
            temp: 0.0,
            pressure: 0.0,
        }
    }
}

static BME_I2C_ADDR: u8 = 0x76;
static BME_PRESS_START: u8 = 0xF7;
static BME_TEMP_START: u8 = 0xFA;

// Function to read BME680 Data
#[allow(non_snake_case)]
fn read_BME (
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
) -> BME680Packet {
    // Create BME680 Object
    let mut bme680 = BME680::default();

    // Read Pressure Data
    let mut pressure_data: [u8; 2] = [0; 2];
    i2c.write_read(BME_I2C_ADDR, &[BME_PRESS_START], &mut pressure_data)
        .unwrap();
    bme680.pressure = ((pressure_data[0] as u32) << 8 | pressure_data[1] as u32) as f32 / 100.00;

    // Read Temperature Data
    let mut temp_data: [u8; 2] = [0; 2];
    i2c.write_read(BME_I2C_ADDR, &[BME_TEMP_START], &mut temp_data)
        .unwrap();
    bme680.temperature = ((temp_data[0] as u32) << 8 | temp_data[1] as u32) as f32 / 100.00;

    bme680
}

// Function to return data from the CAM-M8C GPS from serial as a String
fn read_GPS (serial: &mut hal::serial::Serial<pac::UART0>) -> String {
    let mut gps_data = String::new();
    let mut gps_buffer: [u8; 1] = [0; 1];
    loop {
        if let Ok(_) = serial.read(&mut gps_buffer) {
            if gps_buffer[0] == 0x0A {
                break;
            }
            gps_data.push(gps_buffer[0] as char);
        }
    }
    gps_data
}

// Function to read a pin as ADC to capture voltage
fn read_ADC (
    adc: &mut hal::adc::Adc<pac::ADC0>,
    pin: &mut hal::gpio::pin::Pin<
        hal::gpio::pin::bank0::Gpio2,
        hal::gpio::pin::Function<hal::gpio::Analog>,
    >,
) -> f32 {
    let mut adc_value: u16 = 0;
    let mut adc_voltage: f32 = 0.0;
    adc_value = adc.read(pin).unwrap();
    adc_voltage = (adc_value as f32) * 3.3 / 4096.0;
    adc_voltage
}

// Writes a pin high for 150ms to trigger the camera
fn trigger_camera (
    pin: &mut hal::gpio::pin::Pin<
        hal::gpio::pin::bank0::Gpio3,
        hal::gpio::pin::Function<hal::gpio::Output>,
    >,
) {
    pin.set_high().unwrap();
    delay_ms(150);
    pin.set_low().unwrap();
}