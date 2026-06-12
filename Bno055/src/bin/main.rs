//! Sistema de adquisición de movimiento con IMU BNO055
//!
//! Este proyecto implementa la lectura de un sensor inercial
//! BNO055 en un entorno embebido basado en ESP32C6 utilizando Rust.
//!
//! El objetivo del sistema es obtener en tiempo real:
//! - Orientación absoluta del dispositivo (cuaterniones).
//! - Velocidad angular (giroscopio).
//!
//! El sensor opera en modo de fusión de sensores (NDOF),
//! combinando acelerómetro, giroscopio y magnetómetro.

#![no_std]
#![no_main]

use bno055::{BNO055OperationMode, Bno055};
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::i2c::master::I2c;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    // Inicializa logging por UART
    esp_println::logger::init_logger_from_env();

    // Inicializa periféricos del ESP32C6
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // Configuración del bus I2C
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);
    esp_println::println!("I2C initialized.");
    let mut delay = esp_hal::delay::Delay::new();

    // Inicialización del sensor
    let mut imu = Bno055::new(i2c);
    imu.init(&mut delay)
        .expect("An error occurred while building the IMU");

    // Establecer en modo NDOF
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("An error occurred while setting the IMU mode");

    // Obtener el estado de calibracion
    let mut status = imu.get_calibration_status().unwrap();
    esp_println::println!("The IMU's calibration status is: {:?}", status);

    // Wait for device to auto-calibrate.
    // Please perform steps necessary for auto-calibration to kick in.
    // Required steps are described in Datasheet section 3.11
    // Page 51, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf (As of 2021-07-02)
    esp_println::println!("- About to begin BNO055 IMU calibration...");
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        delay.delay_millis(1000);
        esp_println::println!("Calibration status: {:?}", status);
    }

    let calib = imu.calibration_profile(&mut delay).unwrap();
    imu.set_calibration_profile(calib, &mut delay).unwrap();
    esp_println::println!("       - Calibration complete!");

    loop {
        // Lectura del cuaternion
        match imu.quaternion() {
            Ok(val) => {
                esp_println::println!("IMU Quaternion: {:?}", val);
            }
            Err(e) => {
                esp_println::println!("Error reading quaternion: {:?}", e);
            }
        }

        // Lectura de la velocidad angular
        match imu.gyro_data() {
            Ok(val) => {
                esp_println::println!("IMU Gyro Data: {:?}", val);
            }
            Err(e) => {
                esp_println::println!("Error reading gyro data: {:?}", e);
            }
        }
        delay.delay_millis(1000);
    }
}
