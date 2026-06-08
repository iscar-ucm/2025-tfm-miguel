//! Control de PWM en ESP32C6 usando esp-hal + esp-rtos
//!
//! Este programa implementa la generación de una señal PWM variable
//! utilizando el periférico MCPWM del ESP32C6.
//!
//! El duty cycle se modifica dinámicamente en forma de rampa (0 → 100 → 0),
//! con el objetivo de observar su comportamiento en hardware real.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    time::Rate,
    timer::timg::TimerGroup
};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    // Inicializa logging por UART
    esp_println::logger::init_logger_from_env();
    
    // Inicializa periféricos del ESP32C6
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // Configuración del reloj base del MCPWM (32 MHz)
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    // Configuración de interrupciones software para esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    // Timer del sistema
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // Arranque del runtime RTOS (Embassy sobre esp-rtos)
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Creación del periférico MCPWM
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    // Asignación del timer al operador PWM
    mcpwm.operator0.set_timer(&mcpwm.timer0);

    // Configuración del pin de salida PWM (GPIO9)
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(peripherals.GPIO9, PwmPinConfig::UP_ACTIVE_HIGH);

    // GPIO para el sentido del giro
    let config = OutputConfig::default();
    let mut led = Output::new(peripherals.GPIO15, Level::High, config);

    // Periodo de 100 ticks (0..99) con frecuencia de 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    pwm_pin.set_timestamp(100);
    mcpwm.timer0.start(timer_clock_cfg);
    let mut val: u16 = 100;
    let mut toggle = false;

    loop {
        if toggle {
            if val < 100 {
                val += 10;
            } else {
                led.set_high();
                toggle = false;
            }
        } else {
            if val > 0 {
                val -= 10;
            } else {
                led.set_low();
                toggle = true;
            }
        }

        esp_println::println!("PWM: {}", val);
        pwm_pin.set_timestamp(val);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
