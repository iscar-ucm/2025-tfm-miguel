//! Aplicación embebida sobre control de actitud de un Nanosat para ESP32-C6 con Rust no_std + Embassy.
//!
//! Funciones principales:
//! - Leer la orientación y velocidad angular desde una IMU BNO055 por I2C.
//! - Ejecutar un modelo discreto xDEVS de control de actitud.
//! - Publicar telemetría por MQTT: cuaternión, error de actitud, torque y PWM.
//! - Recibir comandos por MQTT: parada/reanudación de IMU, target, yaw, PWM y Kp/Kd.
//! - Accionar un motor BL4825O mediante MCPWM.

#![no_std]
#![no_main]

use bno055::{BNO055OperationMode, Bno055};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals;
use esp_hal::{
    Async,
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{ClientConfig, ModeConfig, PowerSaveMode, ScanConfig, WifiController};
use nalgebra::Vector3;
use rust_mqtt::{
    buffer::BumpBuffer,
    client::{
        Client,
        event::{Event, Suback},
        options::{ConnectOptions, PublicationOptions, SubscriptionOptions, TopicReference},
    },
    config::SessionExpiryInterval,
    types::{MqttString, TopicName},
};
use xdevs::{
    simulator::{Config, Simulator},
    traits::AsyncInput,
};

use pd_rw_imu::DiscreteTimeModelInput;
use pd_rw_imu::types::{ImuSample, KpKdSample, Quaternion, Vec3};

// Descriptor de aplicación requerido por el bootloader ESP-IDF.
esp_bootloader_esp_idf::esp_app_desc!();

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_net::{Ipv4Address, StackResources, tcp::TcpSocket};
use static_cell::StaticCell;

/// Entradas externas que pueden llegar al modelo xDEVS.
///
/// Estas entradas se reciben desde tareas periféricas, principalmente desde:
/// * La IMU BNO055.
/// * Comandos MQTT.
/// * Ajuste dinámico de ganancias del controlador.
/// * Comandos manuales de PWM.
enum ModelInput {
    /// Muestra completa de IMU: cuaternión de actitud y velocidad angular.
    Imu(ImuSample),
    /// Nuevo cuaternión objetivo para el controlador de actitud.
    QTarget(Quaternion),
    /// Nuevos valores de ganancia proporcional y derivativa del controlador PD.
    KpKd(KpKdSample),
    /// Valor PWM externo representado como vector para mantener la interfaz del modelo.
    PWM(Vec3),
}

/// Mensajes de telemetría que se publican hacia el broker MQTT.
///
/// Se separan de `ModelInput` porque aquí el flujo es de salida:
/// modelo/sensores -> MQTT -> herramientas externas como Node-RED.
enum MqttMsg {
    /// Cuaternión medido por la IMU.
    Q(Quaternion),
    /// Error entre la actitud actual y la actitud objetivo.
    QError(Quaternion),
    /// Torque calculado por el controlador.
    Torque(Vec3),
    /// Duty cycle PWM aplicado al motor.
    PWM(f64),
}

// Tamaño de las colas internas.
const IN_QUEUE_SIZE: usize = 1;
const OUT_QUEUE_SIZE: usize = 1;
// Canal de entrada hacia el simulador xDEVS.
static IN_CHANNEL: Channel<CriticalSectionRawMutex, ModelInput, IN_QUEUE_SIZE> = Channel::new();
// Canal de entrada de la tarea MQTT. Se usa para enviar telemetría desde el modelo hacia MQTT.
static MQTT_IN_CHANNEL: Channel<CriticalSectionRawMutex, MqttMsg, 16> = Channel::new();
// Canal de salida hacia el actuador PWM.
static OUT_CHANNEL: Channel<CriticalSectionRawMutex, Vec3, OUT_QUEUE_SIZE> = Channel::new();

// Credenciales de la red WiFi usada por el ESP32-C6.
const SSID: &str = "...";
const PASSWORD: &str = "...";

// StaticCell permite crear datos estáticos inicializados en runtime, necesario en no_std.
static RADIO_INIT: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
static RX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
static TX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
static MQTT_MEMORY: StaticCell<[u8; 256]> = StaticCell::new();
static MQTT_BUMP: StaticCell<BumpBuffer<'static>> = StaticCell::new();

static IMU_RUNNING: AtomicBool = AtomicBool::new(true);
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(1, 1, 1, 1);
const PORT: u16 = 1883;

/// Configura el controlador WiFi en modo cliente.
///
/// Esta función:
/// 1. Desactiva el ahorro de energía WiFi.
/// 2. Configura SSID y contraseña.
/// 3. Aplica la configuración al controlador.
/// 4. Inicia el subsistema WiFi.
///
/// # Argumentos
/// * `controller` - Referencia mutable al controlador WiFi.
fn configure_wifi(controller: &mut WifiController<'_>) {
    controller.set_power_saving(PowerSaveMode::None).unwrap();

    let client_config = ModeConfig::Client(
        ClientConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    let res = controller.set_config(&client_config);
    esp_println::println!("[MQTT] Configuración WiFi aplicada: {:?}", res);

    match controller.start() {
        Ok(_) => {
            esp_println::println!("[MQTT] WiFi iniciado correctamente");
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error al iniciar WiFi: {:?}", e);
        }
    }
}

/// Escanea redes WiFi disponibles.
///
/// Este método solicita al controlador WiFi que busque redes cercanas
/// y devuelve una lista de puntos de acceso detectados.
///
/// # Argumentos
/// * `controller` - controlador WiFi ya inicializado y arrancado.
fn scan_wifi(controller: &mut WifiController<'_>) {
    esp_println::println!("[MQTT] Iniciando escaneo de redes WiFi");
    let scan_config = ScanConfig::default().with_max(10);

    match controller.scan_with_config(scan_config) {
        Ok(res) => {
            esp_println::println!("[MQTT] Redes encontradas:");
            for ap in res {
                esp_println::println!("[MQTT] {:?}", ap);
            }
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error al escanear WiFi: {:?}", e);
        }
    }
}

/// Conecta el dispositivo a una red WiFi previamente configurada.
///
/// Esta función inicia el proceso de conexión utilizando la configuración
/// ya establecida en el `WifiController`. Después espera en bucle hasta
/// que la conexión se complete correctamente.
///
/// # Argumentos
/// * `controller` - controlador WiFi inicializado.
fn connect_wifi(controller: &mut WifiController<'_>) {
    esp_println::println!("[MQTT] Iniciando conexión WiFi");
    match controller.connect() {
        Ok(_) => {
            esp_println::println!("[MQTT] Conexión WiFi iniciada correctamente");
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error al iniciar conexión WiFi: {:?}", e);
        }
    }

    esp_println::println!("[MQTT] Esperando conexión WiFi...");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => panic!("{:?}", err),
        }
    }
    esp_println::println!("[MQTT] WiFi conectado correctamente");
}

/// Tarea de red Embassy.
///
/// Ejecuta el `Runner` del stack TCP/IP de forma continua.
/// Esta tarea es obligatoria para que la pila de red funcione.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, esp_radio::wifi::WifiDevice<'static>>) {
    runner.run().await
}

/// Tarea MQTT principal.
///
/// Esta tarea realiza dos funciones en paralelo:
/// 1. Escucha mensajes publicados por el broker en los tópicos de control.
/// 2. Publica telemetría generada por el modelo o por los sensores.
///
/// El uso de `select` permite reaccionar tanto a eventos MQTT como a mensajes
/// internos sin bloquear una ruta por esperar a la otra.
#[embassy_executor::task]
async fn mqtt_task(
    mut client: Client<'static, TcpSocket<'static>, BumpBuffer<'static>, 5, 3, 1, 16>,
) {
    // Tópicos de publicación de telemetría.
    let topic_q =
        TopicReference::Name(TopicName::new(MqttString::from_str("/TFM/Q").unwrap()).unwrap());
    let topic_q_error = TopicReference::Name(
        TopicName::new(MqttString::from_str("/TFM/Q_error").unwrap()).unwrap(),
    );
    let topic_torque =
        TopicReference::Name(TopicName::new(MqttString::from_str("/TFM/Torque").unwrap()).unwrap());
    let topic_pwm = TopicReference::Name(
        TopicName::new(MqttString::from_str("/TFM/PWM/Monitoring").unwrap()).unwrap(),
    );
    let receiver = MQTT_IN_CHANNEL.receiver();
    loop {
        // Futuro 1: eventos procedentes del broker MQTT.
        let poll_fut = client.poll();
        // Futuro 2: mensajes internos pendientes de publicar por MQTT.
        let recv_fut = receiver.receive();

        let res = select(poll_fut, recv_fut).await;
        match res {
            Either::First(result_event) => match result_event {
                Ok(event) => {
                    // Solo se procesan mensajes MQTT de tipo Publish.
                    if let Event::Publish(publish) = event {
                        if let Ok(msg) = core::str::from_utf8(&publish.message) {
                            let topic_str = publish.topic.as_ref().as_str();
                            esp_println::println!("[MQTT] TOPIC: {}", topic_str);
                            esp_println::println!("[MQTT] MENSAJE: {}", msg);
                            // Comando de parada/reanudación de la IMU.
                            if topic_str == "/TFM/Stop" {
                                match msg.trim() {
                                    "1" => {
                                        IMU_RUNNING.store(false, Ordering::Relaxed);
                                        esp_println::println!("[MQTT] IMU Parado");
                                    }
                                    "0" => {
                                        IMU_RUNNING.store(true, Ordering::Relaxed);
                                        esp_println::println!("[MQTT] IMU Reanudado");
                                    }
                                    _ => {
                                        esp_println::println!(
                                            "[MQTT] Escribe '1' para parar la IMU o '0' para reanudarla"
                                        );
                                    }
                                }
                            // Comando de target completo como cuaternión: x,y,z,w.
                            } else if topic_str == "/TFM/Target" {
                                let mut parts = msg.split(',');

                                let qx = parts.next().and_then(|v| v.parse::<f64>().ok());
                                let qy = parts.next().and_then(|v| v.parse::<f64>().ok());
                                let qz = parts.next().and_then(|v| v.parse::<f64>().ok());
                                let qw = parts.next().and_then(|v| v.parse::<f64>().ok());

                                if let (Some(x), Some(y), Some(z), Some(w)) = (qx, qy, qz, qw) {
                                    let q_target =
                                        Quaternion(nalgebra::Quaternion::new(w, x, y, z));
                                    esp_println::println!(
                                        "[MQTT] Q_TARGET recibido: {:?}",
                                        q_target
                                    );
                                } else {
                                    esp_println::println!("[MQTT] Target inválido: {}", msg);
                                }
                            // Comando simplificado: objetivo definido únicamente por yaw en grados.
                            } else if topic_str == "/TFM/Target/Yaw" {
                                if let Ok(yaw_deg) = msg.trim().parse::<f64>() {
                                    let q_target = Quaternion::from_yaw_deg(yaw_deg);
                                    esp_println::println!(
                                        "[MQTT] Target Yaw recibido: {} grados -> Cuaternión: {:?}",
                                        yaw_deg,
                                        q_target
                                    );
                                    IN_CHANNEL.send(ModelInput::QTarget(q_target)).await;
                                } else {
                                    esp_println::println!("[MQTT] Target Yaw inválido: {}", msg);
                                }
                            // Comando manual de PWM. Se encapsula en Vec3 usando el eje z.
                            } else if topic_str == "/TFM/PWM" {
                                if let Ok(pwm) = msg.trim().parse::<f64>() {
                                    IN_CHANNEL
                                        .send(ModelInput::PWM(Vec3(nalgebra::Vector3::new(
                                            0.0, 0.0, pwm,
                                        ))))
                                        .await;
                                    esp_println::println!("[MQTT] PWM recibido: {}", pwm);
                                } else {
                                    esp_println::println!("[MQTT] PWM inválido: {}", msg);
                                }
                            // Ajuste remoto de ganancias del controlador PD: Kp,Kd.
                            } else if topic_str == "/TFM/Kp_Kd" {
                                let mut parts = msg.split(',');

                                let kp = parts.next().and_then(|v| v.parse::<f64>().ok());
                                let kd = parts.next().and_then(|v| v.parse::<f64>().ok());

                                if let (Some(kp), Some(kd)) = (kp, kd) {
                                    let kp_kd_sample = KpKdSample { kp, kd };
                                    esp_println::println!(
                                        "[MQTT] Kp_Kd recibido: Kp={}, Kd={}",
                                        kp,
                                        kd
                                    );
                                    IN_CHANNEL.send(ModelInput::KpKd(kp_kd_sample)).await;
                                } else {
                                    esp_println::println!("[MQTT] Kp_Kd inválido: {}", msg);
                                }
                            }
                        } else {
                            esp_println::println!("[MQTT] Error: Mensaje no es texto UTF-8");
                        }
                    }
                }
                Err(e) => {
                    esp_println::println!("[MQTT] error: {:?}", e);
                }
            },
            Either::Second(msg) => match msg {
                // Publicación del cuaternión medido.
                MqttMsg::Q(q) => {
                    let yaw = q.yaw_from_quaternion();
                    let mut buf = [0u8; 40];

                    buf[0..8].copy_from_slice(&q.0.i.to_le_bytes());
                    buf[8..16].copy_from_slice(&q.0.j.to_le_bytes());
                    buf[16..24].copy_from_slice(&q.0.k.to_le_bytes());
                    buf[24..32].copy_from_slice(&q.0.w.to_le_bytes());
                    buf[32..40].copy_from_slice(&yaw.to_le_bytes());

                    let _ = client
                        .publish(
                            &PublicationOptions::new(topic_q.as_borrowed()),
                            rust_mqtt::Bytes::Borrowed(&buf),
                        )
                        .await;
                }
                // Publicación del error de actitud calculado por el modelo.
                MqttMsg::QError(q) => {
                    let yaw = q.yaw_from_quaternion();
                    let mut buf = [0u8; 40];

                    buf[0..8].copy_from_slice(&q.0.i.to_le_bytes());
                    buf[8..16].copy_from_slice(&q.0.j.to_le_bytes());
                    buf[16..24].copy_from_slice(&q.0.k.to_le_bytes());
                    buf[24..32].copy_from_slice(&q.0.w.to_le_bytes());
                    buf[32..40].copy_from_slice(&yaw.to_le_bytes());

                    let _ = client
                        .publish(
                            &PublicationOptions::new(topic_q_error.as_borrowed()),
                            rust_mqtt::Bytes::Borrowed(&buf),
                        )
                        .await;
                }
                // Publicación del torque de control.
                MqttMsg::Torque(t) => {
                    let mut buf = [0u8; 24];

                    buf[0..8].copy_from_slice(&t.0.x.to_le_bytes());
                    buf[8..16].copy_from_slice(&t.0.y.to_le_bytes());
                    buf[16..24].copy_from_slice(&t.0.z.to_le_bytes());

                    let _ = client
                        .publish(
                            &PublicationOptions::new(topic_torque.as_borrowed()),
                            rust_mqtt::Bytes::Borrowed(&buf),
                        )
                        .await;
                }
                // Publicación del duty cycle PWM aplicado.
                MqttMsg::PWM(pwm) => {
                    let _ = client
                        .publish(
                            &PublicationOptions::new(topic_pwm.as_borrowed()),
                            rust_mqtt::Bytes::Borrowed(&pwm.to_le_bytes()),
                        )
                        .await;
                }
                _ => {}
            },
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Tarea de control del motor BL4825O mediante MCPWM.
///
/// Recibe el valor de PWM desde `OUT_CHANNEL` Y actualiza el duty cycle.
#[embassy_executor::task]
async fn bl4825O_task(
    mcpwm0: peripherals::MCPWM0<'static>,
    gpio9: peripherals::GPIO20<'static>,
    gpio15: peripherals::GPIO21<'static>,
) {
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mut mcpwm = McPwm::new(mcpwm0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let mut pwm_pin_1 = mcpwm
        .operator0
        .with_pin_a(gpio9, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut output_1 = Output::new(gpio15, Level::High, OutputConfig::default());

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);
    let pwm = 50.0;
    pwm_pin_1.set_timestamp(pwm as u16);

    loop {
        // Espera un nuevo valor PWM procedente de la salida del modelo.
        let rcv = OUT_CHANNEL.receiver().receive().await;
        match rcv {
            Vec3(_) => {
                esp_println::println!("[bl4825O] Received torque {:?}", rcv);
                esp_println::println!("[bl4825O] Updated duty cycle {:?}", rcv.0.z);
                let _ = MQTT_IN_CHANNEL.try_send(MqttMsg::PWM(rcv.0.z));
                /* if rcv.0.z > 0.0 {
                    output_1.set_high();
                } else {
                    output_1.set_low();
                } */

                pwm_pin_1.set_timestamp(rcv.0.z as u16);
            }
        }
    }
}

/// Tarea de adquisición de la IMU BNO055.
///
/// Lee periódicamente el cuaternión y el giroscopio. Después:
/// * Publica el cuaternión por MQTT,
/// * Envía la muestra al modelo xDEVS mediante `IN_CHANNEL`.
#[embassy_executor::task]
async fn bno055_task(mut imu: Bno055<I2c<'static, Async>>) {
    loop {
        // Si llega un comando MQTT de parada, se pausa la lectura del sensor.
        if !IMU_RUNNING.load(Ordering::Relaxed) {
            esp_println::println!("[BNO055] IMU detenido, esperando reanudación...");
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }
        // Lectura de orientación y velocidad angular desde el BNO055.
        let q = imu.quaternion();
        let w = imu.gyro_data();

        if let (Ok(q), Ok(w)) = (q, w) {
            let q_sample = Quaternion(nalgebra::Quaternion::new(
                q.s as f64,
                q.v.x as f64,
                q.v.y as f64,
                q.v.z as f64,
            ));
            let sample = ImuSample {
                w: Vec3(Vector3::new(w.x as f64, w.y as f64, w.z as f64)),
                q: q_sample,
            };

            let _ = MQTT_IN_CHANNEL.try_send(MqttMsg::Q(sample.q));
            IN_CHANNEL.send(ModelInput::Imu(sample)).await;
            esp_println::println!("[BNO055] Quaternion: {:?}, Gyro: {:?}", sample.q, sample.w);
        }
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

/// Adaptador entre tiempo real y tiempo de simulación xDEVS.
///
/// Guarda el último instante real procesado para sincronizar la simulación
/// con los datos entrantes y controlar el jitter.
struct InputHandler {
    /// Último instante real usado como referencia para calcular el siguiente deadline.
    last_rt: Option<Instant>,
}

impl InputHandler {
    fn new() -> Self {
        Self { last_rt: None }
    }
}

// Implementación de la interfaz que xDEVS usa para solicitar entradas externas.
impl AsyncInput for InputHandler {
    type Input = DiscreteTimeModelInput;

    async fn handle(
        &mut self,
        config: &xdevs::simulator::Config,
        t_from: f64,
        t_until: f64,
        input: &mut Self::Input,
    ) -> f64 {
        // Instante real asociado al último avance de simulación.
        let last_rt = self.last_rt.unwrap_or_else(Instant::now);
        // Calcula cuánto tiempo real debe transcurrir entre t_from y t_until.
        let time_duration = (t_until - t_from) * config.time_scale;
        // Convierte la duración calculada a nanosegundos.
        let time_duration = (time_duration * 1_000_000_000.0) as u64;
        // Calcula el instante real límite hasta el que se puede esperar entrada.
        let next_rt = last_rt + Duration::from_nanos(time_duration);

        let future = async {
            // Espera al menos una entrada externa antes de avanzar el modelo.
            let rcv = IN_CHANNEL.receiver().receive().await;
            // Según el tipo de mensaje recibido, se introduce el valor en el puerto correspondiente del modelo xDEVS.
            match rcv {
                ModelInput::Imu(sample) => {
                    esp_println::println!(
                        "[INPUT HANDLER] - Received IMU Sample - Quaternion: {:?}, Gyro: {:?}",
                        sample.q,
                        sample.w
                    );
                    input.i_q.add_value(sample.q).unwrap();
                    input.i_w.add_value(sample.w).unwrap();
                }
                ModelInput::QTarget(q_target) => {
                    esp_println::println!(
                        "[INPUT HANDLER] - Received new target quaternion: {:?}",
                        q_target
                    );
                    input.i_q_target.add_value(q_target).unwrap();
                }
                ModelInput::KpKd(kp_kd_sample) => {
                    esp_println::println!(
                        "[INPUT HANDLER] - Received new Kp/Kd values: Kp={}, Kd={}",
                        kp_kd_sample.kp,
                        kp_kd_sample.kd
                    );
                    input.i_kp.add_value(kp_kd_sample.kp).unwrap();
                    input.i_kd.add_value(kp_kd_sample.kd).unwrap();
                }
                ModelInput::PWM(pwm) => {
                    esp_println::println!("[INPUT HANDLER] - Received new PWM value: {:?}", pwm);
                    input.i_pwm.add_value(pwm).unwrap();
                }
            }
        };

        // Espera a que ocurra una de estas dos cosas:
        //
        // 1. Que llegue una entrada externa antes de next_rt.
        // 2. Que se alcance el deadline next_rt sin recibir entrada.
        //
        // with_deadline devuelve Err si se alcanza el deadline.
        if let Err(_) = with_deadline(next_rt.into(), future).await {
            // No ha llegado ninguna entrada externa dentro del intervalo permitido.
            if let Some(max_jitter) = config.max_jitter {
                let jitter = Instant::now().duration_since(next_rt);
                let max_jitter_ticks = Duration::from_micros(max_jitter.as_micros() as u64);
                if jitter > max_jitter_ticks {
                    panic!("[INPUT HANDLER] - Jitter too high: {:?}", jitter);
                }
            }
            self.last_rt = Some(next_rt);
            return t_until;
        } else {
            // Ha llegado una entrada externa antes de alcanzar next_rt.

            // Instante real actual, justo después de recibir y procesar la entrada.
            let now = Instant::now();
            // Guarda este instante como nueva referencia temporal.
            self.last_rt = Some(now);
            // Tiempo real transcurrido desde la última referencia.
            let elapsed_rt = now.duration_since(last_rt).as_micros() as f64 / 1_000_000.0;
            // Conversión del tiempo real transcurrido a tiempo de simulación.
            let elapsed_sim = elapsed_rt / config.time_scale;
            esp_println::println!(
                "[INPUT HANDLER] - Received input, elapsed real time: {:?} seconds, elapsed sim time: {:?} seconds",
                elapsed_rt,
                elapsed_sim
            );
            // Devuelve el instante de simulación hasta el que se debe avanzar.
            return t_from + elapsed_sim;
        }
    }
}

/// Propaga las salidas del modelo xDEVS hacia los canales correspondientes.
///
/// Esta función se llama desde el simulador cada vez que el modelo genera salida.
/// Se encarga de enviar:
/// * Error de actitud a MQTT,
/// * PWM al actuador y a MQTT,
/// * Torque a MQTT.
fn propagate_output(output: &pd_rw_imu::DiscreteTimeModelOutput) {
    if let Some(q_error) = output.o_q_error.get_values().last() {
        let _ = MQTT_IN_CHANNEL.try_send(MqttMsg::QError(*q_error));
        esp_println::println!("[OUTPUT] q_error={:?}", q_error);
        esp_println::println!("[OUTPUT] q_error yaw={:?}", q_error.yaw_from_quaternion());
    }

    if let Some(pwm) = output.o_pwm.get_values().last() {
        let _ = OUT_CHANNEL.sender().try_send(*pwm);
        let _ = MQTT_IN_CHANNEL.try_send(MqttMsg::PWM(pwm.0.z));
        esp_println::println!("[OUTPUT] pwm={:?}", pwm);
        esp_println::println!("[OUTPUT] pwm={:?}", pwm.0.z);
    }

    if let Some(torque) = output.o_torque.get_values().last() {
        let _ = MQTT_IN_CHANNEL.try_send(MqttMsg::Torque(*torque));
        esp_println::println!("[OUTPUT] torque={:?}", torque);
        esp_println::println!("[OUTPUT] torque={:?}", torque.0.z);
    }
}

/// Punto de entrada principal de la aplicación.
///
/// Inicializa hardware, sensores, WiFi, MQTT, tareas Embassy y simulador xDEVS.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // Configuración inicial del logger y del heap dinámico.
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let stats: esp_alloc::HeapStats = esp_alloc::HEAP.stats();
    esp_println::println!("[MQTT] Heap stats: {}", stats);

    // Inicialización global de periféricos del ESP32-C6.
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let peripherals::Peripherals {
        WIFI,
        MCPWM0,
        GPIO18,
        GPIO19,
        I2C0,
        GPIO21,
        GPIO20,
        SW_INTERRUPT,
        TIMG0,
        ..
    } = peripherals;
    let sw_int = SoftwareInterruptControl::new(SW_INTERRUPT);
    let timg0 = TimerGroup::new(TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Configuración del bus I2C usado por el sensor BNO055.
    let i2c = I2c::new(I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(GPIO18)
        .with_scl(GPIO19)
        .into_async();
    esp_println::println!("[main] - I2C initialized.");

    let mut delay = esp_hal::delay::Delay::new();
    // Creación e inicialización de la IMU BNO055.
    let mut imu = Bno055::new(i2c);
    imu.init(&mut delay)
        .expect("[BNO055] - An error occurred while building the IMU");

    imu.set_mode(BNO055OperationMode::NDOF, &mut delay)
        .expect("[BNO055] - An error occurred while setting the IMU mode");

    let mut status = imu.get_calibration_status().unwrap();
    esp_println::println!("[BNO055] - The IMU's calibration status is: {:?}", status);

    // Wait for device to auto-calibrate.
    // Please perform steps necessary for auto-calibration to kick in.
    // Required steps are described in Datasheet section 3.11
    // Page 51, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf (As of 2021-07-02)
    /* esp_println::println!("[BNO055] - About to begin BNO055 IMU calibration...");
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        delay.delay_millis(500);
        esp_println::println!("[BNO055] - Calibration status: {:?}", status);
    } */

    let calib = imu.calibration_profile(&mut delay).unwrap();

    imu.set_calibration_profile(calib, &mut delay).unwrap();
    esp_println::println!("[BNO055] - Calibration complete!");

    // Lanzamiento de la tarea de lectura de la IMU.
    spawner.spawn(bno055_task(imu)).ok();

    // Inicialización del controlador de radio WiFi/BLE
    let radio_init = RADIO_INIT
        .init(esp_radio::init().expect("[MQTT] Fallo al inicializar el controlador Wi-Fi/BLE"));

    // Creación del controlador WiFi y sus interfaces (STA/AP)
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, WIFI, Default::default())
            .expect("[MQTT] Fallo al crear el controlador Wi-Fi");
    // Interface en modo estación (cliente WiFi)
    let device = interfaces.sta;

    // Configuración, escaneo y conexión a la red WiFi.
    configure_wifi(&mut wifi_controller);
    scan_wifi(&mut wifi_controller);
    connect_wifi(&mut wifi_controller);

    // Generación de semilla aleatoria para stack de red
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    // Configuración de red IPv4 por DHCP
    let config = embassy_net::Config::dhcpv4(Default::default());

    // Recursos del stack TCP/IP
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    // Inicialización del stack de red Embassy
    let (stack, runner) =
        embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    // Lanzamiento de la tarea de red (obligatoria)
    spawner.spawn(net_task(runner)).unwrap();

    // Espera hasta que exista enlace físico de red y se obtenga una IP por DHCP
    esp_println::println!("[MQTT] Esperando enlace WiFi...");
    while !stack.is_link_up() {
        Timer::after_millis(500).await;
    }
    esp_println::println!("[MQTT] Esperando DHCP...");
    while stack.config_v4().is_none() {
        Timer::after_millis(500).await;
    }
    esp_println::println!("[MQTT] IP obtenida: {:?}", stack.config_v4());

    // Buffers TCP para socket MQTT
    let rx_buffer = RX_BUFFER.init([0; 4096]);
    let tx_buffer = TX_BUFFER.init([0; 4096]);

    // Creación del socket TCP
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    // Conexión TCP al broker MQTT
    esp_println::println!(
        "[MQTT] Conectando al broker MQTT en {}:{}...",
        IP_ADDRESS,
        PORT
    );
    match socket.connect((IP_ADDRESS, PORT)).await {
        Ok(_) => {
            esp_println::println!("[MQTT] Conexión TCP al broker MQTT establecida.");
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error al conectar TCP al broker MQTT: {:?}", e);
        }
    }

    // El socket TCP conectado se usa como transporte del cliente MQTT.
    let transport = socket;
    // Buffer interno del cliente MQTT
    let mqtt_memory = MQTT_MEMORY.init([0; 256]);
    let buffer = MQTT_BUMP.init(BumpBuffer::new(mqtt_memory));
    let mut client = Client::<'_, TcpSocket<'_>, _, 5, 3, 1, 16>::new(buffer);
    // Opciones de conexión MQTT
    let connect_options = ConnectOptions::new()
        .clean_start()
        .session_expiry_interval(SessionExpiryInterval::NeverEnd);
    // Conexión al broker MQTT
    esp_println::println!("[MQTT] Conectando al broker MQTT...");
    match client
        .connect(
            transport,
            &connect_options,
            Some(MqttString::from_str("rust-mqtt-demo").unwrap()),
        )
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Conectado al broker MQTT!");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    };

    // Definición del topic a suscribirse
    let topic_stop = TopicName::new(MqttString::from_str("/TFM/Stop").unwrap()).unwrap();
    let topic_target = TopicName::new(MqttString::from_str("/TFM/Target").unwrap()).unwrap();
    let topic_target_yaw =
        TopicName::new(MqttString::from_str("/TFM/Target/Yaw").unwrap()).unwrap();
    let topic_pwm = TopicName::new(MqttString::from_str("/TFM/PWM").unwrap()).unwrap();
    let topic_kp_kd = TopicName::new(MqttString::from_str("/TFM/Kp_Kd").unwrap()).unwrap();
    let topic_sensor = rust_mqtt::client::options::TopicReference::Name(
        TopicName::new(MqttString::from_str("/TFM/Sensor").unwrap()).unwrap(),
    );

    match client
        .publish(
            &PublicationOptions::new(topic_sensor.as_borrowed().into()),
            "Sensor Calibrado".into(),
        )
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Mensaje publicado");
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error publish: {:?}", e);
        }
    }
    // Suscripción al topic MQTT
    match client
        .subscribe(topic_stop.as_borrowed().into(), SubscriptionOptions::new())
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tpócio '/TFM/Stop' correcta");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    }
    match client
        .subscribe(
            topic_target.as_borrowed().into(),
            SubscriptionOptions::new(),
        )
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tpócio '/TFM/Target' correcta");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    }
    match client
        .subscribe(
            topic_target_yaw.as_borrowed().into(),
            SubscriptionOptions::new(),
        )
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tópico '/TFM/Target/Yaw' correcta");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    }
    match client
        .subscribe(topic_pwm.as_borrowed().into(), SubscriptionOptions::new())
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tópico '/TFM/PWM' correcta");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    }
    match client
        .subscribe(topic_kp_kd.as_borrowed().into(), SubscriptionOptions::new())
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tópico '/TFM/Kp_Kd' correcta");
        }

        Err(e) => {
            esp_println::println!("[MQTT] Error MQTT: {:?}", e);
        }
    }
    esp_println::println!("[MQTT] Esperando SUBACK...");

    // Espera confirmación de suscripción del broker
    loop {
        match client.poll().await {
            Ok(Event::Suback(Suback {
                packet_identifier: _,
                reason_code,
            })) => {
                esp_println::println!("[MQTT] SUBSCRITO! Reason code: {:?}", reason_code);
                break;
            }

            Ok(other) => {
                esp_println::println!("[MQTT] Evento: {:?}", other);
            }

            Err(e) => {
                esp_println::println!("[MQTT] ERROR: {:?}", e);
            }
        }
    }

    // Lanza la tarea MQTT.
    spawner.spawn(mqtt_task(client)).unwrap();
    // Lanza la tarea de control del motor BL4825O.
    spawner.spawn(bl4825O_task(MCPWM0, GPIO20, GPIO21)).unwrap();

    let h = 0.01;
    let controller = pd_rw_imu::common_logic(h);
    //let total_time = 100.0;
    let total_time = f64::INFINITY;
    let mut simulator = Simulator::new(controller);
    let config = Config::new(0.0, total_time, h, None);
    //let input_handler = xdevs::simulator::embassy::SleepAsync::new();
    let input_handler = InputHandler::new();

    esp_println::println!("Starting simulation...");
    // Ejecución de la simulación en tiempo real.
    // `input_handler` introduce entradas externas y `propagate_output` distribuye salidas.
    simulator
        .simulate_rt_async(&config, input_handler, propagate_output)
        .await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
