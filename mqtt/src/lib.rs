#![no_std]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{interrupt::software::SoftwareInterruptControl, rng::Rng, timer::timg::TimerGroup};
use esp_radio::wifi::{ClientConfig, ModeConfig, PowerSaveMode, ScanConfig, WifiController};
use rust_mqtt::{
    buffer::BumpBuffer,
    client::{
        Client,
        event::{Event, Suback},
        options::{ConnectOptions, SubscriptionOptions},
    },
    config::SessionExpiryInterval,
    types::{MqttString, TopicName},
};
// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

use embassy_net::{Config, Ipv4Address, StackResources, tcp::TcpSocket};
use static_cell::StaticCell;

#[cfg(feature = "home")]
const SSID: &str = "DIGIFIBRA-DSC3";

#[cfg(feature = "home")]
const PASSWORD: &str = "aKE4hsZpzs";

#[cfg(feature = "mobile")]
const SSID: &str = "Xiaomi 15 Pro";

#[cfg(feature = "mobile")]
const PASSWORD: &str = "6ws5hkph58t8cct";

static RADIO_INIT: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 1, 213);
const PORT: u16 = 1884;

/// Tarea de red Embassy.
///
/// Ejecuta el `Runner` del stack TCP/IP de forma continua.
/// Esta tarea es obligatoria para que la pila de red funcione.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, esp_radio::wifi::WifiDevice<'static>>) {
    runner.run().await
}


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


pub async fn init_mqtt_client(spawner: Spawner) {
// Inicialización de logs por UART
    esp_println::logger::init_logger_from_env();
    // Inicialización del heap global (memoria dinámica)
    esp_alloc::heap_allocator!(size: 72 * 1024);
    // Estadísticas de memoria (debug)
    let stats: esp_alloc::HeapStats = esp_alloc::HEAP.stats();
    esp_println::println!("{}", stats);

    // Inicialización de periféricos del chip (timers, radio, etc.)
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Inicialización del controlador de radio WiFi/BLE
    let radio_init = RADIO_INIT
        .init(esp_radio::init().expect("[MQTT] Fallo al inicializar el controlador Wi-Fi/BLE"));

    // Creación del controlador WiFi y sus interfaces (STA/AP)
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("[MQTT] Fallo al crear el controlador Wi-Fi");
    // Interface en modo estación (cliente WiFi)
    let device = interfaces.sta;

    configure_wifi(&mut wifi_controller);
    scan_wifi(&mut wifi_controller);
    connect_wifi(&mut wifi_controller);

    // Generación de semilla aleatoria para stack de red
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    // Configuración de red IPv4 por DHCP
    let config = Config::dhcpv4(Default::default());

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
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // Creación del socket TCP
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    // Conexión TCP al broker MQTT (puerto 1884)
    esp_println::println!("[MQTT] Conectando al broker MQTT en {}:{}...", IP_ADDRESS, PORT);
    match socket.connect((IP_ADDRESS, PORT)).await {
        Ok(_) => {
            esp_println::println!("[MQTT] Conexión TCP al broker MQTT establecida.");
        }
        Err(e) => {
            esp_println::println!("[MQTT] Error al conectar TCP al broker MQTT: {:?}", e);
        }
    }

    let transport = socket;
    // Buffer interno del cliente MQTT
    let mut mqtt_buffer: [u8; 2048] = [0; 2048];
    let mut buffer = BumpBuffer::new(&mut mqtt_buffer);
    let mut client = Client::<'_, TcpSocket<'_>, _, 2, 2, 1, 16>::new(&mut buffer);
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
    // Suscripción al topic MQTT
    match client
        .subscribe(topic_stop.as_borrowed().into(), SubscriptionOptions::new())
        .await
    {
        Ok(_) => {
            esp_println::println!("[MQTT] Suscripción al tpócio /TFM/Stop correcta");
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
            esp_println::println!("[MQTT] Suscripción al tpócio /TFM/Target correcta");
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
}