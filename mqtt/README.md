# 📡 MQTT Client en ESP32C6 (Rust `no_std` + Embassy + rust-mqtt)
Este proyecto implementa un **cliente MQTT embebido sobre ESP32C6** utilizando Rust en modo `no_std`, basado en el runtime asíncrono **Embassy** y la librería `rust-mqtt`.

El sistema establece conectividad WiFi, levanta un stack de red IPv4 y se conecta a un broker MQTT para la recepción de mensajes en tiempo real.

## 🚀 Objetivo del proyecto
Implementar un sistema embebido capaz de:

* Inicializar hardware en ESP32C6.
* Conectarse a una red WiFi en modo estación (STA).
* Levantar un stack TCP/IP con DHCP.
* Establecer conexión TCP con un broker MQTT.
* Suscribirse a tópicos MQTT.
* Recibir y procesar mensajes en tiempo real.

## 🛠️ Tecnologías
* Rust no_std. (Bare-metal)
* Embassy y esp-rtos. (runtime async embebido)
* esp-hal. (Capa de abstracción de hardware para ESP32)
* esp-radio. (Conectividad WiFi)
* esp-println. (Depuración)
* esp-backtrace. (Gestión de pánicos)
* embassy-net. (Stack TCP/IP)
* rust-mqtt. (Cliente MQTT)
* StaticCell. (Heap allocator embebido)

## 🧠 Arquitectura del sistema
```text
WiFi → IPv4 (DHCP) → TCP Socket → MQTT Client → Tópicos → Eventos
```

## 📡 Flujo del sistema
### 1. Inicialización del sistema
Se realiza la configuración inicial del entorno embebido, que incluye:
* Configuración del heap dinámico.
* Inicialización de los temporizadores del sistema.
* Inicialización del subsistema de radio WiFi/BLE.
* Arranque del runtime asíncrono de Embassy.

### 2. Conexión WiFi
El dispositivo se conecta a una red WiFi en modo estación (STA):
* Configuración de SSID y contraseña.
* Selección del modo estación.
* Escaneo de redes disponibles.
* Establecimiento de conexión a la red seleccionada.
* Espera de enlace de red y asignación de dirección IP mediante DHCP.

### 3. Stack de red
Una vez establecida la conexión WiFi:
* Se configura la pila IPv4 mediante DHCP.
* Se crea el stack de red basado en `embassy-net`.
* Se inicializa el socket TCP para comunicación de red.

### 4. Conexión MQTT
Se establece la comunicación con el broker MQTT:
* Apertura de conexión TCP con el broker.
* Inicialización del cliente MQTT.
* Configuración de la sesión (`clean_start`).
* Establecimiento de la conexión MQTT.

## 📬 Tópicos de MQTT
Estos tópicos forman parte de la interfaz de comunicación y son utilizados como mecanismo de intercambio de datos con otro proyecto del sistema.

El sistema se suscribe a los siguientes tópicos:

* `/TFM/Stop`: controla la lectura del sensor BNO055.
  * `0` → detiene la lectura.
  * `1` → reanuda la lectura.

* `/TFM/Target`: define la orientación deseada.
* `/TFM/Target/Yaw`: define la orientación deseada en el eje de Yaw (ángulo de Euler).

## 🔁 Bucle principal
El sistema ejecuta un bucle continuo de ejecución:
* Escucha eventos entrantes del broker MQTT.
* Procesa mensajes de tipo `Publish`.
* Decodifica el payload en formato UTF-8.
* Muestra el topic y el contenido del mensaje recibido.
* Mantiene la conexión activa mediante polling periódico.

## 📁 Estructura del proyecto
```
src/
└── main.rs                     # Lógica principal del cliente MQTT en ESP32
```

## 📌 Referencias
* [Framework de Embassy](https://embassy.dev/).
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [rust-mqtt](https://crates.io/crates/rust-mqtt).
* [STA](https://esp32.implrust.com/wifi/sta-mode-access-website.html).
