# 🛰️ 2025-tfm-miguel
Repositorio de desarrollo del Trabajo Fin de Máster de Yuejie Xu. El proyecto estudia la integración progresiva de modelos DEVS, ejecución embebida en Rust `no_std`, sensores y actuadores físicos, y comunicación MQTT para construir una arquitectura de control de actitud de nanosatélites con conceptos de Digital Twin.

El trabajo se organiza como una evolución por capas:

```text
SIL -> HIL -> DT -> pruebas y despliegue
```

Cada subproyecto del repositorio representa una parte de esa evolución: desde una simulación software del sistema ADCS hasta una aplicación embebida ejecutada sobre ESP32-C6 con sensor BNO055, actuador BL4825O, broker Mosquitto y monitorización mediante Node-RED.

## 🎯 Objetivo general
El objetivo del repositorio es desarrollar y validar una arquitectura para un sistema ADCS de nanosatélite basada en:

* Modelado y simulación mediante DEVS;
* Control de actitud con controlador PD y cuaterniones;
* Ruedas de reacción como actuadores;
* Migración progresiva de Rust `std` a Rust `no_std`;
* Ejecución embebida sobre ESP32-C6;
* Integración con sensor BNO055 y actuador BL4825O;
* Comunicación MQTT para telemetría, comandos y monitorización externa;
* Organización del sistema en capas SIL, HIL, DT y despliegue.

## 🔩 Hardware
El sistema final se orienta a:
* ESP32-C6.
* BNO055 como sensor IMU.
* BL4825O como actuador.
* Conexión WiFi hacia un broker Mosquitto.
* Equipo externo con Node-RED para monitorización.

## 🛠️ Tecnologías
* Rust.
* Rust no_std.
* Cargo.
* xdevs y xdevs-no-std.
* esp-hal.
* esp-rtos.
* Embassy.
* embassy-net.
* esp-radio.
* rust-mqtt.
* esp-println.
* esp-backtrace.
* nalgebra y libm.
* BNO055.

## 📚 Orden de lectura
El repositorio contiene tanto prototipos aislados como integraciones completas. La lectura recomendada es:

1. [`PD-RW`](./PD-RW/): entender el modelo ADCS en simulación software.
2. [`PD-RW-no-std`](./PD-RW-no-std/): ver la migración hacia `no_std`.
3. [`PD-RW-IMU`](./PD-RW-IMU/): entender la librería reutilizable ADCS/DT.
4. [`Bno055`](./Bno055/), [`bl48250-015`](./bl48250-015/) y [`mqtt`](./mqtt/): revisar validaciones aisladas de hardware/comunicación.
5. [`Nanosat`](./Nanosat/): estudiar la integración final sobre ESP32-C6.

## 🔗 Relación entre subproyectos
### 🧪 `PD-RW`
Primer modelo funcional del sistema ADCS. Implementa una simulación en Rust con `std` usando `xdevs`, `nalgebra` y `plotters`.

Incluye:
* Controlador PD basado en cuaterniones;
* Modelo de ruedas de reacción;
* Dinámica rotacional del satélite;
* Transductor de telemetría;
* Generación de gráficas de resultados.

Este proyecto corresponde principalmente a la capa SIL inicial.

### 🌱 `PD-RW-no-std`
Versión migrada del modelo ADCS hacia Rust `no_std`. Mantiene la lógica de control y simulación, pero adopta una arquitectura más adecuada para sistemas embebidos.

### 📦 `PD-RW-IMU`
Librería Rust `no_std` que contiene la lógica principal del modelo ADCS/DT reutilizable por otros proyectos.

Define el modelo acoplado formado por:
* `Controller`: controlador PD de actitud;
* `RW`: modelo software de ruedas de reacción;
* `IMUSW`: representación software de actitud y velocidad angular;
* `CCU`: `Control and Calibration Unit`;
* `DPC`: `Digital-to-Physical Converter`.

Esta librería es utilizada por `Nanosat`, que se encarga de desplegarla sobre hardware real.

### 🚀 `Nanosat`
Aplicación embebida principal del TFM. Ejecuta el modelo de `PD-RW-IMU` sobre una placa ESP32-C6 e integra:

* Sensor BNO055 por I2C;
* Actuador BL4825O mediante PWM;
* Ejecución en tiempo real con `xdevs-no-std`;
* `input_handler` para introducir eventos físicos y comandos externos;
* `output_handler` para propagar salidas del modelo;
* WiFi mediante `esp-radio`;
* Pila TCP/IP con `embassy-net`;
* Comunicación MQTT mediante `rust-mqtt`;
* Mosquitto como broker;
* Node-RED como herramienta de monitorización.

Este proyecto representa las capas HIL, DT y pruebas/despliegue.

### 🧲 `Bno055`
Prueba aislada del sensor BNO055 en Rust `no_std`.

Valida:
* Comunicación I2C;
* Inicialización del sensor;
* Modo `NDOF`;
* Calibración;
* Lectura de cuaterniones;
* Lectura de velocidad angular.

Sirve como base para integrar el sensor en `Nanosat`.

### ⚙️ `bl48250-015`
Prueba aislada del actuador BL4825O mediante el periférico MCPWM del ESP32.

Valida:
* Generación de señal PWM;
* Uso de `MCPWM0`;
* Control del duty cycle;
* Respuesta del actuador ante diferentes valores de PWM.

Sirve como base para la actuación física en `Nanosat`.

### 📡 `mqtt`
Prueba aislada de comunicación MQTT en ESP32-C6.

Valida:
* Conexión WiFi;
* Obtención de IP mediante DHCP;
* Apertura de socket TCP;
* Conexión con broker MQTT;
* Suscripción a tópicos;
* Recepción de mensajes.

Sirve como base para la capa de pruebas y despliegue de `Nanosat`.

### 🧰 `GPT`
Prototipo adicional relacionado con el modelado del sistema. Puede utilizarse como referencia experimental, aunque no constituye el flujo principal del TFM.

## 🧱 Capas metodológicas
### 💻 SIL: Software-in-the-Loop
La capa SIL se desarrolla principalmente en `PD-RW`. Su objetivo es validar el modelo acoplado PD-RW en un entorno completamente software antes de introducir restricciones embebidas o interacción con hardware físico.

En esta capa el modelo funciona como una simulación cerrada formada por:
* `SatelliteDynamics`, encargado de la actitud y la velocidad angular del satélite;
* `Controller`, que calcula el error de actitud y el torque mediante una ley PD;
* `RW`, que modela en software las ruedas de reacción;
* `Transducer`, que registra las variables necesarias para el análisis posterior.

La validación se centra en comprobar el cierre del lazo de control, la coherencia dinámica de la actitud y la velocidad angular, el respeto de los límites de torque y velocidad de las ruedas, la estabilidad numérica de la simulación y la observabilidad de las variables registradas.

### 🔌 HIL: Hardware-in-the-Loop
La capa HIL traslada la lógica validada en SIL hacia una placa ESP32-C6 programada en Rust `no_std`, introduciendo restricciones propias de la ejecución embebida y de la interacción con periféricos reales.

Respecto a SIL, el modelo deja de ser completamente cerrado: el `Transducer` deja de formar parte del modelo acoplado y la observación de variables pasa a realizarse mediante salidas externas. Además, se incorporan entradas externas para introducir eventos procedentes del entorno físico o embebido.

En esta capa se comprueba:
* La compilación, despliegue y ejecución del firmware sobre la ESP32-C6.
* La sincronización en tiempo real mediante `xdevs-no-std`.
* La lectura del sensor BNO055 y su introducción en el modelo mediante el `input_handler`.
* La propagación del torque de control mediante el `output_handler`.
* La coexistencia entre tareas asíncronas, lectura de periféricos y ejecución del modelo.

### 🌐 DT: Digital Twin
La capa DT reorganiza el modelo validado en HIL para representar una arquitectura físico-digital más completa. En esta etapa, el modelo ejecutable conserva un papel central dentro del lazo de funcionamiento, recibiendo información del sistema físico, actualizando la lógica de control y generando salidas que pueden propagarse hacia el entorno.

El modelo acoplado PD-RW de esta capa incorpora:
* `Controller`, que mantiene la ley de control PD.
* `RW`, que representa en software el comportamiento de las ruedas de reacción.
* `IMUSW`, que mantiene una representación software de la actitud y la velocidad angular.
* `CCU`, que selecciona la fuente de realimentación según la modalidad de funcionamiento.
* `DPC`, que convierte el torque de control en una señal PWM compatible con la actuación física.

Respecto a HIL, se amplían las entradas y salidas externas del modelo. Además de recibir medidas del sensor BNO055, la capa DT permite modificar la actitud objetivo, ajustar las ganancias `kp` y `kd`, propagar el error de actitud y el torque para monitorización, y enviar el PWM generado hacia la tarea encargada del actuador.

### 🧪 Pruebas y despliegue
La capa de pruebas y despliegue extiende el sistema hacia un entorno distribuido básico. Su objetivo no es modificar la lógica principal del modelo, sino comprobar que la arquitectura embebida puede publicar telemetría, recibir comandos externos y ser monitorizada durante la ejecución.

En esta capa se incorpora una arquitectura de comunicación mediante MQTT:
* La ESP32-C6 actúa como cliente MQTT.
* Mosquitto funciona como broker y centraliza el intercambio de mensajes.
* Node-RED se utiliza como interfaz de monitorización.
* Los tópicos de telemetría publican variables como error de actitud, torque, PWM y medidas del sensor.
* Los tópicos de comandos permiten modificar la actitud objetivo, las ganancias del controlador, el PWM o el estado de lectura del sensor.

Esta capa permite comprobar que el modelo embebido no funciona de forma aislada, sino conectado con procesos externos de supervisión y configuración.

## ▶️ Ejecución básica
Cada subproyecto es un crate independiente. Para ejecutar uno de ellos:

```bash
cd <subproyecto>
cargo run
```

Ejemplos:

```bash
cd PD-RW
cargo run
```

```bash
cd PD-RW-no-std
cargo run
```

```bash
cd Nanosat
cargo run
```

Los proyectos embebidos requieren tener configurado el entorno Rust para ESP32-C6, la toolchain indicada en cada `rust-toolchain.toml` y el hardware conectado.

## ⚙️ Configuración de Nanosat
Antes de ejecutar `Nanosat`, deben revisarse en `Nanosat/src/bin/main.rs`:

```rust
const SSID: &str = "...";
const PASSWORD: &str = "...";
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(...);
const PORT: u16 = 1883;
```

Estos valores definen la red WiFi y la dirección del broker Mosquitto.

## 📬 Tópicos MQTT principales
`Nanosat` publica telemetría en:
* `/TFM/Q`;
* `/TFM/Q_error`;
* `/TFM/Torque`;
* `/TFM/PWM/Monitoring`;
* `/TFM/Sensor`.

`Nanosat` recibe comandos en:
* `/TFM/Stop`;
* `/TFM/Target`;
* `/TFM/Target/Yaw`;
* `/TFM/PWM`;
* `/TFM/Kp_Kd`.

## 📁 Estructura del repositorio
```text
2025-tfm-miguel/
├── PD-RW/             # Simulación ADCS inicial en Rust std
├── PD-RW-no-std/      # Migración del modelo ADCS a Rust no_std
├── PD-RW-IMU/         # Librería no_std con modelo ADCS/DT reutilizable
├── Nanosat/           # Aplicación embebida final sobre ESP32-C6
├── Bno055/            # Prueba aislada del sensor BNO055 por I2C
├── bl48250-015/       # Prueba aislada del actuador BL4825O mediante PWM
├── mqtt/              # Prueba aislada de cliente MQTT en ESP32-C6
├── GPT/               # Prototipo/experimento adicional de modelado
└── README.md          # Este documento
```

## 📌 Referencias
* [Tutorial de Rust no-std para ESP32](https://esp32.implrust.com/index.html).
* [xdevs](https://crates.io/crates/xdevs).
* [xdevs-no-std](https://crates.io/crates/xdevs-no-std).
* [Ecosistema Rust de Espressif](https://docs.espressif.com/projects/rust/).
* [Framework de Embassy](https://embassy.dev/).
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [rust-mqtt](https://crates.io/crates/rust-mqtt).
* [STA](https://esp32.implrust.com/wifi/sta-mode-access-website.html).
* [BL4825O ficha técnica](https://www.makerforge.tech/posts/bl4825o-introduction/).
* [BNO055 ficha técnica](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
* [Crate BNO055](https://crates.io/crates/bno055).
* [Mosquitto](https://mosquitto.org/).
* [MCPWM](https://esp32.implrust.com/core-concepts/pwm/mcpwm.html).
* [Node-RED](https://nodered.org/).
