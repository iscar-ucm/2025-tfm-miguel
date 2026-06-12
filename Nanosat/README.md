# 🛰️ Nanosat: despliegue embebido de ADCS/DT en ESP32-C6
Este proyecto implementa la aplicación embebida encargada de ejecutar sobre una placa ESP32-C6 la lógica de control y simulación definida en la librería `PD-RW-IMU`. Mientras que `PD-RW-IMU` contiene los modelos DEVS del sistema ADCS, `Nanosat` se encarga de integrarlos con hardware real, ejecución en tiempo real, comunicación WiFi/MQTT y monitorización externa.

El objetivo principal es trasladar la arquitectura validada en software hacia un entorno embebido real, incorporando sensores, actuadores y procesos externos. 

## 🛠️ Tecnologías utilizadas
* Rust no-std. (Bare-metal)
* Cargo. (Gestión de dependencias y build)
* xdevs-no-std. (Simulación DEVS no-std)
* pd_rw_imu. (Modelo PD-RW)
* Embassy y esp-rtos. (Ejecución asíncrona en no-std)
* esp-hal. (Capa de abstracción de hardware para ESP32)
* esp-radio. (Conectividad WiFi)
* esp-println. (Depuración)
* esp-backtrace. (Gestión de pánicos)
* embassy-net. (Stack TCP/IP)
* rust-mqtt. (Cliente MQTT)
* StaticCell. (Heap allocator embebido)
* nalgebra y libm. (Operaciones matemáticas)
* bno055.

## 🧩 Componentes principales
### 1. Modelo PD-RW
El modelo se obtiene desde la librería local:

```toml
pd_rw_imu = { path = "../PD-RW-IMU" }
```

La función `pd_rw_imu::common_logic(h)` construye el modelo `PD-RW`, que incluye:
* `Controller`: controlador PD basado en cuaterniones.
* `RW`: modelo software de ruedas de reacción.
* `IMUSW`: representación software de actitud y velocidad angular.
* `CCU`: selecciona la realimentación procedente del modelo software o del sensor físico.
* `DPC`: convierte la acción de control digital en una señal PWM.

### 2. Sensor BNO055
La tarea `bno055_task` lee periódicamente:
* Cuaternión de actitud;
* Velocidad angular del giroscopio.

Cada muestra se transforma en un `ImuSample` y se envía al canal `IN_CHANNEL` como entrada del modelo. También se publica información asociada por MQTT para monitorización.

### 3. Input handler
`InputHandler` implementa `AsyncInput` de `xdevs-no-std`. Su función es esperar eventos externos hasta el siguiente instante de simulación previsto e introducirlos en los puertos del modelo:
* `ModelInput::Imu` -> `i_q`, `i_w`.
* `ModelInput::QTarget` -> `i_q_target`.
* `ModelInput::KpKd` -> `i_kp`, `i_kd`.
* `ModelInput::PWM` -> `i_pwm`.

Este mecanismo permite que datos físicos o comandos externos entren en el modelo DEVS durante la ejecución en tiempo real.

### 4. Output handler
La función `propagate_output` captura salidas generadas por el modelo:
* `o_q_error`;
* `o_torque`;
* `o_pwm`.

Estas salidas se propagan hacia dos destinos:

* `MQTT_IN_CHANNEL`, para publicar telemetría por MQTT.
* `OUT_CHANNEL`, para enviar valores de PWM a la tarea del actuador.

### 5. Actuador BL4825O
La tarea `bl4825O_task` recibe valores desde `OUT_CHANNEL` y actualiza la señal PWM generada mediante el periférico `MCPWM0`. El valor de PWM se aplica sobre el eje utilizado por la implementación y se publica también como telemetría.

### 6. Comunicación MQTT
El sistema se conecta a una red WiFi, obtiene configuración IP mediante DHCP y crea un socket TCP para conectarse al broker MQTT.

Se utiliza Mosquitto como broker MQTT y como punto central para enviar comandos. Node-RED se utiliza para monitorizar la telemetría publicada por la placa.

## 📬 Tópicos MQTT
La placa publica información de monitorización en:
| Tópico | Contenido |
| --- | --- |
| `/TFM/Q` | Actitud medida/estimada, expresada principalmente como yaw para monitorización |
| `/TFM/Q_error` | Error de actitud calculado por el controlador |
| `/TFM/Torque` | Torque de control generado por el modelo |
| `/TFM/PWM/Monitoring` | Valor de PWM aplicado/monitorizado |
| `/TFM/Sensor` | Estado inicial del sensor, por ejemplo calibración |

La placa recibe comandos en:
| Tópico | Formato esperado | Función |
| --- | --- | --- |
| `/TFM/Stop` | `1` o `0` | Detiene o reanuda la lectura del BNO055 |
| `/TFM/Target` | `qx,qy,qz,qw` | Recibe un cuaternión objetivo |
| `/TFM/Target/Yaw` | número en grados | Genera un cuaternión objetivo a partir de yaw |
| `/TFM/PWM` | número | Introduce un valor PWM externo |
| `/TFM/Kp_Kd` | `kp,kd` | Actualiza las ganancias proporcional y derivativa |

## ⚙️ Configuración previa
Antes de ejecutar el proyecto deben ajustarse en `src/bin/main.rs`:

```rust
const SSID: &str = "...";
const PASSWORD: &str = "...";
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(...);
const PORT: u16 = 1883;
```

`SSID` y `PASSWORD` definen la red WiFi. `IP_ADDRESS` y `PORT` indican la dirección y puerto del broker Mosquitto.

### Pines I2C utilizados
| Señal | GPIO |
|---|---|
| SDA | GPIO18 |
| SCL | GPIO19 |

### Actuador 1
| Señal | GPIO |
|---|---|
| PWM | GPIO20 |
| Sentido | GPIO21 |


## 🚀 Ejecución
### 1. Clonar el repositorio
```bash
git clone https://github.com/iscar-ucm/2025-tfm-miguel.git
cd 2025-tfm-miguel/Nanosat
```

### 2. Ejecutar el proyecto
```bash
cargo run
```

## 📁 Estructura del proyecto
```text
src/
 └── main.rs            # Aplicación embebida principal
```

## 📌 Referencias
* [Tutorial de Rust no-std para ESP32](https://esp32.implrust.com/index.html).
* [xdevs-no-std](https://crates.io/crates/xdevs-no-std).
* [Ecosistema Rust de Espressif](https://docs.espressif.com/projects/rust/).
* [Framework de Embassy](https://embassy.dev/).
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [rust-mqtt](https://crates.io/crates/rust-mqtt).
* [STA](https://esp32.implrust.com/wifi/sta-mode-access-website.html).
* [BL4825O ficha técnica](https://www.makerforge.tech/posts/bl4825o-introduction/).
* [Crate BNO055](https://crates.io/crates/bno055).
* [Mosquitto](https://mosquitto.org/).
* [MCPWM](https://esp32.implrust.com/core-concepts/pwm/mcpwm.html).
* [Node-RED](https://nodered.org/).
