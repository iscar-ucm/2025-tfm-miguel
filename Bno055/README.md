# 🧭 Prueba inicial del sensor BNO055 en Rust no-std
Este proyecto es una **prueba sencilla del sensor IMU [BNO055](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)** utilizando Rust no-std sobre un microcontrolador ESP32C6.

El objetivo principal es validar:

* La comunicación I2C.
* La inicialización del sensor.
* La calibración automática.
* La lectura de orientación mediante cuaterniones.
* La lectura de velocidad angular del giroscopio.

El sistema realiza lecturas periódicas del sensor y muestra los datos por consola.

## 🛠️ Tecnologías
* Rust no-std. (Bare-metal)
* Cargo. (Gestión de dependencias y build)
* Embassy y esp-rtos. (Ejecución asíncrona en no-std)
* esp-hal. (Capa de abstracción de hardware para ESP32)
* esp-println. (Depuración)
* esp-backtrace. (Gestión de pánicos)
* bno055.

## ⚙️ Arquitectura del sistema
El sistema está compuesto por:

### 🔹 Inicialización I2C
* Configura el bus I2C del microcontrolador.
* Define los pines SDA y SCL.

### 🔹 Sensor BNO055
* Inicializa la IMU.
* Configura el modo `NDOF`.
* Gestiona la calibración automática.

### 🔹 Lectura de datos
Obtiene periódicamente:

* Cuaterniones.
* Velocidad angular.

## 🧠 Funcionamiento del sistema
### 🔹 Configuración del sensor
El sensor se configura en modo:

```rust
BNO055OperationMode::NDOF
```

Este modo habilita la fusión completa de sensores:

* Acelerómetro
* Giroscopio
* Magnetómetro

permitiendo obtener orientación absoluta en 3D.

### 🔹 Calibración
Antes de comenzar las lecturas, el sistema espera a que la IMU esté completamente calibrada.

```rust
while !imu.is_fully_calibrated().unwrap()
```

Durante este proceso se muestran los estados de calibración por consola.

### 🔹 Lectura de orientación
Se leen los cuaterniones del sensor:

```rust
imu.quaternion()
```

### 🔹 Lectura del giroscopio
Se obtiene la velocidad angular en los ejes:

* X
* Y
* Z

mediante:

```rust
imu.gyro_data()
```

## 🚀 Ejecución
### 1. Clonar el repositorio
```bash
git clone https://github.com/iscar-ucm/2025-tfm-miguel.git
cd 2025-tfm-miguel/Bno055
```

### 2. Ejecutar el proyecto
```bash
cargo run
```

## 🧪 Configuración hardware
### Pines I2C utilizados

| Señal | GPIO |
|---|---|
| SDA | GPIO6 |
| SCL | GPIO7 |

## 📊 Salida esperada
Durante la ejecución se mostrarán mensajes similares a:

```text
I2C initialized.
The IMU's calibration status is: BNO055CalibrationStatus { sys: 0, gyr: 0, acc: 0, mag: 0 }
- About to begin BNO055 IMU calibration...
Calibration status: BNO055CalibrationStatus { sys: 0, gyr: 0, acc: 0, mag: 0 }
...
       - Calibration complete!
IMU Quaternion: Quaternion { v: Vector3 { x: 0.05126953, y: -0.772583, z: -0.36889648 }, s: 0.5142212 }
IMU Gyro Data: Vector3 { x: 152.625, y: 220.8125, z: -87.25 }
...
```

## 📁 Estructura del proyecto
```
src/
 └── main.rs
```

## 📌 Referencias
* [BNO055 ficha técnica](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
* [Crate BNO055](https://crates.io/crates/bno055).
* [Ecosistema Rust de Espressif](https://docs.espressif.com/projects/rust/).
* [Framework de Embassy](https://embassy.dev/).
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [Tutorial de Rust no-std para ESP32](https://esp32.implrust.com/index.html).