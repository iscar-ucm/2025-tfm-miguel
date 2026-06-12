# 🛰️ Integración de Digital Twins en un sistema ADCS en Rust `no-std` (basado en DEVS)
Este proyecto implementa una **librería Rust `no_std`** para modelar un sistema de Control de Actitud para nanosatélites (ADCS) mediante el formalismo **DEVS (Discrete Event System Specification)**. La librería integra conceptos de **Digital Twins (DT)** para modelado, monitorización y validación de sistemas espaciales.

## 🌐 Digital Twins
Un **Digital Twin (DT)** en DEVS es una representación digital ejecutable de un sistema físico real construida mediante modelos DEVS.

A diferencia de una simulación tradicional (offline), un DT mantiene una **relación continua con el sistema físico** mediante el intercambio de telemetría, eventos y señales de control.

---
### 📥 Entradas y salidas del sistema

* $X$: conjunto de eventos de entrada al DT.
* $Y$: conjunto de eventos de salida del DT.

### 📦 Componentes del modelo DT
El sistema acoplado se define como:

```math
C = \lbrace DT,PT,DPC,CCU\rbrace
```
donde:
* **DT (Digital Twin)**: modelo DEVS que simula el comportamiento del sistema físico.
* **PT (Physical Twin)**: sistema físico real.
* **DPC (Digital-to-Physical Converter)**: traduce eventos del dominio DEVS a señales físicas.
* **CCU (Control and Calibration Unit)**: procesa la retroalimentación proveniente tanto del **Digital Twin (DT)** como del **Physical Twin (PT)** y genera la salida final del sistema $Y$.

![DT en DEVS](./docs/ibd%20DT.png)

## 🚀 Objetivo del proyecto
Simular el comportamiento de un sistema ADCS realista compuesto por:

* Controlador de actitud (PD con cuaterniones).
* Ruedas de reacción modeladas en software.
* Dinámica del satélite (cuerpo rígido).
* DPC.
* CCU.

## 🛠️ Tecnologías
* Rust no-std.
* Cargo. (Gestión de dependencias y build)
* xdevs-no-std. (Simulación DEVS no-std)
* nalgebra. (Matemáticas)

## 🧩 Arquitectura del sistema
El sistema está compuesto por los siguientes módulos:

### 🧠 Controlador
* Control PD basado en cuaterniones.
* Genera torque de corrección.
* Calcula error de actitud.

### ⚙️ Reaction Wheels (RW)
* Convierte torque en velocidad angular.
* Modela dinámica rotacional de actuadores.
* Aplica saturación física.

### 🛰️ IMU_SW
* Modelo de cuerpo rígido 3D.
* Dinámica rotacional completa.
* Cinemática de cuaterniones.

### 📡 DT (Digital Twin Architecture)
En este sistema, la interacción entre componentes se define de la siguiente forma:

* CCU (Control and Calibration Unit):
  * Recibe datos del `IMU_SW` (sensor software) o del sensor físico real (IMU hardware).
  * Envía los datos, en función del modo de funcionamiento, al Controlador.

* DPC (Digital-to-Physical Converter):
  * Recibe el torque de control generado por el Controlador.
  * Emite las señales correspondientes hacia el motor / actuadores (reaction wheels).
  * Actúa como interfaz directa entre el dominio digital y el sistema físico.

## 🔁 Flujo de datos
![IBD PD-RW](./docs/ibd%20pd-rw.png)

## 📁 Estructura del proyecto
```
src/
├── main.rs                         # Punto de entrada de la simulación ADCS
├── controller.rs                   # Controlador PD de actitud
├── rw.rs                           # Dinámica de ruedas de reacción
├── imu_sw.rs                       # Representación software de actitud y velocidad angular
├── types.rs                        # Tipos matemáticos (Vec3, Quaternion)
├── ccu.rs                          # Control and Calibration Unit
├── dpc.rs                          # Digital-to-Physical Converter
│
docs/
├── ibd DT.png                      # Conexiones internas del DT (IBD)
└── ibd pd-rw.png                   # Conexiones internas del sistema (IBD)
```

## 📌 Referencias
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [xDevs-no-std](https://crates.io/crates/xdevs-no-std).
* [DT en DEVS](https://www.sciencedirect.com/science/article/pii/S0950584925003131).
* [libm](https://crates.io/crates/libm).
