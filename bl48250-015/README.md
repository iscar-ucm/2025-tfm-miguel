# ⚙️ Control de motor con MCPWM en ESP32 + Rust (esp-hal + esp-rtos)
Este proyecto es una **prueba de control de motor DC utilizando el periférico MCPWM del ESP32**, programado en Rust en entorno `no_std` con `esp-hal`.

El sistema genera una señal PWM avanzada mediante MCPWM (Motor Control PWM), variando dinámicamente el duty cycle para observar el comportamiento real del motor en términos de velocidad, corriente y respuesta mecánica.

A diferencia de un PWM básico, MCPWM permite un control más preciso al separar la gestión del tiempo (timer) de la generación de la señal (operator), lo que lo hace especialmente adecuado para aplicaciones de control de motores.

El sistema implementa una rampa de duty cycle que sube y baja continuamente para analizar la respuesta del motor bajo distintas condiciones de potencia.

## ⚠️ Nota importante sobre la relación PWM y velocidad
En este sistema, es importante destacar que la relación entre el duty cycle y la velocidad del motor **no sigue el comportamiento típico ideal**.

En un modelo teórico estándar, se espera que:
* 0% duty → motor parado.
* 100% duty → velocidad máxima.

Sin embargo, en las mediciones reales de este sistema se observa un comportamiento **invertido y no lineal**, donde a mayor duty cycle la velocidad disminuye.

### 📊 Medidas (24V DC)
* 0% → 3980 RPM | 0.16 A.
* 20% → 3750 RPM | 0.15 A.
* 40% → 3000 RPM | 0.12 A.
* 60% → 2380 RPM | 0.11 A.
* 80% → 1680 RPM | 0.07 A.
* 99% → 770 RPM | 0.04 A.

## 📚 MCPWM
Es un periférico avanzado del ESP32 diseñado para generar señales PWM de alta precisión orientadas al control de motores y sistemas de potencia.

A diferencia de un PWM básico, MCPWM separa claramente el **tiempo de conteo** de la **generación de la señal**, lo que permite un control mucho más fino y flexible.

Su arquitectura se basa en tres bloques fundamentales: **prescaler, timer y operator**.

### ⚙️ Prescaler
El prescaler se encarga de reducir la frecuencia del reloj base del sistema antes de ser utilizado por el MCPWM. En el ESP32, el reloj principal que alimenta este periférico es de **160 MHz**, lo que significa que realiza 160 millones de ciclos por segundo.

Cada ciclo tiene una duración de:

```text
Periodo = 1 / 160 MHz = 6.25 ns
```

Esto implica que cada tick del MCPWM dura 6.25 nanosegundos.

El prescaler modifica este reloj dividiéndolo mediante la fórmula:

```text
PWM Clock Period = 6.25 ns × (Prescaler + 1)
```

Por ejemplo, si el prescaler es 159:

```text
PWM Clock Period = 6.25 ns × 160 = 1000 ns = 1 µs
```

La frecuencia resultante sería:

```text
PWM Frequency = 1 / 1000 ns = 1 MHz
```

En el valor máximo del prescaler (255):

```text
PWM Clock Period = 6.25 ns × 256 = 1600 ns
PWM Frequency ≈ 625 kHz
```

### ⏱️ Timer
El timer es el encargado de contar hasta un valor definido (periodo) y reiniciarse, determinando así la frecuencia final del PWM.

Dispone de un contador de 16 bits y puede operar en tres modos:

- **Increase**: empieza en 0 y cuenta hacia arriba hasta llegar al valor del periodo.
- **Decrease**: empieza en el valor del periodo y cuenta hacia abajo hasta 0.
- **UpDown**: sube y baja de forma alterna, creando un ciclo simétrico.

### 🔧 Operator
El operator es el bloque que genera la señal PWM final. Utiliza la información del timer para decidir cuándo la salida está en alto o en bajo.

Cada operator tiene dos salidas:
- PWMxA.
- PWMxB.

### 📡 set_timestamp()
La función `set_timestamp()` define el punto del ciclo en el que la señal cambia de estado.

Ejemplo:

```rust
pwm_pin.set_timestamp(500);
```

Si el periodo total es 20,000:
* HIGH de 0 a 500.
* LOW de 500 a 20,000.

Esto equivale aproximadamente a un duty cycle del 2.5%.

## 🛠️ Tecnologías
* Rust no-std. (Bare-metal)
* Cargo. (Gestión de dependencias y build)
* Embassy y esp-rtos. (Ejecución asíncrona en no-std)
* esp-hal. (Capa de abstracción de hardware para ESP32)
* esp-println. (Depuración)
* esp-backtrace. (Gestión de pánicos)

## ⚙️ Arquitectura del sistema
El sistema está compuesto por:

### 🔹 Inicialización del runtime
* Configuración del esp-rtos.
* Inicialización de timers y software interrupts.

### 🔹 MCPWM
* Configuración del módulo PWM del ESP32.
* Generación de señal en GPIO9.
* Frecuencia configurada en ~20 kHz.

### 🔹 GPIO
* GPIO15 configurado como salida del sentido de giro del motor.

## 🔁 Comportamiento del sistema
El programa genera un valor PWM (`val`) que:

* Sube progresivamente de 0 → 100.
* Baja progresivamente de 100 → 0.
* Repite el ciclo indefinidamente.

Esto produce una señal tipo:

```
0 → 10 → 20 → ... → 100 → 90 → 80 → ... → 0
```

## 📊 Salida esperada
```
PWM: 0
PWM: 10
PWM: 20
...
PWM: 100
PWM: 90
PWM: 80
...
```
## 🔌 Hardware utilizado

| Señal | Pin |
|---|---|
| PWM (MCPWM) | GPIO9 |
| Sentido | GPIO15 |

## 🚀 Ejecución
### 1. Clonar el repositorio
```bash
git clone https://github.com/iscar-ucm/2025-tfm-miguel.git
cd 2025-tfm-miguel/bl48250-015
```

### 2. Ejecutar el proyecto
```bash
cargo run
```

## 📁 Estructura del proyecto
```text
src/
 └── main.rs
```

## 📌 Referencias
* [MCPWM](https://esp32.implrust.com/core-concepts/pwm/mcpwm.html).
* [Ecosistema Rust de Espressif](https://docs.espressif.com/projects/rust/).
* [Framework de Embassy](https://embassy.dev/).
* [Rust no-std](https://docs.rust-embedded.org/book/intro/no-std.html).
* [BL4825O ficha técnica](https://www.makerforge.tech/posts/bl4825o-introduction/).