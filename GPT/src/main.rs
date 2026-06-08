//! Simulación basada en DEVS usando la librería xdevs.
//!
//! Este ejemplo implementa un sistema compuesto por:
//! - Un generador de trabajos (Generator)
//! - Un procesador de trabajos (Processor)
//! - Un transductor (Transducer) que recoge estadísticas y detiene la simulación

use core::f64;

use xdevs::modeling::*;
use xdevs::simulation::*;

/// Representa un trabajo dentro del sistema.
#[derive(Clone, Debug)]
struct Job {
    /// Tiempo de creación del trabajo.
    time: f64,
    /// Nombre o descripción del trabajo.
    name: String,
}

impl Job {
    /// Crea un nuevo trabajo.
    ///
    /// # Argumentos
    /// * `name` - Nombre del trabajo.
    /// * `time` - Tiempo asociado.
    /// # Returns
    /// Un objeto `Job` con el nombre y tiempo especificados.
    fn new(name: &str, time: f64) -> Self {
        Job {
            time,
            name: name.to_string(),
        }
    }
}

/// Generador de trabajos periódicos.
struct Generator {
    component: Component,
    /// Puerto de entrada para detener la generación.
    i_stop: InPort<bool>,
    /// Puerto de salida de trabajos generados.
    o_job: OutPort<Job>,
    /// Tiempo hasta el próximo evento.
    sigma: f64,
    /// Periodo de generación.
    time: f64,
}

impl Generator {
    /// Crea un nuevo generador.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `time` - Periodo entre generaciones.
    /// # Returns
    /// Una instancia de `Generator` inicializada con el nombre y periodo especificados.
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let i = component.add_in_port::<bool>("i_stop");
        let o = component.add_out_port::<Job>("o_job");
        Generator {
            component,
            i_stop: i,
            o_job: o,
            sigma: 1.0,
            time: time,
        }
    }
}

impl Atomic for Generator {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Genera un nuevo trabajo.
    fn lambda(&self) {
        let job = Job::new("Job Generated", self.get_t_last());
        unsafe { self.o_job.add_value(job.clone()) };
        println!("{}: {} - {}", self.get_name(), job.name, self.get_t_last());
    }

    /// Programa el siguiente evento.
    fn delta_int(&mut self) {
        self.sigma = self.time;
    }

    /// Detiene el generador si llega el senial de stop.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        if !unsafe { self.i_stop.is_empty() } {
            self.sigma = f64::INFINITY;
        }
    }

     /// Tiempo hasta la próxima transición.
    fn ta(&self) -> f64 {
        self.sigma
    }
}

/// Procesador de trabajos.
struct Processor {
    component: Component,
    /// Entrada de trabajos.
    i_job: InPort<Job>,
    /// Salida de trabajos procesados.
    o_job: OutPort<Job>,
    /// Tiempo restante.
    sigma: f64,
    /// Tiempo de procesamiento.
    time: f64,
    /// Trabajo actual en proceso.
    job: Option<Job>,
}

impl Processor {
    /// Crea un nuevo procesador.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente
    /// * `time` - Tiempo que tarda en procesar cada trabajo
    ///
    /// # Returns
    /// Una instancia de `Processor` inicializada con el nombre y tiempo de procesamiento especificados.
    /// El procesador comienza sin trabajo asignado (`job = None`) y con `sigma` en infinito.
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let i = component.add_in_port::<Job>("i_job");
        let o = component.add_out_port::<Job>("o_job");
        Processor {
            component,
            i_job: i,
            o_job: o,
            sigma: f64::INFINITY,
            time: time,
            job: None,
        }
    }
}

impl Atomic for Processor {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Genera la salida cuando el trabajo termina.
    fn lambda(&self) {
        let job = Job::new("Job Processed", self.job.as_ref().unwrap().time);
        unsafe { self.o_job.add_value(job.clone()) };
        println!(
            "{}: {} - {}",
            self.get_name(),
            job.name,
            self.job.as_ref().unwrap().time
        );
    }

    /// Libera el procesador tras completar el trabajo.
    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
        self.job = None;
    }

     /// Recibe un nuevo trabajo si está libre.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        if self.job.is_none() {
            self.job = unsafe { self.i_job.get_values().first().cloned() };
            self.sigma = self.time
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}

/// Recoge estadísticas y detiene la simulación.
struct Transducer {
    component: Component,
    /// Trabajos que llegan.
    i_arrived_job: InPort<Job>,
    /// Trabajos procesados.
    i_solved_job: InPort<Job>,
    /// Señal de parada.
    o_stop: OutPort<bool>,
    /// Lista de trabajos recibidos.
    arrived_job: Vec<Job>,
    /// Lista de trabajos procesados.
    solved_job: Vec<Job>,
    /// Tiempo restante.
    sigma: f64,
}

impl Transducer {
    /// Crea un nuevo transductor.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `time` - Tiempo hasta emitir la señal de parada.
    ///
    /// # Returns
    /// Una instancia de `Transducer` inicializada con el nombre y tiempo especificados.
    /// El transductor comienza con las listas de trabajos vacías y `sigma` igual a `time`.
    fn new(name: &str, time: f64) -> Self {
        let mut component = Component::new(name);
        let a = component.add_in_port::<Job>("i_arrived_Job");
        let s = component.add_in_port::<Job>("i_solved_Job");
        let o = component.add_out_port::<bool>("o_stop");
        Transducer {
            component,
            i_arrived_job: a,
            i_solved_job: s,
            arrived_job: Vec::new(),
            solved_job: Vec::new(),
            o_stop: o,
            sigma: time,
        }
    }
}

impl Atomic for Transducer {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Emite la señal de parada.
    fn lambda(&self) {
        unsafe { self.o_stop.add_value(true) };
    }

    /// Finaliza la simulación.
    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
        println!("Stopped!");
    }

    /// Recoge estadísticas de entrada.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        if !unsafe { self.i_arrived_job.is_empty() } {
            self.arrived_job
                .extend_from_slice(unsafe { self.i_arrived_job.get_values() });
        }
        if !unsafe { self.i_solved_job.is_empty() } {
            self.solved_job
                .extend_from_slice(unsafe { self.i_solved_job.get_values() });
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}

/// Modelo acoplado EF (Generator + Transducer).
struct EF {
    coupled: Coupled,
}

impl EF {
    /// Construye el modelo acoplado `EF`.
    ///
    /// Este modelo está compuesto por:
    /// - Un `Generator` que produce trabajos periódicamente.
    /// - Un `Transducer` que recibe los trabajos generados y controla
    ///   la finalización de la simulación.
    ///
    /// Además, define los acoplamientos internos y externos entre componentes:
    /// - El `Generator` envía trabajos al `Transducer`.
    /// - El `Transducer` puede detener al `Generator`.
    /// - Se permite la entrada externa de trabajos procesados al `Transducer`.
    /// - La salida del `Generator` se expone como salida del modelo.
    ///
    /// # Argumentos
    /// * `name` - Nombre del modelo acoplado
    /// * `generate` - Periodo de generación de trabajos del `Generator`
    /// * `analyze` - Tiempo de análisis del `Transducer`
    ///
    /// # Returns
    /// Una instancia de `EF` que contiene el modelo `Coupled` configurado
    /// con sus componentes y acoplamientos.
    fn new(name: &str, generate: f64, analyze: f64) -> Self {
        let mut coupled = Coupled::new(name);
        coupled.add_in_port::<Job>("i_job");
        coupled.add_out_port::<Job>("o_job");

        let g = Generator::new("Generator", generate);
        let t = Transducer::new("Transducer", analyze);

        coupled.add_component(Box::new(g));
        coupled.add_component(Box::new(t));

        coupled.add_ic("Generator", "o_job", "Transducer", "i_arrived_Job");
        coupled.add_ic("Transducer", "o_stop", "Generator", "i_stop");
        coupled.add_eic("i_job", "Transducer", "i_solved_Job");
        coupled.add_eoc("Generator", "o_job", "o_job");

        Self { coupled }
    }
}

/// Construye el modelo acoplado completo `EFP`.
///
/// Este modelo representa un sistema de procesamiento de trabajos compuesto por:
/// - Un subsistema `EF` (Generator + Transducer)
/// - Un `Processor` que procesa los trabajos generados
///
/// Flujo del sistema:
/// 1. El `Generator` (dentro de `EF`) crea trabajos periódicamente.
/// 2. Los trabajos se envían al `Processor`.
/// 3. El `Processor` procesa los trabajos y los devuelve al `EF`.
/// 4. El `Transducer` (dentro de `EF`) recibe tanto los trabajos generados
///    como los procesados, permitiendo analizar el rendimiento del sistema
///    y eventualmente detener la simulación.
///
/// Acoplamientos definidos:
/// - `EF.o_job -> Processor.i_job`: envío de trabajos para procesamiento.
/// - `Processor.o_job -> EF.i_job`: retorno de trabajos procesados.
///
/// # Argumentos
/// * `g_time` - Periodo de generación de trabajos en el `Generator`
/// * `p_time` - Tiempo de procesamiento de cada trabajo en el `Processor`
/// * `t_time` - Tiempo de análisis del `Transducer` (control de parada)
///
/// # Returns
/// Un modelo `Coupled` completamente configurado que representa el sistema `EFP`
fn efp(g_time: f64, p_time: f64, t_time: f64) -> Coupled {
    let mut efp = Coupled::new("EFP");
    let ef = EF::new("EF", g_time, t_time);
    let p = Processor::new("Processor", p_time);

    efp.add_component(Box::new(ef.coupled));
    efp.add_component(Box::new(p));

    efp.add_ic("EF", "o_job", "Processor", "i_job");
    efp.add_ic("Processor", "o_job", "EF", "i_job");

    efp
}

/// Construye el modelo completo `EFP` y ejecuta la simulación
/// utilizando un coordinador raíz.
/// 
/// # Nota
/// - Aunque se usa `f64::INFINITY`, la simulación puede finalizar antes
///   si el `Transducer` emite la señal de parada.
fn main() {
    let efp = efp(1.0, 3.0, 100.0);
    let mut simulator = RootCoordinator::new(efp);
    simulator.simulate(f64::INFINITY);
}
