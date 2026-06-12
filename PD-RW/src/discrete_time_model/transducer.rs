use crate::discrete_time_model::types::{Quaternion, Vec3};
use xdevs::modeling::*;

/// Este componente no actúa sobre la dinámica del satélite.
/// Su función es:
/// - Registrar datos del sistema.
/// - Almacenar historiales de variables.
/// - Calcular rangos de comportamiento.
///
/// Variables monitorizadas:
/// - Velocidad angular (`w`).
/// - Error de actitud (`q_error`).
/// - Velocidades de ruedas de reacción (`rw_speeds`).
pub struct Transducer {
    component: Component,
    /// Entrada de velocidad angular del satélite.
    i_w: InPort<Vec3>,
    /// Entrada de error de actitud.
    i_q_error: InPort<Quaternion>,
    /// Entrada de velocidades de ruedas de reacción.
    i_rw_speeds: InPort<Vec3>,
    /// Tiempo hasta la próxima activación.
    sigma: f64,
    /// Historial del error de actitud.
    q_error_history: Vec<Quaternion>,
    /// Historial de velocidad angular.
    w_history: Vec<Vec3>,
    /// Historial de velocidades de ruedas.
    rw_speeds_history: Vec<Vec3>,
    /// Rango observado del error de actitud (min, max).
    q_error_range: (f64, f64),
    /// Rango de velocidad angular (min, max).
    w_history_range: (f64, f64),
    /// Rango de velocidades de ruedas (min, max).
    rw_speeds_history_range: (f64, f64),
    /// Margen adicional para análisis estadístico.
    range_margin: f64
}



impl Transducer {
    /// Crea un nuevo transductor de monitorización.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `m` - Margen de expansión para los rangos estadísticos.
    pub fn new(name: &str, m: f64) -> Self {
        let mut component = Component::new(name);
        let i_w = component.add_in_port::<Vec3>("i_w");
        let i_qe = component.add_in_port::<Quaternion>("i_qerror");
        let i_rw = component.add_in_port::<Vec3>("i_rw_speeds");
        Transducer {
            component: component,
            i_w: i_w,
            i_q_error: i_qe,
            i_rw_speeds: i_rw,
            // Transition to Waiting state
            sigma: f64::INFINITY,
            q_error_history: Vec::new(),
            w_history: Vec::new(),
            rw_speeds_history: Vec::new(),
            q_error_range: (0.0, 0.0),
            w_history_range: (0.0, 0.0),
            rw_speeds_history_range: (0.0, 0.0),
            range_margin: m,
        }
    }

    /// Historial del error de actitud.
    pub fn get_q_error_history(&self) -> &[Quaternion] {
        self.q_error_history.as_slice()
    }

    /// Historial de velocidad angular.
    pub fn get_w_history(&self) -> &[Vec3] {
        self.w_history.as_slice()
    }

    /// Historial de velocidades de ruedas de reacción.
    pub fn get_rw_speeds_history(&self) -> &[Vec3] {
        self.rw_speeds_history.as_slice()
    }

    /// Rango ampliado del error de actitud.
    pub fn get_q_error_range_with_margin(&self) -> (f64, f64) {
        let range = self.q_error_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

     /// Rango ampliado de velocidad angular.
    pub fn get_w_history_range_with_margin(&self) -> (f64, f64) {
        let range = self.w_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    /// Rango ampliado de velocidades de ruedas.
    pub fn get_rw_speeds_range_with_margin(&self) -> (f64, f64) {
        let range = self.rw_speeds_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    /// Actualiza el rango mínimo y máximo de un conjunto de valores.
    fn update_range(compare: (f64, f64), values: Vec<f64>) -> (f64, f64) {
        values.iter().fold(compare, |(min_v, max_v), &val| {
            (f64::min(min_v, val), f64::max(max_v, val))
        })
    }
}

impl Atomic for Transducer {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// No genera salida (solo monitorización).
    fn lambda(&self) {}

    /// Permanece inactivo.
    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
    }

    /// Al recibir nuevas muestras:
    /// - Almacena históricos.
    /// - Actualiza rangos mínimos y máximos.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        if !unsafe { self.i_q_error.is_empty() } {
            if let Some(q_error) = unsafe { self.i_q_error.get_values().first().copied() } {
                self.q_error_history.push(q_error);
                let values = vec![q_error.0.i, q_error.0.j, q_error.0.k];
                self.q_error_range = Transducer::update_range(self.q_error_range, values);
            }
        }
        if !unsafe { self.i_w.is_empty() } {
            if let Some(w) = unsafe { self.i_w.get_values().first().copied() } {
                self.w_history.push(w);
                let values = vec![w.0.x, w.0.y, w.0.z];
                self.w_history_range = Transducer::update_range(self.w_history_range, values);
            }
        }
        if !unsafe { self.i_rw_speeds.is_empty() } {
            if let Some(rw_speeds) = unsafe { self.i_rw_speeds.get_values().first().copied() } {
                self.rw_speeds_history.push(rw_speeds);
                let values = vec![rw_speeds.0.x, rw_speeds.0.y, rw_speeds.0.z];
                self.rw_speeds_history_range = Transducer::update_range(self.rw_speeds_history_range, values);
            }
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
