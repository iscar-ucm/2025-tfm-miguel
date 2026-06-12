use crate::types::Vec3;
use nalgebra::Matrix3;
use xdevs::*;

/// Este modelo representa la dinámica simplificada de un conjunto de ruedas de reacción:
/// - Velocidad angular de cada rueda
/// - Dinámica bajo torque aplicado
/// - Cálculo de momento angular
pub struct RWState {
    /// Velocidades actuales de las ruedas.
    rw_speeds: Vec3,
    /// Torque recibido desde el controlador.
    torque: Option<Vec3>,
    /// Momento angular actual de las ruedas.
    h_rw: Vec3,
    /// Tiempo hasta la próxima actualización.
    sigma: f64,
    /// Periodo de integración del modelo.
    time: f64,
    /// Inercia de las ruedas de reacción.
    inertia_rw: Matrix3<f64>,
    /// Velocidad máxima permitida de las ruedas.
    max_speed_rw: f64,
    /// Paso de integración (Euler).
    h: f64,
    /// Derivada de la velocidad de las ruedas (aceleración angular).
    rw_speeds_dot: Vec3,
}

impl RWState {
    /// Crea un nuevo modelo de ruedas de reacción.
    ///
    /// # Argumentos
    /// * `time` - Periodo de actualización del sistema.
    /// * `rw_speeds_initial` - Velocidad inicial de las ruedas.
    /// * `i_rw` - Matriz de inercia de las ruedas.
    /// * `m_speed_rw` - Velocidad máxima permitida.
    /// * `h` - Paso de integración numérica.
    pub fn new(
        time: f64,
        rw_speeds_initial: Vec3,
        i_rw: Matrix3<f64>,
        m_speed_rw: f64,
        h: f64,
    ) -> Self {
        Self {
            rw_speeds: rw_speeds_initial,
            torque: None,
            // Initial reaction wheel angular momentum
            h_rw: Vec3(i_rw * rw_speeds_initial.0),
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            inertia_rw: i_rw,
            max_speed_rw: m_speed_rw,
            h: h,
            rw_speeds_dot: Vec3::default(),
        }
    }
    /// Calcula la derivada de la velocidad de las ruedas.
    fn compute_derivatives(&mut self) {
        if let Some(torque) = self.torque {
            if let Some(inertia_inv) = self.inertia_rw.try_inverse() {
                self.rw_speeds_dot = Vec3(inertia_inv * -torque.0);
            }
        }
    }

    /// Integra el estado de las ruedas (Euler explícito).
    ///
    /// Actualiza:
    /// * Velocidades angulares.
    /// * Saturación de velocidad.
    /// * Momento angular.
    fn compute_next_state(&mut self, h: f64) {
        // Compute the next state:
        // Calculate the next state of the reaction wheels (Euler integration)
        self.rw_speeds = Vec3(self.rw_speeds.0 + h * self.rw_speeds_dot.0);

        // Limit the speed of the reaction wheels
        // At the moment, I am not applying this limit
        self.rw_speeds = self.rw_speeds.clamp(-self.max_speed_rw, self.max_speed_rw);

        // Update the wheel momentum
        self.h_rw = Vec3(self.inertia_rw * self.rw_speeds.0);
    }
}

component! {
    ident = RW,
    input = {
        i_torque<Vec3>,
    },
    output = {
        o_h_rw<Vec3>,
        o_rw_speeds<Vec3>,
    },
    state = RWState
}

impl Atomic for RW {
    /// Actualiza el estado físico del sistema tras la integración.
    fn delta_int(state: &mut Self::State) {
        state.compute_next_state(state.h);
        state.sigma = f64::INFINITY;
    }

    /// Recibe torque desde el controlador y actualiza la dinámica.
    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;

        state.torque = x.i_torque.get_values().first().copied();
        // With the new torque, we can compute the derivatives
        state.compute_derivatives();
        state.sigma = state.time;
    }

    /// Envía:
    /// * Momento angular de las ruedas.
    /// * Velocidades actuales.
    fn lambda(state: &Self::State, output: &mut Self::Output) {
        output.o_h_rw.add_value(state.h_rw).unwrap();
        output.o_rw_speeds.add_value(state.rw_speeds).unwrap();
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}
