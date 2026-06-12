use crate::types::{Quaternion, Vec3};
use xdevs::*;

/// Estado interno del componente CCU.
///
/// Este estado almacena:
/// * w: velocidad angular actual.
/// * q: orientación actual.
pub struct CCUState {
    /// Tiempo hasta el próximo evento interno.
    sigma: f64,
    /// Periodo de control.
    time: f64,
    /// Velocidad angular actual.
    w: Option<Vec3>,
    /// Orientación actual.
    q: Option<Quaternion>,
}

impl CCUState {
    /// Crea un nuevo estado para la CCU.
    ///
    /// # Argumentos
    /// * `time` - Periodo de control.
    /// * `w0` - Velocidad angular inicial.
    /// * `q0` - Orientación inicial.
    pub fn new(time: f64, w0: Vec3, q0: Quaternion) -> Self {
        Self {
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            // Initial state
            w: Some(w0),
            q: Some(q0),
        }
    }
}

component! {
    ident = CCU,
    input = {
        i_q_hw<Quaternion>,
        i_w_hw<Vec3>,
        i_q_sw<Quaternion>,
        i_w_sw<Vec3>,
    },
    output = {
        o_w<Vec3>,
        o_q<Quaternion>,
    },
    state = CCUState
}

impl Atomic for CCU {
    /// Limpia el estado actual y vuelve al estado pasivo
    fn delta_int(state: &mut Self::State) {
        state.q = None;
        state.w = None;
        state.sigma = f64::INFINITY;
    }

    /// Actualizar el estado actual en función del la modalidad establecida y emite una salida inmediata
    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        #[cfg(feature = "simulation")]
        {
            if !x.i_q_sw.is_empty() {
                state.q = x.i_q_sw.get_values().first().copied();
            }
            if !x.i_w_sw.is_empty() {
                state.w = x.i_w_sw.get_values().first().copied();
            }
        }
        #[cfg(feature = "hardware")]
        {
            if !x.i_q_hw.is_empty() {
                state.q = x.i_q_hw.get_values().first().copied();
            }
            if !x.i_w_hw.is_empty() {
                state.w = x.i_w_hw.get_values().first().copied();
            }
        }

        if !state.w.is_none() && !state.q.is_none() {
            // Schedule an immediate output
            state.sigma = state.time;
        }
    }

    /// Envía:
    /// * El cuaternión.
    /// * La velocidad angular.
    fn lambda(state: &Self::State, output: &mut Self::Output) {
        // Send the current attitude and angular velocity
        if let (Some(w), Some(q)) = (state.w, state.q) {
            output.o_q.add_value(q).unwrap();
            output.o_w.add_value(w).unwrap();
        }
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}