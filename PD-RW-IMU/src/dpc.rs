use crate::types::{Vec3};
use nalgebra::Vector3;
use xdevs::*;

/// Estado interno del componente DPC.
pub struct DPCState {
    /// Tiempo hasta el próximo evento interno.
    sigma: f64,
    /// Periodo de control.
    time: f64,
    /// Torque actual almacenado.
    torque: Option<Vec3>,
    /// Valor de PWM.
    pwm: Vec3,
}

impl DPCState {
    /// Crea un nuevo estado para el componente DPC.
    ///
    /// # Parámetros
    /// * `time`: tiempo de espera antes de generar una salida.
    pub fn new(time: f64) -> Self {
        Self {
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            // Initial state
            torque: None,
            pwm: Vec3(Vector3::new(50.0, 50.0, 50.0)),
        }
    }
}

component! {
    ident = DPC,
    input = {
        i_torque<Vec3>,
        i_pwm<Vec3>,
    },
    output = {
        o_pwm<Vec3>,
    },
    state = DPCState
}

impl Atomic for DPC {
    /// Limpia el estado actual y vuelve al estado pasivo
    fn delta_int(state: &mut Self::State) {
        state.torque = None;
        state.sigma = f64::INFINITY;
    }

    /// Actualizar el estado actual y emite una salida inmediata
    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        if !x.i_pwm.is_empty() {
            if let Some(pwm) = x.i_pwm.get_values().first().copied() {
                state.pwm = pwm;
            }
        }
        if !x.i_torque.is_empty() {
            if let Some(torque) = x.i_torque.get_values().first().copied() {
                state.torque = Some(torque);
                state.pwm = Vec3(state.pwm.0 + torque.0).clamp(0.0, 100.0);
            }
        }

        if !state.torque.is_none() {
            // Schedule an immediate output
            state.sigma = state.time;
        }
    }

    /// Envía el valor de PWM.
    fn lambda(state: &Self::State, output: &mut Self::Output) {
        // Send the current PWM value
        output.o_pwm.add_value(state.pwm).unwrap();
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}