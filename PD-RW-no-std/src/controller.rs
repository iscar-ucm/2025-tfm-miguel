use crate::types::{Quaternion, Vec3};
use xdevs::*;

/// Estado interno del controlador de actitud.
/// Implementa un control PD en espacio de cuaterniones para un nanosatélite.
pub struct ControllerState{
    /// Última velocidad angular recibida.
    w: Option<Vec3>,
    /// Última actitud recibida.
    q: Option<Quaternion>,
    /// Torque calculado por el controlador.
    torque: Option<Vec3>,
    /// Error de actitud calculado.
    q_error: Option<Quaternion>,
    /// Tiempo hasta próxima activación del controlador.
    sigma: f64,
    /// Periodo de control.
    time: f64,
    /// Actitud objetivo (referencia).
    q_target: Quaternion,
    /// Ganancia proporcional (PD).
    kp: f64,
    /// Ganancia derivativa (PD).
    kd: f64,
    /// Saturación máxima del torque (ruedas de reacción).
    max_torque_rw: f64,
}

impl ControllerState{
    /// Crea un nuevo ControllerState.
    ///
    /// # Argumentos
    /// * `time` - Periodo de control.
    /// * `q_target` - Actitud objetivo. (referencia)
    /// * `kp` - Ganancia proporcional.
    /// * `kd` - Ganancia derivativa.
    /// * `max_torque_rw` - Saturación máxima del torque.
    pub fn new(
        time: f64,
        q_target: Quaternion,
        kp: f64,
        kd: f64,
        max_torque_rw: f64,
    ) -> Self {
        Self {
            // Initialize the torque command to zero
            w: None,
            q: None,
            torque: None,
            q_error: None,
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            // # q_target is the desired attitude in quaternion form
            q_target: q_target,
            kp,
            kd,
            max_torque_rw: max_torque_rw,
        }
    }

    /// Calcula el error de actitud en cuaterniones.
    ///
    /// # Argumentos
    /// * `q_current` - actitud actual.
    /// * `q_target` - actitud deseada.
    ///
    /// # Returns
    /// Cuaternión de error. (q_error = q_current * conjugado(q_target))
    fn quaternion_error(q_current: Quaternion, q_target: Quaternion) -> Quaternion {
        Quaternion(q_current.0 * q_target.0.conjugate())
    }
}

component!{
    ident = Controller,
    input = {
        i_w<Vec3>,
        i_q<Quaternion>
    },
    output = {
        o_torque<Vec3>,
        o_qerror<Quaternion>,
    },
    state = ControllerState
}

impl Atomic for Controller{
    /// Tras emitir la señal de control, el sistema vuelve al estado de espera.
    fn delta_int(state: &mut Self::State) {
        // After sending the command, go back to waiting
        state.w = None;
        state.q = None;
        state.sigma = f64::INFINITY;
    }

    /// Recibe nuevas medidas del sistema:
    /// - velocidad angular (`w`).
    /// - actitud (`q`).
    ///
    /// Cuando ambos datos están disponibles:
    /// 1. Se calcula el error de actitud.
    /// 2. Se aplica la ley de control PD.
    /// 3. Se satura el torque.
    /// 4. Se programa una salida inmediata.
    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        // Receive new current attitude data
        if !x.i_w.is_empty() {
            state.w = x.i_w.get_values().first().copied();
        }
        if !x.i_q.is_empty() {
            state.q = x.i_q.get_values().first().copied();
        }

        if !state.w.is_none() && !state.q.is_none() {
            /*
            1. Calculate attitude error quaternion (q_error = q_current * conjugate(q_target))
            2. Extract error vector (e.g., from the vector part of q_error)
             */

            if let Some(q) = state.q {
                state.q_error = Some(ControllerState::quaternion_error(q, state.q_target));
            }

            // 3. Apply PD control law:
            if let (Some(q_error), Some(w)) = (state.q_error.as_ref(), state.w.as_ref()) {
                // imag() get the vector (x,y,z) (imaginary) part
                state.torque = Some(Vec3(-state.kp * q_error.0.imag() - state.kd * w.0));
            }

            // Saturate the control torque
            if let Some(torque) = &state.torque {
                state.torque = Some(torque.clamp(-state.max_torque_rw, state.max_torque_rw));
            }

            // Schedule an immediate output
            state.sigma = state.time;
        }
    }

    /// Envía:
    /// * El torque de control calculado.
    /// * El error de actitud asociado.
    fn lambda(state: &Self::State, output: &mut Self::Output) {
        // Send the computed torque command
        if let (Some(q_error), Some(torque)) = (state.q_error, state.torque) {
            output.o_qerror.add_value(q_error).unwrap();
            output.o_torque.add_value(torque).unwrap();
        }
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}