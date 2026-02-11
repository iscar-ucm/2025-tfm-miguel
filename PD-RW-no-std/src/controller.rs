use crate::types::{Quaternion, Vec3};
use xdevs::*;

pub struct ControllerState{
    w: Option<Vec3>,
    q: Option<Quaternion>,
    torque: Option<Vec3>,
    q_error: Option<Quaternion>,
    sigma: f64,
    time: f64,
    q_target: Quaternion,
    kp: f64,
    kd: f64,
    max_torque_rw: f64,
}

impl ControllerState{
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
    // Calculates the error quaternion
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
    fn delta_int(state: &mut Self::State) {
        // After sending the command, go back to waiting
        state.w = None;
        state.q = None;
        state.sigma = f64::INFINITY;
    }

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