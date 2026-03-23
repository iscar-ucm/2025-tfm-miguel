use crate::types::{Quaternion, Vec3};

use xdevs::*;

pub struct CCUState {
    sigma: f64,
    time: f64,
    w: Option<Vec3>,
    q: Option<Quaternion>,
}

impl CCUState {
    pub fn new(sigma: f64, time: f64, w0: Vec3, q0: Quaternion) -> Self {
        Self {
            sigma: sigma, // Send initial state immediately
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
    fn delta_int(state: &mut Self::State) {
        state.q = None;
        state.w = None;
        state.sigma = f64::INFINITY;
    }

    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        // An external event is a new h_rw or torque command
        if !x.i_q_sw.is_empty() {
            state.q = x.i_q_sw.get_values().first().copied();
        }
        if !x.i_w_sw.is_empty() {
            state.w = x.i_w_sw.get_values().first().copied();
        }
        if !x.i_q_hw.is_empty() {
            state.q = x.i_q_hw.get_values().first().copied();
        }
        if !x.i_w_hw.is_empty() {
            state.w = x.i_w_hw.get_values().first().copied();
        }

        if !state.w.is_none() && !state.q.is_none() {
            state.sigma = state.time;
        }
    }

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