use crate::types::{Quaternion, Vec3};
use nalgebra::Matrix3;
use xdevs::*;

pub struct SatelliteDynamicsState {
    sigma: f64,
    _time: f64,
    w: Vec3,
    q: Quaternion,
    h_rw: Option<Vec3>,
    torque: Option<Vec3>,
    h: f64,
    i_sat: Matrix3<f64>,
    wdot: Vec3,
    qdot: Quaternion,
}

impl SatelliteDynamicsState {
    pub fn new(time: f64, w0: Vec3, q0: Quaternion, h: f64, i_sat: Matrix3<f64>) -> Self {
        Self {
            sigma: time, // Send initial state immediately
            _time: time,
            // Initial state
            w: w0,
            q: q0,
            h_rw: None,
            torque: None,
            h: h,
            i_sat,
            wdot: Vec3::default(),
            qdot: Quaternion::default(),
        }
    }

    fn compute_derivatives(&mut self) {
        // --- Dynamics ---
        // Skew-symmetric matrix for cross products
        let w = self.w.0;
        let w_skew = Matrix3::new(0., -w.z, w.y, w.z, 0., -w.x, -w.y, w.x, 0.);

        // Quaternion kinematics matrix
        let omega_q = nalgebra::Quaternion::new(0.0, w.x, w.y, w.z);

        // Update state based on the previous one
        // w
        let h_total = self.i_sat * self.w.0 + self.h_rw.unwrap().0;
        if let Some(i_inv) = self.i_sat.try_inverse() {
            if let Some(torque) = self.torque {
                self.wdot = Vec3(i_inv * (torque.0 - w_skew * h_total));
            }
        }
        self.qdot = Quaternion(0.5 * self.q.0 * omega_q);
    }

    fn compute_next_state(&mut self, h: f64) {
        self.w = Vec3(self.w.0 + h * self.wdot.0); // using a simple Euler integration
        self.q = Quaternion(self.q.0 + h * self.qdot.0); //using a simple Euler integration
    }
}

component! {
    ident = SatelliteDynamics,
    input = {
        i_h_rw<Vec3>,
        i_torque<Vec3>,
    },
    output = {
        o_w<Vec3>,
        o_q<Quaternion>,
    },
    state = SatelliteDynamicsState
}

impl Atomic for SatelliteDynamics {
    fn delta_int(state: &mut Self::State) {
        // Compute the next state if possible
        if !state.h_rw.is_none() && !state.torque.is_none() {
            state.compute_derivatives();
            state.compute_next_state(state.h);
        }
        // Schedule the next output
        state.sigma = state.h
    }

    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        // An external event is a new h_rw or torque command
        if !x.i_h_rw.is_empty() {
            state.h_rw = x.i_h_rw.get_values().first().copied();
        }
        if !x.i_torque.is_empty() {
            state.torque = x.i_torque.get_values().first().copied();
        }
    }

    fn lambda(state: &Self::State, output: &mut Self::Output) {
        // Send the current attitude and angular velocity
        output.o_q.add_value(state.q).unwrap();
        output.o_w.add_value(state.w).unwrap();
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}
