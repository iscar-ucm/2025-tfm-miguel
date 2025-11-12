use crate::types::Vec3;
use nalgebra::Matrix3;
use xdevs::*;

pub struct RWState {
    rw_speeds: Vec3,
    torque: Option<Vec3>,
    h_rw: Vec3,
    sigma: f64,
    time: f64,
    inertia_rw: Matrix3<f64>,
    max_speed_rw: f64,
    h: f64,
    rw_speeds_dot: Vec3,
}

impl RWState {
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

    fn compute_derivatives(&mut self) {
        if let Some(torque) = self.torque {
            if let Some(inertia_inv) = self.inertia_rw.try_inverse() {
                self.rw_speeds_dot = Vec3(inertia_inv * -torque.0);
            }
        }
    }

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
    fn delta_int(state: &mut Self::State) {
        state.compute_next_state(state.h);
        state.sigma = f64::INFINITY;
    }

    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;

        state.torque = x.i_torque.get_values().first().copied();
        // With the new torque, we can compute the derivatives
        state.compute_derivatives();
        state.sigma = state.time;
    }

    fn lambda(state: &Self::State, output: &mut Self::Output) {
        output.o_h_rw.add_value(state.h_rw).unwrap();
        output.o_rw_speeds.add_value(state.rw_speeds).unwrap();
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}
