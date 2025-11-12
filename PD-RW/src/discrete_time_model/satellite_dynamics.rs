use crate::discrete_time_model::types::{Quaternion, Vec3};
use nalgebra::Matrix3;
use xdevs::modeling::*;

pub struct SatelliteDynamics {
    component: Component,
    i_h_rw: InPort<Vec3>,
    i_torque: InPort<Vec3>,
    o_w: OutPort<Vec3>,
    o_q: OutPort<Quaternion>,
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

impl SatelliteDynamics {
    pub fn new(
        name: &str,
        time: f64,
        w0: Vec3,
        q0: Quaternion,
        h: f64,
        i_sat: Matrix3<f64>,
    ) -> Self {
        let mut component = Component::new(name);
        let i_h_rw = component.add_in_port::<Vec3>("i_h_rw");
        let i_t = component.add_in_port::<Vec3>("i_torque");
        let o_w = component.add_out_port::<Vec3>("o_w");
        let o_q = component.add_out_port::<Quaternion>("o_q");
        SatelliteDynamics {
            component: component,
            i_h_rw: i_h_rw,
            i_torque: i_t,
            o_w: o_w,
            o_q: o_q,
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
            if let Some(torque) = &self.torque {
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

impl Atomic for SatelliteDynamics {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {
        // Send the current attitude and angular velocity
        unsafe { self.o_q.add_value(self.q) };
        unsafe { self.o_w.add_value(self.w) };
    }

    fn delta_int(&mut self) {
        // Compute the next state if possible
        if !self.h_rw.is_none() && !self.torque.is_none() {
            self.compute_derivatives();
            self.compute_next_state(self.h);
        }
        // Schedule the next output
        self.sigma = self.h
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        // An external event is a new h_rw or torque command
        if !unsafe { self.i_h_rw.is_empty() } {
            self.h_rw = unsafe { self.i_h_rw.get_values().first().copied() };
        }
        if !unsafe { self.i_torque.is_empty() } {
            self.torque = unsafe { self.i_torque.get_values().first().copied() };
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
