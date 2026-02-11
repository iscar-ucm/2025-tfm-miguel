use crate::discrete_time_model::types::Vec3;
use nalgebra::Matrix3;
use xdevs::modeling::*;

pub struct RW {
    component: Component,
    i_torque: InPort<Vec3>,
    o_h_rw: OutPort<Vec3>,
    o_rw_speeds: OutPort<Vec3>,
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

impl RW {
    pub fn new(
        name: &str,
        time: f64,
        rw_speeds_initial: Vec3,
        i_rw: Matrix3<f64>,
        m_speed_rw: f64,
        h: f64,
    ) -> Self {
        let mut component = Component::new(name);
        let i_t = component.add_in_port::<Vec3>("i_torque");
        let o_h = component.add_out_port::<Vec3>("o_h_rw");
        let o_rw = component.add_out_port::<Vec3>("o_rw_speeds");
        RW {
            component: component,
            i_torque: i_t,
            o_h_rw: o_h,
            o_rw_speeds: o_rw,
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
        if let Some(torque) = &self.torque {
            if let Some(inertia_inv) = self.inertia_rw.try_inverse() {
                self.rw_speeds_dot = Vec3(inertia_inv * -&torque.0);
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

impl Atomic for RW {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {
        unsafe { self.o_h_rw.add_value(self.h_rw) };
        unsafe { self.o_rw_speeds.add_value(self.rw_speeds) };
    }

    fn delta_int(&mut self) {
        self.compute_next_state(self.h);
        self.sigma = f64::INFINITY
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        self.torque = unsafe { self.i_torque.get_values().first().copied() };
        // With the new torque, we can compute the derivatives
        self.compute_derivatives();
        self.sigma = self.time;
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
