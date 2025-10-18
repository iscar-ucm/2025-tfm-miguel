use crate::discrete_time_model::types::{Quaternion, Vec3};
use xdevs::modeling::*;

pub struct Transducer {
    component: Component,
    i_w: InPort<Vec3>,
    i_q_error: InPort<Quaternion>,
    i_rw_speeds: InPort<Vec3>,
    sigma: f64,
    q_error_history: Vec<Quaternion>,
    w_history: Vec<Vec3>,
    rw_speeds_history: Vec<Vec3>,
}

impl Transducer {
    pub fn new(name: &str) -> Self {
        let mut component = Component::new(name);
        let i_w = component.add_in_port::<Vec3>("i_w");
        let i_qe = component.add_in_port::<Quaternion>("i_qerror");
        let i_rw = component.add_in_port::<Vec3>("i_rw_speeds");
        Transducer {
            component: component,
            i_w: i_w,
            i_q_error: i_qe,
            i_rw_speeds: i_rw,
            // Transition to Waiting state
            sigma: f64::INFINITY,
            q_error_history: Vec::new(),
            w_history: Vec::new(),
            rw_speeds_history: Vec::new(),
        }
    }

    pub fn get_q_error_history(&self) -> Vec<Quaternion> {
        self.q_error_history.clone()
    }

    pub fn get_w_history(&self) -> Vec<Vec3> {
        self.w_history.clone()
    }

    pub fn get_rw_speeds_history(&self) -> Vec<Vec3> {
        self.rw_speeds_history.clone()
    }
}

impl Atomic for Transducer {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    fn lambda(&self) {}

    fn delta_int(&mut self) {
        self.sigma = f64::INFINITY;
    }

    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        if !unsafe { self.i_q_error.is_empty() } {
            if let Some(q_error) = unsafe { self.i_q_error.get_values().first().cloned() } {
                self.q_error_history.push(q_error);
            }
        }
        if !unsafe { self.i_w.is_empty() } {
            if let Some(w) = unsafe { self.i_w.get_values().first().cloned() } {
                self.w_history.push(w);
            }
        }
        if !unsafe { self.i_rw_speeds.is_empty() } {
            if let Some(rw_speeds) = unsafe { self.i_rw_speeds.get_values().first().cloned() } {
                self.rw_speeds_history.push(rw_speeds);
            }
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
