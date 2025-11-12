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
    q_error_range: (f64, f64),
    w_history_range: (f64, f64),
    rw_speeds_history_range: (f64, f64),
    range_margin: f64
}

impl Transducer {
    pub fn new(name: &str, m: f64) -> Self {
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
            q_error_range: (0.0, 0.0),
            w_history_range: (0.0, 0.0),
            rw_speeds_history_range: (0.0, 0.0),
            range_margin: m,
        }
    }

    pub fn get_q_error_history(&self) -> &[Quaternion] {
        self.q_error_history.as_slice()
    }

    pub fn get_w_history(&self) -> &[Vec3] {
        self.w_history.as_slice()
    }

    pub fn get_rw_speeds_history(&self) -> &[Vec3] {
        self.rw_speeds_history.as_slice()
    }

    pub fn get_q_error_range_with_margin(&self) -> (f64, f64) {
        let range = self.q_error_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    pub fn get_w_history_range_with_margin(&self) -> (f64, f64) {
        let range = self.w_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    pub fn get_rw_speeds_range_with_margin(&self) -> (f64, f64) {
        let range = self.rw_speeds_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }
    
    fn update_range(compare: (f64, f64), values: Vec<f64>) -> (f64, f64) {
        values.iter().fold(compare, |(min_v, max_v), &val| {
            (f64::min(min_v, val), f64::max(max_v, val))
        })
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
            if let Some(q_error) = unsafe { self.i_q_error.get_values().first().copied() } {
                self.q_error_history.push(q_error);
                let values = vec![q_error.0.i, q_error.0.j, q_error.0.k];
                self.q_error_range = Transducer::update_range(self.q_error_range, values);
            }
        }
        if !unsafe { self.i_w.is_empty() } {
            if let Some(w) = unsafe { self.i_w.get_values().first().copied() } {
                self.w_history.push(w);
                let values = vec![w.0.x, w.0.y, w.0.z];
                self.w_history_range = Transducer::update_range(self.w_history_range, values);
            }
        }
        if !unsafe { self.i_rw_speeds.is_empty() } {
            if let Some(rw_speeds) = unsafe { self.i_rw_speeds.get_values().first().copied() } {
                self.rw_speeds_history.push(rw_speeds);
                let values = vec![rw_speeds.0.x, rw_speeds.0.y, rw_speeds.0.z];
                self.rw_speeds_history_range = Transducer::update_range(self.rw_speeds_history_range, values);
            }
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
