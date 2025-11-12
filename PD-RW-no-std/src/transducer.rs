use crate::types::{Quaternion, Vec3};
use xdevs::*;
use std::{rc::Rc, cell::RefCell};
pub type SharedTransducerState = Rc<RefCell<TransducerState>>;

pub struct TransducerState{
    sigma: f64,
    q_error_history: Vec<Quaternion>,
    w_history: Vec<Vec3>,
    rw_speeds_history: Vec<Vec3>,
    q_error_range: (f64, f64),
    w_history_range: (f64, f64),
    rw_speeds_history_range: (f64, f64),
    range_margin: f64
}

impl TransducerState{
    pub fn new(m: f64) -> Self {
        Self {
            // Transition to Waiting state
            sigma: f64::INFINITY,
            q_error_history: Vec::new(),
            w_history: Vec::new(),
            rw_speeds_history: Vec::new(),
            q_error_range: (f64::INFINITY, f64::NEG_INFINITY),
            w_history_range: (f64::INFINITY, f64::NEG_INFINITY),
            rw_speeds_history_range: (f64::INFINITY, f64::NEG_INFINITY),
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
        let range = &self.q_error_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    pub fn get_w_history_range_with_margin(&self) -> (f64, f64) {
        let range = &self.w_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }

    pub fn get_rw_speeds_range_with_margin(&self) -> (f64, f64) {
        let range = &self.rw_speeds_history_range;
        let margin = (range.1 - range.0).abs() * self.range_margin;
        (range.0 - margin, range.1 + margin)
    }
    
    fn update_range(compare: (f64, f64), values: &[f64]) -> (f64, f64) {
        values.iter().fold(compare, |(min_v, max_v), &x| {
            (min_v.min(x), max_v.max(x))
        })
    }
}

component!{
    ident = Transducer,
    input = {
        i_w<Vec3>,
        i_q_error<Quaternion>,
        i_rw_speeds<Vec3>
    },
    state = SharedTransducerState
}

impl Atomic for Transducer {
    fn delta_int(state: &mut Self::State) {
        state.borrow_mut().sigma = f64::INFINITY;
    }

    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        let mut s = state.borrow_mut();
        s.sigma -= e;

        if !x.i_q_error.is_empty() {
            if let Some(q_error) = x.i_q_error.get_values().first().copied() {
                s.q_error_history.push(q_error);
                let values = [q_error.0.i, q_error.0.j, q_error.0.k];
                s.q_error_range = TransducerState::update_range(s.q_error_range, &values);
            }
        }
        if !x.i_w.is_empty() {
            if let Some(w) = x.i_w.get_values().first().copied() {
                s.w_history.push(w);
                let values = [w.0.x, w.0.y, w.0.z];
                s.w_history_range = TransducerState::update_range(s.w_history_range, &values);
            }
        }
        if !x.i_rw_speeds.is_empty() {
            if let Some(rw_speeds) = x.i_rw_speeds.get_values().first().copied() {
                s.rw_speeds_history.push(rw_speeds);
                let values = [rw_speeds.0.x, rw_speeds.0.y, rw_speeds.0.z];
                s.rw_speeds_history_range = TransducerState::update_range(s.rw_speeds_history_range, &values);
            }
        }
    }

    fn lambda(_state: &Self::State, _output: &mut Self::Output) {
        
    }

    fn ta(state: &Self::State) -> f64 {
        state.borrow_mut().sigma
    }
}