mod controller;
mod plotters;
mod rw;
mod satellite_dynamics;
mod transducer;
mod types;

use crate::{
    controller::{Controller, ControllerState},
    rw::{RW, RWState},
    satellite_dynamics::{SatelliteDynamics, SatelliteDynamicsState},
    transducer::{SharedTransducerState, Transducer, TransducerState},
    types::{Quaternion, Vec3},
};
use libm::{cos, sin};
use nalgebra::{Matrix3, Vector3};
use std::{cell::RefCell, rc::Rc};
use xdevs::{
    component,
    simulator::{Config, Simulator},
};

component! {
    ident = DiscreteTimeModel,
    components = {
        controller: controller::Controller,
        rw: rw::RW,
        transducer: transducer::Transducer,
        satellite_dynamics: satellite_dynamics::SatelliteDynamics
    },
    couplings = {
        controller.o_torque -> rw.i_torque,
        controller.o_torque -> satellite_dynamics.i_torque,
        controller.o_qerror -> transducer.i_q_error,

        rw.o_h_rw -> satellite_dynamics.i_h_rw,
        rw.o_rw_speeds -> transducer.i_rw_speeds,

        satellite_dynamics.o_w -> controller.i_w,
        satellite_dynamics.o_q -> controller.i_q,
        satellite_dynamics.o_w -> transducer.i_w,
    }
}

fn main() {
    let total_time = 100.0;
    let h = 0.01;
    let time = 0.;
    let margin_ratio = 0.1;
    // Target quaternion (identity orientation)
    let q_target = Quaternion::default();

    // Proportional gain
    let kp = 0.01;
    // Derivative gain
    let kd = 0.1;
    // Maximum torque of each reaction wheel [Nm]
    let max_torque_rw = 0.001;

    // Initial conditions for the reaction wheels and satellite
    let rw_speeds_initial = Vec3(Vector3::new(0.0, 0.0, 0.0));
    // Inertia of each reaction wheel
    let i_rw = Matrix3::from_diagonal(&Vector3::new(5.0e-5, 5.0e-5, 5.0e-5));
    // Maximum angular speed of the reaction wheels [rad/s]
    let max_speed_rw = 20.0;
    let w0 = Vec3(Vector3::new(0.1, -0.1, 0.2));
    let angle_initial = core::f64::consts::FRAC_PI_4;
    let axis_initial = Vector3::new(1.0, 1.0, 1.0).normalize();
    let w = cos(angle_initial / 2.0);
    let v = axis_initial * sin(angle_initial / 2.0);
    let q0 = Quaternion(nalgebra::Quaternion::new(w, v.x, v.y, v.z));

    /*
    Nanosatellite Parameters (1U CubeSat)
    Assuming a uniform mass distribution for a 1U CubeSat of 1.33 kg and 10cm side length
    I = M * d^2 / 6
    */
    let i_sat = Matrix3::from_diagonal_element(1.33 * 0.1 * 0.1 / 6.0);

    let controller = Controller::new(ControllerState::new(time, q_target, kp, kd, max_torque_rw));
    let rw = RW::new(RWState::new(time, rw_speeds_initial, i_rw, max_speed_rw, h));
    let sd = SatelliteDynamics::new(SatelliteDynamicsState::new(time, w0, q0, h, i_sat));
    let shared_state: SharedTransducerState =
        Rc::new(RefCell::new(TransducerState::new(margin_ratio)));
    let transducer = Transducer::new(shared_state.clone());
    let discrete_time_model = DiscreteTimeModel::new(controller, rw, transducer, sd);

    let mut simulator = Simulator::new(discrete_time_model);

    let config = Config::new(0.0, total_time, h, None);
    simulator.simulate_rt(&config, xdevs::simulator::std::sleep(&config), |_| {});

    plotters::draw(shared_state, total_time);
}
