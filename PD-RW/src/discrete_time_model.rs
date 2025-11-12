use crate::discrete_time_model::types::{Quaternion, Vec3};
use nalgebra::{Matrix3, Vector3};
use xdevs::modeling::*;

mod controller;
mod rw;
mod satellite_dynamics;
pub(crate) mod transducer;
pub mod types;

use controller::Controller;
use rw::RW;
use satellite_dynamics::SatelliteDynamics;
use transducer::Transducer;

pub struct DiscreteTimeModel {
    pub(crate) coupled: Coupled,
    pub transducer_ref: *const Transducer,
}

impl DiscreteTimeModel {
    pub fn new(name: &str, h: Option<f64>) -> Self {
        let mut coupled = Coupled::new(name);
        let h = h.unwrap_or(0.01);
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
        let angle_initial = std::f64::consts::FRAC_PI_4;
        let axis_initial = Vector3::new(1.0, 1.0, 1.0).normalize();
        let w = (angle_initial / 2.0).cos();
        let v = axis_initial * (angle_initial / 2.0).sin();
        let q0 = Quaternion(nalgebra::Quaternion::new(w, v.x, v.y, v.z));

        /*
        Nanosatellite Parameters (1U CubeSat)
        Assuming a uniform mass distribution for a 1U CubeSat of 1.33 kg and 10cm side length
        I = M * d^2 / 6
        */
        let i_sat = Matrix3::from_diagonal_element(1.33 * 0.1 * 0.1 / 6.0);

        // Instantiate components
        let controller = Controller::new("Controller", time, q_target, kp, kd, max_torque_rw);
        let rw = RW::new("ReationWheels", time, rw_speeds_initial, i_rw, max_speed_rw, h,);
        let sd = SatelliteDynamics::new("SatelliteDynamics", time, w0, q0, h, i_sat);
        let transducer = Box::new(Transducer::new("Transducer", margin_ratio));
        let transducer_ptr: *const Transducer = &*transducer;

        // Add components to model
        coupled.add_component(Box::new(controller));
        coupled.add_component(Box::new(rw));
        coupled.add_component(Box::new(sd));
        coupled.add_component(transducer);

        // Connect components
        coupled.add_ic("Controller", "o_torque", "ReationWheels", "i_torque");
        coupled.add_ic("Controller", "o_torque", "SatelliteDynamics", "i_torque");
        coupled.add_ic("Controller", "o_q_error", "Transducer", "i_qerror");

        coupled.add_ic("ReationWheels", "o_h_rw", "SatelliteDynamics", "i_h_rw");
        coupled.add_ic("ReationWheels", "o_rw_speeds", "Transducer", "i_rw_speeds");

        coupled.add_ic("SatelliteDynamics", "o_w", "Controller", "i_w");
        coupled.add_ic("SatelliteDynamics", "o_q", "Controller", "i_q");
        coupled.add_ic("SatelliteDynamics", "o_w", "Transducer", "i_w");

        DiscreteTimeModel {
            coupled: coupled,
            transducer_ref: transducer_ptr,
        }
    }
    
}
