mod controller;
mod imu_sw;
mod rw;
mod ccu;
mod dpc;
mod types;

use crate::{
    controller::{Controller, ControllerState},
    rw::{RW, RWState},
    imu_sw::{IMUSW, IMUSWState},
    ccu::{CCU, CCUState},
    dpc::{DPC, DPCState},
    types::{Quaternion, Vec3},
};
use libm::{cos, sin};
use nalgebra::{Matrix3, Vector3};

use xdevs::{
    component,
    simulator::{Config, Simulator},
};

component! {
    ident = DiscreteTimeModel,
    input = {
        i_q<Quaternion>,
        i_w<Vec3>,
    },
    output = {
        o_qerror<Quaternion>,
        o_torque<Vec3>,
    },
    components = {
        controller: controller::Controller,
        rw: rw::RW,
        ccu: ccu::CCU,
        imu_sw: imu_sw::IMUSW,
        dpc: dpc::DPC,
    },
    couplings = {
        controller.o_torque -> rw.i_torque,
        controller.o_torque -> imu_sw.i_torque,
        controller.o_torque -> dpc.i_torque,

        rw.o_h_rw -> imu_sw.i_h_rw,

        imu_sw.o_w -> ccu.i_w_sw,
        imu_sw.o_q -> ccu.i_q_sw,
        i_q -> ccu.i_q_hw,
        i_w -> ccu.i_w_hw,

        ccu.o_w -> controller.i_w,
        ccu.o_q -> controller.i_q,

        controller.o_qerror -> o_qerror,
        dpc.o_torque -> o_torque,
/*         controller.o_torque -> rw.i_torque,
        controller.o_torque -> imu_sw.i_torque,

        rw.o_h_rw -> imu_sw.i_h_rw,

        imu_sw.o_w -> controller.i_w,
        imu_sw.o_q -> controller.i_q, */

    }
}

fn main() {
    let total_time = 100.0;
    let h = 0.01;
    let time = 0.;

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
    let imusw = IMUSW::new(IMUSWState::new(time, w0, q0, h, i_sat));
    let ccu = CCU::new(CCUState::new(time, w0, q0));
    let dpc = DPC::new(DPCState::new(time));
    let discrete_time_model = DiscreteTimeModel::new(controller, rw, ccu, imusw, dpc);

    let mut simulator = Simulator::new(discrete_time_model);

    let config = Config::new(0.0, total_time, h, None);
    simulator.simulate_rt(&config, xdevs::simulator::std::sleep(&config),
    |output| {
            if let Some(&job) = output.o_torque.get_values().last() {
                println!("[G] OUTPUT={:?}", job);
            }
            if let Some(&job) = output.o_qerror.get_values().last() {
                println!("[G] OUTPUT={:?}", job);
            }
        });
}