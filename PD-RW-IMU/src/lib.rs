#![no_std]

pub mod controller;
pub mod imu_sw;
pub mod rw;
pub mod ccu;
pub mod dpc;
pub mod types;

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
    component
};

component! {
    ident = DiscreteTimeModel,
    input = {
        i_q<Quaternion>,
        i_w<Vec3>,
        i_q_target<Quaternion>,
        i_kp<f64>,
        i_kd<f64>,
        i_pwm<Vec3>,
    },
    output = {
        o_q_error<Quaternion>,
        o_pwm<Vec3>,
        o_torque<Vec3>,
    },
    components = {
        controller: crate::controller::Controller,
        rw: crate::rw::RW,
        ccu: crate::ccu::CCU,
        imu_sw: crate::imu_sw::IMUSW,
        dpc: crate::dpc::DPC,
    },
    couplings = {
        controller.o_torque -> rw.i_torque,
        controller.o_torque -> imu_sw.i_torque,
        controller.o_torque -> dpc.i_torque,
        controller.o_torque -> o_torque,
        rw.o_h_rw -> imu_sw.i_h_rw,

        imu_sw.o_w -> ccu.i_w_sw,
        imu_sw.o_q -> ccu.i_q_sw,
        i_q -> ccu.i_q_hw,
        i_w -> ccu.i_w_hw,

        ccu.o_w -> controller.i_w,
        ccu.o_q -> controller.i_q,
        i_q_target -> controller.i_q_target,
        i_kp -> controller.i_kp,
        i_kd -> controller.i_kd,
        i_pwm -> dpc.i_pwm,

        controller.o_qerror -> o_q_error,
        dpc.o_pwm -> o_pwm,

    }
}

pub fn common_logic(h: f64) -> DiscreteTimeModel {
    let time = 0.;

    // Target quaternion (identity orientation)
    //let q_target = Quaternion::default();

    // Rotation of 90° around the z-axis
    /* let angle = -core::f64::consts::FRAC_PI_2; // 90°
    let axis = Vector3::new(0.0, 0.0, 1.0);

    let w = cos(angle / 2.0);
    let v = axis * sin(angle / 2.0);

    let q_target = Quaternion(nalgebra::Quaternion::new(w, v.x, v.y, v.z)); */

    // Rotación de -180° alrededor del eje Z
    let angle = -core::f64::consts::PI; // -180°
    let axis = Vector3::new(0.0, 0.0, 1.0);

    let w = cos(angle / 2.0);
    let v = axis * sin(angle / 2.0);

    let q_target = Quaternion(nalgebra::Quaternion::new(w, v.x, v.y, v.z));

    // Proportional gain
    //let kp = 0.01;
    let kp = 0.1;
    // Derivative gain
    //let kd = 0.1;
    let kd = -0.5;
    // Maximum torque of each reaction wheel [Nm]
    let max_torque_rw = 0.001;


    // http://en.cdmmotor.com/
    // Initial conditions for the reaction wheels and satellite
    let rw_speeds_initial = Vec3(Vector3::new(0.0, 0.0, 0.0));
    // Inertia of each reaction wheel
    let i_rw = Matrix3::from_diagonal(&Vector3::new(5.0e-5, 5.0e-5, 5.0e-5));
    // Maximum angular speed of the reaction wheels [rad/s]
    let max_speed_rw = 20.0;
    let w0 = Vec3(Vector3::new(0.1, -0.1, 0.2));
    //let angle_initial = core::f64::consts::FRAC_PI_4;
    //let axis_initial = Vector3::new(1.0, 1.0, 1.0).normalize();
    //let w: f64 = cos(angle_initial / 2.0);
    //let v = axis_initial * sin(angle_initial / 2.0);
    //let q0 = Quaternion(nalgebra::Quaternion::new(w, v.x, v.y, v.z));
    let q0 = Quaternion::default();

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
    DiscreteTimeModel::new(controller, rw, ccu, imusw, dpc)
}