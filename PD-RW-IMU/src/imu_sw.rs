use crate::types::{Quaternion, Vec3};
use nalgebra::Matrix3;
use xdevs::*;

/// Estado dinámico del satélite completo (cuerpo rígido + ruedas de reacción).
///
/// Modelo:
/// - Dinámica rotacional 3D del cuerpo rígido.
/// - Cinemática de cuaterniones.
/// - Acoplamiento con momentum de reaction wheels.
pub struct IMUSWState {
    /// Tiempo hasta próxima integración.
    sigma: f64,
    /// Periodo de simulación.
    _time: f64,
    /// Velocidad angular actual.
    w: Vec3,
    /// Actitud actual.
    q: Quaternion,
    /// Momento angular de ruedas (entrada externa).
    h_rw: Option<Vec3>,
    /// Torque aplicado.
    torque: Option<Vec3>,
    /// Paso de integración.
    h: f64,
    /// Inercia del satélite.
    i_sat: Matrix3<f64>,
    /// Derivada de velocidad angular.
    wdot: Vec3,
    /// Derivada del cuaternión.
    qdot: Quaternion,
}

impl IMUSWState {
    /// Crea un nuevo modelo dinámico del satélite.
    ///
    /// # Argumentos
    /// * `time` - Periodo de simulación.
    /// * `w0` - Velocidad angular inicial.
    /// * `q0` - Actitud inicial (cuaternión).
    /// * `h` - Paso de integración numérica.
    /// * `i_sat` - Matriz de inercia del satélite.
    pub fn new(time: f64, w0: Vec3, q0: Quaternion, h: f64, i_sat: Matrix3<f64>) -> Self {
        Self {
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

    /// Calcula las derivadas del sistema dinámico.
    ///
    /// Incluye:
    /// - Dinámica rotacional rígida
    /// - Cinemática de cuaterniones
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
            if let Some(torque) = self.torque {
                self.wdot = Vec3(i_inv * (torque.0 - w_skew * h_total));
            }
        }
        self.qdot = Quaternion(0.5 * self.q.0 * omega_q);
    }

    /// Integra el estado del sistema usando Euler explícito.
    fn compute_next_state(&mut self, h: f64) {
        self.w = Vec3(self.w.0 + h * self.wdot.0); // using a simple Euler integration
        self.q = Quaternion(self.q.0 + h * self.qdot.0).normalize(); //using a simple Euler integration
    }
}

component! {
    ident = IMUSW,
    input = {
        i_h_rw<Vec3>,
        i_torque<Vec3>,
    },
    output = {
        o_w<Vec3>,
        o_q<Quaternion>,
    },
    state = IMUSWState
}

impl Atomic for IMUSW {
    /// Evolución de la dinámica.
    fn delta_int(state: &mut Self::State) {
        // Compute the next state if possible
        if !state.h_rw.is_none() && !state.torque.is_none() {
            state.compute_derivatives();
            state.compute_next_state(state.h);
        }
        // Schedule the next output
        state.sigma = state.h
    }

    /// Recepción de nuevas entradas.
    fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
        state.sigma -= e;
        // An external event is a new h_rw or torque command
        if !x.i_h_rw.is_empty() {
            state.h_rw = x.i_h_rw.get_values().first().copied();
        }
        if !x.i_torque.is_empty() {
            state.torque = x.i_torque.get_values().first().copied();
        }
    }

    /// Envía:
    /// * Actitud actual.
    /// * Velocidad angular.
    fn lambda(state: &Self::State, output: &mut Self::Output) {
        // Send the current attitude and angular velocity
        output.o_q.add_value(state.q).unwrap();
        output.o_w.add_value(state.w).unwrap();
    }

    fn ta(state: &Self::State) -> f64 {
        state.sigma
    }
}