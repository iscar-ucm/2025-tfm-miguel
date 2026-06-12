use crate::discrete_time_model::types::{Quaternion, Vec3};
use nalgebra::Matrix3;
use xdevs::modeling::*;

/// Este componente modela la evolución dinámica del cuerpo rígido del satélite:
/// - Velocidad angular (`w`).
/// - Actitud en cuaterniones (`q`).
/// - Influencia de ruedas de reacción (`h_rw`).
/// - Torque externo (`torque`).
///
/// Implementa ecuaciones de dinámica rotacional y cinemática de cuaterniones.
pub struct SatelliteDynamics {
    component: Component,
    /// Entrada del momento angular de las ruedas de reacción.
    i_h_rw: InPort<Vec3>,
    /// Entrada del torque externo aplicado al satélite.
    i_torque: InPort<Vec3>,
    /// Salida de velocidad angular del satélite.
    o_w: OutPort<Vec3>,
    /// Salida de actitud del satélite (cuaternión).
    o_q: OutPort<Quaternion>,
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

impl SatelliteDynamics {
    /// Crea un nuevo modelo dinámico del satélite.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `time` - Periodo de simulación.
    /// * `w0` - Velocidad angular inicial.
    /// * `q0` - Actitud inicial (cuaternión).
    /// * `h` - Paso de integración numérica.
    /// * `i_sat` - Matriz de inercia del satélite.
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
            if let Some(torque) = &self.torque {
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

impl Atomic for SatelliteDynamics {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Envía:
    /// - Actitud actual.
    /// - Velocidad angular.
    fn lambda(&self) {
        // Send the current attitude and angular velocity
        unsafe { self.o_q.add_value(self.q) };
        unsafe { self.o_w.add_value(self.w) };
    }

    /// Evolución de la dinámica.
    fn delta_int(&mut self) {
        // Compute the next state if possible
        if !self.h_rw.is_none() && !self.torque.is_none() {
            self.compute_derivatives();
            self.compute_next_state(self.h);
        }
        // Schedule the next output
        self.sigma = self.h
    }

    /// Recepción de nuevas entradas.
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
