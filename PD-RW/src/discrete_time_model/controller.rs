use crate::discrete_time_model::types::{Quaternion, Vec3};
use xdevs::modeling::*;

/// Este componente implementa un controlador tipo PD en espacio de cuaterniones
/// para estabilización y seguimiento de actitud.
///
/// Recibe:
/// - Velocidad angular (`w`).
/// - Actitud actual (`q`).
///
/// Produce:
/// - Torque de reacción (`torque`).
/// - Error de actitud (`q_error`).
pub struct Controller {
    component: Component,
    /// Entrada de velocidad angular del satélite.
    i_w: InPort<Vec3>,
    /// Entrada de actitud actual (cuaternión).
    i_q: InPort<Quaternion>,
    /// Salida de torque de control.
    o_torque: OutPort<Vec3>,
    /// Salida del error de actitud.
    o_qerror: OutPort<Quaternion>,
    /// Última velocidad angular recibida.
    w: Option<Vec3>,
    /// Última actitud recibida.
    q: Option<Quaternion>,
    /// Torque calculado por el controlador.
    torque: Option<Vec3>,
    /// Error de actitud calculado.
    q_error: Option<Quaternion>,
    /// Tiempo hasta próxima activación del controlador.
    sigma: f64,
    /// Periodo de control.
    time: f64,
    /// Actitud objetivo (referencia).
    q_target: Quaternion,
    /// Ganancia proporcional (PD).
    kp: f64,
    /// Ganancia derivativa (PD).
    kd: f64,
    /// Saturación máxima del torque (ruedas de reacción).
    max_torque_rw: f64,
}

impl Controller {
    /// Crea un nuevo controlador de actitud.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `time` - Periodo de control.
    /// * `q_target` - Actitud objetivo. (referencia)
    /// * `kp` - Ganancia proporcional.
    /// * `kd` - Ganancia derivativa.
    /// * `max_torque_rw` - Saturación máxima del torque.
    pub fn new(
        name: &str,
        time: f64,
        q_target: Quaternion,
        kp: f64,
        kd: f64,
        max_torque_rw: f64,
    ) -> Self {
        let mut component = Component::new(name);
        let i_w = component.add_in_port::<Vec3>("i_w");
        let i_q = component.add_in_port::<Quaternion>("i_q");
        let o_t = component.add_out_port::<Vec3>("o_torque");
        let o_qe = component.add_out_port::<Quaternion>("o_q_error");
        Controller {
            component,
            i_w: i_w,
            i_q: i_q,
            o_torque: o_t,
            o_qerror: o_qe,
            // Initialize the torque command to zero
            w: None,
            q: None,
            torque: None,
            q_error: None,
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            // # q_target is the desired attitude in quaternion form
            q_target: q_target,
            kp,
            kd,
            max_torque_rw: max_torque_rw,
        }
    }

    /// Calcula el error de actitud en cuaterniones.
    ///
    /// # Argumentos
    /// * `q_current` - actitud actual.
    /// * `q_target` - actitud deseada.
    ///
    /// # Returns
    /// Cuaternión de error. (q_error = q_current * conjugado(q_target))
    fn quaternion_error(q_current: Quaternion, q_target: Quaternion) -> Quaternion {
        Quaternion(q_current.0 * q_target.0.conjugate())
    }
}

impl Atomic for Controller {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Envía:
    /// - El torque de control calculado.
    /// - El error de actitud asociado.
    fn lambda(&self) {
        // Send the computed torque command
        if let (Some(q_error), Some(torque)) = (self.q_error, self.torque) {
            unsafe { self.o_qerror.add_value(q_error) };
            unsafe { self.o_torque.add_value(torque) };
        }
    }

    /// Tras emitir la señal de control, el sistema vuelve al estado de espera.
    fn delta_int(&mut self) {
        self.w = None;
        self.q = None;
        self.sigma = f64::INFINITY;
    }

    /// Recibe nuevas medidas del sistema:
    /// - velocidad angular (`w`).
    /// - actitud (`q`).
    ///
    /// Cuando ambos datos están disponibles:
    /// 1. Se calcula el error de actitud.
    /// 2. Se aplica la ley de control PD.
    /// 3. Se satura el torque.
    /// 4. Se programa una salida inmediata.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;
        // Receive new current attitude data
        if !unsafe { self.i_w.is_empty() } {
            self.w = unsafe { self.i_w.get_values().first().copied() };
        }
        if !unsafe { self.i_q.is_empty() } {
            self.q = unsafe { self.i_q.get_values().first().copied() };
        }

        if !self.w.is_none() && !self.q.is_none() {
            /*
            1. Calculate attitude error quaternion (q_error = q_current * conjugate(q_target))
            2. Extract error vector (e.g., from the vector part of q_error)
             */

            self.q_error = Some(Controller::quaternion_error(
                self.q.unwrap(),
                self.q_target,
            ));

            // 3. Apply PD control law:
            if let (Some(q_error), Some(w)) = (self.q_error.as_ref(), self.w.as_ref()) {
                // imag() get the vector (x,y,z) (imaginary) part
                self.torque = Some(Vec3(-self.kp * q_error.0.imag() - self.kd * w.0));
            }

            // Saturate the control torque
            if let Some(torque) = &self.torque {
                self.torque = Some(torque.clamp(-self.max_torque_rw, self.max_torque_rw));
            }

            // Schedule an immediate output
            self.sigma = self.time;
        }
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
