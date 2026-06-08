use crate::discrete_time_model::types::Vec3;
use nalgebra::Matrix3;
use xdevs::modeling::*;

/// Este componente representa la dinámica de las ruedas de reacción utilizadas.
///
/// Su función principal es:
/// - Recibir torque de control.
/// - Integrar la dinámica de velocidad de las ruedas.
/// - Actualizar el momento angular generado.
pub struct RW {
    component: Component,
    /// Entrada de torque aplicado sobre las ruedas de reacción.
    i_torque: InPort<Vec3>,
    /// Salida de momento angular de las ruedas.
    o_h_rw: OutPort<Vec3>,
    /// Salida de velocidades angulares de las ruedas.
    o_rw_speeds: OutPort<Vec3>,
    /// Velocidades actuales de las ruedas.
    rw_speeds: Vec3,
    /// Torque recibido desde el controlador.
    torque: Option<Vec3>,
    /// Momento angular actual de las ruedas.
    h_rw: Vec3,
    /// Tiempo hasta la próxima actualización.
    sigma: f64,
    /// Periodo de integración del modelo.
    time: f64,
    /// Inercia de las ruedas de reacción.
    inertia_rw: Matrix3<f64>,
    /// Velocidad máxima permitida de las ruedas.
    max_speed_rw: f64,
    /// Paso de integración (Euler).
    h: f64,
    /// Derivada de la velocidad de las ruedas (aceleración angular).
    rw_speeds_dot: Vec3,
}

impl RW {
    /// Crea un nuevo modelo de ruedas de reacción.
    ///
    /// # Argumentos
    /// * `name` - Nombre del componente.
    /// * `time` - Periodo de actualización del sistema.
    /// * `rw_speeds_initial` - Velocidad inicial de las ruedas.
    /// * `i_rw` - Matriz de inercia de las ruedas.
    /// * `m_speed_rw` - Velocidad máxima permitida.
    /// * `h` - Paso de integración numérica.
    pub fn new(
        name: &str,
        time: f64,
        rw_speeds_initial: Vec3,
        i_rw: Matrix3<f64>,
        m_speed_rw: f64,
        h: f64,
    ) -> Self {
        let mut component = Component::new(name);
        let i_t = component.add_in_port::<Vec3>("i_torque");
        let o_h = component.add_out_port::<Vec3>("o_h_rw");
        let o_rw = component.add_out_port::<Vec3>("o_rw_speeds");
        RW {
            component: component,
            i_torque: i_t,
            o_h_rw: o_h,
            o_rw_speeds: o_rw,
            rw_speeds: rw_speeds_initial,
            torque: None,
            // Initial reaction wheel angular momentum
            h_rw: Vec3(i_rw * rw_speeds_initial.0),
            // Transition to Waiting state
            sigma: f64::INFINITY,
            time: time,
            inertia_rw: i_rw,
            max_speed_rw: m_speed_rw,
            h: h,
            rw_speeds_dot: Vec3::default(),
        }
    }
    /// Calcula la derivada de la velocidad de las ruedas.
    fn compute_derivatives(&mut self) {
        if let Some(torque) = &self.torque {
            if let Some(inertia_inv) = self.inertia_rw.try_inverse() {
                self.rw_speeds_dot = Vec3(inertia_inv * -&torque.0);
            }
        }
    }

    /// Integra el estado de las ruedas (Euler explícito).
    ///
    /// Actualiza:
    /// - Velocidades de las ruedas.
    /// - Saturación de velocidad.
    /// - Momento angular.
    fn compute_next_state(&mut self, h: f64) {
        // Compute the next state:
        // Calculate the next state of the reaction wheels (Euler integration)
        self.rw_speeds = Vec3(self.rw_speeds.0 + h * self.rw_speeds_dot.0);

        // Limit the speed of the reaction wheels
        // At the moment, I am not applying this limit
        self.rw_speeds = self.rw_speeds.clamp(-self.max_speed_rw, self.max_speed_rw);

        // Update the wheel momentum
        self.h_rw = Vec3(self.inertia_rw * self.rw_speeds.0);
    }
}

impl Atomic for RW {
    fn get_component(&self) -> &Component {
        &self.component
    }

    fn get_component_mut(&mut self) -> &mut Component {
        &mut self.component
    }

    /// Envía:
    /// - Momento angular de las ruedas.
    /// - Velocidades actuales.
    fn lambda(&self) {
        unsafe { self.o_h_rw.add_value(self.h_rw) };
        unsafe { self.o_rw_speeds.add_value(self.rw_speeds) };
    }

    /// Actualiza el estado físico del sistema tras la integración.
    fn delta_int(&mut self) {
        self.compute_next_state(self.h);
        self.sigma = f64::INFINITY
    }

    /// Recibe torque desde el controlador y actualiza la dinámica.
    fn delta_ext(&mut self, e: f64) {
        self.sigma -= e;

        self.torque = unsafe { self.i_torque.get_values().first().copied() };
        // With the new torque, we can compute the derivatives
        self.compute_derivatives();
        self.sigma = self.time;
    }

    fn ta(&self) -> f64 {
        self.sigma
    }
}
