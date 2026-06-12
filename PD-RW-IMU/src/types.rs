use nalgebra::{Quaternion as nalgebraQuaternion, Vector3, UnitQuaternion};
use libm::atan2;
use core::f64::consts::PI;

/// Wrapper simple para un vector 3D (f64).
///
/// Se usa para:
/// * Velocidad angular (ω).
/// * Torque (τ).
/// * Momento angular (h).
#[derive(Debug, Clone, Copy)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
    /// Limita cada componente del vector entre un mínimo y un máximo.
    pub fn clamp(&self, min: f64, max: f64) -> Self {
        let v = &self.0;
        Vec3(Vector3::new(
            clamp(v.x, min, max),
            clamp(v.y, min, max),
            clamp(v.z, min, max),
        ))
    }

    /// Devuelve un vector nulo (0,0,0).
    pub fn default() -> Self {
        Vec3(Vector3::zeros())
    }
}

/// Wrapper para cuaterniones de actitud.
///
/// Representa orientación del satélite en 3D.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion(pub nalgebraQuaternion<f64>);

impl Quaternion {
    /// Cuaternión identidad (sin rotación).
    pub fn default() -> Self {
        Quaternion(nalgebra::Quaternion::identity())
    }

    /// Calcula el cuaternión de error
    ///
    /// # Argumentos
    /// * `q_current` - actitud actual.
    /// * `q_target` - actitud deseada.
    pub fn quaternion_error(q_current: Quaternion, q_target: Quaternion) -> Quaternion {
        Quaternion(q_current.0 * q_target.0.conjugate())
    }

    /// Normalización del cuaternión.
    pub fn normalize(self) -> Self {
        Quaternion(self.0.normalize())
    }

    /// Obtiene el yaw a partir de un cuaternión
    pub fn yaw_from_quaternion(self) -> f64 {
        let siny_cosp = 2.0 * (self.0.w * self.0.k + self.0.i * self.0.j) ;
        let cosy_cosp = 1.0 - 2.0 * (self.0.j * self.0.j + self.0.k * self.0.k);
        let yaw_rad = atan2(siny_cosp, cosy_cosp);
        yaw_rad * (180.0 / PI)
    }

    /// Obtiene el cuaternión a partir del yaw
    ///
    /// # Argumentos
    /// * `yaw_deg` - yaw en grados.
    pub fn from_yaw_deg(yaw_deg: f64) -> Self {
        let yaw_rad = yaw_deg.to_radians();

        let uq = UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            yaw_rad,
        );

        Quaternion(uq.into_inner())
    }
}

/// Limitar el valor con un máximo y un mínimo
///
/// # Argumentos
/// * `value` - valor a limitar.
/// * `max` - valor máximo que se puede obtener.
/// * `min` - valor mínimo que se puede obtener.
fn clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Wrapper para los datos de entrada del simulador xdevs no-std
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub w: Vec3,
    pub q: Quaternion
}

#[derive(Debug, Clone, Copy)]
pub struct KpKdSample {
    pub kp: f64,
    pub kd: f64
}