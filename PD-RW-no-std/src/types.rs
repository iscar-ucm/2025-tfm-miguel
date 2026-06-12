use nalgebra::{Quaternion as nalgebraQuaternion, Vector3};

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

    /// Normalización del cuaternión.
    pub fn normalize(self) -> Self {
        Quaternion(self.0.normalize())
    }
}

fn clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
