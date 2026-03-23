use nalgebra::{Quaternion as nalgebraQuaternion, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
    pub fn clamp(&self, min: f64, max: f64) -> Self {
        let v = &self.0;
        Vec3(Vector3::new(
            clamp(v.x, min, max),
            clamp(v.y, min, max),
            clamp(v.z, min, max),
        ))
    }

    pub fn default() -> Self {
        Vec3(Vector3::zeros())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion(pub nalgebraQuaternion<f64>);

impl Quaternion {
    pub fn default() -> Self {
        Quaternion(nalgebra::Quaternion::identity())
    }

    // Calculates the error quaternion
    pub fn quaternion_error(q_current: Quaternion, q_target: Quaternion) -> Quaternion {
        Quaternion(q_current.0 * q_target.0.conjugate())
    }

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