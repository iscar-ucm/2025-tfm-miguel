use nalgebra::{Quaternion as nalgebraQuaternion, Vector3};
use std::str::FromStr;

/// Vector tridimensional usado en dinámica orbital y control de actitud.
///
/// Representa magnitudes físicas como:
/// - Velocidad angular
/// - Torque
/// - Momento angular
#[derive(Debug, Clone, Copy)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
    /// Limita cada componente del vector entre un mínimo y un máximo.
    pub fn clamp(&self, min: f64, max: f64) -> Self {
        let v = &self.0;
        Vec3(Vector3::new(
            v.x.clamp(min, max),
            v.y.clamp(min, max),
            v.z.clamp(min, max),
        ))
    }

    /// Devuelve un vector nulo (0,0,0).
    pub fn default() -> Self {
        Vec3(Vector3::zeros())
    }
}

impl ToString for Vec3 {
    fn to_string(&self) -> String {
        let v = &self.0;
        format!("({},{},{})", v[0], v[1], v[2])
    }
}

/// Error de parseo para `Vec3`.
#[derive(Debug, PartialEq, Eq)]
pub struct ParseVec3Error;
/// Error de parseo para cuaterniones.
pub struct ParseQuaternionError;

impl FromStr for Vec3 {
    type Err = ParseVec3Error;

     /// Convierte un string "(x,y,z)" a un Vec3.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim().trim_start_matches('(').trim_end_matches(')');
        let parts: Vec<_> = s.split(',').collect();
        if parts.len() != 3 {
            return Err(ParseVec3Error);
        }
        let x = parts[0].parse::<f64>().map_err(|_| ParseVec3Error)?;
        let y = parts[1].parse::<f64>().map_err(|_| ParseVec3Error)?;
        let z = parts[2].parse::<f64>().map_err(|_| ParseVec3Error)?;
        Ok(Vec3(Vector3::new(x, y, z)))
    }
}

/// Cuaternión usado para representar actitud del satélite.
///
/// Se basa en la librería `nalgebra` para cálculos matemáticos.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion(pub nalgebraQuaternion<f64>);

impl Quaternion{
    /// Cuaternión identidad (sin rotación).
    pub fn default() -> Self {
        Quaternion(nalgebra::Quaternion::identity())
    }

    /// Normalización del cuaternión.
    pub fn normalize(self) -> Self {
        Quaternion(self.0.normalize())
    }
}

impl ToString for Quaternion {
    fn to_string(&self) -> String {
        let v = self.0.coords;
        format!("(w: {}, x: {}, y: {}, z: {})", v.w, v.x, v.y, v.z)
    }
}

impl FromStr for Quaternion {
    type Err = ParseQuaternionError;

    /// Convierte un string "(w,x,y,z)" a Quaternion.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim().trim_start_matches('(').trim_end_matches(')');
        let parts: Vec<_> = s.split(',').collect();
        if parts.len() != 4 {
            return Err(ParseQuaternionError);
        }
        let w = parts[0].parse::<f64>().map_err(|_| ParseQuaternionError)?;
        let x = parts[1].parse::<f64>().map_err(|_| ParseQuaternionError)?;
        let y = parts[2].parse::<f64>().map_err(|_| ParseQuaternionError)?;
        let z = parts[3].parse::<f64>().map_err(|_| ParseQuaternionError)?;
        let q = nalgebra::Quaternion::new(w, x, y, z);
        Ok(Quaternion(q))
    }
}