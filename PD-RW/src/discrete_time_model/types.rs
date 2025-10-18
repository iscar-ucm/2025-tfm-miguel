use nalgebra::{Quaternion as nalgebraQuaternion, Vector3};
use std::str::FromStr;

#[derive(Debug, Clone)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
    pub fn clamp(&self, min: f64, max: f64) -> Self {
        let v = &self.0;
        Vec3(Vector3::new(
            v.x.clamp(min, max),
            v.y.clamp(min, max),
            v.z.clamp(min, max),
        ))
    }
}

impl ToString for Vec3 {
    fn to_string(&self) -> String {
        let v = &self.0;
        format!("({},{},{})", v[0], v[1], v[2])
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseVec3Error;
pub struct ParseQuaternionError;

impl FromStr for Vec3 {
    type Err = ParseVec3Error;

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

#[derive(Debug, Clone)]
pub struct Quaternion(pub nalgebraQuaternion<f64>);

impl ToString for Quaternion {
    fn to_string(&self) -> String {
        let v = self.0.coords;
        format!("(w: {}, x: {}, y: {}, z: {})", v.w, v.x, v.y, v.z)
    }
}

impl FromStr for Quaternion {
    type Err = ParseQuaternionError;

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

impl From<nalgebraQuaternion<f64>> for Quaternion {
    fn from(q: nalgebraQuaternion<f64>) -> Self {
        Quaternion(q)
    }
}
