use serde::Deserialize;
use serde::Serialize;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quartenion {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl Quartenion {
    pub fn from_values(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quartenion { w, x, y, z }
    }
}
