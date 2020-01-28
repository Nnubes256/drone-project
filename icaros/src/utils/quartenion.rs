use serde::Deserialize;
use serde::Serialize;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quartenion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quartenion {
    pub fn from_values(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quartenion { w, x, y, z }
    }
}
