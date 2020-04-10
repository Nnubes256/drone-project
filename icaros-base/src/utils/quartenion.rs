//! A quartenion, used to represent 3D rotation.

use serde::{Deserialize, Serialize};

///
/// A quartenion, used to represent 3D rotation.
///
/// References: http://www.chrobotics.com/library/understanding-quaternions
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Quartenion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quartenion {
    // Create a quartenion from the given floating-point values.
    pub fn from_values(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quartenion { w, x, y, z }
    }
}
