//! A 3-dimensional vector

use serde::{Deserialize, Serialize};

///
/// A 3-dimensional vector of type T
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Point3<T: Default> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Default> Point3<T> {
    /// Creates a 3-dimensional vector from the given components
    pub fn from_components(x: T, y: T, z: T) -> Self {
        Point3 { x, y, z }
    }
}
