use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Point3<T: Default> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Default> Point3<T> {
    pub fn from_components(x: T, y: T, z: T) -> Self {
        Point3 { x, y, z }
    }
}
