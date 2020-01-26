use serde::Deserialize;
use serde::Serialize;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point3<T> {
    x: T,
    y: T,
    z: T,
}

impl<T> Point3<T> {
    pub fn from_components(x: T, y: T, z: T) -> Self {
        Point3 { x, y, z }
    }
}
