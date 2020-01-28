use serde::Deserialize;
use serde::Serialize;

use crate::utils::quartenion::Quartenion;
use crate::utils::vector::Point3;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorSpeed {
    tl: u16,
    tr: u16,
    bl: u16,
    br: u16,
}

impl MotorSpeed {
    pub fn from_speeds(speeds: (u16, u16, u16, u16)) -> Self {
        MotorSpeed {
            tl: speeds.0,
            tr: speeds.1,
            bl: speeds.2,
            br: speeds.3,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Acceleration {
    vector: Point3<f32>,
}

impl Acceleration {
    pub fn from_vec3(v: Point3<f32>) -> Self {
        Acceleration { vector: v }
    }

    pub fn as_point3(&self) -> &Point3<f32> {
        &self.vector
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    orientation: Quartenion,
}

impl Orientation {
    pub fn from_quartenion(q: Quartenion) -> Self {
        Orientation { orientation: q }
    }

    pub fn as_quartenion(&self) -> &Quartenion {
        &self.orientation
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDParameters {
    p: f32,
    i: f32,
    d: f32,
}

impl PIDParameters {
    pub fn new(p: f32, i: f32, d: f32) -> Self {
        PIDParameters { p, i, d }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDAxes {
    roll: PIDParameters,
    pitch: PIDParameters,
    yaw: PIDParameters,
}

impl PIDAxes {
    pub fn new(roll: PIDParameters, pitch: PIDParameters, yaw: PIDParameters) -> Self {
        PIDAxes { roll, pitch, yaw }
    }
}
