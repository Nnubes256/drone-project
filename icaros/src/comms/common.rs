use serde::Deserialize;
use serde::Serialize;

use crate::utils::quartenion::Quartenion;
use crate::utils::vector::Point3;

pub fn get_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(32);
    config
}

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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    orientation: Quartenion,
}

impl Orientation {
    pub fn from_quartenion(q: Quartenion) -> Self {
        Orientation { orientation: q }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDParameters {
    p: f32,
    i: f32,
    d: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDAxes {
    roll: PIDParameters,
    pitch: PIDParameters,
    yaw: PIDParameters,
}
