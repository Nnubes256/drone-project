//! Base data structures

use core::convert::TryFrom;
use crate::comms::common::{MotorSpeed, Orientation, Acceleration};
use crate::comms::air::G2AControllerAxisState;
use crate::utils::{Quartenion, Point3};
use serde::{Deserialize, Serialize};

///
/// Represents a possibly partial set of data extracted from the GPS
#[derive(Default, Serialize, Clone)]
pub struct GPSData {
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub altitude: Option<f32>,
}

impl From<FullGPSData> for GPSData {
    fn from(value: FullGPSData) -> Self {
        GPSData {
            latitude: Some(value.latitude),
            longitude: Some(value.longitude),
            altitude: Some(value.altitude)
        }
    }
}


///
/// Represents a full set of data extracted from the GPS
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FullGPSData {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f32
}

impl TryFrom<GPSData> for FullGPSData {
    type Error = ();
    fn try_from(value: GPSData) -> Result<Self, Self::Error> {
        if value.latitude.is_none() ||
            value.longitude.is_none() ||
            value.altitude.is_none()
        {
            Err(())
        } else {
            Ok(FullGPSData {
                latitude: value.latitude.unwrap(),
                longitude: value.longitude.unwrap(),
                altitude: value.altitude.unwrap()
            })
        }
    }
}

///
/// Represents the system state on the drone's side.
///
/// This includes the motor speeds, the drone's orientationa and acceleration, and its GPS data.
#[derive(Default, Serialize)]
pub struct DroneState {
    pub motor_speeds: MotorSpeed,
    #[serde(flatten)] pub orientation: Orientation,
    #[serde(flatten)] pub acceleration: Acceleration,
    pub gps: GPSData
}

impl DroneState {
    pub fn new() -> Self {
        DroneState {
            motor_speeds: MotorSpeed::from_speeds((0, 0, 0, 0)),
            orientation: Orientation::from_quartenion(Quartenion::from_values(0.0, 0.0, 0.0, 0.0)),
            acceleration: Acceleration::from_vec3(Point3::from_components(0.0, 0.0, 0.0)),
            gps: GPSData::default()
        }
    }
}

///
/// Represents the system state on the ground control's side (i.e. the gamepad inputs)
#[derive(Serialize)]
pub struct GroundControlState {
    pub controller_state: G2AControllerAxisState,
}

impl Default for GroundControlState {
    fn default() -> Self {
        GroundControlState::new()
    }
}

impl GroundControlState {
    pub fn new() -> Self {
        GroundControlState {
            controller_state: G2AControllerAxisState::new(0, 0, 0, 0),
        }
    }
}
