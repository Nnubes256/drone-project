use icaros_base::utils::map_values;
use icaros_base::comms::air::G2AControllerAxisState;
use gilrs::ev::Code;
use gilrs::Gamepad;
use serde::{Serialize, Deserialize};
use gilrs::GamepadId;
use std::error::Error;
use gilrs::{Gilrs, MappingSource, Axis, Button};
use gilrs::ev::filter::{Filter, deadzone};

#[derive(Serialize, Deserialize, Clone)]
#[serde(tag = "throttle_lock")]
pub enum ThrottleLockState {
    #[serde(rename = "unlocked")]
    Unlocked,
    #[serde(rename = "locking")]
    Locking { value: f32 },
    #[serde(rename = "locked")]
    Locked { value: f32 }
}

impl Default for ThrottleLockState {
    fn default() -> Self {
        ThrottleLockState::Unlocked
    }
}

impl ThrottleLockState {
    pub fn lock(&mut self, value: f32) {
        *self = match *self {
            Self::Unlocked => Self::Locking { value },
            _ => return
        };
    }

    pub fn lock_finish(&mut self) {
        *self = match *self {
            Self::Locking { value } => Self::Locked { value },
            _ => return
        };
    }

    pub fn unlock(&mut self) {
        *self = Self::Unlocked;
    }
}

#[derive(Default, Clone, Serialize, Deserialize)]
pub struct RawAxisState {
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32
}

impl RawAxisState {
    pub fn get_roll(&self) -> f32 {
        self.roll
    }

    pub fn get_pitch(&self) -> f32 {
        self.pitch
    }

    pub fn get_yaw(&self) -> f32 {
        self.yaw
    }

    pub fn get_throttle(&self) -> f32 {
        self.throttle
    }

    pub fn set_roll(&mut self, value: f32) {
        self.roll = f32::min(f32::max(value, -1.0), 1.0);
    }

    pub fn set_pitch(&mut self, value: f32) {
        self.pitch = f32::min(f32::max(value, -1.0), 1.0);
    }

    pub fn set_yaw(&mut self, value: f32) {
        self.yaw = f32::min(f32::max(value, -1.0), 1.0);
    }

    pub fn set_throttle(&mut self, value: f32) {
        self.throttle = f32::min(f32::max(value, -1.0), 1.0);
    }
}

#[derive(Default, Clone, Serialize, Deserialize)]
pub struct ControlState {
    axis_state: RawAxisState,
    throttle_lock: ThrottleLockState
}

impl ControlState {
    pub fn axes_ref(&self) -> &RawAxisState {
        &self.axis_state
    }

    pub fn axes_mut(&mut self) -> &mut RawAxisState {
        &mut self.axis_state
    }

    pub fn start_throttle_lock(&mut self) -> &ThrottleLockState {
        self.throttle_lock.lock(self.axis_state.get_throttle());
        &self.throttle_lock
    }

    pub fn stop_throttle_lock(&mut self) -> &ThrottleLockState {
        self.throttle_lock.unlock();
        &self.throttle_lock
    }
}

pub trait GamepadService {
    fn is_controller_available(&mut self) -> bool;
    fn process(&mut self, state: &mut ControlState) -> Result<bool, Box<dyn Error>>;
    fn control_state_into_message_state(&mut self, msg_state: &mut G2AControllerAxisState, ctrl_state: &ControlState);
}

struct PhysicalGamepadAxesCodes {
    pub left_stick_x: Code,
    pub left_stick_y: Code,
    pub right_stick_x: Code,
    pub right_stick_y: Code,
    pub west_button: Code,
}

impl PhysicalGamepadAxesCodes {
    pub fn new(left_stick_x: Code, left_stick_y: Code, right_stick_x: Code, right_stick_y: Code, west_button: Code) -> Self
    {
        PhysicalGamepadAxesCodes { left_stick_x, left_stick_y, right_stick_x, right_stick_y, west_button }
    }
}

pub struct PhysicalGamepadService {
    driver: Gilrs,
    current_gamepad: Option<GamepadId>,
    axes_codes: Option<PhysicalGamepadAxesCodes>
}

macro_rules! deadzone {
    ($min:expr, $value:expr, $max:expr) => {
        if $value < $max && $value > $min {
            0.0
        } else {
            $value
        }
    }
}

impl GamepadService for PhysicalGamepadService {
    fn is_controller_available(&mut self) -> bool {
        if let Some(id) = self.current_gamepad {
            self.driver.gamepad(id).is_connected()
        } else {
            false
        }
    }

    fn process(&mut self, state: &mut ControlState) -> Result<bool, Box<dyn Error>> {
        while let Some(_) = self.driver.next_event().filter_ev(&deadzone, &mut self.driver) {};

        if self.current_gamepad.is_none() {
            if let Some(id) = self.search_for_gamepad() {
                self.current_gamepad = Some(id);
            }
        }

        if let Some(id) = self.current_gamepad {
            let gpad: Gamepad = self.driver.gamepad(id);

            if !gpad.is_connected() {
                return Ok(false);
            }

            let gpad_state = gpad.state();

            let axes_codes = match &self.axes_codes {
                Some(axes) => &axes,
                None => {
                    self.axes_codes = Some(PhysicalGamepadAxesCodes::new(
                        gpad.axis_code(Axis::LeftStickX).unwrap(),
                        gpad.axis_code(Axis::LeftStickY).unwrap(),
                        gpad.axis_code(Axis::RightStickX).unwrap(),
                        gpad.axis_code(Axis::RightStickY).unwrap(),
                        gpad.button_code(Button::West).unwrap()
                    ));
                    self.axes_codes.as_ref().unwrap()
                }
            };

            let left_stick_x = gpad_state.axis_data(axes_codes.left_stick_x);
            let left_stick_y = gpad_state.axis_data(axes_codes.left_stick_y);
            let right_stick_x = gpad_state.axis_data(axes_codes.right_stick_x);
            let right_stick_y = gpad_state.axis_data(axes_codes.right_stick_y);
            let west_button = gpad_state.button_data(axes_codes.west_button);

            debug!(target: "gamepad", "{:?} {}", left_stick_x, self.driver.counter());

            {
                let axis_control_data = state.axes_mut();

                if let Some(value) = left_stick_x {
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_yaw(deadzone!(-0.01, value.value(), 0.01))
                    }
                }

                if let Some(value) = left_stick_y {
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_throttle(deadzone!(-0.01, value.value(), 0.01))
                    }
                }

                if let Some(value) = right_stick_x {
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_roll(deadzone!(-0.01, value.value(), 0.01))
                    }
                }

                if let Some(value) = right_stick_y {
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_pitch(deadzone!(-0.01, value.value(), 0.01))
                    }
                }
            }

            if let Some(value) = west_button {
                if value.counter() >= self.driver.counter() {
                    let throttle = state.axes_ref().get_throttle();
                    let throttle_lock_pressed = value.is_pressed();
                    if throttle_lock_pressed {
                        match state.throttle_lock {
                            ThrottleLockState::Unlocked if throttle != 0.0 => {
                                state.throttle_lock.lock(throttle);
                            },
                            ThrottleLockState::Unlocked => {},
                            ThrottleLockState::Locking { value: _ } => {
                                state.throttle_lock.unlock();
                            },
                            ThrottleLockState::Locked { value: _ } => {
                                // TODO improve safety
                                state.throttle_lock.unlock();
                            }
                        }
                    }
                }
            }

            if let ThrottleLockState::Locking { value: _ } = state.throttle_lock {
                let axis_control_data = state.axes_ref();
                if axis_control_data.get_throttle() == 0.0 {
                    state.throttle_lock.lock_finish();
                }
            }

            self.driver.inc();

            return Ok(true);
        }

        self.driver.inc();

        return Ok(false);
    }

    fn control_state_into_message_state(&mut self, msg_state: &mut G2AControllerAxisState, ctrl_state: &ControlState) {
        let axes = ctrl_state.axes_ref();

        match ctrl_state.throttle_lock {
            ThrottleLockState::Unlocked => {
                msg_state.throttle = f64::max(0.0, map_values(axes.get_throttle() as f64, (0.0, 1.0), (0.0, 127.0))) as i8;
            },
            ThrottleLockState::Locking { value } => {
                msg_state.throttle = f64::max(0.0, map_values(value as f64, (0.0, 1.0), (0.0, 127.0))) as i8;
            },
            ThrottleLockState::Locked { value } => {
                msg_state.throttle =
                f64::max(
                    f64::min(
                        map_values(axes.get_throttle() as f64, (-1.0, 1.0), ((0.0 + value).into(), (127.0 + value).into())),
                    127.0),
                0.0) as i8;
            }
        }

        msg_state.roll = map_values(axes.get_roll() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
        msg_state.pitch = map_values(-axes.get_pitch() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
        msg_state.yaw = map_values(axes.get_yaw() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
    }
}

impl PhysicalGamepadService {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        Ok(PhysicalGamepadService {
            driver: Gilrs::new()?,
            current_gamepad: None,
            axes_codes: None
        })
    }

    pub fn search_for_gamepad(&mut self) -> Option<GamepadId> {
        for (id, gamepad) in self.driver.gamepads() {
            if gamepad.mapping_source() != MappingSource::None {
                info!("Found gamepad: {} - {} ({:?}, ff={}, {:?})", id, gamepad.name(),
                    gamepad.power_info(), gamepad.is_ff_supported(), gamepad.mapping_source());
                return Some(id);
            }

            return None;
        }

        return None;
    }
}
