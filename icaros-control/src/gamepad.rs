//! Gamepad support

use std::time::Instant;
use icaros_base::utils::map_values;
use icaros_base::comms::air::G2AControllerAxisState;
use gilrs::ev::Code;
use gilrs::Gamepad;
use serde::{Serialize, Deserialize};
use gilrs::GamepadId;
use std::error::Error;
use gilrs::{Gilrs, MappingSource, Axis};
use gilrs::ev::filter::{Filter, deadzone};

/// A constant factor that determines the rate to which the throttle changes
/// when input is provided.
const THROTTLE_CHANGE_SPEED: f32 = 0.25;

/// A constant factor that determines how much a full press of one of the triggers
/// may influence the final throttle, in percentage terms (i.e. `0.1` means it may
/// influence the throttle up or down up to a 10% of the total value range).
const THROTTLE_QUICK_TRIGGER_BOUND: f32 = 0.1;

///
/// Raw processed axis state.
///
/// Serves as the final output of the gamepad logic, but
/// it is formatted as floating-point numbers on the range [-1, 1]
/// (except for the throttle, which works on the range [0, 1]).
#[derive(Default, Clone, Serialize, Deserialize)]
pub struct RawAxisState {
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32
}

impl RawAxisState {
    /// Return the roll, on a range of [-1, 1], where 0 is the neutral position.
    pub fn get_roll(&self) -> f32 {
        self.roll
    }

    /// Return the pitch, on a range of [-1, 1], where 0 is the neutral position.
    pub fn get_pitch(&self) -> f32 {
        self.pitch
    }

    /// Return the yaw, on a range of [-1, 1], where 0 is the neutral position.
    pub fn get_yaw(&self) -> f32 {
        self.yaw
    }

    /// Return the throttle, on a range of [0, 1], where 0 is a minimum throttle.
    pub fn get_throttle(&self) -> f32 {
        self.throttle
    }

    /// Sets the roll, on a range of [-1, 1], where 0 is the neutral position.
    pub fn set_roll(&mut self, value: f32) {
        self.roll = f32::min(f32::max(value, -1.0), 1.0);
    }

    /// Sets the pitch, on a range of [-1, 1], where 0 is the neutral position.
    pub fn set_pitch(&mut self, value: f32) {
        self.pitch = f32::min(f32::max(value, -1.0), 1.0);
    }

    /// Sets the yaw, on a range of [-1, 1], where 0 is the neutral position.
    pub fn set_yaw(&mut self, value: f32) {
        self.yaw = f32::min(f32::max(value, -1.0), 1.0);
    }

    /// Sets the throttle, on a range of [0, 1], where 0 is a minimum throttle.
    pub fn set_throttle(&mut self, value: f32) {
        self.throttle = f32::min(f32::max(value, 0.0), 1.0);
    }

    /// Adds the specified value to the throttle and clamps it to [0, 1].
    pub fn add_throttle(&mut self, value: f32) {
        self.set_throttle(self.throttle + value);
    }
}

///
/// Gamepad state, shared with the main system
#[derive(Default, Clone, Serialize, Deserialize)]
pub struct ControlState {
    axis_state: RawAxisState
}

impl ControlState {
    /// Obtains a reference to the raw axis state
    pub fn axes_ref(&self) -> &RawAxisState {
        &self.axis_state
    }

    /// Obtains a mutable reference to the raw axis state
    pub fn axes_mut(&mut self) -> &mut RawAxisState {
        &mut self.axis_state
    }
}

///
/// A gamepad input service.
///
/// Implementors of this trait can act as services that read user input on the
/// drone's state from a device such as a gamepad and pass it to the main system
/// for routing into the drone and/or appropiate subsystems.
pub trait GamepadService {
    /// Returns whether the controller is available for reading
    fn is_controller_available(&mut self) -> bool;

    /// Read the gamepad inputs and update the given state with them
    fn process(&mut self, state: &mut ControlState) -> Result<bool, Box<dyn Error>>;

    /// Convert the given state into a processed, final control state ready to send to the drone
    fn control_state_into_message_state(&mut self, msg_state: &mut G2AControllerAxisState, ctrl_state: &ControlState);
}

///
/// Holds the driver identifiers for each control (or "codes").
struct PhysicalGamepadAxesCodes {
    pub left_stick_x: Code,
    pub left_stick_y: Code,
    pub right_stick_x: Code,
    pub right_stick_y: Code,
    pub left_trigger: Code,
    pub right_trigger: Code,
}

impl PhysicalGamepadAxesCodes {
    /// Creates a new `PhysicalGamepadAxesCodes` from the collected codes for each control.
    pub fn new(
        left_stick_x: Code, left_stick_y: Code,
        right_stick_x: Code, right_stick_y: Code,
        left_trigger: Code, right_trigger: Code) -> Self
    {
        PhysicalGamepadAxesCodes {
            left_stick_x, left_stick_y, right_stick_x, right_stick_y,
            left_trigger, right_trigger
        }
    }
}

///
/// The physical gamepad input service, which takes inputs from a physical gamepad.
pub struct PhysicalGamepadService {
    driver: Gilrs,
    current_gamepad: Option<GamepadId>,
    axes_codes: Option<PhysicalGamepadAxesCodes>,
    last_timestamp: Instant,
    throttle_modifier: f32,
}

///
/// If the value passed falls within the range specified, returns `0.0`; otherwise returns
/// the value unchanged. Used to handle error margins when the gamepad axis are at the
/// neutral positions due to inexact physical measurement.
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
        // If we have already detected a gamepad before, it depends on whether the
        // gamepad is still connected; otherwise there's no controller available in any case.
        if let Some(id) = self.current_gamepad {
            self.driver.gamepad(id).is_connected()
        } else {
            false
        }
    }

    fn process(&mut self, state: &mut ControlState) -> Result<bool, Box<dyn Error>> {
        // Process all pending input events
        while let Some(_ev) = self.driver.next_event().filter_ev(&deadzone, &mut self.driver) {
            debug!("{:?}", _ev);
        };

        // If we don't have any gamepad, attempt to search for one
        if self.current_gamepad.is_none() {
            if let Some(id) = self.search_for_gamepad() {
                self.current_gamepad = Some(id);
            }
        }

        // If we have one, start processing
        if let Some(id) = self.current_gamepad {
            // Get a hold of it
            let gpad: Gamepad = self.driver.gamepad(id);

            // Check if it is connected
            if !gpad.is_connected() {
                return Ok(false);
            }

            // Obtain the state of its controls
            let gpad_state = gpad.state();

            // Obtain the codes for each control we want to obtain measurements from
            let axes_codes = match &self.axes_codes {
                Some(axes) => &axes, // We already have them
                None => { // We don't have them yet; attempt to obtain them
                    self.axes_codes = Some(PhysicalGamepadAxesCodes::new(
                        gpad.axis_code(Axis::LeftStickX).unwrap(),
                        gpad.axis_code(Axis::LeftStickY).unwrap(),
                        gpad.axis_code(Axis::RightStickX).unwrap(),
                        gpad.axis_code(Axis::RightStickY).unwrap(),
                        gpad.axis_code(Axis::LeftZ).unwrap(),
                        gpad.axis_code(Axis::RightZ).unwrap()
                    ));
                    self.axes_codes.as_ref().unwrap()
                }
            };

            // Obtain the measurements for each control we want to measure
            // using their codes
            let left_stick_x = gpad_state.axis_data(axes_codes.left_stick_x);
            let left_stick_y = gpad_state.axis_data(axes_codes.left_stick_y);
            let right_stick_x = gpad_state.axis_data(axes_codes.right_stick_x);
            let right_stick_y = gpad_state.axis_data(axes_codes.right_stick_y);
            let left_trigger = gpad_state.axis_data(axes_codes.left_trigger);
            let right_trigger = gpad_state.axis_data(axes_codes.right_trigger);

            debug!(target: "gamepad", "{:?} {}", left_stick_x, self.driver.counter());

            // Take the amount of time passed since the last measurement.
            // This is important when measuring how much the throttle has to be modified,
            // because we want it to move a certain amount for each time unit;
            // we can only do that precisely if the time between each measurement is taken
            // into account.
            let delta_time = self.last_timestamp.elapsed().as_secs_f32();

            self.throttle_modifier = 0.0;

            {
                // Obtain a mutable handle to the given state we want to modify
                let axis_control_data = state.axes_mut();

                // Update the yaw from the measurements of the horizontal axis of the
                // left stick (if we have them)
                if let Some(value) = left_stick_x {
                    // ...only if we have values newer than the last ones collected
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_yaw(deadzone!(-0.01, value.value(), 0.01));
                    }
                }

                // Update the throttle from the measurements of the vertical axis
                // of the left stick (if we have them)
                if let Some(value) = left_stick_y {
                    // The updating has to be constant for the throttle; no need to check
                    // the counters.
                    // We add or remove throttle proportionally to the actual stick position.
                    axis_control_data.add_throttle(deadzone!(-0.01, value.value(), 0.01) * delta_time * THROTTLE_CHANGE_SPEED);
                }

                // Remove additional throttle from the measurements of the left trigger
                // (if we have them)
                if let Some(value) = left_trigger {
                    self.throttle_modifier -= value.value() * THROTTLE_QUICK_TRIGGER_BOUND;
                }

                // Add additional throttle from the measurements of the left trigger
                // (if we have them)
                if let Some(value) = right_trigger {
                    self.throttle_modifier += value.value() * THROTTLE_QUICK_TRIGGER_BOUND;
                }

                // Update the roll from the measurements of the horizontal axis of the
                // right stick (if we have them)
                if let Some(value) = right_stick_x {
                    // ...only if we have values newer than the last ones collected
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_roll(deadzone!(-0.01, value.value(), 0.01));
                    }
                }

                // Update the pitch from the measurements of the vertical axis
                // of the right stick (if we have them)
                if let Some(value) = right_stick_y {
                    // ...only if we have values newer than the last ones collected
                    if value.counter() >= self.driver.counter() {
                        axis_control_data.set_pitch(deadzone!(-0.01, value.value(), 0.01));
                    }
                }
            }

            self.last_timestamp = Instant::now();

            // Increment the counter corresponding to the current measurement
            self.driver.inc();

            return Ok(true);
        }

        self.driver.inc();

        return Ok(false);
    }

    fn control_state_into_message_state(&mut self, msg_state: &mut G2AControllerAxisState, ctrl_state: &ControlState) {
        // Obtain a handle to the input state
        let axes = ctrl_state.axes_ref();

        // The final throttle will be the sum of the currently set throttle ([0, 1]) and the
        // throttle modifier adjusted with the gamepad triggers ([0, 1]), mapped from the
        // [0, 1] range to the [0, 127] range, clamped to [0, 127] and converted into a
        // signed 1-byte integer value.
        msg_state.throttle = f64::max( // Clamp maximum
            0.0,
            f64::min( // Clamp minimum
                127.0,
                map_values( // Mapping
                    (axes.get_throttle() + self.throttle_modifier) as f64, // Throttle + modifier
                    (0.0, 1.0),
                    (0.0, 127.0)
                )
            )
        ) as i8; // Type conversion

        // The final values of the rest of axes will be the input values mapped from [-1, 1] to
        // [-128, -127] and converted to signed 1-byte integer value .
        msg_state.roll = map_values(axes.get_roll() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
        msg_state.pitch = map_values(-axes.get_pitch() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
        msg_state.yaw = map_values(-axes.get_yaw() as f64, (-1.0, 1.0), (-128.0, 127.0)) as i8;
    }
}

impl PhysicalGamepadService {
    /// Create a new `PhysicalGamepadService`.
    pub fn new() -> Result<Self, Box<dyn Error>> {
        Ok(PhysicalGamepadService {
            driver: Gilrs::new()?,
            current_gamepad: None,
            axes_codes: None,
            last_timestamp: Instant::now(),
            throttle_modifier: 0.0,
        })
    }

    /// Search for a connected gamepad. Returns its ID (which can be used to get a hold
    /// of its state later) or `None` if no gamepad was found.
    pub fn search_for_gamepad(&mut self) -> Option<GamepadId> {
        // For all connected gamepads...
        for (id, gamepad) in self.driver.gamepads() {
            // ...if their control mappings aren't invalid...
            if gamepad.mapping_source() != MappingSource::None {
                // ...they are valid gamepads! Return them.
                info!("Found gamepad: {} - {} ({:?}, ff={}, {:?})", id, gamepad.name(),
                    gamepad.power_info(), gamepad.is_ff_supported(), gamepad.mapping_source());

                return Some(id);
            }
        }

        // Otherwise, no gamepads were found
        None
    }
}
