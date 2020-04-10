//!
//! Packet scheduler for air-to-ground and corresponding trait (interface) implementations
//!

use crate::comms::common::ApplicationPacket;
use crate::comms::common::ApplicationPacketScheduler;
use crate::core::FullGPSData;
use crate::comms::{
    air::{
        A2GCommandType, A2GMessage, AirCommunicationService, G2ACommandType,
        G2AControllerAxisState, G2AMessage, GPSNoData, ReceiveError
    },
    common::{Acceleration, MotorSpeed, Orientation, PacketScheduler},
};
use std::{collections::VecDeque, error::Error, marker::PhantomData};
use std::time::Instant;

/// The maximum amount of application-specific messages the drone will
/// schedule at once
const APPM_SEND_BURST_LIMIT_DRONE: usize = 8;

/// The maximum amount of application-specific messages ground control will
/// schedule at once
const APPM_SEND_BURST_LIMIT_GROUND: usize = 8;

///
/// A storage for drone in-flight state
///
/// Implementors are able to function as storage for the drone's in-flight state
/// where the packet schedulers can store data on.
/// This could be a memory-only structure or a simple database with logging capabilities,
/// depending on the use case.
pub trait AirState {
    /// Returns the drone's current motor speeds
    fn get_motor_speeds(&self) -> MotorSpeed;

    /// Returns the drone's current absolute orientation
    fn get_orientation(&self) -> Orientation;

    /// Returns the drone's current linear acceleration
    fn get_acceleration(&self) -> Acceleration;

    /// Stores the current gamepad state received from ground control
    fn set_controller_state(&mut self, new: G2AControllerAxisState);

    /// If full GPS data is available, returns it; otherwise returns `None`
    fn get_gps_coordinates(&self) -> Option<FullGPSData>;

    /// Stores the provided GPS data
    fn set_gps_coordinates(&mut self, new_latlon: (Option<f64>, Option<f64>), alt: Option<f32>);
}

///
/// A storage for ground control
///
/// Implementors are able to function as storage for the ground control state
/// where the packet schedulers can store data on.
/// This could be a memory-only structure or a simple database with logging capabilities,
/// depending on the use case.
pub trait GroundState {
    /// Stores the drone's current motor speeds
    fn set_motor_speeds(&mut self, new: MotorSpeed);

    /// Stores the drone's current absolute orientation
    fn set_orientation(&mut self, new: Orientation);

    /// Stores the drone's current linear acceleration
    fn set_acceleration(&mut self, new: Acceleration);

    /// Returns the current gamepad state, ready for transmission to the drone
    fn get_controller_state(&self) -> G2AControllerAxisState;

    /// Stores the provided full GPS data, if available
    fn set_gps_coordinates(&mut self, new: Option<FullGPSData>);
}

///
/// Packet scheduler for ground-to-air and air-to-ground communications
///
/// - `T` defines the underlying device driver to which packets are scheduled
/// - `S` defines the storage type to where the scheduler will go to fetch/store data
/// - `Tx` defines the type of the messages being sent through the device driver
/// - `Rx` defines the type of the messages being received through the device driver
///
/// This packet scheduler holds several implementations, one for each side
/// of the communication.
pub struct AirPacketScheduler<T, S, Tx, Rx> where
    T: AirCommunicationService<Tx, Rx>
{
    comm_service: T,
    tx_queue: VecDeque<Tx>,
    tx_app_queue: VecDeque<ApplicationPacket>,
    rx_app_queue: VecDeque<ApplicationPacket>,
    latency_recv: u32,
    pkt_counter: u16,
    last_gps_data_send: Instant,
    _marker_state: PhantomData<S>,
    _marker_rx: PhantomData<Rx>,
}

///
/// Common routines for all implementations
impl <T, S, Tx, Rx> AirPacketScheduler<T, S, Tx, Rx> where
    T: AirCommunicationService<Tx, Rx>
{
    /// Increments the packet counter for the next packet to be sent
    fn increment_pkt_counter(&mut self) {
        let (new_value, _) = self.pkt_counter.overflowing_add(1);
        self.pkt_counter = new_value;
    }
}

///
/// Implementation-specific methods for implementation of the packet scheduler
/// for the **drone side** and its corresponding device driver and state storage implementations.
impl<T, S> AirPacketScheduler<T, S, A2GMessage, G2AMessage>
where
    T: AirCommunicationService<A2GMessage, G2AMessage>,
    S: AirState,
{
    /// Initializes a new air-side packet scheduler, together with its underlying driver
    /// and given the latter's settings to be used
    pub fn initialize(config: T::AirCommunicationOptions) -> Result<Self, T::HardwareDriverError> {
        Ok(AirPacketScheduler::<T, S, A2GMessage, G2AMessage> {
            comm_service: T::setup(config)?,
            tx_queue: VecDeque::new(),
            tx_app_queue: VecDeque::new(),
            rx_app_queue: VecDeque::new(),
            latency_recv: 0,
            pkt_counter: 0,
            _marker_state: PhantomData,
            _marker_rx: PhantomData,
            last_gps_data_send: Instant::now()
        })
    }

    /// Generates a set of messages that convey the given state and schedules them
    /// for sending
    fn generate_message_from_state(&mut self, state: &S)
    where
        S: AirState,
    {
        // Motor speeds
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::MOTR(state.get_motor_speeds()),
        ));
        self.increment_pkt_counter();

        // Orientation
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::ORNT(state.get_orientation()),
        ));
        self.increment_pkt_counter();

        // Acceleration
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::ACEL(state.get_acceleration()),
        ));
        self.increment_pkt_counter();

        // GPS data
        // (which we only send once every second, as that's the frequency the GPS
        // hardware updates the data)
        if self.last_gps_data_send.elapsed().as_secs() >= 1 {
            self.tx_queue.push_back(A2GMessage::new(
                self.pkt_counter,
                if let Some(gps) = state.get_gps_coordinates() {
                    A2GCommandType::GPSD(gps)
                } else {
                    A2GCommandType::GPSN(GPSNoData::default())
                })
            );
            self.increment_pkt_counter();
            self.last_gps_data_send = Instant::now();
        }

        // Outgoing application messages
        // (we only send a certain amount of them at a time in order to avoid overflowing
        // the underlying device driver)
        while !self.tx_app_queue.is_empty() && self.tx_queue.len() <= APPM_SEND_BURST_LIMIT_DRONE {
            self.tx_queue.push_back(A2GMessage::new(
                self.pkt_counter, A2GCommandType::APPM(self.tx_app_queue.pop_front().unwrap())));
            self.increment_pkt_counter();
        }
    }

    /// Updates the given system state from the given incoming message
    fn update_state_from_message(&mut self, state: &mut S, message: G2AMessage)
    where
        S: AirState,
    {
        match message.command {
            G2ACommandType::CNTA(cnta) => state.set_controller_state(cnta),
            G2ACommandType::APPM(appm) => self.rx_app_queue.push_back(appm),
            _ => {}
        };
    }
}

///
/// Implementation of the packet scheduler for the **drone side** and its corresponding
/// device driver and state storage implementations.
impl<T, S> PacketScheduler<A2GMessage, G2AMessage, S> for AirPacketScheduler<T, S, A2GMessage, G2AMessage>
where
    T: AirCommunicationService<A2GMessage, G2AMessage>,
    S: AirState,
{
    fn can_send(&mut self) -> bool {
        !self.comm_service.is_tx_busy()
    }

    fn send_state(&mut self, state: &S) -> Result<bool, Box<dyn Error>> {
        let mut last_error = None;
        let mut ok_status = true;

        // Generate new messages from the given state
        self.generate_message_from_state(state);

        // Try to send each message generated
        while self.tx_queue.len() > 0 {
            let packet = self.tx_queue.pop_front().unwrap();
            match self.comm_service.send(packet) {
                Ok(ok) => {
                    if !ok {
                        ok_status = false;
                    }
                }
                Err(err) => {
                    error!("[AirPacketScheduler] packet send error: {}", err);
                    last_error = Some(err);
                }
            }
        }

        if last_error.is_some() {
            Err(last_error.unwrap().into())
        } else {
            Ok(ok_status)
        }
    }

    fn recv_state(&mut self, state: &mut S) -> Result<bool, Box<dyn Error>> {
        let mut any_message_received = false;

        // While we have packets available, receive them
        while self.comm_service.recv_available() > 0 {
            debug!("recv! {}", self.comm_service.recv_available());
            let packet = match self.comm_service.recv() {
                Ok(packet) => packet,
                Err(err) => match err {
                    ReceiveError::NoPacketsAvailable => break,
                    _ => return Err(err.into()),
                },
            };

            any_message_received = true;
            self.latency_recv = 0;

            // From them, update our own state
            self.update_state_from_message(state, packet);
        }

        if !any_message_received {
            self.latency_recv += 1;
            if self.latency_recv > 100 {
                warn!(
                    "[ControllerPacketScheduler] Have not received anything for {} > 100 ticks!",
                    self.latency_recv
                );
            }
            return Ok(false);
        }

        Ok(true)
    }
}

///
/// Implementation-specific methods for implementation of the packet scheduler
/// for the **ground control side** and its corresponding device driver
/// and state storage implementations.
impl<T, S> AirPacketScheduler<T, S, G2AMessage, A2GMessage>
where
    T: AirCommunicationService<G2AMessage, A2GMessage>,
    S: GroundState,
{
    /// Initializes a new ground-side packet scheduler, together with its underlying driver
    /// and given the latter's settings to be used
    pub fn initialize(config: T::AirCommunicationOptions) -> Result<Self, T::HardwareDriverError> {
        Ok(AirPacketScheduler::<T, S, G2AMessage, A2GMessage> {
            comm_service: T::setup(config)?,
            tx_queue: VecDeque::new(),
            tx_app_queue: VecDeque::new(),
            rx_app_queue: VecDeque::new(),
            latency_recv: 0,
            pkt_counter: 0,
            _marker_state: PhantomData,
            _marker_rx: PhantomData,
            last_gps_data_send: Instant::now()
        })
    }

    fn generate_message_from_state(&mut self, state: &S)
    where
        S: GroundState,
    {
        self.tx_queue.push_back(G2AMessage::new(
            self.pkt_counter,
            G2ACommandType::CNTA(state.get_controller_state())));
        self.increment_pkt_counter();

        while !self.tx_app_queue.is_empty() && self.tx_queue.len() <= APPM_SEND_BURST_LIMIT_GROUND {
            self.tx_queue.push_back(G2AMessage::new(
                self.pkt_counter, G2ACommandType::APPM(self.tx_app_queue.pop_front().unwrap())));
            self.increment_pkt_counter();
        }
    }

    fn update_state_from_message(&mut self, state: &mut S, message: A2GMessage)
    where
        S: GroundState,
    {
        match message.command {
            A2GCommandType::MOTR(v) => state.set_motor_speeds(v),
            A2GCommandType::ORNT(v) => state.set_orientation(v),
            A2GCommandType::ACEL(v) => state.set_acceleration(v),
            A2GCommandType::APPM(v) => self.rx_app_queue.push_back(v),
            A2GCommandType::GPSN(_) => state.set_gps_coordinates(None),
            A2GCommandType::GPSD(v) => state.set_gps_coordinates(Some(v))
        };
    }
}

///
/// Implementation of the packet scheduler for the **ground control side** and its corresponding
/// device driver and state storage implementations.
impl<T, S> PacketScheduler<G2AMessage, A2GMessage, S> for AirPacketScheduler<T, S, G2AMessage, A2GMessage>
where
    T: AirCommunicationService<G2AMessage, A2GMessage>,
    S: GroundState,
{
    fn can_send(&mut self) -> bool {
        !self.comm_service.is_tx_busy()
    }

    fn send_state(&mut self, state: &S) -> Result<bool, Box<dyn Error>> {
        let mut last_error = None;
        let mut ok_status = true;

        // Generate a new message from the given state
        self.generate_message_from_state(state);

        // Try to send each message generated
        while self.tx_queue.len() > 0 {
            let packet = self.tx_queue.pop_front().unwrap();
            match self.comm_service.send(packet) {
                Ok(ok) => {
                    if !ok {
                        ok_status = false
                    }
                }
                Err(err) => {
                    error!("[AirPacketScheduler] packet send error: {}", err);
                    last_error = Some(err);
                }
            }
        }

        if last_error.is_some() {
            Err(last_error.unwrap().into())
        } else {
            Ok(ok_status)
        }
    }

    fn recv_state(&mut self, state: &mut S) -> Result<bool, Box<dyn Error>> {
        let mut any_message_received = false;

        // While we have packets available, receive them
        while self.comm_service.recv_available() > 0 {
            debug!("recv! {}", self.comm_service.recv_available());
            let packet = match self.comm_service.recv() {
                Ok(packet) => packet,
                Err(err) => match err {
                    ReceiveError::NoPacketsAvailable => break,
                    _ => return Err(err.into()),
                },
            };

            any_message_received = true;
            self.latency_recv = 0;

            // From them, update our own state
            self.update_state_from_message(state, packet);
        }

        if !any_message_received {
            self.latency_recv += 1;
            if self.latency_recv > 100 {
                warn!(
                    "[ControllerPacketScheduler] Have not received anything for {} > 100 ticks!",
                    self.latency_recv
                );
            }
            return Ok(false);
        }

        Ok(true)
    }
}

///
/// Implementation of the application packet scheduler for both sides of the communication.
impl<T, S, Tx, Rx> ApplicationPacketScheduler for AirPacketScheduler<T, S, Tx, Rx>
where
    T: AirCommunicationService<Tx, Rx>
{
    fn send_app_packet(&mut self, data: ApplicationPacket) -> Result<bool, Box<dyn Error>> {
        // Push the received packet into our internal outgoing application messages queue
        self.tx_app_queue.push_back(data);
        Ok(true)
    }

    fn recv_app_packets(&mut self) -> Result<&mut VecDeque<ApplicationPacket>, Box<dyn Error>> {
        // Just give access to our own incoming application message queue
        Ok(&mut self.rx_app_queue)
    }
}
