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

pub trait AirState {
    fn get_motor_speeds(&self) -> MotorSpeed;
    fn get_orientation(&self) -> Orientation;
    fn get_acceleration(&self) -> Acceleration;
    fn set_controller_state(&mut self, new: G2AControllerAxisState);
    fn get_gps_coordinates(&self) -> Option<FullGPSData>;
    fn set_gps_coordinates(&mut self, new_latlon: (Option<f64>, Option<f64>), alt: Option<f32>);
}

pub trait GroundState {
    fn set_motor_speeds(&mut self, new: MotorSpeed);
    fn set_orientation(&mut self, new: Orientation);
    fn set_acceleration(&mut self, new: Acceleration);
    fn get_controller_state(&self) -> G2AControllerAxisState;
    fn set_gps_coordinates(&mut self, new: Option<FullGPSData>);
}

pub struct AirPacketScheduler<T, S, Tx, Rx> {
    comm_service: T,
    tx_queue: VecDeque<Tx>,
    tx_app_queue: VecDeque<ApplicationPacket>,
    rx_app_queue: VecDeque<ApplicationPacket>,
    latency_recv: u32,
    pkt_counter: u16,
    _marker_state: PhantomData<S>,
    _marker_rx: PhantomData<Rx>,
    last_gps_data_send: Instant,
}

impl<T, S> AirPacketScheduler<T, S, A2GMessage, G2AMessage>
where
    T: AirCommunicationService<A2GMessage, G2AMessage>,
    S: AirState,
{
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

    fn increment_pkt_counter(&mut self) {
        let (new_value, _) = self.pkt_counter.overflowing_add(1);
        self.pkt_counter = new_value;
    }

    fn generate_message_from_state(&mut self, state: &S)
    where
        S: AirState,
    {
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::MOTR(state.get_motor_speeds()),
        ));
        self.increment_pkt_counter();
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::ORNT(state.get_orientation()),
        ));
        self.increment_pkt_counter();
        self.tx_queue.push_back(A2GMessage::new(
            self.pkt_counter,
            A2GCommandType::ACEL(state.get_acceleration()),
        ));
        self.increment_pkt_counter();

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

        while !self.tx_app_queue.is_empty() && self.tx_queue.len() <= 8 {
            self.tx_queue.push_back(A2GMessage::new(
                self.pkt_counter, A2GCommandType::APPM(self.tx_app_queue.pop_front().unwrap())));
            self.increment_pkt_counter();
        }
    }

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

impl<T, S> AirPacketScheduler<T, S, G2AMessage, A2GMessage>
where
    T: AirCommunicationService<G2AMessage, A2GMessage>,
    S: GroundState,
{
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

    fn increment_pkt_counter(&mut self) {
        let (new_value, _) = self.pkt_counter.overflowing_add(1);
        self.pkt_counter = new_value;
    }

    fn generate_message_from_state(&mut self, state: &S)
    where
        S: GroundState,
    {
        self.tx_queue.push_back(G2AMessage::new(
            self.pkt_counter,
            G2ACommandType::CNTA(state.get_controller_state())));
        self.increment_pkt_counter();

        while !self.tx_app_queue.is_empty() && self.tx_queue.len() <= 8 {
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

        self.generate_message_from_state(state);

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

        while self.comm_service.recv_available() > 0 {
            //debug!("recv! {}", self.comm_service.recv_available());
            let packet = match self.comm_service.recv() {
                Ok(packet) => packet,
                Err(err) => match err {
                    ReceiveError::NoPacketsAvailable => break,
                    _ => return Err(err.into()),
                },
            };

            any_message_received = true;
            self.latency_recv = 0;

            self.update_state_from_message(state, packet);
        }

        if !any_message_received {
            self.latency_recv += 1;
            if self.latency_recv > 100 {
                /*warn!(
                    "[ControllerPacketScheduler] Have not received anything for {} > 100 ticks!",
                    self.latency_recv
                );*/
            }
            return Ok(false);
        }

        Ok(true)
    }
}

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

        self.generate_message_from_state(state);

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

            self.update_state_from_message(state, packet);
        }

        if !any_message_received {
            self.latency_recv += 1;
            if self.latency_recv > 100 {
                /*warn!(
                    "[ControllerPacketScheduler] Have not received anything for {} > 100 ticks!",
                    self.latency_recv
                );*/
            }
            return Ok(false);
        }

        Ok(true)
    }
}

impl<T, S, Tx, Rx> ApplicationPacketScheduler for AirPacketScheduler<T, S, Tx, Rx>
where
    T: AirCommunicationService<Tx, Rx>
{
    fn send_app_packet(&mut self, data: ApplicationPacket) -> Result<bool, Box<dyn Error>> {
        self.tx_app_queue.push_back(data);
        Ok(true)
    }

    fn recv_app_packets(&mut self) -> Result<&mut VecDeque<ApplicationPacket>, Box<dyn Error>> {
        Ok(&mut self.rx_app_queue)
    }
}
