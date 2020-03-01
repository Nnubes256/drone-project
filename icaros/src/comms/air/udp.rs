use std::time::Instant;
use std::net::SocketAddr;
use icaros_base::comms::air::ReceiveError;
use icaros_base::comms::air::SendError;
use icaros_base::comms::air::AirCommunicationService;
use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::G2AMessage;
use icaros_base::BincodeConfig;
use std::net::UdpSocket;
use std::io;
use icaros_base::comms::air::get_air_codec;

pub struct UDPCommunicationOptions {
    pub port: u16
}

impl UDPCommunicationOptions {
    pub fn new(port: u16) -> Self {
        UDPCommunicationOptions { port }
    }
}

pub struct UDPCommunicationService {
    driver: UdpSocket,
    codec: BincodeConfig,
    remote: Option<SocketAddr>,
    recv_done: bool,
    recv_success: bool,
    send_timer: Instant
}

impl AirCommunicationService<A2GMessage, G2AMessage> for UDPCommunicationService {
    type AirCommunicationOptions = UDPCommunicationOptions;
    type HardwareDriverError = io::Error;

    fn setup(options: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        let socket = UdpSocket::bind(("0.0.0.0", options.port))?;
        socket.set_nonblocking(true)?;
        Ok(UDPCommunicationService {
            driver: socket,
            codec: get_air_codec(),
            remote: None,
            recv_done: false,
            recv_success: false,
            send_timer: Instant::now()
        })
    }

    fn get_max_app_message_size() -> usize {
        25
    }

    fn is_tx_busy(&mut self) -> bool {
        !self.recv_success || self.send_timer.elapsed().as_millis() < 1
    }

    fn send(&mut self, message: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        if let Some(remote_addr) = self.remote {
            let message_byte = self.codec.serialize(&message)?;
            debug!("Sending to {}", remote_addr);
            self.recv_done = false;
            self.recv_success = false;
            self.send_timer = Instant::now();
            match self.driver.send_to(&message_byte, remote_addr) {
                Ok(bytes) if bytes > 0 => Ok(true),
                Ok(_) => Ok(false),
                Err(e) => Err(SendError::DriverSendError(e.into()))
            }
        } else {
            Ok(false)
        }
    }

    fn recv_available(&mut self) -> usize {
        if self.recv_done {
            0
        } else {
            1
        }
    }

    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>> {
        let mut message = vec![0; 32];

        let addr = match self.driver.recv_from(&mut message) {
            Ok((_, addr)) => addr,
            Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => {
                return Err(ReceiveError::NoPacketsAvailable)
            },
            Err(err) => return Err(ReceiveError::DriverRecvError(err.into()))
        };

        if self.remote.is_none() {
            debug!("new client thingy!");
            self.remote = Some(addr);
        }

        self.recv_done = true;
        self.recv_success = true;

        Ok(self.codec.deserialize(&message)?)
    }
}
