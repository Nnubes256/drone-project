use icaros_base::comms::air::ReceiveError;
use icaros_base::comms::air::SendError;
use icaros_base::comms::air::AirCommunicationService;
use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::G2AMessage;
use icaros_base::BincodeConfig;
use std::net::UdpSocket;
use std::io;
use std::time::Instant;
use icaros_base::comms::air::get_air_codec;

pub struct UDPCommunicationOptions {
    pub host: String,
    pub port: u16
}

impl UDPCommunicationOptions {
    pub fn new(host: String, port: u16) -> Self {
        UDPCommunicationOptions { host, port }
    }
}

pub struct UDPCommunicationService {
    driver: UdpSocket,
    codec: BincodeConfig,
    recv_done: bool,
    send_timer: Instant
}

impl AirCommunicationService<G2AMessage, A2GMessage> for UDPCommunicationService {
    type AirCommunicationOptions = UDPCommunicationOptions;
    type HardwareDriverError = io::Error;

    fn setup(options: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        let socket = UdpSocket::bind(("127.0.0.1", 55555))?;
        socket.connect((options.host.as_str(), options.port))?;
        socket.set_nonblocking(true)?;
        Ok(UDPCommunicationService {
            driver: socket,
            codec: get_air_codec(),
            recv_done: false,
            send_timer: Instant::now()
        })
    }

    fn get_max_app_message_size() -> usize {
        25
    }

    fn is_tx_busy(&mut self) -> bool {
        self.send_timer.elapsed().as_millis() < 2
    }

    fn send(&mut self, message: G2AMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        let message_byte = self.codec.serialize(&message)?;

        debug!("send data to {:?}", self.driver.peer_addr());
        self.send_timer = Instant::now();

        match self.driver.send(&message_byte) {
            Ok(bytes) if bytes > 0 => Ok(true),
            Ok(_) => Ok(false),
            Err(e) => Err(SendError::DriverSendError(e.into()))
        }
    }

    fn recv_available(&mut self) -> usize {
        if self.recv_done {
            self.recv_done = false;
            0
        } else {
            1
        }
    }

    fn recv(&mut self) -> Result<A2GMessage, ReceiveError<Self::HardwareDriverError>> {
        let mut message = vec![0; 32];

        match self.driver.recv(&mut message) {
            Ok(recv) => debug!("received {} bytes!", recv),
            Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => {
                debug!("No packets!");
                return Err(ReceiveError::NoPacketsAvailable)
            },
            Err(err) => return Err(ReceiveError::DriverRecvError(err.into()))
        };

        self.recv_done = true;

        Ok(self.codec.deserialize(&message)?)
    }
}
