//! Dummy implementation of a flight controller communication service (used to test
//! system's architecture)

use err_derive::Error;
use icaros_base::comms::controller::{
    get_controller_codec, A2RMessage, ControllerCommunicationService, ControllerReceiveError,
    ControllerSendError, R2ADesiredRotationAndThrottle, R2AMessage,
};

#[derive(Debug, Error)]
pub enum DummyError {
    #[error(display = "{}", _0)]
    Error(String),
}

impl From<String> for DummyError {
    fn from(value: String) -> Self {
        DummyError::Error(value)
    }
}

pub struct DummyController {
    arduino_queue: Vec<Vec<u8>>,
    raspberry_queue: Vec<Vec<u8>>,
    codec: bincode2::Config,
}

impl ControllerCommunicationService for DummyController {
    type ControllerCommunicationOptions = ();
    type HardwareDriverError = DummyError;

    fn setup(_: Self::ControllerCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        Ok(DummyController {
            arduino_queue: Vec::new(),
            raspberry_queue: Vec::new(),
            codec: get_controller_codec(),
        })
    }

    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        let message = match self.codec.serialize(&msg) {
            Ok(msg_bytes) => msg_bytes,
            Err(e) => {
                return Err(ControllerSendError::SerializationError(e));
            }
        };

        self.arduino_queue.push(message);

        Ok(true)
    }

    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError> {
        Ok(self.raspberry_queue.len())
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let new_msg = match self.raspberry_queue.pop() {
            Some(el) => el,
            None => return Err(ControllerReceiveError::NoPacketsAvailable),
        };

        match self.codec.deserialize::<A2RMessage>(&new_msg) {
            Ok(message) => Ok(message),
            Err(err) => Err(ControllerReceiveError::DeserializationError(err)),
        }
    }

    fn is_tx_busy(&mut self) -> bool {
        false
    }
}

impl DummyController {
    pub fn recv_controller(&mut self) -> Option<Vec<u8>> {
        self.arduino_queue.pop()
    }

    pub fn send_self_bytes(&mut self, packet: &[u8]) {
        self.raspberry_queue.push(packet.to_vec());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;

    #[test]
    fn r2a_packet_serialize() {
        let test_packet = R2AMessage {
            rpyt: R2ADesiredRotationAndThrottle::new(0, -10, 0, 256),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[0x00, 0x00, 0xF6, 0xFF, 0x00, 0x00, 0x00, 0x01];
        let obtained = dummy_comms.recv_controller().unwrap();
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(
            expected.len(),
            obtained.len(),
            "expected length ({}) != obtained length ({})",
            expected.len(),
            obtained.len()
        );
        assert_eq!(
            expected.iter().zip(obtained).all(|(a, b)| (*a == b)),
            true,
            "Contents are not equal"
        );
    }

    #[test]
    fn a2r_packet_deserialize() {
        let message: &[u8] = &[
            0x09, 0x00, // Counter
            0xDC, 0x05, // PWM speed of motor 1 = 1500
            0xB0, 0x04, // PWM speed of motor 2 = 1200
            0xB0, 0x04, // PWM speed of motor 3 = 1200
            0xDC, 0x05, // PWM speed of motor 4 = 1500
            0xC7, 0x4B, 0x77, 0x3f, // Orientation quartenion: W = 0.966
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: X = 0.000
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: Y = 0.000
            0xA6, 0x9b, 0x84, 0xbe, // Orientation quartenion: Z = -0.259
            0x00, 0x00, 0xA0, 0x40, // Acceleration vector: X = 5
            0x00, 0x00, 0x00, 0x00, // Acceleration vector: Y = 0
            0x00, 0x00, 0x00, 0x00, // Acceleration vector: Z = 0
        ];

        let mut dummy_comms = DummyController::setup(()).unwrap();

        dummy_comms.send_self_bytes(message);

        let message_dec = match dummy_comms.recv() {
            Ok(msg) => msg,
            Err(err) => panic!("{}", err),
        };

        println!("I have message in bytes: {:?}", message);
        println!("I parse it and get: {:?}", message_dec);

        assert_eq!(message_dec.counter, 9);
        assert_eq!(message_dec.motor_speed.tl, 1500);
        assert_eq!(message_dec.motor_speed.tr, 1200);
        assert_eq!(message_dec.motor_speed.bl, 1200);
        assert_eq!(message_dec.motor_speed.br, 1500);
        assert_eq!(message_dec.acceleration.as_point3().x, 5.0);
        assert_eq!(message_dec.acceleration.as_point3().y, 0.0);
        assert_eq!(message_dec.acceleration.as_point3().z, 0.0);
    }

    #[bench]
    fn a2r_packet_deserialize_bench(b: &mut Bencher) {
        let message: &[u8] = &[
            0x09, 0x00, // Counter
            0xDC, 0x05, // PWM speed of motor 1 = 1500
            0xB0, 0x04, // PWM speed of motor 2 = 1200
            0xB0, 0x04, // PWM speed of motor 3 = 1200
            0xDC, 0x05, // PWM speed of motor 4 = 1500
            0xC7, 0x4B, 0x77, 0x3f, // Orientation quartenion: W = 0.966
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: X = 0.000
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: Y = 0.000
            0xA6, 0x9b, 0x84, 0xbe, // Orientation quartenion: Z = -0.259
            0x00, 0x00, 0xA0, 0x40, // Acceleration vector: X = 5
            0x00, 0x00, 0x00, 0x00, // Acceleration vector: Y = 0
            0x00, 0x00, 0x00, 0x00, // Acceleration vector: Z = 0
        ];

        let mut dummy_comms = DummyController::setup(()).unwrap();

        b.iter(|| {
            dummy_comms.send_self_bytes(message);

            match dummy_comms.recv() {
                Ok(msg) => msg,
                Err(err) => panic!("{}", err),
            }
        })
    }
}
