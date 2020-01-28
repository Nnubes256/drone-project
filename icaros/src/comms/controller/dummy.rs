use crate::comms::controller::base::ControllerReceiveError;
use crate::comms::controller::base::ControllerSendError;
use crate::comms::controller::base::A2RMessage;
use crate::comms::controller::base::ControllerCommunicationService;
use crate::comms::controller::base::R2AMessage;
use crate::comms::controller::base::R2ADesiredRotationAndThrottle;
use crate::comms::controller::base::get_controller_codec;
use crate::comms::common::{PIDAxes, PIDParameters};

pub struct DummyController {
    arduino_queue: Vec<Vec<u8>>,
    raspberry_queue: Vec<Vec<u8>>,
    codec: bincode2::Config
}

impl ControllerCommunicationService for DummyController {
    type ControllerCommunicationOptions = ();
    type HardwareDriverError = String;

    fn setup(_: Self::ControllerCommunicationOptions) -> Self {
        DummyController {
            arduino_queue: Vec::new(),
            raspberry_queue: Vec::new(),
            codec: get_controller_codec()
        }
    }

    fn send(&mut self, msg: R2AMessage) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        let message = match self.codec.serialize(&msg) {
            Ok(msg_bytes) => msg_bytes,
            Err(e) => {
                return Err(ControllerSendError::SerializationError { inner: e });
            }
        };

        self.arduino_queue.push(message);

        Ok(true)
    }

    fn recv_available(&mut self) -> usize {
        self.raspberry_queue.len()
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let new_msg = match self.raspberry_queue.pop() {
            Some(el) => el,
            None => return Err(ControllerReceiveError::NoPacketsAvailable),
        };

        match self.codec.deserialize::<A2RMessage>(&new_msg) {
            Ok(message) => Ok(message),
            Err(err) => Err(ControllerReceiveError::DeserializationError { inner: err }),
        }
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

    #[test]
    fn r2a_packet_serialize() {
        let test_packet = R2AMessage {
            header: 0x4F,
            rpyt: R2ADesiredRotationAndThrottle::new(0, -10, 0, 256),
            pid: PIDAxes::new(
                PIDParameters::new(2.0, 5.0, 1.0),
                PIDParameters::new(2.0, 5.0, 1.0),
                PIDParameters::new(2.0, 5.0, 1.0)
            )
        };

        let mut dummy_comms = DummyController::setup(());

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err)
        }

        let expected: &[u8] = &[
            0x4F, // Header
            0x00, 0x00, 0xF6, 0xFF, 0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0x80, 0x3F,
            0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0x80, 0x3F,
            0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0xA0, 0x40, 0x00, 0x00, 0x80, 0x3F
        ];
        let obtained = dummy_comms.recv_controller().unwrap();
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len(), "expected length ({}) != obtained length ({})", expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == b)), true, "Contents are not equal");
    }

    #[test]
    fn r2a_packet_deserialize() {
        let message: &[u8] = &[
            0x4F,           // Header
            0x09, 0x00,     // Counter
            0xDC, 0x05,     // PWM speed of motor 1 = 1500
            0xB0, 0x04,     // PWM speed of motor 1 = 1200
            0xB0, 0x04,     // PWM speed of motor 1 = 1200
            0xDC, 0x05,     // PWM speed of motor 1 = 1500
            0xC7, 0x4B, 0x77, 0x3f, // Orientation quartenion: W = 0.966
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: X = 0.000
            0x00, 0x00, 0x00, 0x00, // Orientation quartenion: Y = 0.000
            0xA6, 0x9b, 0x84, 0xbe, // Orientation quartenion: Z = -0.259
            0x00, 0x00, 0xA0, 0x40, // Acceleration vector: X = 5
            0x00, 0x00, 0x00, 0x00, // Acceleration vector: Y = 0
            0x00, 0x00, 0x00, 0x00 // Acceleration vector: Z = 0
        ];

        let mut dummy_comms = DummyController::setup(());

        dummy_comms.send_self_bytes(message);

        let message_dec = match dummy_comms.recv() {
            Ok(msg) => msg,
            Err(err) => panic!("{}", err),
        };

        println!("I have message in bytes: {:?}", message);
        println!("I parse it and get: {:?}", message_dec);

        assert_eq!(message_dec.header, 0x4F);
        assert_eq!(message_dec.counter, 9);
    }
}
