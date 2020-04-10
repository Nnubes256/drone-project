//! Dummy implementation of an air-to-ground communication service (used to test
//! system's architecture)

use bincode2;
use err_derive::Error;
use icaros_base::comms::air::{
    get_air_codec, A2GMessage, AirCommunicationService, G2AMessage, ReceiveError, SendError,
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
    pub ground_queue: Vec<Vec<u8>>,
    pub drone_queue: Vec<Vec<u8>>,
    codec: bincode2::Config,
}

impl AirCommunicationService<A2GMessage, G2AMessage> for DummyController {
    type AirCommunicationOptions = ();
    type HardwareDriverError = DummyError;
    fn setup(_: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        info!("[AirDummyController] Ready!");
        Ok(DummyController {
            ground_queue: Vec::new(),
            drone_queue: Vec::new(),
            codec: get_air_codec(),
        })
    }

    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        let message = match self.codec.serialize(&msg) {
            Ok(msg_bytes) => msg_bytes,
            Err(e) => {
                return Err(SendError::SerializationError(e));
            }
        };

        self.ground_queue.push(message);

        Ok(true)
    }

    fn recv_available(&mut self) -> usize {
        self.drone_queue.len()
    }

    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>> {
        let new_msg = match self.drone_queue.pop() {
            Some(el) => el,
            None => return Err(ReceiveError::NoPacketsAvailable),
        };

        match self.codec.deserialize::<G2AMessage>(&new_msg) {
            Ok(message) => Ok(message),
            Err(err) => Err(ReceiveError::DeserializationError(err)),
        }
    }

    fn get_max_app_message_size() -> usize {
        26
    }

    fn is_tx_busy(&mut self) -> bool {
        false
    }
}

impl DummyController {
    /*pub fn send_drone_message(&mut self, msg: G2AMessage) {
        self.drone_queue.push(get_codec().serialize(&msg).unwrap());
    }*/

    #[cfg(test)]
    pub fn send_drone_message_bytes(&mut self, msg: Vec<u8>) {
        self.drone_queue.push(msg);
    }

    /*pub fn recv_ground_message_bytes(&mut self) -> Option<Vec<u8>> {
        self.ground_queue.pop()
    }*/
}

#[cfg(test)]
mod tests {
    use super::*;
    use icaros_base::{
        comms::{
            air::{A2GCommandType, G2ACommandType},
            common::{Acceleration, MotorSpeed, Orientation, ApplicationPacket},
        },
        utils::{Point3, Quartenion},
    };

    #[test]
    fn a2g_packet_motr_serialize() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::MOTR(MotorSpeed::from_speeds((1000, 0, 0, 1000))),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[
            0x7F, 0x03, 0x00, 0x02, 0x08, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03,
        ];
        let obtained = &dummy_comms.ground_queue[0];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == *b)), true);
    }

    #[test]
    fn a2g_packet_ornt_serialize() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::ORNT(Orientation::from_quartenion(Quartenion::from_values(
                2.0, 4.0, 2.0, 4.0,
            ))),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[
            0x7F, 0x03, 0x00, 0x03, 0x10, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00,
            0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40,
        ];
        let obtained = &dummy_comms.ground_queue[0];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == *b)), true);
    }

    #[test]
    fn a2g_packet_acel_serialize() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::ACEL(Acceleration::from_vec3(Point3::from_components(
                2.0, 4.0, 2.0,
            ))),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[
            0x7F, 0x03, 0x00, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00,
            0x00, 0x00, 0x40,
        ];
        let obtained = &dummy_comms.ground_queue[0];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == *b)), true);
    }

    #[test]
    fn a2g_packet_appm_serialize() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::APPM(ApplicationPacket::new(
                0x7F,
                vec![0x00, 0x01, 0x02, 0x03],
            )),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[
            0x7F, 0x03, 0x00, 0x10, 0x06, 0x7F, 0x04, 0x00, 0x01, 0x02, 0x03,
        ];
        let obtained = &dummy_comms.ground_queue[0];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == *b)), true);
    }

    #[test]
    fn a2g_packet_appm_serialize_full() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::APPM(ApplicationPacket::new(
                0x7F,
                vec![
                    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                    23, 24, 25,
                ],
            )),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(result) => assert_eq!(result, true),
            Err(err) => panic!("{}", err),
        }

        let expected: &[u8] = &[
            0x7F, 0x03, 0x00, 0x10, 0x1B, 0x7F, 0x19, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
            14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
        ];
        let obtained = &dummy_comms.ground_queue[0];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", obtained);
        assert_eq!(expected.len(), obtained.len());
        assert_eq!(expected.iter().zip(obtained).all(|(a, b)| (*a == *b)), true);
    }

    #[test]
    #[should_panic(expected = "Deserialization failed: the size limit has been reached")]
    fn a2g_packet_appm_serialize_overfill() {
        let test_packet = A2GMessage {
            header: 0x7F,
            counter: 0x3,
            command: A2GCommandType::APPM(ApplicationPacket::new(
                0x7F,
                vec![
                    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                    23, 24, 25, 26,
                ],
            )),
        };

        let mut dummy_comms = DummyController::setup(()).unwrap();

        println!("I attempt to send the structure: {:?}", test_packet);

        match dummy_comms.send(test_packet) {
            Ok(res) => assert_eq!(res, true),
            Err(err) => panic!("{}", err),
        }
    }

    #[test]
    fn g2a_packet_cnta_deserialize() {
        let message = vec![0x7F, 0x03, 0x00, 0x02, 0x04, 0x00, 0x00, 0xCE, 0x32];
        let mut dummy_comms = DummyController::setup(()).unwrap();

        dummy_comms.send_drone_message_bytes(message.clone());

        assert_eq!(dummy_comms.recv_available(), 1);

        let message_dec = match dummy_comms.recv() {
            Ok(msg) => msg,
            Err(err) => panic!("{}", err),
        };

        println!("I have message in bytes: {:?}", message);
        println!("I parse it and get: {:?}", message_dec);

        assert_eq!(message_dec.header, 0x7F);
        assert_eq!(message_dec.counter, 0x03);
        match message_dec.command {
            G2ACommandType::CNTA(data) => {
                assert_eq!(data.roll, 0);
                assert_eq!(data.pitch, 0);
                assert_eq!(data.yaw, -50);
                assert_eq!(data.throttle, 50);
            }
            G2ACommandType::APPM(_) => panic!("Expected CNTA, got APPM"),
            G2ACommandType::HEAR(_) => panic!("Expected CNTA, got HEAR"),
        };
    }

    #[test]
    fn g2a_packet_appm_deserialize() {
        let message = vec![
            0x7F, 0x03, 0x00, 0x10, 0x08, 0x7F, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        ];
        let mut dummy_comms = DummyController::setup(()).unwrap();

        dummy_comms.send_drone_message_bytes(message.clone());

        assert_eq!(dummy_comms.recv_available(), 1);

        let message_dec = match dummy_comms.recv() {
            Ok(msg) => msg,
            Err(err) => panic!("{}", err),
        };

        println!("I have message in bytes: {:?}", message);
        println!("I parse it and get: {:?}", message_dec);

        assert_eq!(message_dec.header, 0x7F);
        assert_eq!(message_dec.counter, 0x03);
        match message_dec.command {
            G2ACommandType::CNTA(_) => panic!("Expected APPM, got CNTA"),
            G2ACommandType::APPM(data) => {
                assert_eq!(data.app_id(), 0x7F);

                let expected_app_msg = 1u8..6u8;

                assert_eq!(
                    expected_app_msg
                        .zip(data.message_iter())
                        .all(|(a, b)| (a == *b)),
                    true
                );
            }
            G2ACommandType::HEAR(_) => panic!("Expected APPM, got HEAR"),
        };
    }

    #[test]
    fn g2a_packet_hear_deserialize() {
        let message = vec![0x7F, 0x03, 0x00, 0xFF, 0x04, 0x5B, 0xA2, 0x2D, 0x5E];
        let mut dummy_comms = DummyController::setup(()).unwrap();

        dummy_comms.send_drone_message_bytes(message.clone());

        assert_eq!(dummy_comms.recv_available(), 1);

        let message_dec = match dummy_comms.recv() {
            Ok(msg) => msg,
            Err(err) => panic!("{}", err),
        };

        println!("I have message in bytes: {:?}", message);
        println!("I parse it and get: {:?}", message_dec);

        assert_eq!(message_dec.header, 0x7F);
        assert_eq!(message_dec.counter, 0x03);
        match message_dec.command {
            G2ACommandType::CNTA(_) => panic!("Expected HEAR, got CNTA"),
            G2ACommandType::APPM(_) => panic!("Expected HEAR, got APPM"),
            G2ACommandType::HEAR(data) => assert_eq!(data.timestamp(), 1580048987),
        };
    }
}
