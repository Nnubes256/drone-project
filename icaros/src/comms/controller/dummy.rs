use crate::comms::controller::base::A2RMessage;
use crate::comms::controller::base::ControllerCommunicationService;
use crate::comms::controller::base::R2AMessage;

pub struct DummyController {
    arduino_queue: Vec<Vec<u8>>,
    raspberry_queue: Vec<Vec<u8>>,
}

impl ControllerCommunicationService for DummyController {
    type ControllerCommunicationOptions = ();
    fn setup(_: Self::ControllerCommunicationOptions) -> Self {
        DummyController {
            arduino_queue: Vec::new(),
            raspberry_queue: Vec::new(),
        }
    }

    fn send(&mut self, _: R2AMessage) -> bool {
        unimplemented!()

        // TODO:
        // serialize
        // push 1 to beginning of arduino queue
    }

    fn recv_available(&mut self) -> usize {
        self.raspberry_queue.len()
    }

    fn recv(&mut self) -> Option<A2RMessage> {
        unimplemented!()

        // TODO:
        // take 1 from raspberry queue
        // parse and return
    }
}

#[cfg(test)]
mod tests {
    // TODO write tests
}
