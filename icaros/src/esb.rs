//! External Service Broker

//
// Imports
use std::error::Error;
use icaros_base::comms::common::ApplicationPacket;
use std::collections::VecDeque;
use crate::comms::application::ApplicationService;
use crate::hardware::camera::CameraService;
use std::thread::{self, JoinHandle};
use std::sync::Arc;
use std::time::Duration;
use parking_lot::Mutex;


///
/// Thread-shared data structures of the external service broker
pub struct ESBSharedData {
    camera: Option<Arc<Mutex<CameraService>>>,
    app_services: ApplicationService
}

impl ESBSharedData {
    /// Creates a new ESB shared data structure with no camera support
    #[allow(unused)]
    pub fn new() -> Self {
        ESBSharedData {
            camera: None,
            app_services: ApplicationService::new()
        }
    }

    /// Creates a new ESB shared data structure with camera support
    #[allow(unused)]
    pub fn with_camera() -> Self {
        // TODO handle
        let cam = Arc::new(Mutex::new(CameraService::new().unwrap()));
        ESBSharedData {
            camera: Some(cam.clone()),
            app_services: ApplicationService::with_camera(cam.clone())
        }
    }
}


///
/// The external service broker.
///
/// This part of the system handles secondary tasks, most specifically the communication with
/// the applications running with the drone and the camera operation.
///
/// In order to disturb main operations the least possible, this code is run through a separate
/// process, with communication between the main process and the external service broker happening
/// through shared data structures, which have to be locked by the process before it can read,
/// write or execute code operating on them.
pub struct ExternalServiceBroker {
    /// The thread handle
    thread: Option<JoinHandle<()>>,

    /// The shared data structures
    shared_data: Arc<ESBSharedData>
}

impl ExternalServiceBroker {
    /// Creates a new external service broker with no camera support
    #[allow(unused)]
    pub fn new() -> Self {
        ExternalServiceBroker {
            thread: None,
            shared_data: Arc::new(ESBSharedData::new())
        }
    }

    /// Creates a new external service broker with camera support
    #[allow(unused)]
    pub fn with_camera() -> Self {
        ExternalServiceBroker {
            thread: None,
            shared_data: Arc::new(ESBSharedData::with_camera())
        }
    }

    /// Starts the external service broker. This involves initializing its sub-functions and
    /// launching the thread that handles it.
    pub fn start(&mut self) -> Result<(), Box<dyn Error>> {
        // If we have camera support...
        if let Some(cam) = &self.shared_data.camera {
            // Get a hold of and initialize the camera
            cam.lock().start().map_err(|e| Box::new(e))?;
        }

        // Start listening on the given port for application-specific messages
        self.shared_data.app_services.listen(6969)?;

        // Get a shared reference to our shared data
        let shared = self.shared_data.clone();

        // Launch the thread
        self.thread = Some(thread::spawn(move || {
            Self::esb_thread(shared);
        }));

        Ok(())
    }

    /// Code that runs within the ESB thread
    fn esb_thread(shared_data: Arc<ESBSharedData>) {
        // Forever...
        loop {
            // ...process the application-specific messages
            shared_data.app_services.process_messages();

            // Sleep for 0.5ms, in order to consume less power
            thread::sleep(Duration::from_micros(500));
        }
    }

    /// Obtains a locked reference to the queue that holds the incoming (i.e. from ground control)
    /// application-specific messages. It has to be unlocked before reading or writing to it.
    pub fn app_messages_rx(&self) -> &Mutex<VecDeque<ApplicationPacket>> {
        &self.shared_data.app_services.appmsg_rx_queue
    }

    /// Obtains a locked reference to the queue that holds the outgoing (i.e. from applications)
    /// application-specific messages. It has to be unlocked before reading or writing to it.
    pub fn app_messages_tx(&self) -> &Mutex<VecDeque<ApplicationPacket>> {
        &self.shared_data.app_services.appmsg_tx_queue
    }
}
