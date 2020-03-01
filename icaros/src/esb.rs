use std::error::Error;
use icaros_base::comms::common::ApplicationPacket;
use std::collections::VecDeque;
use crate::comms::application::ApplicationService;
use crate::hardware::camera::CameraService;
use std::thread::{self, JoinHandle};
use std::sync::Arc;
use std::time::Duration;
use parking_lot::Mutex;

pub struct ESBSharedData {
    camera: Option<Arc<Mutex<CameraService>>>,
    app_services: ApplicationService
}

impl ESBSharedData {
    pub fn new() -> Self {
        ESBSharedData {
            camera: None,
            app_services: ApplicationService::new()
        }
    }

    pub fn with_camera() -> Self {
        // TODO handle
        let cam = Arc::new(Mutex::new(CameraService::new().unwrap()));
        ESBSharedData {
            camera: Some(cam.clone()),
            app_services: ApplicationService::with_camera(cam.clone())
        }
    }
}

pub struct ExternalServiceBroker {
    thread: Option<JoinHandle<()>>,
    shared_data: Arc<ESBSharedData>
}

impl ExternalServiceBroker {
    pub fn new() -> Self {
        ExternalServiceBroker {
            thread: None,
            shared_data: Arc::new(ESBSharedData::new())
        }
    }

    pub fn with_camera() -> Self {
        ExternalServiceBroker {
            thread: None,
            shared_data: Arc::new(ESBSharedData::with_camera())
        }
    }

    pub fn start(&mut self) -> Result<(), Box<dyn Error>> {
        if let Some(cam) = &self.shared_data.camera {
            cam.lock().start().map_err(|e| Box::new(e))?;
        }
        self.shared_data.app_services.listen(6969)?;
        let shared = self.shared_data.clone();
        self.thread = Some(thread::spawn(move || {
            Self::esb_thread(shared);
        }));
        Ok(())
    }

    fn esb_thread(shared_data: Arc<ESBSharedData>) {
        loop {
            shared_data.app_services.process_messages();
            thread::sleep(Duration::from_micros(500));
        }
    }

    pub fn app_messages_rx(&self) -> &Mutex<VecDeque<ApplicationPacket>> {
        &self.shared_data.app_services.appmsg_rx_queue
    }

    pub fn app_messages_tx(&self) -> &Mutex<VecDeque<ApplicationPacket>> {
        &self.shared_data.app_services.appmsg_tx_queue
    }
}
