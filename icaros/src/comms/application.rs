use std::sync::Arc;
use core::fmt::Display;
use icaros_base::comms::common::ApplicationPacket;
use core::time::Duration;
use std::error::Error;
use std::io::{self, BufReader};
use std::io::prelude::*;
use crate::hardware::camera::CameraService;
use std::path::PathBuf;
use std::collections::{HashMap, VecDeque};
use std::net::{TcpListener, TcpStream};
use std::ops::{Deref, DerefMut};
use parking_lot::Mutex;
use serde::{Serialize, Deserialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(from = "u8")]
pub struct ApplicationID(u8);

impl From<u8> for ApplicationID {
    fn from(value: u8) -> Self {
        ApplicationID(value)
    }
}

impl From<ApplicationID> for u8 {
    fn from(value: ApplicationID) -> Self {
        value.0
    }
}

impl From<&ApplicationID> for u8 {
    fn from(value: &ApplicationID) -> Self {
        value.0
    }
}

impl Display for ApplicationID {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(fmt, "{}", self.0)
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[serde(into = "u64", from = "u64")]
pub struct RequestID(u64);

impl From<u64> for RequestID {
    fn from(value: u64) -> Self {
        RequestID(value)
    }
}

impl From<RequestID> for u64 {
    fn from(value: RequestID) -> Self {
        value.0
    }
}

pub enum DataRecvResult<T> {
    Complete(T),
    Partial,
    Idle
}

#[derive(Debug, Serialize)]
#[serde(tag = "type", content = "data")]
pub enum ApplicationServiceMessage {
    #[serde(rename = "ok")]
    GeneralOperationOk { msg_id: RequestID },
    #[serde(rename = "not_ok")]
    GeneralOperationNotOk { msg_id: RequestID },
    #[serde(rename = "camera_pic")]
    CameraPicture { path: PathBuf },
    #[serde(rename = "message")]
    ReceiveApplicationPacket(Vec<u8>)
}

impl ApplicationServiceMessage {
    pub fn ok(rid: RequestID) -> Self {
        Self::GeneralOperationOk { msg_id: rid }
    }

    pub fn nok(rid: RequestID) -> Self {
        Self::GeneralOperationNotOk { msg_id: rid }
    }

    pub fn camera_picture(path: PathBuf) -> Self {
        Self::CameraPicture { path }
    }

    pub fn app_msg(data: Vec<u8>) -> Self {
        Self::ReceiveApplicationPacket(data)
    }
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum ApplicationClientMessageType {
    #[serde(rename = "register_application")]
    RegisterApplication { id: ApplicationID },
    #[serde(rename = "req_camera_pic")]
    RequestCameraPicture,
    #[serde(rename = "message")]
    SendApplicationPacket(Vec<u8>)
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ApplicationClientMessage {
    id: RequestID,
    #[serde(flatten)]
    message_type: ApplicationClientMessageType
}

pub struct ApplicationConnection {
    connection_reader: BufReader<TcpStream>,
    scratch: String,
    curr_message: String,
    curr_length: usize
}

impl ApplicationConnection {
    // TODO make Result
    pub fn from_connection(connection: TcpStream) -> Result<Self, io::Error> {
        connection.set_read_timeout(Some(Duration::from_micros(200)))?;
        Ok(ApplicationConnection {
            connection_reader: BufReader::new(connection),
            scratch: String::with_capacity(1024),
            curr_message: String::with_capacity(1024),
            curr_length: 0,
        })
    }

    pub fn receive(&mut self) -> Result<DataRecvResult<ApplicationClientMessage>, Box<dyn Error>> {
        match self.connection_reader.read_line(&mut self.scratch) {
            Ok(_) => {
                if self.curr_length > 0 {
                    // We have a partial read ongoing, push into partial message
                    self.curr_message.push_str(&self.scratch);
                } else {
                    self.curr_message = self.scratch.clone();
                }
                self.curr_length = 0;
            },
            Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => {
                if self.scratch.len() > 0 {
                    self.curr_length = self.scratch.len();
                    self.curr_message = self.scratch.clone();
                    self.scratch.clear();
                    return Ok(DataRecvResult::Partial);
                } else {
                    return Ok(DataRecvResult::Idle);
                }
            }
            Err(err) => return Err(err.into())
        };

        Ok(DataRecvResult::Complete(serde_json::from_str(&self.curr_message)?))
    }

    pub fn send(&mut self, message: ApplicationServiceMessage) -> Result<(), Box<dyn Error>> {
        let message = serde_json::to_string(&message)?;
        self.connection_reader.get_mut().write(&(message.into_bytes()))
            .map(|_| ())
            .map_err(|e| e.into())
    }
}

pub const READ_LOOP_LIMIT: usize = 2048;

pub struct ApplicationService {
    listener_socket: Mutex<Option<TcpListener>>,
    awaiting_applications: Mutex<Vec<ApplicationConnection>>,
    applications: Mutex<HashMap<ApplicationID, ApplicationConnection>>,
    camera_service: Option<Arc<Mutex<CameraService>>>,
    pub appmsg_tx_queue: Mutex<VecDeque<ApplicationPacket>>,
    pub appmsg_rx_queue: Mutex<VecDeque<ApplicationPacket>>
}

impl ApplicationService {
    pub fn new() -> Self {
        ApplicationService {
            listener_socket: Mutex::new(None),
            awaiting_applications: Mutex::new(Vec::new()),
            applications: Mutex::new(HashMap::new()),
            camera_service: None,
            appmsg_tx_queue: Mutex::new(VecDeque::new()),
            appmsg_rx_queue: Mutex::new(VecDeque::new())
        }
    }

    pub fn with_camera(camera: Arc<Mutex<CameraService>>) -> Self {
        ApplicationService {
            listener_socket: Mutex::new(None),
            awaiting_applications: Mutex::new(Vec::new()),
            applications: Mutex::new(HashMap::new()),
            camera_service: Some(camera),
            appmsg_tx_queue: Mutex::new(VecDeque::new()),
            appmsg_rx_queue: Mutex::new(VecDeque::new())
        }
    }

    pub fn listen(&self, port: u16) -> Result<(), io::Error> {
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        listener.set_nonblocking(true)?;
        *self.listener_socket.lock() = Some(listener);
        Ok(())
    }

    pub fn process_messages(&self) {
        let mut loops = 0;
        let mut consecutive_idle_loops = 0;
        let mut is_idle_loop;

        let apps = self.applications.lock();

        if let Some(listener) = self.listener_socket.lock().deref() {
            for incoming_connection in listener.incoming() {
                match incoming_connection {
                    Ok(conn) => {
                        let application = match ApplicationConnection::from_connection(conn) {
                            Ok(app) => app,
                            Err(err) => {
                                error!("Error while registering application: {}", err);
                                continue;
                                // Application connection is automatically dropped; socket is closed.
                            }
                        };
                        self.awaiting_applications.lock().push(application);
                    },
                    Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {},
                    Err(e) => {
                        error!("error on application listening socket: {}", e)
                    }
                }
                break;
            }
        } else {
            return;
        }

        if apps.is_empty() {
            return;
        }

        let mut indexes_to_move: Vec<(usize, RequestID, ApplicationID)> = Vec::new();
        let mut indexes_to_remove = Vec::new();

        while loops < READ_LOOP_LIMIT && !(consecutive_idle_loops > 3) {
            is_idle_loop = true;

            {
                let mut await_apps = self.awaiting_applications.lock();

                'inner: for (idx, conn) in await_apps.deref_mut().iter_mut().enumerate() {
                    let message = match conn.receive() {
                        Ok(result) => match result {
                            DataRecvResult::Complete(data) => {
                                is_idle_loop = false;
                                consecutive_idle_loops = 0;
                                data
                            },
                            DataRecvResult::Partial => {
                                is_idle_loop = false;
                                consecutive_idle_loops = 0;
                                continue 'inner
                            },
                            DataRecvResult::Idle => continue 'inner
                        },
                        Err(err) => {
                            error!("Error reading message from awaiting application: {}", err);
                            indexes_to_remove.push(idx);
                            continue;
                        }
                    };

                    let message_id = message.id;
                    match message.message_type {
                        ApplicationClientMessageType::RegisterApplication { id } => {
                            indexes_to_move.push((idx, message_id, id));
                        },
                        _ => {
                            warn!("Illegal message from awaiting application!");
                        }
                    };
                }
                if !indexes_to_remove.is_empty() {
                    for idx in &indexes_to_remove {
                        await_apps.remove(*idx);
                    }
                    indexes_to_remove.clear();
                }

                if !indexes_to_move.is_empty() {
                    let mut apps = self.applications.lock();
                    for (idx, message_id, id) in &indexes_to_move {
                        if apps.contains_key(id) {
                            warn!("Awaiting application attempted to impersonate application with ID {}", id);
                            let conn_temp = await_apps.get_mut(*idx).unwrap();
                            match conn_temp.send(ApplicationServiceMessage::nok(*message_id)) {
                                Ok(_) => {},
                                Err(err) => {
                                    error!("Failed to send failure status for impersonating application for ID {}: {}", id, err);
                                }
                            }
                        } else {
                            let mut conn = await_apps.remove(*idx);
                            match conn.send(ApplicationServiceMessage::ok(*message_id)) {
                                Ok(_) => {
                                    apps.insert(*id, conn);
                                },
                                Err(err) => {
                                    error!("Error sending application confirmation to NEW application ID {}: {}", id, err);
                                    // conn is then dropped
                                }
                            }
                        }
                    }
                    indexes_to_move.clear();
                }
            }

            self.applications.lock().deref_mut().retain(|id, conn| {
                let message = match conn.receive() {
                    Ok(result) => match result {
                        DataRecvResult::Complete(data) => {
                            is_idle_loop = false;
                            consecutive_idle_loops = 0;
                            data
                        },
                        DataRecvResult::Partial => {
                            is_idle_loop = false;
                            consecutive_idle_loops = 0;
                            return true;
                        },
                        DataRecvResult::Idle => return true
                    },
                    Err(err) => {
                        error!("Error reading message from application ID {}: {}", id, err);
                        return false;
                    }
                };

                let message_id = message.id;
                match message.message_type {
                    ApplicationClientMessageType::RequestCameraPicture => {
                        if let Some(camera_service_lock) = &self.camera_service {
                            let camera_service = camera_service_lock.lock();
                            let path = PathBuf::from(format!("/tmp/{}.jpg", id));
                            match camera_service.take_capture(path.clone()) {
                                Ok(_) => {
                                    match conn.send(ApplicationServiceMessage::camera_picture(path)) {
                                        Ok(_) => {},
                                        Err(err) => {
                                            error!("Failed to send camera image path to ID {}: {}", id, err);
                                        }
                                    }
                                },
                                Err(err) => {
                                    error!("Error taking camera capture for application ID {}: {}", id, err);
                                    match conn.send(ApplicationServiceMessage::nok(message_id)) {
                                        Ok(_) => {},
                                        Err(err) => {
                                            error!("Failed to send failure status for ID {}: {}", id, err);
                                        }
                                    }
                                }
                            }
                        } else {
                            match conn.send(ApplicationServiceMessage::nok(message_id)) {
                                Ok(_) => {},
                                Err(err) => {
                                    error!("Error sending message to application ID {}: {}", id, err);
                                    return false;
                                }
                            }
                        }
                    },
                    ApplicationClientMessageType::SendApplicationPacket(data) => {
                        let mut tx_queue = self.appmsg_tx_queue.lock();
                        tx_queue.push_back(ApplicationPacket::new(id.into(), data));
                    },
                    _ => {
                        match conn.send(ApplicationServiceMessage::nok(message_id)) {
                            Ok(_) => {},
                            Err(err) => {
                                error!("Error sending message to application ID {}: {}", id, err);
                                return false;
                            }
                        }
                    }
                }

                return true;
            });

            {
                // TODO implement packet sending logic
            }

            if is_idle_loop {
                consecutive_idle_loops += 1;
            }

            loops += 1;
        }
    }
}
