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

///
/// Represents an unique ID that identifies a single application running on the drone and
/// communicating with the ICAROS system
#[derive(Debug, Copy, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(from = "u8")]
pub struct ApplicationID(u8);

impl From<u8> for ApplicationID {
    /// Creates an application ID from the given integer value
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

///
/// Represents an unique ID that identifies a single request received from a given application
/// running on the drone and communicating with the ICAROS system
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[serde(into = "u64", from = "u64")]
pub struct RequestID(u64);

impl From<u64> for RequestID {
    /// Creates an application ID from the given integer value
    fn from(value: u64) -> Self {
        RequestID(value)
    }
}

impl From<RequestID> for u64 {
    fn from(value: RequestID) -> Self {
        value.0
    }
}

///
/// The result of attempting to receive a certain amount of data of type T through the TCP socket
pub enum DataRecvResult<T> {
    /// Complete piece of data with type T received
    Complete(T),

    /// Partial piece of data with type T received
    Partial,

    /// No data currently being received
    Idle
}


///
/// A complete message sent through the TCP socket by the ICAROS system itself to the application
#[derive(Debug, Serialize)]
#[serde(tag = "type", content = "data")]
#[allow(unused)]
pub enum ApplicationServiceMessage {
    /// The operation done with the given request ID has succeeded
    #[serde(rename = "ok")]
    GeneralOperationOk { msg_id: RequestID },

    /// The operation done with the given request ID has failed
    #[serde(rename = "not_ok")]
    GeneralOperationNotOk { msg_id: RequestID },

    /// The camera picture was taken, and is available for reading at the given filesystem path
    #[serde(rename = "camera_pic")]
    CameraPicture { path: PathBuf },

    /// The application has received a message from ground control
    #[serde(rename = "message")]
    ReceiveApplicationPacket(Vec<u8>)
}

impl ApplicationServiceMessage {
    /// Create an "operation completed successfully" message for the given request ID
    pub fn ok(rid: RequestID) -> Self {
        Self::GeneralOperationOk { msg_id: rid }
    }

    /// Create an "operation not completed successfully" message for the given request ID
    pub fn nok(rid: RequestID) -> Self {
        Self::GeneralOperationNotOk { msg_id: rid }
    }

    /// Create an "camera picture taken" message with the given image path
    pub fn camera_picture(path: PathBuf) -> Self {
        Self::CameraPicture { path }
    }

    /// Create a "received application packet" message with the given bitstream
    #[allow(unused)]
    pub fn app_msg(data: Vec<u8>) -> Self {
        Self::ReceiveApplicationPacket(data)
    }
}


///
/// The payload type of a complete message sent through the TCP socket by an application to the ICAROS system
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum ApplicationClientMessageType {
    /// Register application with ID
    #[serde(rename = "register_application")]
    RegisterApplication { id: ApplicationID },

    /// Request camera picture
    #[serde(rename = "req_camera_pic")]
    RequestCameraPicture,

    /// Send application packet to ground control
    #[serde(rename = "message")]
    SendApplicationPacket(Vec<u8>)
}

///
/// A complete message sent through the TCP socket by an application to the ICAROS system
#[derive(Debug, Serialize, Deserialize)]
pub struct ApplicationClientMessage {
    /// The request ID of the message
    id: RequestID,

    /// The message type and payload
    #[serde(flatten)]
    message_type: ApplicationClientMessageType
}

///
/// The TCP connection and associated state data between an application and the ICAROS system
pub struct ApplicationConnection {
    /// The TCP connection, wrapped on a buffered reader
    connection_reader: BufReader<TcpStream>,

    /// Scratch memory space to store the packet being received, in order to avoid reallocations.
    scratch: String,

    /// The current packet being received
    curr_message: String,

    /// The current length of the packet currently being received
    curr_length: usize
}

impl ApplicationConnection {
    /// Creates an application connection from the given open TCP stream
    pub fn from_connection(connection: TcpStream) -> Result<Self, io::Error> {
        // The TCP read timeout is set to an appropiate value in order to avoid having single
        // applications saturate the entire system
        connection.set_read_timeout(Some(Duration::from_micros(200)))?;

        Ok(ApplicationConnection {
            connection_reader: BufReader::new(connection),
            scratch: String::with_capacity(1024),
            curr_message: String::with_capacity(1024),
            curr_length: 0,
        })
    }

    /// Attempts to receive new structured data from the socket
    pub fn receive(&mut self) -> Result<DataRecvResult<ApplicationClientMessage>, Box<dyn Error>> {
        // Attempt to read a single line from the connection into `self.scratch`
        match self.connection_reader.read_line(&mut self.scratch) {
            Ok(_) => { // Success
                if self.curr_length > 0 {
                    // We have a partial read ongoing, push into partial message
                    self.curr_message.push_str(&self.scratch);
                } else {
                    // We have received the entire message in a single piece; just copy the
                    // received string into the partial message
                    self.curr_message = self.scratch.clone();
                }

                // Clear the scratch string buffer
                self.scratch.clear();

                // Reset the length of the current partial message
                self.curr_length = 0;

                // We may have received a complete message; try to parse it and return it
                Ok(DataRecvResult::Complete(serde_json::from_str(&self.curr_message)?))
            },
            Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => { // Not enough data received
                // If at least some data was received,
                if self.scratch.len() > 0 {
                    // ...and we have a partial read already ongoing
                    if self.curr_length > 0 {
                        // We push the additional received data
                        self.curr_length += self.scratch.len();
                        self.curr_message.push_str(&self.scratch);
                    } else { //... or not ...
                        // We copy the newly received data
                        self.curr_length = self.scratch.len();
                        self.curr_message = self.scratch.clone();
                    }

                    // We inform the caller of the current state
                    Ok(DataRecvResult::Partial)
                } else {
                    // Nothing was received, for now!
                    Ok(DataRecvResult::Idle)
                }
            },
            Err(err) => Err(err.into()) // Another error was stumbled upon
        }
    }

    /// Sends a message back to the application
    pub fn send(&mut self, message: ApplicationServiceMessage) -> Result<(), Box<dyn Error>> {
        // We serialize the message
        let message = serde_json::to_string(&message)?;

        // Then, we send it down the wire
        self.connection_reader.get_mut().write(&(message.into_bytes()))
            .map(|_| ())
            .map_err(|e| e.into())
    }
}

/// The amount of loops that will be done within a single iteration to attempt to read messages
pub const READ_LOOP_LIMIT: usize = 2048;

/// The application service
///
/// Part of the external services, this (thread-safe) service is responsible for handling the
/// communication with secondary applications running within the drone's hardware, and
/// to provide them a simple API to certain hardware facilities built into the drone,
/// such as the camera.
pub struct ApplicationService {
    /// The listening socket for applications to connect
    listener_socket: Mutex<Option<TcpListener>>,

    /// A vector containing applications awaiting to be registered
    awaiting_applications: Mutex<Vec<ApplicationConnection>>,

    /// A key-value store containing registered applications
    applications: Mutex<HashMap<ApplicationID, ApplicationConnection>>,

    /// A handle to the camera service
    camera_service: Option<Arc<Mutex<CameraService>>>,

    /// A queue where outgoing messages are inserted
    /// Messages to be sent to the ground control are obtained by popping elements off this queue
    pub appmsg_tx_queue: Mutex<VecDeque<ApplicationPacket>>,

    /// A queue where incoming messages are read from
    /// Messages received from the ground control are pushed onto this queue
    pub appmsg_rx_queue: Mutex<VecDeque<ApplicationPacket>>
}

impl ApplicationService {
    /// Create a new application service with no camera support
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

    /// Create a new application service with camera support, and the given camera service
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

    /// Start listening for application connections on the given port
    pub fn listen(&self, port: u16) -> Result<(), io::Error> {
        // Bind a network socket to the given port in the loopback interface
        let listener = TcpListener::bind(("127.0.0.1", port))?;

        // Make this socket non-blocking (otherwise, it would freeze the entire ESB until a
        // connection was estabilished)
        listener.set_nonblocking(true)?;

        // Store it in our own state
        *self.listener_socket.lock() = Some(listener);

        Ok(())
    }

    /// Process the currently-pending application messages
    pub fn process_messages(&self) {
        let mut loops = 0;
        let mut consecutive_idle_loops = 0;
        let mut is_idle_loop;

        // Get a hold of the applications key-value store for the entire duration of the function
        let mut apps = self.applications.lock();
        let mut await_apps = self.awaiting_applications.lock();

        // If we have a socket listening for connections...
        if let Some(listener) = self.listener_socket.lock().deref() {
            // Handle incoming connections
            for incoming_connection in listener.incoming() {
                match incoming_connection {
                    Ok(conn) => { // Connection successful
                        // Create an application connection structure from the new connection
                        let application = match ApplicationConnection::from_connection(conn) {
                            Ok(app) => app,
                            Err(err) => {
                                error!("Error while registering application: {}", err);
                                continue;
                                // Application connection is automatically dropped; socket is closed.
                            }
                        };

                        // Push it into the awaiting applications
                        await_apps.push(application);
                    },
                    Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {}, // No connections done
                    Err(e) => { // Some other error happened
                        error!("Error on application listening socket: {}", e)
                    }
                }
                break;
            }
        } else {
            // Otherwise, we have nothing to do; return immediately
            return;
        }

        // If we have no apps to attend to, return immediately
        if apps.is_empty() && await_apps.is_empty() {
            return;
        }

        let mut indexes_to_move: Vec<(usize, RequestID, ApplicationID)> = Vec::new();
        let mut indexes_to_remove = Vec::new();

        // While we haven't reached the maximum limit of loops, and we haven't been idiling for
        // more than 3 iterations...
        while loops < READ_LOOP_LIMIT && !(consecutive_idle_loops > 3) {
            is_idle_loop = true;

            // We handle the registrations for applications pending one
            {
                // For each pending application...
                'inner: for (idx, conn) in await_apps.deref_mut().iter_mut().enumerate() {
                    // Receive messages from the application's socket
                    let message = match conn.receive() {
                        Ok(result) => match result {
                            DataRecvResult::Complete(data) => { // Complete message
                                is_idle_loop = false;
                                consecutive_idle_loops = 0;
                                data
                            },
                            DataRecvResult::Partial => { // Partial message
                                is_idle_loop = false;
                                consecutive_idle_loops = 0;
                                continue 'inner
                            },
                            DataRecvResult::Idle => continue 'inner // No message
                        },
                        Err(err) => { // Error receiving message
                            error!("Error reading message from awaiting application: {}", err);
                            indexes_to_remove.push(idx); // We queue the socket for disconnection
                            continue 'inner;
                        }
                    };

                    // We parse the message
                    let message_id = message.id;
                    match message.message_type {
                        ApplicationClientMessageType::RegisterApplication { id } => {
                            // Received a valid application registering message!
                            // Queue it for upgrade into registered application
                            indexes_to_move.push((idx, message_id, id));
                        },
                        _ => {
                            // Received any other message; we ignore it
                            warn!("Illegal message from awaiting application!");
                        }
                    };
                }

                // If sockets pending registration are queued for deletion, we loop through them
                // and simply remove them from our vector; this will cause them to drop out of
                // scope and automatically shut down and close.
                if !indexes_to_remove.is_empty() {
                    for &idx in &indexes_to_remove {
                        await_apps.remove(idx);
                        // The removed socket will be dropped here; this will shut it down
                        // automatically with no extra logic.
                    }
                    indexes_to_remove.clear();
                }

                // If sockets pending registration are queued for upgrading into registered
                // applications, we handle them
                if !indexes_to_move.is_empty() {
                    // For each application for which a registration request has been received...
                    for &(idx, message_id, id) in &indexes_to_move {
                        // If we have already an application registered with the ID the application
                        // wants to get, we have a problem.
                        if apps.contains_key(&id) {
                            warn!("Awaiting application attempted to impersonate application with ID {}", id);
                            let conn_temp = await_apps.get_mut(idx).unwrap();
                            // We let the application know this.
                            match conn_temp.send(ApplicationServiceMessage::nok(message_id)) {
                                Ok(_) => {},
                                Err(err) => {
                                    error!("Failed to send failure status for impersonating application for ID {}: {}", id, err);
                                }
                            }
                        } else {
                            // Otherwise, we are fine; we proceed to upgrade the connection
                            // and acknowledge the successful upgrade to the application
                            let mut conn = await_apps.remove(idx);
                            match conn.send(ApplicationServiceMessage::ok(message_id)) {
                                Ok(_) => {
                                    // Success! We finish the operation by storing it into our
                                    // key-value storage
                                    apps.insert(id, conn);
                                },
                                Err(err) => { // We have an error sending an acknowledgement
                                    error!("Error sending application confirmation to NEW application ID {}: {}", id, err);
                                    // Connection is automatically dropped and disconnected
                                }
                            }
                        }
                    }
                    indexes_to_move.clear();
                }
            }

            // For each connection...
            // (within the following closure, returning true will make the connection persist,
            // whereas returning false will automatically remove the connection from the
            // key-value store and drop it automatically, therefore shutting down the socket)
            // (see: https://doc.rust-lang.org/std/collections/struct.HashMap.html#method.retain)
            apps.deref_mut().retain(|id, conn| {
                // Receive messages from the application's socket
                let message = match conn.receive() {
                    Ok(result) => match result {
                        DataRecvResult::Complete(data) => { // Complete message
                            is_idle_loop = false;
                            consecutive_idle_loops = 0;
                            data
                        },
                        DataRecvResult::Partial => { // Partial message
                            is_idle_loop = false;
                            consecutive_idle_loops = 0;
                            return true;
                        },
                        DataRecvResult::Idle => return true // No message
                    },
                    Err(err) => { // Error receiving message
                        error!("Error reading message from application ID {}: {}", id, err);
                        return false; // Connection is dropped
                    }
                };

                // We match and act based on the received message type
                let message_id = message.id;
                match message.message_type {
                    ApplicationClientMessageType::RequestCameraPicture => { // Camera picture request
                        // If we have a camera service available...
                        if let Some(camera_service_lock) = &self.camera_service {
                            // We get a hold of it
                            let camera_service = camera_service_lock.lock();

                            // Create a filesystem path for the new file from the application ID
                            let path = PathBuf::from(format!("/tmp/{}.jpg", id));

                            // Then, ask the camera service for a capture
                            match camera_service.take_capture(path.clone()) {
                                Ok(_) => { // Success, we send the path of the new file back
                                    match conn.send(ApplicationServiceMessage::camera_picture(path)) {
                                        Ok(_) => {},
                                        Err(err) => {
                                            error!("Failed to send camera image path to ID {}: {}", id, err);
                                            return false; // Connection is dropped
                                        }
                                    }
                                },
                                Err(err) => { // Failure, we let the application know
                                    error!("Error taking camera capture for application ID {}: {}", id, err);
                                    match conn.send(ApplicationServiceMessage::nok(message_id)) {
                                        Ok(_) => {},
                                        Err(err) => {
                                            error!("Failed to send failure status for ID {}: {}", id, err);
                                            return false; // Connection is dropped
                                        }
                                    }
                                }
                            }
                        } else { // Otherwise, we let the application know it's not possible
                            match conn.send(ApplicationServiceMessage::nok(message_id)) {
                                Ok(_) => {},
                                Err(err) => {
                                    error!("Error sending message to application ID {}: {}", id, err);
                                    return false; // Connection is dropped
                                }
                            }
                        }
                    },
                    ApplicationClientMessageType::SendApplicationPacket(data) => { // Application message send
                        // We push it into the outgoing queue for ground control messages
                        let mut tx_queue = self.appmsg_tx_queue.lock();
                        tx_queue.push_back(ApplicationPacket::new(id.into(), data));
                    },
                    _ => { // Any other unimplemented message --> Not OK
                        match conn.send(ApplicationServiceMessage::nok(message_id)) {
                            Ok(_) => {},
                            Err(err) => {
                                error!("Error sending message to application ID {}: {}", id, err);
                                return false; // Connection is dropped
                            }
                        }
                    }
                }

                return true; // Connection is retained otherwise
            });

            {
                // NOT IMPLEMENTED --- ground control to application packet sending
            }

            // If there hasn't been any activity on this loop, increase the number of counted idle loops
            if is_idle_loop {
                consecutive_idle_loops += 1;
            }

            // Increase the number of total loops made
            loops += 1;
        }
    }
}
