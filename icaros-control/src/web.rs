//! Main webserver thread logic
//!
//! Unlike the rest of the code, this thread runs on the Tokio asyncronomous event loop
//! for better performance and semantics for the task.
//!
//! To handle the web side itself, the `warp` web server library is used. Three endpoints
//! are exposed from it:
//! - `/feed`, which represents our `core` WebSocket endpoint, and streams the main system state
//!   and telemetry to connected users.
//! - `/video`, which represents our `video` WebSocket endpoint, and streams the live video feed
//!   received from the drone through WifiBroadcast.
//! - A catch-all endpoint, which serves the front-end side of the web application.

use std::error::Error;
use crate::web::video::{separate_nal, VideoControlPayload};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::collections::HashMap;
use warp::ws::Message;
use warp::{ws, Filter};
use parking_lot::Mutex;
use std::sync::Arc;
use tokio::sync::mpsc;
use crate::core::GroundSystemState;
use parking_lot::RwLock;
use core::future::Future;
use futures::{future, FutureExt, StreamExt, SinkExt};
use serde_json;
use std::process::Stdio;
use tokio_util::codec::FramedRead;

mod video;

/// A bare-bones key-value store of WebSocket sockets, mapped by user ID
type Users = Mutex<HashMap<usize, WebsocketSender>>;

/// Message-passing interface for WebSocket messages behind which is the WebSocket socket
/// Messages for WebSocket clients are written through here.
type WebsocketSender = mpsc::UnboundedSender<Result<Message, warp::Error>>;

/// Kinds of signals the main system can send to the webserver
pub enum ControllerRXWebserverMessage {
    Update
}

/// Kinds of signals the webserver can send to the main system
pub enum ControllerTXWebserverMessage {}

/// The complete state of a single WebSocket endpoint.
///
/// This includes the connected clients and the user ID that will be assigned
/// to the next client to connect.
pub struct WebsocketEndpoint {
    /// Connected clients
    pub ws_users: Users,

    /// User ID of the next client to connect
    pub ws_nextuid: AtomicUsize
}

impl WebsocketEndpoint {
    /// Create a new state structure for a WebSocket endpoint
    pub fn new() -> Self {
        WebsocketEndpoint {
            ws_users: Mutex::new(HashMap::new()),
            ws_nextuid: AtomicUsize::new(1)
        }
    }

    /// Broadcast a text message to all clients connected to the endpoint.
    ///
    /// Returns either `Ok(true)` for a successful send to all clients, or a list of errors
    /// ocurred.
    pub fn broadcast_text<T: AsRef<str>>(&self, msg: T) -> Result<bool, Vec<Box<dyn Error>>> {
        // Take a hold of our users store
        let users = &*self.ws_users.lock();

        // By iterating through all users...
        let ret: (Vec<_>, Vec<_>) = users.iter().map(|(_, user)| {
            // ...send each user the message
            user.send(Ok(Message::text(msg.as_ref()))).map(|_| true).map_err(|e| e.into())
        // This gives us a list of successes and failures
        }).partition(Result::is_ok); // ...which we partition into two vectors

        // The first one (ret.0) contains all successes, whereas the second one (ret.1)
        // contains all failures.
        // Thus, if there are any error results...
        if ret.1.len() > 0 {
            // We extract the errors themselves out of the `Result`, and return them
            let err: Vec<_> = ret.1.into_iter().map(Result::unwrap_err).collect();
            Err(err)
        } else {
            // Otherwise, all the clients were sent the message successfully
            Ok(true)
        }
    }

    /// Broadcasts a binary message to all clients connected to the endpoint.
    ///
    /// Returns either `Ok(true)` for a successful send to all clients, or a list of errors
    /// ocurred.
    pub fn broadcast_binary(&self, msg: Vec<u8>) -> Result<bool, Vec<Box<dyn Error>>> {
        // Take a hold of our users store
        let users = &*self.ws_users.lock();

        // By iterating through all users...
        let ret: (Vec<_>, Vec<_>) = users.iter().map(|(_, user)| {
            // ...send each user the message
            user.send(Ok(Message::binary(msg.as_slice()))).map(|_| true).map_err(|e| e.into())
        // This gives us a list of successes and failures
        }).partition(Result::is_ok);// ...which we partition into two vectors

        // The first one (ret.0) contains all successes, whereas the second one (ret.1)
        // contains all failures.
        // Thus, if there are any error results...
        if ret.1.len() > 0 {
            // We extract the errors themselves out of the `Result`, and return them
            let err: Vec<_> = ret.1.into_iter().map(Result::unwrap_err).collect();
            Err(err)
        } else {
            // Otherwise, all the clients were sent the message successfully
            Ok(true)
        }
    }
}

///
/// Contains the two main WebSocket endpoints, `core` and `video`.
pub struct Websockets {
    /// `core` WebSocket endpoint
    pub core_ws: Arc<WebsocketEndpoint>,

    /// `video` WebSocket endpoint
    pub video_ws: Arc<WebsocketEndpoint>,
}

impl Websockets {
    /// Creates the main websocket endpoints' state structures
    pub fn new_base() -> Self {
        Websockets {
            core_ws: Arc::new(WebsocketEndpoint::new()),
            video_ws: Arc::new(WebsocketEndpoint::new())
        }
    }
}

///
/// The complete state of the webserver
///
/// It holds shared, syncronized references of the ground system state,
/// the websocket endpoints and the sending end of a message-passing interface with the
/// main system.
pub struct WebserverSharedData {
    /// Shared reference to the ground system state
    pub system_state: Arc<RwLock<GroundSystemState>>,

    /// Websocket endpoints
    pub ws: Websockets,

    /// Message-passing interface sender to the main system
    pub core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>,

    /// Whether the video stream is active at the moment (i.e. data has been received)
    pub video_stream_active: Arc<Mutex<bool>>
}

impl WebserverSharedData {
    /// Creates a webserver state structure from a shared reference to the ground
    /// system state and the sending end of a message-passing interface with the
    /// main system.
    pub fn from_core(
        system_state: Arc<RwLock<GroundSystemState>>,
        core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>
    ) -> Self {
        WebserverSharedData {
            system_state,
            ws: Websockets::new_base(),
            core_tx,
            video_stream_active: Arc::new(Mutex::new(false)),
        }
    }

    /// Sets up the `core` WebSocket endpoint with the logic to accept users
    /// and serve requests.
    fn setup_core(&mut self, core_rx: mpsc::UnboundedReceiver<ControllerRXWebserverMessage>) {
        let ws_users_local = self.ws.core_ws.clone();
        let sys_state_local = self.system_state.clone();

        // Spawn a new asyncronomous task
        // This task will collect signals that we receive from the main system
        // as they come, and perform different actions depending on the passed signal
        tokio::spawn(core_rx.for_each(move |msg| {
            // Match the signal received
            match msg {
                ControllerRXWebserverMessage::Update => { // The main system's state got updated
                    let sys_state_json: String;

                    { // Scope lock begin
                        // Get a hold of the (updated) ground system state
                        let sys_state = &*sys_state_local.read();

                        // Serialize the ground system state into JSON
                        sys_state_json = match serde_json::to_string(sys_state) {
                            Ok(json) => json,
                            Err(e) => {
                                // If failed, just go back to processing signals
                                error!("Error while processing update core -> web: {}", e);
                                return future::ready(())
                            }
                        }
                    } // Scope lock end, our hold to sys_state is dropped

                    // We broadcast the serialized state to all our clients
                    match ws_users_local.broadcast_text(sys_state_json) {
                        Ok(_) => { // Scope lock begin
                            // After doing so, we get another hold of the ground system state...
                            let mut sys_state = sys_state_local.write();

                            // ...and clear the incoming application messages queue
                            sys_state.rx_app_messages.clear();
                        }, // Scope lock end, our hold to sys_state is dropped
                        Err(e) => {
                            // Print all the errors gotten
                            for err in e {
                                error!("Error while sending update core -> web to ws: {}", err);
                            }
                        }
                    }

                    // We go back to processing signals
                    future::ready(())
                },
                //_ => future::ready(())
            }
        }));
    }

    /// Sets up the `video` WebSocket endpoint with the logic to accept users
    /// serve a video feed from WifiBroadcast.
    fn setup_video(&mut self) -> Result<(), std::io::Error> {
        let ws_users_local = self.ws.video_ws.clone();

        // Initialize the command to spawn a new child process
        let mut cmd = tokio::process::Command::new("./rx");
        cmd.args( // ...with the given command-line arguments
            &["-p", "0", "-b", "4", "-r", "8", "-f", "1100", "wlan1"]
        );
        cmd.stdout(Stdio::piped()); // And piping the standard output (i.e. received data)
                                    // back to us instead to the terminal.

        // Create the child process from the command we created
        let mut child = cmd.spawn()?;

        // Get a hold of the child process' standard output, and filter it through
        // the H.264 NAL unit separator.
        let stdout = child.stdout.take().ok_or(std::io::ErrorKind::NotConnected)?;
        let stdout_buffered = FramedRead::new(stdout, separate_nal());

        // Obtain a shared reference to our video stream active flag
        let video_stream_active_flag = self.video_stream_active.clone();

        // Spawn a new asyncronomous task
        tokio::spawn(async move {
            // This task will:
            // - Spawn another task which will `await` the child process until it exits.
            //   This is necessary in asyncronomous Rust programming, as the child process
            //   won't actually spawn (!!) until we `await` it.
            //   However, `await`ing in this case would, by design, block the program
            //   execution until the process exited; thus, we `await` the child process
            //   on its own separate task so our program can keep running.
            tokio::spawn(async {
                match child.await {
                    Ok(status) => info!("WifiBroadcast exited with status code {}", status),
                    Err(err) => error!("WifiBroadacast encountered an error: {}", err)
                };
            });

            // - Then, it will start continuously reading the stream of processed
            //   (i.e. NAL-separated) H.264 frames (each represented by a byte vector)
            //   and broadacst the frames receivied to all the clients.
            stdout_buffered.for_each(|frame_result| {
                match frame_result {
                    Ok(frame) => { // Got a successful read of a frame
                        { // Scope lock begin
                            // Take a hold on our video stream active flag
                            let mut video_stream_active = video_stream_active_flag.lock();

                            // If the video stream is not yet marked as inactive...
                            if !*video_stream_active {
                                // ...we mark it as active...
                                *video_stream_active = true;

                                // Then, we serialize and broadacast a message letting all
                                // clients know the stream is now active
                                // (crafting and serializing)
                                let msg = match serde_json::to_string(&VideoControlPayload::StreamActive(true)) {
                                    Ok(json) => json,
                                    Err(e) => {
                                        error!("Error while processing update video -> web: {}", e);
                                        return future::ready(())
                                    }
                                };

                                // (broadcasting)
                                match ws_users_local.broadcast_text(msg) {
                                    Ok(_) => {},
                                    Err(e) => {
                                        for err in e {
                                            error!("Error while sending update video -> web to ws: {}", err);
                                        }
                                        return future::ready(())
                                    }
                                };
                            }
                        } // Scope lock end

                        // Broadcast our video frame to all clients
                        match ws_users_local.broadcast_binary(frame) {
                            Ok(_) => {},
                            Err(e) => {
                                for err in e {
                                    error!("Error while sending update video -> web to ws: {}", err);
                                }
                                return future::ready(())
                            }
                        };
                    },
                    Err(err) => error!("Video RX read error: {}", err)
                };
                future::ready(())
            }).await;
        });

        Ok(())
    }
}

/// Websocket common event definitions (i.e. connect, disconnect)
mod ws_events {
    use warp::ws::WebSocket;
    use super::*;

    /// Event called when a WebSocket client connects.
    ///
    /// Receives the client connection itself, a shared reference to the websocket endpoint
    /// it has to be attached to and a callback that runs every time a message is
    /// received from the client.
    pub fn connect<F>(ws: WebSocket, shared_data: Arc<WebsocketEndpoint>, on_message: F) -> impl Future<Output = Result<(), ()>>
        where F: Fn(usize, Message, &Arc<WebsocketEndpoint>)
    {
        // Atomically increment the ID to be assigned to the next user (that is us).
        // The incremented ID will be the one we will get.
        let new_uid = shared_data.ws_nextuid.fetch_add(1, Ordering::Relaxed);

        info!("websocket client {} connected", new_uid);

        // Split the WebSocket into sending (TX) and receiving (RX) parts
        let (user_ws_tx, user_ws_rx) = ws.split();

        // Create a new message-passing interface. We will wrap the sending part
        // of the WebSocket with it for easier handling.
        let (tx, rx) = mpsc::unbounded_channel();

        // To do so, we will spawn a new asyncronomous task which will forward packets
        // on the receiving end of our message-passing interface to the sending part
        // of the WebSocket.
        // We'll then be able to use the sending end of our message-passing interface
        // to send packets through the WebSocket.
        tokio::task::spawn(rx.forward(user_ws_tx).map(|result| {
            if let Err(e) = result {
                error!("websocket send error: {}", e);
            }
        }));

        { // Scope lock begin
            // Insert the new client (specifically, the part of the socket that will
            // allow us to send packets to it) into our WebSocket client key-value store
            shared_data.ws_users.lock().insert(new_uid, tx);
        } // Scope lock end

        { // Scope lock begin
            // Obtain an extra shared reference to our websocket endpoint
            // (to pass to our disconect handler; the incoming message handler will
            // have the hold on the one passed to us through the function's arguments)
            let shared_data_disc = shared_data.clone();

            // Take all incoming messages from the WebSocket...
            user_ws_rx
                .for_each(move |msg| { // Call our incoming message handler as we receive them
                    on_message(new_uid, msg.unwrap(), &shared_data);
                    future::ready(())
                })
                .then(move |result| { // Then (i.e. when the WebSocket closes), call our
                                      // disconnection handler
                    disconnect(new_uid, &shared_data_disc);
                    future::ok(result)
                })
        } // Scope lock end
    }

    /// Event called when a WebSocket client disconnects.
    ///
    /// Receives the ID of the connection itself and a shared reference to the WebSocket
    /// endpoint it is attached to.
    pub fn disconnect(uid: usize, shared_data: &Arc<WebsocketEndpoint>) {
        info!("websocket client {} disconnected", uid);

        // We take a hold on our WebSocket endpoint and, using the corresponding ID,
        // remove the user from our database. As this will cause the corresponding
        // socket end to be dropped out of scope, and therefore destroyed, the connection
        // will be torn down from our side automatically once we do this.
        shared_data.ws_users.lock().remove(&uid);
    }
}

/// Shorthand macro to attach the client connection handler to a given endpoint
macro_rules! recv_messages {
    ($sock: expr, $shared: expr => $cb: expr) => {
        match ws_events::connect($sock, $shared.clone(), $cb).await {
            Ok(()) => {},
            Err(()) => { error!("Unknown error happened on connection") }
        };
        return ();
    }
}

///
/// Entry point for the web server thread.
///
/// Gets a shared reference to the system state and message-passing interfaces with the
/// main system.
#[tokio::main]
pub async fn webserver_main(
    system_state: Arc<RwLock<GroundSystemState>>,
    core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>,
    core_rx: mpsc::UnboundedReceiver<ControllerRXWebserverMessage>
) {
    // We start by initalizing our web server's data structures
    let mut data = WebserverSharedData::from_core(system_state, core_tx);

    // Then, we set up our WebSocket endpoints
    data.setup_core(core_rx);
    match data.setup_video() {
        Ok(_) => {},
        Err(err) => error!("Failure setting up video: {}", err)
    };

    // The way `warp` is configured to serve requests is by combining "filters" in order to
    // represent the various endpoints of the web server.
    //
    // Let's first add a route to serve our static files. This use case can be already provided
    //
    let static_route = warp::fs::dir("www");

    // The logic for the next endpoints we will make requires access to our web server's
    // data structures. To accomplish this, we can take advantage of the fact that filters
    // can also pass additional data down for the next filters in line to use.
    // Thus, we can build a filter that doesn't actually filter anything; but instead will
    // just return a shared reference to our web server's data structures.
    // We'll see how this comes into play later.

    // Wrap our web server's data structures into a shareable reference.
    let shared_data = Arc::new(data);

    // And create our filter...
    let shared_data_filter_feed = {
        // ...by getting a shared reference of our web server's data structures...
        let _data = shared_data.clone();

        warp::any() // ...and create a filter that matches anything (i.e. doesn't filter anything)
            .map(move || _data.clone()) // ...and compose it with a function that doesn't take
                                        // anything, but returns a new shared reference of
                                        // our web server's data structures.

        // This effectively gives us a fresh shared reference of our web server's data structures
        // every time a request is made that comes in contact with the filter, and passes it
        // down to our request handler functions at the end.
    };

    // With that said, we can now create our next endpoint: our `core` WebSocket endpoint
    // Filters are composed using `and(...)`, which rejects a request if the filter finds
    // the request unsuitable; otherwise passing it down the next filter.
    // At the end of the filter chain, `map(...)` takes requests that passed through
    // all the filters and handles them.

    // We create our `core` WebSocket endpoint...
    let stream_ws = warp::path("feed")    // ...accepting requests on "/path"...
        .and(warp::ws())                  // ...accepting only WebSocket connections...
                                          // (this consumes the requests and returns a WebSocket
                                          // server handling them; this technically means that the
                                          // WebSocket filter is the one handling the requests)
        .and(shared_data_filter_feed)     // ...(and matches anything, but returns a new shared
                                          // reference to our shared data structures)...
        .map(|ws: ws::Ws, shared: Arc<WebserverSharedData>| {
            // Finally, we are in our handler!
            // (which is technically not a request handler anymore; that's now the job of the
            // WebSocket filter)

            // Here, we can now attach event handlers to our WebSocket server.
            // We attach the event handler corresponding to new client connections received.
            ws.on_upgrade(async move |socket| {
                // And use it to wire each new connection up with our WebSocket endpoint logic
                recv_messages!(socket, shared.ws.core_ws => |_, _, _| {});
            })
        });

    // For the `video` WebSocket endpoint, we will need a similar filter as above, so we create it.
    let shared_data_filter_video = {
        let _data = shared_data.clone();
        warp::any().map(move || _data.clone())
    };

    // We create our `core` WebSocket endpoint...
    let video_ws = warp::path("video")    // ...accepting requests on "/video"...
        .and(warp::ws())                  // ...accepting only WebSocket connections...
        .and(shared_data_filter_video)    // ...(returns a new shared  reference to our
                                          // shared data structures)...
        .map(|ws: ws::Ws, shared: Arc<WebserverSharedData>| {
            // On our handler, we take a new shared reference of our webserver data
            let shared = shared.clone();

            // Then, for each client connection we receive...
            ws.on_upgrade(async move |mut socket| {
                let msg_init;
                let msg_stream_active;

                { // Scope lock begin
                    // We take a hold of our video stream active flag
                    let stream_active = *shared.video_stream_active.lock();

                    // Then, craft a message telling the newly-connected client to initialize
                    // the video stream with the given parameters
                    msg_init = match serde_json::to_string(&VideoControlPayload::Initialize {
                            width: 960,
                            height: 540,
                            stream_active
                        }) {
                        Ok(json) => json,
                        Err(e) => {
                            error!("Error while processing init message video -> web: {}", e);
                            socket.close().map(|_| ()).await;
                            return ();
                        }
                    };

                    // And another one signaling it whether the video stream is
                    // active or not from our flag
                    msg_stream_active = match serde_json::to_string(&VideoControlPayload::StreamActive(stream_active)) {
                        Ok(json) => json,
                        Err(e) => {
                            error!("Error while processing stream_active message video -> web: {}", e);
                            socket.close().map(|_| ()).await;
                            return ();
                        }
                    };
                } // Scope lock end

                // Then send both messages
                match socket.send(Message::text(msg_init)).await {
                    Ok(_) => {},
                    Err(err) => {
                        error!("Error while sending init video -> web to NEW ws: {}", err);
                        socket.close().map(|_| ()).await;
                        return ();
                    }
                };

                match socket.send(Message::text(msg_stream_active)).await {
                    Ok(_) => {},
                    Err(err) => {
                        error!("Error while sending stream_active video -> web to NEW ws: {}", err);
                        socket.close().map(|_| ()).await;
                        return ();
                    }
                };

                // And finally wire our new connection up with the rest of its endpoint logic
                recv_messages!(socket, shared.ws.video_ws => |_, _, _| {});
            })
        });

    // Finally, we combine our three endpoints together into a filter chain
    // which represents our entire web server.
    // We do this using `or(...)`, which works backwards from `and(...)`: if a request
    // is rejected on one of our endpoints, it will try it on the next one.
    let full_routes = video_ws.or(stream_ws).or(static_route);

    // With our full web server filter chain done, we can now start listening
    // for request using it.
    warp::serve(full_routes).run(([0, 0, 0, 0], 3030)).await;
}
