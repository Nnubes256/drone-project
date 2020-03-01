use crate::gamepad::ControlState;
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
use futures::{future, FutureExt, StreamExt};
use serde::Deserialize;
use serde_json;

mod video;

type WebsocketSender = mpsc::UnboundedSender<Result<Message, warp::Error>>;
type Users = Mutex<HashMap<usize, WebsocketSender>>;

pub enum ControllerRXWebserverMessage {
    Update
}

pub enum ControllerTXWebserverMessage {}

#[derive(Deserialize)]
#[serde(tag = "type")]
pub enum WebsocketCoreClientMessage {
    #[serde(rename = "gmp")]
    VirtualControllerState(ControlState)
}

pub struct WebsocketData {
    pub ws_users: Users,
    pub ws_nextuid: AtomicUsize
}

impl WebsocketData {
    pub fn new() -> Self {
        WebsocketData {
            ws_users: Mutex::new(HashMap::new()),
            ws_nextuid: AtomicUsize::new(1)
        }
    }
}

pub struct Websockets {
    pub core_ws: Arc<WebsocketData>,
    pub video_ws: Arc<WebsocketData>,
}

impl Websockets {
    pub fn new_base() -> Self {
        Websockets {
            core_ws: Arc::new(WebsocketData::new()),
            video_ws: Arc::new(WebsocketData::new())
        }
    }
}

pub struct WebserverSharedData {
    pub system_state: Arc<RwLock<GroundSystemState>>,
    pub ws: Websockets,
    pub core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>,
    pub virtual_gamepad_state: Option<Arc<Mutex<ControlState>>>
}

impl WebserverSharedData {
    pub fn from_core(
        system_state: Arc<RwLock<GroundSystemState>>,
        core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>
    ) -> Self {
        WebserverSharedData {
            system_state,
            ws: Websockets::new_base(),
            virtual_gamepad_state: Some(Arc::new(Mutex::new(ControlState::default()))),
            core_tx,
        }
    }

    fn setup_core(&mut self, core_rx: mpsc::UnboundedReceiver<ControllerRXWebserverMessage>) {
        let ws_users_local = self.ws.core_ws.clone();
        let sys_state_local = self.system_state.clone();
        tokio::task::spawn(core_rx.for_each(move |msg| {
            match msg {
                ControllerRXWebserverMessage::Update => {
                    let sys_state_json: String;
                    {
                        let sys_state = &*sys_state_local.read();
                        sys_state_json = match serde_json::to_string(sys_state) {
                            Ok(json) => json,
                            Err(e) => {
                                error!("Error while processing update core -> web: {}", e);
                                return future::ready(())
                            }
                        }
                    }
                    {
                        let users = &*ws_users_local.ws_users.lock();
                        for (uid, user) in users {
                            match user.send(Ok(Message::text(sys_state_json.clone()))) {
                                Ok(()) => {
                                    let mut sys_state = sys_state_local.write();
                                    sys_state.rx_app_messages.clear();
                                },
                                Err(e) => {
                                    error!("Error while sending update core -> web to ws uid {}: {}", uid, e);
                                    return future::ready(())
                                }
                            };
                        }
                    }
                    future::ready(())
                },
                _ => future::ready(())
            }
        }));
    }

    fn setup_video(&mut self) {
        let ws_users_local = self.ws.video_ws.clone();
        let sys_state_local = self.system_state.clone();

        // TODO command exec
    }
}

mod ws_events {
    use warp::ws::WebSocket;
    use super::*;

    pub fn connect<F>(ws: WebSocket, shared_data: Arc<WebsocketData>, on_message: F) -> impl Future<Output = Result<(), ()>>
        where F: Fn(usize, Message, &Arc<WebsocketData>)
    {
        let new_uid = shared_data.ws_nextuid.fetch_add(1, Ordering::Relaxed);

        info!("websocket client {} connected", new_uid);

        let (user_ws_tx, user_ws_rx) = ws.split();

        let (tx, rx) = mpsc::unbounded_channel();

        tokio::task::spawn(rx.forward(user_ws_tx).map(|result| {
            if let Err(e) = result {
                error!("websocket send error: {}", e);
            }
        }));

        {
            shared_data.ws_users.lock().insert(new_uid, tx);
        }

        {
            let shared_data_disc = shared_data.clone();


            user_ws_rx
                .for_each(move |msg| {
                    on_message(new_uid, msg.unwrap(), &shared_data);
                    future::ready(())
                })
                .then(move |result| {
                    disconnect(new_uid, &shared_data_disc);
                    future::ok(result)
                })
        }
    }

    pub fn disconnect(uid: usize, shared_data: &Arc<WebsocketData>) {
        info!("websocket client {} disconnected", uid);

        // Stream closed up, so remove from the user list
        shared_data.ws_users.lock().remove(&uid);
    }
}

macro_rules! recv_messages {
    ($sock: expr, $shared: expr => $cb: expr) => {
        return ws_events::connect($sock, $shared.clone(), $cb).map(|result| result.unwrap())
    }
}

#[tokio::main]
pub async fn webserver_main(
    system_state: Arc<RwLock<GroundSystemState>>,
    core_tx: mpsc::UnboundedSender<ControllerTXWebserverMessage>,
    core_rx: mpsc::UnboundedReceiver<ControllerRXWebserverMessage>
) {
    let static_route = warp::fs::dir("www");
    let mut data = WebserverSharedData::from_core(system_state, core_tx);
    data.setup_core(core_rx);
    let shared_data = Arc::new(data);
    let shared_data_filter_feed = {
        let _data = shared_data.clone();
        warp::any().map(move || _data.clone())
    };
    let stream_ws = warp::path("feed")
        .and(warp::ws())
        .and(shared_data_filter_feed)
        .map(|ws: ws::Ws, shared: Arc<WebserverSharedData>| {
            ws.on_upgrade(move |socket| {
                recv_messages!(socket, shared.ws.core_ws => |_, _, _| {});
            })
        });
    let shared_data_filter_video = {
        let _data = shared_data.clone();
        warp::any().map(move || _data.clone())
    };
    let video_ws = warp::path("video")
        .and(warp::ws())
        .and(shared_data_filter_video)
        .map(|ws: ws::Ws, shared: Arc<WebserverSharedData>| {
            ws.on_upgrade(move |socket| {
                recv_messages!(socket, shared.ws.video_ws => |_, _, _| {});
            })
        });
    let full_routes = static_route.or(stream_ws).or(video_ws);
    /*let dynamic_route = warp::path::end()
        .and(state)
        .map(|state: Arc<RwLock<GroundSystemState>>| {
            let state_r = state.read();
            format!("Hello, your coords are: x = {}, y = {}, z = {}",
                state_r.x, state_r.y, state_r.z)
        });*/

    warp::serve(full_routes).run(([127, 0, 0, 1], 3030)).await;
}
