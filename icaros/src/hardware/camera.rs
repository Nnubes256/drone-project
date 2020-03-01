use std::error::Error;
use std::thread::{self, JoinHandle};
use std::sync::{Arc, mpsc::{self, Receiver, Sender, TryRecvError}};
use parking_lot::Mutex;
use std::path::PathBuf;
use inline_python::{
    python,
    pyo3::{PyErr, Python, types::PyModule}
};
use std::io;
use err_derive::Error;

#[derive(Debug, Error)]
pub enum CameraServiceError {
    #[error(display = "Python code threw exception: {:?}", _0)]
    PythonException(PyErr),
    #[error(display = "I/O error: {}", _0)]
    IOError(io::Error),
    #[error(display = "Camera service was not initialized")]
    UninitializedError
}

pub enum CameraServiceCommand {
    RequestCameraPicture(PathBuf)
}

impl From<PyErr> for CameraServiceError {
    fn from(err: PyErr) -> Self {
        CameraServiceError::PythonException(err)
    }
}

impl From<io::Error> for CameraServiceError {
    fn from(err: io::Error) -> Self {
        CameraServiceError::IOError(err)
    }
}

pub struct CameraService {
    py_context: Arc<Mutex<inline_python::Context>>,
    py_thread: Option<JoinHandle<()>>,
    comm_in: Option<Sender<CameraServiceCommand>>,
}

impl CameraService {
    pub fn new() -> Result<Self, PyErr> {
        Ok(CameraService {
            py_context: Arc::new(Mutex::new(inline_python::Context::new_checked()?)),
            py_thread: None,
            comm_in: None,
        })
    }

    pub fn start(&mut self) -> Result<(), CameraServiceError> {
        let ctx = self.py_context.clone();
        let (comm_in, comm_out) = mpsc::channel();
        self.comm_in = Some(comm_in);
        {
            let gil_guard = Python::acquire_gil();
            let gil_py = gil_guard.python();
            let picamerad = PyModule::from_code(gil_py,
                include_str!("../../ext/picamerad.py"), "picamerad.py", "picamerad")?;
            ctx.lock().globals(gil_py).set_item("picamerad", picamerad)?;
        }
        self.py_thread = Some(thread::Builder::new()
            .name("Camera grabber service".to_string())
            .spawn(move || {
                Self::camera_thread(ctx, comm_out);
            })?);
        Ok(())
    }

    pub fn take_capture(&self, path: PathBuf) -> Result<(), Box<dyn Error>> {
        if let Some(tx) = &self.comm_in {
            tx.send(CameraServiceCommand::RequestCameraPicture(path))?;
            Ok(())
        } else {
            return Err(CameraServiceError::UninitializedError.into());
        }
    }

    fn camera_thread(py_ctx: Arc<Mutex<inline_python::Context>>, recv: Receiver<CameraServiceCommand>) {
        {
            let ctx_unlocked = py_ctx.lock();
            python! {
                #![context = &ctx_unlocked]
                pi_camera = picamerad.ICAROSCameraControl()
                pi_camera.init()
            }
        }
        loop {
            let message = match recv.try_recv() {
                Ok(message) => Some(message),
                Err(err) => match err {
                    TryRecvError::Empty => None,
                    TryRecvError::Disconnected => {
                        error!("Communication pipe on camera thread was disconnected!");
                        None
                    }
                }
            };

            if let Some(msg) = message {
                match msg {
                    CameraServiceCommand::RequestCameraPicture(path) => {
                        let path_str = path.to_str();
                        let ctx_unlocked = py_ctx.lock();
                        python! {
                            #![context = &ctx_unlocked]
                            pi_camera.capture('path_str)
                        }
                    }
                }
            }

            {
                let gil = Python::acquire_gil();
                let gil_py = gil.python();
                if PyErr::occurred(gil_py) {
                    error!("Python error thrown on camera thread: {:?}", PyErr::fetch(gil_py));
                }
            }
        }
    }
}
