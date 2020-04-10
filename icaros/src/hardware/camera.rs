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

/// An error thrown by the camera service
#[derive(Debug, Error)]
pub enum CameraServiceError {
    /// Python-thrown exception
    #[error(display = "Python code threw exception: {:?}", _0)]
    PythonException(PyErr),
    /// I/O related error
    #[error(display = "I/O error: {}", _0)]
    IOError(io::Error),
    /// Error due to the camera service being uninitialized
    #[error(display = "Camera service was not initialized")]
    UninitializedError
}

/// Commands available to send to the Python script handling the camera
pub enum CameraServiceCommand {
    /// Requests a camera picture to be saved in the specified location
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

///
/// The camera service
///
/// Part of the external services, this service handles camera control. It does so by using a
/// Python script, which we run programmatically through the Python interpreter. This allows us
/// not only for the Python script's picamera facilities to easily handle the camera itself,
/// but it also allows the service to directly execute functions and code on these facilities,
/// which essentially allows it to control the camera itself.
///
/// For practical reasons, the Python interpreter runs in a separate process, and communicates with
/// the external service broker through a message-passing interface.
pub struct CameraService {
    /// The Python interpreter context
    py_context: Arc<Mutex<inline_python::Context>>,

    /// The separate thread the Python process is running on
    py_thread: Option<JoinHandle<()>>,

    /// The message-passing interface between the ESB and the camera process.
    comm_in: Option<Sender<CameraServiceCommand>>,
}

impl CameraService {
    /// Initialize a new camera service
    pub fn new() -> Result<Self, PyErr> {
        Ok(CameraService {
            py_context: Arc::new(Mutex::new(inline_python::Context::new_checked()?)),
            py_thread: None,
            comm_in: None,
        })
    }

    /// Starts the camera service. This involves launching the Python code that handles the
    /// interaction with the camera and opening a thread to control its execution.
    pub fn start(&mut self) -> Result<(), CameraServiceError> {
        // We obtain a shared reference to the Python interpreter context
        let ctx = self.py_context.clone();

        // We create the message-passing interface
        let (comm_in, comm_out) = mpsc::channel();
        self.comm_in = Some(comm_in);

        { // Lock scope begin
            // We adquire the Python interpreter's Global Interpreter Lock (GIL).
            let gil_guard = Python::acquire_gil();
            let gil_py = gil_guard.python();

            // We load our camera handling code as a Python module
            // The code is not loaded from the filesystem on runtime; instead, the
            // [`include_str!`](std::include_str) compiler macro includes the script itself
            // as an static string at compile time.
            let picamerad = PyModule::from_code(gil_py,
                include_str!("../../ext/picamerad.py"), "picamerad.py", "picamerad")?;

            // Using the GIL we current have locked, we then obtain our Python context's global
            // state dictionary and add our module as an item in this dictionary.
            ctx.lock().globals(gil_py).set_item("picamerad", picamerad)?;
        } // Lock scope end

        // Then, we spawn the thread that will handle normal communication with our Python instance
        self.py_thread = Some(thread::Builder::new()
            .name("Camera grabber service".to_string())
            .spawn(move || {
                Self::camera_thread(ctx, comm_out);
            })?);
        Ok(())
    }

    /// Asks the camera service to take a capture and deposit it on the given path
    /// on the filesystem.
    pub fn take_capture(&self, path: PathBuf) -> Result<(), Box<dyn Error>> {
        // We only can do this if we have initialized the camera service
        if let Some(tx) = &self.comm_in {
            // On which case, we send the appropiate command through the message-passing interface
            tx.send(CameraServiceCommand::RequestCameraPicture(path))?;
            Ok(())
        } else {
            // Otherwise, return an error
            return Err(CameraServiceError::UninitializedError.into());
        }
    }

    /// Code ran on the thread that handles the camera
    fn camera_thread(py_ctx: Arc<Mutex<inline_python::Context>>, recv: Receiver<CameraServiceCommand>) {
        { // Python GIL scope begin
            // Take a hold of Python's GIL
            let ctx_unlocked = py_ctx.lock();

            // Our camera handling module is already loaded onto Python; we just have
            // to initialize an instace of the class it exposes and run it with
            // some additional code. (What follows below is Python code)
            python! {
                #![context = &ctx_unlocked]
                pi_camera = picamerad.ICAROSCameraControl()
                pi_camera.init()
            }
        } // Python GIL scope end

        // The code above will run in the background. We can now start processing messages
        loop {
            // Attempt to receive a message from our message-passing interface
            let message = match recv.try_recv() {
                Ok(message) => Some(message),
                Err(err) => match err {
                    TryRecvError::Empty => None, // No message was received
                    TryRecvError::Disconnected => {
                        error!("Communication pipe on camera thread was disconnected!");
                        None
                    }
                }
            };

            // If we received any message...
            if let Some(msg) = message {
                // We filter by message type
                match msg {
                    CameraServiceCommand::RequestCameraPicture(path) => { // Python GIL scope begin
                        // We convert the destination path passed to an string
                        let path_str = path.to_str();

                        // Then, obtain a GIL lock
                        let ctx_unlocked = py_ctx.lock();

                        // And execute the appropiate function on the Python side
                        python! {
                            #![context = &ctx_unlocked]
                            pi_camera.capture('path_str)
                        }
                    } // Python GIL scope begin
                }
            }

            // This way of using Python requires us to come every once in a while to check if
            // the Python interpreter has thrown an exception. The code below is responsible
            // for checking for such condition and printing the exception if one has happened.

            { // Python GIL scope begin
                let gil = Python::acquire_gil();
                let gil_py = gil.python();
                if PyErr::occurred(gil_py) {
                    error!("Python error thrown on camera thread: {:?}", PyErr::fetch(gil_py));
                }
            } // Python GIL scope end
        }
    }
}
