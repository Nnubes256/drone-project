import picamera
import process


class ICAROSCameraControl(object):
    """docstring for ICAROSCameraControl."""

    def __init__(self, arg):
        super(ICAROSCameraControl, self).__init__()
        self.camera = None

    def init(self):
        self.camera = picamera.PiCamera(sensor_mode='6')
        self.camera.resolution = (960, 540)
        self.camera.framerate = 50
        self.camera.exposure_mode = 'sports'
        self.camera.shutter_speed = 15000
        self.camera.iso = 400
        # TODO change pipe to wifi
        self.camera.start_recording(process.stdout, format='h264',
                                    bitrate=24000000, intra_period=64,
                                    inline_headers=True)

    def capture(self, into_image_file="/tmp/out.jpg"):
        self.camera.capture(into_image_file, use_video_port=True)
        return True
