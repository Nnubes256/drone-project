import picamera
import subprocess


class ICAROSCameraControl(object):
    """docstring for ICAROSCameraControl."""

    def __init__(self):
        super(ICAROSCameraControl, self).__init__()
        self.camera = None
        self.wifibroadcast = None

    def init(self):
        self.wifibroadcast = subprocess.Popen([
            "./tx", "-p", "0", "-b", "4", "-r", "8", "-f", "1100", "wlan1"
        ], stdin=subprocess.PIPE, bufsize=0)
        self.camera = picamera.PiCamera(sensor_mode=6)
        self.camera.resolution = (960, 540)
        self.camera.framerate = 30
        self.camera.exposure_mode = 'sports'
        self.camera.shutter_speed = 12000
        self.camera.iso = 400
        self.camera.vflip = True
        self.camera.hflip = True
        # TODO change pipe to wifi
        self.camera.start_recording(self.wifibroadcast.stdin, format='h264',
                                    bitrate=2500000, intra_period=6,
                                    inline_headers=True, profile='baseline')

    def capture(self, into_image_file="/tmp/out.jpg"):
        self.camera.capture(into_image_file, use_video_port=True)
        return True
