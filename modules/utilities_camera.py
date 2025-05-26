
from libcamera import Transform
from picamera2 import MappedArray, Picamera2
from picamera2.outputs import FfmpegOutput
from picamera2.encoders import H264Encoder
import time

#*------------------------
###*CAMERA SET
#*------------------------
CAMERA_MAIN_RESOLUTION = (900,780) #this was the optimal resolution
CAMERA_LORE_RESOLUTION = (CAMERA_MAIN_RESOLUTION[0]//2, CAMERA_MAIN_RESOLUTION[1]//2)
CAMERA_FPS = 30
FORMAT_RGB = 'RGB888'
FORMAT_YUV = 'YUV420'

def init_camera():
    picam2 = Picamera2()
    picam2.configure( picam2.create_video_configuration(
        transform=Transform(vflip=True, hflip=True),
        main={"size": CAMERA_MAIN_RESOLUTION, "format": FORMAT_RGB},
        lores={"size": CAMERA_LORE_RESOLUTION, "format": FORMAT_YUV},))
    return picam2

def take_image(camera, save_it=False):
    camera.start()
    time.sleep(0.05)
    if save_it:
        camera.capture_file(f'camera_pi_{int(time.time())}.jpg')
    image_camera = camera.capture_array()
    time.sleep(0.05)
    return image_camera

def record_video(camera, post_callback, action):
    camera.post_callback = post_callback
    output = FfmpegOutput(f'test_{int(time.time())}.mp4', audio=False)
    camera.start_recording(H264Encoder(), output)
    action()
    camera.stop_recording()
