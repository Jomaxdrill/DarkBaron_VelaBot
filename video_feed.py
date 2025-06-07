from flask import Flask,Response
from picamera2 import Picamera2
from libcamera import Transform
import cv2
import time
import os
import sys
import traceback

RESOLUTION = (900,780)
CENTER = (450,390)


from modules.basic_init import *
from modules.utilities_perception import *
from modules.utilities_camera import *
from modules.utilities_sensors import *
from modules.control_actions import *
from modules.constants_pi import *

colors_to_look = ['green', 'blue', 'red']
color_align = {
    'green': 10,
    'red': 300,
    'blue': 600
}
#imu_sensor = init_serial_read()
app = Flask(__name__)



def generate_frames():
    while True:
        frame = camera.capture_array()
        ##* if looking half the image
        # frame = crop_half_image(frame)
        # frame = draw_center_image(frame,(450,195))
        #*RUN FROM THIS LINE TO THIS LINE TO VISUALIZE
        #frame = hide_caught_block(frame)
        frame = draw_center_image(frame,CAMERA_LORE_RESOLUTION)
        frame = convert_white_area(frame)
        for color_show in colors_to_look:
            frame = show_area(frame,color_show ,color_align[color_show])
        frame = cv2.putText(frame,f'sonar: {distance_sonar()+8.5} cm',(50,370),FONT,2,(0,128,128),2) 
        # ###*
        #frame = top_view_plane(frame)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)
        #time.sleep(0.066)
        #time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
try:
    #setup initialize camera
    init_gpio()
    camera = Picamera2()
    camera.configure( camera.create_video_configuration(
        transform=Transform(vflip=True, hflip=True),
        main={"size": RESOLUTION, "format": "RGB888"},
        lores={"size": (450, 390), "format": "YUV420"},))
    camera.start()
    if __name__ == '__main__':
        app.run(host='0.0.0.0', port=5000)
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
except Exception as error:
    print("An error occurred:", error)
    print(traceback.format_exc())
finally:
    camera.stop()
    print("Camera stopped")
    gameover()
    print("GPIO cleaned up")
    sys.exit(0)