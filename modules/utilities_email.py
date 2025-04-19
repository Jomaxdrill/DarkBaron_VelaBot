import os
from datetime import datetime
import smtplib
from smtplib import SMTPException
import email
import time
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from picamera2 import Picamera2
from libcamera import Transform

CAMERA_MAIN_RESOLUTION = (900,780)
CAMERA_LORE_RESOLUTION = (450, 375)
CAMERA_FPS = 30
FORMAT_RGB = 'RGB888'
FORMAT_YUV = 'YUV420'
# Define time stamp & record an image
unix_timestamp= time.time()
pic_time = datetime.fromtimestamp(unix_timestamp).strftime('%Y%m%d%H%M%S')
#command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
#os.system(command)
#setup initialize camera
picam2 = Picamera2()
picam2.configure( picam2.create_video_configuration(
        transform=Transform(vflip=True, hflip=True),
        main={"size": CAMERA_MAIN_RESOLUTION, "format": FORMAT_RGB},
        lores={"size": CAMERA_LORE_RESOLUTION, "format": FORMAT_YUV},))
picam2.start()
time.sleep(2)
picam2.capture_file(f'{pic_time}.jpg')
# Email information
smtpUser = 'mailtestdarkbaronvelabot@gmail.com'
smtpPass = 'fvvelfyvyihcifss'

# Destination email information
toAdd = ['jcrespo@umd.edu','ENPM701S19@gmail.com']
fromAdd = smtpUser
subject = 'Block color red delivered'
msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = ', '.join(toAdd)
msg.preamble = "Block color red delivered"

# Email text
body = MIMEText("In construction zone at " + pic_time)
msg.attach(body)

# Attach
fp = open(pic_time + '.jpg', 'rb')
img = MIMEImage(fp.read())
img.add_header('Content-Disposition', 'attachment', filename='red block at '+ pic_time + '.jpg')  # Set the filename
fp.close()
msg.attach(img)

# Send email
s = smtplib.SMTP('smtp.gmail.com', 587)
s.ehlo()
s.starttls()
s.ehlo()
s.login(smtpUser, smtpPass)
s.sendmail(fromAdd, toAdd, msg.as_string())
s.quit()

print("Email delivered!")

