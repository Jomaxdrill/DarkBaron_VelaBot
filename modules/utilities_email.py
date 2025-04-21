#import os
from datetime import datetime
import smtplib
#from smtplib import SMTPException
#import email
import time
import cv2
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

#command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
#os.system(command)
#setup initialize camera
# Email information
smtpUser = 'mailtestdarkbaronvelabot@gmail.com'
smtpPass = 'fvvelfyvyihcifss'
# Destination email information
toAdd = ['jcrespo@umd.edu','ENPM701S19@gmail.com']


def send_email(camera, color):
        # Define time stamp & record an image
        unix_timestamp= time.time()
        pic_time = datetime.fromtimestamp(unix_timestamp).strftime('%Y%m%d%H%M%S')
        image_camera = camera.take_image()
        success , image_encoded = cv2.imencode('.jpg', image_camera)
        if not success:
                raise Exception("Failed to encode image to JPEG")
        img_data = image_encoded.tobytes()
        fromAdd = smtpUser
        subject = f"Block of color {color} delivered"
        #Create the email
        msg = MIMEMultipart()
        msg['Subject'] = subject
        msg['From'] = fromAdd
        msg['To'] = ', '.join(toAdd)
        msg.preamble = f"Block color {color} delivered"
        # Email text
        body = MIMEText("In construction zone at " + pic_time)
        msg.attach(body)
        # Attach image
        attach_image = MIMEImage(img_data)
        attach_image.add_header('Content-Disposition', 'attachment', filename='{color} block at '+ pic_time + '.jpg')
        msg.attach(attach_image)
        # Send email
        send_email = smtplib.SMTP('smtp.gmail.com', 587)
        send_email.ehlo()
        send_email.starttls()
        send_email.ehlo()
        send_email.login(smtpUser, smtpPass)
        send_email.sendmail(fromAdd, toAdd, msg.as_string())
        send_email.quit()
        print("Email delivered!")
        return True