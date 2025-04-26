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
from utilities_camera import take_image
#command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
#os.system(command)
#setup initialize camera
# Email information
smtpUser = 'mailtestdarkbaronvelabot@gmail.com'
smtpPass = 'fvvelfyvyihcifss'
# Destination email information
toAdd = ['pocres94@gmail.com','jcrespo@umd.edu']


def send_email(camera, color):
	# Define time stamp & record an image
	unix_timestamp= time.time()
	pic_time = datetime.fromtimestamp(unix_timestamp).strftime('%Y%m%d%H%M%S')
	image_camera = take_image(camera)
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
	attach_image.add_header('Content-Disposition', 'attachment', filename=f'{color} block at '+ pic_time + '.jpg')
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

def save_file_info(data):
	unix_timestamp= time.time()
	txt_file = datetime.fromtimestamp(unix_timestamp).strftime('%Y%m%d%H%M%S')
	#write the time of the processed frame
	with open(f'{txt_file}.txt','a') as file_data_process:
		for record in data:
			outstring = str(record) + '\n'
			file_data_process.write(outstring)