#!/usr/bin/python3

# import the necessary packages

from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
from std_msgs.msg import Float64, Bool
import imutils
import pickle
import time
import cv2
import requests
import socket
import rospy


def send_notification():

    # IP address and port of the Qt application on Windows
    target_ip = '192.168.43.116'  # Replace with the IP address of your Windows machine
    target_port = 1234 # Replace with the port number used by your Qt application

    # Create a socket and connect to the target
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((target_ip, target_port))

    # Send the integer value
    value_to_send = 2# Replace with the integer value you want to seSnd
    data = value_to_send.to_bytes(4, byteorder='little')  # Convert integer to byte array
    client_socket.sendall(data)
    PersonFound_pub.publish(10)
    # Close the socket
    client_socket.close()


#Determine faces from encodings.pickle file model created from train_model.py
encodingsP = "/home/pi/ros_catkin_ws_team5/src/team5/src/encodings.pickle"
#use this xml file
cascade = "haarcascade_frontalface_default.xml"



#function for setting up emails
def send_message(name):
    import smtplib
    from email.mime.multipart import MIMEMultipart
    from email.mime.image import MIMEImage
    from email.mime.text import MIMEText
    
    sender_email = 'os5975798@gmail.com'
    sender_password = 'vsarxzubpnwyltmr'
    receiver_email = 'os5975798@gmail.com'
    subject = 'haraamyyyV5'

    # create message object instance
    msg = MIMEMultipart()

    # attach image to message
    with open('/home/pi/ros_catkin_ws_team5/src/team5/src/image.jpg', 'rb') as f:
        img = MIMEImage(f.read())
        msg.attach(img)

    # create the message body
    body1 = '' + name + ' is in your room'

    # create the MIMEText object
    text1 = MIMEText(body1)

    # attach the text to the MIMEMultipart object
    msg.attach(text1)
    
    # setup the message parameters
    msg['From'] = sender_email
    msg['To'] = receiver_email
    msg['Subject'] = subject

    # create SMTP session
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()

    # login to sender email account
    server.login(sender_email, sender_password)

    # send email
    text1 = msg.as_string()
    #text2 = msg.as_string()
    server.sendmail(sender_email, receiver_email, text1)
    server.quit()


# load the known faces and embeddings along with OpenCV's Haar
# cascade for face detection
print("[INFO] loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())
detector = cv2.CascadeClassifier(cascade)

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)

# start the FPS counter
fps = FPS().start()

#Ros
rospy.init_node("Camera")
PersonFound_pub = rospy.Publisher("PersonFound",Float64,queue_size = 10)

# loop over frames from the video file stream
while True:
	#Initialize 'currentname' to trigger only when a new person is identified.
	currentname = "unknown"
	
	# grab the frame from the threaded video stream and resize it
	# to 500px (to speedup processing)
	
	
	frame = vs.read()
	frame = imutils.resize(frame, width=500)
	
	# convert the input frame from (1) BGR to grayscale (for face
	# detection) and (2) from BGR to RGB (for face recognition)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

	# detect faces in the grayscale frame
	rects = detector.detectMultiScale(gray, scaleFactor=1.05, 
		minNeighbors=15, minSize=(30, 30),maxSize=(90,90),
		flags=cv2.CASCADE_SCALE_IMAGE)
	
	#FILTERRRRR
	
	filtered_rects = []
	for (x, y, w, h) in rects:
	    aspect_ratio = float(w) / h
	    if 0.5 < aspect_ratio < 1.5:
	        filtered_rects.append((x, y, w, h))  
	
	
	
	# OpenCV returns bounding box coordinates in (x, y, w, h) order
	# but we need them in (top, right, bottom, left) order, so we
	# need to do a bit of reordering
	boxes = [(y, x + w, y + h, x) for (x, y, w, h) in rects]

	# compute the facial embeddings for each face bounding box
	encodings = face_recognition.face_encodings(rgb, boxes)
	names = []

	# loop over the facial embeddings
	for encoding in encodings:
		# attempt to match each face in the input image to our known
		# encodings
		matches = face_recognition.compare_faces(data["encodings"],
			encoding)
		name = "Unknown" #if face is not recognized, then print Unknown

		# check to see if we have found a match
		if True in matches:
			# find the indexes of all matched faces then initialize a
			# dictionary to count the total number of times each face
			# was matched
			matchedIdxs = [i for (i, b) in enumerate(matches) if b]
			counts = {}

			# loop over the matched indexes and maintain a count for
			# each recognized face face
			for i in matchedIdxs:
				name = data["names"][i]
				counts[name] = counts.get(name, 0) + 1

			# determine the recognized face with the largest number
			# of votes (note: in the event of an unlikely tie Python
			# will select first entry in the dictionary)
			name = max(counts, key=counts.get)
			#If someone in your dataset is identified, print their name on the screen
		if currentname != name:
		    currentname = name
		    print(currentname)
		    #time.sleep(2.0)
		    #Take a picture to send in the email
		    img_name = "image.jpg"
		    cv2.imwrite(img_name, frame)
		    #Now send me an email to let me know who is at the door
		    send_notification()
		    send_message(name)
		    time.sleep(5)
		    print('Taking a picture.')
				#Now send me an email to let me know who is at the door
				#request = send_message(name)
				#print ('Status Code: '+format(request.status_code)) #200 status code means email sent successfully
				
		# update the list of names
		names.append(name)
		

	# loop over the recognized faces
	for ((top, right, bottom, left), name) in zip(boxes, names):
		# draw the predicted face name on the image - color is in BGR
		cv2.rectangle(frame, (left, top), (right, bottom),
			(0, 255, 225), 2)
		y = top - 15 if top - 15 > 15 else top + 15
		cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
			.8, (0, 255, 255), 2)

	# display the image to our screen
	cv2.imshow("Facial Recognition is Running", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	# update the FPS counter
	fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()


