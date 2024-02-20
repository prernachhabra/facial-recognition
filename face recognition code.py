print("initilization ongoing wait for some seconds")
from RPLCD.i2c import CharLCD
import face_recognition
import cv2
import numpy as np
import os
import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
import time                 # To access delay function

GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning
# Define GPIO to LCD mapping
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)

buzzer_pin =36
motor_pin1 =31
motor_pin2 =32
switch_pin =33
LED_PIN = 29;

GPIO.setup(LED_PIN, GPIO.OUT) 
GPIO.setup(motor_pin1, GPIO.OUT) 
GPIO.setup(motor_pin2, GPIO.OUT) 
GPIO.setup(buzzer_pin,GPIO.OUT)   # Set pin function as output
GPIO.setup(switch_pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)   # Set pin function as input
# Set pin function as input  

CurrentFolder = os.getcwd() #Read current folder path
image = "/home/capstone/Desktop/capstone/rishabh.png"
image2 = "/home/capstone/Desktop/capstone/shagun.jpeg"
image3 = "/home/capstone/Desktop/capstone/prerna.jpeg"

# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is not required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.

# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)

# Load a sample picture and learn how to recognize it.
Rishabh_image = face_recognition.load_image_file(image)
Rishabh_face_encoding = face_recognition.face_encodings(Rishabh_image)[0]

# Load a second sample picture and learn how to recognize it.
Shagun_image = face_recognition.load_image_file(image2)
Shagun_face_encoding = face_recognition.face_encodings(Shagun_image)[0]

#load a third image
Prerna_image = face_recognition.load_image_file(image3)
Prerna_face_encoding = face_recognition.face_encodings(Prerna_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    Rishabh_face_encoding,
    Shagun_face_encoding,
    Prerna_face_encoding,
]
known_face_names = [
    "Rishabh Raj",
    "shagun",
    "prerna",
    "Saurabh"
]

#variable to store status of person inside home or not
door_open_status =0
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True


# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
delay = 1


# Define some device constants
 
# Define delay between readings
delay = 5
lcd.clear()
lcd.write_string('Welcome')
time.sleep(2)
# 000001 Clear display
lcd.clear()
lcd.write_string('Face Detection')
lcd.crlf()
lcd.write_string('System')
time.sleep(2)
GPIO.output(LED_PIN,GPIO.LOW)  #LED ON
GPIO.output(buzzer_pin,GPIO.LOW)  #LED ON
GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
GPIO.output(motor_pin2,GPIO.LOW)  #LED ON  
while True:
    #take input from switch
    door_open_status = 0
    lcd.clear()
    lcd.write_string('Press the Bell')
    time.sleep(2)
    if GPIO.input(switch_pin) == GPIO.HIGH:
        GPIO.output(LED_PIN,GPIO.HIGH)  #LED ON
    
        time.sleep(3)
        GPIO.output(LED_PIN,GPIO.LOW)  #LED ON
        #GPIO.output(buzzer_pin,GPIO.LOW)  #LED ON
        while(1):        
            # Grab a single frame of video
            ret, frame = video_capture.read()

            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = np.ascontiguousarray(small_frame[:, :, ::-1])

            # Only process every other frame of video to save time
            if process_this_frame:
                # Find all the faces and face encodings in the current frame of video
                face_locations = face_recognition.face_locations(rgb_small_frame)
                face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                face_names = []
                for face_encoding in face_encodings:
                    # See if the face is a match for the known face(s)
                    matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                    name = "Unknown"

                    # # If a match was found in known_face_encodings, just use the first one.
                    # if True in matches:
                    #     first_match_index = matches.index(True)
                    #     name = known_face_names[first_match_index]

                    # Or instead, use the known face with the smallest distance to the new face
                    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = known_face_names[best_match_index]
                    face_names.append(name)
                    if(name != "Unknown"):
                        lcd.clear() # 000001 Clear display
                        lcd.write_string('Welcome')
                        lcd.crlf()
                        lcd.write_string(name)
                        if(door_open_status == 0):
                            door_open_status = 1
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.HIGH)  #LED ON
                            time.sleep(0.3)
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON
                            time.sleep(2)
                            GPIO.output(motor_pin1,GPIO.HIGH)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON
                            time.sleep(0.3)
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON                        
                            time.sleep(1)
                    else:
                        lcd.clear()
                        lcd.write_string(name)
                        GPIO.output(buzzer_pin,GPIO.HIGH)
                        time.sleep(2)
                        GPIO.output(buzzer_pin,GPIO.LOW)
                        
                                    
            process_this_frame = not process_this_frame

            # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

            # Display the resulting image
            cv2.imshow('Video', frame)
            if (cv2.waitKey(2) == 27):
                cv2.destroyAllWindows()
                break


# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
