#!/usr/bin/env python

#screw cyberglove, at RIM we reverse engineer the documentation ourselves.

import serial
import os
import time


print "Let's identify which port goes to which glove. Move your right hand and see if the number changes."
raw_input("Press enter when you're ready.")

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.write("S") #command to start streaming data
for i in xrange(200):
	s = ""
	s = ser.read(48)        # read up to ten bytes (timeout)
	value =  s[38].encode('hex') #39-1 since index starts at 0
	print value
ser.write("\x03") #Send ctrl+c to stop stream
ser.close()

correct = raw_input("Did it change? Y/N: ")
if correct.upper() == "Y":
	right_port = "/dev/ttyUSB0"
	left_port = "/dev/ttyUSB1"
else:
	right_port = "/dev/ttyUSB1"
	left_port = "/dev/ttyUSB0"


########################RIGHT CALIBRATION#############################

print "Ok, now let's calibrate your RIGHT hand."
raw_input("Open your RIGHT hand and press enter when you're ready to begin.")

print "Calibrating..."
ser = serial.Serial(right_port, 115200, timeout=1)
ser.write("S") #command to start streaming data
open_avg = 0
for i in xrange(200):
	s = ""
	s = ser.read(48)        # read up to ten bytes (timeout)
	#print "Read: "+s
	#We assume that we're reading the correct 48 bytes and aren't offset. If we get offset, we're gonna have a bad time...
	#We're going to look at the index finger metacarpal to decide the state of the hand.
	value =  s[38].encode('hex') #39-1 since index starts at 0
	#print value
	open_avg += int(value, 16) #This is hex
ser.write("\x03") #Send ctrl+c to stop stream
ser.close()
open_avg = open_avg/200 #arithmetic average
print "Average is "+str(open_avg)
print "Done!"
raw_input("Make a fist with your RIGHT hand and press enter when you're ready to begin.")

print "Calibrating..."
ser = serial.Serial(right_port, 115200, timeout=1)
ser.write("S") #command to start streaming data
closed_avg = 0
for i in xrange(200):
	s = ""
	s = ser.read(48)        # read up to ten bytes (timeout)
	#print "Read: "+s
	value =  s[38].encode('hex') #39-1 since index starts at 0
	#print value
	closed_avg += int(value, 16) #This is hex
ser.write("\x03") #Send ctrl+c to stop stream
ser.close()
closed_avg = closed_avg/200 #arithmetic average
print "Average is "+str(closed_avg)
right_closed_avg = closed_avg
right_open_avg = open_avg
print "Done!"


##################################LEFT CALIBRATION#########################

print "Ok, now let's calibrate your LEFT hand."
raw_input("Open your LEFT hand and press enter when you're ready to begin.")

print "Calibrating..."
ser = serial.Serial(left_port, 115200, timeout=1)
ser.write("S") #command to start streaming data
open_avg = 0
for i in xrange(200):
	s = ""
	s = ser.read(48)        # read up to ten bytes (timeout)
	#print "Read: "+s
	value =  s[38].encode('hex') #39-1 since index starts at 0
	#print value
	open_avg += int(value, 16) #This is hex
ser.write("\x03") #Send ctrl+c to stop stream
ser.close()
open_avg = open_avg/200 #arithmetic average
print "Average is "+str(open_avg)
print "Done!"
raw_input("Make a fist with your LEFT hand and press enter when you're ready to begin.")
print "Calibrating..."
ser = serial.Serial(left_port, 115200, timeout=1)
ser.write("S") #command to start streaming data
closed_avg = 0
for i in xrange(200):
	s = ""
	s = ser.read(48)        # read up to ten bytes (timeout)
	#print "Read: "+s
	value =  s[38].encode('hex') #39-1 since index starts at 0
	#print value
	closed_avg += int(value, 16) #This is hex
ser.write("\x03") #Send ctrl+c to stop stream
ser.close()
closed_avg = closed_avg/200 #arithmetic average
print "Average is "+str(closed_avg)
left_closed_avg = closed_avg
left_open_avg = open_avg
print "Done!"

#############################ROS PUBLISHER################################

raw_input("Ready to begin publishing to ROS. Press enter to begin controlling Atlas.")
while(1):
	######RIGHT SENSE#######
	right_ser = serial.Serial(right_port, 115200, timeout=1)
	right_ser.write("S") #begin streaming
	right_s = ""
	right_s = right_ser.read(48)
	right_glove_value = right_s[38].encode('hex')
	#print int(glove_value, 16)-open_avg
	right_ser.write("\x03")
	right_ser.close()
	right_scaled_value = float((int(right_glove_value, 16)-float(right_open_avg))/(float(right_closed_avg)-float(right_open_avg)))
	print "RIGHT scaled is "+str(right_scaled_value)
	if right_scaled_value < 0:
		right_scaled_value = 0
	if right_scaled_value > 1.0:
		right_scaled_value = 1.0
	right_command = "rosservice call /sandia_hands/r_hand/simple_grasp ' { grasp: { name: \"spherical\", closed_amount: "+str(right_scaled_value)+"} }'"
	#print command
	os.system(right_command)

	######LEFT SENSE########
	left_ser = serial.Serial(left_port, 115200, timeout=1)
	left_ser.write("S") #begin streaming
	left_s = ""
	left_s = left_ser.read(48)
	left_glove_value = left_s[38].encode('hex')
	#print int(glove_value, 16)-open_avg
	left_ser.write("\x03")
	left_ser.close()
	left_scaled_value = float((int(left_glove_value, 16)-float(left_open_avg))/(float(left_closed_avg)-float(left_open_avg)))
	print "left scaled is "+str(left_scaled_value)
	if left_scaled_value < 0:
		left_scaled_value = 0
	if left_scaled_value > 1.0:
		left_scaled_value = 1.0
	left_command = "rosservice call /sandia_hands/l_hand/simple_grasp ' { grasp: { name: \"spherical\", closed_amount: "+str(left_scaled_value)+"} }'"
	#print command
	os.system(left_command)


	time.sleep(1) # one second delay so we don't kill the ros server with requests


