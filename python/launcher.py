#!/usr/bin/env python

import os
import signal
import sys
import termios
from subprocess import Popen, PIPE
import subprocess

os.system("clear")

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

print bcolors.OKBLUE+"""

 _    ______  ______   __                           __             
| |  / / __ \/ ____/  / /   ____ ___  ______  _____/ /_  ___  _____
| | / / /_/ / /      / /   / __ `/ / / / __ \/ ___/ __ \/ _ \/ ___/
| |/ / _, _/ /___   / /___/ /_/ / /_/ / / / / /__/ / / /  __/ /    
|___/_/ |_|\____/  /_____/\__,_/\__,_/_/ /_/\___/_/ /_/\___/_/     

""" + bcolors.ENDC

print ""
print ""
print ""
#ros is hard to kill...

def signal_handler(signal, frame):
	print '\nExiting...'
	if gazebo:
		gazebo.send_signal(signal.SIGINT) #Python whines about this, but it works.
		gazebo.terminate()
		gazebo.kill()
	if spacenav:
		spacenav.send_signal(signal.SIGINT)
		spacenav.terminate()
		spacenav.kill()
	if spnav:
		spnav.send_signal(signal.SIGINT)
		spnav.terminate()
		spnav.kill()
	if keyboard:
		keyboard.send_signal(signal.SIGINT)
		keyboard.terminate()
		keyboard.kill()
	if cyberglove:
		cyberglove.kill()
	#if fastrak:
		#fastrak.kill()
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

processess = set()
gazebo = 0
spacenav = 0
spnav = 0
keyboard = 0
cyberglove = 0
#fastrak = 0


print "Launching gazebo..."
gazebo = Popen(['roslaunch atlas_utils vrc_task_3.launch' ], stdout=PIPE)
stdout, stderr = gazebo.communicate()
#processess.add(gazebo)
print "Launching spacenav..."
spacenav = Popen(['rosrun', 'spacenav_node', 'spacenav_node', '&'], stdout=PIPE)
stdout, stderr = spacenav.communicate()
processess.add(spacenav)
print "Launching spnav..."
spnav = Popen(['../bin/spnav_all', '&'], stdout=PIPE)
stdout, stderr = spnav.communicate()
print "Launching keyboard teleop..."
keyboard = Popen(['roslaunch', 'atlas_utils', 'keyboard_teleop.launch', '&'], stdout=PIPE)
stdout, stderr = keyboard.communicate()
print "Launching Cyberglove..."
cyberglove = Popen(['./cyberglove.py', '&'], stdout=PIPE)
stdout, stderr = cyberglove.communicate()

#print "Launching Fastrak..."
#os.system("sudo /usr/local/etc/init.d/somatic start")
#os.system("ach -C fastrak")
#fastrak = Popen(['../fastrak/fastrakd', '-p', '/dev/ttyUSB0', '-vvvv'], stdout=PIPE)
#stdout, stderr = fastrak.communicate()

