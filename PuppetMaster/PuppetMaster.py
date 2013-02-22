'''
==PuppetMaster==
Georgia Tech DRC group

This script tracks a single skeleton with a kinect and generates a Traj_data.yaml file with a recording
of the movements made. This lets you use a kinect as a virtual waldo rig to quickly animate Atlas. 

Requirements:
-Windows for now :(
-Kinect SDK from Microsoft
-vpython
-pykinect
-vpykinect module

Installation:
Install the Kinect SDK here: http://www.microsoft.com/en-us/kinectforwindows/
Install Visual Python here: http://www.vpython.org/
Install PyKinect here: http://pypi.python.org/pypi/pykinect/1.0 (Either libfreenect or OpenNI would work equally well, this code has only been tested with pykinect)
Run or import the vpykinect module here: https://sites.google.com/site/erfarmer/downloads

Running:
Double click on PuppetMaster.py 
Stand with your arms above your head so the Kinect can acquire you
A skeleton will appear when tracking begins. Move around while it's recording.
When you're done, close the command window
Make sure you've followed the setup instructions in the DRCSIM tutorial.
Copy Traj_data_kinect.yaml from the current directory to ~/ros/vrc-golem/Tutorials/tutorial_atlas_control
Run $python traj_yaml.py Traj_data_kinect.yaml PuppetMaster

Notes:
-Doesn't support tracking multiple skeletons
-You can't index joints directly from vpykinect, you have to use pykinect.
-This assumes the default traj_yaml.py file and that it hasn't been modified after installing gazebo.
''' 

from visual import *
from math import log10, floor
import vpykinect
import re

print 'Initializing Kinect. Please stand in the phi position.'
f = open('Traj_data_kinect.yaml', 'w')
f.write('PuppetMaster:\n')
vpykinect.draw_sensor(frame())
skeleton = vpykinect.Skeleton(frame(visible=False))
while True:
	rate(30)
	skeleton.frame.visible = skeleton.update()
	p2 = skeleton.joints[vpykinect.JointId.HandRight.value].pos #test right hand joint data object
	#might also need to downsample, since sample rate is pretty high
	#not sure which of the 6 right arm joints is the wrist...
	#Can drop z component, since we can't assign z movements.
	p2 = p2[0]
	print (round(p2,1)) #Drop some sig figs
	f.write('  - [0.05, \"')
	f.write('0   0  0 0 0 0       0   0   0 0 0 0     0 0 0 0 0 0      0 0 0 0 0 ')
	f.write(str(round(p2,1)) + '        0 0 0 0')
	f.write('   \" ]\n')
f.close()


#Example output line:
# - [1.0, "0   0  -1.7 1.8 -0.10 0       0   0   -1.7 1.8 -0.1 0     -1.6 -1.6 0 0 0 0      1.6 1.6 0 0 0 0        0 0 0 0   " ]
#What each of these numbers control is defined in traj_yaml.py