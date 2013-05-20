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