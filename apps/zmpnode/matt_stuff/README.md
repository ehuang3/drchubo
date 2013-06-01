hubomz
======

Hubo planning, control, and visualization software
Original code from Matt Zucker. Adapted to be used with DART/GRIP


Building
========
The usual cmake and make routine. However there is one more thing: To run the software, 
you will need an OpenRAVE model of the Hubo plus. Get the openHubo package from 

https://github.com/daslrobotics/openHubo

and patch it using the huboplus.patch in the root directory of this
project.

    cd /path/to/openHubo
    patch -p0 < /path/to/hubomz/huboplus.patch

Next, symlink the huboplus directory from openHubo into the root
directory of this repository.

    cd /path/to/hubomz
    ln -s /path/to/openHubo/robots/huboplus
   
Notice that in planningTab I have added hardcoded the path to myhubo.kinbody.xml as

"../myhubo.kinbody.xml"

I made this since I usually run the executable from a build folder and the xml file is on the
source directory. If you want to change this local path, change line 414 of planninTab
to suit your preferences.

Run Tab
======
1. Load a world with a huboplus on it
2. Press Set start (to set initial configuration)
3. Press run

Run will generate a trajectory with 1000 waypoints and will store them in a text file named traj.txt.

4. If you want to see the movement, press Next Step to advance 10 waypoints or Prev Step to go back 10 waypoints. You can change this by changing mPathDelta to another value in PlanningTab
ToDo: Pass the movements to the  Timeline , make the movement w.r.t. the standing foot
Alternatively, if you wish to test the controller gains, you may instead of pressing next step to see the trajectory, you may press the "Set Controller" button and then press play for the simulation to start running. Doing these will generate a trajectory that is fed to a controller on each timestep with tuned gains. The result should be a simulation of the robot attempting to walk in a completly open-loop system.

