cmake and make
To generate a sdf from an urdf run:

./converterURDF FILE.urdf 

The output will be named test.sdf (default). It
calls the meshes from drchubo package so you 
only need the sdf and a config file.

NOTE: For the urdf with fingers closed I only changed:

<link name (RF1 / RF2/ RF3/LF1/LF2/LF3)>

  <visual>
    <pose>rpy = 0 0 0 xyz=...</pose>
    ...
  </visual>
</link>

I changed rpy from pose to rpy = 0 1.1 0 since
the fingers rotate around y axis

Did the same for collision (change the p angle to 1.1)


