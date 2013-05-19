import openhubo.mirror_urdf as mir
import openhubo.urdf as urdf

mir.run("lefthalf.drchubo.urdf","drchubo.urdf")
model=urdf.URDF.load_xml_file("drchubo.urdf")

model.write_openrave_files()




