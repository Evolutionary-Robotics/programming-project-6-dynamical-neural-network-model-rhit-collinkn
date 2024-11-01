import pyrosim.pyrosim as ps

def Create_Robot1():
    ps.Start_URDF("body.urdf")
    ps.Send_Cube(name="Foot", pos=[0, 0, 1.5], size=[1, 1, 1])  # Parent

    ps.Send_Joint(name="Foot_Torso1", parent="Foot", child="Torso1", type="revolute", position=[0.5, 0, 1.0])
    ps.Send_Cube(name="Torso1", pos=[0.5, 0.5, 1.0], size=[0.5, 0.5, 0.5])

    ps.Send_Joint(name="Foot_Torso2", parent="Foot", child="Torso2", type="revolute", position=[-0.5, 0.0, 1.0])
    ps.Send_Cube(name="Torso2", pos=[-0.5, 0.5, 1.0], size=[0.5, 0.5, 0.5])

    ps.Send_Joint(name="Foot_Torso3", parent="Foot", child="Torso3", type="revolute", position=[0.5, -0.8, 1.0])
    ps.Send_Cube(name="Torso3", pos=[0.5, -0.5, 1.0], size=[0.5, 0.5, 0.5])

    ps.Send_Joint(name="Foot_Torso4", parent="Foot", child="Torso4", type="revolute", position=[-0.5, -0.8, 1.0])
    ps.Send_Cube(name="Torso4", pos=[-0.5, -0.5, 1.0], size=[0.5, 0.5, 0.5])

    ps.End()

Create_Robot1()
