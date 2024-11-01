import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import time
import math
from izhikevich import IzhikevichNetwork

def run_simulation(nn_parameters):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())


    p.setGravity(0, 0, -9.8)
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("body.urdf")
    ps.Prepare_To_Simulate(robot_id)

    initial_position = p.getBasePositionAndOrientation(robot_id)[0][0]
    duration = 60000 
    dt = 0.005

    joint_names = ["Foot_Torso1", "Foot_Torso2", "Foot_Torso3", "Foot_Torso4"]

    joint_indices = []
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('UTF-8')  # Convert bytes to string
        if joint_name in joint_names:
            joint_indices.append(i)

    nn_size = 4 
    nn = IzhikevichNetwork(nn_size)

    nn.a = nn_parameters['a']
    nn.b = nn_parameters['b']
    nn.c = nn_parameters['c']
    nn.d = nn_parameters['d']
    nn.voltages = [-65.0 for _ in range(nn_size)]
    nn.u = [nn.b[i] * nn.voltages[i] for i in range(nn_size)]
    nn.weights = nn_parameters['weights']
    constant_input = nn_parameters['inputs']

    for step in range(duration):
        nn.step(dt, constant_input)
        motor_values = nn.outputs 
        motor_values_normalized = []
        for v in motor_values:
            min_voltage = -80
            max_voltage = 30
            min_angle = -math.pi / 2
            max_angle = math.pi / 2
            normalized_value = ((v - min_voltage) / (max_voltage - min_voltage)) * (max_angle - min_angle) + min_angle
            motor_values_normalized.append(normalized_value)
        for idx in range(len(joint_indices)):
            joint_index = joint_indices[idx]
            motor_value = motor_values_normalized[idx]
            p.setJointMotorControl2(bodyIndex=robot_id,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=motor_value,
                                    force=100)
        p.stepSimulation()
        time.sleep(dt)

    final_position = p.getBasePositionAndOrientation(robot_id)[0][0]
    distance_traveled = final_position - initial_position

    print(f"Distance traveled: {distance_traveled} meters")
    p.disconnect()

    return distance_traveled

if __name__ == "__main__":
    nn_params_list = [
        # nn1
        {
            'a': [0.1, 0.12, 0.15, 0.1],
            'b': [0.25, 0.3, 0.2, 0.27],
            'c': [-65.0, -64.0, -66.0, -63.0],
            'd': [2.0, 5.0, 3.0, 4.0],
            'weights': [[0.0]*4 for _ in range(4)],
            'inputs': [10.0, 15.0, 12.0, 14.0]
        },
        # nn2
        {
            'a': [0.08, 0.1, 0.12, 0.09],
            'b': [0.2, 0.25, 0.22, 0.23],
            'c': [-65.0, -66.0, -64.0, -65.0],
            'd': [5.0, 3.0, 4.0, 2.0],
            'weights': [[0.0]*4 for _ in range(4)],
            'inputs': [12.0, 13.0, 11.0, 15.0]
        },
        # nn3
        {
            'a': [0.15, 0.14, 0.16, 0.13],
            'b': [0.3, 0.28, 0.32, 0.29],
            'c': [-64.0, -63.0, -65.0, -66.0],
            'd': [4.0, 5.0, 2.0, 3.0],
            'weights': [[0.0]*4 for _ in range(4)],
            'inputs': [14.0, 16.0, 13.0, 12.0]
        },
        # nn4
        {
            'a': [0.1, 0.1, 0.1, 0.1],
            'b': [0.25, 0.25, 0.25, 0.25],
            'c': [-65.0, -65.0, -65.0, -65.0],
            'd': [2.0, 2.0, 2.0, 2.0],
            'weights': [[0.0]*4 for _ in range(4)],
            'inputs': [15.0, 15.0, 15.0, 15.0]
        },
        # nn5
        {
            'a': [0.05, 0.07, 0.06, 0.08],
            'b': [0.2, 0.22, 0.21, 0.23],
            'c': [-66.0, -65.0, -64.0, -63.0],
            'd': [6.0, 5.0, 4.0, 3.0],
            'weights': [[0.0]*4 for _ in range(4)],
            'inputs': [13.0, 14.0, 12.0, 11.0]
        },
    ]

    distances = []
    for idx, nn_params in enumerate(nn_params_list):
        print(f"\nRunning simulation for Neural Network {idx + 1}")
        distance = run_simulation(nn_params)
        distances.append(distance)
    print("\nExperiment Results:")
    for idx, distance in enumerate(distances):
        print(f"Neural Network {idx + 1}: Distance traveled = {distance} meters")
