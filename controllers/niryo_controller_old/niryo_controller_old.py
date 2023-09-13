"""niryo_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import niryoNed, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
import sys

import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import tempfile

from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

import numpy as np

# create the niryoNed instance.
#niryoNed = Robot()
#supervisorNED = Supervisor()

niryoNed = Supervisor()
supervisorNED = niryoNed

box_node = supervisorNED.getFromDef('BOX')
if box_node is None:
    sys.stderr.write("No DEF BOX node found in the current world file\n")
    sys.exit(1)
box_translation_field = box_node.getField("translation")
box_rotation_field = box_node.getField("rotation")

niryo_node = supervisorNED.getFromDef('NIRYO')
if niryo_node is None:
    sys.stderr.write("No DEF NIRYO node found in the current world file\n")
    sys.exit(1)
niryo_translation_field = niryo_node.getField("translation")
niryo_rotation_field = niryo_node.getField("rotation")


# get the time step of the current world.
timestep = int(niryoNed.getBasicTimeStep())

with tempfile.NamedTemporaryFile(suffix='.urdf', delete = False) as file:
    filename = file.name
    file.write(niryoNed.getUrdf().encode('utf-8'))

armChain = Chain.from_urdf_file(filename, 
           active_links_mask = [False, True, True, True, True, True, True, False, False])


joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripper::left', 'gripper::right']

initPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

robot_joints = []

vel = 0.5

for i in range(0, 8):
    temp = niryoNed.getDevice(joint_names[i])
    robot_joints.append(temp)
    robot_joints[i].setPosition(initPos[i])
    robot_joints[i].setVelocity(vel)

# You should insert a getDevice-like function in order to get the
# instance of a device of the niryoNed. Something like:
#  motor = niryoNed.getDevice('motorname')
#  ds = niryoNed.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


niryo_translation = niryo_translation_field.getSFVec3f()
#print("niryo is at translation: %g %g %g" % (niryo_translation[0], niryo_translation[1], niryo_translation[2]))
niryo_rotation = niryo_rotation_field.getSFRotation()
#print("niryo is at rotation: %g %g %g %g" % (niryo_rotation[0], niryo_rotation[1], niryo_rotation[2],niryo_rotation[3]))

niryo_rotation_q = pr.quaternion_from_axis_angle(niryo_rotation)
print("niryo is at q-rotation: %g %g %g %g" % (niryo_rotation_q[0], niryo_rotation_q[1], niryo_rotation_q[2],niryo_rotation_q[3]))

niryo_pose = pt.transform_from_pq(np.hstack((niryo_translation, niryo_rotation_q)))

box_translation = box_translation_field.getSFVec3f()

#correct heigt against box
#box_translation[2] = box_translation[2] + 0.085 - (0.018/2)

#print("cardboard box is at translation: %g %g %g" % (box_translation[0], box_translation[1], box_translation[2]))
box_rotation = box_rotation_field.getSFRotation()
#print("cardboard box is at rotation: %g %g %g %g" % (box_rotation[0], box_rotation[1], box_rotation[2],box_rotation[3]))

box_rotation_q = pr.quaternion_from_axis_angle(box_rotation)
box_rotation_q = [0,0,0,1]
print("cardboard box is at q-rotation: %g %g %g %g" % (box_rotation_q[0], box_rotation_q[1], box_rotation_q[2], box_rotation_q[3]))

box_pose = pt.transform_from_pq(np.hstack((box_translation, box_rotation_q)))

tm = TransformManager()
tm.add_transform('niryo', 'world',  niryo_pose)
tm.add_transform('box', 'world', box_pose)

box2niryo = tm.get_transform('box', 'niryo')
# print(box2niryo)

#print(pt.pq_from_transform(box2niryo))
translation_translation = pt.pq_from_transform(box2niryo)[0:3]
#print(translation)
print("translation translation: %g %g %g" % (translation_translation[0], translation_translation[1], translation_translation[2]))

translation_rotation = pt.pq_from_transform(box2niryo)[3:7]
translation_rotation = [0,0,0,1]
print("translation rotation: %g %g %g %g" % (translation_rotation[0], translation_rotation[1], translation_rotation[2],translation_rotation[3]))


import matplotlib.pyplot

plot_poses = False

if plot_poses:
    ax = tm.plot_frames_in("world", s=0.1)
    matplotlib.pyplot.show()


x = 0.2
y = 0.3
z = 0.085

#translation =[x, y, z]

PI = 3.1428
roll = PI/2
pitch = PI/2
yaw = PI



euler = [roll,pitch,yaw]# 
#euler = pr.euler_from_quaternion(translation, True)

ikSolution = armChain.inverse_kinematics(translation_translation, euler, orientation_mode = "all")
#ikSolution = armChain.inverse_kinematics([x, y, z])

plot_ik_solution = True

if plot_ik_solution:
    from mpl_toolkits.mplot3d import Axes3D
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    armChain.plot(ikSolution, ax)
    matplotlib.pyplot.show()
    

for i in range(0, 6):
    #print(i)
    robot_joints[i].setPosition(ikSolution[i + 1])
    pass

while niryoNed.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    

    pass

# Enter here exit cleanup code.
