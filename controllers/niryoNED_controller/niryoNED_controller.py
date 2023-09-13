"""niryo_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import niryoNED, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
import sys

import ikpy
from ikpy.chain import Chain
#from ikpy.link import OriginLink, urdfLink

import math

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

IKPY_MAX_ITERATIONS = 4

import tempfile

from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

import numpy as np
import matplotlib.pyplot

supervisor_niryoNED = Supervisor()
niryoNED = supervisor_niryoNED

class Pose:
    def __init__(self, Def):
        self.Def = Def
        self.node = supervisor_niryoNED.getFromDef(self.Def)
        if self.node is None:
            sys.stderr.write("No DEF %s node found in the world file\n" % Def)
            sys.exit(1)
        self.translation_field = self.node.getField("translation")
        self.rotation_field = self.node.getField("rotation")

    def get(self):
        self.position = self.translation_field.getSFVec3f()
        self.rotation = self.rotation_field.getSFRotation()
        return self.position , self.rotation

    def get_quaternion(self):
        self.position = self.translation_field.getSFVec3f()
        self.rotation = self.rotation_field.getSFRotation()
        #print("rotation: %g %g %g %g" % (self.rotation[0], self.rotation[1], self.rotation[2],self.rotation[3]))
        self.rotation_q  = pr.quaternion_from_axis_angle(self.rotation)
        #print("q-rotation: %g %g %g %g" % (self.rotation_q[0], self.rotation_q[1], self.rotation_q[2],self.rotation_q[3]))
        return self.position , self.rotation_q
        
    def get_position_quaternion(self):
        return pt.transform_from_pq(np.hstack(self.get_quaternion()))
        #return pt.transform_from_pq(np.hstack((niryo_translation, niryo_rotation_q)))


# get the time step of the cniryoNEDrent world.
timeStep = int(4 * niryoNED.getBasicTimeStep())



class NiryoRobot:
    def __init__(self, robot_node, vel):
        self.homePos = [0.0, 0.0, 0.0, 0.0, -math.pi/2, 0.0, 0.0, 0.0]
        self.dropPos = [math.pi/4, math.pi/4, -math.pi/4, 0.0, 0, 0.0, 0.0, 0.0]
        
        self.x_offset = 0
        self.y_offest = 0.015
        
        self.grasp_offset = [self.x_offset, self.y_offest, 0.04]
        self.post_grasp_offset = [self.x_offset, self.y_offest, 0.1]
        self.pre_grasp_offset = [self.x_offset, self.y_offest, 0.1]
        
        self.gripperOpen = 0.02
        self.gripperClose = 0.00
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripper::left', 'gripper::right']

        self.motors = []

        self.vel = vel
        self.robot_node = robot_node

        self.setpoint_joints = []

        for i in range(0, 8):
            #print(self.joint_names[i])
            motor = self.robot_node.getDevice(self.joint_names[i])
            motor.setVelocity(vel)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            self.motors.append(motor)
            self.setpoint_joints.append(position_sensor.getValue())
            
        
    def setJoints(self, joint_values):
        for i in range(0, 5):
            self.motors[i].setPosition(joint_values[i])
            self.setpoint_joints[i] = joint_values[i]
            pass
    def getJoints(self):
       return [m.getPositionSensor().getValue() for m in self.motors]
    
    def setGripper(self, value):
        self.motors[6].setPosition(value)
        self.setpoint_joints[6] = value
        #self.motors[7].setPosition(value)
        
    def isBusy(self):
        current_joints = [m.getPositionSensor().getValue() for m in self.motors]
        error = np.subtract(current_joints, self.setpoint_joints) 
        error = abs(error)
        #print(current_joints)
        #print(self.setpoint_joints)        
        #print(error)
        busy = False
        for i in range(0, 8):
            busy = busy or (error[i] > 0.1)
        #print(busy)
        return busy
            

plot_poses = False

class Transform:
    def __init__(self, fixed_pose):
        self.tm = TransformManager()
        self.tm.add_transform('fixed', 'world',  fixed_pose)
    def get(self, requested_pose):
        self.tm.add_transform('requested', 'world',  requested_pose)
        self.tr = self.tm.get_transform('requested', 'fixed')
        
        if plot_poses:
            self.ax = self.tm.plot_frames_in("world", s=0.1)
            matplotlib.pyplot.show()
        
        #delete last transform
        return pt.pq_from_transform(self.tr)[0:3], pt.pq_from_transform(self.tr)[3:7]

plot_ik_solution = False

class KinematicsSolver:
    def __init__(self):
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete = False) as file:
            self.filename = file.name
            file.write(niryoNED.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(self.filename, active_links_mask = [False, True, True, True, True, True, True, False, False])
            
    def get(self, translation_position, rotation_euler, initial_joints, offset):
    
        self.initial_joints = [0] + initial_joints
        self.translation_position = translation_position + offset

        self.ikResults = self.armChain.inverse_kinematics(self.translation_position, rotation_euler, orientation_mode = "Z", max_iter=IKPY_MAX_ITERATIONS)#, max_iter=IKPY_MAX_ITERATIONS, initial_joints=initial_joints, orientation_mode = "all")
        
        # Keep the hand orientation down.
        #ikResults[4] = -ikResults[2] - ikResults[3] - math.pi / 2
        #print(self.ikResults)
        #ikResults[5] = -math.pi/2
        # Keep the hand orientation perpendicular.
        self.ikResults[6] = self.ikResults[1]
        
        if plot_ik_solution:
            from mpl_toolkits.mplot3d import Axes3D
            self.ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
            self.armChain.plot(self.ikResults, self.ax)
            matplotlib.pyplot.show()

        return self.ikResults[1:6]
    

kinematics_solver = KinematicsSolver()    

niryo = NiryoRobot(niryoNED, 0.5)

box_pose_handler = Pose('BOX')
niryo_pose_handler = Pose('NIRYO_NED')

niryo_pose = niryo_pose_handler.get_position_quaternion()

transform = Transform(niryo_pose)


state = 'IDLE'
state_start_time = niryoNED.getTime()


while niryoNED.step(timeStep) != -1:
    #print(state)
    current_time = niryoNED.getTime()
    
    if state == 'IDLE':
        if (current_time - state_start_time) > 4:
            state = 'START_GOTO_HOME'
            state_start_time = niryoNED.getTime()
            
    elif state == 'START_GOTO_HOME':
        niryo.setJoints(niryo.homePos)
        # Open Gripper
        niryo.setGripper(niryo.gripperOpen)
        state = 'WAIT_GOTO_HOME_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_GOTO_HOME_READY':
        niryo.isBusy()
        if not niryo.isBusy():
            state = 'START_GOTO_PRE_GRASP'
            state_start_time = niryoNED.getTime()

    elif state == 'START_GOTO_PRE_GRASP':
        box_pose = box_pose_handler.get_position_quaternion()
        translation_position, translation_rotation = transform.get(box_pose)
    
        #roll = 0
        #pitch = 0
        #yaw = 0
        
        #rotation_euler = [roll,pitch,yaw]
        rotation_euler = pr.euler_from_quaternion(translation_rotation,0,1,2, False)
        initial_joints = niryo.getJoints()
            
        joints = kinematics_solver.get(translation_position, rotation_euler, initial_joints, niryo.pre_grasp_offset)
 
        niryo.setJoints(joints)
        
        state = 'WAIT_GOTO_PRE_GRASP_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_GOTO_PRE_GRASP_READY':
        if not niryo.isBusy():
            state = 'START_GOTO_GRASP'
            #state = 'READY'
            state_start_time = niryoNED.getTime()

    elif state == 'START_GOTO_GRASP':
 
        initial_joints = niryo.getJoints()
            
        joints = kinematics_solver.get(translation_position, rotation_euler, initial_joints, niryo.grasp_offset)
 
        niryo.setJoints(joints)
        
        state = 'WAIT_GOTO_GRASP_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_GOTO_GRASP_READY':
        if not niryo.isBusy():
            state = 'START_CLOSE_GRIPPER'
            #state = 'READY'
            state_start_time = niryoNED.getTime()

    elif state == 'START_CLOSE_GRIPPER':
        niryo.setGripper(niryo.gripperClose)
        
        state = 'WAIT_CLOSE_GRIPPER_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_CLOSE_GRIPPER_READY':
        if not niryo.isBusy():
            state = 'START_GOTO_POST_GRASP'
            state_start_time = niryoNED.getTime()

    elif state == 'START_GOTO_POST_GRASP':
 
        initial_joints = niryo.getJoints()
            
        joints = kinematics_solver.get(translation_position, rotation_euler, initial_joints, niryo.post_grasp_offset)
 
        niryo.setJoints(joints)
        
        state = 'WAIT_GOTO_POST_GRASP_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_GOTO_POST_GRASP_READY':
        if not niryo.isBusy():
            state = 'START_GOTO_DROP'
            #state = 'READY'
            state_start_time = niryoNED.getTime()

    elif state == 'START_GOTO_DROP':
        niryo.setJoints(niryo.dropPos)
        
        state = 'WAIT_GOTO_DROP_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_GOTO_DROP_READY':
        if not niryo.isBusy():
            state = 'START_OPEN_GRIPPER'
            state_start_time = niryoNED.getTime()

    elif state == 'START_OPEN_GRIPPER':
        niryo.setGripper(niryo.gripperOpen)
        
        state = 'WAIT_OPEN_GRIPPER_READY'
        state_start_time = niryoNED.getTime()

    elif state == 'WAIT_OPEN_GRIPPER_READY':
        if not niryo.isBusy():
            state = 'READY'
            state_start_time = niryoNED.getTime()

    elif state == 'READY':
      supervisor_niryoNED.simulationSetMode(supervisor_niryoNED.SIMULATION_MODE_PAUSE)
      supervisor_niryoNED.simulationResetPhysics()
      print('Pause...')
      pass

    else:
        continue

    pass

# Enter here exit cleanup code.
