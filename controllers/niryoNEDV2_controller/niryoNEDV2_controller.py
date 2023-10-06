"""niryo_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import niryoNED_node, Motor, DistanceSensor
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

niryoNED_node = Supervisor()

# get the time step of the cniryoNEDrent world.
timeStep = int(4 * niryoNED_node.getBasicTimeStep())

class NiryoRobot:
    def __init__(self, robot_node, vel):
        self.homePos = [0.0, 0.0, 0.0, 0.0, -math.pi/2, 0.0, 0.0, 0.0]
        
        self.x_offset = -0.01
        self.y_offest = 0.0
        
        self.grasp_offset = [self.x_offset, self.y_offest, 0.04]
        self.post_grasp_offset = [self.x_offset, self.y_offest, 0.1]
        self.pre_grasp_offset = [self.x_offset, self.y_offest, 0.1]
        
        self.gripperOpen = 0.01
        self.gripperClose = -0.01
        
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
        return True

    def getJoints(self):
        joint_values = [m.getPositionSensor().getValue() for m in self.motors]
        return True, joint_values
    
    def setGripper(self, value):
        self.motors[6].setPosition(value)
        self.setpoint_joints[6] = value
        return True
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
        return busy
    def isReady(self):
        return not self.isBusy()

class Pose:
    def __init__(self, Def):
        self.Def = Def
        self.node = niryoNED_node.getFromDef(self.Def)
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

class Transform:
    def __init__(self, fixed_pose, plot_poses = False):
        self.plot_poses = plot_poses
        self.tm = TransformManager()
        self.tm.add_transform('fixed', 'world',  fixed_pose)
    def get(self, requested_pose):
        self.tm.add_transform('requested', 'world',  requested_pose)
        self.tr = self.tm.get_transform('requested', 'fixed')
        
        if self.plot_poses:
            self.ax = self.tm.plot_frames_in("world", s=0.1)
            matplotlib.pyplot.show()
        
        #delete last transform
        return pt.pq_from_transform(self.tr)[0:3], pt.pq_from_transform(self.tr)[3:7]


class KinematicsSolver:
    def __init__(self, plot_ik_solution = False):
        self.plot_ik_solution = plot_ik_solution
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete = False) as file:
            self.filename = file.name
            file.write(niryoNED_node.getUrdf().encode('utf-8'))
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
        
        if self.plot_ik_solution:
            from mpl_toolkits.mplot3d import Axes3D
            self.ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
            self.armChain.plot(self.ikResults, self.ax)
            matplotlib.pyplot.show()

        return True, self.ikResults[1:6]

class Timer:
    def __init__(self, device):
        self.device = device
        self.start_time = self.device.getTime()
        self.time_out = 0
        pass
    def start(self, time_out):
        self.time_out  = time_out
        self.start_time = self.device.getTime()
        return True
    def isReady(self):
        current_time = self.device.getTime()
        return (current_time - self.start_time) > self.time_out

timer = Timer(niryoNED_node)

kinematics_solver = KinematicsSolver(False)    

niryo = NiryoRobot(niryoNED_node, 0.5)

box_pose_handler = Pose('BOX')
niryo_pose_handler = Pose('NIRYO_NED')

niryo_pose = niryo_pose_handler.get_position_quaternion()

transform = Transform(niryo_pose, False)


state_start_time = niryoNED_node.getTime()

dropPos = [math.pi/4, math.pi/4, -math.pi/4, 0.0, -math.pi/2, 0.0, 0.0, 0.0]


def getBoxPosition():
    box_pose = box_pose_handler.get_position_quaternion()
    translation_position, translation_rotation = transform.get(box_pose)
        
    #roll = 0
    #pitch = 0
    #yaw = 0
    
    #rotation_euler = [roll,pitch,yaw]
    translation_rotation_euler = pr.euler_from_quaternion(translation_rotation,0,1,2, False)
    print(translation_position)
    return True, translation_position, translation_rotation_euler

# pick and drop state definition
# Alle toestanden(states) kunnen meerder functies hebben b.v. Entry, Do, Exit functie. Deze worden in 1 regel als List gedefineerd
# Elke Functie dient ten minste een isReady als result te hebben, welke aangeeft dat de volgende functei/toestand aan de beurt is
pick_n_drop_states = [['isReady = niryo.setJoints(niryo.homePos)', 'isReady = niryo.setGripper(niryo.gripperClose)', 'isReady = niryo.isReady()'], # Goto to Home position
          ['isReady = timer.start(3)', 'isReady = timer.isReady()'], # wacht 3 seconden
          ['isReady, translation_position, translation_rotation = getBoxPosition()'], # Verkrijg de positie van de op te pakken doos
          #['isReady, initial_joints = niryo.getJoints()', 'isReady, joint_values = kinematics_solver.get(translation_position, translation_rotation, initial_joints, niryo.pre_grasp_offset)'], # Bereken de jounts waardes voor de pre-grasp positie  
          #['isReady = niryo.setJoints(joint_values)', 'isReady = niryo.isReady()'], # Verplaats de robot naar de pre-grasp positie
          #['isReady = niryo.setGripper(niryo.gripperOpen)', 'isReady = niryo.isReady()'], # Open de gripper
          ['isReady, initial_joints = niryo.getJoints()', 'isReady, joint_values = kinematics_solver.get(translation_position, translation_rotation, initial_joints, niryo.grasp_offset)'],# Bereken de jounts waardes voor de grasp positie 
          #['isReady = niryo.setJoints(joint_values)', 'isReady = niryo.isReady()'],# Verplaats de robot naar de grasp positie
          #['isReady = niryo.setGripper(niryo.gripperClose)', 'isReady = niryo.isReady()'], # Sluit de gripper
          #['isReady, initial_joints = niryo.getJoints()', 'isReady, joint_values = kinematics_solver.get(translation_position, translation_rotation, initial_joints, niryo.post_grasp_offset)'],# Bereken de jounts waardes voor de post-grasp positie 
          #['isReady = niryo.setJoints(joint_values)', 'isReady = niryo.isReady()'],# Verplaats de robot naar de post-grasp positie
          #['isReady = niryo.setJoints(dropPos)', 'isReady = niryo.isReady()'], # Ga naar de drop positie
          #['isReady = niryo.setGripper(niryo.gripperOpen)', 'isReady = niryo.isReady()'], # Open de gripper
          ['isReady = timer.start(3)', 'isReady = timer.isReady()']] # Wacht 3 seconden


def execute_states_list(states):

    lcls = locals()
    for state in pick_n_drop_states:
        for function in state:
            isReady = False
            while not isReady:
                if niryoNED_node.step(timeStep) == -1:
                    break
                #print(function)
                exec(function, globals(), lcls)
                isReady = lcls["isReady"]
                #print(isReady)
            if niryoNED_node.step(timeStep) == -1:
                break
        if niryoNED_node.step(timeStep) == -1:
            break


state = "IDLE"

while niryoNED_node.step(timeStep) != -1:
    #print(state)
    if state == "IDLE":
        state = "EXECUTE_STATES"
    elif state == "EXECUTE_STATES":
        execute_states_list(pick_n_drop_states)
        state = "FINISH"
    elif state == "FINISH":
        niryoNED_node.simulationSetMode(niryoNED_node.SIMULATION_MODE_PAUSE)
        niryoNED_node.simulationResetPhysics()
        break
    else:
        print("Undefined state")

print("ready")
# Enter here exit cleanup code.
