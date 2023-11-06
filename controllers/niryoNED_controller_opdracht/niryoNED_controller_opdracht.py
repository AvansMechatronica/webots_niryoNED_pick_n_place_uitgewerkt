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


# get the time step of the cniryoNED_noderent world.
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


class Transform:
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

class TransformCalculator:
    def __init__(self, robot_transform, camera_transform, plot_transforms = False):
        self.plot_transforms = plot_transforms
        self.tm = TransformManager()
        self.tm.add_transform('robot', 'world',  robot_transform)
        #print(robot_transform)
        self.tm.add_transform('camera', 'robot',  camera_transform)
    def get(self, box_transform):
        self.tm.add_transform('box', 'camera',  box_transform)
        self.tr = self.tm.get_transform('box', 'robot')
        
        if self.plot_transforms:
            self.ax = self.tm.plot_frames_in("world", s=0.1)
            self.ax = self.tm.plot_connections_in("world")
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

        print(translation_position)
        print(rotation_euler)

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

kinematics_solver = KinematicsSolver(True)    

niryo = NiryoRobot(niryoNED_node, 0.5)

niryo_transform_handler = Transform('NIRYO_NED')
camera_transform_handler = Transform('CAMERA')

niryo_transform = niryo_transform_handler.get_position_quaternion()

camera = niryoNED_node.getDevice('camera')
if not camera.hasRecognition():
    print("No recognition")

camera.enable(timeStep)
camera.recognitionEnable(timeStep)

camera_transform = camera_transform_handler.get_position_quaternion()
transform = TransformCalculator(niryo_transform, camera_transform, True)


state = 'IDLE'
state_start_time = niryoNED_node.getTime()

dropPos = [math.pi/4, math.pi/4, -math.pi/4, 0.0, -math.pi/2, 0.0, 0.0, 0.0]

def getObjectTranslationFromCamera():
    number_of_objects = camera.getRecognitionNumberOfObjects()
    #print(number_of_objects)
    if number_of_objects:
        objects = camera.getRecognitionObjects()
        for object in objects:
            #print(object.getModel())
            position_temp = object.getPosition()
            translation_position = [position_temp[0], position_temp[1], position_temp[2]]
            print(translation_position)
            rotation_temp = object.getOrientation()
            translation_rotation = [rotation_temp[0], rotation_temp[1], rotation_temp[2], rotation_temp[3]]
            print(translation_rotation)
            
            rotation_q  = pr.quaternion_from_axis_angle(translation_rotation)
            #return self.position , self.rotation_q
            box_transform = pt.transform_from_pq(np.hstack((translation_position , rotation_q)))

            translation_position, translation_rotation = transform.get(box_transform)

            translation_rotation_euler = pr.euler_from_quaternion(translation_rotation,0,1,2, False)
    
            return True, translation_position, translation_rotation_euler
        
        pass

# pick and drop state definition
# Alle toestanden(states) kunnen meerder functies hebben b.v. Entry, Do, Exit functie. Deze worden in 1 regel als List gedefineerd
# Elke Functie dient ten minste een isReady als result te hebben, welke aangeeft dat de volgende functei/toestand aan de beurt is
pick_n_drop_states = [['isReady = niryo.setJoints(niryo.homePos)', 'isReady = niryo.setGripper(niryo.gripperClose)', 'isReady = niryo.isReady()'], # Goto to Home position
          ['isReady = timer.start(3)', 'isReady = timer.isReady()'], # wacht 3 seconden

          # Plaats hier je eigen states
          ['isReady = niryo.setJoints(dropPos)', 'isReady = niryo.isReady()'], # Ga naar de drop positie
          # Plaats hier je eigen states

          ['isReady = timer.start(3)', 'isReady = timer.isReady()']] # Wacht 3 seconden


def execute_states_list(states):

    lcls = locals()
    for state in states:
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
