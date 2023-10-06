

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

class NiryoRobot:
    def __init__(self, robot_node, vel):
        self.homePos = [0.0, 0, 0.0, 0.0, -math.pi/2, 0.0, 0.0, 0.0]
        
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

