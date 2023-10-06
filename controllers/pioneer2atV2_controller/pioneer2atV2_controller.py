"""pioneer_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import pioneer_node, Motor, DistanceSensor
from controller import Robot

# create the pioneer_node instance.
pioneer_node = Robot()

# get the time step of the current world.
timestep = int(pioneer_node.getBasicTimeStep())


class Pioneer3at:
    def __init__(self, node):
        self.node = node
        self.LF_motor = self.node.getDevice('front left wheel')
        self.RF_motor = self.node.getDevice('front right wheel')
        self.LB_motor = self.node.getDevice('back left wheel')
        self.RB_motor = self.node.getDevice('back right wheel')
        
        self.LF_motor.setVelocity(1)
        self.RF_motor.setVelocity(1)
        self.LB_motor.setVelocity(1)
        self.RB_motor.setVelocity(1)
        
        self.LF_motor.setPosition(float('inf'))
        self.RF_motor.setPosition(float('inf'))
        self.LB_motor.setPosition(float('inf'))
        self.RB_motor.setPosition(float('inf'))

    def moveForward(self, speed):
        self.LF_motor.setVelocity(speed)
        self.RF_motor.setVelocity(speed)
        self.LB_motor.setVelocity(speed)
        self.RB_motor.setVelocity(speed)
        return True
    
    def moveBackward(self, speed):
        self.LF_motor.setVelocity(-speed)
        self.RF_motor.setVelocity(-speed)
        self.LB_motor.setVelocity(-speed)
        self.RB_motor.setVelocity(-speed)
        return True
    
    def rotateRight(self, speed):
        self.LF_motor.setVelocity(-speed)
        self.RF_motor.setVelocity(speed)
        self.LB_motor.setVelocity(-speed)
        self.RB_motor.setVelocity(speed)
        return True
    
    def rotateLeft(self, speed):
        self.LF_motor.setVelocity(speed)
        self.RF_motor.setVelocity(-speed)
        self.LB_motor.setVelocity(speed)
        self.RB_motor.setVelocity(-speed)
        return True

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
        
        
pioneer = Pioneer3at(pioneer_node)

use_camera = False
use_recognition = False

if use_camera:
    camera = pioneer_node.getDevice('camera')
    camera.enable(timestep)
	if use_recognition:
		if not camera.hasRecognition():
			print("Camera has no recognition")
		else:
			camera.recognitionEnable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the pioneer_node. Something like:
#  motor = pioneer_node.getDevice('motorname')
#  ds = pioneer_node.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


timer = Timer(pioneer_node)


# Voeg hier je eigen bewegingen aan toe
# movement_states definition
# Alle toestanden(states) kunnen meerder functies hebben b.v. Entry, Do, Exit functie. Deze worden in 1 regel als List gedefineerd
# Elke Functie dient ten minste een isReady als result te hebben, welke aangeeft dat de volgende functie/toestand aan de beurt is
movement_states =[['isReady = pioneer.moveForward(4)', 'isReady = timer.start(2)', 'isReady = timer.isReady()'],
                  ['isReady = pioneer.rotateRight(1.5)', 'isReady = timer.start(1.5)', 'isReady = timer.isReady()'],
                  ['isReady = pioneer.moveForward(3)', 'isReady = timer.start(3)', 'isReady = timer.isReady()'],
                  ['isReady = pioneer.moveForward(0)', 'isReady = timer.start(1.5)', 'isReady = timer.isReady()']]


def execute_states_list(states):

    lcls = locals()
    for state in states:
        for function in state:
            isReady = False
            while not isReady:
                if pioneer_node.step(timestep) == -1:
                    break
                #print(function)
                exec(function, globals(), lcls)
                isReady = lcls["isReady"]
                #print(isReady)
            if pioneer_node.step(timestep) == -1:
                break
        if pioneer_node.step(timestep) == -1:
            break


state = "IDLE"

while pioneer_node.step(timestep) != -1:
    #print(state)
    if state == "IDLE":
        state = "EXECUTE_STATES"
    elif state == "EXECUTE_STATES":
        execute_states_list(movement_states)
        state = "FINISH"
    elif state == "FINISH":
        #pioneer_node.simulationSetMode(supervisor_niryoNED.SIMULATION_MODE_PAUSE)
        #pioneer_node.simulationResetPhysics()
        break
    else:
        print("Undefined state")

print("ready")

# Enter here exit cleanup code.
