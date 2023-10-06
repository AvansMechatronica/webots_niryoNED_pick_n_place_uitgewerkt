"""pioneer_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import pioneer_device, Motor, DistanceSensor
from controller import Robot

# create the pioneer_device instance.
pioneer_device = Robot()

# get the time step of the current world.
timestep = int(pioneer_device.getBasicTimeStep())


class Pioneer3at:
    def __init__(self, device):
        self.device = device
        self.LF_motor = self.device.getDevice('front left wheel')
        self.RF_motor = self.device.getDevice('front right wheel')
        self.LB_motor = self.device.getDevice('back left wheel')
        self.RB_motor = self.device.getDevice('back right wheel')
        
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
        
        
pioneer = Pioneer3at(pioneer_device)

camera = pioneer_device.getDevice('camera')
if not camera.hasRecognition():
    print("No recognition")
else:
    camera.enable(timestep)
    camera.recognitionEnable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the pioneer_device. Something like:
#  motor = pioneer_device.getDevice('motorname')
#  ds = pioneer_device.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

state = "IDLE"

timer = Timer(pioneer_device)

while pioneer_device.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    #print(state)

    if state == "IDLE":
        pioneer.moveForward(4);
        timer.start(2)
        state = "MOVE_FORWARD"
    elif state == "MOVE_FORWARD":
        if timer.isReady():
            pioneer.rotateRight(1.5)
            timer.start(3)
            state = "TURN_RIGHT"
    elif state == "TURN_RIGHT":
        if timer.isReady():
            pioneer.moveForward(0)
            state = "READY"
    elif state == "READY":
        break
    else:
        print("Invalid state") 


# Enter here exit cleanup code.
