"""conveyor_belt_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import conveyor_belt, Motor, Distancedistance_sensor
#from controller import conveyor_belt
from controller import Supervisor
import random
# create the conveyor_belt instance.
supervisor = Supervisor()
conveyor_belt = supervisor


#set distance distance_sensor to random position
distance_sensor_node = supervisor.getFromDef("DIST_SENSOR")
trans_field = distance_sensor_node.getField("translation")
distance_sensor_position = trans_field.getSFVec3f()
# calculate new translation
distance_sensor_position[0] = 0.50 + (random.random() * 0.2)
trans_field.setSFVec3f(distance_sensor_position)
distance_sensor_node.resetPhysics()

# get the time step of the current world.
timestep = int(conveyor_belt.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the conveyor_belt. Something like:
belt = conveyor_belt.getDevice('belt_motor')
ds = conveyor_belt.getDevice('Distance Sensor')
ds.enable(timestep)

belt.setVelocity(0.1)
belt.setPosition(float('inf'))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while conveyor_belt.step(timestep) != -1:
    # Read the distance_sensors:
    # Enter here functions to read distance_sensor data, like:
    val = ds.getValue()
    #print(val)

    if val > 1.0:
        belt.setVelocity(0)
    
    pass

# Enter here exit cleanup code.
