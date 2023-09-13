from controller import Supervisor
import sys

TIME_STEP = 32

SIMULATION_END_TIME = 20 # seconds

supervisor = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
    
i = 0
while supervisor.step(TIME_STEP) != -1:
  tr = supervisor.getTime()
  if(tr >= SIMULATION_END_TIME):
      supervisor.simulationSetMode(supervisor.SIMULATION_MODE_PAUSE)
      supervisor.simulationResetPhysics()
      print('Pause...')

  i += 1