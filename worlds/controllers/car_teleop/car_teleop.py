''' 
R Navin Sriram / 13/9/23 / n47insriram@gmail.com
No constraints on lead car dynamics
Classic WASD and SPACE for accel, L, R, Reverse, Brake
TO DO - include pre programmed scenarios  
  '''
from controller import Robot, Keyboard
from vehicle import Driver
import numpy as np
import math


# based on experimental tests for the Tesla model S,
# Brake Intensity of 0.9 delivered a deceleration of 6.43 m/s^2
# so we take max braking deceleration as 6.43/0.9

step=32
speed=0
Cd=0.3            # coefficient of drag
A=2.4             # frontal area
m=100            # mass in Kg
Rc=0.01           # Rolling Resistance coefficient
Vmax=8.3          # m/s
Amax = 1.2        # max Acceleration
Dmax = 7.14        # max deceleration ie. BrakingIntensity(1)
Pmax=5000         # max power output
Tmax =110         # max wheel torque
R=0.24            # radius of the wheel
Rpmmax= Vmax * 60 / (6.28 * R)
rev_speed = 25    # max rev speed in kmph

driver = Driver()
keyboard= Keyboard()
keyboard.enable(step)
 
    
def drag(speed):

  air_drag = 0.5*1.1*Cd*A*(speed**2)     # rolling and air resistance 
  rolling_res=Rc*9.8*m
  drag_tot= air_drag + rolling_res
  drag_decel=drag_tot/m
  drag_decelf = min(Dmax,drag_decel)  # to make sure that drag remains
  bi_drag= drag_decelf / Dmax         # mapped to brake intensity
  return bi_drag


# main function performing vehicle functions
def drive(cmd_trq,cmd_steer,cmd_vel):
   driver.setThrottle(cmd_trq)
   driver.setSteeringAngle(cmd_steer)
   
   # want negative command steer

#first element is the cmd_trq, second is stee-
#-ring angle, third is gear, fourth is brakes, Fifth is cmd_vel mode for future use


while driver.step() != -1:
                        
    speed=driver.getCurrentSpeed()  # kmph 
    # print(speed)
    key=keyboard.getKey()
    driver.setGear(1)            # setting up keyboard node
    bi_drag=drag(speed)
    drive(0.4,-0.05,100)
                       
       
    # if key in command.keys():
       # driver.setGear(1)
       # drive(1,command[key][1],command[key][4])
       # driver.setBrakeIntensity(bi_drag + 0.5*command[key][3])
       
    # elif key == ord("E"):
       
       # driver.setCruisingSpeed(30)      # const speed mode
       
         
    # else:
       # driver.setCruisingSpeed(0.0)     # no inputs, comes to stop gradually because of drag
       # driver.setThrottle(0.0)
       # driver.setBrakeIntensity(bi_drag)
       