#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
All vector are defined inside the robot frame, no world frame is used for now.

So, we have 4 frames on each leg and one which is commun for all, which is the geometric center of the robot (very close to the CoM).

For our setup, there are 2 important frames on the leg, which are the frame0 (coxa frame or first frame on the leg) and frame4 (foot frame or last frame on the leg).

So now, we are going to calculate Inverse Kinematics between those frames
"""
import numpy

def checkdomain(D):
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1: 
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D
#this is based on this paper: 
#"https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/____________  x       "
"""
#IK equations now written in pybullet frame.

"""
#coord -- coordinate of foot frame
    -coord[0]--z 
     coord[1]--x
     coord[2]--y

#coxa -- length L1 
#femur -- length L2 
#tibia -- length L3 

#theta -- 01
#alpha -- 02
#gamma -- 03


#These fuction provide angles that attain by each leg when position of foot is provided
for geting the angle for the right leg -- solve_R()
for geting the angle for the left leg -- solve_L()
"""

def solve_R(coord , coxa , femur , tibia):  
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles

def solve_L(coord , coxa , femur , tibia):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles


# Now we can locate foot in the 3D space, inside the frame0 (i.e. w.r.t to frame0, the coxa frame)

#Next step is to move the body frame in relation with the feet. Which is essentialy the kinematic model, kinematic_model.py file in the code.