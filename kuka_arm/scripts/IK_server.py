#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
#from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from kuka_arm.srv import *
from mpmath import *
from sympy import symbols, cos, sin, pi, sqrt, simplify
from sympy.matrices import Matrix
import numpy as np
from numpy import pi
from time import time

t1 = time()
# Define DH param symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angle
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #distance between x axes
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #distance between z axes
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #axis skew


s = {alpha0:     0, a0:      0, d1:  0.75,  q1: q1,
     alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,  q3: q3, 
     alpha3: -pi/2, a3: -0.054, d4:   1.5,  q4: q4,
     alpha4:  pi/2, a4:      0, d5:     0,  q5: q5, 
     alpha5: -pi/2, a5:      0, d6:     0,  q6: q6,
     alpha6:     0, a6:      0, d7: 0.303,  q7: 0}

# Create individual transformation matrices
T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                  0,                   0,            0,               1]])

T0_1 = T0_1.subs(s)

T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
               [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                  0,                   0,            0,               1]])

T1_2 = T1_2.subs(s)

T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
               [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                  0,                   0,            0,               1]])

T2_3 = T2_3.subs(s)

T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
               [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                  0,                   0,            0,               1]])

T3_4 = T3_4.subs(s)

T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
               [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                  0,                   0,            0,               1]])

T4_5 = T4_5.subs(s)

T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
               [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                  0,                   0,            0,               1]])

T5_6 = T5_6.subs(s)

T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
               [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                  0,                   0,            0,               1]])

T6_G = T6_G.subs(s)

#print('T0_1 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
T0_2 = T0_1*T1_2
#print('T0_2 = ', T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
T0_3 = T0_2*T2_3
#print('T0_3 = ', T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
T0_4 = T0_3*T3_4
#print('T0_4 = ', T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
T0_5 = T0_4*T4_5
#print('T0_5 = ', T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
T0_6 = T0_5*T5_6
#print('T0_6 = ', T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))


T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
#print('T0_G = ', T0_G.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))

r, p, y = symbols('r p y')
Rx   = Matrix([[                  1,             0,      sin(p)],
               [                  0,        cos(r),     -sin(r)],
               [                  0,        sin(r),      cos(r)]])

Ry   = Matrix([[             cos(p),             0,      sin(p)],
               [                  0,             1,           0],
               [            -sin(p),             0,      cos(p)]])

Rz   = Matrix([[             cos(y),       -sin(y),           0],
               [             sin(y),        cos(y),           0],
               [                  0,             0,           1]])

t2 = time()
print("Precalculations took: ", t2-t1)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()


        # Initialize service response
        joint_trajectory_list = []
        for i in xrange(0, len(req.poses)):

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[i].orientation.x, req.poses[i].orientation.y,
                    req.poses[i].orientation.z, req.poses[i].orientation.w])

            Rot_adj = (Rz*Ry).subs({'r':0, 'p':-pi/2, 'y':pi})
            Rot_G = (Rz*Ry*Rx).subs({'r':roll, 'p':pitch, 'y':yaw})*Rot_adj

            #T_total = T0_G*Rcorr
                        # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            d = 0.303

            print(roll, pitch, yaw)
            # Calculate joint angles using Geometric IK method
            nx = Rot_G[0,2]
            ny = Rot_G[1,2]
            nz = Rot_G[2,2]
            # WC - Wrist Center
            wx = px - d*nx
            wy = py - d*ny
            wz = pz - d*nz

            #joint 4 is wrist center
            l1  = 0.35 #length of link1 (lies on x-y plane)
            l04 = sqrt(wx*wx + wy*wy) #base to wrist center distance on x-y plane
            l24 = (l04 - l1) #joint2 to wrist center distance on x-y plane

            l02_z = 0.75 #z coordinate of joint2 (height)
            l24_z = wz - l02_z  #height from joint2 to wrist center 

            side_a = 1.501
            side_b = sqrt(l24**2 + l24_z**2)
            side_c = 1.25 #length of link3

            alpha = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))
            beta  = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))
            gamma = acos((side_a**2 + side_b**2 - side_c**2)/(2*side_a*side_b))

            theta1 = atan2(wy, wx) #project wrist center to ground plane
            theta2 = pi/2 - alpha - atan2(l24_z, l24)
            theta3 = pi/2 - beta - 0.036

            #start_time = time()
            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3] #rotation matrix from base to joint3 frame
            #print('time1: ', time()-start_time)

            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3.inv("LU") * Rot_G

            x = R3_6[0,2]
            y = R3_6[1,2]
            z = R3_6[2,2]
            #Euler angles
            theta4 = atan2(z, -x)
            theta5 = atan2(sqrt(x**2 + z**2), y)
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            #print('T0_G = ', T0_G.evalf(subs={q1: theta1, q2: theta2, q3:theta3, q4: theta4, q5:theta5, q6:theta6}))
            #print('px = ', px, ' py = ', py, ' pz = ', pz)
                
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

def test():
            # Define DH param symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angle
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #distance between x axes
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #distance between z axes
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #axis skew

            
            s = {alpha0:     0, a0:      0, d1:  0.75,
                 alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
                 alpha2:     0, a2:   1.25, d3:     0,  
                 alpha3: -pi/2, a3: -0.054, d4:  0.96,  
                 alpha4:     0, a4:      0, d5:  0.54,  
                 alpha5:     0, a5:      0, d6: 0.193,  
                 alpha6:     0, a6:      0, d7:  0.11,  q7:       0}
                
            # Create individual transformation matrices
            T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                           [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                           [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                           [                  0,                   0,            0,               1]])

            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                           [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                           [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                           [                  0,                   0,            0,               1]])

            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                           [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                           [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                           [                  0,                   0,            0,               1]])

            T2_3 = T2_3.subs(s)

            T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                           [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                           [                  0,                   0,            0,               1]])

            T3_4 = T3_4.subs(s)

            T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                           [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                           [                  0,                   0,            0,               1]])

            T4_5 = T4_5.subs(s)

            T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                           [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                           [                  0,                   0,            0,               1]])

            T5_6 = T5_6.subs(s)

            T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                           [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                           [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                           [                  0,                   0,            0,               1]])

            T6_G = T6_G.subs(s)

            print('T0_1 = ', T0_1.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))
            T0_2 = T0_1*T1_2
            print('T0_2 = ', T0_2.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))
            T0_3 = T0_2*T2_3
            print('T0_3 = ', T0_3.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))
            T0_4 = T0_3*T3_4
            print('T0_4 = ', T0_4.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))
            T0_5 = T0_4*T4_5
            print('T0_5 = ', T0_5.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))
            T0_6 = T0_5*T5_6
            print('T0_6 = ', T0_6.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))


            T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
            print('T0_G = ', T0_G.evalf(subs={q1: 0, q2: 1, q3: 0, q4: 0, q5: 0, q6:0}))


            r, p, y = symbols('r p y')
            Rz   = Matrix([[             cos(y),       -sin(y),           0,             0],
                           [             sin(y),        cos(y),           0,             0],
                           [                  0,             0,           1,             0],
                           [                  0,             0,           0,             1]])

            Ry   = Matrix([[             cos(p),             0,      sin(p),             0],
                           [                  0,             1,           0,             0],
                           [            -sin(p),             0,      cos(p),             0],
                           [                  0,             0,           0,             1]])

            Rcorr = Rz*Ry

            T_total = T0_G*Rcorr
                        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            nx = T_total[0][2]
            ny = T_total[1][2]
            nz = T_total[2][2]
            # WC - Wrist Center
            wx = px - (d6 + l)*nx
            wy = py - (d6 + l)*ny
            wz = pz - (d6 + l)*nz
        
            #First 3 joint angles:
            l3 = 1.2
            l4 = 0.95
            theta1 = atan2(wy, wx) #project wrist center to ground plane
            theta2, theta3 = RR_IK(wx, wy, wz, l3, l4)
            theta4 = 0
            theta5 = 0
            theta6 = 0



if __name__ == "__main__":
    IK_server()
    #test()

