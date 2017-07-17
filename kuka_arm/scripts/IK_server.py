#!/usr/bin/env python
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy
import pickle, os

p0, p1, p2, p3, p4, p5, p6 = symbols('p0:7') # twist angles
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angles

def rotate(R, fixed=False):   
    roll = Matrix([[1,         0,          0],
                   [0, cos(R[0]), -sin(R[0])],
                   [0, sin(R[0]),  cos(R[0])]])
    pitch = Matrix([[ cos(R[1]), 0,  sin(R[1])],
                    [         0, 1,          0],
                    [-sin(R[1]), 0,  cos(R[1])]])
    yaw = Matrix([[cos(R[2]), -sin(R[2]), 0],
                  [sin(R[2]),  cos(R[2]), 0],
                  [        0,          0, 1]])
    if fixed:
        return simplify(roll * pitch * yaw)
    else:
        return simplify(yaw * pitch * roll)
def transform(R, D, fixed=False):
    roll = Matrix([[1,         0,          0],
                   [0, cos(R[0]), -sin(R[0])],
                   [0, sin(R[0]),  cos(R[0])]])
    pitch = Matrix([[ cos(R[1]), 0,  sin(R[1])],
                    [         0, 1,          0],
                    [-sin(R[1]), 0,  cos(R[1])]])
    yaw = Matrix([[cos(R[2]), -sin(R[2]), 0],
                  [sin(R[2]),  cos(R[2]), 0],
                  [        0,          0, 1]])
    if fixed:
        r = roll * pitch * yaw
    else:
        r = yaw * pitch * roll
    T = Matrix([[r[0, 0], r[0, 1], r[0, 2], D[0]],
                [r[1, 0], r[1, 1], r[1, 2], D[1]],
                [r[2, 0], r[2, 1], r[2, 2], D[2]], 
                [      0,       0,       0,   1]])
    return simplify(T)
def homogen(alpha, a, d, phi):
    T = Matrix([[            cos(phi),           -sin(phi),           0,             a],
                [ sin(phi)*cos(alpha), cos(phi)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(phi)*sin(alpha), cos(phi)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                   0,                   0,           0,             1]])
    return simplify(T)
def DH():
    return {p0: 0,         a0: 0,      d1: 0.75,
            p1: -pi/2,     a1: 0.35,   d2: 0,      q2: q2 - pi / 2,
            p2: 0,         a2: 1.25,   d3: 0,
            p3: -pi/2,     a3: -0.054, d4: 1.5,
            p4: pi/2,      a4: 0,      d5: 0,
            p5: -pi/2,     a5: 0,      d6: 0,
            p6: 0,         a6: 0,      d7: 0.303,  q7: 0}
def pickleit(filename, M=0, readonly=false):
    if readonly:
        file = open(filename, 'rb')
        N = pickle.load(file)
        file.close()
    else:
        if os.path.isfile(filename):
            file = open(filename, 'rb')
            N = pickle.load(file)
            file.close()
        else:
            N = simplify(M)
            file = open(filename, 'wb')
            pickle.dump(N, file)
            file.close()      
    return N
def wrist(position, orientation):
    posx, posy, posz = position
    roll, pitch, yaw = orientation
    O_EE = rotate([roll, pitch, yaw], fixed=True)
    P_EE = Matrix([[posx],[posy],[posz]])
    P_WC = simplify(P_EE - 0.303 * O_EE * Matrix([[0],[0],[1]]))
    return P_WC, O_EE
def forward_kinematics(dh_table):
    T0_1 = homogen(p0, a0, d1, q1).subs(dh_table)
    T1_2 = homogen(p1, a1, d2, q2).subs(dh_table)
    T2_3 = homogen(p2, a2, d3, q3).subs(dh_table)
    T3_4 = homogen(p3, a3, d4, q4).subs(dh_table)
    T4_5 = homogen(p4, a4, d5, q5).subs(dh_table)
    T5_6 = homogen(p5, a5, d6, q6).subs(dh_table)
    T6_G = homogen(p6, a6, d7, q7).subs(dh_table)

    #   FORWARD KINEMATICS   #####################################################################
    T0_3 = pickleit("T0_3.pckl", T0_1*T1_2*T2_3)
    T0_6 = pickleit("T0_6.pckl", T3_4*T4_5*T5_6*T6_G)
    T_EE = pickleit("T_EE.pckl", T0_3*T0_6)
    T_CO = pickleit("T_CO.pckl", transform([0, -pi/2, pi], [0,0,0]))
    TO_G = pickleit("TO_G.pckl", T_EE*T_CO)
    
def inverse_kinematics(position, orientation):
    wrist_center, wrist_orient = wrist(position, orientation)

    ### POSITION #######################################################################################
    wx, wy, wz = wrist_center[0], wrist_center[1], wrist_center[2]
    d2, a3, d3, a4 = 0, 1.25, -0.054, 1.5
    l2 = sqrt(d2**2 + a3**2)
    l3 = sqrt(d3**2 + a4**2)

    '''
    h1 = sqrt(wx**2 + wy**2)
    theta1 = atan2(wy, wx)
    ct3 = cos((wx**2 + wy**2 + wz**2 - l2**2 - l3**2)/2*l2*l3)
    theta3 = atan2(-1 * sqrt(1-(ct3**2)), ct3)
    as2 = sin(theta3)*l3
    ac2 = cos(theta3)*l3
    s1=((l2+ac2) * wz - as2 * h1) / (h1**2+ wz**2)
    c1=((l2+ac2) * h1 + as2 * wz) / (h1**2 + wz**2)
    theta2 = atan2(s1, c1)
    '''
    ca = sqrt(l2*2 + l3*3)
    c3 = cos((wx*2 + wy*2 + wz*2 - l2*2 - l3*2)/2*l2*l3)
    s3 = sin(sqrt(1-c3))

    theta1 = atan2(wy, wx)
    theta3 = atan2(s3, c3)
    theta2 = atan2(wz, ca) - atan2(l2*s3, l2 + l3*c3)
   
    ### ORIENTATION ####################################################################################
    R0_3 = pickleit("T0_3.pckl", readonly=true)[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    R3_6 = R0_3.T * wrist_orient

    theta6 = atan2(R3_6[1,0],R3_6[0,0])
    theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0]))
    theta4 = atan2(R3_6[2,1],R3_6[2,2])
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]

def calculations(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        #forward_kinematics(DH())
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()
            position = req.poses[x].position.x, req.poses[x].position.y, req.poses[x].position.z
            orientation = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x, req.poses[x].orientation.y, 
                req.poses[x].orientation.z, req.poses[x].orientation.w])
        
	    joint_trajectory_point.positions = inverse_kinematics(position, orientation)
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)
def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, calculations)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()