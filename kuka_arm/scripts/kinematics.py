from sympy import *
from sympy.matrices import Matrix
import numpy as np
import pickle, os

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

def pickleit(M, filename):
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

#define symbolics
p0, p1, p2, p3, p4, p5, p6 = symbols('p0:7') # twist angles
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angles

T0_1 = homogen(    0,      0,  0.75,      q1)
T1_2 = homogen(-pi/2,   0.35,     0, q2-pi/2)
T2_3 = homogen(    0,   1.25,     0,      q3)
T3_4 = homogen(-pi/2, -0.054,   1.5,      q4)
T4_5 = homogen( pi/2,      0,     0,      q5)
T5_6 = homogen(-pi/2,      0,     0,      q6)
T6_G = homogen(    0,      0, 0.303,       0)

###
px, py, pz = 1.2, 0.8, 0.1
roll, pitch, yaw = 0, 0, 0
###

### calculate the correction
T0_3 = pickleit(T0_1*T1_2*T2_3, "T0_3.pckl")
T0_6 = pickleit(T3_4*T4_5*T5_6*T6_G, "T0_6.pckl")
T_EE = pickleit(T0_3*T0_6, "T_EE.pckl")
T_CO = pickleit(transform([0, -pi/2, pi], [0,0,0]), "T_CO.pckl")

### last transform
T0_G = T_EE*T_CO

### in the loop
### kinematic decoupling
O_EE = rotate([roll, pitch, yaw], fixed=True)
P_EE = Matrix([[px],[py],[pz]])
P_WC = simplify(P_EE - 0.303 * O_EE * Matrix([[0],[0],[1]]))

### POSITION
a2 = 0.35
a3 = 1.25
ca = sqrt(a2*a2 + a3*a2)
d6 = 0.303
xc = P_WC[0]
yc = P_WC[1]
zc = P_WC[2]

c3 = cos((xc*xc + yc*yc + zc*zc - a2*a2 - a3*a3)/2*a2*a3)
s3 = sin(sqrt(1-c3*c3))

theta1 = atan2(yc, xc)
theta2 = atan2(s3, c3)
theta3 = atan2(zc, ca) - atan2(a2*s3, a2 + a3*c3)

R0_3 = T0_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
I0_3 = (-1) * R0_3
R3_6 = I0_3 * O_EE

theta6 = atan2(R3_6[1,0],R3_6[0,0])
theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0]))
theta4 = atan2(R3_6[2,1],R3_6[2,2])

print theta1, theta2, theta3
print theta4, theta5, theta6

