## Writeup / README
#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!  

[//]: # "Image References"

[image1]: ./misc_images/001.jpg
[image2]: ./misc_images/002.jpg
[image3]: ./misc_images/003.jpg

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.



|   i   | alpha(i-1) | a(i-1) | d(i)  | theta(i) |
| :---: | :--------: | :----: | :---: | :------: |
| **1** |     0      |   0    | 0.75  |    q1    |
| **2** |   -pi/2    |  0.35  |   0   | q2-pi/2  |
| **3** |     0      |  1.25  |   0   |    q3    |
| **4** |   -pi/2    | -0.054 |  1.5  |    q4    |
| **5** |    pi/2    |   0    |   0   |    q5    |
| **6** |   -pi/2    |   0    |   0   |    q6    |
| **7** |     0      |   0    | 0.303 |    0     |



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

```python
def homogen(alpha, a, d, phi):
    T = Matrix([[            cos(phi),           -sin(phi),           0,             a],
                [ sin(phi)cos(alpha), cos(phi)cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(phi)sin(alpha), cos(phi)sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                   0,                   0,           0,             1]])
return simplify(T)
```


```python
T0_1 = homogen(    0,      0,  0.75,      q1)
T1_2 = homogen(-pi/2,   0.35,     0, q2-pi/2)
T2_3 = homogen(    0,   1.25,     0,      q3)
T3_4 = homogen(-pi/2, -0.054,   1.5,      q4)
T4_5 = homogen( pi/2,      0,     0,      q5)
T5_6 = homogen(-pi/2,      0,     0,      q6)
T6_G = homogen(    0,      0, 0.303,       0)
```



```python
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
```



```python
T0_3 = pickleit(T0_1*T1_2*T2_3, "T0_3.pckl")
T0_6 = pickleit(T3_4*T4_5*T5_6*T6_G, "T0_6.pckl")
T_EE = pickleit(T0_3*T0_6, "T_EE.pckl")
```



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][./misc_images/003.jpg]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 





