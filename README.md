

## Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!  

[//]: # "Image References"

[image1]: ./misc_images/001.jpg
[image2]: ./misc_images/002.jpg
[image3]: ./misc_images/003.jpg
[image4]: ./misc_images/004.jpg
[image5]: ./misc_images/005.jpg
[image6]: ./misc_images/006.jpg
[image7]: ./misc_images/007.jpg
[image8]: ./misc_images/008.jpg
[image9]: ./misc_images/009.jpg
[image10]: ./misc_images/010.jpg
[image11]: ./misc_images/011.jpg
[image12]: ./misc_images/012.jpg
[image13]: ./misc_images/013.jpg
[image14]: ./misc_images/014.jpg
[image15]: ./misc_images/015.jpg
[image16]: ./misc_images/016.jpg
[image17]: ./misc_images/017.jpg
[image18]: ./misc_images/018.jpg
[image19]: ./misc_images/019.jpg
[image20]: ./misc_images/020.jpg
[image21]: ./misc_images/021.jpg
[image22]: ./misc_images/022.jpg

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Based on URDF.XARCO file the DH parameters were extracted as below:

|   i   | alpha(i-1) | a(i-1) | d(i)  | theta(i) |
| :---: | :--------: | :----: | :---: | :------: |
| **1** |     0      |   0    | 0.75  |    q1    |
| **2** |   -pi/2    |  0.35  |   0   | q2-pi/2  |
| **3** |     0      |  1.25  |   0   |    q3    |
| **4** |   -pi/2    | -0.054 |  1.5  |    q4    |
| **5** |    pi/2    |   0    |   0   |    q5    |
| **6** |   -pi/2    |   0    |   0   |    q6    |
| **7** |     0      |   0    | 0.303 |    0     |

worth noting that from DH-parameters, one can make the scratch of robot arm, following rules:

![alt text][image14]

![alt text][image19]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

So, for forward kinematics, i used the homogeneous matrix:

So, forward kinematics is a chain rotation + displacement = transformation of space from base of arm to end effector. And as a chain it can be a product of each and every those transforms:

![alt text][image12]

However, given that all of the industry works with DH parameters, a transform from one joint to another would be: 

![alt text][image13]

And as a result we can morph the transformation matrix that uses DH parameters:

![alt text][image15]

So,, not to repeat the code (i got warned/advices from Project1 not to repeat code), i made a method that would take the DH parameters and would use them in transformation matrix and would make calculations and return the transformation.

```python
def homogen(alpha, a, d, phi):
    T = Matrix([[            cos(phi),           -sin(phi),           0,             a],
                [ sin(phi)cos(alpha), cos(phi)cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(phi)sin(alpha), cos(phi)sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                   0,                   0,           0,             1]])
return simplify(T)
```

Then, i would call the method for each **i** (row of DH table), that would make transformation matrices for each joint.


```python
T0_1 = homogen(    0,      0,  0.75,      q1)
T1_2 = homogen(-pi/2,   0.35,     0, q2-pi/2)
T2_3 = homogen(    0,   1.25,     0,      q3)
T3_4 = homogen(-pi/2, -0.054,   1.5,      q4)
T4_5 = homogen( pi/2,      0,     0,      q5)
T5_6 = homogen(-pi/2,      0,     0,      q6)
T6_G = homogen(    0,      0, 0.303,       0)
```

Because this is a CPU intensive task it would take some seconds to make the calculations from base joint to end effector, i coded a method that would save the results as a pickle file. Since this is calculated only once, as the link length or angles of DH don't change (not to mix with joints angle 'revolute joints' and joints length 'prismatic joints'). 

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

So for the first run, it will save the transformation matrix, then every other time, it will check if it exist, if yes, takes that and uses, if not, makes the intensive calculations.

Because the robot can be 'kinematically-decoupled' i separated even the FK in 1-3 joints and 3-6, then multiplied together to get end-effector-

```python
T0_3 = pickleit(T0_1*T1_2*T2_3, "T0_3.pckl")
T0_6 = pickleit(T3_4*T4_5*T5_6*T6_G, "T0_6.pckl")
T_EE = pickleit(T0_3*T0_6, "T_EE.pckl")
```





#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image20]

![alt text][image21]

![alt text][image22]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 





