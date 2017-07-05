

## Writeup / README

#### 1. Provide a Writeup / README.  

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
#### 1. DH parameters.

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

worth noting that from DH-parameters, one can make the sketch of robot arm, following rules:

![alt text][image14]

![alt text][image19]

#### 2. Individual transformations.

So, forward kinematics is a chain rotation + displacement = transformation of space from base of arm to end effector. And as a chain it can be a product of each and every those transforms:

![alt text][image12]

However, given that all of the industry works with DH parameters, a transform from one joint to another would be: 

![alt text][image13]

And as a result we can morph the transformation matrix that uses DH parameters:

![alt text][image15]

Not to repeat the code (i got warned/advices from Project1 not to repeat code), i made a method that would take the DH parameters and would use them in transformation matrix and would make calculations and return the transformation.

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

For the first run, it will save the transformation matrix, then every other time, it will check if it exist, if yes, takes that and uses, if not, makes the intensive calculations.

Because the robot can be 'kinematically-decoupled' i separated even the Forward Kinematics (FK) in 1-3 joints and 3-6, then multiplied together to get end-effector position.

```python
T0_3 = pickleit(T0_1*T1_2*T2_3, "T0_3.pckl")
T0_6 = pickleit(T3_4*T4_5*T5_6*T6_G, "T0_6.pckl")
T_EE = pickleit(T0_3*T0_6, "T_EE.pckl")
```



#### 3. Decouple Inverse Kinematics.

Well, i was surprised to learn that Inverse Kinematics (IK) is actually a very hard task. There exist a ton of quality research dealing with it (with quality i mean research papers, not blog posts, even tho the later one was sometimes more humanly readable). There are many problems with IK if you dig deep on the topic!  One of them is the DOF and redundant joints. 

As a illustration below, one robot arm, in its workspace, with two joints, has two possibilities to get top the point 3 in this case. If this arm would have just one more joint, then the possibilities would be infinite. And we know that infinite is human concept, robots dont know that :P

![alt text][image17]

Turns out, one arm is fully capable to maneuver in its workspace with 6DOF. A method named after his name, a man named Piper found out that you can simplify a quiet difficult problem into two small ones, solve each of them, then and them together for result. Kinematic decoupling can be applied to most of industrial robots that have more than 6DOF. But in order to use it, the robot must comply two iportant rules:

- Three last adjacent joint axis should intersect, and
- Three last adjacent joint are 'revolute'joints (has a spherical wrist)

![alt text][image20]

If that is satisfied, the robot arm IK can be calculated using kinematic decoupling where:

- three first joints determine position of wrist
- three last joints make for the orientation of the wrist


Firstly is important to find the wrist center position then the orientation. The wrist center is the intersection of three joints axes. So from there to get the position of whist center is quiet simple. We are actually given the desired orientation and position where the end-efector should be. And we simply transform from there to wrist center for distance d in z axis from end-efector POSE. 

So, first three thetas control position:

![alt text][image21]

Finding theta1, theta2 and theta3 then is just a mater of trigonometric math maneuvers, as seen below:

![alt text][image22]

Now for theta 4, 5, and 6.

From FK, we concluded that T06 = T03 * T36. Now from Poosition part of IK, we know T03. We need to find the orientation, or transformation from link 3 to 6 - T36. So we can derive to this equation: T36 = T03(-1) * R. And we see that the right hand side is completely known. The final three thetas can be found as a set of Euler angles corresponding to T36.

![alt text][image9]



### Project Implementation

As described before, i did not follow completely the way the code was given to us (not to repeat code). So i made few methods that wold help the process.

Firstly i made a rotation matrix method. The method would take three angles and would make a matrix out of them. As Euler's rule, any rotation can be described as three consecutive rotation in respective 3D axes.

The method would then return either INTRINSIC or EXTRINSIC  (if fixed=False) rotation matrix. I used this method to correct the rotation between the Base frame and Gripper frame.

```python
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
```

Secind method i  coded was a transformation matrix. It will take roll, pitch, yaw, x, y, z and would return a complete transformation matrix (not to mix with the homogeneous transform from DH parameters). I ended up not using it, but still decided to let it there in case i want to make any modifications.

```python
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
```

Then i made two other methods that were described above, the PICKLE method and the Transformation method (see 2nd part of this writeup on how i implemented the forward kinematics).

Then lastly i put everything together to get the code working!

To be honest, i could not make the arm moving, because i  don't see it, even-though i exported GAZEBO_MODELS! I decided to submit the homework just because i worked a lot to understand all the logic behind the forward and inverse kinematics.