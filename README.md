# 3R-Motion
Calculation of the motion of the 3R robotic arm TriVex SL/SLi vertical pack loader. Final assignment of the Kinematics and Dynamics course.

This repository contains the "RobotArm3R" script, which, when run, simulates the vertical movement of the end effector of the TriVex SL/SLi vertical pack loader 3R robotic arm that can be seen in the following video: https://www.youtube.com/watch?v=58p0063SkLE&feature=youtu.be.

The objective of the script is that, known the parameters of the joints of the robotic arm and given the vertical movement wanted to be performed by the end effector it finds the movements that each joint need to make during the process.

The other files in the repository are all the functions needed in the script. In the beginning of each file there is its description commented.

## Results

3R-motion

In the following gif there is the representation of the robot arm performing the desired motion resulted from the script mentioned above.

<img src="Images%20and%20gifs/3Rmotion.gif" width="500">

Joint speeds versus time

The following picture figure is the plot of each joint speed versus time.

<img src="Images%20and%20gifs/3RJointSpeeds.PNG" width="500">


I think that the process done to achieve this results can be easily understanded from the written code.

I've not been able to find time to do the optional part even if I had the intentions to do so. I may try to do it sometime in the near future.

