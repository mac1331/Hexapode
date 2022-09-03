# In this file I'll try to implement a simple forward kinematics exaple and simulate it in a 3D plot
# 
# 
# 
# the coordinates are defined as [x, y, z]

import math
from ntpath import join
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# The home coordinates will be [0,0,0]
coord_home = [0,0,0]

len_coxa = 2
len_femur = 10
len_tibia = 10

joint1 = 0
joint2 = 0
joint3 = 0

point1 = [0,0,0]
point2 = [0,-5,0]
point3 = [0,0,0]



coord_end = [5, 5,-5]



fig = plt.figure(figsize=(4,4))

ax = fig.add_subplot(111, projection='3d')

axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
sliderX = Slider(axSlider, 'X', -0, 360.0, valinit=0, valstep = 1)
sliderY = Slider(aySlider, 'Y', -0, 360.0, valinit=0, valstep = 1)
sliderZ = Slider(azSlider, 'Z', -0, 360.0, valinit=0, valstep = 1)



def forwardKinematics(ang1, ang2, ang3):
    # point 1
    trans = np.dot(np.matrix([[1,0,0,len_coxa],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point1[0] = trans[(0,0)]
    point1[1] = trans[(1,0)]
    point1[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point1[0],point1[1],point1[2],1]).transpose())
    
    point1[0] = rot[(0,0)]
    point1[1] = rot[(1,0)]
    point1[2] = rot[(2,0)]


    # point 2
    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_coxa],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    point2[0] = trans[(0,0)]
    point2[1] = trans[(1,0)]
    point2[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point2[0],point2[1],point2[2],1]).transpose())
    
    point2[0] = rot[(0,0)]
    point2[1] = rot[(1,0)]
    point2[2] = rot[(2,0)]


    # point 3
    trans = np.dot(np.matrix([[1,0,0,len_tibia],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([coord_home[0],coord_home[1],coord_home[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang3)),0,math.sin(math.radians(ang3)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang3)),0,math.cos(math.radians(ang3)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]

    trans = np.dot(np.matrix([[1,0,0,len_femur],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang2)),0,math.sin(math.radians(ang2)),0],
                        [0,1,0,0],
                        [-math.sin(math.radians(ang2)),0,math.cos(math.radians(ang2)),0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())

    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)] 
    
    trans = np.dot(np.matrix([[1,0,0,len_coxa],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    point3[0] = trans[(0,0)]
    point3[1] = trans[(1,0)]
    point3[2] = trans[(2,0)]

    rot = np.dot(np.matrix([[math.cos(math.radians(ang1)),-math.sin(math.radians(ang1)),0,0],
                        [math.sin(math.radians(ang1)),math.cos(math.radians(ang1)),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]),np.matrix([point3[0],point3[1],point3[2],1]).transpose())
    
    point3[0] = rot[(0,0)]
    point3[1] = rot[(1,0)]
    point3[2] = rot[(2,0)]


    return point1, point2, point3




def plotUpdateX(val = 0):
    global joint1
    joint1 = val
    ax.clear()

    point1, point2, point3 = forwardKinematics(joint1,joint2,joint3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def plotUpdateY(val = 0):
    global joint2
    joint2 = val
    ax.clear()

    point1, point2, point3 = forwardKinematics(joint1,joint2,joint3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])


    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def plotUpdateZ(val = 0):
    global joint3
    joint3 = val
    ax.clear()

    point1, point2, point3 = forwardKinematics(joint1,joint2,joint3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')


sliderX.on_changed(plotUpdateX)
sliderY.on_changed(plotUpdateY)
sliderZ.on_changed(plotUpdateZ)
plt.show()
