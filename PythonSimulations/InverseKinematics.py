# In this file I'll try to implement a simple inverse kinematics exaple and simulate it in a 3D plot
# 
# 
# 
# the coordinates are defined as [x, y, z]

import math
from ntpath import join
from turtle import color
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# The home coordinates will be [0,0,0]
coord_home = [0,0,0]
coord_end = [0, 0,0]

len_coxa = 2
len_femur = 10
len_tibia = 10

joint1 = 0
joint2 = 0
joint3 = 0

point1 = [0,0,0]
point2 = [0,-5,0]
point3 = [0,0,0]






fig = plt.figure(figsize=(4,4))

ax = fig.add_subplot(111, projection='3d')

axSlider = plt.axes([0.2, 0.1, 0.65, 0.03] )
aySlider = plt.axes([0.2, 0.065, 0.65, 0.03] )
azSlider = plt.axes([0.2, 0.03, 0.65, 0.03] )
sliderX = Slider(axSlider, 'X', -10, 10, valinit=0, valstep = 0.1, color = 'red')
sliderY = Slider(aySlider, 'Y', -10, 10, valinit=0, valstep = 0.1, color = 'blue')
sliderZ = Slider(azSlider, 'Z', -10, 10, valinit=0, valstep = 0.1, color = 'green')



def forwardKinematics(ang1, ang2, ang3):
    print("Angle 1: ", ang1,"Angle 2: ", ang2,"Angle 3: ", ang3 )
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

def coord2deg(x, y): 
    if x >= 0 and y >= 0:  
        angle = math.degrees(math.atan(y/x))
    elif x < 0 and y >= 0:  
        angle = 180 + math.degrees(math.atan(y/x))
    elif x < 0 and y < 0:   
        angle = 180 + math.degrees(math.atan(y/x))
    elif x >= 0 and y < 0:  
        angle = 360 + math.degrees(math.atan(y/x))
    return round(angle,2)


def inverseKinematics(pos):
 
    x, y, z = pos[0], pos[1], pos[2]
    x += 0.0000001 # we need to avoid zero-division error

    theta1 = coord2deg(x, y)    # this is the angle from x-axis in anticlockwise rotation

    # remove the offset due to the length of coxa    
    x -= len_coxa*math.cos(math.radians(theta1))
    y -= len_coxa*math.sin(math.radians(theta1))
    
    if theta1 > 180: 
        theta1 -= 360

    P = math.sqrt(x**2 + y**2)
        
    if math.sqrt(x**2 + y**2 + z**2) > len_femur + len_tibia: 
        print("MATH ERROR: coordinate too far")
        return [theta1, 0, 0]
    
    alpha = math.atan(z/P)

    c = math.sqrt(P**2 + z**2)
    
    beta = math.acos((len_femur**2+c**2-len_tibia**2)/(2*len_femur*c))
    theta2 = beta + alpha
    theta3 = math.acos((len_tibia**2+len_femur**2-c**2)/(2*len_tibia*len_femur)) - math.pi
    
    return round(theta1,2), round(360 - math.degrees(theta2),2), round(360-math.degrees(theta3),2)




def plotUpdateX(val = 0):
    global coord_end
    coord_end[0] = val
    ax.clear()

    ang1, ang2, ang3 = inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def plotUpdateY(val = 0):
    global coord_end
    coord_end[1] = val
    ax.clear()

    ang1, ang2, ang3 = inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')

def plotUpdateZ(val = 0):
    global coord_end
    coord_end[2] = val
    ax.clear()

    ang1, ang2, ang3 = inverseKinematics(coord_end)
    point1, point2, point3 = forwardKinematics(ang1, ang2, ang3)

    ax.plot([coord_home[0],point1[0]],[coord_home[1],point1[1]],[coord_home[2],point1[2]])
    ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]])
    ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]])

    ax.plot([coord_end[0],coord_end[0]+1],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]], color='red')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]+1],[coord_end[2],coord_end[2]], color='blue')
    ax.plot([coord_end[0],coord_end[0]],[coord_end[1],coord_end[1]],[coord_end[2],coord_end[2]+1], color='green')

    ax.plot([-10,10],[0,0],[0,0], color='red')
    ax.plot([0,0],[-10,10],[0,0], color='blue')
    ax.plot([0,0],[0,0],[-10,10], color='green')


sliderX.on_changed(plotUpdateX)
sliderY.on_changed(plotUpdateY)
sliderZ.on_changed(plotUpdateZ)
plt.show()
