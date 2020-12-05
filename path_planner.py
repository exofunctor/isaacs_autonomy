import rospy
import numpy as np
import dji_control

max_x = 20
max_y = 20
threshold = 0.3
grid = np.zeros((max_x,max_y)) #occupancy grid
traversed = np.zeros((max_x, max_y)) #have we already been there?

def flood_fill(x,y):
    if is_open(x+1, y):
        move_forward()
        flood_fill(x+1,y)
    if is_open(x-1, y):
        move_back()
        flood_fill(x-1,y)
    if is_open(x, y+1):
        move_right()
        flood_fill(x,y+1)
    if is_open(x, y-1):
        move_left()
        flood_fill(x,y-1)

def is_open(x,y):
    if x < 0 or y < 0 or x > max_x or y > max_y:
        return False
    if traversed[x, y] > 0:
        return False
    return grid[x, y] < threshold

def move_forward():

    #DJI publish message

    traversed[x,y] = 1

def move_left():
    #rotate
    move_forward()

def move_right()
    #rotate
    move_forward()

def move_back():
    #rotate
    move_forward()
