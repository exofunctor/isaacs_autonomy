import matplotlib.pyplot as plt
import numpy as np
import cv2
import time

'''
matrix = np.random.random((10,10))
plt.imshow(matrix)

plt.show()'''

# draw the figure so the animations will work

x = 0
z = 0
done = False
sleep = 0.2
grid = np.matrix([
        [0,0,-2,0,0,0],
        [0,0,-2,0,0,0],
        [0,0,-2,0,0,0],
        [0,-2,-2,-2,0,0],
        [0,0,0,0,0,0],
        [0,0,0,0,0,0]])

max_x = grid.shape[0]
max_z = grid.shape[1]
traversed = np.zeros((max_x, max_z))
traversed[x,z] = 1
threshold = -1

def explore():
    print("beginning search")

    print("max_x: ", max_x)
    print("max_z: ", max_z)
    #max_x = self.Grid.grid.shape[1] #2*radius
    #max_z = self.Grid.grid.shape[0] #2*radius
    #threshold = 0.3

    x = get_x()
    z = get_z()

    def is_open(x, z):
        if x < 0 or z < 0 or x >= max_x or z >= max_z:
            return False
        if traversed[int(x), int(z)] > 0:
            print("x: ", int(x), "z: ", int(z), "is OCCUPIED/TRAVELED")
            return False
        occupied = grid[int(x), int(z)]

        print("x: ", x, "z: ", z, "is OPEN. going there next")
        return  occupied > threshold #returns false if occupied = 0 for safety reasons

    def flood_fill(x,z):
        try:
            if check_mission_accomplished():
                return
            update_map()
            if is_open(x, z+1):
                move_up(x, z+1)
                flood_fill(x,z+1)
                if check_mission_accomplished():
                    return
                move_down(x, z)
            #need to come back to where we started from, because we can't jump around the map
            if is_open(x, z-1):
                move_down(x, z-1)
                flood_fill(x,z-1)
                if check_mission_accomplished():
                    return
                move_up(x, z)
            if is_open(x+1, z):
                move_right(x+1, z)
                flood_fill(x+1,z)
                if check_mission_accomplished():
                    return
                move_left(x,z)
            if is_open(x-1, z):
                move_left(x-1,z)
                flood_fill(x-1, z)
                if check_mission_accomplished():
                    return
                move_right(x,z)
            return
        except KeyboardInterrupt:
            print("Exiting...")

    flood_fill(x, z)


def move_up(desired_x, desired_z):
    #need to go from (x,z) to (x,z+1)
    #print("moving forward")
    x = get_x()
    z = get_z()
    print("moving forward, from z=", z, " to z=", desired_z)

    #first, rotate so we are facing up(forwards)
    rotate(0)

    #go from (x,z) to (x, z+1)
    while(abs(desired_z - z) > 0.2):
        #print("current z: ", z)
        #print("trying to go to: ", desired_z)
        set_z(desired_z)
        z = get_z()


    #update traversal matrix
    traversed[int(desired_x), int(desired_z)] = 1
    print(traversed)
    #print(traversed)
    print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")
    if np.all(traversed == 1):
        global done
        done = True

    #update occupancy grid
    update_map()
    time.sleep(sleep)

def move_left(desired_x, desired_z):
    #need to go from (x,z) to (x-1, z)
    #print("moving left")
    x = get_x()
    z = get_z()
    print("moving left, from x=", x, " to x=", desired_x)

    rotate((3/2)*np.pi)
    #TODO: move
    while(abs(x - desired_x) > 0.2):
        #print("current x: ", x)
        #print("trying to go to: ", desired_x)
        #print("Going left.")
        set_x(desired_x)
        x = get_x()

    traversed[int(desired_x), int(desired_z)] = 1
    print(traversed)
    if np.all(traversed == 1):
        global done
        done = True
    print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")
    update_map()
    time.sleep(sleep)

def move_right(  desired_x, desired_z):
    print("moving right")
    x = get_x()
    z = get_z()
    print("moving right, from x=", x, " to x=", desired_x)
    #rotate
    rotate((1/2)*np.pi)
    #move
    while(abs(x - desired_x) > 0.2):
        #print("current x: ", x)
        #print("trying to go to: ", desired_x)
        #print("Going right.")
        set_x(desired_x)
        x = get_x()

    traversed[int(desired_x), int(desired_z)] = 1
    print(traversed)
    if np.all(traversed == 1):
        global done
        done = True
    print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

    update_map()
    time.sleep(sleep)

def move_down( desired_x, desired_z):
    x = get_x()
    z = get_z()
    print("moving backwards, from z=", z, " to z=", desired_z)
    #rotate

    #move
    while(abs(desired_z - z) > 0.2):
        #print("current z: ", z)
        #print("trying to go to: ", desired_z)
        set_z(desired_z)
        z = get_z()


    traversed[int(desired_x), int(desired_z)] = 1
    print(traversed)
    if np.all(traversed == 1):
        global done
        done = True
    print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

    update_map()
    time.sleep(sleep)

def set_yaw( change): #might need to multiply the change value
    #msg = Joy()
    #msg.axes = [0, 0, self.height, change] #double check these values
    #self.position_control.publish(msg)
    #time.sleep(1)
    return

def rotate( desired_yaw):
    return
    print("rotating to ", desired_yaw, "radians")
    error = (np.pi)/16
    yaw = self.StreamAttitude.yaw_y
    while (yaw < desired_yaw - error or yaw > desired_yaw + error):
        print("current yaw is ", yaw)
        print("setting yaw to ", desired_yaw)
        self.set_yaw(desired_yaw + error) #if our yaw is lower, we increase it. if its too high, we decrease it
        yaw = self.StreamAttitude.yaw_y
    self.set_yaw(0) #stop rotating
    return

def set_z( change):
    global z
    z = change
    return

def set_x( change):
    global x
    x = change
    return
def get_x():
    return x
def get_z():
    return z
def check_mission_accomplished():
    if done:
        print("************************")
        print("DONE. MISSION ACCOMPLISHED")
        print("************************")
    return done

def update_map():
    B = np.zeros(traversed.shape)
    G = (traversed > 0.2) * traversed
    R = (traversed < 0.2) * traversed
    #R += (grid < 0) * grid * -1
    BGR_grid = np.dstack([B, G, R])

    cv2.namedWindow("Grid", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Grid", 600, 600)
    cv2.imshow("Grid", 30*BGR_grid)

    cv2.waitKey(3)
    return

def main():
    explore()

if __name__ == '__main__':
    #main(sys.argv)
    main()
