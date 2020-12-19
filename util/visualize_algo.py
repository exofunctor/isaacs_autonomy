import numpy as np
import cv2
import time
from grid import Grid

x = 0
z = 0
yaw = 0
done = False
sleep = 0.5
grid = np.array([
        [0,  0,  0,  0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -2, -2, -2,  0,-2, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0, 0,-2, 0],
        [0,  0, -2,  0,  0,-2, 0, 0, 0, -2,-2,-2, 0, 0,-2, 0, 0, 0, 0,-2, 0, 0,-2, 0],
        [0,  0, -2,  0,  0,-2, 0, 0, 0,  0, 0,-2, 0, 0,-2,-2,-2,-2, 0,-2, 0,-2, 0, 0],
        [0,  0, -2,  0,  0,-2,-2,-2, 0, -2,-2,-2, 0, 0,-2, 0, 0,-2, 0,-2,-2, 0, 0, 0],
        [0,  0, -2,  0,  0,-2, 0,-2, 0, -2, 0,-2, 0, 0,-2, 0, 0,-2, 0,-2, 0,-2, 0, 0],
        [0,  0, -2,  0,  0,-2, 0,-2, 0, -2,-2, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0,-2, 0],
        [0,  0,  0,  0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0,  0,  0,  0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0,  0,  0,  0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0,  0,  0, -2,  0, 0, 0,-2, 0,  0,-2,-2, 0, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0, -2,  0, 0, 0,-2, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0, -2,  0, 0, 0,-2, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0, -2,  0, 0, 0,-2, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0, -2,  0, 0, 0,-2, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0, -2, 0,-2, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0, -2, 0,-2, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0, -2, 0, 0,-2, 0,-2, 0, 0,-2, 0, 0, 0, 0, 0, 0],
        [0,  0,  0,  0,  0,-2, 0, 0, 0,  0,-2,-2, 0, 0, 0,-2,-2, 0, 0,-2, 0, 0, 0, 0],
        [0,  0,  0,  0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

radius = 12
max_x = 2*radius
max_z = 2*radius
# Grid = Grid(radius)



# random_obstacles = -np.floor(1.3 * np.random.rand(2*radius, 2*radius))
# random_obstacles = random_obstacles.astype(Grid.grid.dtype)
# Grid.grid = Grid.grid + random_obstacles


original_grid = grid.copy()
traversed = np.zeros((2*radius, 2*radius))
traversed[x, z] = 1
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
    yaw = get_yaw()

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
    rotate(np.pi)

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


def rotate(desired_yaw):
    set_yaw(desired_yaw)
    return


def set_z(change):
    global z
    z = change
    return


def set_x(change):
    global x
    x = change
    return


def set_yaw(change):
    global yaw
    yaw = change
    return yaw


def get_x():
    return x


def get_z():
    return z


def get_yaw():
    return yaw


def check_mission_accomplished():
    if done:
        print("************************")
        print("DONE. MISSION ACCOMPLISHED.")
        print("************************")
    return done


def update_map():

    x = get_x()
    z = get_z()
    # yaw = get_yaw()

    # f = 6
    # if yaw == 0:
        # for i in f

    # depth_map = 5*np.ones(25)
    # box, z1, z2, x1, x2 = Grid.update(depth_map, np.pi/3, yaw, x, z)

    # boxx = np.zeros(traversed.shape)
    # boxx[z1:z2, x1:x2] = 1

    traversed_tiles = (traversed > 0.2) * traversed * 255
    tB = traversed_tiles.copy()
    tG = np.ones(original_grid.shape) + original_grid - traversed_tiles
    tR = -original_grid

    tB[x, z] = 1
    tG[x, z] = 1
    tR[x, z] = 1

    BGR_traversed = np.dstack([tB, tG, tR])

    out = BGR_traversed

    # if yaw == 0:
        # depth_map = np.min(Grid.grid[], axis=0)


    # B = np.zeros(Grid.grid.shape)
    # G = (Grid.grid > 0) * Grid.grid
    # R = (Grid.grid < 0) * Grid.grid * -1
    # BGR_grid = np.dstack([B, G, R]) * gK
    # out = np.hstack([BGR_traversed, BGR_grid])

    cv2.namedWindow("Planning", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Planning", 600, 600)
    cv2.imshow("Planning", out)

    cv2.waitKey(3)
    return

# def main():
    # explore()

# if __name__ == '__main__':
    # #main(sys.argv)
    # main()
##

