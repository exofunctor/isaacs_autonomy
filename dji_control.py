import rospy
from sensor_msgs.msg import Joy
#might need fixing ^^

target_x = 0
target_y = 0
target_z = 0


def main():
    rospy.init_node('control_planner')
    pub = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)
    r = rospy.Rate(10)
    msg = Joy()
    while not rospy.is_shutdown():
        input = raw_input("Enter control command")
        #wasd are standard forward/back/left/right
        #i/k is up/down
        #j/l is to rotate left/right
        if (input == "w"):
            msg.axes = [2, 0, 0, 0]
            print("w: move forward in x")
        elif (input == "s"):
            msg.axes = [-2, 0, 0, 0]
            print("s: move backward in x")
        elif (input == "a"):
            msg.axes = [0, -2, 0, 0]
            print("a: move back in y")
        elif (input == "d"):
            msg.axes = [0, 2, 0, 0]
            print("d: move forward in y")
        elif (input == "j"):
            msg.axes = [0, 0, 0, -5]
            print("j: rotate with negative yaw")
        elif (input == "l"):
            msg.axes = [0, 0, 0, 5]
            print("j: rotate with positive yaw")
        elif (input == "i"):
            msg.axes = [0, 0, 2, 0]
            print("i: increase height")
        elif (input == "k"):
            msg.axes = [0, 0, -2, 0]
            print("k: decrease height")
        else:
            msg.axes = [0, 0, 0, 0]
            print("incorrect input: sending stop command")
            
        pub.publish(msg)
        # Use our rate object to sleep until it is time to publish again
        r.sleep()

if __name__ == '__main__':
    main()
