import rospy
from sensor_msgs.msg import Joy
from dji_sdk.msg import SDKControlAuthority, DroneTaskControl
import math

#might need fixing ^^


def main():
    rospy.init_node('control_planner')
    pub = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)

    get_auth = rospy.ServiceProxy("dji_sdk/sdk_control_authority", SDKControlAuthority)
    control = rospy.ServiceProxy("/dji_sdk/drone_task_control", DroneTaskControl)

    set_auth(1)

    r = rospy.Rate(10)
    msg = Joy()
    while not rospy.is_shutdown():
        input = raw_input("Enter control command")
        #wasd = move forward/back/left/right
        #i/k = up/down
        #j/l =rotate left/right
        #t/g = takeoff/land
        if (input == "1"):
            msg.axes = [10, 0, 10, 0]
            print("w: move forward 10 in x, at height 10. only publish once"

        if (input == "2"):
            msg.axes = [2, 0, 10, 0]
            print("w: move forward by 2 in x, at height 10. publish 30 times")
            for i in range(30):
                pub.publish(msg)
                r.sleep()
        if (input == "3"):
            msg.axes = [10, 0, 10, 0]
            print("w: move forward 10 in x, at height 10. publish 30 times")
            for i in range(30):
                pub.publish(msg)
                r.sleep()

        if (input == "4"):
            msg.axes = [10, -10, 10, math.pi]
            print("w: move forward in x, backward in y, yaw of pi, at height 10. publish 30 times")
            for i in range(30):
                pub.publish(msg)
                r.sleep()

        if (input == "w"):
            msg.axes = [2, 0, 10, 0]
            print("w: move forward in x")
        elif (input == "s"):
            msg.axes = [-2, 0, 10, 0]
            print("s: move backward in x")
        elif (input == "a"):
            msg.axes = [0, -2, 10, 0]
            print("a: move back in y")
        elif (input == "d"):
            msg.axes = [0, 2, 10, 0]
            print("d: move forward in y")
        elif (input == "j"):
            msg.axes = [0, 0,10, -5]
            print("j: rotate with negative yaw")
        elif (input == "l"):
            msg.axes = [0, 0, 10, 5]
            print("l: rotate with positive yaw")
        elif (input == "i"):
            msg.axes = [0, 0, 20, 0]
            print("i: increase height. only works the first time")
        elif (input == "k"):
            msg.axes = [0, 0, 5, 0]
            print("k: decrease height")
        elif (input == "t"):
            print("takeoff")
            takeoff()
        elif (input == "g"):
            print("landing")
            land()
        else:
            msg.axes = [0, 0, 0, 0]
            print("incorrect input: sending stop command")

        pub.publish(msg)
        # Use our rate object to sleep until it is time to publish again
        r.sleep()


def set_auth(status):
    get_auth(status)

def takeoff():
    control(4)

def land():
    control(6)
    #format if it were a publisher:
    #task = DroneTaskControl()
    #task.task = 6
    #control(.publish(task))


if __name__ == '__main__':
    main()
