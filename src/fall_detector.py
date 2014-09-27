#!/usr/bin/env python
import roslib
roslib.load_manifest('human_fall_detector')
import rospy
import std_msgs.msg
import human_fall_detector.msg
import math


Tv = 1.0
TvH = 0.12
TvWD = 0.2
TiH = 0.05
counter_a = counter_b = 0
counter_falling = 7
counter_inactivity = 7
activity_detection = False
inactivity_detection = False

last_state = None

def dist(x, y):
    return math.sqrt(x**2 + y**2)

def state_callback(state):
    global TvH, TvWD, TiH
    global counter_a, counter_b
    global counter_falling, counter_inactivity
    global activity_detection, inactivity_detection

    global vH_pub, vWD_pub

    global last_state
    if last_state is not None:
        dt = (state.header.stamp - last_state.header.stamp).to_sec()
        if dt == 0:
            return
        vH = (state.height - last_state.height) / dt
        vWD = (dist(state.width, state.depth)
               - dist(last_state.width, last_state.depth)) / dt
        if abs(vH) > Tv or abs(vWD) > Tv:
            return
        vH_pub.publish(vH)
        vWD_pub.publish(vWD)


        if (vH < -TvH) and (vWD > TvWD):
            if counter_a >= counter_falling:
                rospy.loginfo("Falling Detected")
                activity_detection = True
            else:
                counter_a = counter_a + 1
        elif counter_a > 0:
            counter_a = counter_a - 1
        if activity_detection:
            if abs(vH) < TiH:
                if counter_b >= counter_inactivity:
                    inactivity_detection = True
                    rospy.loginfo("Fall Detected")
                else:
                    counter_b = counter_b + 1
            elif counter_b > 0:
                counter_b = counter_b - 1
            else:
                activity_detection = False
        else:
            counter_b = 0
        
        rospy.loginfo("State: %r, %r - vH=%f, vWD=%f", activity_detection, inactivity_detection, vH, vWD)

    last_state = state


if __name__ == '__main__':
    rospy.init_node('human_fall_detector')
    reference_frame = rospy.get_param('~reference_frame', 'openni_depth_frame')
    rospy.Subscriber("human/state", human_fall_detector.msg.PersonState, state_callback)
    vH_pub = rospy.Publisher('human/vH', std_msgs.msg.Float64, queue_size=1)
    vWD_pub = rospy.Publisher('human/vWD', std_msgs.msg.Float64, queue_size=1)
    rospy.spin()
