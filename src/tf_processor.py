#!/usr/bin/env python
import roslib
roslib.load_manifest('human_fall_detector')
import rospy
import std_msgs
import tf
import human_fall_detector.msg
import visualization_msgs.msg
import geometry_msgs.msg
import collections
import math

if __name__ == '__main__':
    rospy.init_node('human_tf_processor')
    reference_frame = rospy.get_param('~reference_frame', 'openni_depth_frame')
    body_index = 1
    max_body_count = 15

    listener = tf.TransformListener()

    state_publisher = rospy.Publisher('human/state', human_fall_detector.msg.PersonState, queue_size=1)
    visualization_publisher = rospy.Publisher('human/visualization', visualization_msgs.msg.Marker, queue_size=1)
    joint_names = ["head",
              "neck",
              "torso",
              "left_shoulder",
              #"left_elbow",
              #"left_hand",
              "left_hip",
              #"left_knee",
              #"left_foot",
              "right_shoulder",
              #"right_elbow",
              #"right_hand",
              "right_hip",
              #"right_knee",
              #"right_foot"
    ]

    transform_timeout = rospy.Duration(0.2)
    rate = rospy.Rate(15.0)
    while not rospy.is_shutdown():
        rate.sleep()
        x_max = y_max = z_max = float("-infinity")
        x_min = y_min = z_min = float("infinity")
        try:
            now = rospy.Time.now()
            for joint_name in joint_names:
                listener.waitForTransform(reference_frame, joint_name+"_"+str(body_index), now, transform_timeout)
                (trans, rot) = listener.lookupTransform(reference_frame, joint_name+"_"+str(body_index), now)
                x_max = max(x_max, trans[0])
                y_max = max(y_max, trans[1])
                z_max = max(z_max, trans[2])
                x_min = min(x_min, trans[0])
                y_min = min(y_min, trans[1])
                z_min = min(z_min, trans[2])

                state = human_fall_detector.msg.PersonState()
                state.header.stamp = now
                state.header.frame_id = reference_frame
                state.width = abs(x_max - x_min)
                state.depth = abs(y_max - y_min)
                state.height = abs(z_max - z_min)
                state_publisher.publish(state)

                marker = visualization_msgs.msg.Marker()
                marker.header.frame_id = reference_frame
                marker.header.stamp = now
                marker.ns = "human_detector"
                marker.id = 0
                marker.type = visualization_msgs.msg.Marker.LINE_LIST
                marker.action = visualization_msgs.msg.Marker.ADD
                def add_line(x1, y1, z1, x2, y2, z2):
                    position1 = geometry_msgs.msg.Point()
                    position1.x = x1
                    position1.y = y1
                    position1.z = z1
                    position2 = geometry_msgs.msg.Point()
                    position2.x = x2
                    position2.y = y2
                    position2.z = z2
                    marker.points.append(position1)
                    marker.points.append(position2)
                add_line(x_max, y_max, z_max, x_min, y_max, z_max)
                add_line(x_max, y_max, z_max, x_max, y_min, z_max)
                add_line(x_min, y_min, z_max, x_min, y_max, z_max)
                add_line(x_min, y_min, z_max, x_max, y_min, z_max)

                add_line(x_max, y_max, z_min, x_min, y_max, z_min)
                add_line(x_max, y_max, z_min, x_max, y_min, z_min)
                add_line(x_min, y_min, z_min, x_min, y_max, z_min)
                add_line(x_min, y_min, z_min, x_max, y_min, z_min)

                add_line(x_max, y_max, z_max, x_max, y_max, z_min)
                add_line(x_min, y_max, z_max, x_min, y_max, z_min)
                add_line(x_max, y_min, z_max, x_max, y_min, z_min)
                add_line(x_min, y_min, z_max, x_min, y_min, z_min)

                marker.scale.x = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                visualization_publisher.publish(marker)


        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            body_index = ((body_index - 1 + 1) % max_body_count) + 1
            last_time = None
            print "using body index: "+str(body_index)
            continue

