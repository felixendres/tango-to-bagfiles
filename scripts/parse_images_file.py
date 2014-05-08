#!/usr/bin/python

from __future__ import print_function
import numpy as np
import sys

import rosbag
import rospy
from tf import transformations
from tf.msg import tfMessage
from geometry_msgs.msg import Transform, TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def main():
    if len(sys.argv) != 3:
        print("Usage: {} /location/to/images.txt outputbag.bag".format(sys.argv[0]))
        return

    images_file = open(sys.argv[1])

    with rosbag.Bag(sys.argv[2], 'w') as outputbag:
        for lineno, line in enumerate(images_file):
            #lines must be of the form:
            #image_basename,t_android,t_movidius,r11,r12,r12,t1,r21,r22,r23,t2,r31,r32,r33,t3
            tokens = line.strip('\n').split(',')

            if (len(tokens)) != 15:
                print("Line {0} appears to be wrong: \"{1}\" ".format(lineno, tokens))
                continue

            #first elements before transformation
            image_name = tokens[0]
            ts1 = float(tokens[1])
            ts2 = float(tokens[2])

            #getting the quaterion out the rotation matrix
            rotation_matrix = np.array(map(float, tokens[3:])).reshape(3, 4)
            rotation_matrix = np.vstack((rotation_matrix, [0, 0, 0, 1]))
            quaternion = transformations.quaternion_from_matrix(rotation_matrix)

            #creating the pose
            p = PoseStamped()
            p.header.frame_id = "/tango/world"
            p.header.stamp = rospy.Time.from_seconds(ts1)
            p.header.seq = lineno
            p.pose.position.x = rotation_matrix[0, 3]
            p.pose.position.y = rotation_matrix[1, 3]
            p.pose.position.z = rotation_matrix[2, 3]
            p.pose.orientation.x = quaternion[0]
            p.pose.orientation.y = quaternion[1]
            p.pose.orientation.z = quaternion[2]
            p.pose.orientation.w = quaternion[3]
            outputbag.write("tango_imu_pose", p, rospy.Time.from_seconds(ts1))

            #creating the odometry entry
            #TODO get covariances from the covariances.txt file
            o = Odometry()
            o.header = p.header
            o.child_frame_id = o.header.frame_id
            o.pose.pose = p.pose
            outputbag.write("tango_imu_odom", o, rospy.Time.from_seconds(ts1))

            #creating the tf messages
            tfmsg = tfMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.frame_id = "/tango/world"
            tf_stamped.header.seq = lineno
            tf_stamped.header.stamp = rospy.Time.from_seconds(ts1)
            tf_stamped.child_frame_id = "/tango/imu"
            tf_stamped.transform.translation.x = rotation_matrix[0, 3]
            tf_stamped.transform.translation.y = rotation_matrix[1, 3]
            tf_stamped.transform.translation.z = rotation_matrix[2, 3]
            tf_stamped.transform.rotation = p.pose.orientation
            tfmsg.transforms.append(tf_stamped)
            outputbag.write("tf", tfmsg, rospy.Time.from_seconds(ts1))

    print("Bag creation complete, bagfile: {}".format(sys.argv[2]))

if __name__ == '__main__':
    main()