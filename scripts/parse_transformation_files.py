#!/usr/bin/python
from __future__ import print_function
import numpy as np
import argparse
import os

import rosbag
import rospy
from tf import transformations
from tf.msg import tfMessage
from geometry_msgs.msg import Transform, TransformStamped


def main():

    parser = argparse.ArgumentParser(description="Create the tf messages from "
                                                 "the Tango logs.")
    parser.add_argument('-d', '--directory',
                        help="The folder must have images.txt, "
                             "depth_to_imu_transformation.txt, "
                             "fisheye_to_imu_transformation.txt, "
                             "narrow_to_imu_transformation.txt",
                        required=True)
    parser.add_argument('-o', '--output',
                        help='output bag file (with location)',
                        metavar='bag_filename',
                        type=str,
                        required=True
                        )
    parser.add_argument('-y',
                        help='if images_adjusted.txt is found in the same folder'
                             'as the supplied filename, then use it without'
                             'asking',
                        metavar='True/False',
                        type=bool,
                        default=False,
                        choices=[True, False]
                        )

    arguments = parser.parse_args()
    folder_name = arguments.directory
    standard_file = os.path.join(folder_name, 'images.txt')
    alt_file = os.path.join(folder_name, 'images_adjusted.txt')
    if os.path.exists(alt_file):
        if arguments.y:
            images_file = open(alt_file)
        else:
            reply = None
            while reply not in ['y', 'n']:
                print("The images_adjusted.txt file is in the folder {}, do you"
                      " want to use that instead? [y/n]".format(folder_name))
                reply = raw_input()
            if reply == 'y':
                images_file = open(alt_file)
            else:
                images_file = open(standard_file)
    else:
        images_file = open(standard_file)
    print("Processing data from {}...".format(images_file.name))

    #depth to imu
    depth_np = np.loadtxt(os.path.join(folder_name,
                                       "depth_to_imu_transformation.txt",
                                       ),
                          delimiter=',').reshape(3, 4)
    rotation_matrix = np.vstack((depth_np, [0, 0, 0, 1]))
    quaternion = transformations.quaternion_from_matrix(rotation_matrix)
    depth_transform_msg = Transform()
    depth_transform_msg.translation.x = rotation_matrix[0, 3]
    depth_transform_msg.translation.y = rotation_matrix[1, 3]
    depth_transform_msg.translation.z = rotation_matrix[2, 3]
    depth_transform_msg.rotation.x = quaternion[0]
    depth_transform_msg.rotation.y = quaternion[1]
    depth_transform_msg.rotation.z = quaternion[2]
    depth_transform_msg.rotation.w = quaternion[3]

    #fisheye to imu
    depth_np = np.loadtxt(os.path.join(folder_name,
                                       "fisheye_to_imu_transformation.txt"),
                          delimiter=',').reshape(3, 4)
    rotation_matrix = np.vstack((depth_np, [0, 0, 0, 1]))
    quaternion = transformations.quaternion_from_matrix(rotation_matrix)
    fisheye_transform_msg = Transform()
    fisheye_transform_msg.translation.x = rotation_matrix[0, 3]
    fisheye_transform_msg.translation.y = rotation_matrix[1, 3]
    fisheye_transform_msg.translation.z = rotation_matrix[2, 3]
    fisheye_transform_msg.rotation.x = quaternion[0]
    fisheye_transform_msg.rotation.y = quaternion[1]
    fisheye_transform_msg.rotation.z = quaternion[2]
    fisheye_transform_msg.rotation.w = quaternion[3]

    #narrow to imu
    depth_np = np.loadtxt(os.path.join(folder_name,
                                       "narrow_to_imu_transformation.txt"),
                          delimiter=',').reshape(3, 4)
    rotation_matrix = np.vstack((depth_np, [0, 0, 0, 1]))
    quaternion = transformations.quaternion_from_matrix(rotation_matrix)
    narrow_transform_msg = Transform()
    narrow_transform_msg.translation.x = rotation_matrix[0, 3]
    narrow_transform_msg.translation.y = rotation_matrix[1, 3]
    narrow_transform_msg.translation.z = rotation_matrix[2, 3]
    narrow_transform_msg.rotation.x = quaternion[0]
    narrow_transform_msg.rotation.y = quaternion[1]
    narrow_transform_msg.rotation.z = quaternion[2]
    narrow_transform_msg.rotation.w = quaternion[3]

    #for each entry in images.txt, add a transformation with the values above
    #TODO skip some values? we don't need high frame rate!
    with rosbag.Bag(arguments.output, 'w') as outputbag:
        for lineno, line in enumerate(images_file):
            #lines must be of the form:
            #image_basename,t_android,t_movidius,r11,r12,r12,t1,r21,r22,r23,t2,r31,r32,r33,t3
            tokens = line.strip('\n').split(',')

            if (len(tokens)) != 15:
                print("Line {0} appears to be wrong: \"{1}\" ".format(lineno, tokens))
                continue

            msg = tfMessage()

            #using the android_ts as the timestamp
            ts = float(tokens[1])

            #depth
            tf_stamped = TransformStamped()
            tf_stamped.header.frame_id = "/tango/imu"
            tf_stamped.header.seq = lineno
            tf_stamped.header.stamp = rospy.Time.from_seconds(ts)
            tf_stamped.child_frame_id = "/tango/depth"
            tf_stamped.transform = depth_transform_msg
            msg.transforms.append(tf_stamped)

            #fisheye
            tf_stamped = TransformStamped()
            tf_stamped.header.frame_id = "/tango/imu"
            tf_stamped.header.seq = lineno
            tf_stamped.header.stamp = rospy.Time.from_seconds(ts)
            tf_stamped.child_frame_id = "/tango/fisheye"
            tf_stamped.transform = fisheye_transform_msg
            msg.transforms.append(tf_stamped)

            #narrow
            tf_stamped = TransformStamped()
            tf_stamped.header.frame_id = "/tango/imu"
            tf_stamped.header.seq = lineno
            tf_stamped.header.stamp = rospy.Time.from_seconds(ts)
            tf_stamped.child_frame_id = "/tango/narrow"
            tf_stamped.transform = narrow_transform_msg
            msg.transforms.append(tf_stamped)

            outputbag.write("tf", msg, rospy.Time.from_seconds(ts))

    outputbag.close()
    print("Bag creation complete, bagfile: {}".format(outputbag.filename))

if __name__ == '__main__':
    main()
