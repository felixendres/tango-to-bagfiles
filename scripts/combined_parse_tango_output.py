#!/usr/bin/python

from __future__ import print_function
import argparse
from subprocess import call
import os


def main():

    parser = argparse.ArgumentParser(description="Runs a series of scripts and "
                                                 "programs to parse the Tango "
                                                 "output. The /tmp folder will "
                                                 "be used for temporary bags.")
    parser.add_argument('-d', '--directory',
                        help="The folder must be of the kind produced by Tango "
                             "mapper. The superframes subfolder should be there "
                             "too. The folder must have images.txt, "
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


    arguments = parser.parse_args()
    folder_name = arguments.directory
    output_bag = arguments.output

    #parse image file
    print("Parsing the images.txt file")
    call(['rosrun',
          'superframe_parser',
          'parse_images_file.py',
          '-i', os.path.join(folder_name, 'images.txt'),
          '-o', '/tmp/tango_poses.bag',
          '-y', 'True'
         ])

    if not os.path.exists('/tmp/tango_poses.bag'):
        print("The bag /tmp/tango_poses.bag doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    #parse transformation file
    print("Parsing the transformation files")
    call(['rosrun',
          'superframe_parser',
          'parse_transformation_files.py',
          '-d', folder_name,
          '-o', '/tmp/tango_tfs.bag',
          '-y', 'True'])

    if not os.path.exists('/tmp/tango_tfs.bag'):
        print("The bag /tmp/tango_tfs.bag doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    #parse superframes
    print("Parsing the superframes")
    call(['rosrun',
          'superframe_parser',
          'parse_all_frames',
          folder_name,
          '/tmp/tango_superframes.bag',
          '-y'])

    if not os.path.exists('/tmp/tango_superframes.bag'):
        print("The bag /tmp/tango_superframes.bag doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    #mergin the bags
    print("Mergin the bags")
    call(['rosrun',
          'superframe_parser',
          'merge_bags.py',
          '/tmp/tango_poses.bag',
          '/tmp/tango_tfs.bag',
          '/tmp/tango_superframes.bag',
          '-o', output_bag])

    if not os.path.exists(output_bag):
        print("The bag " + output_bag + " doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    print("Alles zolt gut!")


if __name__ == '__main__':
    main()

