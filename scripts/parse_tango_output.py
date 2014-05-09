#!/usr/bin/python

from __future__ import print_function
import sys
from subprocess import call
import os


def main():
    if len(sys.argv) != 3:
        print("Usage: {} /location/to/files outputbag.bag".format(sys.argv[0]))
        print("The folder must be of the kind produced by Tango mapper. "
              "The superframes subfolder should be there too."
              )
        return 1

    folder_name = sys.argv[1]
    output_bag =sys.argv[2]

    #parse image file
    print("Parsing the images.txt file")
    call(['rosrun',
          'superframe_parser',
          'parse_images_file.py',
          os.path.join(folder_name, 'images.txt'),
          '/tmp/tango_poses.bag'])

    if not os.path.exists('/tmp/tango_poses.bag'):
        print("The bag /tmp/tango_poses.bag doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    #parse image file
    print("Parsing the transformation files")
    call(['rosrun',
          'superframe_parser',
          'parse_transformation_files.py',
          folder_name,
          '/tmp/tango_tfs.bag'])

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
          '/tmp/tango_superframes.bag'])

    if not os.path.exists('/tmp/tango_superframes.bag'):
        print("The bag /tmp/tango_superframes.bag doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    #mergin the bags
    print("Mergin the bags")
    call(['rosrun',
          'superframe_parser',
          'merge_bags.py',
          output_bag,
          '/tmp/tango_poses.bag',
          '/tmp/tango_tfs.bag',
          '/tmp/tango_superframes.bag'])

    if not os.path.exists(output_bag):
        print("The bag " + output_bag + " doesn't seem to exist, "
              "there probably was a problem in the previous run")
        return 1

    print("Alles zolt gut!")


if __name__ == '__main__':
    main()

