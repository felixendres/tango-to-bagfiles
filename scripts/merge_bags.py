#!/usr/bin/python

from __future__ import print_function

import rosbag
import argparse


def main():
    parser = argparse.ArgumentParser(description="Merge several bags into one. "
                                                 "Does not perform sorting of "
                                                 "messages.")

    parser.add_argument('input_bags',
                        help='list of bags to merge',
                        nargs='+')
    parser.add_argument('--output', '-o',
                        help='output bag',
                        type=str)
    arguments = parser.parse_args()

    in_bags = []
    for filename in arguments.input_bags:
        print("Opening bag {}".format(filename))
        in_bags.append(rosbag.Bag(filename))

    outbag = rosbag.Bag(arguments.output, 'w')
    for in_bag in in_bags:
        print("Reading {}".format(in_bag))
        for topic, msg, t in in_bag.read_messages():
            outbag.write(topic, msg, t)

    outbag.close()
    print("Bag merging complete. bagfile: {}".format(outbag.filename))

if __name__ == "__main__":
    main()
