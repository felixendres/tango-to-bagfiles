from __future__ import print_function

import rosbag
import sys


def main():
    if len(sys.argv) <2 :
        print("Usage: {} output_bag in_bag1 in_bag2 ...".format(sys.argv[0]))
        return

    in_bags = []
    for filename in sys.argv[2:]:
        print("Opening bag {}".format(filename))
        in_bags.append(rosbag.Bag(filename))

    outbag = rosbag.Bag(sys.argv[1], 'w')
    for in_bag in in_bags:
        print("Reading {}".format(in_bag))
        for topic, msg, t in in_bag.read_messages():
            outbag.write(topic, msg, t)

    outbag.close()
    print("Bag merging complete. bagfile: {}".format(sys.argv[1]))
    print("Probably {} needs to be sorted using rosrun rosbag bagsort".format(sys.argv[1]))

if __name__ == "__main__":
    main()
