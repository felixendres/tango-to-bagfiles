// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.

#include <superframe_parser/superframe_v2.h>
#include <super_frame_parser.h>

namespace fs = boost::filesystem3;

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        printf ("Too few arguments.\n");
        printf ("Usage: %s super_frames_dir bag_output\n", argv[0]);
        exit (0);
    }

    fs::path fs_path (argv[1]);
    if (!fs::is_directory (fs_path) || !fs::exists (fs_path))
        return -1;

    fs::directory_iterator it (argv[1]), eod;
    rosbag::Bag bag (argv[2], rosbag::bagmode::Write);
    FILE *fp;
    while (it != eod)
    {
        if (fs::is_regular_file (it->path ()))
        {
            std::cout << "parsing " << it->path ().string () << std::endl;
            SuperFrameParser super_frame_parser (it->path ().string ());

            bag.write ("tango/small_image", super_frame_parser.getSmallImage ().header.stamp, super_frame_parser.getSmallImage ());
            // save it to disk
            std::stringstream ss_small;
            ss_small << "small_img_" << it->path ().filename ().string() ;
            if ((fp = fopen (ss_small.str ().c_str (), "wb")) != NULL)
            {
                fprintf (fp, "P5\n640 480\n255\n");
                fwrite (&super_frame_parser.getSmallImage().data[0], SMALL_IMG_WIDTH, SMALL_IMG_HEIGHT, fp);
                fclose (fp);
            }
        }
        it++;
    }

    bag.close();
    return 0;
}
