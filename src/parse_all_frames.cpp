// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.

#include <superframe_parser/superframe_v2.h>
#include <super_frame_parser.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
namespace fs = boost::filesystem;

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
    FILE *fp3;
    std::stringstream small_img_header;
    small_img_header<< "P5\n" << SMALL_IMG_WIDTH << " " << SMALL_IMG_HEIGHT << "\n255\n";
    std::stringstream big_img_header;
    big_img_header << "P5\n" << BIG_RGB_WIDTH << " " << BIG_RGB_HEIGHT << "\n255\n";
    std::stringstream depth_img_header;
    depth_img_header << "P5\n" << DEPTH_IMG_WIDTH << " " << DEPTH_IMG_HEIGHT << "\n65535\n";
    while (it != eod)
    {
        if (!fs::is_regular_file (it->path ()))
            continue;

        std::string file_path = it->path ().string ();
        std::cout << "parsing " << file_path << std::endl;
        SuperFrameParser super_frame_parser (it->path ().string ());

        bag.write ("tango/small_image", super_frame_parser.getSmallImage ()->header.stamp, *super_frame_parser.getSmallImage ());
        bag.write ("tango/big_image", super_frame_parser.getBigImage ()->header.stamp, *super_frame_parser.getBigImage ());
        bag.write ("tango/pointcloud", super_frame_parser.getPointCloud ()->header.stamp, *super_frame_parser.getPointCloud ());


        // save big img to disk
//        std::stringstream ss_big;
//        ss_big << "big_img_" << it->path ().filename ().string() ;
//        if ((fp3 = fopen (ss_big.str ().c_str (), "wb")) != NULL)
//        {
//            fprintf (fp3, big_img_header.str ().c_str ());
//            fwrite (&super_frame_parser.getBigImage()->data[0], 1, BIG_RGB_WIDTH * BIG_RGB_HEIGHT, fp3);
//            fclose (fp3);
//        }

        it++;
    }

    bag.close();
    return 0;
}
