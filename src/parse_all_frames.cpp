// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.

#include <superframe_parser/super_frame_parser.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;
int main (int argc, char **argv)
{
    if (argc < 4)
    {
        std::cout << "Too few arguments.\n";
        std::cout << "Usage: " << argv[0] << " super_frames_dir bag_output\n";
        exit (0);
    }

    fs::path fs_path (argv[1]);
    if (!fs::is_directory (fs_path) || !fs::exists (fs_path))
    {
        std::cout << "Provided first argument (super_frames_dir) is not a valid dir or does not exist!\n";
        return -1;
    }

    po::options_description desc ("Allowed options");
    std::string name_space;
    std::string fisheye_name;
    std::string narrow_name;
    std::string pointcloud_name;
    desc.add_options()
            ("namespace", po::value<std::string> (&name_space)->default_value ("namespace"), "namespace for topics and frame ids")
            ("fisheye", po::value<std::string> (&fisheye_name)->default_value ("fisheye"), "name for fisheye topic and frame id")
            ("narrow", po::value<std::string> (&narrow_name)->default_value ("narrow"), "name for narrow topic and frame id")
            ("pointcloud", po::value<std::string> (&pointcloud_name)->default_value ("pointcloud"), "name for pointcloud topic and frame id")
            ;

    // Parse the command line catching and displaying any
    // parser errors
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line (argc, argv, desc), vm);
        po::notify (vm);
    }
    catch (std::exception &e)
    {
        std::cout << std::endl << e.what() << std::endl;
        std::cout << desc << std::endl;
    }

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
        bool correct_file = true;
        if (!fs::is_regular_file (it->path ()))
            continue;

        std::string file_path = it->path ().string ();
        std::cout << "parsing " << file_path << std::endl;
        SuperFrameParser sf_parser (name_space, fisheye_name, narrow_name, pointcloud_name);
        try
        {
            sf_parser.parse (it->path ().string ());
        }
        catch (std::exception &e)
        {
            correct_file = false;
        }

        if (correct_file)
        {
            bag.write (name_space + "/" + fisheye_name, sf_parser.getSmallImage ()->header.stamp, *sf_parser.getSmallImage ());
            bag.write (name_space + "/" + narrow_name, sf_parser.getBigImage ()->header.stamp, *sf_parser.getBigImage ());
            bag.write (name_space + "/" + pointcloud_name, sf_parser.getPointCloud ()->header.stamp, *sf_parser.getPointCloud ());


            // save big img to disk
//            std::stringstream ss_big;
//            ss_big << "big_img_" << it->path ().filename ().string() ;
//            if ((fp3 = fopen (ss_big.str ().c_str (), "wb")) != NULL)
//            {
//                fprintf (fp3, big_img_header.str ().c_str ());
//                fwrite (&sf_parser.getBigImage()->data[0], 1, BIG_RGB_WIDTH * BIG_RGB_HEIGHT, fp3);
//                fclose (fp3);
//            }
        }

        it++;
    }

    bag.close();
    return 0;
}
