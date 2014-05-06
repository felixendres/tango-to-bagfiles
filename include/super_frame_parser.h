#ifndef SUPER_FRAME_PARSER_H
#define SUPER_FRAME_PARSER_H

#ifndef NOT_COMPILING_WITH_MDK
#define NOT_COMPILING_WITH_MDK
#endif
// Tick Rate for Peanut, SF version 0x100
#define PEANUT_TICKS_PER_MICROSECOND 180.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <superframe_parser/superframe_v2.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>


/** Parses a super frame file to ros messages like sensor_msgs::Image. */
class SuperFrameParser
{
public:
    /** Opens the super frame file and already converts it to ros messages.
        Get the messages with getter functions.
        \param[in] super_frame_file string containing the path to the super frame file
        \param[in] cols number of columns of the super frame
        \param[in] rows number of rows of the super frame
        \param[in] bpp bytes per pixel of the super frame */
    SuperFrameParser (const std::string &super_frame_file,
                      const size_t rows = 1168, const size_t cols = 1280, const float bpp = 1.5);

    /** Closes the file descriptor and frees allocated memory. */
    ~SuperFrameParser ();

    /////////////         Getter Functions         /////////////////////////
    sf2_t* getSuperFrame () const { return super_frame_; }

    sensor_msgs::Image getSmallImage () const { return small_img_msgs_; }
    sensor_msgs::Imu getImu () const { return imu_msgs_; }
    //////////////////////////////////////////////////////////////////////

private:
    FILE *fd_;
    sf2_t *super_frame_;
    uint16_t *buffer_;

    ros::Time time_now_;

    const size_t cols_;
    const size_t rows_;
    const float bpp_;

    sensor_msgs::Image small_img_msgs_;
    sensor_msgs::Imu imu_msgs_;

    void fileToSF ();
    double convertTicksToSeconds (const uint32_t super_frame_version, const TimeStamp& raw_timestamp);

    void fillSmallImgMsg ();
    void fillImuMsg ();

};

#endif // SUPER_FRAME_PARSER_H
