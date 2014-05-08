#ifndef SUPER_FRAME_PARSER_H
#define SUPER_FRAME_PARSER_H

#ifndef NOT_COMPILING_WITH_MDK
#define NOT_COMPILING_WITH_MDK
#endif
// Tick Rate for Peanut, SF version 0x100
#define PEANUT_TICKS_PER_MICROSECOND 180.
#define DEPTH_TIMESTAMP_OFFSET -0.175

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <superframe_parser/superframe_v2.h>

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

/** Parses a given super frame file with intrinsic parameters for all cameras to ros messages.

    Currently extracted ros messages:
    sensor_msgs::Image for the small fisheye camera
    sensor_msgs::Image for the big narrow camera
    sensor_msgs::PointCloud2 for the depth camera (converted to a pointcloud)

    sensor_msgs::CameraInfo for the fisheye camera info
    sensor_msgs::CameraInfo for the narrow camera info
    sensor_msgs::CameraInfo for the depth camera info */
class SuperFrameParser
{
public:
    /** Default constructor.
        \param[in] name_space namespace for the topics and frame ids
        \param[in] fisheye_name name for the fisheye topic and frame id
        \param[in] narrow_name name for the narrow topic and frame id
        \param[in] pointcloud_name name for the pointcloud topic and frame id
        \param[in] timestamp_file name path to the file with the timestamps */
    SuperFrameParser (const std::string &name_space = "tango",
                      const std::string &fisheye_name = "fisheye",
                      const std::string &narrow_name = "narrow",
                      const std::string &pointcloud_name = "depth",
                      const std::string &timestamp_file = "images.txt");

    /** Frees allocated memory. */
    ~SuperFrameParser ();

    /** Opens the super frame file and parses it to ros messages.
        Get the messages with getter functions.
        \param[in] super_frame_file string containing the path to the super frame file
        \param[in] depth_intrinsics name of the file for the depth instrinsic parameters
        \param[in] fisheye_intrinsics name of the file for the fisheye instrinsic parameters
        \param[in] narrow_intrinsics name of the file for the narrow instrinsic parameters */
    void parse (const std::string &super_frame_file,
                const std::string &depth_intrinsics = "depth_intrinsics.txt",
                const std::string &fisheye_intrinsics = "fisheye_intrinsics.txt",
                const std::string &narrow_intrinsics = "narrow_intrinsics.txt");

    /** converts ticks got from the superframe header to seconds */
    double convertTicksToSeconds (const uint32_t super_frame_version, const TimeStamp& raw_timestamp);

    /** Returs a pointer to the internal super frame structure */
    inline sf2_t* getSuperFrame () const { return super_frame_; }

    /** Returns a pointer to the small image message */
    inline sensor_msgs::ImagePtr getFisheyeImage () const { return fisheye_msgs_; }

    /** Returns a pointer to the big image message */
    inline sensor_msgs::ImagePtr getNarrowImage () const { return narrow_msgs_; }

    /** Returns a pointer to the pointcoud message */
    inline sensor_msgs::PointCloud2Ptr getPointCloud () const { return point_cloud_msgs_; }

    /** Returns a pointer to the Imu message */
    inline sensor_msgs::ImuPtr getImu () const { return imu_msgs_; }

    /** Returns a pointer to the fish eye camera info message */
    inline sensor_msgs::CameraInfoPtr getFisheyeCameraInfo () const { return fisheye_info_; }

    /** Returns a pointer to the narrow camera info message */
    inline sensor_msgs::CameraInfoPtr getNarrowCameraInfo () const { return narrow_info_; }

    /** Returns a pointer to the depth camera info message */
    inline sensor_msgs::CameraInfoPtr getDepthCameraInfo () const { return depth_info_; }
private:
    sf2_t *super_frame_;

    //////////  TOPIC AND FRAME ID NAMES ////////
    std::string name_space_;
    std::string fisheye_name_;
    std::string narrow_name_;
    std::string pointcloud_name_;

    /////////         MESSAGES         //////////
    sensor_msgs::ImagePtr fisheye_msgs_;
    sensor_msgs::ImagePtr narrow_msgs_;
    sensor_msgs::PointCloud2Ptr point_cloud_msgs_;
    sensor_msgs::ImuPtr imu_msgs_;

    ////////     CAMERA INFOS    //////////
    sensor_msgs::CameraInfoPtr depth_info_;
    sensor_msgs::CameraInfoPtr fisheye_info_;
    sensor_msgs::CameraInfoPtr narrow_info_;

    std::map<std::string, double> timestamp_map_;
    std::string file_name_;

    /** parses the data from the superframe file to internal superframe structure */
    void parseSfFile (const std::string &file);

    /** parses the intrinsic parameters from the file to internal structure */
    void parseCameraInfo (const std::string &params_file, std::vector<std::string> &params);

    /** converts a sensor_msgs::Image to a pcl::PointCloud */
    void convertImageToPointCloud (const sensor_msgs::ImagePtr &depth_msg, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    ///////   FILL FUNCTIONS FOR THE MESSAGES   ////////
    void fillFisheyeMsg (const std::string &params_file);
    void fillNarrowMsg (const std::string &params_file);
    void fillPointCloudMsg (const std::string &params_file);
    void fillImuMsg (const std::string &params_file);

    void buildTimestampMap (const std::string &timestamp_file);

};

#endif // SUPER_FRAME_PARSER_H
