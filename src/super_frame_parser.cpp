#include "superframe_parser/super_frame_parser.h"
#include <sensor_msgs/image_encodings.h>

SuperFrameParser::SuperFrameParser (const std::string &name_space,
                                    const std::string &fisheye_name,
                                    const std::string &narrow_name,
                                    const std::string &pointcloud_name) :
    name_space_ (name_space),
    fisheye_name_ (fisheye_name),
    narrow_name_ (narrow_name),
    pointcloud_name_ (pointcloud_name)
{
    std::cout << name_space_ << " " << fisheye_name_ << " " << narrow_name_ << " " << pointcloud_name_ << " " << std::endl;
}

SuperFrameParser::~SuperFrameParser ()
{
    free (super_frame_);
}

void SuperFrameParser::parse (const std::string &super_frame_file,
                              const std::string &depth_intrinsics,
                              const std::string &fisheye_intrinsics,
                              const std::string &narrow_intrinsics)
{
    // create object
    small_img_msgs_.reset (new sensor_msgs::Image ());
    big_img_msgs_.reset(new sensor_msgs::Image ());
    point_cloud_msgs_.reset (new sensor_msgs::PointCloud2 ());
    imu_msgs_.reset (new sensor_msgs::Imu ());

    // allocate super_frame
    super_frame_ = static_cast<sf2_t*> (malloc (sizeof (sf2_t)));

    // parse all intrinsic parameters
    parseIntrinsicParams (depth_intrinsics, depth_intrinsics_);
    parseIntrinsicParams (fisheye_intrinsics, fisheye_intrinsics_);
    parseIntrinsicParams (narrow_intrinsics, narrow_intrinsics_);

    // init ros time for timestamps, just a hack right now
    ros::Time::init ();
    time_now_ = ros::Time::now ();

    // convert file to super frame format
    parseSfFile (super_frame_file);
    // fill in the data for the small image
    fillSmallImgMsg ();
    // fill in the data for the big image
    fillBigImgMsg ();
    // fill in the data for the depth image
    fillPointCloudMsg ();
    // fill in the data for the imu
    fillImuMsg ();
}

void SuperFrameParser::parseSfFile (const std::string &file)
{
    FILE *fd;
    if ((fd = fopen (file.c_str (), "rb")) == NULL)
        throw std::runtime_error ("Failed to open file");

    int bytes_read;
    // Parse the PGM file, skipping the # comment bits.  The # comment pad is
    // to maintain a 4kB block alignment for EXT4 writing.
    unsigned int img_width, img_height;
    char comment_str[8192] = {0};

    // get header dimensions
    if (fscanf (fd, "P5\n%d %d\n", &img_width, &img_height) != 2)
        throw std::runtime_error ("Failed to parse header dimensions\n");

    // get comments
    char *ret_str = fgets (comment_str, 4082, fd);
    if (ret_str == NULL)
        throw std::runtime_error ("Failed to parse comments");

    // get max value
    unsigned int max_val;
    if (fscanf(fd, "%d\n", &max_val) != 1)
        throw std::runtime_error ("Failed to parse max value\n");

    // Read in the data portion starting here;
    uint16_t *buffer =  static_cast<uint16_t*> (malloc (sizeof (sf2_t)));
    bytes_read = fread (buffer, 1, sizeof (sf2_t), fd);

    // connvert YUV420p to SF2
    memcpy (super_frame_, buffer, sizeof (sf2_t));
    fclose (fd);
}

void SuperFrameParser::parseIntrinsicParams (const std::string &intrinsic_params, CameraIntrinsics &intrinsics)
{
    std::ifstream f (intrinsic_params.c_str ());
    if (!f.is_open ())
        throw std::runtime_error ("Could not open intrinsic parameters file!");

    std::string line;
    getline (f, line);

    size_t pos = 0;
    std::string token;
    std::vector<std::string> params;
    std::string delimiter = ",";

    while ((pos = line.find (delimiter)) != std::string::npos)
    {
        token = line.substr (0, pos);
        params.push_back (token);
        line.erase (0, pos + 1);
    }
    params.push_back (line);

    intrinsics.width = atoi (params[0].c_str ());
    intrinsics.height = atoi (params[1].c_str ());
    intrinsics.focal_length[0] = atof (params[2].c_str ());
    intrinsics.focal_length[1] = atof (params[3].c_str ());
    intrinsics.principal_point[0] = atof (params[4].c_str ());
    intrinsics.principal_point[1] = atof (params[5].c_str ());
    intrinsics.omega = atof (params[6].c_str ());
    intrinsics.max_angle = atof (params[7].c_str ());
}

double SuperFrameParser::convertTicksToSeconds (const uint32_t super_frame_version, const TimeStamp& raw_timestamp)
{
    if (super_frame_version & 0x100)
    {
        return (((static_cast<uint64_t> (raw_timestamp.superframe_v2.ticks_hi) << 32) |
                 (static_cast<uint64_t> (raw_timestamp.superframe_v2.ticks_lo))) / (1000000. * PEANUT_TICKS_PER_MICROSECOND));
    }
    else
    {
        return static_cast<double> (raw_timestamp.superframe_v1) / (static_cast<double> (PEANUT_TICKS_PER_MICROSECOND) * 1000. * 1000.);
    }
}

void SuperFrameParser::fillSmallImgMsg ()
{
    small_img_msgs_->header.frame_id = name_space_ + "/" + fisheye_name_;

    small_img_msgs_->header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                      super_frame_->header.frame.small.timestamp));
    small_img_msgs_->height = SMALL_IMG_HEIGHT;
    small_img_msgs_->width = SMALL_IMG_WIDTH;
    small_img_msgs_->step = small_img_msgs_->width;
    small_img_msgs_->encoding = sensor_msgs::image_encodings::MONO8;
    small_img_msgs_->data.resize (small_img_msgs_->width * small_img_msgs_->height);
    memcpy (&small_img_msgs_->data[0], super_frame_->small_img, small_img_msgs_->data.size ());
}

void SuperFrameParser::fillBigImgMsg ()
{
    big_img_msgs_->header.frame_id = name_space_ + "/" + narrow_name_;
    big_img_msgs_->header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                    super_frame_->header.frame.big.timestamp));
    big_img_msgs_->height = BIG_RGB_HEIGHT;
    big_img_msgs_->width = BIG_RGB_WIDTH;
    big_img_msgs_->step = big_img_msgs_->width/* * 2*/;
    big_img_msgs_->encoding = sensor_msgs::image_encodings::MONO8;
    big_img_msgs_->data.resize (big_img_msgs_->width * big_img_msgs_->height/* * 2*/);
    memcpy (&big_img_msgs_->data[0], super_frame_->big_rgb, big_img_msgs_->data.size ());
}

void SuperFrameParser::fillPointCloudMsg ()
{
    sensor_msgs::ImagePtr depth_image (new sensor_msgs::Image ());
    depth_image->height = DEPTH_IMG_HEIGHT;
    depth_image->width = DEPTH_IMG_WIDTH;
    depth_image->step = 2 * depth_image->width;
    depth_image->data.resize (depth_image->width * depth_image->height * 2);
    memcpy (&depth_image->data[0], super_frame_->depth_img, depth_image->data.size ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    convertImageToPointCloud (depth_image, point_cloud);

    pcl::toROSMsg (*point_cloud, *point_cloud_msgs_);
    point_cloud_msgs_->header.frame_id = name_space_ + "/" + pointcloud_name_;
    point_cloud_msgs_->header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                        super_frame_->header.frame.depth.timestamp));
    point_cloud_msgs_->row_step = point_cloud_msgs_->width * 2;

}


void SuperFrameParser::fillImuMsg ()
{

}

void SuperFrameParser::convertImageToPointCloud (const sensor_msgs::ImagePtr& depth_msg, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    cloud->height = depth_msg->height;
    cloud->width = depth_msg->width;
    cloud->resize (cloud->height * cloud->width);
    // Use correct principal point from calibration
    float center_x = depth_intrinsics_.principal_point[0]; // c_x
    float center_y = depth_intrinsics_.principal_point[1]; // c_y

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 0.001f;
    float constant_x = unit_scaling / depth_intrinsics_.focal_length[0]; // f_x
    float constant_y = unit_scaling / depth_intrinsics_.focal_length[1]; // f_y
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin ();
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*> (&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof (uint16_t);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)depth_msg->width; ++u)
        {
            pcl::PointXYZ& pt = *pt_iter++;
            uint16_t depth = depth_row[u];

            // Missing points denoted by NaNs
            if (depth == 0.0f)
            {
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            // Fill in XYZ
            pt.x = (u - center_x) * depth * constant_x;
            pt.y = (v - center_y) * depth * constant_y;
            pt.z = depth * 0.001f;
        }
    }
}

