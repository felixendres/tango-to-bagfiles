#include "super_frame_parser.h"

SuperFrameParser::SuperFrameParser (const std::string &super_frame_file)
{
    if ((fd_ = fopen (super_frame_file.c_str (), "rb")) == NULL)
        throw std::runtime_error ("Failed to open file");

    // create object
    small_img_msgs_.reset (new sensor_msgs::Image ());
    big_img_msgs_.reset(new sensor_msgs::Image ());
    point_cloud_msgs_.reset (new sensor_msgs::PointCloud2 ());
    imu_msgs_.reset (new sensor_msgs::Imu ());

    // allocate buffer and super_frame
    buffer_ = static_cast<uint16_t*> (malloc (sizeof (sf2_t)));
    super_frame_ = static_cast<sf2_t*> (malloc (sizeof (sf2_t)));

    // init ros time for timestamps, just a hack right now
    ros::Time::init ();
    time_now_ = ros::Time::now ();

    // convert file to super frame format
    fileToSF ();
    // fill in the data for the small image
    fillSmallImgMsg ();
    // fill in the data for the big image
    fillBigImgMsg ();
    // fill in the data for the depth image
    fillPointCloudMsg ();
    // fill in the data for the imu
    fillImuMsg ();
}

SuperFrameParser::~SuperFrameParser()
{
    fclose (fd_);
    free (super_frame_);
    free (buffer_);
}

void SuperFrameParser::fileToSF ()
{
    int bytes_read;
    // Parse the PGM file, skipping the # comment bits.  The # comment pad is
    // to maintain a 4kB block alignment for EXT4 writing.
    unsigned int img_width, img_height;
    char comment_str[8192] = {0};

    // get header dimensions
    if (fscanf (fd_, "P5\n%d %d\n", &img_width, &img_height) != 2)
        throw std::runtime_error ("Failed to parse header dimensions\n");

    // get comments
    char *ret_str = fgets (comment_str, 4082, fd_);
    if (ret_str == NULL)
        throw std::runtime_error ("Failed to parse comments");

    // get max value
    unsigned int max_val;
    if (fscanf(fd_, "%d\n", &max_val) != 1)
        throw std::runtime_error ("Failed to parse max value\n");

    // Read in the data portion starting here;
    bytes_read = fread (buffer_, 1, sizeof (sf2_t), fd_);

    // connvert YUV420p to SF2
    memcpy (super_frame_, buffer_, sizeof (sf2_t));

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
    small_img_msgs_->header.frame_id = "superframe/small_image";
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
    big_img_msgs_->header.frame_id = "superframe/big_image";
    big_img_msgs_->header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                    super_frame_->header.frame.big.timestamp));
    big_img_msgs_->height = BIG_RGB_HEIGHT;
    big_img_msgs_->width = BIG_RGB_WIDTH;
    big_img_msgs_->step = big_img_msgs_->width * 2;
    big_img_msgs_->encoding = sensor_msgs::image_encodings::YUV422;
    big_img_msgs_->data.resize (big_img_msgs_->width * big_img_msgs_->height * 2);
    memcpy (&big_img_msgs_->data[0], super_frame_->big_rgb, big_img_msgs_->data.size ());
}

void SuperFrameParser::fillPointCloudMsg ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::Image depth_image;
    depth_image.height = point_cloud->height = DEPTH_IMG_HEIGHT;
    depth_image.width = point_cloud->width = DEPTH_IMG_WIDTH;
    depth_image.step = 2 * depth_image.width;
    depth_image.data.resize (depth_image.width * depth_image.height * 2);
    memcpy (&depth_image.data[0], super_frame_->depth_img, depth_image.data.size ());

    point_cloud->resize (point_cloud->height * point_cloud->width);
    convertImageToPointCloud (depth_image, point_cloud);

    pcl::toROSMsg (*point_cloud, *point_cloud_msgs_);
    point_cloud_msgs_->header.frame_id = "superframe/pointcloud";
    point_cloud_msgs_->header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                        super_frame_->header.frame.depth.timestamp));
    point_cloud_msgs_->row_step = point_cloud_msgs_->width * 2;

}


void SuperFrameParser::fillImuMsg ()
{

}

void SuperFrameParser::convertImageToPointCloud (const sensor_msgs::Image& depth_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_msg)
{
    // Use correct principal point from calibration
    float center_x = 162.384; // c_x
    float center_y = 85.5884; // c_y

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 0.001f;
    float constant_x = unit_scaling / 234.973; // f_x
    float constant_y = unit_scaling / 234.91; // f_y
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin ();
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*> (&depth_msg.data[0]);
    int row_step = depth_msg.step / sizeof (uint16_t);
    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)cloud_msg->width; ++u)
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

