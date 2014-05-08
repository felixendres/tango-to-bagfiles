#include "superframe_parser/super_frame_parser.h"
#include <sensor_msgs/image_encodings.h>

SuperFrameParser::SuperFrameParser (const std::string &name_space,
                                    const std::string &fisheye_name,
                                    const std::string &narrow_name,
                                    const std::string &pointcloud_name,
                                    const std::string &timestamp_file) :
    name_space_ (name_space),
    fisheye_name_ (fisheye_name),
    narrow_name_ (narrow_name),
    pointcloud_name_ (pointcloud_name)
{
    buildTimestampMap (timestamp_file);
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
    fisheye_msgs_.reset (new sensor_msgs::Image ());
    narrow_msgs_.reset(new sensor_msgs::Image ());
    point_cloud_msgs_.reset (new sensor_msgs::PointCloud2 ());
    imu_msgs_.reset (new sensor_msgs::Imu ());

    fisheye_info_.reset (new sensor_msgs::CameraInfo ());
    narrow_info_.reset (new sensor_msgs::CameraInfo ());
    depth_info_.reset (new sensor_msgs::CameraInfo ());

    // allocate super_frame
    super_frame_ = static_cast<sf2_t*> (malloc (sizeof (sf2_t)));

    // parse the file name for accesing the timestamp map
    boost::filesystem::path path (super_frame_file);
    file_name_ = path.filename ().string ();
    file_name_.erase (file_name_.size () -  path.extension ().string ().size ());

    // convert file to super frame format
    parseSfFile (super_frame_file);
    // fill in the data for the small image
    fillFisheyeMsg (fisheye_intrinsics);
    // fill in the data for the big image
    fillNarrowMsg (narrow_intrinsics);
    // fill in the data for the depth image
    fillPointCloudMsg (depth_intrinsics);
    // fill in the data for the imu
//    fillImuMsg ();

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
    memcpy (super_frame_, buffer, bytes_read);
    free (buffer);
    fclose (fd);
}

void SuperFrameParser::parseCameraInfo (const std::string &params_file, std::vector<std::string> &params)
{
    std::ifstream f (params_file.c_str ());
    if (!f.is_open ())
        throw std::runtime_error ("Could not open intrinsic parameters file!");

    std::string line;
    getline (f, line);

    size_t pos = 0;
    std::string token;

    std::string delimiter = ",";

    while ((pos = line.find (delimiter)) != std::string::npos)
    {
        token = line.substr (0, pos);
        params.push_back (token);
        line.erase (0, pos + 1);
    }
    params.push_back (line);
    f.close ();
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

void SuperFrameParser::fillFisheyeMsg (const std::string &params_file)
{
    fisheye_msgs_->header.frame_id = "/" + name_space_ + "/" + fisheye_name_;
    fisheye_msgs_->header.stamp.fromSec (timestamp_map_.find (file_name_)->second);
    fisheye_msgs_->height = SMALL_IMG_HEIGHT;
    fisheye_msgs_->width = SMALL_IMG_WIDTH;
    fisheye_msgs_->step = fisheye_msgs_->width;
    fisheye_msgs_->encoding = sensor_msgs::image_encodings::MONO8;
    fisheye_msgs_->data.resize (fisheye_msgs_->width * fisheye_msgs_->height);
    memcpy (&fisheye_msgs_->data[0], super_frame_->small_img, fisheye_msgs_->data.size ());


    ///// fill in the camera infos ////////
    std::vector<std::string> params;
    parseCameraInfo (params_file, params);

    fisheye_info_->header.frame_id = fisheye_msgs_->header.frame_id;
    fisheye_info_->header.stamp = fisheye_msgs_->header.stamp;
    fisheye_info_->width = atoi (params[0].c_str ());
    fisheye_info_->height = atoi (params[1].c_str ());
    fisheye_info_->distortion_model = "devernay";
    fisheye_info_->D.resize (2);
    fisheye_info_->D[0] = atof (params[6].c_str ()); // omega
    fisheye_info_->D[1] = atof (params[7].c_str ()); // max_angle

    /* Intrinsic camera matrix for the raw (distorted) images.
       #     [fx  0 cx]
       # K = [ 0 fy cy]
       #     [ 0  0  1] */
    boost::array<double, 9> K = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()),
                                  0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()),
                                  0.0,                       0.0,                       1.0  } ;
    fisheye_info_->K = K;

    /* # Projection/camera matrix
       #     [fx'  0  cx' Tx]
       # P = [ 0  fy' cy' Ty]
       #     [ 0   0   1   0] */

    boost::array<double, 12> P = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()), 0.,
                                   0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()), 0.,
                                   0.0,                       0.0,                       1.0,                       0. } ;
    fisheye_info_->P = P;
}

void SuperFrameParser::fillNarrowMsg (const std::string &params_file)
{
    narrow_msgs_->header.frame_id = "/" + name_space_ + "/" + narrow_name_;

    // big image has apperently some offset, adding the offset here
//    big_img_msgs_->header.stamp.fromSec (timestamp_map_.find (file_name_)->second +
//                                         (convertTicksToSeconds (super_frame_->header.frame.sf_version, super_frame_->header.frame.big.timestamp) -
//                                          convertTicksToSeconds (super_frame_->header.frame.sf_version, super_frame_->header.frame.small.timestamp)));

    narrow_msgs_->header.stamp.fromSec (timestamp_map_.find (file_name_)->second);
    narrow_msgs_->height = BIG_RGB_HEIGHT;
    narrow_msgs_->width = BIG_RGB_WIDTH;
    narrow_msgs_->step = narrow_msgs_->width/* * 2*/;
    narrow_msgs_->encoding = sensor_msgs::image_encodings::MONO8;
    narrow_msgs_->data.resize (narrow_msgs_->width * narrow_msgs_->height/* * 2*/);
    memcpy (&narrow_msgs_->data[0], super_frame_->big_rgb, narrow_msgs_->data.size ());


    ///// fill in the camera infos ////////
    std::vector<std::string> params;
    parseCameraInfo (params_file, params);

    narrow_info_->header.frame_id = narrow_msgs_->header.frame_id;
    narrow_info_->header.stamp = narrow_msgs_->header.stamp;
    narrow_info_->width = atoi (params[0].c_str ());
    narrow_info_->height = atoi (params[1].c_str ());
    narrow_info_->distortion_model = "devernay";
    narrow_info_->D.resize (2);
    narrow_info_->D[0] = atof (params[6].c_str ()); // omega
    narrow_info_->D[1] = atof (params[7].c_str ()); // max_angle

    /* Intrinsic camera matrix for the raw (distorted) images.
       #     [fx  0 cx]
       # K = [ 0 fy cy]
       #     [ 0  0  1] */
    boost::array<double, 9> K = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()),
                                  0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()),
                                  0.0,                       0.0,                       1.0  } ;
    narrow_info_->K = K;

    /* # Projection/camera matrix
       #     [fx'  0  cx' Tx]
       # P = [ 0  fy' cy' Ty]
       #     [ 0   0   1   0] */

    boost::array<double, 12> P = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()), 0.,
                                   0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()), 0.,
                                   0.0,                       0.0,                       1.0,                       0. } ;
    narrow_info_->P = P;
}

void SuperFrameParser::fillPointCloudMsg (const std::string &params_file)
{
    sensor_msgs::ImagePtr depth_image (new sensor_msgs::Image ());
    depth_image->height = DEPTH_IMG_HEIGHT;
    depth_image->width = DEPTH_IMG_WIDTH;
    depth_image->step = 2 * depth_image->width;
    depth_image->data.resize (depth_image->width * depth_image->height * 2);
    memcpy (&depth_image->data[0], super_frame_->depth_img, depth_image->data.size ());

    ///// fill in the camera infos ////////
    std::vector<std::string> params;
    parseCameraInfo (params_file, params);

    depth_info_->header.frame_id = point_cloud_msgs_->header.frame_id;
    depth_info_->header.stamp = point_cloud_msgs_->header.stamp;
    depth_info_->width = atoi (params[0].c_str ());
    depth_info_->height = atoi (params[1].c_str ());
    depth_info_->distortion_model = "devernay";
    depth_info_->D.resize (2);
    depth_info_->D[0] = atof (params[6].c_str ()); // omega
    depth_info_->D[1] = atof (params[7].c_str ()); // max_angle

    /* Intrinsic camera matrix for the raw (distorted) images.
       #     [fx  0 cx]
       # K = [ 0 fy cy]
       #     [ 0  0  1] */
    boost::array<double, 9> K = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()),
                                  0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()),
                                  0.0,                       0.0,                       1.0  } ;
    depth_info_->K = K;

    /* # Projection/camera matrix
       #     [fx'  0  cx' Tx]
       # P = [ 0  fy' cy' Ty]
       #     [ 0   0   1   0] */

    boost::array<double, 12> P = { atof (params[2].c_str ()), 0.0,                       atof (params[4].c_str ()), 0.,
                                   0.0,                       atof (params[3].c_str ()), atof (params[5].c_str ()), 0.,
                                   0.0,                       0.0,                       1.0,                       0. } ;
    depth_info_->P = P;


    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    convertImageToPointCloud (depth_image, point_cloud);

    pcl::toROSMsg (*point_cloud, *point_cloud_msgs_);
    point_cloud_msgs_->header.frame_id = "/" + name_space_ + "/" + pointcloud_name_;

    // pre defined offset from depth to small image timestamp
//    point_cloud_msgs_->header.stamp.fromSec (timestamp_map_.find (file_name_)->second + DEPTH_TIMESTAMP_OFFSET);
    // calculate depth offset by getting the difference from the superframe timestamps
    point_cloud_msgs_->header.stamp.fromSec (timestamp_map_.find (file_name_)->second +
                                             (convertTicksToSeconds (super_frame_->header.frame.sf_version, super_frame_->header.frame.depth.timestamp) -
                                              convertTicksToSeconds (super_frame_->header.frame.sf_version, super_frame_->header.frame.small.timestamp)));

    point_cloud_msgs_->row_step = point_cloud_msgs_->width * 2;

}


void SuperFrameParser::fillImuMsg (const std::string &params_file)
{

}

void SuperFrameParser::buildTimestampMap (const std::string &timestamp_file)
{
    std::ifstream f (timestamp_file.c_str ());
    if (!f.is_open ())
        throw std::runtime_error ("Could not open timestamp file!");

    std::string line;
    std::string token;
    std::string delimiter = ",";
    while (getline (f, line))
    {
        size_t pos = 0;
        std::pair<std::string, double> p;

        // first token is the image name
        pos = line.find (delimiter);
        token = line.substr (0, pos);
        p.first = token;
        line.erase (0, pos + 1);

        // second token is the timestamp
        pos = line.find (delimiter);
        token = line.substr (0, pos);
        p.second = atof (token.c_str ());

        timestamp_map_.insert (p);
    }

}

void SuperFrameParser::convertImageToPointCloud (const sensor_msgs::ImagePtr& depth_msg, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    cloud->height = depth_msg->height;
    cloud->width = depth_msg->width;
    cloud->resize (cloud->height * cloud->width);
    // Use correct principal point from calibration
    float center_x = depth_info_->K[2]; // c_x
    float center_y = depth_info_->K[5]; // c_y

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 0.001f;
    float constant_x = unit_scaling / depth_info_->K[0]; // f_x
    float constant_y = unit_scaling / depth_info_->K[4]; // f_y
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

