#include "super_frame_parser.h"

SuperFrameParser::SuperFrameParser (const std::string &super_frame_file,
                                    const size_t rows, const size_t cols, const float bpp) :
    cols_ (cols),
    rows_ (rows),
    bpp_ (bpp)
{
    if ((fd_ = fopen (super_frame_file.c_str (), "rb")) == NULL)
        throw std::runtime_error ("Failed to open file");

    // allocate buffer and super_frame
    buffer_ = static_cast<uint16_t*> (malloc (cols_ * rows_ * bpp_));
    super_frame_ = static_cast<sf2_t*> (malloc (sizeof (sf2_t)));

    // init ros time for timestamps, just a hack right now
    ros::Time::init ();
    time_now_ = ros::Time::now ();

    // convert file to super frame format
    fileToSF ();
    // fill in the data for the small image
    fillSmallImgMsg ();
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

    // Read in the data portion starting here
    bytes_read = fread (buffer_, 1, cols_ * rows_ * bpp_, fd_);

    // connvert YUV420p to SF2
    memcpy (super_frame_, buffer_, cols_ * rows_ * bpp_);

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
    small_img_msgs_.header.frame_id = "superframe/small_image";
    small_img_msgs_.header.stamp = time_now_ + ros::Duration (convertTicksToSeconds (super_frame_->header.frame.sf_version,
                                                                                    super_frame_->header.frame.small.timestamp));
    small_img_msgs_.height = SMALL_IMG_HEIGHT;
    small_img_msgs_.width = SMALL_IMG_WIDTH;
    small_img_msgs_.encoding = sensor_msgs::image_encodings::MONO8;
    small_img_msgs_.data.resize (small_img_msgs_.width * small_img_msgs_.height);
    memcpy (&small_img_msgs_.data[0], super_frame_->small_img, small_img_msgs_.data.size ());

}

void SuperFrameParser::fillImuMsg ()
{

}

