// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
#ifndef FRAME_DATA_SUPERFRAME_V2_H_
#define FRAME_DATA_SUPERFRAME_V2_H_

#include <stdint.h>

#ifdef NOT_COMPILING_WITH_MDK
// Pack structs on host.
#define PACKED __attribute__ ((__packed__))
#else
#define PACKED
#endif

// Maximum number of features per superframe.
#define MAX_FEATURE_COUNT 320
// Number of 'cells' the pixel pipeline runs fast over.
#define MAX_FEATURE_CELL_COUNT 16
// Number of IMU Packets per superframe.
#define MAX_IMU_COUNT 60

#define MAX_FEATURES_PER_CELL 20

// Super frame data.
// Total sf2_t structure is 20K, this allows both 4K and 1280 16bit scanline
// alignment.
// This block is designed to be 0 % 4096 to allow for fast logging.
// We also align to 1280 16bit scanlines to make MIPI happy.
// Padding allows individual structures to change without disturbing neighbors.
#define SF_WIDTH (1280)
#define SF_HEIGHT (1168)

#define SF_HEADER_LINES 16

#define BIG_RGB_WIDTH 1280
#define BIG_RGB_HEIGHT 720
#define BIG_RGB_SF_LINES 720

#define SMALL_IMG_WIDTH 640
#define SMALL_IMG_HEIGHT 480
#define SMALL_IMG_SF_LINES 240

#define PYRAMID_PAD 8
#define PYRAMID_SIZE  \
    ((SMALL_IMG_WIDTH / 2 + PYRAMID_PAD * 2) * (SMALL_IMG_HEIGHT / 2 + PYRAMID_PAD * 2) +  /*NOLINT*/\
     (SMALL_IMG_WIDTH / 4 + PYRAMID_PAD * 2) * (SMALL_IMG_HEIGHT / 4 + PYRAMID_PAD * 2) +  /*NOLINT*/\
     (SMALL_IMG_WIDTH / 8 + PYRAMID_PAD * 2) * (SMALL_IMG_HEIGHT / 8 + PYRAMID_PAD * 2))  /*NOLINT*/
#define PYRAMID_SF_LINES 96

#define DEPTH_IMG_WIDTH 320
#define DEPTH_IMG_HEIGHT 180
#define DEPTH_IMG_BPP 2
#define DEPTH_IMG_SF_LINES 96

#define SF2_HEADER_SIZE 320
#define SF2_PROFILE_SIZE 320
#define SF2_IMU_SIZE 1920

#define SF2_VTRACK_VALID 1
#define SF2_DEPTH_VALID  2
#define SF2_BIG_VALID    4

// Flags used on small, big and depth frame tearing.
#define SF_TEARING_TRUE 1
#define SF_TEARING_FALSE 0

// Timestamps.
typedef struct PACKED sf2_time_t_ {
    uint32_t ticks_hi;
    uint32_t ticks_lo;
} sf2_time_t_;

union TimeStamp {
    uint64_t superframe_v1;
    sf2_time_t_ superframe_v2;
};

typedef struct PACKED sf_frame_data_t {
  // Could be trigger time or trigger + exposure / 2.
  union TimeStamp timestamp;

  // Image exposure time in uS.
  uint32_t exposureUs;

  // Image color temperature.
  uint32_t color_temp_k;

  // Auto-focus settings.
  uint32_t af_setting;

  // Requested gain.
  uint32_t CAM_GAIN;

  // Requested exposure.
  uint32_t CAM_EXP;

  // Image tear detected flag.
  uint32_t tear_flag;
} sf_frame_data_t;

typedef struct PACKED sf_frame_t {
  // The version of this superframe structure.
  // Lowest 8 bits is minor number.
  // Upper 24 bits are major number.
  // 0x100 - version 1.00
  uint32_t sf_version;

  // The version number of firmware.
  // Upper 16-bits is the year.
  // Lower 16-bits is MMDD#, Month, Day, release # on that day.
  // Version 2014.01211 = 0x07de04bb, 0x07de = 2014, 0x04bb = 1211
  uint16_t firmware_version[2];

  // Superframe frame count from the myriad side.
  uint32_t super_frame_count;

  // Number of valid IMU packets in this superframe.
  uint32_t imu_packet_count;

  // BitMask indicating which parts below are valid.
  // For each 16-bit element a bit refers to an image segment
  //   0x01: small image (wide angle camera)
  //   0x02: depth image
  //   0x04: big color image
  // The semantics in each 16-bit element refer to if the image is newly
  // captured in this frame or not, and if the image data in that segment
  // is "valid"  or not.
  //   sf_valid_flags[1]: New flags.
  //   sf_valid_flags[0]: Data OK flags.
  uint16_t sf_valid_flags[2];

  // Frame data from the global-shutter fisheye.
  sf_frame_data_t small;

  // Frame data from the RGB-IR.
  sf_frame_data_t big;

  // Frame data from the depth sensor.
  sf_frame_data_t depth;
} sf_frame_t;

// Feature matching data.
typedef struct PACKED sf_fm_data_t {
  // FM profiling id.
  uint32_t id;

  // FM time in MV clocks.
  uint32_t time;

  // FM number of.
  uint32_t feature_count;
} sf_fm_data_t;

// Pixel pipeline profile data.
typedef struct PACKED sf_profile_data_t {
  // Processing time for pixel pipeline in ms.
  uint32_t pixel_pipe_time;

  // Processing time for optical flow in ms.
  uint32_t optical_flow_time;

  // Feature maintenance time in ms.
  uint32_t feature_maint_time;

  // Number of features detected by pixel pipeline.
  uint32_t feature_count;

  // FM data for bad feature removal stage.
  sf_fm_data_t fm1;

  // FM data for new feature search stage.
  sf_fm_data_t fm2;

  // FM data for new features admitted stage.
  sf_fm_data_t fm3;
} sf_profile_data_t;

// Raw IMU data.
typedef struct PACKED sf_imu_data_t {
  // IMU timestamp in seconds.
  union TimeStamp timestamp;

  // Enum type decoder.
  enum {IMU_INVALID, IMU_ACCEL, IMU_GYRO, IMU_MAG, IMU_BARO, IMU_TEMP,
      IMU_LAST} type;

  union {
    // Raw Accel data.
    int16_t accel[4];

    // Raw gyro data.
    int16_t gyro[4];

    // Raw mag data.
    int16_t mag[4];

    // Barometer data.
    uint32_t barometer;

    // Temperature data.
    uint32_t temperature;

    // 32bit array
    uint32_t data32[2];
  } data;
} sf_imu_data_t;

typedef struct PACKED sf_meta_data_t {
  // Rolling identifier of which id
  uint32_t id;

  // Number of frames this feature has been tracked.
  uint32_t age;

  // Original harris score when feature was detected.
  float harris_score;
} sf_meta_data_t;

typedef struct PACKED sf_xy_data_t {
  // X location of the feature.
  float x;

  // Y Location of the feature.
  float y;
} sf_xy_data_t;

typedef struct PACKED sf_vtrack_data_t {
  // MAX_FEATURE_COUNT.
  uint32_t max_features;

  // Number of valid features in prev_meta and prev_xy.
  uint32_t prev_feature_count;

  // Number of valid features in curr_meta and curr_xy.
  uint32_t curr_feature_count;

  // Meta data for the previous feature points.
  sf_meta_data_t prev_meta[MAX_FEATURE_COUNT];

  // Meta data for the current feature points.
  sf_meta_data_t curr_meta[MAX_FEATURE_COUNT];

  // Location of previous features.
  sf_xy_data_t prev_xy[MAX_FEATURE_COUNT];

  // Location of current features.
  sf_xy_data_t curr_xy[MAX_FEATURE_COUNT];

  // Feature matching error.
  float feature_error[MAX_FEATURE_COUNT];

  // Number of features per cell.
  uint32_t features_per_cell[MAX_FEATURE_CELL_COUNT];

  // Fast threshold use for current frame.
  float fast_thresholds[MAX_FEATURE_CELL_COUNT];
} sf_vtrack_data_t;

typedef struct PACKED sf2_header_t {
  // Superframe header.
  sf_frame_t frame;
  uint8_t frame_pad[SF2_HEADER_SIZE - sizeof(sf_frame_t)];

  // Profiling data.
  sf_profile_data_t profile;
  uint8_t profile_pad[SF2_PROFILE_SIZE - sizeof(sf_profile_data_t)];

  // IMU Samples.
  sf_imu_data_t imu[MAX_IMU_COUNT];
  uint8_t imu_pad[SF2_IMU_SIZE - sizeof(sf_imu_data_t) * MAX_IMU_COUNT];

  // vTrack Data.
  sf_vtrack_data_t vtrack;
  uint8_t vtrack_pad[SF_HEADER_LINES * SF_WIDTH -
      SF2_HEADER_SIZE -
      SF2_PROFILE_SIZE -
      SF2_IMU_SIZE -
      sizeof(sf_vtrack_data_t)];
}sf2_header_t;

// Application processor sees Superframes as YUV420SP format.
// The formatting here describes the Y-plane format.
typedef struct PACKED sf2_t {
  // Start Firmware Overlay 0 (Firmware internal YUV400 format).
  // Superframe header.
  sf2_header_t header;
  // Small RGB Data.
  // small_img uses 120 scanlines.
  uint8_t  small_img[SMALL_IMG_WIDTH * SMALL_IMG_HEIGHT];

  // Small image pyramid data.
  // pyramid and pyramid pad use 96 scanlines.
  uint8_t  pyramid[PYRAMID_SIZE];
  // Pad to align pyramid to 20K.
  uint8_t  pyramid_pad[PYRAMID_SF_LINES * SF_WIDTH - PYRAMID_SIZE];

  // Z Depth image 16bits.
  // depth_img and depth_pad use 96 scanlines.
  uint16_t depth_img[DEPTH_IMG_WIDTH * DEPTH_IMG_HEIGHT];
  // Pad to align depth_img to 20K.
  uint8_t  depth_pad[DEPTH_IMG_SF_LINES * SF_WIDTH - DEPTH_IMG_WIDTH *
                     DEPTH_IMG_HEIGHT * DEPTH_IMG_BPP];

  // Start Firmware Overlay 1 (Firmware internal YUV422 format).
  // Big RGB Data YUV422.
  uint8_t big_rgb[BIG_RGB_WIDTH * BIG_RGB_HEIGHT];
} sf2_t;

#endif  // FRAME_DATA_SUPERFRAME_V2_H_
