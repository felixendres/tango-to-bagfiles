#include <stdint.h>
/**
 * Converts YUV420sp to RGB8 image
 * 
 * @param yuv420 byte array in YUV420 NV21 format 
 * @param rgb8 byte array in RGB8 format. Must be preallocated!
 * @param image width (in pixels)
 * @param height height (in pixels) 
 * @param uv_offset additional pixel offset of the uv-plane from the end of the y-plane
 *
 */
void convertYUV420SPtoRGB8Image(uint8_t* yuv420, uint8_t* rgb8, int width, int height, int uv_offset = 0) ;

/**
 * Converts YUV to RGB8 
 */
void convertYUVtoRGB8Pixel(int y, int u, int v, uint8_t& r, uint8_t& g, uint8_t& b);
