#include "yuv2rgb.h"
#include <stddef.h>
#include <assert.h>
#include <initializer_list>

inline void convertYUVtoRGB8Pixel(int y, int u, int v, uint8_t& r, uint8_t& g, uint8_t& b) {
    int tmp_r = y + (int)(1.772f*v);
    int tmp_g = y - (int)(0.344f*v + 0.714f*u);
    int tmp_b = y + (int)(1.402f*u);
    r = tmp_r>255? 255 : tmp_r<0 ? 0 : tmp_r;
    g = tmp_g>255? 255 : tmp_g<0 ? 0 : tmp_g;
    b = tmp_b>255? 255 : tmp_b<0 ? 0 : tmp_b;
}

void convertYUV420SPtoRGB8Image(uint8_t* yuv420, uint8_t* rgb8, int width, int height, int uv_offset) {
    assert(yuv420 && "Nullpointer as input");
    assert(rgb8 && "Nullpointer as output");

    size_t pixel_size = width*height;
    //uv is stored below the luminance image: add offset
    size_t offset = pixel_size + uv_offset; 
    
    for(size_t y=0, uv=0; y < pixel_size; y+=2, uv+=2) {

      int v = yuv420[offset + uv  ] & 0xff;
      v -= 128;
      int u = yuv420[offset + uv+1] & 0xff;
      u -= 128;
      
      for(int i : { 0, 1, width, width+1 }) {//Walk through pixels belonging to uv block
        size_t pix_rgb = 3*(y + i); //location of rgb pixel
        int luma_val = yuv420[y + i] & 0xff;
        convertYUVtoRGB8Pixel(luma_val, u, v, rgb8[pix_rgb], rgb8[pix_rgb+1], rgb8[pix_rgb+2]);
      }

      if (y!=0 && (y+2)%width==0){//End of line
        y += width; //Skip one line in input b/c uv blocks are 2x2
      }
    }
}

