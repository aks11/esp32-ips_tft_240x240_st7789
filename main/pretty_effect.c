/*
   This code generates an effect that should pass the 'fancy graphics' qualification
   as set in the comment in the spi_master code.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <math.h>
#include "pretty_effect.h"
#include "decode_image.h"

uint16_t **pixels;

//Grab a rgb16 pixel from the esp32_tiles image
static inline uint16_t get_bgnd_pixel(int x, int y)
{
    return pixels[y][x];
}


//This variable is used to detect the next frame.
static int prev_frame=-1;

//Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
//these as we go through all the pixels in the frame. This is much, much faster.
static int8_t xofs[240], yofs[240];
static int8_t xcomp[240], ycomp[240];

//Calculate the pixel data for a set of lines (with implied line size of 240). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.
void pretty_effect_calc_lines(uint16_t *dest, int line, int frame, int linect)
{
    for (int y=line; y<line+linect; y++) {
        for (int x=0; x<240; x++) {
            *dest++=get_bgnd_pixel(x+yofs[y]+xcomp[x], y+xofs[x]+ycomp[y]);
        }
    }
}


esp_err_t pretty_effect_init() 
{
    return decode_image(&pixels);
}
