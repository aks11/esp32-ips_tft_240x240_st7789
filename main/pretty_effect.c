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

pixel_s **pixels;

//Grab a rgb16 pixel from the esp32_tiles image
static inline pixel_s get_bgnd_pixel(int x, int y)
{
    return pixels[y][x];
}

//Calculate the pixel data for a set of lines (with implied line size of 240). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.
void pretty_effect_calc_lines(pixel_s *dest, int line, int frame, int linect)
{
    for (int y=line; y<line+linect; y++) {
        for (int x=0; x<240; x++) {
            *dest++=get_bgnd_pixel(x, y);
        }
    }
}


esp_err_t pretty_effect_init() 
{
    return decode_image(&pixels);
}
