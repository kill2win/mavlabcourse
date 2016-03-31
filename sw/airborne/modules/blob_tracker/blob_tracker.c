/*
 * Copyright (C) Michaël Ozo
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/blob_tracker/blob_tracker.c"
 * @author Michaël Ozo
 * Tracks a colored blob using integral image technique. Draws a crosshair at the blob center.
 */
// Own header
#include "modules/computer_vision/cv.h"
#include "modules/blob_tracker/blob_tracker.h"
#include <stdio.h>
#include <std.h>
#include <stdlib.h>
#include <string.h>


// OpenCV libraries
//#include "opencv/cv.h"
//#define CV_H
//#include "opencv/highgui.h"
//#include "opencv/cxcore.h"

// Filter Settings
uint8_t color_lum_min = 5;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 5;
uint8_t color_cb_max  = 180;
uint8_t color_cr_min  = 50;
uint8_t color_cr_max  = 255;

uint16_t x_pos = 0;
uint16_t y_pos = 0;
uint16_t u_value = 0;
uint16_t v_value = 0;

// Result
int color_count = 0;

// Function
bool_t colorblob_func(struct image_t* img);
bool_t colorblob_func(struct image_t* img)
{
  // Filter
  color_count = colorblob_uyvy(img,img,
      color_lum_min,color_lum_max,
      color_cb_min,color_cb_max,
      color_cr_min,color_cr_max,
      &x_pos,&y_pos,
      &u_value,&v_value
      );
  //DOWNLINK_SEND_COLORFILTER(DefaultChannel, DefaultDevice, &color_count);
  return FALSE;
}

void blob_tracker_init(void)
{
  cv_add(colorblob_func);
}


uint16_t colorblob_uyvy(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *pix_x, uint16_t *pix_y, uint16_t *cp_u, uint16_t *cp_v)
{
  uint16_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  
  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
    
  char match = 0;
  uint16_t hold = 0;
  uint16_t old = 0;
  uint16_t x_mid = 0;
  uint16_t y_mid = 0;
  
  uint16_t x_integral [162] = {};//82 162
  uint16_t y_integral [240] = {};//uint16_t y_integral [240] = {};
      

  for (int y=0;y<output->h;y++)//output->h
  {
    for (int x=1;x<161;x++)//161 81
    {
      
      //center color picker
      if(x == 80 && y == output->h/2)
      {
	*cp_u = dest[0];       // U
	//*cp_u = dest[1];       // Y
        *cp_v = dest[2];        // V
      }
      
      // Color Check:
      if (
          // Light
               (dest[1] >= y_m)
            && (dest[1] <= y_M)
            && (dest[0] >= u_m)
            && (dest[0] <= u_M)
            && (dest[2] >= v_m)
            && (dest[2] <= v_M)
         )// && (dest[2] > 128))
      {
        cnt ++;
        // UYVY
        dest[0] = 250;//64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 60;//255;        // V
        dest[3] = source[3];  // Y
        
        //Binary image 1
        match = 1;
        
        
      }
      else
      {
        // UYVY
        char u = source[0]-127;
        u/=4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
        
        // Binary image 0
        match = 0;
      }

      //blob center cross
      if(x == *pix_x || y == *pix_y)
      {
	dest[0] = 64;        // U
        dest[2] = 255;        // V
      }
      
      //center pix cross
      if(x == 80 || y == output->h/2)
      {
	dest[0] = 250;        // U
        dest[2] = 60;        // V
      }
      
      
      hold = match + x_integral[x-1] + x_integral[x] - old;
      old = x_integral[x];
      x_integral[x] = hold;
      
      
      dest+=4;
      source+=4;
    } 
    old = 0;
    y_integral[y] = hold;
  }
  
  x_mid = x_integral[160] / 2;//80 160
  y_mid = y_integral[239] / 2;//119 239
  
  for (uint16_t i=1;i<161;i++)//81 161
  {
    if(x_integral[i] > x_mid)
    {
      *pix_x = i;
      break;
    }
  }
  
  for (uint16_t j=0;j<240;j++)//120 240
  {
    if(y_integral[j] > y_mid)
    {
      *pix_y = j;
      break;
    }
  }
  
  return cnt;
}



