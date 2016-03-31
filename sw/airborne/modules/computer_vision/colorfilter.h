/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.h
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>

// Module functions
extern void colorfilter_init(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;

struct opticflow_result_t {
  float fps;              ///< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    ///< The amount of coners found by FAST9
  uint16_t tracked_cnt;   ///< The amount of tracked corners

  int16_t flow_x;         ///< Flow in x direction from the camera (in subpixels)
  int16_t flow_y;         ///< Flow in y direction from the camera (in subpixels)
  int16_t flow_der_x;     ///< The derotated flow calculation in the x direction (in subpixels)
  int16_t flow_der_y;     ///< The derotated flow calculation in the y direction (in subpixels)

  float vel_x;            ///< The velocity in the x direction
  float vel_y;            ///< The velocity in the y direction

  float div_size;         ///< Divergence as determined with the size_divergence script

  float surface_roughness; ///< Surface roughness as determined with a linear optical flow fit
  float divergence;       ///< Divergence as determined with a linear flow fit

  float noise_measurement;  ///< noise of measurement, for state filter
};


#endif /* COLORFILTER_CV_PLUGIN_H */
