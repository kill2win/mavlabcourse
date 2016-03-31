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
 * @file "modules/monocular_avoidance/monocular_avoidance.h"
 * @author Michaël Ozo
 * Obstacle avoidance unsing bebob frontfacing camera imu and optitrack data.
 */

#ifndef MONOCULAR_AVOIDANCE_H
#define MONOCULAR_AVOIDANCE_H

#include <std.h>
#include "lib/vision/image.h"

extern void monocular_avoidance(void);
extern float div1;

/* The result calculated from the opticflow */
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

struct opticflow_t {
  bool_t got_first_img;             ///< If we got a image to work with
  float prev_phi;                   ///< Phi from the previous image frame
  float prev_theta;                 ///< Theta from the previous image frame
  struct image_t img_gray;          ///< Current gray image frame
  struct image_t prev_img_gray;     ///< Previous gray image frame
  struct timeval prev_timestamp;    ///< Timestamp of the previous frame, used for FPS calculation

  uint8_t max_track_corners;        ///< Maximum amount of corners Lucas Kanade should track
  uint16_t window_size;             ///< Window size of the Lucas Kanade calculation (needs to be even)
  uint8_t subpixel_factor;          ///< The amount of subpixels per pixel
  uint8_t max_iterations;           ///< The maximum amount of iterations the Lucas Kanade algorithm should do
  uint8_t threshold_vec;            ///< The threshold in x, y subpixels which the algorithm should stop

  bool_t fast9_adaptive;            ///< Whether the FAST9 threshold should be adaptive
  uint8_t fast9_threshold;          ///< FAST9 corner detection threshold
  uint16_t fast9_min_distance;      ///< Minimum distance in pixels between corners
};

#endif

