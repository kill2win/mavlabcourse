/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * opencv
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"

//derotation test
//state 
#include "state.h"
#include "subsystems/imu.h"

  float theta_prev = 0;
  float phi_prev = 0;
  float psi_prev = 0;


// Function
bool_t opencv_func(struct image_t* img);
bool_t opencv_func(struct image_t* img)
{
  
  float theta = stateGetNedToBodyEulers_f()->theta;//fetch the body angles
  float phi = stateGetNedToBodyEulers_f()->phi;
  float psi = stateGetNedToBodyEulers_f()->psi;

  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    opencv_example((char*) img->buf, img->w, img->h);
    //derotation_test((char*) img->buf, img->w, img->h, theta, phi, psi, theta_prev, phi_prev, psi_prev);
  }

  theta_prev = theta;
  phi_prev = phi;
  psi_prev = psi;
  
// opencv_example(NULL, 10,10);

  return FALSE;
}

void opencvdemo_init(void)
{
  cv_add(opencv_func);
}


