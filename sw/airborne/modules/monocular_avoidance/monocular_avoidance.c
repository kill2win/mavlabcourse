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
 * @file "modules/monocular_avoidance/monocular_avoidance.c"
 * @author Michaël Ozo
 * Obstacle avoidance unsing bebob frontfacing camera imu and optitrack data.
 */

// Own header
#include "modules/computer_vision/cv.h"
#include "modules/monocular_avoidance/monocular_avoidance.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

//state 
#include "state.h"
#include "subsystems/imu.h"

// Calculations
#include <math.h>
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "boards/ardrone/navdata.h" // for ultrasound Height

//coordinates
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps/gps_datalink.h"

//divergence
#include "modules/computer_vision/opticflow/size_divergence.h"


#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20//30//10//20//5
#endif

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE FALSE
#endif

#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25//50
#endif

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 40//60//20//10
#endif

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 20//10//20
#endif

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif

/*
#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
*/
struct opticflow_result_t result;
struct opticflow_t opticflow;


// Filter Settings
/*
uint8_t color_lum_min = 5;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 5;
uint8_t color_cb_max  = 200;
uint8_t color_cr_min  = 5;
uint8_t color_cr_max  = 230;

int color_count = 0;
*/
//Attitude
int32_t phi_temp = 0;
int32_t theta_temp = 0;
int32_t psi_temp = 0;

//Position
int32_t x_temp = 0;
int32_t y_temp = 0;
int32_t z_temp = 0;

//divergence
float divergence = 0;
float u = 0;
float v = 0;
float div1 = 0;


bool_t process_frame(struct image_t* img);
bool_t process_frame(struct image_t* img)
{
  
  
  // Convert image to grayscale
  image_to_grayscale(img, &opticflow.img_gray);

  // Copy to previous image if not set
  if (!opticflow.got_first_img) {
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    opticflow.got_first_img = TRUE;
  }
  
  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threshold)
  struct point_t *corners = fast9_detect(img, OPTICFLOW_FAST9_THRESHOLD, OPTICFLOW_FAST9_MIN_DISTANCE,
                                         50, 50, &result.corner_cnt);
  
  // Adaptive threshold
  if (opticflow.fast9_adaptive) {

    // Decrease and increase the threshold based on previous values
    if (result.corner_cnt < 40 && opticflow.fast9_threshold > 5) {
      opticflow.fast9_threshold--;
    } else if (result.corner_cnt > 50 && opticflow.fast9_threshold < 60) {
      opticflow.fast9_threshold++;
    }
  }
  
  //image_show_points(img, corners, result.corner_cnt);
  
  int32_t debug = result.corner_cnt;
  int32_t debug_tr = opticflow.fast9_threshold;
  phi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->phi);
  theta_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->theta);
  psi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->psi);
  x_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->x);
  y_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->y);
  z_temp = POS_BFP_OF_REAL(stateGetPositionEnu_f()->z);
 
 //int32_t debug = img->h;
  
 // DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &debug, &debug_tr, &phi_temp, &theta_temp, &psi_temp, &x_temp, &y_temp, &z_temp);
  
  
  // Check if we found some corners to track
  if (result.corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow.img_gray, &opticflow.prev_img_gray);
    return FALSE;
  }
  
  
  
  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow
  result.tracked_cnt = result.corner_cnt;
  
  struct flow_t *vectors = opticFlowLK(&opticflow.img_gray, &opticflow.prev_img_gray, corners, &result.tracked_cnt,
                                       opticflow.window_size / 2, opticflow.subpixel_factor, opticflow.max_iterations,
                                       opticflow.threshold_vec, opticflow.max_track_corners);

  //image_show_flow(img, vectors, result.tracked_cnt, opticflow.subpixel_factor);
  
  divergence = 0;
  u = 0;
  v = 0;
  div1 = 0;
  float divMag = 0;
  float x_prev = 0;
  float y_prev = 0;
  for(int i = 0; i < result.tracked_cnt;i++)
  {
  u += vectors[i].flow_x;
  v += vectors[i].flow_y;
  
   x_prev = ((vectors[i].pos.x/opticflow.subpixel_factor) - 136);
   y_prev = ((vectors[i].pos.y/opticflow.subpixel_factor) - 136);
  
  div1 = (((vectors[i].flow_x/opticflow.subpixel_factor) / x_prev) + ((vectors[i].flow_y/opticflow.subpixel_factor) / y_prev)) / 2;
  divMag = sqrt((pow(((vectors[i].flow_x/opticflow.subpixel_factor) / x_prev),2)) + (pow(((vectors[i].flow_y/opticflow.subpixel_factor) / y_prev),2)))/2;
  //if(div != INFINITY && div != NAN)divergence += div;
  if((x_prev > 0.00001 || x_prev < -0.00001)&&(y_prev > 0.00001 || y_prev < -0.00001))divergence += div1;
  //divergence += div;
  }
  if(result.tracked_cnt>0)
  {
  u = u / result.tracked_cnt;
  v = v / result.tracked_cnt;
  }
  //stateGetPositionEnu_f()->x;
  
  //paparazi divergence
  int n_samples = 0;//100;
  float  size_divergence = get_size_divergence(vectors, result.tracked_cnt, n_samples);
  float focus_x = 0;
  float focus_y = 0;
  
  //focus_of_expansion(vectors,&focus_x,&focus_y,result.tracked_cnt);
  
  
  DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &debug, &debug_tr, &size_divergence, &divMag, &div1, &phi_temp, &theta_temp, &psi_temp, &x_temp, &y_temp, &z_temp);

 // DOWNLINK_SEND_MONOCULAR_AVOIDANCE(DefaultChannel, DefaultDevice, &vector_debug);
  
  free(corners);
  free(vectors);
  image_switch(&opticflow.img_gray, &opticflow.prev_img_gray);
 
  return FALSE;
}

 void monocular_avoidance(void) 
{
   cv_add(process_frame);
   
   /* Create the image buffers */
  image_create(&opticflow.img_gray, 272, 272, IMAGE_GRAYSCALE);
  image_create(&opticflow.prev_img_gray, 272, 272, IMAGE_GRAYSCALE);
  
  /* Create the image buffers */
  //image_create(&opticflow.img_gray, 400, 400, IMAGE_GRAYSCALE);
  //image_create(&opticflow.prev_img_gray, 400, 400, IMAGE_GRAYSCALE);
  
  /* Create the image buffers */
  //image_create(opticflow.img_gray, 272, 272, IMAGE_GRAYSCALE);
  //image_create(opticflow.prev_img_gray, 272, 272, IMAGE_GRAYSCALE);
   
   /* Set the default values */
   
   opticflow.got_first_img = FALSE;
  opticflow.max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow.window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow.window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow.subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow.max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow.threshold_vec = OPTICFLOW_THRESHOLD_VEC;

  opticflow.fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow.fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow.fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
  
}

void focus_of_expansion(struct flow_t *vectors, float *foe_x, float *foe_y, uint16_t points_cnt)
{
		float a_1 = 0;
		float a_2 = 0;
		float a_3 = 0;
		float a_4 = 0;
		float f_1 = 0;
		float f_2 = 0;

		for (int i = 0; i < points_cnt; i++) {

			float x_div = (float)vectors[i].flow_x;
			float prev_corner_x = (float)vectors[i].pos.x;

			//A matrix and f vector elements for least squares
			a_1 += 1;
			a_2 += prev_corner_x;
			a_3 += prev_corner_x;
			a_4 += prev_corner_x*prev_corner_x;
			f_1 += x_div;
			f_2 += x_div*prev_corner_x;
			
		}

		//elements of B = A^-1
		float det = (a_1*a_4) - (a_2*a_3);
		float b_1 = a_4 / det;
		float b_2 = -a_2 / det;
		float b_3 = -a_3 / det;
		float b_4 = a_1 / det;

		float b = f_1*b_1 + f_2*b_2;
		float a = f_1*b_3 + f_2*b_4;

		*foe_x = (-b / a)/10;
		
		 a_1 = 0;
		 a_2 = 0;
		 a_3 = 0;
		 a_4 = 0;
		 f_1 = 0;
		 f_2 = 0;

		for (int i = 0; i < points_cnt; i++) {

			float y_div = (float)vectors[i].flow_y;
			float prev_corner_y = (float)vectors[i].pos.y;

			//A matrix and f vector elements for least squares
			a_1 += 1;
			a_2 += prev_corner_y;
			a_3 += prev_corner_y;
			a_4 += prev_corner_y*prev_corner_y;
			f_1 += y_div;
			f_2 += y_div*prev_corner_y;
			
		}

		//elements of B = A^-1
		 det = (a_1*a_4) - (a_2*a_3);
		 b_1 = a_4 / det;
		 b_2 = -a_2 / det;
		 b_3 = -a_3 / det;
		 b_4 = a_1 / det;

		 b = f_1*b_1 + f_2*b_2;
		 a = f_1*b_3 + f_2*b_4;

		*foe_y = (-b / a)/10;
  
  
}

