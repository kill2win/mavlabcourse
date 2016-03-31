/*
 * Copyright (C) charles
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ext_colorfilter/ext_colorfilter.c"
 * @author charles
 * detect ext_colorfilter
 */

#include "modules/ext_colorfilter/ext_colorfilter.h"

#include "modules/computer_vision/cv.h"
//#include "modules/computer_vision/colorfilter.h"

#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>
#include <inttypes.h>

// Filter Settings Orange
uint8_t color_lum_min = 5;
uint8_t color_lum_max = 250;
uint8_t color_cb_min  = 5;
uint8_t color_cb_max  = 250;
uint8_t color_cr_min  = 5;
uint8_t color_cr_max  = 250;

float floor_0=118.0;
float floor_1=60.0;
float floor_2=126.0;
float floor_3=60.0;
float floor_tol=0.05;
float floor_tol2=0.2;
//Tresholds for floor color


float fh_u=0.2;  // upper free space
float fh_l=0.2;  // lower free space
float avg_tol = 0.01;  //Tolerance Factor for determing average pixels!
float centre_width=0.5; //Set width of combined middle sectors


float lcnt = 0;  //leftcount
float clcnt = 0; //leftcentrecount
float crcnt = 0; //rightcentrecount
float rcnt = 0; //rightcount

float avg_lcnt = 0;  //averageleftcount
float avg_clcnt = 0; //averageleftcentrecount
float avg_crcnt = 0; //averagerightcentrecount
float avg_rcnt = 0; //averagerightcount

uint16_t cnt[4] = {0, 0, 0, 0};
uint16_t avg_count [4] = {0, 0, 0, 0};
uint32_t img_prop[4][4] = {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}};
float avg_img_prop[4][4] = {{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0}};

uint32_t n_pixel_gray = 0;

uint32_t n_pixel_avg[4] = {1, 1, 1, 1};
uint32_t n_pixel_orange[4] = {1, 1, 1, 1};


uint16_t ext_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
          uint8_t u_M, uint8_t v_m, uint8_t v_M, float *left_cnt, float *centrel_cnt, float *centrer_cnt , float *right_cnt, 
		float *left_avgcnt, float *centrel_avgcnt, float *centrer_avgcnt , float *right_avgcnt);
uint16_t ext_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
          uint8_t u_M, uint8_t v_m, uint8_t v_M, float *left_cnt, float *centrel_cnt, float *centrer_cnt, float *right_cnt, 
		float *left_avgcnt, float *centrel_avgcnt, float *centrer_avgcnt , float *right_avgcnt)
{

//Set all counting variables to zero
for(uint8_t j = 0; j <=3; j++)
{
	cnt[j] = 0;
	avg_count [j] = 0;	
    	n_pixel_avg[j]=1;
	n_pixel_orange[j]=1;
    	img_prop[0][j] = 0;
	img_prop[1][j] = 0;
	img_prop[2][j] = 0;
	img_prop[3][j] = 0;	
	avg_img_prop[0][j] = 0.0;
	avg_img_prop[1][j] = 0.0;
	avg_img_prop[2][j] = 0.0;
	avg_img_prop[3][j] = 0.0;
	n_pixel_gray=0;	
}

  //memset(n_pixel_avg, 0, 3 * sizeof(n_pixel_avg[0]));

  //memset(img_prop, 0, sizeof(img_prop[0][0]) * 3* 3);
  //memset(avg_img_prop, 0.0, sizeof(img_prop[0][0]) * 3* 3);


  uint16_t w_max;
  uint16_t h_max;
  
  //float centre_width=0.5;
  float side_width=(1-centre_width)/2;
  //fh_u=0.2;  // upper free space
  //fh_l=0.2;  // lower free space



  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
  w_max=output->w;
  h_max=output->h;
  
  //--------- Calculation of an average Value for uyvy---------------------------------------------------
  uint8_t *source2 = input->buf;
  uint8_t *dest2 = output->buf;


float floor_0min = (1-floor_tol)* floor_0;
float floor_0max = (1+floor_tol)* floor_0;
float floor_1min = (1-floor_tol2)* floor_1;
float floor_1max = (1+floor_tol2)* floor_1;
float floor_2min = (1-floor_tol)* floor_2;
float floor_2max = (1+floor_tol)* floor_2;
float floor_3min = (1-floor_tol2)* floor_3;
float floor_3max = (1+floor_tol2)* floor_3;
  
  // Go trough all the pixels and sum up all pixel values
  for (uint16_t y = 0; y < (h_max); y++) { 
    for (uint16_t x = 0; x < (w_max); x += 2) {
	
	// Dont take gray pixels into account. Need to adjust the thresholds for gray floor
	if (
	(source2[0]>floor_0min && source2[0]<floor_0max) &&	 
	(source2[2]>floor_2min && source2[2]<floor_2max) &&
	(source2[1]>floor_1min && source2[1]<floor_1max) &&
	(source2[3]>floor_3min && source2[3]<floor_3max) )
	
	{n_pixel_gray ++;
			/*dest2[0] = 200;        // U
			dest2[1] = source2[1];  // Y
			dest2[2] = 100;        // V
			dest2[3] = source2[3];  // Y  	*/

	}
	else
	{	
		/*for (uint8_t nn=0; nn<=3; nn++) {
		if ((x>=((0.25*(nn))*w_max))&&(x<((0.25*(nn+1))*w_max))&&(y>fh_u*h_max)&&(y<(1-0.3)*h_max)) {
		img_prop[0][nn] += source2[0];
		img_prop[1][nn] += source2[1];
		img_prop[2][nn] += source2[2];
		img_prop[3][nn] += source2[3];
		
		n_pixel_avg[nn] ++; //
		}
		}*/
		if (x<(1.0*side_width*w_max)&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // sector left
		img_prop[0][0] += source2[0];
		img_prop[0][1] += source2[1];
		img_prop[0][2] += source2[2];
		img_prop[0][3] += source2[3];
		n_pixel_avg[0] ++; 
		}
		else if  ((x>=(1.0*side_width*w_max))&&(x<(0.5*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // sector left centre
		img_prop[1][0] += source2[0];
		img_prop[1][1] += source2[1];
		img_prop[1][2] += source2[2];
		img_prop[1][3] += source2[3];
		n_pixel_avg[1] ++; 
		}
		else if  ((x>=(0.5)*w_max)&&(x<((1-side_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // sector right centre
		img_prop[2][0] += source2[0];
		img_prop[2][1] += source2[1];
		img_prop[2][2] += source2[2];
		img_prop[2][3] += source2[3];
		n_pixel_avg[2] ++;
		}
		else if  ((x>=(1-side_width)*w_max)&&(x<(1.0*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // sector right 
		img_prop[3][0] += source2[0];
		img_prop[3][1] += source2[1];
		img_prop[3][2] += source2[2];
		img_prop[3][3] += source2[3];
		n_pixel_avg[3] ++;
		}
		
	}
		
      dest2 += 4;
      source2 += 4;
    
  }
}
// Calculate the average values
for (uint8_t nn=0; nn<=3; nn++) {
  avg_img_prop[nn][0] = (1.0*img_prop[nn][0])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][1] = (1.0*img_prop[nn][1])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][2] = (1.0*img_prop[nn][2])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][3] = (1.0*img_prop[nn][3])/(1.0*n_pixel_avg[nn]);
}
  
 printf("avgimg0 %3f, avgimg1 %3f, avgimg2 %3f, avgimg3 %3f ", avg_img_prop[1][0], avg_img_prop[1][1], avg_img_prop[1][2], avg_img_prop[1][3]);

//-----------------------Count pixels (Average,Orange)---------------------------------------------------
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  
  //float avg_tol = 0.1;  //Tolerance Factor for determing average pixels!

  // Go trough all the pixels
  for (uint16_t y = 0; y < (h_max); y++) { 
    for (uint16_t x = 0; x < (w_max); x += 2) {
	
	if (x<(1.0*side_width*w_max)&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //
	// Check if the deviation of the color from the average is within a certain range
	     if(	(((float)dest[0] >= (1.0-avg_tol)*avg_img_prop[0][0])&& ((float)dest[0] <= (1.0+avg_tol)*avg_img_prop[0][0]))
		&& 	(((float)dest[1] >= (1.0-2.0*avg_tol)*avg_img_prop[0][1])&& ((float)dest[1] <= (1.0+2.0*avg_tol)*avg_img_prop[0][1]))
		&&	(((float)dest[2] >= (1.0-avg_tol)*avg_img_prop[0][2])&& ((float)dest[2] <= (1.0+avg_tol)*avg_img_prop[0][2]))
		&&	(((float)dest[3] >= (1.0-2.0*avg_tol)*avg_img_prop[0][3])&& ((float)dest[3] <= (1.0+2.0*avg_tol)*avg_img_prop[0][3]))
		){
			dest[0] = 200;         // U
			dest[1] = source[1];  // Y
			dest[2] = 120;       // V
			dest[3] = source[3];  // Y 
			avg_count[0] ++;
		}}
		
	else if ((x>=(1.0*side_width*w_max))&&(x<(0.5*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //
	// Check if the deviation of the color from the average is within a certain range
	     if(	(((float)dest[0] >= (1.0-avg_tol)*avg_img_prop[1][0])&& ((float)dest[0] <= (1.0+avg_tol)*avg_img_prop[1][0]))
		&& 	(((float)dest[1] >= (1.0-2.0*avg_tol)*avg_img_prop[1][1])&& ((float)dest[1] <= (1.0+2.0*avg_tol)*avg_img_prop[1][1]))
		&&	(((float)dest[2] >= (1.0-avg_tol)*avg_img_prop[1][2])&& ((float)dest[2] <= (1.0+avg_tol)*avg_img_prop[1][2]))
		&&	(((float)dest[3] >= (1.0-2.0*avg_tol)*avg_img_prop[1][3])&& ((float)dest[3] <= (1.0+2.0*avg_tol)*avg_img_prop[1][3]))
		){
			dest[0] = 150;         // U
			dest[1] = source[1];  // Y
			dest[2] = 80;       // V
			dest[3] = source[3];  // Y 
			avg_count[1] ++;	
		}}
	else if ((x>=(0.5)*w_max)&&(x<((1-side_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //
	// Check if the deviation of the color from the average is within a certain range
	     if(	(((float)dest[0] >= (1.0-avg_tol)*avg_img_prop[2][0])&& ((float)dest[0] <= (1.0+avg_tol)*avg_img_prop[2][0]))
		&& 	(((float)dest[1] >= (1.0-2.0*avg_tol)*avg_img_prop[2][1])&& ((float)dest[1] <= (1.0+2.0*avg_tol)*avg_img_prop[2][1]))
		&&	(((float)dest[2] >= (1.0-avg_tol)*avg_img_prop[2][2])&& ((float)dest[2] <= (1.0+avg_tol)*avg_img_prop[2][2]))
		&&	(((float)dest[3] >= (1.0-2.0*avg_tol)*avg_img_prop[2][3])&& ((float)dest[3] <= (1.0+2.0*avg_tol)*avg_img_prop[2][3]))
		){
			dest[0] = 80;         // U
			dest[1] = source[1];  // Y
			dest[2] = 120;       // V
			dest[3] = source[3]; // Y	
			avg_count[2] ++;
		}} 
	else if ((x>=(1-side_width)*w_max)&&(x<(1.0*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //
	// Check if the deviation of the color from the average is within a certain range
	     if(	(((float)dest[0] >= (1.0-avg_tol)*avg_img_prop[3][0])&& ((float)dest[0] <= (1.0+avg_tol)*avg_img_prop[3][0]))
		&& 	(((float)dest[1] >= (1.0-2.0*avg_tol)*avg_img_prop[3][1])&& ((float)dest[1] <= (1.0+2.0*avg_tol)*avg_img_prop[3][1]))
		&&	(((float)dest[2] >= (1.0-avg_tol)*avg_img_prop[3][2])&& ((float)dest[2] <= (1.0+avg_tol)*avg_img_prop[3][2]))
		&&	(((float)dest[3] >= (1.0-2.0*avg_tol)*avg_img_prop[3][3])&& ((float)dest[3] <= (1.0+2.0*avg_tol)*avg_img_prop[3][3]))
		){
			dest[0] = 100;         // U
			dest[1] = source[1];  // Y
			dest[2] = 200;       // V
			dest[3] = source[3];  // Y	
			avg_count[3] ++;
		}} 
        else{}
	
	// Calculating all pixels considered in orange test
	if (x<(1.0*side_width*w_max)&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_orange[0] ++; }
	else if ((x>=(1.0*side_width*w_max))&&(x<(0.5*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max))  { n_pixel_orange[1] ++; }
	else if ((x>=(0.5)*w_max)&&(x<((1-side_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_orange[2] ++; }
	else if ((x>=(1-side_width)*w_max)&&(x<(1.0*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_orange[3] ++; }
	
	
      // Check if the color is inside the specified values (for orange)
	      if (
		(dest[1] >= y_m)
		&& (dest[1] <= y_M)
		&& (dest[0] >= u_m)
		&& (dest[0] <= u_M)
		&& (dest[2] >= v_m)
		&& (dest[2] <= v_M) 
	      ) {
		if (x<(1.0*side_width*w_max)&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) {  //left sector
			cnt[0] ++;
			// UYVY
			/*dest[0] = 20;//64;        // U
			dest[1] = source[1];  // Y
			dest[2] = 180;        // V
			dest[3] = source[3];  // Y*/
			}
			
		else if ((x>=(1.0*side_width*w_max))&&(x<(0.5*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // middle left sector
			cnt[1] ++;
			// UYVY
			/*dest[0] = 120;        // U
			dest[1] = source[1];  // Y
			dest[2] = 255;        // V
			dest[3] = source[3];  // Y*/
			}
                else if ((x>=(0.5)*w_max)&&(x<((1-side_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { // middle right sector
			cnt[2] ++;
			// UYVY
			/*dest[0] = 120;        // U
			dest[1] = source[1];  // Y
			dest[2] = 10;        // V
			dest[3] = source[3];  // Y*/
			}
		else if ((x>=(1-side_width)*w_max)&&(x<(1.0*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //right sector
			cnt[3] ++;
			// UYVY
			/*dest[0] = 200;        // U
			dest[1] = source[1];  // Y
			dest[2] = 100;        // V
			dest[3] = source[3];  // Y*/
			} 		

	      } else {
		/*// UYVY
		char u = source[0] - 127;
		u /= 4;
		dest[0] = 127;        // U
		dest[1] = source[1];  // Y
		u = source[2] - 127;
		u /= 4;
		dest[2] = 127;        // V
		dest[3] = source[3];  // Y*/
	      }
	
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  } 
// Orange Count
  *left_cnt=1.0*cnt[0]/n_pixel_orange[0];
  *centrel_cnt=1.0*cnt[1]/n_pixel_orange[1];
  *centrer_cnt=1.0*cnt[2]/n_pixel_orange[2];
  *right_cnt=1.0*cnt[3]/n_pixel_orange[3];
// Average Count
  *left_avgcnt=1.0*(avg_count[0])/n_pixel_avg[0];
  *centrel_avgcnt=1.0*(avg_count[1])/n_pixel_avg[1];
  *centrer_avgcnt=1.0*(avg_count[2])/n_pixel_avg[2];
  *right_avgcnt=1.0*(avg_count[3])/n_pixel_avg[3];
  return n_pixel_gray; //n_pixel_gray to determine how many pixels are neglegted
}

// Result
//int color_count;
color_count =0;

// Function
bool_t colorfilter_func(struct image_t* img);
bool_t colorfilter_func(struct image_t* img)
{
  //Orange Count
  lcnt = 0;
  clcnt = 0;
  crcnt = 0; 
  rcnt = 0;
  
  //Average Count
  avg_lcnt = 0;
  avg_clcnt = 0;
  avg_crcnt = 0; 
  avg_rcnt = 0;

  // Filter
  color_count = ext_colorfilter(img,img,
      color_lum_min,color_lum_max,
      color_cb_min,color_cb_max,
      color_cr_min,color_cr_max, &lcnt, &clcnt, &crcnt ,&rcnt, &avg_lcnt, &avg_clcnt, &avg_crcnt ,&avg_rcnt
      );

	DOWNLINK_SEND_EXT_COLORFILTER(DefaultChannel, DefaultDevice, &color_count);
  return FALSE;
}





void ext_colorfilter_init(void) {
cv_add(colorfilter_func);
}



