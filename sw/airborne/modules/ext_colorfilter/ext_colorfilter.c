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

//Tresholds for floor color
float floor_0=118.0;
float floor_1=60.0;
float floor_2=126.0;
float floor_3=60.0;
float floor_tol=0.05;
float floor_tol2=0.45;

//Tresholds for black color
float black_0=126.9;
float black_1=27.0;
float black_2=126.9;
float black_3=27.0;
float black_tol=0.05;
float black_tol2=0.15;


float fh_u=0.2;  // upper free space
float fh_l=0.2;  // lower free space
float avg_tol = 0.075;  //Tolerance Factor for determing average pixels!
float avg_tol2 = 0.1;
float centre_width=0.5; //Set width of combined middle sectors
float s_width=0.2; //Set width of combined middle sectors

float lcnt = 0;  //leftcount
float clcnt = 0; //leftcentrecount
float cccnt = 0; //centrecentrecount
float crcnt = 0; //rightcentrecount
float rcnt = 0; //rightcount

float black_lcnt = 0;  //leftcount
float black_clcnt = 0; //leftcentrecount
float black_cccnt = 0; //leftcentrecount
float black_crcnt = 0; //rightcentrecount
float black_rcnt = 0; //rightcount

float avg_lcnt = 0;  //averageleftcount
float avg_clcnt = 0; //averageleftcentrecount
float avg_cccnt = 0; //averagecentrecentrecount
float avg_crcnt = 0; //averagerightcentrecount
float avg_rcnt = 0; //averagerightcount 

uint16_t cnt[5] = {0, 0, 0, 0, 0};
uint16_t black_cnt[5] = {0, 0, 0, 0, 0};
uint16_t avg_count [5] = {0, 0, 0, 0, 0};
uint32_t img_prop[5][4] = {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}};
float avg_img_prop[5][4] = {{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0}};

uint32_t n_pixel_gray = 0;

uint32_t n_pixel_avg[5] = {1, 1, 1, 1, 1};
uint32_t n_pixel_color[5] = {1, 1, 1, 1, 1};


uint16_t ext_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
          uint8_t u_M, uint8_t v_m, uint8_t v_M, float *left_cnt, float *centrel_cnt, float *centrec_cnt, float *centrer_cnt , float *right_cnt, float *left_avgcnt, float *centrel_avgcnt, float *centrec_avgcnt, float *centrer_avgcnt , float *right_avgcnt, float *left_black_cnt, float *centrel_black_cnt, float *centrec_black_cnt, float *centrer_black_cnt, float *right_black_cnt);
uint16_t ext_colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
          uint8_t u_M, uint8_t v_m, uint8_t v_M, float *left_cnt, float *centrel_cnt , float *centrec_cnt, float *centrer_cnt, float *right_cnt, 	float *left_avgcnt, float *centrel_avgcnt , float *centrec_avgcnt, float *centrer_avgcnt , float *right_avgcnt, float *left_black_cnt, float *centrel_black_cnt, float *centrec_black_cnt, float *centrer_black_cnt, float *right_black_cnt)
{

//Set all counting variables to zero
for(uint8_t j = 0; j <=3; j++)
{
	
    	img_prop[0][j] = 0;
	img_prop[1][j] = 0;
	img_prop[2][j] = 0;
	img_prop[3][j] = 0;
	img_prop[4][j] = 0;	
	avg_img_prop[0][j] = 0.0;
	avg_img_prop[1][j] = 0.0;
	avg_img_prop[2][j] = 0.0;
	avg_img_prop[3][j] = 0.0;
	avg_img_prop[4][j] = 0.0;
	n_pixel_gray=0;	
}
for(uint8_t j = 0; j <=4; j++)
{	
	cnt[j] = 0;
	black_cnt[j]=0;
	avg_count [j] = 0;	
    	n_pixel_avg[j]=1;
	n_pixel_color[j]=1;
}
 
  uint16_t w_max;
  uint16_t h_max;


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


float black_0min = (1-black_tol)* black_0;
float black_0max = (1+black_tol)* black_0;
float black_1min = (1-black_tol2)* black_1;
float black_1max = (1+black_tol2)* black_1;
float black_2min = (1-black_tol)* black_2;
float black_2max = (1+black_tol)* black_2;
float black_3min = (1-black_tol2)* black_3;
float black_3max = (1+black_tol2)* black_3;
  
  // Go trough all the pixels and sum up all pixel values
 /* for (uint16_t y = 0; y < (h_max); y++) { 
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
			dest2[3] = source2[3];  // Y  

	}
	else
	{	
		for (uint8_t nn=0; nn<=4; nn++) {
		if ((x>=(1.0*nn*s_width*w_max))&&(x<(1.0*(nn+1)*s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) {
		img_prop[nn][0] += source2[0];
		img_prop[nn][1] += source2[1];
		img_prop[nn][2] += source2[2];
		img_prop[nn][3] += source2[3];
		
		n_pixel_avg[nn] ++; //
		}
		}
	}
		
      dest2 += 4;
      source2 += 4;
    
  }
} */
// Calculate the average values
/*for (uint8_t nn=0; nn<=4; nn++) {
  avg_img_prop[nn][0] = (1.0*img_prop[nn][0])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][1] = (1.0*img_prop[nn][1])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][2] = (1.0*img_prop[nn][2])/(1.0*n_pixel_avg[nn]);
  avg_img_prop[nn][3] = (1.0*img_prop[nn][3])/(1.0*n_pixel_avg[nn]);
}*/
  
// printf("avgimg0 %3f, avgimg1 %3f, avgimg2 %3f, avgimg3 %3f \n", avg_img_prop[1][0], avg_img_prop[1][1], avg_img_prop[1][2], avg_img_prop[1][3]);

//-----------------------Count pixels (Average,Orange)---------------------------------------------------
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Go trough all the pixels
  for (uint16_t y = 0; y < (h_max); y++) { 
    for (uint16_t x = 0; x < (w_max); x += 2) {
	/*
	for (uint8_t j =0; j<=4; j++){
	if  ((x>=(1.0*j*s_width*w_max))&&(x<(1.0*(j+1)*s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { //
	// Check if the deviation of the color from the average is within a certain range
	     if(	(((float)dest[0] >= (1.0-avg_tol)*avg_img_prop[j][0])&& ((float)dest[0] <= (1.0+avg_tol)*avg_img_prop[j][0]))
		&& 	(((float)dest[1] >= (1.0-avg_tol2)*avg_img_prop[j][1])&& ((float)dest[1] <= (1.0+avg_tol2)*avg_img_prop[j][1]))
		&&	(((float)dest[2] >= (1.0-avg_tol)*avg_img_prop[j][2])&& ((float)dest[2] <= (1.0+avg_tol)*avg_img_prop[j][2]))
		&&	(((float)dest[3] >= (1.0-avg_tol2)*avg_img_prop[j][3])&& ((float)dest[3] <= (1.0+avg_tol2)*avg_img_prop[j][3]))
		){
			dest[0] = 230-j*50;         // U
			dest[1] = source[1];  // Y
			dest[2] = 20+j*30;       // V
			dest[3] = source[3];  // Y 
			avg_count[j] ++;	
		}}
	}*/
	
	
	// Calculating all pixels considered in orange test
	if ((x<(s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_color[0] ++; }
	else if  ((x>=(s_width*w_max))&&(x<(2.0*s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max))  { n_pixel_color[1] ++; }
	else if  ((x>=(2.0*s_width*w_max))&&(x<((3.0*s_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_color[2] ++; }
	else if  ((x>=(3.0*s_width*w_max))&&(x<((4.0*s_width)*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_color[3] ++; }
	else if  ((x>=(4.0*s_width*w_max))&&(x<(1.0*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) { n_pixel_color[4] ++; }
	
	
      // Oranger colorfilter
	      if (
		(dest[1] >= y_m)
		&& (dest[1] <= y_M)
		&& (dest[0] >= u_m)
		&& (dest[0] <= u_M)
		&& (dest[2] >= v_m)
		&& (dest[2] <= v_M) 
	      ) {
		for (uint8_t j =0; j<=4; j++){
		if ((x>=(1.0*j*s_width*w_max))&&(x<(1.0*(j+1)*s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) {  //left sector
			cnt[j] ++;
			// UYVY
			dest[0] = 230-j*50;         // U
			dest[1] = source[1];  // Y
			dest[2] = 20+j*30;       // V
			dest[3] = source[3];  // Y 
			}
		}	
		

	      } 

	// Black colorfilter
		if (
		(source[0]>black_0min && source[0]<black_0max) &&	 
		(source[2]>black_2min && source[2]<black_2max) &&
		(source[1]>black_1min && source[1]<black_1max) &&
		(source[3]>black_3min && source[3]<black_3max) )
		{
		for (uint8_t j =0; j<=4; j++){
		if ((x>=(1.0*j*s_width*w_max))&&(x<(1.0*(j+1)*s_width*w_max))&&(y>fh_u*h_max)&&(y<(1-fh_l)*h_max)) {  //left sector
			black_cnt[j] ++;
			// UYVY
			/*dest[0] = 230-j*50;         // U
			dest[1] = source[1];  // Y
			dest[2] = 20+j*30;       // V
			dest[3] = source[3];  // Y */
			}
		}	
		

	      } 
	
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  } 
//printf("orange %d %d %d %d %d " ,cnt[0],cnt[1],cnt[2],cnt[3],cnt[4]);
// Orange Count
  *left_cnt=1.0*cnt[0]/n_pixel_color[0];
  *centrel_cnt=1.0*cnt[1]/n_pixel_color[1];
  *centrec_cnt=1.0*cnt[2]/n_pixel_color[2];
  *centrer_cnt=1.0*cnt[3]/n_pixel_color[3];
  *right_cnt=1.0*cnt[4]/n_pixel_color[4];
// Black Count
  *left_black_cnt=1.0*black_cnt[0]/n_pixel_color[0];
  *centrel_black_cnt=1.0*black_cnt[1]/n_pixel_color[1];
  *centrec_black_cnt=1.0*black_cnt[2]/n_pixel_color[2];
  *centrer_black_cnt=1.0*black_cnt[3]/n_pixel_color[3];
  *right_black_cnt=1.0*black_cnt[4]/n_pixel_color[4];
// Average Count
  *left_avgcnt=1.0*(avg_count[0])/n_pixel_avg[0];
  *centrel_avgcnt=1.0*(avg_count[1])/n_pixel_avg[1];
  *centrec_avgcnt=1.0*(avg_count[2])/n_pixel_avg[2];
  *centrer_avgcnt=1.0*(avg_count[3])/n_pixel_avg[3];
  *right_avgcnt=1.0*(avg_count[4])/n_pixel_avg[4];
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
  cccnt = 0;
  crcnt = 0; 
  rcnt = 0;

//Black Count
  black_lcnt = 0;
  black_clcnt = 0;
  black_cccnt = 0;
  black_crcnt = 0; 
  black_rcnt = 0;
  
  //Average Count
  avg_lcnt = 0;
  avg_clcnt = 0;
  avg_cccnt = 0;
  avg_crcnt = 0; 
  avg_rcnt = 0;

  // Filter
  color_count = ext_colorfilter(img,img,
      color_lum_min,color_lum_max,
      color_cb_min,color_cb_max,
      color_cr_min,color_cr_max, &lcnt, &clcnt,&cccnt, &crcnt ,&rcnt, &avg_lcnt, &avg_clcnt, &avg_cccnt, &avg_crcnt ,&avg_rcnt,     
      &black_lcnt, &black_clcnt, &black_cccnt, &black_crcnt, &black_rcnt
      );

	DOWNLINK_SEND_EXT_COLORFILTER(DefaultChannel, DefaultDevice, &color_count);
  return FALSE;
}





void ext_colorfilter_init(void) {
cv_add(colorfilter_func);
}



