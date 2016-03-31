/*
 * Copyright (C) charles
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ext_colorfilter/ext_colorfilter.h"
 * @author charles
 * detect ext_colorfilter
 */

#ifndef EXT_COLORFILTER_H
#define EXT_COLORFILTER_H

#include <stdint.h>

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern float floor_0;
extern float floor_1;
extern float floor_2;
extern float floor_3;
extern float floor_tol;
extern float floor_tol2;


extern int color_count;

extern float fh_u;  // upper free space
extern float fh_l;  // lower free space
extern float avg_tol;  //Tolerance Factor for determing average pixels!
extern float centre_width; //Set width of combined middle sectors 


extern float lcnt;  //leftcount
extern float clcnt; //leftcentrecount
extern float crcnt; //rightcentrecount
extern float rcnt; //rightcount

extern float avg_lcnt;  //leftcount
extern float avg_clcnt; //leftcentrecount
extern float avg_crcnt; //rightcentrecount
extern float avg_rcnt; //rightcount

extern void ext_colorfilter_init(void);

#endif

