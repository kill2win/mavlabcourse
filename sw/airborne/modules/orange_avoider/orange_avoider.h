/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H
#include <inttypes.h>

extern uint8_t safeToGoForwards_Orange;
extern uint8_t safeToGoForwards_Average;
extern uint8_t safeToGoForwards_Black;

extern uint8_t safeToGoForwards;
extern uint8_t stopGoingHigher;
extern int32_t incrementForAvoidance;
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);

extern float tresholdOrange_lcnt;
extern float tresholdOrange_clcnt;
extern float tresholdOrange_cccnt;
extern float tresholdOrange_crcnt;
extern float tresholdOrange_rcnt;
extern float tresholdBlack_lcnt;
extern float tresholdBlack_clcnt;
extern float tresholdBlack_cccnt;
extern float tresholdBlack_crcnt;
extern float tresholdBlack_rcnt;
extern float tresholdavg_lcnt;
extern float tresholdavg_clcnt;
extern float tresholdavg_cccnt;
extern float tresholdavg_crcnt;
extern float tresholdavg_rcnt;

extern uint8_t safe_Left;
extern uint8_t safe_LeftCentre;
extern uint8_t safe_Centre;
extern uint8_t safe_RightCentre;
extern uint8_t safe_Right;


extern uint8_t change_waypoint_random_inside_obstacle(uint8_t waypoint);
extern float checkHeight(float maxHeight);

#endif

