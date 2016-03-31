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
extern float tresholdOrange_crcnt;
extern float tresholdOrange_rcnt;
extern float tresholdavg_lcnt;
extern float tresholdavg_clcnt;
extern float tresholdavg_crcnt;
extern float tresholdavg_rcnt;


extern uint8_t change_waypoint_random_inside_obstacle(uint8_t waypoint);
extern float checkHeight(float maxHeight);

#endif

