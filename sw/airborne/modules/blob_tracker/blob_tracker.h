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
 * @file "modules/blob_tracker/blob_tracker.h"
 * @author Michaël Ozo
 * Tracks a colored blob using integral image technique. Draws a crosshair at the blob center.
 */

#ifndef BLOB_TRACKER_H
#define BLOB_TRACKER_H

#include <stdint.h>
#include "lib/vision/image.h"

 extern void blob_tracker_init(void);
 
 uint16_t colorblob_uyvy(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *pix_x, uint16_t *pix_y, uint16_t *cp_u, uint16_t *cp_v);

#endif

