/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_isx012.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/isx012.h>
#include <nuttx/video/video.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_isx012_power_on
 *
 * Description:
 *   Power on ISX012
 *
 ****************************************************************************/

int board_isx012_power_on(void);

/****************************************************************************
 * Name: board_isx012_power_off
 *
 * Description:
 *   Power off ISX012
 *
 ****************************************************************************/

int board_isx012_power_off(void);

/****************************************************************************
 * Name: board_isx012_set_reset
 *
 * Description:
 *   Set reset ISX012
 *
 ****************************************************************************/

void board_isx012_set_reset(void);

/****************************************************************************
 * Name: board_isx012_release_reset
 *
 * Description:
 *   Release reset ISX012
 *
 ****************************************************************************/

void board_isx012_release_reset(void);

/****************************************************************************
 * Name: board_isx012_set_sleep
 *
 * Description:
 *   Set sleep ISX012
 *
 ****************************************************************************/

void board_isx012_set_sleep(int kind);

/****************************************************************************
 * Name: board_isx012_release_sleep
 *
 * Description:
 *   Release sleep ISX012
 *
 ****************************************************************************/

void board_isx012_release_sleep(void);

/****************************************************************************
 * Name: board_isx012_initialize
 *
 * Description:
 *   Initialize ISX012 i2c driver and register the ISX012 device.
 *
 ****************************************************************************/

struct i2c_master_s *board_isx012_initialize(void);

/****************************************************************************
 * Name: board_isx012_uninitialize
 *
 * Description:
 *   Uninitialize ISX012 i2c driver and register the ISX012 device.
 *
 ****************************************************************************/

int board_isx012_uninitialize(struct i2c_master_s *i2c);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX012_H */
