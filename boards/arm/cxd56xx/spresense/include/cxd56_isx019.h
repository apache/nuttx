/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_isx019.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX019_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX019_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/isx019.h>
#include <nuttx/video/video.h>

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
 * Name: board_isx019_power_on
 *
 * Description:
 *   Power on ISX019
 *
 ****************************************************************************/

int board_isx019_power_on(void);

/****************************************************************************
 * Name: board_isx019_power_off
 *
 * Description:
 *   Power off ISX019
 *
 ****************************************************************************/

int board_isx019_power_off(void);

/****************************************************************************
 * Name: board_isx019_set_reset
 *
 * Description:
 *   Set reset ISX019
 *
 ****************************************************************************/

void board_isx019_set_reset(void);

/****************************************************************************
 * Name: board_isx019_release_reset
 *
 * Description:
 *   Release reset ISX019
 *
 ****************************************************************************/

void board_isx019_release_reset(void);

/****************************************************************************
 * Name: board_isx019_initialize
 *
 * Description:
 *   Initialize ISX019 i2c driver and register the ISX019 device.
 *
 ****************************************************************************/

struct i2c_master_s *board_isx019_initialize(void);

/****************************************************************************
 * Name: board_isx019_uninitialize
 *
 * Description:
 *   Uninitialize ISX019 i2c driver and register the ISX019 device.
 *
 ****************************************************************************/

int board_isx019_uninitialize(struct i2c_master_s *i2c);

/****************************************************************************
 * Name: board_isx019_get_master_clock
 *
 * Description:
 *   Get ISX019 master clock.
 *
 ****************************************************************************/

uint32_t board_isx019_get_master_clock(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ISX019_H */
