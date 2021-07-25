/****************************************************************************
 * include/nuttx/analog/ioctl.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_IOCTL_H
#define __INCLUDE_NUTTX_ANALOG_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The analog driver sub-system uses the standard character driver framework.
 * However, since the driver is a devices control interface rather than a
 * data transfer interface, the majority of the functionality is implemented
 * in driver ioctl calls.  Standard ioctl commands are listed below:
 */

/* DAC/ADC */

#define ANIOC_TRIGGER       _ANIOC(0x0001)  /* Trigger one conversion
                                             * IN: None
                                             * OUT: None */
#define ANIOC_WDOG_UPPER    _ANIOC(0x0002)  /* Set upper threshold for
                                             * watchdog
                                             * IN: Threshold value
                                             * OUT: None */
#define ANIOC_WDOG_LOWER    _ANIOC(0x0003)  /* Set lower threshold for
                                             * watchdog
                                             * IN: Threshold value
                                             * OUT: None */
#define ANIOC_GET_NCHANNELS _ANIOC(0x0004)  /* Get the number of
                                             * configured channels
                                             * IN: None
                                             * OUT: Number of channels */

#define AN_FIRST          0x0001          /* First common command */
#define AN_NCMDS          4               /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half driver to the lower-half driver via the ioctl()
 * method of the lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/analog/ads1242.h */

#define AN_ADS2142_FIRST  (AN_FIRST + AN_NCMDS)
#define AN_ADS2142_NCMDS  6

/* See include/nuttx/analog/lm92001.h */

#define AN_LMP92001_FIRST (AN_ADS2142_FIRST + AN_ADS2142_NCMDS)
#define AN_LMP92001_NCMDS 7

/* See include/nuttx/analog/ads7828.h */

#define AN_ADS7828_FIRST (AN_LMP92001_FIRST + AN_LMP92001_NCMDS)
#define AN_ADS7828_NCMDS 6

/* See arch/arm/src/stm32l4/stm32l4_adc.h */

#define AN_STM32L4_FIRST (AN_ADS7828_FIRST + AN_ADS7828_NCMDS)
#define AN_STM32L4_NCMDS 2

/* See include/nuttx/analog/max1161x.h */

#define AN_MAX1161X_FIRST (AN_STM32L4_FIRST + AN_STM32L4_NCMDS)
#define AN_MAX1161X_NCMDS 8

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_IOCTL_H */
