/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450-xgevk.h
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

#ifndef __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_H
#define __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int lc823450_adc_setup(void);
#endif

/****************************************************************************
 * Name: lc823450_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int lc823450_bringup(void);
#endif

/****************************************************************************
 * Name: lc823450_bma250initialize
 ****************************************************************************/

#ifdef CONFIG_BMA250
int lc823450_bma250initialize(FAR const char *devpath);
#endif

#ifdef CONFIG_AUDIO_WM8776
int lc823450_wm8776initialize(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_H */
