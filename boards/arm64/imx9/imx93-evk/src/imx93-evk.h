/****************************************************************************
 * boards/arm64/imx9/imx93-evk/src/imx93-evk.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM64_IMX9_IMX93_EVK_SRC_IMX93_EVK_H
#define __BOARDS_ARM64_IMX9_IMX93_EVK_SRC_IMX93_EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDSLOTNO
#  define SDIO_SLOTNO   CONFIG_NSH_MMCSDSLOTNO
#else
#  define SDIO_SLOTNO   0
#endif

#ifdef CONFIG_NSH_MMCSDMINOR
#  define SDIO_MINOR   CONFIG_NSH_MMCSDMINOR
#else
#  define SDIO_MINOR   0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imx9_bringup(void);
#endif

/****************************************************************************
 * Name: imx9_pwm_setup
 *
 * Description:
 *   Initialize PWM outputs
 *
 ****************************************************************************/

#if defined(CONFIG_PWM)
int imx9_pwm_setup(void);
#endif

/****************************************************************************
 * Name: imx9_i2c_setup
 *
 * Description:
 *   Initialize I2C devices and driver
 *
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER)
int imx9_i2c_initialize(void);
#endif

/****************************************************************************
 * Name: imx9_spi_setup
 *
 * Description:
 *   Initialize SPI devices and driver
 *
 ****************************************************************************/

#if defined(CONFIG_SPI_DRIVER)
int imx9_spi_initialize(void);
#endif

/****************************************************************************
 * Name: imx9_usdhc_init
 *
 * Description:
 *   Initialize uSDHC driver
 *
 ****************************************************************************/

#if defined(CONFIG_MMCSD)
int imx9_usdhc_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_IMX9_IMX93_EVK_SRC_IMX93_EVK_H */
