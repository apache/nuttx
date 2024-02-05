/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_iomuxc_ver2.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER2_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#  include "hardware/rt117x/imxrt117x_iomuxc.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Defaults for drive conditions for each set of pins. These are a good
 * starting point but should be updated once you've got real hardware
 * to measure.
 */

#  define IOMUX_UART_DEFAULT              (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_LPSPI_DEFAULT             (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)

#  define IOMUX_LPI2C_DEFAULT             (IOMUX_OPENDRAIN | IOMUX_DRIVE_NORMALSTRENGTH | GPIO_SION_ENABLE)

#  define IOMUX_LCD_DEFAULT               (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)
#  define IOMUX_LCD_BL_DEFAULT            (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_LED_DEFAULT               (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_ENET_MDIO_DEFAULT         (IOMUX_OPENDRAIN | IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_ENET_MDC_DEFAULT          (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_ENET_EN_DEFAULT           (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST | GPIO_SION_ENABLE)
#  define IOMUX_ENET_RXERR_DEFAULT        (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_ENET_DATA_DEFAULT         (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST | GPIO_SION_ENABLE)
#  define IOMUX_ENET_INT_DEFAULT          (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST | GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE)
#  define IOMUX_ENET_RST_DEFAULT          (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_ENET_TX_CLK_DEFAULT       (IOMUX_PULL_DOWN | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST | GPIO_SION_ENABLE)

#  define IOMUX_ENC_DEFAULT               (0)

#  define IOMUX_CAN_DEFAULT               (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_PWM_DEFAULT               (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)

#  define IOMUX_USDHC1_DATAX_DEFAULT      (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_USDHC1_CMD_DEFAULT        (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_USDHC1_CLK_DEFAULT        (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)
#  define IOMUX_USDHC1_CD_DEFAULT         (IOMUX_PULL_UP)
#  define IOMUX_VSD_DEFAULT               (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_SW_DEFAULT                (IOMUX_PULL_UP | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST)

#  define IOMUX_GOUT_DEFAULT              (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)

#  define IOMUX_USBOTG_ID_DEFAULT         (IOMUX_PULL_UP)
#  define IOMUX_USBOTG_PWR_DEFAULT        (IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_SLOW)
#  define IOMUX_USBOTG_OC_DEFAULT         (IOMUX_PULL_UP)

#  define IOMUX_ADC_DEFAULT               (0)

#  define IOMUX_FLEXSPI_DEFAULT           (IOMUX_PULL_NONE | IOMUX_DRIVE_HIGHSTRENGTH)

#  define IOMUX_FLEXSPI_CLK_DEFAULT       (IOMUX_PULL_NONE | IOMUX_DRIVE_HIGHSTRENGTH | GPIO_SION_ENABLE)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER2_H */
