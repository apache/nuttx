/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_iomuxc_ver1.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER1_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_iomuxc.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_iomuxc.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_iomuxc.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pad Mux Registers */

#define PADMUX_MUXMODE_SHIFT              (0)       /* Bit 0-2:    MUX Mode Select Field */
#define PADMUX_MUXMODE_MASK               (0x07 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT0             (0 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT1             (1 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT2             (2 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT3             (3 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT4             (4 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT5             (5 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT6             (6 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT7             (7 << PADMUX_MUXMODE_SHIFT)
#define PADMUX_SION_SHIFT                 (4)       /* Bit 4:      Software Input On Field */
#  define PADMUX_SION                     (1 << PADMUX_SION_SHIFT)

/* Pad Control Registers */

#define DRIVE_HIZ                         (0)       /* HI-Z */
#define DRIVE_260OHM                      (1)       /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#define DRIVE_130OHM                      (2)       /*  75 Ohm @3.3V, 130 Ohm @1.8V */
#define DRIVE_90OHM                       (3)       /*  50 Ohm @3.3V,  90 Ohm @1.8V */
#define DRIVE_60OHM                       (4)       /*  37 Ohm @3.3V,  60 Ohm @1.8V */
#define DRIVE_50OHM                       (5)       /*  30 Ohm @3.3V,  50 Ohm @1.8V */
#define DRIVE_40OHM                       (6)       /*  25 Ohm @3.3V,  40 Ohm @1.8V */
#define DRIVE_33OHM                       (7)       /*  20 Ohm @3.3V,  33 Ohm @1.8V */

#define SPEED_LOW                         (0)       /* Low frequency (50 MHz) */
#define SPEED_MEDIUM                      (2)       /* Medium frequency (100 MHz) */
#define SPEED_MAX                         (3)       /* Maximum frequency (200 MHz) */

#define PULL_DOWN_100K                    (0)       /* 100K Ohm Pull Down */
#define PULL_UP_47K                       (1)       /*  47K Ohm Pull Up */
#define PULL_UP_100K                      (2)       /* 100K Ohm Pull Up */
#define PULL_UP_22K                       (3)       /*  22K Ohm Pull Up */

#define PADCTL_SRE                        (1 << 0)  /* Bit 0:      Slew Rate Field (SRE) */
                                                    /* Bits 1-2:   Reserved */
#define PADCTL_DSE_SHIFT                  (3)       /* Bits 3-5:   Drive Strength Field (DSE) */
#define PADCTL_DSE_MASK                   (0x07 << PADCTL_DSE_SHIFT)
#  define PADCTL_DSE(n)                   ((uint32_t)(n) << PADCTL_DSE_SHIFT) /* n=DRIVE_* */
#  define PADCTL_DSE_HIZ                  (0x00 << PADCTL_DSE_SHIFT)          /* HI-Z */
#  define PADCTL_DSE_260OHM               (0x01 << PADCTL_DSE_SHIFT)          /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define PADCTL_DSE_130OHM               (0x02 << PADCTL_DSE_SHIFT)          /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define PADCTL_DSE_90OHM                (0x03 << PADCTL_DSE_SHIFT)          /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define PADCTL_DSE_60OHM                (0x04 << PADCTL_DSE_SHIFT)          /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define PADCTL_DSE_50OHM                (0x05 << PADCTL_DSE_SHIFT)          /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define PADCTL_DSE_40OHM                (0x06 << PADCTL_DSE_SHIFT)          /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define PADCTL_DSE_33OHM                (0x07 << PADCTL_DSE_SHIFT)          /* 20 Ohm @3.3V, 33 Ohm @1.8V */

#define PADCTL_SPEED_SHIFT                (6)       /* Bits 6-7:   Speed Field (SPEED) */
#define PADCTL_SPEED_MASK                 (0x03 << PADCTL_SPEED_SHIFT)
#  define PADCTL_SPEED(n)                 ((uint32_t)(n) << PADCTL_SPEED_SHIFT) /* n=SPEED_* */
#  define PADCTL_SPEED_LOW                (0x00 << PADCTL_SPEED_SHIFT)          /* Low frequency (50 MHz) */
#  define PADCTL_SPEED_MEDIUM             (0x01 << PADCTL_SPEED_SHIFT)          /* Medium frequency (100, 150 MHz) */
#  define PADCTL_SPEED_MAX                (0x03 << PADCTL_SPEED_SHIFT)          /* Maximum frequency (100, 150, 200 MHz) */

                                                    /* Bits 8-10:  Reserved */
#define PADCTL_ODE                        (1 << 11) /* Bit 11:     Open Drain Enable Field (ODE) */
#define PADCTL_PKE                        (1 << 12) /* Bit 12:     Pull / Keep Enable Field (PKE) */
#define PADCTL_PUE                        (1 << 13) /* Bit 13:     Pull / Keep Select Field (PUE) */
#define PADCTL_PUS_SHIFT                  (14)      /* Bits 14-15: Pull Up / Down Config. Field (PUS) */
#define PADCTL_PUS_MASK                   (0x03 << PADCTL_PUS_SHIFT)
#  define PADCTL_PUS(n)                   ((uint32_t)(n) << PADCTL_PUS_SHIFT) /* n=PULL_* */
#  define PADCTL_PUS_DOWN_100K            (0x00 << PADCTL_PUS_SHIFT)          /* 100K Ohm Pull Down */
#  define PADCTL_PUS_UP_47K               (0x01 << PADCTL_PUS_SHIFT)          /* 47K Ohm Pull Up */
#  define PADCTL_PUS_UP_100K              (0x02 << PADCTL_PUS_SHIFT)          /* 100K Ohm Pull Up */
#  define PADCTL_PUS_UP_22K               (0x03 << PADCTL_PUS_SHIFT)          /*  22K Ohm Pull Up */

#define PADCTL_HYS                        (1 << 16) /* Bit 16:     Hysteresis Enable Field (HYS) */
                                                    /* Bits 17-31: Reserved */

/* Defaults for drive conditions for each set of pins. These are a good
 * starting point but should be updated once you've got real hardware
 * to measure.
 */

#define IOMUX_UART_DEFAULT                (IOMUX_PULL_UP_22K | IOMUX_DRIVE_40OHM | \
                                           IOMUX_SLEW_SLOW | IOMUX_SPEED_LOW | IOMUX_SCHMITT_TRIGGER)

#define IOMUX_LPSPI_DEFAULT               (IOMUX_PULL_UP_100K | IOMUX_DRIVE_40OHM | \
                                           IOMUX_SLEW_FAST | IOMUX_SPEED_MEDIUM)

#define IOMUX_LPI2C_DEFAULT               (GPIO_SION_ENABLE | IOMUX_OPENDRAIN | \
                                           IOMUX_SPEED_MEDIUM | IOMUX_DRIVE_33OHM)

#define IOMUX_LCD_DEFAULT                 (IOMUX_PULL_UP_100K | IOMUX_DRIVE_40OHM | \
                                           IOMUX_SLEW_SLOW | IOMUX_SPEED_MEDIUM | IOMUX_SCHMITT_TRIGGER)
#define IOMUX_LCD_BL_DEFAULT              (IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)

#define IOMUX_LED_DEFAULT                 (IOMUX_SLEW_SLOW | IOMUX_DRIVE_50OHM | IOMUX_SPEED_LOW)

#define IOMUX_ENET_MDIO_DEFAULT           (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_OPENDRAIN | \
                                           IOMUX_SPEED_LOW | IOMUX_PULL_UP_100K)
#define IOMUX_ENET_MDC_DEFAULT            (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MAX | \
                                           IOMUX_PULL_UP_100K)
#define IOMUX_ENET_EN_DEFAULT             (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MAX | \
                                           IOMUX_PULL_UP_100K)
#define IOMUX_ENET_RXERR_DEFAULT          (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MAX | \
                                           IOMUX_PULL_UP_100K)
#define IOMUX_ENET_DATA_DEFAULT           (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MAX | \
                                           IOMUX_PULL_UP_100K)
#define IOMUX_ENET_INT_DEFAULT            (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | IOMUX_SLEW_FAST | \
                                           IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K)
#define IOMUX_ENET_RST_DEFAULT            (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | \
                                           IOMUX_PULL_UP_100K)
#define IOMUX_ENET_TX_CLK_DEFAULT         (IOMUX_SLEW_FAST | IOMUX_DRIVE_40OHM | IOMUX_SPEED_LOW | \
                                           IOMUX_PULL_DOWN_100K | IOMUX_PULL_KEEP | GPIO_SION_ENABLE)

#define IOMUX_ENC_DEFAULT                 (0)

#define IOMUX_CAN_DEFAULT                 (IOMUX_SLEW_SLOW | IOMUX_DRIVE_50OHM | IOMUX_SPEED_LOW)

#define IOMUX_PWM_DEFAULT                 (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_LOW)

#define IOMUX_USDHC1_DATAX_DEFAULT        (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | \
                                           IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define IOMUX_USDHC1_CMD_DEFAULT          (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | \
                                           IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define IOMUX_USDHC1_CLK_DEFAULT          (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | IOMUX_SPEED_MAX)
#define IOMUX_USDHC1_CD_DEFAULT           (IOMUX_PULL_UP_100K)

#define IOMUX_VSD_DEFAULT                 (IOMUX_SLEW_SLOW | IOMUX_DRIVE_90OHM | IOMUX_SPEED_MEDIUM)

#define IOMUX_SW_DEFAULT                  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                                           IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K)

#define IOMUX_GOUT_DEFAULT                (IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)

#define IOMUX_USBOTG_ID_DEFAULT           (IOMUX_PULL_UP_100K)
#define IOMUX_USBOTG_PWR_DEFAULT          (IOMUX_SLEW_SLOW | IOMUX_DRIVE_50OHM | IOMUX_SPEED_LOW )
#define IOMUX_USBOTG_OC_DEFAULT           (IOMUX_PULL_UP_100K)

#define IOMUX_ADC_DEFAULT                 (0)

#define IOMUX_FLEXSPI_DEFAULT             (IOMUX_SLEW_FAST | IOMUX_DRIVE_40OHM | IOMUX_SPEED_MAX | \
                                           IOMUX_PULL_DOWN_100K | IOMUX_PULL_KEEP | GPIO_SION_ENABLE)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_IOMUXC_VER1_H */
