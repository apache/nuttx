/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_usb_analog.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_ANALOG_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_ANALOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_OFFSET       0x01a0  /* USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_SET_OFFSET   0x01a4  /* USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_CLR_OFFSET   0x01a8  /* USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_TOG_OFFSET   0x01ac  /* USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_OFFSET       0x01b0  /* USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_SET_OFFSET   0x01b4  /* USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_CLR_OFFSET   0x01b8  /* USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_TOG_OFFSET   0x01bc  /* USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT_OFFSET  0x01c0  /* USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_STAT_OFFSET  0x01d0  /* USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_MISC_OFFSET              0x01f0  /* USB Misc Register */
#define IMXRT_USB_ANALOG_USB1_MISC_SET_OFFSET          0x01f4  /* USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB1_MISC_CLR_OFFSET          0x01f8  /* USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB1_MISC_TOG_OFFSET          0x01fc  /* USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_OFFSET       0x0200  /* USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_SET_OFFSET   0x0204  /* USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_CLR_OFFSET   0x0208  /* USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_TOG_OFFSET   0x020c  /* USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_OFFSET       0x0210  /* USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_SET_OFFSET   0x0214  /* USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_CLR_OFFSET   0x0218  /* USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_TOG_OFFSET   0x021c  /* USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_STAT_OFFSET  0x0220  /* USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_STAT_OFFSET  0x0230  /* USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_MISC_OFFSET              0x0250  /* USB Misc Register */
#define IMXRT_USB_ANALOG_USB2_MISC_SET_OFFSET          0x0254  /* USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB2_MISC_CLR_OFFSET          0x0258  /* USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB2_MISC_TOG_OFFSET          0x025c  /* USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_DIGPROG_OFFSET                0x0260  /* Chip Silicon Version */

/* Register addresses *******************************************************/

/* Analog USB1 Register Addresses */

#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_OFFSET)       /* USB_ANALOG1 USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_SET_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_CLR_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_TOG_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_OFFSET)       /* USB_ANALOG1 USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_SET_OFFSET)   /* USB_ANALOG1 USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_CLR_OFFSET)   /* USB_ANALOG1 USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_TOG_OFFSET)   /* USB_ANALOG1 USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT_OFFSET)  /* USB_ANALOG1 USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_STAT_OFFSET)  /* USB_ANALOG1 USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_MISC                     (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_OFFSET)              /* USB_ANALOG1 USB Misc Register */
#define IMXRT_USB_ANALOG_USB1_MISC_SET                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_SET_OFFSET)          /* USB_ANALOG1 USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB1_MISC_CLR                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_CLR_OFFSET)          /* USB_ANALOG1 USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB1_MISC_TOG                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_TOG_OFFSET)          /* USB_ANALOG1 USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_OFFSET)       /* USB_ANALOG1 USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_SET_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_CLR_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_TOG_OFFSET)   /* USB_ANALOG1 USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_OFFSET)       /* USB_ANALOG1 USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_SET_OFFSET)   /* USB_ANALOG1 USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_CLR_OFFSET)   /* USB_ANALOG1 USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_TOG_OFFSET)   /* USB_ANALOG1 USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_STAT_OFFSET)  /* USB_ANALOG1 USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_STAT_OFFSET)  /* USB_ANALOG1 USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_MISC                     (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_OFFSET)              /* USB_ANALOG1 USB Misc Register */
#define IMXRT_USB_ANALOG_USB2_MISC_SET                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_SET_OFFSET)          /* USB_ANALOG1 USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB2_MISC_CLR                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_CLR_OFFSET)          /* USB_ANALOG1 USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB2_MISC_TOG                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_TOG_OFFSET)          /* USB_ANALOG1 USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_DIGPROG                       (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_DIGPROG_OFFSET)                /* USB_ANALOG1 Chip Silicon Version */

/* Analog USB2 Register Addresses */

#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_OFFSET)       /* USB_ANALOG2 USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_SET_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_CLR_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_TOG_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_OFFSET)       /* USB_ANALOG2 USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_SET_OFFSET)   /* USB_ANALOG2 USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_CLR_OFFSET)   /* USB_ANALOG2 USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_TOG_OFFSET)   /* USB_ANALOG2 USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT_OFFSET)  /* USB_ANALOG2 USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_CHRG_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_CHRG_DETECT_STAT_OFFSET)  /* USB_ANALOG2 USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB1_MISC                     (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_OFFSET)              /* USB_ANALOG2 USB Misc Register */
#define IMXRT_USB_ANALOG_USB1_MISC_SET                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_SET_OFFSET)          /* USB_ANALOG2 USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB1_MISC_CLR                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_CLR_OFFSET)          /* USB_ANALOG2 USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB1_MISC_TOG                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB1_MISC_TOG_OFFSET)          /* USB_ANALOG2 USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_OFFSET)       /* USB_ANALOG2 USB VBUS Detect Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_SET_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_CLR_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_TOG_OFFSET)   /* USB_ANALOG2 USB VBUS Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT              (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_OFFSET)       /* USB_ANALOG2 USB Charger Detect Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_SET          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_SET_OFFSET)   /* USB_ANALOG2 USB Charger Detect Set Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_CLR          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_CLR_OFFSET)   /* USB_ANALOG2 USB Charger Detect Clear Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_TOG          (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_TOG_OFFSET)   /* USB_ANALOG2 USB Charger Detect Toggle Register */
#define IMXRT_USB_ANALOG_USB2_VBUS_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_VBUS_DETECT_STAT_OFFSET)  /* USB_ANALOG2 USB VBUS Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_CHRG_DETECT_STAT         (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_CHRG_DETECT_STAT_OFFSET)  /* USB_ANALOG2 USB Charger Detect Status Register */
#define IMXRT_USB_ANALOG_USB2_MISC                     (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_OFFSET)              /* USB_ANALOG2 USB Misc Register */
#define IMXRT_USB_ANALOG_USB2_MISC_SET                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_SET_OFFSET)          /* USB_ANALOG2 USB Misc Set Register */
#define IMXRT_USB_ANALOG_USB2_MISC_CLR                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_CLR_OFFSET)          /* USB_ANALOG2 USB Misc Clear Register */
#define IMXRT_USB_ANALOG_USB2_MISC_TOG                 (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_USB2_MISC_TOG_OFFSET)          /* USB_ANALOG2 USB Misc Toggle Register */
#define IMXRT_USB_ANALOG_DIGPROG                       (IMXRT_ANATOP_BASE + IMXRT_USB_ANALOG_DIGPROG_OFFSET)                /* USB_ANALOG2 Chip Silicon Version */

/* Register Bit Definitions *************************************************/

/* USB VBUS Detect Register */

#define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT  (0)        /* Bits: 0-2  Set the threshold for the VBUSVALID comparator. */
#define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_MASK   (7 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH(n)   ((uint32_t)(n) << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V0  (0 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V0 — 4.0V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V1  (1 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V1 — 4.1V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V2  (2 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V2 — 4.2V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V3  (3 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V3 — 4.3V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V4  (4 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V4 — 4.4V (default) */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V5  (5 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V5 — 4.5V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V6  (6 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V6 — 4.6V           */
#  define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_4V7  (7 << USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_THRESH_SHIFT)  /* 4V7 — 4.7V           */

                                                  /* Bits: 3-19  Reserved */

#define USB_ANALOG_USB_VBUS_DETECT_VBUSVALID_PWRUP_CMPS    (1 << 20)  /* Bit: 20 Powers up comparators for vbus_valid detector. */
                                                                      /* Bits: 21-25  Reserved */
#define USB_ANALOG_USB_VBUS_DETECT_DISCHARGE_VBUS          (1 << 26)  /* Bit: 26 USB OTG discharge VBUS. */
#define USB_ANALOG_USB_VBUS_DETECT_CHARGE_VBUS             (1 << 27)  /* Bit: 27 USB OTG charge VBUS. */
                                                                      /* Bits: 28-31  Reserved */

/* USB Charger Detect Register */

                                                  /* Bits: 0-17  Reserved */

#define USB_ANALOG_USB_CHRG_DETECT_CHK_CONTACT             (1 << 18)  /* Bit: 18 Check the contact of USB plug */
#define USB_ANALOG_USB_CHRG_DETECT_CHK_CHRG_B              (1 << 19)  /* Bit: 19 Check the charger connection */
#define USB_ANALOG_USB_CHRG_DETECT_EN_B                    (1 << 20)  /* Bit: 20 Control the charger detector. */
                                                                      /* Bits: 21-22  Reserved */
                                                                      /* Bit: 23  Reserved */
                                                                      /* Bits: 24-31  Reserved */

/* USB VBUS Detect Status Register */

#define USB_ANALOG_USB_VBUS_DETECT_STAT_SESSEND            (1 << 0)   /* Bit: 0  Session End for USB OTG. */
#define USB_ANALOG_USB_VBUS_DETECT_STAT_BVALID             (1 << 1)   /* Bit: 1  Indicates VBus is valid for a B-peripheral. */
#define USB_ANALOG_USB_VBUS_DETECT_STAT_AVALID             (1 << 2)   /* Bit: 2  Indicates VBus is valid for a A-peripheral. */
#define USB_ANALOG_USB_VBUS_DETECT_STAT_VBUS_VALID         (1 << 3)   /* Bit: 3  VBus valid for USB OTG. */
#if defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#define USB_ANALOG_USB_VBUS_DETECT_STAT_VBUS_3V_VALID      (1 << 4)   /* Bit: 4  VBUS_VALID_3V detector status */
                                                                      /* Bits: 5-31  Reserved */
#else
                                                                      /* Bits: 4-31  Reserved */
#endif

/* USB Charger Detect Status Register */

#define USB_ANALOG_USB_CHRG_DETECT_STAT_PLUG_CONTACT       (1 << 0)   /* Bit: 0  State of the USB plug contact detector. */
#define USB_ANALOG_USB_CHRG_DETECT_STAT_CHRG_              (1 << 1)   /* Bit: 1  DETECTED */
#define USB_ANALOG_USB_CHRG_DETECT_STAT_DM_STATE           (1 << 2)   /* Bit: 2  DM line state output of the charger detector. */
#define USB_ANALOG_USB_CHRG_DETECT_STAT_DP_STATE           (1 << 3)   /* Bit: 3  DP line state output of the charger detector. */
                                                                      /* Bits: 4-31  Reserved */

/* USB Misc Register */

#define USB_ANALOG_USB_MISC_HS_USE_EXTERNAL_R              (1 << 0)   /* Bit: 0  Use external resistor to generate the current bias for the high speed transmitter. */
#define USB_ANALOG_USB_MISC_EN_DEGLITCH                    (1 << 1)   /* Bit: 1  Enable the deglitching circuit of the USB PLL output. */
                                                                      /* Bits: 2-29  Reserved */
#define USB_ANALOG_USB_MISC_EN_CLK_UTMI                    (1 << 30)  /* Bit: 30 Enables the clk to the UTMI block. */
                                                                      /* Bit: 31  Reserved */

/* Chip Silicon Version */

#define USB_ANALOG_DIGPROG_SILICON_REVISION                0x006C0000 /* Silicon revision 1.0 */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_USB_ANALOG_H */
