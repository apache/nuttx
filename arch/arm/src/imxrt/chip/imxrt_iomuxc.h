/************************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "chip/imxrt105x_iomuxc.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "chip/imxrt106x_iomuxc.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Pad Mux Registers */

#define PADMUX_MUXMODE_SHIFT                  (0)       /* Bit 0-2: Software Input On Field */
#define PADMUX_MUXMODE_MASK                   (7 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT0                 (0 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT1                 (1 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT2                 (2 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT3                 (3 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT4                 (4 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT5                 (5 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT7                 (7 << PADMUX_MUXMODE_SHIFT)
#define PADMUX_SION_SHIFT                     (4)       /* Bit 4: Software Input On Field */
#  define PADMUX_SION                         (1 << PADMUX_SION_SHIFT)

/* Pad Control Registers */

#define DRIVE_HIZ                             (0)       /* HI-Z */
#define DRIVE_260OHM                          (1)       /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#define DRIVE_130OHM                          (2)       /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#define DRIVE_90OHM                           (3)       /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#define DRIVE_60OHM                           (4)       /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#define DRIVE_50OHM                           (5)       /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#define DRIVE_40OHM                           (6)       /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#define DRIVE_33OHM                           (7)       /* 20 Ohm @3.3V, 33 Ohm @1.8V */

#define SPEED_LOW                             (0)       /* Low frequency (50 MHz) */
#define SPEED_MEDIUM                          (2)       /* Medium frequency (100, MHz) */
#define SPEED_MAX                             (3)       /* Maximum frequency (200 MHz) */

#define PULL_DOWN_100K                        (0)       /* 100K Ohm Pull Down */
#define PULL_UP_47K                           (1)       /*  47K Ohm Pull Up */
#define PULL_UP_100K                          (2)       /* 100K Ohm Pull Up */
#define PULL_UP_22K                           (3)       /*  22K Ohm Pull Up */

#define PADCTL_SRE                            (1 << 0)  /* Bit 0: Slew Rate Field */
#define PADCTL_DSE_SHIFT                      (3)       /* Bits 3-5: Drive Strength Field */
#define PADCTL_DSE_MASK                       (7 << PADCTL_DSE_SHIFT)
#  define PADCTL_DSE(n)                       ((uint32_t)(n) << PADCTL_DSE_SHIFT) /* n=DRIVE_* */
#  define PADCTL_DSE_HIZ                      (0 << PADCTL_DSE_SHIFT) /* HI-Z */
#  define PADCTL_DSE_260OHM                   (1 << PADCTL_DSE_SHIFT) /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define PADCTL_DSE_130OHM                   (2 << PADCTL_DSE_SHIFT) /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define PADCTL_DSE_90OHM                    (3 << PADCTL_DSE_SHIFT) /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define PADCTL_DSE_60OHM                    (4 << PADCTL_DSE_SHIFT) /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define PADCTL_DSE_50OHM                    (5 << PADCTL_DSE_SHIFT) /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define PADCTL_DSE_40OHM                    (6 << PADCTL_DSE_SHIFT) /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define PADCTL_DSE_33OHM                    (7 << PADCTL_DSE_SHIFT) /* 20 Ohm @3.3V, 33 Ohm @1.8V */
#define PADCTL_SPEED_SHIFT                    (6)       /* Bits 6-7: Speed Field */
#define PADCTL_SPEED_MASK                     (3 << PADCTL_SPEED_SHIFT)
#  define PADCTL_SPEED(n)                     ((uint32_t)(n) << PADCTL_SPEED_SHIFT) /* n=SPEED_* */
#  define PADCTL_SPEED_LOW                    (0 << PADCTL_SPEED_SHIFT) /* Low frequency (50 MHz) */
#  define PADCTL_SPEED_MEDIUM                 (1 << PADCTL_SPEED_SHIFT) /* Medium frequency (100, 150 MHz) */
#  define PADCTL_SPEED_MAX                    (3 << PADCTL_SPEED_SHIFT) /* Maximum frequency (100, 150, 200 MHz) */
#define PADCTL_ODE                            (1 << 11) /* Bit 11: Open Drain Enable Field */
#define PADCTL_PKE                            (1 << 12) /* Bit 12: Pull / Keep Enable Field */
#define PADCTL_PUE                            (1 << 13) /* Bit 13: Pull / Keep Select Field */
#define PADCTL_PUS_SHIFT                      (14)      /* Bits 14-15: Pull Up / Down Config. Field */
#define PADCTL_PUS_MASK                       (3 << PADCTL_PUS_SHIFT)
#  define PADCTL_PUS(n)                       ((uint32_t)(n) << PADCTL_PUS_SHIFT) /* n=PULL_* */
#  define PADCTL_PUS_DOWN_100K                (0 << PADCTL_PUS_SHIFT) /* 100K Ohm Pull Down */
#  define PADCTL_PUS_UP_47K                   (1 << PADCTL_PUS_SHIFT) /* 47K Ohm Pull Up */
#  define PADCTL_PUS_UP_100K                  (2 << PADCTL_PUS_SHIFT) /* 100K Ohm Pull Up */
#  define PADCTL_PUS_UP_22K                   (3 << PADCTL_PUS_SHIFT) /*  22K Ohm Pull Up */
#define PADCTL_HYS                            (1 << 16) /* Bit 16: Hysteresis Enable Field */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H */
