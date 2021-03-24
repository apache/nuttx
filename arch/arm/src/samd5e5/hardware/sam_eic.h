/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_eic.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_EIC_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_EIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EIC register offsets *****************************************************/

#define SAM_EIC_CTRLA_OFFSET         0x0000  /* Control A register */
#define SAM_EIC_NMICTRL_OFFSET       0x0001  /* Non-maskable interrupt control register */
#define SAM_EIC_NMIFLAG_OFFSET       0x0002  /* Non-maskable interrupt flasg status and clear register */
#define SAM_EIC_SYNCBUSY_OFFSET      0x0004  /* Synchronization busy register */
#define SAM_EIC_EVCTRL_OFFSET        0x0008  /* Event control register */
#define SAM_EIC_INTENCLR_OFFSET      0x000c  /* Interrupt enable clear register */
#define SAM_EIC_INTENSET_OFFSET      0x0010  /* Interrupt enable set register */
#define SAM_EIC_INTFLAG_OFFSET       0x0014  /* Interrupt flag and status clear register */
#define SAM_EIC_ASYNCH_OFFSET        0x0018  /* External interrupt asynchronous mode register */
#define SAM_EIC_CONFIG0_OFFSET       0x001c  /* Configuration 0 register */
#define SAM_EIC_CONFIG1_OFFSET       0x0020  /* Configuration 1 register */
#define SAM_EIC_DEBOUNCEN_OFFSET     0x0030  /* Debouncer enable */
#define SAM_EIC_DPRESCALER_OFFSET    0x0034  /* Debouncer prescaler */
#define SAM_EIC_PINSTATE_OFFSET      0x0038  /* Pin state */

/* EIC register addresses ***************************************************/

#define SAM_EIC_CTRLA                (SAM_EIC_BASE + SAM_EIC_CTRLA_OFFSET)
#define SAM_EIC_NMICTRL              (SAM_EIC_BASE + SAM_EIC_NMICTRL_OFFSET)
#define SAM_EIC_NMIFLAG              (SAM_EIC_BASE + SAM_EIC_NMIFLAG_OFFSET)
#define SAM_EIC_SYNCBUSY             (SAM_EIC_BASE + SAM_EIC_SYNCBUSY_OFFSET)
#define SAM_EIC_EVCTRL               (SAM_EIC_BASE + SAM_EIC_EVCTRL_OFFSET)
#define SAM_EIC_INTENCLR             (SAM_EIC_BASE + SAM_EIC_INTENCLR_OFFSET)
#define SAM_EIC_INTENSET             (SAM_EIC_BASE + SAM_EIC_INTENSET_OFFSET)
#define SAM_EIC_INTFLAG              (SAM_EIC_BASE + SAM_EIC_INTFLAG_OFFSET)
#define SAM_EIC_ASYNCH               (SAM_EIC_BASE + SAM_EIC_ASYNCH_OFFSET)
#define SAM_EIC_CONFIG0              (SAM_EIC_BASE + SAM_EIC_CONFIG0_OFFSET)
#define SAM_EIC_CONFIG1              (SAM_EIC_BASE + SAM_EIC_CONFIG1_OFFSET)
#define SAM_EIC_DEBOUNCEN            (SAM_EIC_BASE + SAM_EIC_DEBOUNCEN_OFFSET)
#define SAM_EIC_DPRESCALER           (SAM_EIC_BASE + SAM_EIC_DPRESCALER_OFFSET)
#define SAM_EIC_PINSTATE             (SAM_EIC_BASE + SAM_EIC_PINSTATE_OFFSET)

/* EIC register bit definitions *********************************************/

/* Control A register */

#define EIC_CTRLA_SWRST              (1 << 0)        /* Bit 0:  Software reset */
#define EIC_CTRLA_ENABLE             (1 << 1)        /* Bit 1:  Enable */
#define EIC_CTRLA_CKSEL              (1 << 4)        /* Bit 4:  Clock selection */
#  define EIC_CTRLA_CKSEL_GCLK_EIC   (0)             /*   0=EIC clocked by GCLK_EIC */
#  define EIC_CTRLA_CKSEL_CLK_ULP32K EIC_CTRLA_CKSEL /*   1=EIC clocked by CLK_ULP32K */

/* Non-maskable interrupt control register */

#define EIC_NMITRCL_NMISENSE_SHIFT   (0)       /* Bits 0-2: Non-maskable interrupt sense */
#define EIC_NMITRCL_NMISENSE_MASK    (7 << EIC_NMITRCL_NMISENSE_SHIFT)
#  define EIC_NMITRCL_NMISENSE_NONE  (0 << EIC_NMITRCL_NMISENSE_SHIFT) /* No detection */
#  define EIC_NMITRCL_NMISENSE_RISE  (1 << EIC_NMITRCL_NMISENSE_SHIFT) /* Rising edge detection */
#  define EIC_NMITRCL_NMISENSE_FALL  (2 << EIC_NMITRCL_NMISENSE_SHIFT) /* Falling edge detection */
#  define EIC_NMITRCL_NMISENSE_BOTH  (3 << EIC_NMITRCL_NMISENSE_SHIFT) /* Both edge detection */
#  define EIC_NMITRCL_NMISENSE_HIGH  (4 << EIC_NMITRCL_NMISENSE_SHIFT) /* High level detection */
#  define EIC_NMITRCL_NMISENSE_LOW   (5 << EIC_NMITRCL_NMISENSE_SHIFT) /* Low level detection */
#define EIC_NMITRCL_NMIFLTEN         (1 << 3)                          /* Bit 3: Non-maskable interrupt filter enable */
#define EIC_NMITRCL_ASYNC            (1 << 4)                          /* Bit 4: Asynchronous edge detection mode */

/* Non-maskable interrupt flas status and clear register */

#define EIC_NMIFLAG_NMI              (1 << 0)  /* Non-maskable interrupt */

/* Synchronization busy register */

#define EIC_SYNCBUSY_SWRST           (1 << 0)  /* Bit 0:  Software reset syncrhonization busy */
#define EIC_SYNCBUSY_ENABLE          (1 << 1)  /* Bit 1:  Enable synchronization busy */

/* Event control, Interrupt enable clear, interrupt enable set register,
 * interrupt flag status and clear, and External interrupt asynchronous mode
 * registers.
 */

#define EIC_EXTINT_SHIFT             (0)       /* Bits 0-15: External interrupt n */
#define EIC_EXTINT_MASK              (0xffff << EIC_EXTINT_SHIFT)
#  define EIC_EXTINT(n)              (1 << (n))
#  define EIC_EXTINT_0               (1 << 0)  /* Bit 0:  External interrupt 0 */
#  define EIC_EXTINT_1               (1 << 1)  /* Bit 1:  External interrupt 1 */
#  define EIC_EXTINT_2               (1 << 2)  /* Bit 2:  External interrupt 2 */
#  define EIC_EXTINT_3               (1 << 3)  /* Bit 3:  External interrupt 3 */
#  define EIC_EXTINT_4               (1 << 4)  /* Bit 4:  External interrupt 4 */
#  define EIC_EXTINT_5               (1 << 5)  /* Bit 5:  External interrupt 5 */
#  define EIC_EXTINT_6               (1 << 6)  /* Bit 6:  External interrupt 6 */
#  define EIC_EXTINT_7               (1 << 7)  /* Bit 7:  External interrupt 7 */
#  define EIC_EXTINT_8               (1 << 8)  /* Bit 8:  External interrupt 8 */
#  define EIC_EXTINT_9               (1 << 9)  /* Bit 9:  External interrupt 9 */
#  define EIC_EXTINT_10              (1 << 10) /* Bit 10: External interrupt 10 */
#  define EIC_EXTINT_11              (1 << 11) /* Bit 11: External interrupt 11 */
#  define EIC_EXTINT_12              (1 << 12) /* Bit 12: External interrupt 12 */
#  define EIC_EXTINT_13              (1 << 13) /* Bit 13: External interrupt 13 */
#  define EIC_EXTINT_14              (1 << 14) /* Bit 14: External interrupt 14 */
#  define EIC_EXTINT_15              (1 << 15) /* Bit 15: External interrupt 15 */

#define EIC_EXTINT_ALL               EIC_EXTINT_MASK

/* Configuration 0 register */

#define EIC_CONFIG0_FILTEN(n)        (0x8 << ((n) << 2))               /* Filter n enable, n=0-7 */
#define EIC_CONFIG0_SENSE_SHIFT(n)   ((n) << 2)                        /* Filter n input sense, n=0-7 */
#define EIC_CONFIG0_SENSE_MASK(n)    (7 << EIC_CONFIG0_SENSE_SHIFT(n))
#  define EIC_CONFIG0_SENSE_NONE(n)  (0 << EIC_CONFIG0_SENSE_SHIFT(n)) /* No detection */
#  define EIC_CONFIG0_SENSE_RISE(n)  (1 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Rising edge detection */
#  define EIC_CONFIG0_SENSE_FALL(n)  (2 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Falling edge detection */
#  define EIC_CONFIG0_SENSE_BOTH(n)  (3 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Both edge detection */
#  define EIC_CONFIG0_SENSE_HIGH(n)  (4 << EIC_CONFIG0_SENSE_SHIFT(n)) /* High level detection */
#  define EIC_CONFIG0_SENSE_LOW(n)   (5 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Low level detection */

/* Configuration 1 register */

#define EIC_CONFIG1_FILTEN(n)        (0x8 << (((n) - 8) << 2))         /* Filter n enable, n=8-15 */
#define EIC_CONFIG1_SENSE_SHIFT(n)   (((n) - 8) << 2)                  /* Filter n input sense, n=8-17 */
#define EIC_CONFIG1_SENSE_MASK(n)    (7 << EIC_CONFIG1_SENSE_SHIFT(n))
#  define EIC_CONFIG1_SENSE_NONE(n)  (0 << EIC_CONFIG1_SENSE_SHIFT(n)) /* No detection */
#  define EIC_CONFIG1_SENSE_RISE(n)  (1 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Rising edge detection */
#  define EIC_CONFIG1_SENSE_FALL(n)  (2 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Falling edge detection */
#  define EIC_CONFIG1_SENSE_BOTH(n)  (3 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Both edge detection */
#  define EIC_CONFIG1_SENSE_HIGH(n)  (4 << EIC_CONFIG1_SENSE_SHIFT(n)) /* High level detection */
#  define EIC_CONFIG1_SENSE_LOW(n)   (5 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Low level detection */

/* Debouncer enable */

#define EIC_DEBOUNCEN_SHIFT             (0)       /* Bits 0-15: EXTINT n Debouncer enable */
#define EIC_DEBOUNCEN_MASK              (0xffff << EIC_DEBOUNCEN_SHIFT)
#  define EIC_DEBOUNCEN(n)              (1 << (n))
#  define EIC_DEBOUNCEN_0               (1 << 0)  /* Bit 0:  EXTINT 0 edge input is debounced */
#  define EIC_DEBOUNCEN_1               (1 << 1)  /* Bit 1:  EXTINT 1 edge input is debounced */
#  define EIC_DEBOUNCEN_2               (1 << 2)  /* Bit 2:  EXTINT 2 edge input is debounced */
#  define EIC_DEBOUNCEN_3               (1 << 3)  /* Bit 3:  EXTINT 3 edge input is debounced */
#  define EIC_DEBOUNCEN_4               (1 << 4)  /* Bit 4:  EXTINT 4 edge input is debounced */
#  define EIC_DEBOUNCEN_5               (1 << 5)  /* Bit 5:  EXTINT 5 edge input is debounced */
#  define EIC_DEBOUNCEN_6               (1 << 6)  /* Bit 6:  EXTINT 6 edge input is debounced */
#  define EIC_DEBOUNCEN_7               (1 << 7)  /* Bit 7:  EXTINT 7 edge input is debounced */
#  define EIC_DEBOUNCEN_8               (1 << 8)  /* Bit 8:  EXTINT 8 edge input is debounced */
#  define EIC_DEBOUNCEN_9               (1 << 9)  /* Bit 9:  EXTINT 9 edge input is debounced */
#  define EIC_DEBOUNCEN_10              (1 << 10) /* Bit 10: EXTINT 10 edge input is debounced */
#  define EIC_DEBOUNCEN_11              (1 << 11) /* Bit 11: EXTINT 11 edge input is debounced */
#  define EIC_DEBOUNCEN_12              (1 << 12) /* Bit 12: EXTINT 12 edge input is debounced */
#  define EIC_DEBOUNCEN_13              (1 << 13) /* Bit 13: EXTINT 13 edge input is debounced */
#  define EIC_DEBOUNCEN_14              (1 << 14) /* Bit 14: EXTINT 14 edge input is debounced */
#  define EIC_DEBOUNCEN_15              (1 << 15) /* Bit 15: EXTINT 15 edge input is debounced */

/* Debouncer prescaler */

#define EIC_DPRESCALER_PRESCALER0_SHIFT (0)       /* Bitx 0-2: Debouncer Prescaler. EXTINT 0-7 */
#define EIC_DPRESCALER_PRESCALER0_MASK  (7 << EIC_DPRESCALER_PRESCALER0_SHIFT)
#  define EIC_DPRESCALER_PRESCALER0_DIV2   (0 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 2 */
#  define EIC_DPRESCALER_PRESCALER0_DIV4   (1 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 4 */
#  define EIC_DPRESCALER_PRESCALER0_DIV8   (2 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 8 */
#  define EIC_DPRESCALER_PRESCALER0_DIV16  (3 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 16 */
#  define EIC_DPRESCALER_PRESCALER0_DIV32  (4 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 32 */
#  define EIC_DPRESCALER_PRESCALER0_DIV64  (5 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 64 */
#  define EIC_DPRESCALER_PRESCALER0_DIV128 (6 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 128 */
#  define EIC_DPRESCALER_PRESCALER0_DIV256 (7 << EIC_DPRESCALER_PRESCALER0_SHIFT) /* EIC clock divided by 256 */
#define EIC_DPRESCALER_STATES0          (1 << 3)                                  /* Bit 3:  Debouncer number of states. EXTINT 0-7 */
#  define EIC_DPRESCALER_STATES0_3      (0)                                       /* 3 low frequency samples */
#  define EIC_DPRESCALER_STATES0_7      EIC_DPRESCALER_STATES0                    /* 7 low frequency samples */
#define EIC_DPRESCALER_PRESCALER1_SHIFT (4)                                       /* Bitx 4-6: Debouncer Prescaler. EXTINT 8-15 */
#define EIC_DPRESCALER_PRESCALER1_MASK  (7 << EIC_DPRESCALER_PRESCALER1_SHIFT)
#  define EIC_DPRESCALER_PRESCALER1_DIV2   (0 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 2 */
#  define EIC_DPRESCALER_PRESCALER1_DIV4   (1 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 4 */
#  define EIC_DPRESCALER_PRESCALER1_DIV8   (2 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 8 */
#  define EIC_DPRESCALER_PRESCALER1_DIV16  (3 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 16 */
#  define EIC_DPRESCALER_PRESCALER1_DIV32  (4 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 32 */
#  define EIC_DPRESCALER_PRESCALER1_DIV64  (5 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 64 */
#  define EIC_DPRESCALER_PRESCALER1_DIV128 (6 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 128 */
#  define EIC_DPRESCALER_PRESCALER1_DIV256 (7 << EIC_DPRESCALER_PRESCALER1_SHIFT) /* EIC clock divided by 256 */
#define EIC_DPRESCALER_STATES1          (1 << 7)                                  /* Bit 7:  Debouncer number of states. EXTINT 8-15 */
#  define EIC_DPRESCALER_STATES1_3      (0)                                       /* 3 low frequency samples */
#  define EIC_DPRESCALER_STATES1_7      EIC_DPRESCALER_STATES1                    /* 7 low frequency samples */
#define EIC_DPRESCALER_TICKON           (1 << 16)                                 /* Bit 16: Pin Sampler frequency selection */
#  define EIC_DPRESCALER_TICKON_GCLKEIC (0)                                       /* Bounce sampler uses GCLK_EIC */
#  define EIC_DPRESCALER_TICKON_LFCLK   EIC_DPRESCALER_TICKON                     /* Bounce sampler uses low frequency clock */

/* Pin state */

#define EIC_PINSTATE_SHIFT              (0)       /* Bits 0-15: EXTINT n Debouncer enable */
#define EIC_PINSTATE_MASK               (0xffff << EIC_PINSTATE_SHIFT)
#  define EIC_PINSTATE(n)               (1 << (n))
#  define EIC_PINSTATE_0                (1 << 0)  /* Bit 0:  EXTINT 0 pin state */
#  define EIC_PINSTATE_1                (1 << 1)  /* Bit 1:  EXTINT 1 pin state */
#  define EIC_PINSTATE_2                (1 << 2)  /* Bit 2:  EXTINT 2 pin state */
#  define EIC_PINSTATE_3                (1 << 3)  /* Bit 3:  EXTINT 3 pin state */
#  define EIC_PINSTATE_4                (1 << 4)  /* Bit 4:  EXTINT 4 pin state */
#  define EIC_PINSTATE_5                (1 << 5)  /* Bit 5:  EXTINT 5 pin state */
#  define EIC_PINSTATE_6                (1 << 6)  /* Bit 6:  EXTINT 6 pin state */
#  define EIC_PINSTATE_7                (1 << 7)  /* Bit 7:  EXTINT 7 pin state */
#  define EIC_PINSTATE_8                (1 << 8)  /* Bit 8:  EXTINT 8 pin state */
#  define EIC_PINSTATE_9                (1 << 9)  /* Bit 9:  EXTINT 9 pin state */
#  define EIC_PINSTATE_10               (1 << 10) /* Bit 10: EXTINT 10 pin state */
#  define EIC_PINSTATE_11               (1 << 11) /* Bit 11: EXTINT 11 pin state */
#  define EIC_PINSTATE_12               (1 << 12) /* Bit 12: EXTINT 12 pin state */
#  define EIC_PINSTATE_13               (1 << 13) /* Bit 13: EXTINT 13 pin state */
#  define EIC_PINSTATE_14               (1 << 14) /* Bit 14: EXTINT 14 pin state */
#  define EIC_PINSTATE_15               (1 << 15) /* Bit 15: EXTINT 15 pin state */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_EIC_H */
