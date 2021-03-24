/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_qei.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_QEI_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_QEI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Control registers */

#define LPC17_40_QEI_CON_OFFSET           0x0000 /* Control register */
#define LPC17_40_QEI_STAT_OFFSET          0x0004 /* Encoder status register */
#define LPC17_40_QEI_CONF_OFFSET          0x0008 /* Configuration register */

/* Position, index, and timer registers */

#define LPC17_40_QEI_POS_OFFSET           0x000c /* Position register */
#define LPC17_40_QEI_MAXPOS_OFFSET        0x0010 /* Maximum position register */
#define LPC17_40_QEI_CMPOS0_OFFSET        0x0014 /* Position compare register */
#define LPC17_40_QEI_CMPOS1_OFFSET        0x0018 /* Position compare register */
#define LPC17_40_QEI_CMPOS2_OFFSET        0x001c /* Position compare register */
#define LPC17_40_QEI_INXCNT_OFFSET        0x0020 /* Index count register */
#define LPC17_40_QEI_INXCMP_OFFSET        0x0024 /* Index compare register */
#define LPC17_40_QEI_LOAD_OFFSET          0x0028 /* Velocity timer reload register */
#define LPC17_40_QEI_TIME_OFFSET          0x002c /* Velocity timer register */
#define LPC17_40_QEI_VEL_OFFSET           0x0030 /* Velocity counter register */
#define LPC17_40_QEI_CAP_OFFSET           0x0034 /* Velocity capture register */
#define LPC17_40_QEI_VELCOMP_OFFSET       0x0038 /* Velocity compare register */
#define LPC17_40_QEI_FILTER_OFFSET        0x003c /* Digital filter register */

#ifdef LPC178
#  define LPC17_40_QEI_INXCMP0_OFFSET     0x0024 /* Index compare0 register */
#  define LPC17_40_QEI_INXCMP1_OFFSET     0x004c /* Index compare1 register */
#  define LPC17_40_QEI_INXCMP2_OFFSET     0x0050 /* Index compare2 register */
#  define LPC17_40_QEI_FILTER_PHA_OFFSET  0x003c /* Digital filter register */
#  define LPC17_40_QEI_FILTER_PHB_OFFSET  0x0040 /* Digital filter register */
#  define LPC17_40_QEI_FILTER_INX_OFFSET  0x0044 /* Digital filter register */
#  define LPC17_40_QEI_WINDOW_OFFSET      0x0048 /* Index acceptance register */
#endif

/* Interrupt registers */

#define LPC17_40_QEI_IEC_OFFSET           0x0fd8 /* Interrupt enable clear register */
#define LPC17_40_QEI_IES_OFFSET           0x0fdc /* Interrupt enable set register */
#define LPC17_40_QEI_INTSTAT_OFFSET       0x0fe0 /* Interrupt status register */
#define LPC17_40_QEI_IE_OFFSET            0x0fe4 /* Interrupt enable register */
#define LPC17_40_QEI_CLR_OFFSET           0x0fe8 /* Interrupt status clear register */
#define LPC17_40_QEI_SET_OFFSET           0x0fec /* Interrupt status set register */

/* Register addresses *******************************************************/

/* Control registers */

#define LPC17_40_QEI_CON                  (LPC17_40_QEI_BASE+LPC17_40_QEI_CON_OFFSET)
#define LPC17_40_QEI_STAT                 (LPC17_40_QEI_BASE+LPC17_40_QEI_STAT_OFFSET)
#define LPC17_40_QEI_CONF                 (LPC17_40_QEI_BASE+LPC17_40_QEI_CONF_OFFSET)

/* Position, index, and timer registers */

#define LPC17_40_QEI_POS                  (LPC17_40_QEI_BASE+LPC17_40_QEI_POS_OFFSET)
#define LPC17_40_QEI_MAXPOS               (LPC17_40_QEI_BASE+LPC17_40_QEI_MAXPOS_OFFSET)
#define LPC17_40_QEI_CMPOS0               (LPC17_40_QEI_BASE+LPC17_40_QEI_CMPOS0_OFFSET)
#define LPC17_40_QEI_CMPOS1               (LPC17_40_QEI_BASE+LPC17_40_QEI_CMPOS1_OFFSET)
#define LPC17_40_QEI_CMPOS2               (LPC17_40_QEI_BASE+LPC17_40_QEI_CMPOS2_OFFSET)
#define LPC17_40_QEI_INXCNT               (LPC17_40_QEI_BASE+LPC17_40_QEI_INXCNT_OFFSET)
#define LPC17_40_QEI_INXCMP               (LPC17_40_QEI_BASE+LPC17_40_QEI_INXCMP_OFFSET)
#define LPC17_40_QEI_LOAD                 (LPC17_40_QEI_BASE+LPC17_40_QEI_LOAD_OFFSET)
#define LPC17_40_QEI_TIME                 (LPC17_40_QEI_BASE+LPC17_40_QEI_TIME_OFFSET)
#define LPC17_40_QEI_VEL                  (LPC17_40_QEI_BASE+LPC17_40_QEI_VEL_OFFSET)
#define LPC17_40_QEI_CAP                  (LPC17_40_QEI_BASE+LPC17_40_QEI_CAP_OFFSET)
#define LPC17_40_QEI_VELCOMP              (LPC17_40_QEI_BASE+LPC17_40_QEI_VELCOMP_OFFSET)
#define LPC17_40_QEI_FILTER               (LPC17_40_QEI_BASE+LPC17_40_QEI_FILTER_OFFSET)

/* Interrupt registers */

#define LPC17_40_QEI_IEC                  (LPC17_40_QEI_BASE+LPC17_40_QEI_IEC_OFFSET)
#define LPC17_40_QEI_IES                  (LPC17_40_QEI_BASE+LPC17_40_QEI_IES_OFFSET)
#define LPC17_40_QEI_INTSTAT              (LPC17_40_QEI_BASE+LPC17_40_QEI_INTSTAT_OFFSET)
#define LPC17_40_QEI_IE                   (LPC17_40_QEI_BASE+LPC17_40_QEI_IE_OFFSET)
#define LPC17_40_QEI_CLR                  (LPC17_40_QEI_BASE+LPC17_40_QEI_CLR_OFFSET)
#define LPC17_40_QEI_SET                  (LPC17_40_QEI_BASE+LPC17_40_QEI_SET_OFFSET)

/* Register bit definitions *************************************************/

/* The following registers hold 32-bit integer values and have no bit fields
 * defined in this section:
 *
 *   Position register (POS)
 *   Maximum position register (MAXPOS)
 *   Position compare register 0 (CMPOS0)
 *   Position compare register 1 (CMPOS)
 *   Position compare register 2 (CMPOS2)
 *   Index count register (INXCNT)
 *   Index compare register (INXCMP)
 *   Velocity timer reload register (LOAD)
 *   Velocity timer register (TIME)
 *   Velocity counter register (VEL)
 *   Velocity capture register (CAP)
 *   Velocity compare register (VELCOMP)
 *   Digital filter register (FILTER)
 */

/* Control registers */

/* Control register */

#define QEI_CON_RESP                      (1 << 0)  /* Bit 0:  Reset position counter */
#define QEI_CON_RESPI                     (1 << 1)  /* Bit 1:  Reset position counter on index */
#define QEI_CON_RESV                      (1 << 2)  /* Bit 2:  Reset velocity */
#define QEI_CON_RESI                      (1 << 3)  /* Bit 3:  Reset index counter */
                                                    /* Bits 4-31: reserved */

/* Encoder status register */

#define QEI_STAT_DIR                      (1 << 0)  /* Bit 0:  Direction bit */
                                                    /* Bits 1-31: reserved */

/* Configuration register */

#define QEI_CONF_DIRINV                   (1 << 0)  /* Bit 0:  Direction invert */
#define QEI_CONF_SIGMODE                  (1 << 1)  /* Bit 1:  Signal Mode */
#define QEI_CONF_CAPMODE                  (1 << 2)  /* Bit 2:  Capture Mode */
#define QEI_CONF_INVINX                   (1 << 3)  /* Bit 3:  Invert Index */

#ifdef LPC178x_40xx
#  define QEI_CONF_CRESPI                 (1 << 4)  /* Bit 4:  Continuous Index reset */
                                                    /* Bits 5-15: reserved */
#  define QEI_CONF_INXGATE_SHIFT          (16)      /* Bit 16:19  Index Gating */
#  define QEI_CONF_INXGATE_MASK           (15 << QEI_CONF_INXGATE_SHIFT)
#endif
                                                    /* Bits 20-31: reserved */

/* Position, index, and timer registers
 * (all 32-bit integer values with not bit fields
 */

/* Interrupt registers */

/* Interrupt enable clear register (IEC), Interrupt enable set register
 * (IES), Interrupt status register (INTSTAT), Interrupt enable register
 * (IE), Interrupt status clear register (CLR), and Interrupt status set
 * register (SET) common bit definitions.
 */

#define QEI_INT_INX                       (1 << 0)  /* Bit 0:  Index pulse detected */
#define QEI_INT_TIM                       (1 << 1)  /* Bit 1:  Velocity timer overflow occurred */
#define QEI_INT_VELC                      (1 << 2)  /* Bit 2:  Captured velocity less than compare velocity */
#define QEI_INT_DIR                       (1 << 3)  /* Bit 3:  Change of direction detected */
#define QEI_INT_ERR                       (1 << 4)  /* Bit 4:  Encoder phase error detected */
#define QEI_INT_ENCLK                     (1 << 5)  /* Bit 5:  Eencoder clock pulse detected */
#define QEI_INT_POS0                      (1 << 6)  /* Bit 6:  Position 0 compare equal to current position */
#define QEI_INT_POS1                      (1 << 7)  /* Bit 7:  Position 1 compare equal to current position */
#define QEI_INT_POS2                      (1 << 8)  /* Bit 8:  Position 2 compare equal to current position */
#define QEI_INT_REV                       (1 << 9)  /* Bit 9:  Index compare value equal to current index count */
#define QEI_INT_POS0REV                   (1 << 10) /* Bit 10: Combined position 0 and revolution count interrupt */
#define QEI_INT_POS1REV                   (1 << 11) /* Bit 11: Position 1 and revolution count interrupt */
#define QEI_INT_POS2REV                   (1 << 12) /* Bit 12: Position 2 and revolution count interrupt */

#ifdef LPC178x_40xx
#  define QEI_INT_REV1                    (1 << 13) /* Bit 13: Index compare1 value to current index interrupt */
#  define QEI_INT_REV2                    (1 << 14) /* Bit 14: Index compare2 value to current index interrupt */
#  define QEI_INT_MAXPOS                  (1 << 15) /* Bit 15: Current position count interrupt */
#endif
                                                    /* Bits 16-31: reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_QEI_H */
