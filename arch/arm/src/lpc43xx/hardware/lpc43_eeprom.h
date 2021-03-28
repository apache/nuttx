/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_eeprom.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EEPROM_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EEPROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* EEPROM registers */

#define LPC43_EEPROM_CMD_OFFSET        0x000 /* EEPROM command register */
#define LPC43_EEPROM_RWSTATE_OFFSET    0x008 /* EEPROM read wait state register */
#define LPC43_EEPROM_AUTOPROG_OFFSET   0x00c /* EEPROM auto programming register */
#define LPC43_EEPROM_WSTATE_OFFSET     0x010 /* EEPROM wait state register */
#define LPC43_EEPROM_CLKDIV_OFFSET     0x014 /* EEPROM clock divider register */
#define LPC43_EEPROM_PWRDWN_OFFSET     0x018 /* EEPROM power-down register */

/* EEPROM interrupt registers */

#define LPC43_EEPROM_INTENCLR_OFFSET   0xfd8 /* EEPROM interrupt enable clear */
#define LPC43_EEPROM_INTENSET_OFFSET   0xfdc /* EEPROM interrupt enable set */
#define LPC43_EEPROM_INTSTAT_OFFSET    0xfe0 /* EEPROM interrupt status */
#define LPC43_EEPROM_INTEN_OFFSET      0xfe4 /* EEPROM interrupt enable */
#define LPC43_EEPROM_INTSTATCLR_OFFSET 0xfe8 /* EEPROM interrupt status clear */
#define LPC43_EEPROM_INTSTATSET_OFFSET 0xfec /* EEPROM interrupt status set */

/* Register Addresses *******************************************************/

/* EEPROM registers */

#define LPC43_EEPROM_CMD               (LPC43_EEPROMC_BASE+LPC43_EEPROM_CMD_OFFSET)
#define LPC43_EEPROM_RWSTATE           (LPC43_EEPROMC_BASE+LPC43_EEPROM_RWSTATE_OFFSET)
#define LPC43_EEPROM_AUTOPROG          (LPC43_EEPROMC_BASE+LPC43_EEPROM_AUTOPROG_OFFSET)
#define LPC43_EEPROM_WSTATE            (LPC43_EEPROMC_BASE+LPC43_EEPROM_WSTATE_OFFSET)
#define LPC43_EEPROM_CLKDIV            (LPC43_EEPROMC_BASE+LPC43_EEPROM_CLKDIV_OFFSET)
#define LPC43_EEPROM_PWRDWN            (LPC43_EEPROMC_BASE+LPC43_EEPROM_PWRDWN_OFFSET)

/* EEPROM interrupt registers */

#define LPC43_EEPROM_INTENCLR          (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTENCLR_OFFSET)
#define LPC43_EEPROM_INTENSET          (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTENSET_OFFSET)
#define LPC43_EEPROM_INTSTAT           (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTSTAT_OFFSET)
#define LPC43_EEPROM_INTEN             (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTEN_OFFSET)
#define LPC43_EEPROM_INTSTATCLR        (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTSTATCLR_OFFSET)
#define LPC43_EEPROM_INTSTATSET        (LPC43_EEPROMC_BASE+LPC43_EEPROM_INTSTATSET_OFFSET)

/* Register Bit Definitions *************************************************/

/* EEPROM registers */

/* EEPROM command register */

#define EEPROM_CMD_SHIFT               (0)      /* Bits 0-2: Command */
#define EEPROM_CMD_MASK                (7 << EEPROM_CMD_SHIFT)
#  define EEPROM_CMD_PROGRAM           6        /* 110=erase/program page */
                                                /* Bits 3-31: Reserved */

/* EEPROM read wait state register */

#define EEPROM_RWSTATE_RPHASE2_SHIFT   (0)       /* Bits 0-7: Wait states 2 (minus 1) */
#define EEPROM_RWSTATE_RPHASE2_MASK    (0xff << EEPROM_RWSTATE_RPHASE2_SHIFT)
#  define EEPROM_RWSTATE_RPHASE2(n)    (((n)-1) << EEPROM_RWSTATE_RPHASE2_SHIFT)
#define EEPROM_RWSTATE_RPHASE1_SHIFT   (8)       /* Bits 8-15: Wait states 1 (minus 1) */
#define EEPROM_RWSTATE_RPHASE1_MASK    (0xff << EEPROM_RWSTATE_RPHASE1_SHIFT)
#  define EEPROM_RWSTATE_RPHASE1(n)    (((n)-1) << EEPROM_RWSTATE_RPHASE1_SHIFT)
                                                /* Bits 16-31: Reserved */

/* EEPROM auto programming register */

#define EEPROM_AUTOPROG_SHIFT          (0)       /* Bits 0-1: Auto programming mode */
#define EEPROM_AUTOPROG_MASK           (3 << EEPROM_AUTOPROG_SHIFT)
#  define EEPROM_AUTOPROG_OFF          (0 << EEPROM_AUTOPROG_SHIFT) /* auto programming off */
#  define EEPROM_AUTOPROG_FIRST        (1 << EEPROM_AUTOPROG_SHIFT) /* erase/program cycle triggered by first word */
#  define EEPROM_AUTOPROG_LAST         (2 << EEPROM_AUTOPROG_SHIFT) /* erase/program cycle triggered by last word */

                                                /* Bits 2-31: Reserved */

/* EEPROM wait state register */

#define EEPROM_WSTATE_PHASE3_SHIFT     (0)       /* Bits 0-7: Wait states for phase 3 (minus 1) */
#define EEPROM_WSTATE_PHASE3_MASK      (0xff << EEPROM_WSTATE_PHASE3_SHIFT)
#define EEPROM_WSTATE_PHASE2_SHIFT     (8)       /* Bits 8-15: Wait states for phase 2 (minus 1) */
#define EEPROM_WSTATE_PHASE2_MASK      (0xff << EEPROM_WSTATE_PHASE2_SHIFT)
#define EEPROM_WSTATE_PHASE1_SHIFT     (16)      /* Bits 16-23: Wait states for phase 1 (minus 1) */
#define EEPROM_WSTATE_PHASE1_MASK      (0xff << EEPROM_WSTATE_PHASE1_SHIFT)
                                                 /* Bits 24-30: Reserved */
#define EEPROM_WSTATE_LCK_PARWEP       (1 << 31) /* Bit 31: Lock for write, erase and program */

/* EEPROM clock divider register */

#define EEPROM_CLKDIV_MASK             (0xffff)  /* Bits 0-15: Division factor (minus 1) */
#define EEPROM_CLKDIV(n)               ((n)-1)   /* Bits 0-15: Division factor (minus 1) */
                                                 /* Bits 16-31: Reserved */

/* EEPROM power-down register */

#define EEPROM_PWRDWN                  (1 << 0)  /* Bit 0:  Power down mode bit */
                                                 /* Bits 1-31: Reserved */

/* EEPROM interrupt registers */

/* EEPROM interrupt enable clear */

/* EEPROM interrupt enable set */

/* EEPROM interrupt status */

/* EEPROM interrupt enable */

/* EEPROM interrupt status clear */

/* EEPROM interrupt status set */

                                                /* Bits 0-1: Reserved */
#define EEPROM_INT_ENDOFPROG           (1 << 2) /* Bit 2:  Program operation finished interrupt */
                                                /* Bits 3-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EEPROM_H */
