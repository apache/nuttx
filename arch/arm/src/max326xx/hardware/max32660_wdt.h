/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_wdt.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_WDT_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_WDT0_CTRL_OFFSET     0x0000  /* Watchdog Timer 0 Control Register */
#define MAX326_WDT0_RST_OFFSET      0x0004  /* Watchdog Timer 0 Reset Register */

/* Register Addresses *******************************************************/

#define MAX326_WDT0_CTRL            (MAX326_WDT0_BASE + MAX326_WDT0_CTRL_OFFSET)
#define MAX326_WDT0_RST             (MAX326_WDT0_BASE + MAX326_WDT0_RST_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* Watchdog Timer 0 Control Register */

#define WDT0_CTRL_INTPERIOD_SHIFT   (0)      /* Bit 0-3: Interrupt Period */
#define WDT0_CTRL_INTPERIOD_MASK    (15 << WDT0_CTRL_INTPERIOD_SHIFT)
#  define WDT0_CTRL_INTPERIOD(n)    ((uint32_t)(n) << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^16 x Tpclk */

#  define WDT0_CTRL_INTPERIOD_216X  (15 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^16 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_217X  (14 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^17 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_218X  (13 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^18 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_219X  (12 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^19 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_220X  (11 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^20 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_221X  (10 << WDT0_CTRL_INTPERIOD_SHIFT) /* 2^21 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_222X  (9 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^22 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_223X  (8 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^23 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_224X  (7 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^24 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_225X  (6 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^25 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_226X  (5 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^26 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_227X  (4 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^27 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_228X  (3 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^28 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_229X  (2 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^29 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_230X  (1 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^30 x Tpclk */
#  define WDT0_CTRL_INTPERIOD_231X  (0 << WDT0_CTRL_INTPERIOD_SHIFT)  /* 2^31 x Tpclk */

#define WDT0_CTRL_RSTPERIOD_SHIFT   (4)      /* Bits 4-7: Reset Period */
#define WDT0_CTRL_RSTPERIOD_MASK    (15 << WDT0_CTRL_RSTPERIOD_SHIFT)
#  define WDT0_CTRL_RSTPERIOD(n)    ((uint32_t)(n) << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^16 x Tpclk */

#  define WDT0_CTRL_RSTPERIOD_216X  (15 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^16 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_217X  (14 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^17 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_218X  (13 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^18 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_219X  (12 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^19 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_220X  (11 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^20 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_221X  (10 << WDT0_CTRL_RSTPERIOD_SHIFT) /* 2^21 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_222X  (9 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^22 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_223X  (8 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^23 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_224X  (7 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^24 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_225X  (6 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^25 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_226X  (5 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^26 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_227X  (4 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^27 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_228X  (3 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^28 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_229X  (2 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^29 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_230X  (1 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^30 x Tpclk */
#  define WDT0_CTRL_RSTPERIOD_231X  (0 << WDT0_CTRL_RSTPERIOD_SHIFT)  /* 2^31 x Tpclk */

#define WDT0_CTRL_WDTEN             (1 << 8)  /* Bit 8:  Enable */
#define WDT0_CTRL_INTFLAG           (1 << 9)  /* Bit 9:  Interrupt Flag */
#define WDT0_CTRL_INTEN             (1 << 10) /* Bit 10: Interrupt Enable */
#define WDT0_CTRL_RSTEN             (1 << 11) /* Bit 11: Reset Enable */
#define WDT0_CTRL_RSTFLAG           (1 << 31) /* Bit 31: Reset Flag */

/* Watchdog Timer 0 Reset Register */

#define WDT0_RST_SHIFT              (0)       /* Bits 0-7:  Reset command */
#define WDT0_RST_MASK               (0xff << WDT0_RST_SHIFT)
#  define WDT0_RST_SEQ1             (0xa5 << WDT0_RST_SHIFT)
#  define WDT0_RST_SEQ2             (0x5a << WDT0_RST_SHIFT)

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_WDT_H */
