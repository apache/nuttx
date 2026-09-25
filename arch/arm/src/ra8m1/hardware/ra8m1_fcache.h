/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_fcache.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_FCACHE_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_FCACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_FCACHE_FCACHEE_OFFSET             0x0000  /* Flash Cache Enable Register (16-bits) */
#define R_FCACHE_FCACHEIV_OFFSET            0x0004  /* Flash Cache Invalidate Register (16-bits) */
#define R_FCACHE_FLWT_OFFSET                0x001c  /* Flash Wait Cycle Register (8-bits) */
#define R_FCACHE_FSAR_OFFSET                0x0040  /* Flash Security Attribution Register (16-bits) */

/* Register Addresses *******************************************************/

/* FCACHE Registers */

#define R_FCACHE_FCACHEE                   (R_FCACHE_BASE + R_FCACHE_FCACHEE_OFFSET)
#define R_FCACHE_FCACHEIV                  (R_FCACHE_BASE + R_FCACHE_FCACHEIV_OFFSET)
#define R_FCACHE_FLWT                      (R_FCACHE_BASE + R_FCACHE_FLWT_OFFSET)
#define R_FCACHE_FSAR                      (R_FCACHE_BASE + R_FCACHE_FSAR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Cache Enable Register (16-bits) ************************************/

#define R_FCACHE_FCACHEE_FCACHEEN (1 <<  0)  /* 01: Flash Cache Enable */

/* Flash Cache Invalidate Register (16-bits) ********************************/

#define R_FCACHE_FCACHEIV_FCACHEIV (1 <<  0)  /* 01: Flash Cache Invalidate */

/* Flash Wait Cycle Register (8-bits) ***************************************/

#define R_FCACHE_FLWT_FLWT_SHIFT (0)
#define R_FCACHE_FLWT_FLWT_MASK (0x7)
#  define R_FCACHE_FLWT_FLWT_V0_WAIT_ICLK_48MHZ (0 << R_FCACHE_FLWT_FLWT_SHIFT)  /* 0 wait (ICLK <= 48MHz) */
#  define R_FCACHE_FLWT_FLWT_V001 (1 << R_FCACHE_FLWT_FLWT_SHIFT)                /* 1 wait (48MHz < ICLK <= 96MHz) */
#  define R_FCACHE_FLWT_FLWT_V010 (2 << R_FCACHE_FLWT_FLWT_SHIFT)                /* 2 waits (96MHz < ICLK <= 144MHz) */
#  define R_FCACHE_FLWT_FLWT_V011 (3 << R_FCACHE_FLWT_FLWT_SHIFT)                /* 3 waits (144Hz < ICLK <= 192MHz) */
#  define R_FCACHE_FLWT_FLWT_V100 (4 << R_FCACHE_FLWT_FLWT_SHIFT)                /* 4 waits (192Hz < ICLK <= 240MHz) */

/* Flash Security Attribution Register (16-bits) ****************************/

#define R_FCACHE_FSAR_FLWTSA (1 <<  0)      /* 01: FLWT Security Attribution */
#define R_FCACHE_FSAR_FCACHEENSA (1 <<  1)  /* 02: FCHACHEEN Security Attribution */
#define R_FCACHE_FSAR_FCKMHZSA (1 <<  8)    /* 100: FCKMHZ Security Attribution */
#define R_FCACHE_FSAR_FACICOMISA (1 <<  9)  /* 200: FACI command Issuing Security Attribution */
#define R_FCACHE_FSAR_FACICOMRSA (1 << 10)  /* 400: FACI command Registers Security Attribution */
#define R_FCACHE_FSAR_FACITRSA (1 << 11)    /* 800: FACI transfer Security Attribution */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_FCACHE_H */
