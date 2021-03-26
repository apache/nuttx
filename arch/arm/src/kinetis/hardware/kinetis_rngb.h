/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_rngb.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_RNGB_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_RNGB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NRNG) && KINETIS_NRNG > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_RNG_VER_OFFSET  0x0000 /* RNGB Version ID Register */
#define KINETIS_RNG_CMD_OFFSET  0x0004 /* RNGB Command Register */
#define KINETIS_RNG_CR_OFFSET   0x0008 /* RNGB Control Register */
#define KINETIS_RNG_SR_OFFSET   0x000c /* RNGB Status Register */
#define KINETIS_RNG_ESR_OFFSET  0x0010 /* RNGB Error Status Register */
#define KINETIS_RNG_OUT_OFFSET  0x0014 /* RNGB Output FIFO */

/* Register Addresses *******************************************************/

#define KINETIS_RNG_VER         (KINETIS_RNGB_BASE+KINETIS_RNG_VER_OFFSET)
#define KINETIS_RNG_CMD         (KINETIS_RNGB_BASE+KINETIS_RNG_CMD_OFFSET)
#define KINETIS_RNG_CR          (KINETIS_RNGB_BASE+KINETIS_RNG_CR_OFFSET)
#define KINETIS_RNG_SR          (KINETIS_RNGB_BASE+KINETIS_RNG_SR_OFFSET)
#define KINETIS_RNG_ESR         (KINETIS_RNGB_BASE+KINETIS_RNG_ESR_OFFSET)
#define KINETIS_RNG_OUT         (KINETIS_RNGB_BASE+KINETIS_RNG_OUT_OFFSET)

/* Register Bit Definitions *************************************************/

/* RNGB Version ID Register */

#define RNG_VER_MINOR_SHIFT     (0)       /* Bits 0-7: Minor version number */
#define RNG_VER_MINOR_MASK      (0xff << RNG_VER_MINOR_SHIFT)
#define RNG_VER_MAJOR_SHIFT     (8)       /* Bits 8-15: Major version number */
#define RNG_VER_MAJOR_MASK      (0xff << RNG_VER_MAJOR_SHIFT)
                                          /* Bits 27�16: Reserved */
#define RNG_VER_TYPE_SHIFT      (28)      /* Bits 28-31: Random number generator type */
#define RNG_VER_TYPE_MASK       (15 << RNG_VER_TYPE_SHIFT)
#  define RNG_VER_TYPE_RNGA     (0 << RNG_VER_TYPE_SHIFT)
#  define RNG_VER_TYPE_RNGB     (1 << RNG_VER_TYPE_SHIFT)
#  define RNG_VER_TYPE_RNGC     (2 << RNG_VER_TYPE_SHIFT)

/* RNGB Command Register */

#define RNG_CMD_ST              (1 << 0)  /* Bit 0:  Self test */
#define RNG_CMD_GS              (1 << 1)  /* Bit 1:  Generate seed */
                                          /* Bits 2-3: Reserved */
#define RNG_CMD_CI              (1 << 4)  /* Bit 4:  Clear interrupt */
#define RNG_CMD_CE              (1 << 5)  /* Bit 5:  Clear error */
#define RNG_CMD_SR              (1 << 6)  /* Bit 6:  Software reset */
                                          /* Bits 7-31: Reserved */

/* RNGB Control Register */

#define RNG_CR_FUFMOD_SHIFT     (0)       /* Bits 0-1: FIFO underflow response mode */
#define RNG_CR_FUFMOD_MASK      (3 << RNG_CR_FUFMOD_SHIFT)
#  define RNG_CR_FUFMOD_ZEROS   (0 << RNG_CR_FUFMOD_SHIFT) /* Return zeros, set RNG_ESR[FUFE] */
#  define RNG_CR_FUFMOD_ERROR   (2 << RNG_CR_FUFMOD_SHIFT) /* Generate bus transfer error */
#  define RNG_CR_FUFMOD_INT     (3 << RNG_CR_FUFMOD_SHIFT) /* Generate interrupt, return zeros */

                                          /* Bits 2-3: Reserved */
#define RNG_CR_AR               (1 << 4)  /* Bit 4:  Auto-reseed */
#define RNG_CR_MASKDONE         (1 << 5)  /* Bit 5:  Mask done interrupt */
#define RNG_CR_MASKERR          (1 << 6)  /* Bit 6:  Mask error interrupt */
                                          /* Bits 7-31: Reserved */

/* RNGB Status Register */

                                          /* Bit 0: Reserved */
#define RNG_SR_BUSY             (1 << 1)  /* Bit 1:  Busy */
#define RNG_SR_SLP              (1 << 2)  /* Bit 2:  Sleep */
#define RNG_SR_RS               (1 << 3)  /* Bit 3:  Reseed needed */
#define RNG_SR_STDN             (1 << 4)  /* Bit 4:  Self test done */
#define RNG_SR_SDN              (1 << 5)  /* Bit 5:  Seed done */
#define RNG_SR_NSDN             (1 << 6)  /* Bit 6:  New seed done */
                                          /* Bit 7: Reserved */
#define RNG_SR_FIFO_LVL_SHIFT   (8)       /* Bits 8-11: FIFO level */
#define RNG_SR_FIFO_LVL_MASK    (15 << RNG_SR_FIFO_LVL_SHIFT)
#define RNG_SR_FIFO_SIZE_SHIFT  (12)      /* Bits 12-15: FIFO size */
#define RNG_SR_FIFO_SIZE_MASK   (15 << RNG_SR_FIFO_SIZE_SHIFT)
#define RNG_SR_ERR              (1 << 16) /* Bit 16: Error */
                                          /* Bits 17-20: Reserved */
#define RNG_SR_ST_PF_SHIFT      (21)      /* Bits 21-23: Self Test Pass Fail */
#define RNG_SR_ST_PF_MASK       (7 << RNG_SR_ST_PF_SHIFT)
#  define RNG_SR_ST_PF_TRNG     (4 << RNG_SR_ST_PF_SHIFT) /* TRNG self test pass/fail */
#  define RNG_SR_ST_PF_PRNG     (2 << RNG_SR_ST_PF_SHIFT) /* PRNG self test pass/fail */
#  define RNG_SR_ST_PF_RESEED   (1 << RNG_SR_ST_PF_SHIFT) /* RESEED self test pass/fail */

#define RNG_SR_STATPF_SHIFT     (24)      /* Bits 24-31: Statistics test pass fail */
#define RNG_SR_STATPF_MASK      (0xff << RNG_SR_STATPF_SHIFT)
#  define RNG_SR_STATPF_LONG    (0x80 << RNG_SR_STATPF_SHIFT) /* Long run test (>34) */
#  define RNG_SR_STATPF_LEN6    (0x40 << RNG_SR_STATPF_SHIFT) /* Length 6+ run test */
#  define RNG_SR_STATPF_LEN5    (0x20 << RNG_SR_STATPF_SHIFT) /* Length 5 run test */
#  define RNG_SR_STATPF_LEN4    (0x10 << RNG_SR_STATPF_SHIFT) /* Length 4 run test */
#  define RNG_SR_STATPF_LEN3    (0x08 << RNG_SR_STATPF_SHIFT) /* Length 3 run test */
#  define RNG_SR_STATPF_LEN2    (0x04 << RNG_SR_STATPF_SHIFT) /* Length 2 run test */
#  define RNG_SR_STATPF_LEN1    (0x02 << RNG_SR_STATPF_SHIFT) /* Length 1 run test */
#  define RNG_SR_STATPF_MONO    (0x01 << RNG_SR_STATPF_SHIFT) /* Monobit test */

/* RNGB Error Status Register */

#define RNG_ESR_LFE             (1 << 0)  /* Bit 0:  Linear feedback shift register (LFSR) error */
#define RNG_ESR_OSCE            (1 << 1)  /* Bit 1:  Oscillator error */
#define RNG_ESR_STE             (1 << 2)  /* Bit 2:  Self test error */
#define RNG_ESR_SATE            (1 << 3)  /* Bit 3:  Statistical test error */
#define RNG_ESR_FUFE            (1 << 4)  /* Bit 4:  FIFO underflow error */
                                          /* Bits 5-31: Reserved */

/* RNGB Output FIFO (32-bit random output) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* KINETIS_NRNG && KINETIS_NRNG > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_RNGB_H */
