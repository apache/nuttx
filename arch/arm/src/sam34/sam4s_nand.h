/****************************************************************************
 * arch/arm/src/sam34/sam4s_nand.h
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM4S_NAND_H
#define __ARCH_ARM_SRC_SAM34_SAM4S_NAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/mtd/nand_raw.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "chip.h"
#include "sam_gpio.h"
#include "hardware/sam_smc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only NCS0 can support NAND.  The rest is a fantasy */

#if defined(CONFIG_SAM34_EXTNAND)
  #  define CONFIG_SAM34_NCS0_NAND 1
#else
  # undef CONFIG_SAM34_NCS0_NAND
#endif

/* On-Die ECC, Requires Micron Flash to support on-die */

#define SAM34_NCS0_ECCTYPE NANDECC_NONE

#if defined(CONFIG_SAM34_NCS0_NAND)
#  if defined(CONFIG_SAM34_NCS0_ECCNONE) ||\
      defined(CONFIG_MTD_NAND_EMBEDDEDECC)
#    define SAM34_NCS0_ECCTYPE NANDECC_NONE
#  elif defined(CONFIG_MTD_NAND_SWECC)
#    define SAM34_NCS0_ECCTYPE NANDECC_SWECC
#  endif
#endif /* CONFIG_SAM34_NCS0_NAND */

/* Count the number of banks that configured for NAND with PMECC support
 * enabled.
 */

#ifdef CONFIG_SAM34_NCS0_NAND
#  define CONFIG_SAM34_HAVE_NAND 1
#  define NAND_HAVE_NCS0 1
#else
#  define NAND_HAVE_NCS0 0
#endif

#define MAX_READ_STATUS_COUNT    100000 /* Read status timeout */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This type represents the state of a raw NAND MTD device on a single chip
 * select.  The struct nand_raw_s must appear at the beginning of the
 * definition so that you can freely cast between pointers to struct
 * nand_raw_s and struct sam_nandcs_s.
 *
 * NOTE: Currently, only SAM4S CS0 can support NAND.  The logic here would
 * support NAND on any CS, but that capability is not needed.
 */

struct sam_nandcs_s
{
  struct nand_raw_s raw;     /* Externally visible part of the driver */

  /* Static configuration */

  uint8_t cs;                /* Chip select number (0..3) */
  gpio_pinset_t rb;          /* NAND Ready/Busy detect GPIO pin */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* NAND global state */

EXTERN struct sam_nand_s g_nand;

/****************************************************************************
 * Name: sam_nand_initialize
 *
 * Description:
 *   Create and initialize an raw NAND device instance.  This driver
 *   implements the RAW NAND interface:  No software ECC or sparing is
 *   performed here.  Those necessary NAND features are provided by common,
 *   higher level NAND MTD layers found in drivers/mtd.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
 *   On success a non-NULL pointer to an MTD device structure is returned;
 *   NULL is returned on a failure.
 *
 ****************************************************************************/

struct mtd_dev_s;
struct mtd_dev_s *sam_nand_initialize(int cs);

/****************************************************************************
 * Name: board_nandflash_config
 *
 * Description:
 *   If CONFIG_SAM34_BOOT_CS3FLASH is defined, then NAND FLASH support is
 *   enabled.  This function provides the board-specific implementation of
 *   the logic to reprogram the SMC to support NAND FLASH on the specified
 *   CS.  As a minimum, this board-specific initialization should do the
 *   following:
 *
 *     1. Enable clocking to the HSMC
 *     2. Configure timing for the HSMC CS
 *     3. Configure NAND PIO pins
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
 *   OK if the HSMC was successfully configured for this CS.  A negated
 *   errno value is returned on a failure.  This would fail with -ENODEV,
 *   for example, if the board does not support NAND FLASH on the requested
 *   CS.
 *
 ****************************************************************************/

int board_nandflash_config(int cs);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM34_SAM4S_NAND_H */
