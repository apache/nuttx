/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_nandflash.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_periphclks.h"
#include "sam4s_nand.h"
#include "hardware/sam_smc.h"
#include "hardware/sam4s_pinmap.h"
#include "hardware/sam_matrix.h"

#include "sam4s-xplained-pro.h"

#ifdef  HAVE_NAND

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

static const gpio_pinset_t g_nandpins[] =
{
  GPIO_SMC_NCS0, GPIO_SMC_NANDALE, GPIO_SMC_NANDCLE,
  GPIO_SMC_NANDOE, GPIO_SMC_NANDWE, GPIO_SMC_RB,

  GPIO_SMC_D0,   GPIO_SMC_D1,  GPIO_SMC_D2,  GPIO_SMC_D3,
  GPIO_SMC_D4,   GPIO_SMC_D5,  GPIO_SMC_D6,  GPIO_SMC_D7
};

#define NAND_NPINS (sizeof(g_nandpins) / sizeof(gpio_pinset_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_nandflash_config
 *
 * Description:
 *   If CONFIG_SAM34_EXTNAND is defined, then NAND FLASH support is
 *   enabled.  This function provides the board-specific implementation of
 *   the logic to reprogram the SMC to support NAND FLASH on the specified
 *   CS.
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

int board_nandflash_config(int cs)
{
  /* The Embest and Ronetix CM boards and one Hynix NAND HY27UF(08/16)2G2B
   * Series NAND (MT29F2G08ABAEAWP).
   * This part has a capacity of 256Mx8bit () with spare 8Mx8 bit capacity.
   * The device contains 2048 blocks, composed by 64 x 2112 byte pages.
   * The effective size is approximately 256MiB.
   *
   * NAND is available on NCS0.
   */

  int i;

  /* Configure GPIO pins (leaving SRAM in the disabled state) */

  for (i = 0; i < NAND_NPINS; i++)
    {
      sam_configgpio(g_nandpins[i]);
    }

  /* SMC NAND Flash Chip Select Configuration Register */

  putreg32(MATRIX_CCFG_SMCNFCS_SMC_NFCS(cs), SAM_MATRIX_CCFG_SMCNFCS);

  /* below from sam4s-xplained */

  sam_smc_enableclk();

  /* Configure SMC setup timing */

  putreg32(SMCCS_SETUP_NWESETUP(3) | SMCCS_SETUP_NCSWRSETUP(1) |
           SMCCS_SETUP_NRDSETUP(2) | SMCCS_SETUP_NCSRDSETUP(1),
           SAM_SMCCS_SETUP(cs));

  /* Configure the SMC pulse timing */

  putreg32(SMCCS_PULSE_NWEPULSE(5) | SMCCS_PULSE_NCSWRPULSE(5) |
           SMCCS_PULSE_NRDPULSE(5) | SMCCS_PULSE_NCSRDPULSE(5),
           SAM_SMCCS_PULSE(cs));

  /* Configure the SMC cycle timing */

  /**
   * Select 0. Chip Select 0 has been programmed with:
   * NRD_HOLD = 4; READ_MODE = 1 (NRD controlled)
   * NWE_SETUP = 3; WRITE_MODE = 1 (NWE controlled)
   * TDF_CYCLES = 6; TDF_MODE = 1 (optimization enabled).
   */

  putreg32(SMCCS_CYCLE_NWECYCLE(12) | SMCCS_CYCLE_NRDCYCLE(11),
           SAM_SMCCS_CYCLE(cs));

  /* Configure the SMC mode */

  /**
   *
   * READ_MODE:
   *    0: The read operation is controlled by the NCS signal.
   *    1: The read operation is controlled by the NRD signal.
   *
   **/

  putreg32(SMCCS_MODE_TDFCYCLES(6) | SMCCS_MODE_TDFMODE |
           SMCCS_MODE_WRITEMODE | SMCCS_MODE_READMODE,
           SAM_SMCCS_MODE(cs));

  /* Configure NAND PIO pins
   *
   * NAND Interface  NAND     DESC
   *
   *   NCS0           CE    - Dedicated pin; no configuration needed
   *   NANDCLE        CLE   - Dedicated pin; no configuration needed
   *   NANDALE        ALE   - Dedicated pin; no configuration needed
   *   NANDOE         RE    - Dedicated pin; no configuration needed
   *   NANDWE         WE    - Dedicated pin; no configuration needed
   *   NAND_RB        RB    - PC13
   *   IO_D0-7        IO0-7 - Dedicated pins; no configuration needed
   */

  sam_configgpio(GPIO_SMC_NANDALE);
  sam_configgpio(GPIO_SMC_NANDCLE);

  return OK;
}

/****************************************************************************
 * Name: sam_nand_automount
 *
 * Description:
 *   Initialize and configure the NAND on CS3
 *
 ****************************************************************************/

int sam_nand_automount(int minor)
{
  struct mtd_dev_s *mtd;
  static bool initialized = false;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Create and initialize an NAND MATD device */

      mtd = sam_nand_initialize(SAM_SMC_CS0);

      if (!mtd)
        {
          ferr("ERROR: Failed to create the NAND driver on CS%d\n",
                  SAM_SMC_CS0);
          return -ENODEV;
        }

#if defined(CONFIG_SAM34_NAND_FTL)
      /* Use the FTL layer to wrap the MTD driver as a block driver */

      int ret = OK;
      ret = ftl_initialize(NAND_MINOR, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAM34_NAND_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      int ret = OK;
      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/nand */

      ret = nx_mount(NULL, "/mnt/nand", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_NAND */
