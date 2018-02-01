/****************************************************************************
 * configs/sama5d4-ek/src/sam_nandflash.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Most of this file derives from Atmel sample code for the SAMA5D4-EK
 * board.  That sample code has licensing that is compatible with the NuttX
 * modified BSD license:
 *
 *   Copyright (c) 2012, Atmel Corporation
 *   All rights reserved.
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
 * 3. Neither the name NuttX nor Atmel nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

#include "up_arch.h"
#include "sam_periphclks.h"
#include "sam_pio.h"
#include "sam_nand.h"
#include "chip/sam_hsmc.h"
#include "chip/sam_pinmap.h"

#include "sama5d4-ek.h"

#ifdef CONFIG_SAMA5_EBICS3_NAND

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 *   If CONFIG_SAMA5_EBICS3_NAND is defined, then NAND FLASH support is
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
  uint32_t regval;

  /* The Embest and Ronetix CM boards and one Hynix NAND HY27UF(08/16)2G2B
   * Series NAND (MT29F2G08ABAEAWP).  This part has a capacity of 256Mx8bit
   * () with spare 8Mx8 bit capacity. The device contains 2048 blocks, composed
   * by 64 x 2112 byte pages.  The effective size is approximately 256MiB.
   *
   * NAND is available on CS3.
   */

  if (cs == HSMC_CS3)
    {
      /* Make sure that the SMC peripheral is enabled. */

      sam_hsmc_enableclk();

      /* Configure the SMC */

      regval = HSMC_SETUP_NWE_SETUP(1) |  HSMC_SETUP_NCS_WRSETUP(1) |
               HSMC_SETUP_NRD_SETUP(2) | HSMC_SETUP_NCS_RDSETUP(1);
      putreg32(regval, SAM_HSMC_SETUP(HSMC_CS3));

      regval = HSMC_PULSE_NWE_PULSE(5) | HSMC_PULSE_NCS_WRPULSE(7) |
               HSMC_PULSE_NRD_PULSE(5) | HSMC_PULSE_NCS_RDPULSE(7);
      putreg32(regval, SAM_HSMC_PULSE(HSMC_CS3));

      regval = HSMC_CYCLE_NWE_CYCLE(8) | HSMC_CYCLE_NRD_CYCLE(9);
      putreg32(regval, SAM_HSMC_CYCLE(HSMC_CS3));

      regval = HSMC_TIMINGS_TCLR(3) | HSMC_TIMINGS_TADL(10) |
               HSMC_TIMINGS_TAR(3) | HSMC_TIMINGS_TRR(4) |
               HSMC_TIMINGS_TWB(5) | HSMC_TIMINGS_RBNSEL(3) |
               HSMC_TIMINGS_NFSEL;
      putreg32(regval, SAM_HSMC_TIMINGS(HSMC_CS3));

      regval = HSMC_MODE_READMODE | HSMC_MODE_WRITEMODE |
               HSMC_MODE_BIT_8 | HSMC_MODE_TDFCYCLES(1);
      putreg32(regval, SAM_HSMC_MODE(HSMC_CS3));

      /* Configure NAND PIO pins
       *
       * NAND Interface:
       *
       *   NCS3/NANDCE - Dedicated pin; no configuration needed
       *   NANDCLE     - PE21
       *   NANDALE     - PE22
       *   NRD/NANDOE  - Dedicated pin; no configuration needed
       *   NWE/NANDWE  - Dedicated pin; no configuration needed
       *   NANDRDY     - Dedicated pin; no configuration needed
       *   M_EBI_D0-7  - Dedicated pins; no configuration needed
       */

      sam_configpio(PIO_HSMC_NANDALE);
      sam_configpio(PIO_HSMC_NANDCLE);

      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: sam_nand_automount
 *
 * Description:
 *   Initialize and configure the NAND on CS3
 *
 ****************************************************************************/

#ifdef HAVE_NAND
int sam_nand_automount(int minor)
{
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Create and initialize an NAND MATD device */

      mtd = sam_nand_initialize(HSMC_CS3);
      if (!mtd)
        {
          ferr("ERROR: Failed to create the NAND driver on CS%d\n", HSMC_CS3);
          return -ENODEV;
        }

#if defined(CONFIG_SAMA5D4EK_NAND_FTL)
      /* Use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(NAND_MINOR, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAMA5D4EK_NAND_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/nand */

      ret = mount(NULL, "/mnt/nand", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
          return ret;
        }
#endif
      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
#endif

#endif /* CONFIG_SAMA5_EBICS3_NAND */
