/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_emc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/clock.h>

#include "arm_internal.h"
#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_emc.h"
#include "lpc54_config.h"
#include "lpc54_enableclk.h"
#include "lpc54_reset.h"
#include "lpc54_emc.h"

#ifdef CONFIG_LPC54_EMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EMC_SDRAM_MODE_CL_SHIFT       (4)
#define EMC_SDRAM_MODE_CL_MASK        (7 << EMC_SDRAM_MODE_CL_SHIFT)

#define EMC_DYNCTL_COLUMNBASE_SHIFT   (0)
#define EMC_DYNCTL_COLUMNBASE_MASK    (3 << EMC_DYNCTL_COLUMNBASE_SHIFT)
#define EMC_DYNCTL_COLUMNPLUS_SHIFT   (3)
#define EMC_DYNCTL_COLUMNPLUS_MASK    (3 << EMC_DYNCTL_COLUMNPLUS_SHIFT)
#define EMC_DYNCTL_BUSWIDTH_MASK      (0x80)
#define EMC_DYNCTL_BUSADDRMAP_MASK    (0x20)
#define EMC_DYNCTL_DEVBANKS_BITS_MASK (0x1c)

#define EMC_SDRAM_BANKCS_BA0_MASK     (uint32_t)(0x2000)
#define EMC_SDRAM_BANKCS_BA1_MASK     (uint32_t)(0x4000)
#define EMC_SDRAM_BANKCS_BA_MASK      (EMC_SDRAM_BANKCS_BA0_MASK | EMC_SDRAM_BANKCS_BA1_MASK)

#define EMC_REFRESH_CLOCK_SCALE        16

#define EMC_SDRAM_WAIT_CYCLES          2000
#define MHZ_PER_HZ                     1000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_dram_csbase[LPC54_EMC_NCS] =
{
  LPC54_DRAMCS0_BASE, LPC54_DRAMCS1_BASE, LPC54_DRAMCS2_BASE,
  LPC54_DRAMCS3_BASE
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_emc_timercycles
 *
 * Description:
 *   Convert nanoseconds to EMC clock cycles and clip to the provided range.
 *
 * Input Parameters:
 *   nsec  - Nanoseconds to be converted.
 *   lower - Lower valid limit
 *   upper - Upper valid limit
 *
 ****************************************************************************/

static uint32_t lpc54_emc_timercycles(uint32_t nsec, uint32_t lower,
                                      uint32_t upper)
{
  uint32_t cycles;

  cycles = BOARD_EMC_FREQUENCY / MHZ_PER_HZ * nsec;
  return ((cycles + MSEC_PER_SEC - 1) / MSEC_PER_SEC);

  /* Decrease according to the plus */

  if (cycles < lower)
    {
      cycles = lower;
    }
  else if (cycles > upper)
    {
      cycles = upper;
    }

  return cycles;
}

/****************************************************************************
 * Name: lpc54_emc_timercycles
 *
 * Description:
 *   Get the shift value to shift the mode register content by.
 *
 * Input Parameters:
 *   addrmap  - EMC address map for the dynamic memory configuration.
 *              This is bit 14 ~ bit 7 of the EMC_DYNCONFIG.
 *
 * Returned Value:
 *   The offset value to shift the mode register content by.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC_DYNAMIC
static uint32_t lpc54_emc_modeoffset(uint32_t addrmap)
{
  uint8_t offset = 0;
  uint32_t columbase = addrmap & EMC_DYNCTL_COLUMNBASE_MASK;

  /* First calculate the column length. */

  if (columbase == 0x10)
    {
      offset = 8;
    }
  else
    {
      if (!columbase)
        {
          offset = 9;
        }
      else
        {
          offset = 8;
        }

      /* Add column length increase check. */

      if (((addrmap & EMC_DYNCTL_COLUMNPLUS_MASK) >>
            EMC_DYNCTL_COLUMNPLUS_SHIFT) == 1)
        {
          offset += 1;
        }
      else if (((addrmap & EMC_DYNCTL_COLUMNPLUS_MASK) >>
                 EMC_DYNCTL_COLUMNPLUS_SHIFT) == 2)
        {
          offset += 2;
        }
      else
        {
          /* To avoid MISRA rule 14.10 error. */
        }
    }

  /* Add Buswidth/16. */

  if (addrmap & EMC_DYNCTL_BUSWIDTH_MASK)
    {
      offset += 2;
    }
  else
    {
      offset += 1;
    }

  /* Add bank select bit if the sdram address map mode is RBC
   * (row-bank-column) mode.
   */

  if (!(addrmap & EMC_DYNCTL_BUSADDRMAP_MASK))
    {
      if (!(addrmap & EMC_DYNCTL_DEVBANKS_BITS_MASK))
        {
          offset += 1;
        }
      else
        {
          offset += 2;
        }
    }

  return offset;
}
#endif /* CONFIG_LPC54_EMC_DYNAMIC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_emc_initialize
 *
 * Description:
 *   This function enables the EMC clock, initializes the emc system
 *   configuration, and enable the EMC module.
 *
 * Input Parameters:
 *   config - Describes the EMC configuration.
 *
 ****************************************************************************/

void lpc54_emc_initialize(const struct emc_config_s *config)
{
  uint32_t regval;

  /* Enable EMC clock */

  lpc54_emc_enableclk();

  /* Reset the EMC */

  lpc54_reset_emc();

  /* Set the EMC system configuration */

  putreg32(SYSCON_EMCCLKDIV_DIV(config->clkdiv), LPC54_SYSCON_EMCCLKDIV);

  regval = config->clksrc ? SYSCON_EMCSYSCTRL_FBCLKINSEL : 0;
  putreg32(regval, LPC54_SYSCON_EMCSYSCTRL);

  /* Set the endian mode */

  regval = config->bigendian ? EMC_CONFIG_EM : 0;
  putreg32(regval, LPC54_EMC_CONFIG);

  /* Enable the EMC module with normal memory map mode and normal
   * work mode.
   */

  putreg32(EMC_CONTROL_E, LPC54_EMC_CONTROL);
}

/****************************************************************************
 * Name: lpc54_emc_sdram_initialize
 *
 * Description:
 *   This function initializes the dynamic memory controller in external
 *   memory controller. This function must be called after
 *   lpc54_emc_initialize and before accessing the external dynamic memory.
 *
 * Input Parameters:
 *   timing   - The timing and latency for dynamica memory controller
 *              setting. It will be used for all dynamic memory chips,
 *              therefore the worst timing value for all used chips must be
 *              given.
 *   chconfig - The EMC dynamic memory controller chip-independent
 *              by configuration array.  The dimension of the array is given
 *              nchips.
 *   nchips   - The number of chips to configure and the dimension of the
 *              chconfig array.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC_DYNAMIC
void lpc54_emc_sdram_initialize(
                        const struct emc_dynamic_timing_config_s *timing,
                        const struct emc_dynamic_chip_config_s *chconfig,
                        unsigned int nchips)
{
  const struct emc_dynamic_chip_config_s *config;
  uintptr_t addr;
  uint32_t regval;
  uint32_t offset;
  uint32_t data;
  unsigned int i;
  volatile unsigned int j;

  DEBUGASSERT(timing != NULL && chconfig != NULL && nchips > 0);

  /* Setting for dynamic memory controller chip independent configuration */

  for (i = 0, config = chconfig;
       i < nchips && config != NULL;
       i++, config++)
    {
      uint8_t caslat;

      regval = EMC_DYNCONFIG_MD(config->dyndev) |
               EMC_DYNCONFIG_ADDRMAP(config->addrmap);
      putreg32(regval, LPC54_EMC_DYNCONFIG(config->chndx));

      /* Abstract CAS latency from the SDRAM mode register setting values */

      caslat = (config->mode & EMC_SDRAM_MODE_CL_MASK) >>
                EMC_SDRAM_MODE_CL_SHIFT;
      regval = EMC_DYNRASCAS_RAS(config->rasnclk) |
               EMC_DYNRASCAS_CAS(caslat);
      putreg32(regval, LPC54_EMC_DYNRASCAS(config->chndx));
    }

  /* Configure the Dynamic Memory controller timing/latency for all chips. */

  regval = EMC_DYNREADCONFIG(timing->rdconfig);
  putreg32(regval, LPC54_EMC_DYNREADCONFIG);

  regval = lpc54_emc_timercycles(timing->rp, 1, 16);
  putreg32(EMC_DYNRP(regval), LPC54_EMC_DYNRP);

  regval = lpc54_emc_timercycles(timing->ras, 1, 16);
  putreg32(EMC_DYNRAS(regval), LPC54_EMC_DYNRAS);

  regval = lpc54_emc_timercycles(timing->srex, 1, 16);
  putreg32(EMC_DYNSREX(regval), LPC54_EMC_DYNSREX);

  regval = lpc54_emc_timercycles(timing->apr, 1, 16);
  putreg32(EMC_DYNAPR(regval), LPC54_EMC_DYNAPR);

  regval = lpc54_emc_timercycles(timing->dal, 0, 15);
  putreg32(EMC_DYNDAL(regval), LPC54_EMC_DYNDAL);

  regval = lpc54_emc_timercycles(timing->wr, 1, 16);
  putreg32(EMC_DYNWR(regval), LPC54_EMC_DYNWR);

  regval = lpc54_emc_timercycles(timing->rc, 1, 32);
  putreg32(EMC_DYNRC(regval), LPC54_EMC_DYNRC);

  regval = lpc54_emc_timercycles(timing->rfc, 1, 32);
  putreg32(EMC_DYNRFC(regval), LPC54_EMC_DYNRFC);

  regval = lpc54_emc_timercycles(timing->xsr, 1, 32);
  putreg32(EMC_DYNXSR(regval), LPC54_EMC_DYNXSR);

  regval = lpc54_emc_timercycles(timing->rrd, 1, 16);
  putreg32(EMC_DYNRRD(regval), LPC54_EMC_DYNRRD);

  regval = EMC_DYNRRD(timing->mrd);
  putreg32(regval, LPC54_EMC_DYNMRD);

  /* Initialize the SDRAM. */

  for (j = 0; j < EMC_SDRAM_WAIT_CYCLES; j++)
    {
    }

  /* Step 2. Issue NOP command. */

  regval = EMC_DYNCONTROL_CE | EMC_DYNCONTROL_CS | EMC_DYNCONTROL_I_MODE;
  putreg32(regval, LPC54_EMC_DYNCONTROL);

  for (j = 0; j < EMC_SDRAM_WAIT_CYCLES; j++)
    {
    }

  /* Step 3. Issue precharge all command. */

  regval = EMC_DYNCONTROL_CE | EMC_DYNCONTROL_CS | EMC_DYNCONTROL_I_PALL;
  putreg32(regval, LPC54_EMC_DYNCONTROL);

  /* Step 4. Issue two auto-refresh command. */

  putreg32(2 * EMC_REFRESH_CLOCK_SCALE, LPC54_EMC_DYNREFRESH);

  for (j = 0; j < EMC_SDRAM_WAIT_CYCLES / 2; j++)
    {
    }

  regval = lpc54_emc_timercycles(timing->refresh, 0,
                                 EMC_REFRESH_CLOCK_SCALE * 2047);
  putreg32(regval / EMC_REFRESH_CLOCK_SCALE, LPC54_EMC_DYNREFRESH);

  /* Step 5. Issue a mode command and set the mode value. */

  regval = EMC_DYNCONTROL_CE | EMC_DYNCONTROL_CS | EMC_DYNCONTROL_I_MODE;
  putreg32(regval, LPC54_EMC_DYNCONTROL);

  /* Calculate the mode settings here and to reach the 8 auto-refresh time
   * requirement.
   */

  for (i = 0, config = chconfig;
       i < nchips && config != NULL;
       i++, config++)
    {
      /* Get the shift value first. */

      offset = lpc54_emc_modeoffset(config->addrmap);
      addr   = g_dram_csbase[config->chndx] |
               ((uint32_t)(config->mode & ~EMC_SDRAM_BANKCS_BA_MASK) <<
                offset);

      /* Set the right mode setting value. */

      data = *(volatile uint32_t *)addr;
      data = data;
    }

  if (config->dyndev)
    {
      /* Add extended mode register if the low-power sdram is used. */

      regval = EMC_DYNCONTROL_CE | EMC_DYNCONTROL_CS |
               EMC_DYNCONTROL_I_MODE;
      putreg32(regval, LPC54_EMC_DYNCONTROL);

      /* Calculate the mode settings for extended mode register. */

      for (i = 0, config = chconfig;
           i < nchips && config != NULL;
           i++, config++)
        {
          /* Get the shift value first. */

          offset = lpc54_emc_modeoffset(config->addrmap);
          addr = (g_dram_csbase[config->chndx] |
                 (((uint32_t)(config->extmode & ~EMC_SDRAM_BANKCS_BA_MASK) |
                 EMC_SDRAM_BANKCS_BA1_MASK) << offset));

          /* Set the right mode setting value. */

          data = *(volatile uint32_t *)addr;
          data = data;
        }
    }

  /* Step 6. Issue normal operation command. */

  regval = EMC_DYNCONTROL_I_NORMAL;
  putreg32(regval, LPC54_EMC_DYNCONTROL);

  /* The buffer will be disabled when do the sdram initialization and
   * enabled after the initialization during normal operation.
   */

  for (i = 0, config = chconfig;
       i < nchips && config != NULL;
       i++, config++)
    {
      uintptr_t regaddr = LPC54_EMC_DYNCONFIG(config->chndx);

      regval  = getreg32(regaddr);
      regval |= EMC_DYNCONFIG_B;
      putreg32(regval, regaddr);
    }
}
#endif /* CONFIG_LPC54_EMC_DYNAMIC */

/****************************************************************************
 * Name: lpc54_emc_sram_initialize
 *
 * Description:
 *   This function initializes the static memory controller in external
 *   memory controller. This function must be called after
 *   lpc54_emc_initialize and before accessing the external dynamic memory.
 *
 * Input Parameters:
 *   extwait    - The extended wait timeout or the read/write transfer time.
 *                This is common for all static memory chips and set with
 *                NULL if not required.
 *   statconfig - The EMC static memory controller chip independent
 *                configuration array.  The dimension of the array is nchips.
 *   nchips     - The total static memory chip numbers been used and the
 *                number of entries in the statconfig array.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC_STATIC
void lpc54_emc_sram_initialize(uint32_t *extwait,
                    const struct emc_static_chip_config_s *statconfig,
                     uint32_t nchips)
{
  const struct emc_static_chip_config_s *config;
  uint32_t regval;
  unsigned int i;

  /* Initialize extended wait. */

  DEBUGASSERT(statconfig != NULL && nchips > 0);

  if (extwait)
    {
#ifdef CONFIG_DEBUG_ASSERTIONS
      for (i = 0, config = statconfig;
           i < nchips && config != NULL; i++,
           config++)
        {
          DEBUGASSERT(config->specconfig & EMC_ASYNCPAGEENABLE);
        }
#endif

      regval = lpc54_emc_timercycles(*extwait, 1, 1024);
      putreg32(EMC_STATEXTWAIT(regval), LPC54_EMC_STATEXTWAIT);
    }

  /* Initialize the static memory chip specific configure. */

  for (i = 0, config = statconfig;
       i < nchips && config != NULL; i++,
       config++)
    {
      regval = config->specconfig | config->memwidth;
      putreg32(regval, LPC54_EMC_STATCONFIG(config->chndx));

      regval = lpc54_emc_timercycles(config->waitwriteen, 1, 16);
      putreg32(EMC_STATWAITWEN(regval),
               LPC54_EMC_STATWAITWEN(config->chndx));

      regval = lpc54_emc_timercycles(config->waitouten, 0, 15);
      putreg32(EMC_STATWAITOEN(regval),
               LPC54_EMC_STATWAITOEN(config->chndx));

      regval = lpc54_emc_timercycles(config->waitread, 1, 32);
      putreg32(EMC_STATWAITRD(regval),
               LPC54_EMC_STATWAITRD(config->chndx));

      regval = lpc54_emc_timercycles(config->waitreadpage, 1, 32);
      putreg32(EMC_STATWAITPAGE(regval),
               LPC54_EMC_STATWAITPAGE(config->chndx));

      regval = lpc54_emc_timercycles(config->waitwrite, 2, 33);
      putreg32(EMC_STATWAITWR(regval),
               LPC54_EMC_STATWAITWR(config->chndx));

      regval = lpc54_emc_timercycles(config->waitturn, 1, 16);
      putreg32(EMC_STATWAITTURN(regval),
               LPC54_EMC_STATWAITTURN(config->chndx));
    }
}
#endif /* CONFIG_LPC54_EMC_STATIC */

#endif /* CONFIG_LPC54_EMC */
