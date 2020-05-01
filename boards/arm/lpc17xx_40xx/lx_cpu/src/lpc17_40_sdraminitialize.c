/************************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_sdraminitialize.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nuttx/signal.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_emc.h"

#include "lx_cpu.h"

#if defined(CONFIG_LPC17_40_EMC) && defined(CONFIG_LPC17_40_EXTDRAM)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The core clock is LPC17_40_EMCCLK which may be either LPC17_40_CCLK* (undivided)
 * or LPC17_40_CCLK / 2 as determined by settings in the board.h header file.
 *
 * For example:
 *   LPC17_40_CCLCK      =  120,000,000
 *   EMCCLKSEL        -> LPC17_40_CCLK divided by 2
 *   LPC17_40_EMCCLK     =  60,000,000
 *   LPC17_40_EMCCLK_MHZ =  60 (Rounded to an integer)
 *   EMC_NSPERCLK     =  16.667 (Represented with 4 bits of fraction, 267)
 *
 *   EMC_NS2CLK(63)   = ((63 << 4) + 266) / 267 = 4 (actual 3.78)
 *   EMC_NS2CLK(20)   = ((20 << 4) + 266) / 267 = 2 (actual 1.20)
 */

#define LPC17_40_EMCCLK_MHZ    ((LPC17_40_EMCCLK + 500000) / 1000000)
#define EMC_NSPERCLK_B4     (((1000 << 4) + (LPC17_40_EMCCLK_MHZ >> 1)) / LPC17_40_EMCCLK_MHZ)
#define EMC_NS2CLK(ns)      (((ns << 4) + (EMC_NSPERCLK_B4 - 1)) / EMC_NSPERCLK_B4)
#define MDKCFG_RASVAL   2
#define MDKCFG_CASVAL   2

/* Set up for 32-bit SDRAM at CS0 */

#ifdef CONFIG_LPC17_40_EXTDRAMSIZE
#  define SDRAM_SIZE CONFIG_LPC17_40_EXTDRAMSIZE
#endif

#ifdef CONFIG_LPC17_40_SDRAM_16BIT
#  ifndef SDRAM_SIZE
#    define SDRAM_SIZE      0x02000000 /* 256Mbit */
#  endif
#else /* if defined(CONFIG_LPC17_40_SDRAM_32BIT) */
#  undef CONFIG_LPC17_40_SDRAM_32BIT
#  define CONFIG_LPC17_40_SDRAM_32BIT 1
#  ifndef SDRAM_SIZE
#    define SDRAM_SIZE      0x04000000 /* 512Mbit */
#  endif
#endif

#define SDRAM_BASE          0xa0000000 /* CS0 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile uint32_t lx_cpu_ringosccount[2] =
{
  0, 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name:
 *   lx_cpu_running_from_sdram
 *
 * Descriptions:
 *   Check whether currently execution from SDRAM.
 *
 * Returned value:
 *   1 running from SDRAM, otherwise 0
 *
 ****************************************************************************/

static int lx_cpu_running_from_sdram(void)
{
  uint32_t extdram_bank_size = LPC17_40_EXTDRAM_CS3 - LPC17_40_EXTDRAM_CS2;
  uint32_t extdram_end = LPC17_40_EXTDRAM_CS3 + extdram_bank_size;

  if (((uint32_t)lx_cpu_running_from_sdram >= LPC17_40_EXTDRAM_CS0) &&
      ((uint32_t)lx_cpu_running_from_sdram < extdram_end))
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/* SDRAM code based on NXP application notes and emc_sdram.c example */

/*****************************************************************************
 * Name:
 *   lx_cpu_sdram_test
 *
 * Descriptions:
 *   sdram test
 *
 * Returned value:
 *   1 if test passed, otherwise 0
 *
 ****************************************************************************/

static uint32_t lx_cpu_sdram_test(void)
{
  volatile uint32_t *wr_ptr;
  volatile uint16_t *short_wr_ptr;
  uint32_t data;
  uint32_t i;
  uint32_t j;

  wr_ptr = (uint32_t *)LPC17_40_EXTDRAM_CS0;
  short_wr_ptr = (uint16_t *)wr_ptr;

  /* 16 bit write */

  for (i = 0; i < 64; i++)
    {
      for (j = 0; j < 0x100; j++)
        {
          *short_wr_ptr++ = (i + j);
          *short_wr_ptr++ = (i + j) + 1;
        }
    }

  /* Verifying */

  wr_ptr = (uint32_t *)LPC17_40_EXTDRAM_CS0;
  for (i = 0; i < 64; i++)
    {
      for (j = 0; j < 0x100; j++)
        {
          data = *wr_ptr;
          if (data != (((((i + j) + 1) & 0xffff) << 16) | ((i + j) & 0xffff)))
            {
              return 0x0;
            }

          wr_ptr++;
        }
    }

  return 0x1;
}

/****************************************************************************
 * Name: find_cmddly
 *
 * Descriptions:
 *   find CMDDLY
 *
 * Returned value:
 *   1 if test passed, otherwise 0
 *
 ****************************************************************************/

static uint32_t lx_cpu_sdram_find_cmddly(void)
{
  uint32_t cmddly;
  uint32_t cmddlystart;
  uint32_t cmddlyend;
  uint32_t regval;
  uint32_t ppass = 0x0;
  uint32_t pass = 0x0;

  cmddly = 0x0;
  cmddlystart = cmddlyend = 0xff;

  while (cmddly < 32)
    {
      regval = getreg32(LPC17_40_SYSCON_EMCDLYCTL);
      regval &= ~SYSCON_EMCDLYCTL_CMDDLY_MASK;
      regval |= cmddly << SYSCON_EMCDLYCTL_CMDDLY_SHIFT;
      putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);

      if (lx_cpu_sdram_test() == 0x1)
        {
          /* Test passed */

          if (cmddlystart == 0xff)
            {
              cmddlystart = cmddly;
            }

          ppass = 0x1;
        }
      else
        {
          /* Test failed */

          if (ppass == 1)
            {
              cmddlyend = cmddly;
              pass = 0x1;
              ppass = 0x0;
            }
        }

      /* Try next value */

      cmddly++;
    }

  /* If the test passed, the we can use the average of the min and max
   * values to get an optimal DQSIN delay
   */

  if (pass == 0x1)
    {
      cmddly = (cmddlystart + cmddlyend) / 2;
    }
  else if (ppass == 0x1)
    {
      cmddly = (cmddlystart + 0x1f) / 2;
    }
  else
    {
      /* A working value couldn't be found, just pick something
       * safe so the system doesn't become unstable
       */
      cmddly = 0x10;
    }

  regval  = getreg32(LPC17_40_SYSCON_EMCDLYCTL);
  regval &= ~SYSCON_EMCDLYCTL_CMDDLY_MASK;
  regval |= cmddly << SYSCON_EMCDLYCTL_CMDDLY_SHIFT;
  putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);

  return (pass | ppass);
}

/****************************************************************************
 * Name: lx_cpu_sdram_find_fbclkdly
 *
 * Descriptions:
 *   find FBCLKDLY
 *
 * Returned value:
 *   1 if test passed, otherwise 0
 *
 ****************************************************************************/

static uint32_t lx_cpu_sdram_find_fbclkdly(void)
{
  uint32_t fbclkdly;
  uint32_t fbclkdlystart;
  uint32_t fbclkdlyend;
  uint32_t regval;
  uint32_t ppass = 0x0;
  uint32_t pass = 0x0;

  fbclkdly = 0x0;
  fbclkdlystart = fbclkdlyend = 0xff;

  while (fbclkdly < 32)
    {
      regval = getreg32(LPC17_40_SYSCON_EMCDLYCTL);
      regval &= ~SYSCON_EMCDLYCTL_FBCLKDLY_MASK;
      regval |= fbclkdly << SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT;
      putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);

      if (lx_cpu_sdram_test() == 0x1)
        {
          /* Test passed */

          if (fbclkdlystart == 0xff)
            {
              fbclkdlystart = fbclkdly;
            }

          ppass = 0x1;
        }
      else
        {
          /* Test failed */

          if (ppass == 1)
            {
              fbclkdlyend = fbclkdly;
              pass = 0x1;
              ppass = 0x0;
            }
        }

      /* Try next value */

      fbclkdly++;
    }

  /* If the test passed, the we can use the average of the
   * min and max values to get an optimal DQSIN delay
   */

  if (pass == 0x1)
    {
      fbclkdly = (fbclkdlystart + fbclkdlyend) / 2;
    }
  else if (ppass == 0x1)
    {
      fbclkdly = (fbclkdlystart + 0x1f) / 2;
    }
  else
    {
      /* A working value couldn't be found, just pick something
       * safe so the system doesn't become unstable
       */

      fbclkdly = 0x10;
    }

  regval  = getreg32(LPC17_40_SYSCON_EMCDLYCTL);
  regval &= ~SYSCON_EMCDLYCTL_FBCLKDLY_MASK;
  regval |= fbclkdly << SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT;
  putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);

  return (pass | ppass);
}

/****************************************************************************
 * Name: lx_cpu_sdram_calibration
 *
 * Descriptions:
 *   Calibration
 *
 * Returned value:
 *   current ring osc count
 *
 ****************************************************************************/

static uint32_t lx_cpu_sdram_calibration(void)
{
  uint32_t regval;
  uint32_t cnt = 0;
  uint32_t i;

  for (i = 0; i < 10; i++)
    {
      regval = getreg32(LPC17_40_SYSCON_EMCCAL);
      regval |= SYSCON_EMCCAL_START_MASK;
      putreg32(regval, LPC17_40_SYSCON_EMCCAL);

      regval = getreg32(LPC17_40_SYSCON_EMCCAL);
      while ((regval & SYSCON_EMCCAL_DONE_SHIFT) == 0)
        {
          regval = getreg32(LPC17_40_SYSCON_EMCCAL);
        }

      cnt += (regval & 0xff);
    }

  return (cnt / 10);
}

/****************************************************************************
 * Name: lx_cpu_sdram_adjust_timing
 *
 * Descriptions:
 *   Adjust timing
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

static void lx_cpu_sdram_adjust_timing(void)
{
  uint32_t regval;
  uint32_t cmddly;
  uint32_t fbclkdly;

  /* Current value */

  lx_cpu_ringosccount[1] = lx_cpu_sdram_calibration();

  regval     = getreg32(LPC17_40_SYSCON_EMCDLYCTL);

  cmddly     = regval & SYSCON_EMCDLYCTL_CMDDLY_MASK;
  cmddly   >>= SYSCON_EMCDLYCTL_CMDDLY_SHIFT;
  cmddly     = cmddly * lx_cpu_ringosccount[0] / lx_cpu_ringosccount[1];
  cmddly   <<= SYSCON_EMCDLYCTL_CMDDLY_SHIFT;
  cmddly    &= SYSCON_EMCDLYCTL_CMDDLY_MASK;

  fbclkdly   = regval & SYSCON_EMCDLYCTL_FBCLKDLY_MASK;
  fbclkdly >>= SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT;
  fbclkdly   = fbclkdly * lx_cpu_ringosccount[0] / lx_cpu_ringosccount[1];
  fbclkdly <<= SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT;
  fbclkdly  &= SYSCON_EMCDLYCTL_FBCLKDLY_MASK;

  regval    &= ~SYSCON_EMCDLYCTL_CMDDLY_MASK | SYSCON_EMCDLYCTL_FBCLKDLY_MASK;
  regval    |= cmddly | fbclkdly;

  putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: lpc17_40_setup_sdram
 *
 * Descriptions:
 *   Setup SDRAM
 *
 * Returned value:
 *   Return negative value if SDRAM Fails
 *
 ****************************************************************************/

void lx_cpu_sdram_initialize(void)
{
  uint32_t regval;
  uint32_t dynctl;
  uint32_t modeval;
  volatile uint32_t delay;

  if (lx_cpu_running_from_sdram())
    {
      return;
    }

  /* Memory MT48LC4M32B2P
   * 4 Meg x 32 (1 Meg x 32 x 4 banks)
   * Configuration     1 Meg x 32 x 4 banks
   * Refresh count             4K
   * Row addressing        4K  12-bit  A[11:0]
   * Bank addressing        4   2-bit  BA[1:0]
   * Column addressing    256   8-bit  A[7:0]
   */

  /* Reconfigure delays:
   *
   * CMDDLY: Programmable delay value for EMC outputs in command delayed
   *   mode.  The delay amount is roughly CMDDLY * 250 picoseconds.
   * FBCLKDLY: Programmable delay value for the feedback clock that controls
   *   input data sampling.  The delay amount is roughly (FBCLKDLY+1) * 250
   *   picoseconds.
   * CLKOUT0DLY: Programmable delay value for the CLKOUT0 output. This would
   *   typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT0DLY+1) * 250 picoseconds.
   * CLKOUT1DLY: Programmable delay value for the CLKOUT1 output. This would
   *  typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT1DLY+1) * 250 picoseconds.
   */

  regval = SYSCON_EMCDLYCTL_CMDDLY(32) |
           SYSCON_EMCDLYCTL_FBCLKDLY(32) |
           SYSCON_EMCDLYCTL_CLKOUT0DLY(1) |
           SYSCON_EMCDLYCTL_CLKOUT1DLY(1);
  putreg32(regval, LPC17_40_SYSCON_EMCDLYCTL);
  putreg32(0, LPC17_40_EMC_CONFIG);

  /* Timing for 72 MHz Bus */

  regval  = MDKCFG_CASVAL << EMC_DYNAMICRASCAS_CAS_SHIFT;
  regval |= MDKCFG_RASVAL << EMC_DYNAMICRASCAS_RAS_SHIFT;
  putreg32(regval, LPC17_40_EMC_DYNAMICRASCAS0); /* 2 RAS, 2 CAS latency */
  putreg32(1, LPC17_40_EMC_DYNAMICREADCONFIG);   /* Command delayed strategy, using EMCCLKDELAY */

                                                 /* EMC_NS2CLK(20)  TRP   = 20 nS */
  putreg32(1, LPC17_40_EMC_DYNAMICRP);           /* ( n + 1 ) -> 2 clock cycles */

  putreg32(3, LPC17_40_EMC_DYNAMICRAS);          /* ( n + 1 ) -> 4 clock cycles */

  putreg32(5, LPC17_40_EMC_DYNAMICSREX);         /* ( n + 1 ) -> 6 clock cycles */

  putreg32(2, LPC17_40_EMC_DYNAMICAPR);          /* ( n + 1 ) -> 3 clock cycles */

                                                 /* EMC_NS2CLK(20) + 2 TRP + TDPL = 20ns + 2clk */
  putreg32(3, LPC17_40_EMC_DYNAMICDAL);          /* ( n ) -> 3 clock cycles */

  putreg32(1, LPC17_40_EMC_DYNAMICWR);           /* ( n + 1 ) -> 2 clock cycles */

                                                 /* EMC_NS2CLK(63) */
  putreg32(4, LPC17_40_EMC_DYNAMICRC);           /* ( n + 1 ) -> 5 clock cycles */

                                                 /* EMC_NS2CLK(63) */
  putreg32(4, LPC17_40_EMC_DYNAMICRFC);          /* ( n + 1 ) -> 5 clock cycles */

  putreg32(5, LPC17_40_EMC_DYNAMICXSR);          /* ( n + 1 ) -> 6 clock cycles */

                                                 /* EMC_NS2CLK(63) */
  putreg32(1, LPC17_40_EMC_DYNAMICRRD);          /* ( n + 1 ) -> 2 clock cycles */

  putreg32(1, LPC17_40_EMC_STATICEXTENDEDWAIT);  /* ( n + 1 ) -> 2 clock cycles */

  dynctl = EMC_DYNAMICCONTROL_CE | EMC_DYNAMICCONTROL_CS;
  up_mdelay(100);
  regval = dynctl | EMC_DYNAMICCONTROL_I_NOP;
  putreg32(regval, LPC17_40_EMC_DYNAMICCONTROL); /* Issue NOP command */

  up_mdelay(200);                                /* wait 200ms */
  regval = dynctl | EMC_DYNAMICCONTROL_I_PALL;
  putreg32(regval, LPC17_40_EMC_DYNAMICCONTROL); /* Issue PAL command */
  putreg32(2, LPC17_40_EMC_DYNAMICREFRESH);      /* ( n * 16 ) -> 32 clock cycles */

  for (delay = 0; delay < 0x80; delay++);        /* wait 128 AHB clock cycles */

  /* Timing for 72MHz Bus */

  /* ( n * 16 ) -> 1120 clock cycles -> 15.556uS at 72MHz <= 15.625uS (64ms / 4096 row) */

  regval   = 64000000 / (1 << 12);
  regval  -= 16;
  regval >>= 4;
  regval   = regval * LPC17_40_EMCCLK_MHZ / 1000;
  putreg32(regval, LPC17_40_EMC_DYNAMICREFRESH);

  regval   = dynctl | EMC_DYNAMICCONTROL_I_MODE;
  putreg32(regval, LPC17_40_EMC_DYNAMICCONTROL); /* Issue MODE command */

  /* Timing for 48/60/72MHZ Bus */

  modeval  = LPC17_40_EXTDRAM_CS0;
  modeval |= 0x22 << (2 + 2 + 9); /* 4 burst, 2 CAS latency */
  regval   = *(volatile uint32_t *)modeval;
  putreg32(EMC_DYNAMICCONTROL_I_NORMAL, LPC17_40_EMC_DYNAMICCONTROL); /* Issue NORMAL command */

  /* [re]enable buffers */

  /* 256MB, 8Mx32, 4 banks, row=12, column=9 */

  regval  = EMC_DYNAMICCONFIG_MD_SDRAM;
  regval |= 9 << EMC_DYNAMICCONFIG_AM0_SHIFT;
  regval |= 1 * EMC_DYNAMICCONFIG_AM1;
  regval |= EMC_DYNAMICCONFIG_B;
  putreg32(regval, LPC17_40_EMC_DYNAMICCONFIG0);

  /* Nominal value */

  lx_cpu_ringosccount[0] = lx_cpu_sdram_calibration();

  if (lx_cpu_sdram_find_cmddly() == 0x0)
    {
      return;        /* fatal error */
    }

  if (lx_cpu_sdram_find_fbclkdly() == 0x0)
    {
        return;        /* fatal error */
    }

  lx_cpu_sdram_adjust_timing();
  return;
}

#endif /* CONFIG_LPC17_40_EMC && CONFIG_LPC17_40_EXTDRAM */
