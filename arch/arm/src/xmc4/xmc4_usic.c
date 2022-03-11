/****************************************************************************
 * arch/arm/src/xmc4/xmc4_usic.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 * May include some logic from sample code provided by Infineon:
 *
 * Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <arch/xmc4/chip.h>

#include "arm_internal.h"
#include "hardware/xmc4_usic.h"
#include "hardware/xmc4_scu.h"
#include "xmc4_clockconfig.h"
#include "xmc4_usic.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Provides mapping of USIC enumeration value to USIC channel base address */

static uintptr_t g_channel_baseaddress[2 * XMC4_NUSIC] =
{
  XMC4_USIC0_CH0_BASE,
  XMC4_USIC0_CH1_BASE
#if XMC4_NUSIC > 1
  ,
  XMC4_USIC1_CH0_BASE,
  XMC4_USIC1_CH1_BASE
#if XMC4_NUSIC > 2
  ,
  XMC4_USIC2_CH0_BASE,
  XMC4_USIC2_CH1_BASE
#if XMC4_NUSIC > 3
#  error Extend table values for addition USICs
#endif
#endif
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_enable_usic
 *
 * Description:
 *   Enable the USIC module indicated by the 'usic' enumeration value
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_enable_usic(enum usic_e usic)
{
  switch (usic)
    {
      case USIC0:
#ifdef XMC4_SCU_GATING
        /* Check if USIC0 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT0) & SCU_CGAT0_USIC0) == 0)
          {
            /* Ungate USIC0 clocking */

            putreg32(SCU_CGAT0_USIC0, XMC4_SCU_CGATCLR0);

            /* Set bit in PRCLR0 to de-assert USIC0 peripheral reset */

            putreg32(SCU_PR0_USIC0RS, XMC4_SCU_PRCLR0);
          }
#else
        /* Set bit in PRCLR0 to de-assert USIC0 peripheral reset */

        putreg32(SCU_PR0_USIC0RS, XMC4_SCU_PRCLR0);
#endif
        break;

#if XMC4_NUSIC > 1
      case USIC1:
#ifdef XMC4_SCU_GATING
        /* Check if USIC1 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT1) & SCU_CGAT1_USIC1) == 0)
          {
            /* Ungate USIC1 clocking */

            putreg32(SCU_CGAT1_USIC1, XMC4_SCU_CGATCLR1);

            /* Set bit in PRCLR1 to de-assert USIC1 peripheral reset */

            putreg32(SCU_PR1_USIC1RS, XMC4_SCU_PRCLR1);
          }
#else
        /* Set bit in PRCLR1 to de-assert USIC1 peripheral reset */

        putreg32(SCU_PR1_USIC1RS, XMC4_SCU_PRCLR1);
#endif
        break;

#if XMC4_NUSIC > 2
      case USIC2:
#ifdef XMC4_SCU_GATING
        /* Check if USIC2 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT1) & SCU_CGAT1_USIC2) == 0)
          {
            /* Ungate USIC2 clocking */

            putreg32(SCU_CGAT1_USIC2, XMC4_SCU_CGATCLR1);

            /* Set bit in PRCLR1 to de-assert USIC2 peripheral reset */

            putreg32(SCU_PR1_USIC2RS, XMC4_SCU_PRCLR1);
          }
#else
        /* Set bit in PRCLR1 to de-assert USIC2 peripheral reset */

        putreg32(SCU_PR1_USIC2RS, XMC4_SCU_PRCLR1);
#endif
        break;

#endif /* XMC4_NUSIC > 2 */
#endif /* XMC4_NUSIC > 1 */

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: xmc4_disable_usic
 *
 * Description:
 *   Disable the USIC module indicated by the 'usic' enumeration value
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_disable_usic(enum usic_e usic)
{
  switch (usic)
    {
      case USIC0:

        /* Set bit in PRSET0 to assert USIC0 peripheral reset */

        putreg32(SCU_PR0_USIC0RS, XMC4_SCU_PRSET0);

#ifdef XMC4_SCU_GATING
        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT0_USIC0, XMC4_SCU_CGATSET0);
#endif
        break;

#if XMC4_NUSIC > 1
      case USIC1:

        /* Set bit in PRSET1 to assert USIC1 peripheral reset */

        putreg32(SCU_PR1_USIC1RS, XMC4_SCU_PRSET1);

#ifdef XMC4_SCU_GATING
        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT1_USIC1, XMC4_SCU_CGATSET1);
#endif
        break;

#if XMC4_NUSIC > 2
      case USIC2:

        /* Set bit in PRSET1 to assert USIC2 peripheral reset */

        putreg32(SCU_PR1_USIC2RS, XMC4_SCU_PRSET1);

#ifdef XMC4_SCU_GATING
        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT1_USIC2, XMC4_SCU_CGATSET1);
#endif
        break;

#endif /* XMC4_NUSIC > 2 */
#endif /* XMC4_NUSIC > 1 */

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: xmc4_channel_baseaddress
 *
 * Description:
 *   Given a USIC channel enumeration value, return the base address of the
 *   channel registers.
 *
 * Returned Value:
 *   The non-zero address of the channel base registers is return on success.
 *   Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t xmc4_channel_baseaddress(enum usic_channel_e channel)
{
  if ((unsigned int)channel < (2 * XMC4_NUSIC))
    {
      return g_channel_baseaddress[channel];
    }

  return 0;
}

/****************************************************************************
 * Name: xmc4_enable_usic_channel
 *
 * Description:
 *   Enable the USIC channel indicated by 'channel'.  Also enable and reset
 *   the USIC module if it is not already enabled.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_enable_usic_channel(enum usic_channel_e channel)
{
  uintptr_t base;
  uintptr_t regaddr;
  uint32_t regval;
  int ret;

  /* Get the base address of the registers for this channel */

  base = xmc4_channel_baseaddress(channel);
  if (base == 0)
    {
      return -EINVAL;
    }

  /* Enable the USIC module */

  ret = xmc4_enable_usic(xmc4_channel2usic(channel));
  if (ret < 0)
    {
      return ret;
    }

  /* Enable USIC channel */

  regaddr = base + XMC4_USIC_KSCFG_OFFSET;
  putreg32(USIC_KSCFG_MODEN | USIC_KSCFG_BPMODEN, regaddr);

  /* Wait for the channel to become fully enabled */

  while ((getreg32(regaddr) & USIC_KSCFG_MODEN) == 0)
    {
    }

  /* Set USIC channel in IDLE mode */

  regaddr = base + XMC4_USIC_CCR_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~USIC_CCR_MODE_MASK;
  putreg32(regval, regaddr);

  return OK;
}

/****************************************************************************
 * Name: xmc4_disable_usic_channel
 *
 * Description:
 *   Disable the USIC channel indicated by 'channel'.  Also disable and reset
 *   the USIC module if both channels have been disabled.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_disable_usic_channel(enum usic_channel_e channel)
{
  uintptr_t base;
  uintptr_t other;
  uintptr_t regaddr;
  uint32_t regval;

  /* Get the base address of the registers for this channel */

  base = xmc4_channel_baseaddress(channel);
  if (base == 0)
    {
      return -EINVAL;
    }

  /* Disable this channel */

  regaddr = base + XMC4_USIC_KSCFG_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~USIC_KSCFG_MODEN;
  regval |= USIC_KSCFG_BPMODEN;
  putreg32(regval, regaddr);

  /* Get the base address of other channel for this USIC module */

  other = xmc4_channel_baseaddress(channel ^ 1);
  DEBUGASSERT(other != 0);

  /* Check if the other channel has also been disabled */

  regaddr = other + XMC4_USIC_KSCFG_OFFSET;
  if ((getreg32(regaddr) & USIC_KSCFG_MODEN) == 0)
    {
      /* Yes... Disable the USIC module */

      xmc4_disable_usic(xmc4_channel2usic(channel));
    }

  return OK;
}

/****************************************************************************
 * Name: xmc4_usic_baudrate
 *
 * Description:
 *   Set the USIC baudrate for the USIC channel
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int xmc4_usic_baudrate(enum usic_channel_e channel, uint32_t baud,
                       uint32_t oversampling)
{
  uintptr_t base;
  uint32_t periphclock;
  uint32_t clkdiv;
  uint32_t clkdiv_min;
  uint32_t pdiv;
  uint32_t pdiv_int;
  uint32_t pdiv_int_min;
  uint32_t pdiv_frac;
  uint32_t pdiv_frac_min;
  uint32_t regval;
  int ret;

  /* Get the base address of the registers for this channel */

  base = xmc4_channel_baseaddress(channel);
  if (base == 0)
    {
      return -EINVAL;
    }

  /* The baud and peripheral clock are divided by 100 to be able to use only
   * 32-bit arithmetic.
   */

  if (baud >= 100 && oversampling != 0)
    {
      periphclock   = xmc4_get_periphclock() / 100;
      baud          = baud / 100;

      clkdiv_min    = 1;
      pdiv_int_min  = 1;
      pdiv_frac_min = 0x3ff;

      for (clkdiv = 1023; clkdiv > 0; --clkdiv)
        {
          pdiv      = ((periphclock * clkdiv) / (baud * oversampling));
          pdiv_int  = pdiv >> 10;
          pdiv_frac = pdiv & 0x3ff;

          if (pdiv_int < 1024 && pdiv_frac < pdiv_frac_min)
            {
              pdiv_frac_min = pdiv_frac;
              pdiv_int_min  = pdiv_int;
              clkdiv_min    = clkdiv;
            }
        }

      /* Select and setup the fractional divider */

      regval = USIC_FDR_DM_FRACTIONAL | USIC_FDR_STEP(clkdiv_min);
      putreg32(regval, base + XMC4_USIC_FDR_OFFSET);

      /* Setup and enable the baud rate generator */

      regval  = getreg32(base + XMC4_USIC_BRG_OFFSET);
      regval &=  ~(USIC_BRG_DCTQ_MASK | USIC_BRG_PDIV_MASK |
                   USIC_BRG_PCTQ_MASK | USIC_BRG_PPPEN);
      regval |= (USIC_BRG_DCTQ(oversampling - 1) |
                 USIC_BRG_PDIV(pdiv_int_min - 1));
      putreg32(regval, base + XMC4_USIC_BRG_OFFSET);

      ret = OK;
    }
  else
    {
      ret = -ERANGE;
    }

  return ret;
}
