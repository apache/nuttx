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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>

#include <arch/xmc4/chip.h>

#include "up_arch.h"
#include "chip/xmc4_usic.h"
#include "chip/xmc4_scu.h"
#include "xmc4_usic.h"

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
        /* Check if USIC0 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT0) & SCU_CGAT0_USIC0) == 0)
          {
            /* Ungate USIC0 clocking */

            putreg32(SCU_CGAT0_USIC0, XMC4_SCU_CGATCLR0);

            /* De-assert peripheral reset USIC0 */

            putreg32(SCU_PR0_USIC0RS, XMC4_SCU_PRCLR0);
          }

        break;

#if XMC4_NUSIC > 1
      case USIC1:
        /* Check if USIC1 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT1) & SCU_CGAT1_USIC1) == 0)
          {
            /* Ungate USIC1 clocking */

            putreg32(SCU_CGAT1_USIC1, XMC4_SCU_CGATCLR1);

            /* De-assert peripheral reset USIC1 */

            putreg32(SCU_PR1_USIC1RS, XMC4_SCU_PRCLR1);
          }

        break;

#if XMC4_NUSIC > 2
      case USIC2:
        /* Check if USIC2 is already ungated */

        if ((getreg32(XMC4_SCU_CGATSTAT1) & SCU_CGAT1_USIC2) == 0)
          {
            /* Ungate USIC2 clocking */

            putreg32(SCU_CGAT1_USIC2, XMC4_SCU_CGATCLR1);

            /* De-assert peripheral reset USIC2 */

            putreg32(SCU_PR1_USIC2RS, XMC4_SCU_PRCLR1);
          }

        break;
#endif
#endif

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
        /* Assert peripheral reset USIC0 */

        putreg32(SCU_PR0_USIC0RS, XMC4_SCU_PRSET0);

        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT0_USIC0, XMC4_SCU_CGATSET0);
        break;

#if XMC4_NUSIC > 1
      case USIC1:
        /* Assert peripheral reset USIC0 */

        putreg32(SCU_PR1_USIC1RS, XMC4_SCU_PRSET1);

        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT1_USIC1, XMC4_SCU_CGATSET1);
        break;

#if XMC4_NUSIC > 2
      case USIC2:
        /* Assert peripheral reset USIC0 */

        putreg32(SCU_PR1_USIC2RS, XMC4_SCU_PRSET1);

        /* Gate USIC0 clocking */

        putreg32(SCU_CGAT1_USIC2, XMC4_SCU_CGATSET1);
        break;
#endif
#endif

      default:
        return -EINVAL;
    }

  return OK;
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

  switch (channel)
    {
      case USIC0_CHAN0:
        /* USIC0 Channel 0 base address */

        base = XMC4_USIC0_CH0_BASE;

        /* Enable USIC0 */

        xmc4_enable_usic(USIC0);
        break;

      case USIC0_CHAN1:
        /* USIC0 Channel 1 base address */

        base = XMC4_USIC0_CH1_BASE;

        /* Enable USIC0 */

        xmc4_enable_usic(USIC0);
        break;

#if XMC4_NUSIC > 1
      case USIC1_CHAN0:
        /* USIC1 Channel 0 base address */

        base = XMC4_USIC1_CH0_BASE;

        /* Enable USIC1 */

        xmc4_enable_usic(USIC1);
        break;

      case USIC1_CHAN1:
        /* USIC1 Channel 1 base address */

        base = XMC4_USIC1_CH1_BASE;

        /* Enable USIC1 */

        xmc4_enable_usic(USIC1);
        break;

#if XMC4_NUSIC > 2
      case USIC2_CHAN0:
        /* USIC2 Channel 0 base address */

        base = XMC4_USIC2_CH0_BASE;

        /* Enable USIC2 */

        xmc4_enable_usic(USIC2);
        break;

      case USIC2_CHAN1:
        /* USIC2 Channel 1 base address */

        base = XMC4_USIC2_CH1_BASE;

        /* Enable USIC2 */

        xmc4_enable_usic(USIC2);
        break;
#endif
#endif

      default:
        return -EINVAL;
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
  enum usic_e usic;

  switch (channel)
    {
      case USIC0_CHAN0:
        /* Enable USIC0 Channel 0 base address */

        base  = XMC4_USIC0_CH0_BASE;
        other = XMC4_USIC0_CH1_BASE;
        usic  = USIC0;
        break;

      case USIC0_CHAN1:
        /* Enable USIC0 Channel 1 base address */

        base  = XMC4_USIC0_CH1_BASE;
        other = XMC4_USIC0_CH0_BASE;
        usic  = USIC0;
        break;

#if XMC4_NUSIC > 1
      case USIC1_CHAN0:
        /* Enable USIC1 Channel 0 base address */

        base  = XMC4_USIC1_CH0_BASE;
        other = XMC4_USIC1_CH1_BASE;
        usic  = USIC1;
        break;

      case USIC1_CHAN1:
        /* Enable USIC1 Channel 1 base address */

        base  = XMC4_USIC1_CH1_BASE;
        other = XMC4_USIC1_CH0_BASE;
        usic  = USIC1;
        break;

#if XMC4_NUSIC > 2
      case USIC2_CHAN0:
        /* Enable USIC2 Channel 0 base address */

        base  = XMC4_USIC2_CH0_BASE;
        other = XMC4_USIC2_CH1_BASE;
        usic  = USIC2;
        break;

      case USIC2_CHAN1:
        /* Enable USIC2 Channel 1 base address */

        base  = XMC4_USIC2_CH1_BASE;
        other = XMC4_USIC2_CH0_BASE;
        usic  = USIC2;
        break;
#endif
#endif

      default:
        return -EINVAL;
    }

  /* Disable this channel */

  regaddr = base + XMC4_USIC_KSCFG_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~USIC_KSCFG_MODEN;
  regval |= USIC_KSCFG_BPMODEN;
  putreg32(regval, regaddr);

  /* Check if the other channel has also been disabled */

  regaddr = other + XMC4_USIC_KSCFG_OFFSET;
  if ((getreg32(regaddr) & USIC_KSCFG_MODEN) == 0)
    {
      /* Yes... Disable the USIC module */

      xmc4_disable_usic(usic);
    }

  return OK;
}