/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_pindma.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include "arm_internal.h"

#include "s32k3xx_config.h"
#include "chip.h"

#ifdef CONFIG_S32K3XX_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_pindmaenable
 *
 * Description:
 *   Enable DMA for specified pin
 *
 ****************************************************************************/

void s32k3xx_pindmaenable(uint32_t pinset)
{
#if 0
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K3XX_NPORTS);
  if (port < S32K3XX_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  S32K3XX_PORT_BASE(port);

      /* Modify the IRQC field of the port PCR register in order to
       * enable DMA.
       */

      regval = getreg32(base + S32K3XX_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;

      switch (pinset & _PIN_INT_MASK)
        {
          case PIN_DMA_RISING : /* DMA Request on rising edge */
            regval |= PORT_PCR_IRQC_DMARISING;
            break;

          case PIN_DMA_FALLING : /* DMA Request on falling edge */
            regval |= PORT_PCR_IRQC_DMAFALLING;
            break;

          case PIN_DMA_BOTH : /* DMA Request on either edge */
            regval |= PORT_PCR_IRQC_DMABOTH;
            break;

          default:
            return;
        }

      putreg32(regval, base + S32K3XX_PORT_PCR_OFFSET(pin));
    }
#endif
}

/****************************************************************************
 * Name: s32k3xx_pindmadisable
 *
 * Description:
 *   Disable DMA for specified pin
 *
 ****************************************************************************/

void s32k3xx_pindmadisable(uint32_t pinset)
{
#if 0
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K3XX_NPORTS);
  if (port < S32K3XX_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  S32K3XX_PORT_BASE(port);

      /* Clear the IRQC field of the port PCR register in order to disable
       * DMA.
       */

      regval = getreg32(base + S32K3XX_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;
      putreg32(regval, base + S32K3XX_PORT_PCR_OFFSET(pin));
    }
#endif
}

#endif
