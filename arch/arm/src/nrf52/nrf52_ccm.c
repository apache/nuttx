/****************************************************************************
 * /home/v01d/coding/nuttx_nrf_ble/nuttx/arch/arm/src/nrf52/nrf52_ccm.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf52_ccm.h"
#include "hardware/nrf52_ccm.h"
#include "arm_arch.h"
#include "irq/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

uint8_t g_scratch[43];    /* Scratch area for CCM */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf52_ccm_set_inptr(struct nrf52_ccm_pkt_s *pkt)
{
  putreg32((uint32_t)pkt, NRF52_CCM_INPTR);
}

void nrf52_ccm_set_outptr(struct nrf52_ccm_pkt_s *pkt)
{
  putreg32((uint32_t)pkt, NRF52_CCM_OUTPTR);
}

struct nrf52_ccm_pkt_s *nrf52_ccm_get_inptr(void)
{
  return (struct nrf52_ccm_pkt_s *)getreg32(NRF52_CCM_INPTR);
}

struct nrf52_ccm_pkt_s *nrf52_ccm_get_outptr(void)
{
  return (struct nrf52_ccm_pkt_s *)getreg32(NRF52_CCM_OUTPTR);
}

void nrf52_ccm_set_nonce(const struct nrf52_ccm_nonce_s *nonce)
{
  putreg32((uint32_t)nonce, NRF52_CCM_CNFPTR);
}

const struct nrf52_ccm_nonce_s *nrf52_ccm_get_nonce(void)
{
  return (const struct nrf52_ccm_nonce_s *)getreg32(NRF52_CCM_CNFPTR);
}

void nrf52_ccm_configure(enum nrf52_ccm_datarate_e rate,
                         enum nrf52_ccm_pktlen_e len)
{
  uint32_t setbits = 0;
  uint32_t clearbits = 0;

  switch (rate)
    {
      case CCM_DATARATE_1M:
        clearbits |= NRF52_CCM_MODE_DATARATE_2M;
        break;
      case CCM_DATARATE_2M:
        setbits |= NRF52_CCM_MODE_DATARATE_2M;
        break;
    }

  switch (len)
    {
      case CCM_PKTLEN_DEFAULT:
        clearbits |= NRF52_CCM_MODE_LENGTH_EXT;
        break;
      case CCM_PKTLEN_EXTENDED:
        setbits |= NRF52_CCM_MODE_LENGTH_EXT;
        break;
    }

  modifyreg32(NRF52_CCM_MODE, clearbits, setbits);
}

void nrf52_ccm_setmode(enum nrf52_ccm_mode_e mode)
{
  modifyreg32(NRF52_CCM_MODE,
              (mode == CCM_MODE_DECRYPT ? 0 : NRF52_CCM_MODE_DECRYPT),
              (mode == CCM_MODE_DECRYPT ? NRF52_CCM_MODE_DECRYPT : 0));
}

void nrf52_ccm_shortcut(bool enable)
{
  putreg32(enable ? 1 : 0, NRF52_CCM_SHORTS);
}

bool nrf52_ccm_passed(void)
{
  return (getreg32(NRF52_CCM_MICSTATUS) == NRF52_CCM_MICSTATUS_PASS);
}

void nrf52_ccm_enable(bool enable)
{
  putreg32(enable ? NRF52_CCM_ENABLE_ENABLED : 0, NRF52_CCM_ENABLE);
}

void nrf52_ccm_enableint(enum nrf52_ccm_int_e interrupt, bool enable)
{
  uint32_t intval = 0;

  switch (interrupt)
    {
      case CCM_INT_ENDCRYPT:
        intval = NRF52_CCM_INTENSET_ENDCRYPT;
        break;
      case CCM_INT_ENDKSGEN:
        intval = NRF52_CCM_INTENSET_ENDKSGEN;
        break;
      case CCM_INT_ERROR:
        intval = NRF52_CCM_INTENSET_ERROR;
        break;
    }

  putreg32(intval, enable ? NRF52_CCM_INTENSET : NRF52_CCM_INTENCLR);
}

void nrf52_ccm_ackint(enum nrf52_ccm_int_e interrupt)
{
  switch (interrupt)
    {
      case CCM_INT_ENDCRYPT:
        putreg32(0, NRF52_CCM_EVENTS_ENDCRYPT);
        break;
      case CCM_INT_ENDKSGEN:
        putreg32(0, NRF52_CCM_EVENTS_ENDKSGEN);
        break;
      case CCM_INT_ERROR:
        putreg32(0, NRF52_CCM_EVENTS_ERROR);
        break;
    }
}

bool nrf52_ccm_checkint(enum nrf52_ccm_int_e interrupt)
{
  switch (interrupt)
    {
      case CCM_INT_ENDCRYPT:
        return getreg32(NRF52_CCM_EVENTS_ENDCRYPT);
        break;
      case CCM_INT_ENDKSGEN:
        return getreg32(NRF52_CCM_EVENTS_ENDKSGEN);
        break;
      case CCM_INT_ERROR:
        return getreg32(NRF52_CCM_EVENTS_ERROR);
        break;
    }

  return false;
}

void nrf52_ccm_setisr(xcpt_t handler, void *arg)
{
  if (!handler)
    {
      /* Disable interrupt when callback is removed */

      up_disable_irq(NRF52_IRQ_CCM_AAR);
      irq_detach(NRF52_IRQ_CCM_AAR);
    }
  else
    {
      /* Otherwise set callback and enable interrupt */

      irq_attach(NRF52_IRQ_CCM_AAR, handler, arg);
      up_enable_irq(NRF52_IRQ_CCM_AAR);
    }
}

void nrf52_ccm_initialize(void)
{
  /* Enable peripheral */

  nrf52_ccm_enable(true);

  /* Zero events */

  putreg32(0, NRF52_CCM_EVENTS_ENDCRYPT);
  putreg32(0, NRF52_CCM_EVENTS_ENDKSGEN);
  putreg32(0, NRF52_CCM_EVENTS_ERROR);

  /* Set scratch pointer */

  putreg32((uint32_t)g_scratch, NRF52_CCM_SCRATCHPTR);
}
