/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ble_hw.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Derived from Apache Mynewt NimBLE (nimble/drivers/nrf5x/src/ble_hw.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nrf52_ble_priv.h"
#include "hardware/nrf52_aar.h"
#include "hardware/nrf52_ecb.h"
#include "hardware/nrf52_ficr.h"
#include "hardware/nrf52_radio.h"
#include "hardware/nrf52_rng.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HCI error codes returned to the Link Layer */

#define BLE_ERR_SUCCESS          0x00
#define BLE_ERR_MEM_CAPACITY     0x07

/* Address types */

#define BLE_ADDR_RANDOM          0x01

/* Total size of the AAR resolving list */

#define NRF52_BLE_RESOLV_LIST_SIZE 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_ble_hw_whitelist_mask;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
uint32_t g_nrf52_ble_irk_list[NRF52_BLE_IRK_ENTRIES * 4];
uint8_t g_nrf52_ble_num_irks;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_le32 / get_le16
 ****************************************************************************/

static inline uint32_t get_le32(FAR const uint8_t *buf)
{
  return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline uint16_t get_le16(FAR const uint8_t *buf)
{
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/****************************************************************************
 * Name: nrf52_ble_hw_ficr_addr
 ****************************************************************************/

static void nrf52_ble_hw_ficr_addr(FAR uint8_t *addr)
{
  uint32_t addr_low;
  uint32_t addr_high;

  addr_low  = getreg32(NRF52_FICR_DEVICEADDR0);
  addr_high = getreg32(NRF52_FICR_DEVICEADDR1);

  addr[0] = (uint8_t)addr_low;
  addr[1] = (uint8_t)(addr_low >> 8);
  addr[2] = (uint8_t)(addr_low >> 16);
  addr[3] = (uint8_t)(addr_low >> 24);
  addr[4] = (uint8_t)addr_high;
  addr[5] = (uint8_t)(addr_high >> 8);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_hw_get_public_addr
 *
 * Description:
 *   Get the public device address from FICR if it is programmed as public.
 *
 ****************************************************************************/

int nrf52_ble_hw_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                 FAR uint8_t *addr)
{
  if ((getreg32(NRF52_FICR_DEVICEADDRTYPE) & 1) !=
      FICR_DEVICEADDRTYPE_PUBLIC)
    {
      return -1;
    }

  nrf52_ble_hw_ficr_addr(addr);
  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_get_static_addr
 *
 * Description:
 *   Get the random static address from FICR.
 *
 ****************************************************************************/

int nrf52_ble_hw_get_static_addr(FAR struct ble_ll_lower_s *lower,
                                 FAR uint8_t *addr)
{
  if ((getreg32(NRF52_FICR_DEVICEADDRTYPE) & 1) !=
      FICR_DEVICEADDRTYPE_RANDOM)
    {
      return -1;
    }

  nrf52_ble_hw_ficr_addr(addr);
  addr[5] |= 0xc0;
  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_clear
 ****************************************************************************/

void nrf52_ble_hw_whitelist_clear(FAR struct ble_ll_lower_s *lower)
{
  putreg32(0, NRF52_RADIO_DACNF);
  g_ble_hw_whitelist_mask = 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_add
 *
 * Description:
 *   Add a device to the hardware filter accept list.
 *
 ****************************************************************************/

int nrf52_ble_hw_whitelist_add(FAR struct ble_ll_lower_s *lower,
                               FAR const uint8_t *addr, uint8_t addr_type)
{
  uint32_t mask;
  int i;

  mask = 0x01;
  for (i = 0; i < NRF52_BLE_WHITELIST_SIZE; ++i)
    {
      if ((mask & g_ble_hw_whitelist_mask) == 0)
        {
          putreg32(get_le32(addr), NRF52_RADIO_DAB(i));
          putreg32(get_le16(addr + 4), NRF52_RADIO_DAP(i));
          if (addr_type == BLE_ADDR_RANDOM)
            {
              modifyreg32(NRF52_RADIO_DACNF, 0, mask << 8);
            }

          g_ble_hw_whitelist_mask |= mask;
          return BLE_ERR_SUCCESS;
        }

      mask <<= 1;
    }

  return BLE_ERR_MEM_CAPACITY;
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_rmv
 *
 * Description:
 *   Remove a device from the hardware filter accept list.
 *
 ****************************************************************************/

void nrf52_ble_hw_whitelist_rmv(FAR struct ble_ll_lower_s *lower,
                                FAR const uint8_t *addr, uint8_t addr_type)
{
  uint8_t cfg_addr;
  uint16_t txadd;
  uint16_t dap;
  uint32_t dab;
  uint32_t mask;
  int i;

  dab   = get_le32(addr);
  dap   = get_le16(addr + 4);
  txadd = getreg32(NRF52_RADIO_DACNF) >> 8;
  mask  = 0x01;

  for (i = 0; i < NRF52_BLE_WHITELIST_SIZE; ++i)
    {
      if ((mask & g_ble_hw_whitelist_mask) &&
          dab == getreg32(NRF52_RADIO_DAB(i)) &&
          dap == getreg32(NRF52_RADIO_DAP(i)))
        {
          cfg_addr = txadd & mask;
          if ((addr_type == BLE_ADDR_RANDOM && cfg_addr != 0) ||
              (addr_type != BLE_ADDR_RANDOM && cfg_addr == 0))
            {
              break;
            }
        }

      mask <<= 1;
    }

  if (i < NRF52_BLE_WHITELIST_SIZE)
    {
      g_ble_hw_whitelist_mask &= ~mask;
      modifyreg32(NRF52_RADIO_DACNF, mask, 0);
    }
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_size
 ****************************************************************************/

uint8_t nrf52_ble_hw_whitelist_size(FAR struct ble_ll_lower_s *lower)
{
  return NRF52_BLE_WHITELIST_SIZE;
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_enable
 ****************************************************************************/

void nrf52_ble_hw_whitelist_enable(FAR struct ble_ll_lower_s *lower)
{
  modifyreg32(NRF52_RADIO_DACNF, 0, g_ble_hw_whitelist_mask);
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_disable
 ****************************************************************************/

void nrf52_ble_hw_whitelist_disable(FAR struct ble_ll_lower_s *lower)
{
  modifyreg32(NRF52_RADIO_DACNF, 0x000000ff, 0);
}

/****************************************************************************
 * Name: nrf52_ble_hw_whitelist_match
 ****************************************************************************/

int nrf52_ble_hw_whitelist_match(FAR struct ble_ll_lower_s *lower)
{
  return (int)getreg32(NRF52_RADIO_EVENTS_DEVMATCH);
}

/****************************************************************************
 * Name: nrf52_ble_hw_encrypt_block
 *
 * Description:
 *   AES-128 encrypt one block with the ECB peripheral.  The block
 *   structure layout (key, plain text, cipher text) matches ECBDATAPTR.
 *
 ****************************************************************************/

int nrf52_ble_hw_encrypt_block(FAR struct ble_ll_lower_s *lower,
                               FAR struct ble_encryption_block *ecb)
{
  uint32_t end;
  uint32_t err;

  putreg32(1, NRF52_ECB_TASKS_STOPECB);
  putreg32(0, NRF52_ECB_EVENTS_ENDECB);
  putreg32(0, NRF52_ECB_EVENTS_ERRORECB);
  putreg32((uint32_t)ecb, NRF52_ECB_ECBDATAPTR);
  putreg32(1, NRF52_ECB_TASKS_STARTECB);

  do
    {
      end = getreg32(NRF52_ECB_EVENTS_ENDECB);
      err = getreg32(NRF52_ECB_EVENTS_ERRORECB);
    }
  while (!end && !err);

  return err ? -1 : 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_rng_isr
 ****************************************************************************/

int nrf52_ble_hw_rng_isr(int irq, FAR void *context, FAR void *arg)
{
  uint8_t rnum;

  if (getreg32(NRF52_RNG_EVENTS_RDY))
    {
      putreg32(0, NRF52_RNG_EVENTS_RDY);
      rnum = (uint8_t)getreg32(NRF52_RNG_VALUE);
      g_nrf52_ble_cb->rng_sample(rnum);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_hw_rng_start
 ****************************************************************************/

int nrf52_ble_hw_rng_start(FAR struct ble_ll_lower_s *lower)
{
  irqstate_t flags;

  flags = enter_critical_section();
  putreg32(0, NRF52_RNG_EVENTS_RDY);
  putreg32(RNG_INT_RDY, NRF52_RNG_INTSET);
  putreg32(1, NRF52_RNG_TASKS_START);
  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_rng_stop
 ****************************************************************************/

int nrf52_ble_hw_rng_stop(FAR struct ble_ll_lower_s *lower)
{
  irqstate_t flags;

  flags = enter_critical_section();
  putreg32(RNG_INT_RDY, NRF52_RNG_INTCLR);
  putreg32(1, NRF52_RNG_TASKS_STOP);
  putreg32(0, NRF52_RNG_EVENTS_RDY);
  leave_critical_section(flags);

  return 0;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: nrf52_ble_hw_resolv_list_clear
 ****************************************************************************/

void nrf52_ble_hw_resolv_list_clear(FAR struct ble_ll_lower_s *lower)
{
  g_nrf52_ble_num_irks = 0;
}

/****************************************************************************
 * Name: nrf52_ble_hw_resolv_list_add
 *
 * Description:
 *   Add an IRK to the AAR resolving list.
 *
 ****************************************************************************/

int nrf52_ble_hw_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                 FAR const uint8_t *irk)
{
  if (g_nrf52_ble_num_irks == NRF52_BLE_IRK_ENTRIES)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  memcpy(&g_nrf52_ble_irk_list[4 * g_nrf52_ble_num_irks], irk, 16);
  ++g_nrf52_ble_num_irks;

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: nrf52_ble_hw_resolv_list_rmv
 ****************************************************************************/

void nrf52_ble_hw_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                  int index)
{
  FAR uint32_t *irk_entry;

  if (index < g_nrf52_ble_num_irks)
    {
      --g_nrf52_ble_num_irks;
      irk_entry = &g_nrf52_ble_irk_list[4 * index];
      if (g_nrf52_ble_num_irks > index)
        {
          memmove(irk_entry, irk_entry + 4,
                  16 * (g_nrf52_ble_num_irks - index));
        }
    }
}

/****************************************************************************
 * Name: nrf52_ble_hw_resolv_list_size
 ****************************************************************************/

uint8_t nrf52_ble_hw_resolv_list_size(FAR struct ble_ll_lower_s *lower)
{
  return NRF52_BLE_RESOLV_LIST_SIZE;
}

/****************************************************************************
 * Name: nrf52_ble_hw_resolv_list_match
 *
 * Description:
 *   Return the index of the IRK that resolved the last address or -1.
 *
 ****************************************************************************/

int nrf52_ble_hw_resolv_list_match(FAR struct ble_ll_lower_s *lower)
{
  if (getreg32(NRF52_AAR_ENABLE) && getreg32(NRF52_AAR_EVENTS_END) &&
      getreg32(NRF52_AAR_EVENTS_RESOLVED))
    {
      return (int)getreg32(NRF52_AAR_STATUS);
    }

  return -1;
}
#endif
