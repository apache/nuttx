/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ble_hw.c
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

#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nrf54l_ble_priv.h"
#include "hardware/nrf54l_aar.h"
#include "hardware/nrf54l_ficr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HCI error codes returned to the Link Layer */

#define BLE_ERR_SUCCESS          0x00
#define BLE_ERR_MEM_CAPACITY     0x07

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_ble_hw_ficr_addr
 ****************************************************************************/

static void nrf54l_ble_hw_ficr_addr(FAR uint8_t *addr)
{
  uint32_t addr_low;
  uint32_t addr_high;

  addr_low  = getreg32(NRF54L_FICR_DEVICEADDR(0));
  addr_high = getreg32(NRF54L_FICR_DEVICEADDR(1));

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
 * Name: nrf54l_ble_hw_get_public_addr
 *
 * Description:
 *   Get the public device address from FICR if it is programmed as public.
 *
 ****************************************************************************/

int nrf54l_ble_hw_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                  FAR uint8_t *addr)
{
  if ((getreg32(NRF54L_FICR_DEVICEADDRTYPE) & FICR_DEVICEADDRTYPE_RANDOM) !=
      0)
    {
      return -1;
    }

  nrf54l_ble_hw_ficr_addr(addr);
  return 0;
}

/****************************************************************************
 * Name: nrf54l_ble_hw_get_static_addr
 *
 * Description:
 *   Get the random static address from FICR.
 *
 ****************************************************************************/

int nrf54l_ble_hw_get_static_addr(FAR struct ble_ll_lower_s *lower,
                                  FAR uint8_t *addr)
{
  if ((getreg32(NRF54L_FICR_DEVICEADDRTYPE) & FICR_DEVICEADDRTYPE_RANDOM) ==
      0)
    {
      return -1;
    }

  nrf54l_ble_hw_ficr_addr(addr);
  addr[5] |= 0xc0;
  return 0;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: nrf54l_ble_hw_resolv_list_clear
 ****************************************************************************/

void nrf54l_ble_hw_resolv_list_clear(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->num_irks = 0;
}

/****************************************************************************
 * Name: nrf54l_ble_hw_resolv_list_add
 *
 * Description:
 *   Add an IRK to the AAR resolving list.
 *
 ****************************************************************************/

int nrf54l_ble_hw_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *irk)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  if (dev->num_irks == NRF54L_BLE_IRK_ENTRIES)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  memcpy(&dev->irk_list[4 * dev->num_irks], irk, 16);
  ++dev->num_irks;

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: nrf54l_ble_hw_resolv_list_rmv
 ****************************************************************************/

void nrf54l_ble_hw_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                   int index)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  FAR uint32_t *irk_entry;

  if (index < dev->num_irks)
    {
      --dev->num_irks;
      irk_entry = &dev->irk_list[4 * index];
      if (dev->num_irks > index)
        {
          memmove(irk_entry, irk_entry + 4,
                  16 * (dev->num_irks - index));
        }
    }
}

/****************************************************************************
 * Name: nrf54l_ble_hw_resolv_list_size
 ****************************************************************************/

uint8_t nrf54l_ble_hw_resolv_list_size(FAR struct ble_ll_lower_s *lower)
{
  return NRF54L_BLE_IRK_ENTRIES;
}

/****************************************************************************
 * Name: nrf54l_ble_hw_resolv_list_match
 *
 * Description:
 *   Return the index of the IRK that resolved the last address or -1.
 *
 ****************************************************************************/

int nrf54l_ble_hw_resolv_list_match(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  if (getreg32(NRF54L_AAR_ENABLE) == AAR_ENABLE_ENABLED &&
      getreg32(NRF54L_AAR_EVENTS_END) &&
      getreg32(NRF54L_AAR_EVENTS_RESOLVED) &&
      dev->aar_job.status < dev->num_irks)
    {
      return (int)dev->aar_job.status;
    }

  return -1;
}
#endif
