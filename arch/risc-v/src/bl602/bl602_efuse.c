/****************************************************************************
 * arch/risc-v/src/bl602/bl602_efuse.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include "chip.h"
#include "riscv_arch.h"

#include "hardware/bl602_ef.h"
#include "bl602_romapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bl602_romapi_efuse_ctrl_load_r0 \
  ((void (*)(void))BL602_ROMAPI_EFUSE_CTRL_LOAD_R0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t count_zero_bits_in_byte(uint8_t val)
{
  uint32_t cnt = 0;
  uint32_t i   = 0;
  for (i = 0; i < 8; i++)
    {
      if ((val & (1 << i)) == 0)
        {
          cnt += 1;
        }
    }

  return cnt;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_efuse_read_mac_address
 *
 * Description:
 *   Read MAC address from efuse.
 *
 * Input Parameters:
 *   mac: the buffer to hold mac address
 *
 * Returned Value:
 *   0: OK
 *   ENODATA: Failed
 *
 ****************************************************************************/

int bl602_efuse_read_mac_address(uint8_t mac[6])
{
  DEBUGASSERT(mac != NULL);

  uint8_t *maclow  = (uint8_t *)mac;
  uint8_t *machigh = (uint8_t *)(mac + 4);
  uint32_t tmp_val;
  uint32_t i   = 0;
  uint32_t cnt = 0;

  /* Trigger read data from efuse */

  bl602_romapi_efuse_ctrl_load_r0();

  tmp_val   = getreg32(BL602_EF_WIFI_MAC_LOW);
  maclow[0] = tmp_val & 0xff;
  maclow[1] = (tmp_val >> 8) & 0xff;
  maclow[2] = (tmp_val >> 16) & 0xff;
  maclow[3] = (tmp_val >> 24) & 0xff;

  tmp_val    = getreg32(BL602_EF_WIFI_MAC_HIGH);
  machigh[0] = tmp_val & 0xff;
  machigh[1] = (tmp_val >> 8) & 0xff;

  /* Check parity */

  for (i = 0; i < 6; i++)
    {
      cnt += count_zero_bits_in_byte(mac[i]);
    }

  if ((cnt & 0x3f) == ((tmp_val >> 16) & 0x3f))
    {
      /* Change to network order */

      for (i = 0; i < 3; i++)
        {
          tmp_val    = mac[i];
          mac[i]     = mac[5 - i];
          mac[5 - i] = tmp_val;
        }

      return 0;
    }
  else
    {
      return -ENODATA;
    }
}

