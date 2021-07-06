/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_efuse.c
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

#include "hal_api.h"
#include "hal_efuse.h"
#include "hal_efuse_nsc.h"
#include "platform_conf.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_EFUSE_EN
/* #define EFUSE_LOGICAL_SIM */

#define EFUSE_LOGICAL_MAP_SIZE 512
#define EFUSE_LOGICAL_MAP_HW_SIZE 0xd0 /* sync with EFUSE_OOB_PROTECT_BYTES */
#define EFUSE_LOGICAL_SBLOCK_OFFSET 0x19
#ifdef EFUSE_LOGICAL_SIM
uint8_t ameba_efuse_sim_map[256];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ameba_efuse_logical_read(uint16_t laddr, uint16_t size, uint8_t *pbuf)
{
  uint8_t offset;
  uint8_t wden;
  uint8_t header;
  uint8_t extheader;
  uint8_t data;
  uint16_t phy_addr = 0;
  uint16_t i;
  uint32_t ret;
  if (!pbuf)
    {
      return -EIO;
    }

  rtw_memset(pbuf, 0xff, size);
  while (phy_addr < EFUSE_LOGICAL_MAP_HW_SIZE)
    {
      /* First two bytes are reserved for physical */

      if (phy_addr == 0 || phy_addr == 1)
        {
          phy_addr++;
          continue;
        }

#ifdef EFUSE_LOGICAL_SIM
      static int map_inited = 0;
      if (!map_inited)
        {
          map_inited = 1;
          for (i = 0; i < EFUSE_LOGICAL_MAP_HW_SIZE; i++)
            {
              hal_efuse_read(i, &ameba_efuse_sim_map[i],
                             LDO_OUT_DEFAULT_VOLT);
            }
        }

      ret = _TRUE;
      header = ameba_efuse_sim_map[phy_addr++];
#else
      ret = hal_efuse_read(phy_addr++, &header, LDO_OUT_DEFAULT_VOLT);
#endif
      if (ret != _TRUE)
        {
          return -EIO;
        }

      if (header == 0xff)
        {
          break;
        }

      /* Check PG header for section num. */

      if ((header & 0x1f) == 0x0f)
        {
          /* extended header */

          offset = (header & 0xe0) >> 5;
#ifdef EFUSE_LOGICAL_SIM
          ret = _TRUE;
          extheader = ameba_efuse_sim_map[phy_addr++];
#else
          ret = hal_efuse_read(phy_addr++, &extheader, LDO_OUT_DEFAULT_VOLT);
#endif
          if (ret != _TRUE)
            {
              return -EIO;
            }

          if (((extheader & 0x0f) == 0x0f))
            {
              continue;
            }

          offset |= ((extheader & 0xf0) >> 1);
          wden = (extheader & 0x0f);
        }

      else
        {
          offset = ((header >> 4) & 0x0f);
          wden = (header & 0x0f);
        }

      /* One section has 8 bytes data, logical map has 512/8 = 64 sections */

      if (offset < (EFUSE_LOGICAL_MAP_SIZE >> 3))
        {
          uint16_t addr = 0;

          /* Get word enable value from PG header */

          addr = offset * 8;

          /* Each section has 4 words data */

          for (i = 0; i < 4; i++)
            {
              /* Check word enable condition in the section */

              if (!(wden & (0x01 << i)))
                {
#ifdef EFUSE_LOGICAL_SIM
                  ret = _TRUE;
                  data = ameba_efuse_sim_map[phy_addr++];
#else
                  ret = hal_efuse_read(phy_addr++, &data,
                                       LDO_OUT_DEFAULT_VOLT);
#endif
                  if (ret != _TRUE)
                    {
                      return -EIO;
                    }

                  if (addr >= laddr && addr < (laddr + size))
                    {
                      pbuf[addr - laddr] = data;
                    }

#ifdef EFUSE_LOGICAL_SIM
                  ret = _TRUE;
                  data = ameba_efuse_sim_map[phy_addr++];
#else
                  ret = hal_efuse_read(phy_addr++, &data,
                                       LDO_OUT_DEFAULT_VOLT);
#endif
                  if (ret != _TRUE)
                    {
                      return -EIO;
                    }

                  if ((addr + 1) >= laddr && (addr + 1) < (laddr + size))
                    {
                      pbuf[addr + 1 - laddr] = data;
                    }
                }

              addr += 2;
            }
        }

      else
        {
          uint8_t word_cnts = 0;
          if (!(wden & BIT(0)))
            {
              word_cnts++;  /* 0 : write enable */
            }

          if (!(wden & BIT(1)))
            {
              word_cnts++;
            }

          if (!(wden & BIT(2)))
            {
              word_cnts++;
            }

          if (!(wden & BIT(3)))
            {
              word_cnts++;
            }

          phy_addr += word_cnts * 2;
        }
    }

  /* return used bytes */

  return phy_addr - EIO;
}

#endif
