/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_hci_board.c
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

#include <stdint.h>
#include <hal_api.h>
#include <hal_efuse.h>
#include <hal_sys_ctrl.h>
#include <nuttx/config.h>
#include "amebaz_hci_board.h"
#include "ameba_efuse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_LGC_EFUSE_LEN           0x20
#define BT_PHY_EFUSE_LEN           0x12
#define BT_PHY_EFUSE_BASE          0x100
#define BT_LGC_EFUSE_OFFSET        0x190
#define BT_MAC_ADDR_LEN            6
#define BT_CONFIG_SIGNATURE        0x8723ab55
#define BT_CONFIG_HEADER_LEN       6
#define EFUSE_SW_USE_FLASH_PATCH   BIT0
#define EFUSE_SW_BT_FW_LOG         BIT1
#define EFUSE_SW_RSVD              BIT2
#define EFUSE_SW_IQK_HCI_OUT       BIT3
#define EFUSE_SW_UPPERSTACK_SWITCH BIT4
#define EFUSE_SW_TRACE_SWITCH      BIT5
#define EFUSE_SW_DRIVER_DEBUG_LOG  BIT6
#define EFUSE_SW_RSVD2             BIT7
#define LEFUSE(x)                 (x-0x190)
#define FLASH_BT_PARA_ADDR        (SYS_DATA_FLASH_BASE + 0xff0)
#define CHECK_SW(x)               (HAL_READ32(SPI_FLASH_BASE, FLASH_BT_PARA_ADDR) &x)
#define MERGE_PATCH_ADDRESS       0x110000
#define LE_ARRAY_TO_UINT16(u16, a)  {     \
  u16 = ((uint16_t)(*(a + 0)) << 0) +   \
        ((uint16_t)(*(a + 1)) << 8);      \
  }

#define LE_ARRAY_TO_UINT32(u32, a) {    \
  u32 = ((uint32_t)(*(a + 0)) <<  0)+ \
        ((uint32_t)(*(a + 1)) <<   8)+  \
        ((uint32_t)(*(a + 2)) << 16)+ \
        ((uint32_t)(*(a + 3)) << 24);   \
  }

#define LE_UINT32_TO_ARRAY(a, u32) {                 \
    *((uint8_t *)(a) + 0) = (uint8_t)((u32) >>  0); \
    *((uint8_t *)(a) + 1) = (uint8_t)((u32) >>  8); \
    *((uint8_t *)(a) + 2) = (uint8_t)((u32) >> 16); \
    *((uint8_t *)(a) + 3) = (uint8_t)((u32) >> 24); \
  }

#define LE_STREAM_TO_UINT8(u8, s) {     \
  u8 = (uint8_t)(*s);                   \
  s += 1;                               \
}

#define LE_STREAM_TO_UINT16(u16, s) {   \
  u16 = ((uint16_t)(*(s + 0)) << 0) + \
        ((uint16_t)(*(s + 1)) << 8);    \
  s += 2;                               \
}

#define LE_UINT16_TO_STREAM(s, u16) {   \
  *s++ = (uint8_t)((u16) >> 0);         \
  *s++ = (uint8_t)((u16) >> 8);         \
}

#define LE_STREAM_TO_UINT32(u32, s) {    \
  u32 = ((uint32_t)(*(s + 0)) <<  0) + \
        ((uint32_t)(*(s + 1)) <<   8)  + \
        ((uint32_t)(*(s + 2)) << 16) + \
        ((uint32_t)(*(s + 3)) << 24);    \
  s += 4;                                \
}

enum _RT_DEV_LOCK_E
{
  RT_DEV_LOCK_EFUSE  = 0,
  RT_DEV_LOCK_FLASH  = 1,
  RT_DEV_LOCK_CRYPTO = 2,
  RT_DEV_LOCK_PTA    = 3,
  RT_DEV_LOCK_WLAN   = 4,
  RT_DEV_LOCK_MAX    = 5
};

typedef struct
{
  uint32_t IQK_XX;
  uint32_t IQK_YY;
  uint16_t IDAC;
  uint16_t QDAC;
  uint16_t IDAC2;
  uint16_t QDAC2;
} IQK_T;

extern void     rtw_msleep_os(int ms);
extern void     rtw_mfree(uint8_t *pbuf, uint32_t n);
extern void     device_mutex_lock(uint32_t device);
extern void     device_mutex_unlock(uint32_t device);
extern uint32_t bt_lok_write(uint16_t idac,
                             uint16_t qdac, uint16_t idac2, uint16_t qdac2);
extern uint32_t bt_dck_write(uint8_t q_dck, uint8_t i_dck);
extern uint32_t bt_iqk_8710c(IQK_T *cal_data, uint8_t store);
extern uint32_t bt_flatk_8710c(uint16_t txgain_flatk);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t        hci_phy_efuse[BT_PHY_EFUSE_LEN] =
{
  0
};

static uint8_t        hci_lgc_efuse[BT_LGC_EFUSE_LEN] =
{
  0
};

static IQK_T          hci_iqk_data =
{
  0x100,
  0x00,
  0x20,
  0x20,
  0x20,
  0x20
};

static uint8_t        *rtl_actual_command = NULL;
static uint32_t rtl_actual_command_size;
extern const uint8_t  rtl_vendor_command[];
extern uint32_t rtl_vendor_command_size;
static uint32_t hci_cfg_baudrate;
static uint8_t rtl_vendor_init_config[] =
{
  0x55, 0xab, 0x23, 0x87,                                 /* header */

  0x32, 0x00,                                             /* Config length: header + len + preload */

  0x30, 0x00, 0x06, 0x99, 0x88, 0x77, 0x44, 0x55, 0x66,   /* BT MAC address */

  0x0c, 0x00, 0x04, 0x04, 0x50, 0xf7, 0x05,               /* Baudrate 921600 */

  0x18, 0x00, 0x01, 0x5c,                                 /* flow control */

  0x94, 0x01, 0x06, 0x0a, 0x08, 0x00, 0x00, 0x2e, 0x07,   /* phy flatk */

  0x9f, 0x01, 0x05, 0x2a, 0x2a, 0x2a, 0x2a, 0x1c,         /* unknow 1 */

  0xa4, 0x01, 0x04, 0xfe, 0xfe, 0xfe, 0xfe,               /* unknow 2 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static uint32_t cal_bit_shift(uint32_t mask)
{
  uint32_t i;
  for (i = 0; i < 31; i++)
    {
      if ((mask >> i) & 0x1)
        {
          break;
        }
    }

  return i;
}

static void set_reg_val(uint32_t reg, uint32_t mask, uint32_t val)
{
  if (reg % 4)
    {
      return;
    }

  uint32_t data = HAL_READ32(reg, 0);
  data = ((data & (~mask)) | (val << cal_bit_shift(mask)));
  HAL_WRITE32(reg, 0, data);
}

static int hci_iqk_phy_efuse_valid(FAR IQK_T           *iqk_data)
{
  if ((hci_phy_efuse[3] == 0xff) &&
      (hci_phy_efuse[4] == 0xff) &&
      (hci_phy_efuse[5] == 0xff) &&
      (hci_phy_efuse[6] == 0xff))
    {
      /* No Phy Efuse Data */

      return -EIO;
    }

  else
    {
      /* Phy Efuse Has Data */

      iqk_data->IQK_XX = hci_phy_efuse[3] | hci_phy_efuse[4] << 8;
      iqk_data->IQK_YY = hci_phy_efuse[5] | hci_phy_efuse[6] << 8;
      iqk_data->QDAC   = hci_phy_efuse[0x0c];
      iqk_data->IDAC   = hci_phy_efuse[0x0d];
      iqk_data->QDAC2  = hci_phy_efuse[0x0e];
      iqk_data->IDAC2  = hci_phy_efuse[0x0f];
      return 0;
    }
}

static int hci_iqk_lgc_efuse_valid(FAR IQK_T *iqk_data)
{
  if ((hci_lgc_efuse[0x16] == 0xff) &&
      (hci_lgc_efuse[0x17] == 0xff) &&
      (hci_lgc_efuse[0x18] == 0xff) &&
      (hci_lgc_efuse[0x19] == 0xff))
    {
      /* No Lgc Efuse Data */

      return -EIO;
    }
  else
    {
      /* Lgc Efuse Has Data */

      iqk_data->IQK_XX = (uint32_t)(((uint32_t)hci_lgc_efuse[0x17]) << 8) |
                         hci_lgc_efuse[0x16];
      iqk_data->IQK_YY = (uint32_t)(((uint32_t)hci_lgc_efuse[0x19]) << 8) |
                         hci_lgc_efuse[0x18];
      iqk_data->QDAC   = hci_lgc_efuse[0x1a];
      iqk_data->IDAC   = hci_lgc_efuse[0x1b];
      iqk_data->QDAC2  = hci_lgc_efuse[0x1c];
      iqk_data->IDAC2  = hci_lgc_efuse[0x1d];
      return 0;
    }
}

int hci_check_iqk(void)
{
  IQK_T iqk_data;

  if (!hci_lgc_efuse[LEFUSE(0x1a1)] & BIT0)
    {
      /* Use Fix Logic Efuse */

      if (0 == hci_iqk_lgc_efuse_valid(&iqk_data))
        {
          bt_lok_write(iqk_data.IDAC, iqk_data.QDAC, iqk_data.IDAC2,
                       iqk_data.QDAC2);
          hci_phy_efuse[0] = 0;
          hci_phy_efuse[1] = hci_phy_efuse[1] & (~BIT0);
          hci_phy_efuse[3] = iqk_data.IQK_XX & 0xff;
          hci_phy_efuse[4] = (iqk_data.IQK_XX >> 8) & 0xff;
          hci_phy_efuse[5] = iqk_data.IQK_YY & 0xff;
          hci_phy_efuse[6] = (iqk_data.IQK_YY >> 8) & 0xff;
          return 0;
        }

      /* No Logic Efuse Data */

      return -EIO;
    }

  if (0 == hci_iqk_phy_efuse_valid(&iqk_data))
    {
      if (hci_phy_efuse[0] != 0)
        {
          bt_dck_write(hci_phy_efuse[0x0e], hci_phy_efuse[0x0f]);
        }

      bt_lok_write(iqk_data.IDAC, iqk_data.QDAC, iqk_data.IDAC2,
                   iqk_data.QDAC2);
      return 0;
    }

  else if (0 == hci_iqk_lgc_efuse_valid(&iqk_data))
    {
      bt_lok_write(iqk_data.IDAC, iqk_data.QDAC, iqk_data.IDAC2,
                   iqk_data.QDAC2);
      hci_phy_efuse[0]    = 0;
      hci_phy_efuse[1]    = hci_phy_efuse[1] & (~BIT0);
      hci_phy_efuse[3]    = iqk_data.IQK_XX & 0xff;
      hci_phy_efuse[4]    = (iqk_data.IQK_XX >> 8) & 0xff;
      hci_phy_efuse[5]    = iqk_data.IQK_YY & 0xff;
      hci_phy_efuse[6]    = (iqk_data.IQK_YY >> 8) & 0xff;
      hci_phy_efuse[0x0e] = hci_lgc_efuse[0x1e];
      hci_phy_efuse[0x0f] = hci_lgc_efuse[0x1f];
      bt_dck_write(hci_phy_efuse[0x0e], hci_phy_efuse[0x0f]);
      return 0;
    }

  else
    {
      /* No IQK LOK Data, Need Start LOK */

      return -EIO;
    }
}

int hci_start_iqk(void)
{
  if (_FAIL == bt_iqk_8710c(&hci_iqk_data, 0))
    {
      /* IQK Fail, Ensure Connected */

      return -EIO;
    }

  bt_lok_write(hci_iqk_data.IDAC, hci_iqk_data.QDAC, hci_iqk_data.IDAC2,
               hci_iqk_data.QDAC2);
  hci_phy_efuse[0] = 0;
  hci_phy_efuse[1] = hci_phy_efuse[1] & (~BIT0);
  hci_phy_efuse[3] = hci_iqk_data.IQK_XX & 0xff;
  hci_phy_efuse[4] = (hci_iqk_data.IQK_XX >> 8) & 0xff;
  hci_phy_efuse[5] = hci_iqk_data.IQK_YY & 0xff;
  hci_phy_efuse[6] = (hci_iqk_data.IQK_YY >> 8) & 0xff;
  return 0;
}

int hci_set_init_config_mac(FAR uint8_t *addr, uint8_t diffvalue)
{
  for (uint8_t i = 0; i < BT_MAC_ADDR_LEN; i++)
    {
      rtl_vendor_init_config[9 + i] = addr[(BT_MAC_ADDR_LEN - 1) - i];
    }

  rtl_vendor_init_config[9] -= diffvalue;
  return 0;
}

int hci_get_baudrate(FAR uint32_t *bt_baudrate, FAR uint32_t *uart_baudrate)
{
  typedef struct
  {
    uint32_t bt_baudrate;
    uint32_t uart_baudrate;
  }baudrate_map;

  const baudrate_map maps[] =
  {
    {0x0000701d, 115200},
    {0x0252c00a, 230400},
    {0x05f75004, 921600},
    {0x00005004, 1000000},
    {0x04928002, 1500000},
    {0x00005002, 2000000},
    {0x0000b001, 2500000},
    {0x04928001, 3000000},
    {0x052a6001, 3500000},
    {0x00005001, 4000000},
  };

  uint32_t i;
  *bt_baudrate = hci_cfg_baudrate;
  for (i = 0; i < sizeof(maps); i++)
    {
      if (*bt_baudrate  == maps[i].bt_baudrate)
        {
          break;
        }
    }

  if (i == sizeof(maps))
    {
      return -EINVAL;
    }

  *uart_baudrate = maps[i].uart_baudrate;
  return 0;
}

int hci_find_fw_patch(uint8_t chipid)
{
#if 0
  const uint8_t single_patch_signature[4] =
  {
    0xfd,
    0x63,
    0x05,
    0x62
  };

  const uint8_t merged_patch_signature[8] =
  {
    0x52,
    0x65,
    0x61,
    0x6c,
    0x74,
    0x65,
    0x63,
    0x68
  };

  phal_spic_adaptor_t flash;

  /* FIXME: Distiguish Normal and MP Ptach (rltk_bt_get_patch_code) */

  const uint8_t *fw_patch        = rtl_vendor_command;
  uint32_t fw_patch_len    = rtl_vendor_command_size;
  uint32_t fw_patch_offset = 0;
  uint32_t lmp_subversion  = 0;
  uint16_t num_of_patch    = 0;
  uint16_t chipid_in_fw    = 0;
  uint8_t i;
  if (!CHECK_SW(EFUSE_SW_USE_FLASH_PATCH))
    {
      /* Check flash img */

      uint8_t tmp_patch_head[8];
      hal_flash_stream_read(&flash, MERGE_PATCH_ADDRESS, 8, tmp_patch_head);
      if (!memcmp(tmp_patch_head, merged_patch_signature,
                  sizeof(merged_patch_signature)))
        {
          /* use the changed patch */

          hal_flash_stream_read(&flash, MERGE_PATCH_ADDRESS + 8, 4,
                                (uint8_t *)&lmp_subversion);
          hal_flash_stream_read(&flash, MERGE_PATCH_ADDRESS + 12, 2,
                                (uint8_t *)&num_of_patch);
          for (i = 0 ; i < num_of_patch; i++)
            {
              hal_flash_stream_read(&flash,
                                    MERGE_PATCH_ADDRESS + 0x0e + 2 * i, 2,
                                    (uint8_t *)&chipid_in_fw);
              if (chipid_in_fw == chipid)
                {
                  hal_flash_stream_read(&flash,
                                        MERGE_PATCH_ADDRESS + 0x0e \
                                        + 2 * num_of_patch + 2 * i,
                                        2, (uint8_t *)&fw_patch_len);
                  hal_flash_stream_read(&flash,
                                        MERGE_PATCH_ADDRESS + 0x0e \
                                        + 4 * num_of_patch + 4 * i,
                                        4, (uint8_t *)&fw_patch_offset);
                  break;
                }
            }

          rtl_actual_command = (uint8_t *)rtw_zmalloc(fw_patch_len);
          if (!rtl_actual_command)
            {
              return -EIO;
            }

          hal_flash_stream_read(&flash,
                                MERGE_PATCH_ADDRESS + fw_patch_offset,
                                fw_patch_len, rtl_actual_command);
          LE_UINT32_TO_ARRAY(rtl_actual_command + fw_patch_len - 4,
                             lmp_subversion);
          return 0;
        }
    }

  if (!memcmp(fw_patch, single_patch_signature,
              sizeof(single_patch_signature)))
    {
      /* Use Single Patch, Do Nothing */

      rtl_actual_command      = fw_patch;
      rtl_actual_command_size = fw_patch_len;
    }

  else if (!memcmp(fw_patch, merged_patch_signature,
                   sizeof(merged_patch_signature)))
    {
      /* Use Merged Patch */

      LE_ARRAY_TO_UINT32(lmp_subversion, fw_patch + 0x08);
      LE_ARRAY_TO_UINT16(num_of_patch, fw_patch + 0x0c);
      for (i = 0; i < num_of_patch; i++)
        {
          LE_ARRAY_TO_UINT16(chipid_in_fw, fw_patch + 0x0e + 2 * i);
          if (chipid_in_fw == chipid)
            {
              LE_ARRAY_TO_UINT16(fw_patch_len,
                                 fw_patch + 0x0e + 2 * num_of_patch + 2 * i);
              LE_ARRAY_TO_UINT32(fw_patch_offset,
                                 fw_patch + 0x0e + 4 * num_of_patch + 4 * i);
              break;
            }
        }

      if (i >= num_of_patch)
        {
          /* No Match Patch Found */

          return -EIO;
        }

      else
        {
          rtl_actual_command      = (uint8_t *)rtw_zmalloc(fw_patch_len);
          rtl_actual_command_size = fw_patch_len;
          if (!rtl_actual_command)
            {
              /* Malloc rtl_actual_command failed */

              return -EIO;
            }

          else
            {
              rtw_memcpy(rtl_actual_command, fw_patch + fw_patch_offset,
                         fw_patch_len);
              LE_UINT32_TO_ARRAY(rtl_actual_command + fw_patch_len - 4,
                                 lmp_subversion);
            }
        }
    }

  else
    {
      /* Something is Wrong with the Patch */

      return -EIO;
    }

#else

  /* FIXME: Dummy Here */

  rtl_actual_command      = (uint8_t *)&rtl_vendor_command[0];
  rtl_actual_command_size = rtl_vendor_command_size;
#endif
  return 0;
}

int hci_fetch_command(FAR uint8_t *command)
{
  unsigned int config_size = sizeof(rtl_vendor_init_config);
  static unsigned int command_offset;
  int fragment_size = 0;
  int index;
  if (command_offset >= config_size + rtl_actual_command_size)
    {
      if (rtl_actual_command && rtl_actual_command != rtl_vendor_command)
        {
          rtw_mfree(rtl_actual_command, rtl_actual_command_size);
        }

      return AMEBAZ_COMMAND_DONE;
    }

  if (command_offset < rtl_actual_command_size)
    {
      if (command_offset + AMEBAZ_COMMAND_FRAGMENT_SIZE
                                            > rtl_actual_command_size)
        {
          fragment_size = rtl_actual_command_size - command_offset;
        }

      else
        {
          fragment_size = AMEBAZ_COMMAND_FRAGMENT_SIZE;
        }

      memcpy(command + 5, rtl_actual_command + command_offset,
             fragment_size);
      command_offset += fragment_size;
    }

  if (command_offset >= rtl_actual_command_size)
    {
      int config_offset = command_offset - rtl_actual_command_size;
      int config_len = config_size - config_offset;
      if (fragment_size < AMEBAZ_COMMAND_FRAGMENT_SIZE)
        {
          int free = AMEBAZ_COMMAND_FRAGMENT_SIZE - fragment_size;
          int copy_size;
          if (config_len > free)
            {
              copy_size = free;
            }

          else
            {
              copy_size = config_len;
            }

          memcpy(command + 5 + fragment_size,
                 rtl_vendor_init_config + config_offset, copy_size);
          command_offset += copy_size;
          fragment_size += copy_size;
        }
    }

  index = (command_offset / AMEBAZ_COMMAND_FRAGMENT_SIZE) - 1;
  if (command_offset % AMEBAZ_COMMAND_FRAGMENT_SIZE > 0)
    {
      index++;
    }

  if (command_offset >= config_size + rtl_actual_command_size)
    {
      index |= 0x80;
    }

  command[3] = fragment_size + 1;
  command[4] = index;
  return AMEBAZ_COMMAND_VALID;
}

int hci_get_efuse_iqk_data(uint8_t *data)
{
  data[0] = 0x0c;
  uint8_t *iqk_data = data + 1;
  memcpy(iqk_data, hci_phy_efuse, data[0]);
  return 0;
}

static void hci_read_efuse(void)
{
  uint16_t bt_phy_efuse_base = 0x100;
  device_mutex_lock(RT_DEV_LOCK_EFUSE);

  /* Read Phy Efuse */

  for (int i = 0; i < 16; i++)
    {
      hal_efuse_read(bt_phy_efuse_base + i, hci_phy_efuse + i,
                     LDO_OUT_DEFAULT_VOLT);
    }

  hal_efuse_read(0xf8, hci_phy_efuse + 16, LDO_OUT_DEFAULT_VOLT);
  hal_efuse_read(0xf9, hci_phy_efuse + 17, LDO_OUT_DEFAULT_VOLT);

  /* Read Logic Efuse */

  ameba_efuse_logical_read(BT_LGC_EFUSE_OFFSET, BT_LGC_EFUSE_LEN,
                           hci_lgc_efuse);

  device_mutex_unlock(RT_DEV_LOCK_EFUSE);
}

static int hci_parse_config(void)
{
  uint32_t signature;
  uint16_t payload_len;
  uint16_t entry_offset;
  uint16_t entry_len;
  uint8_t *p_entry;
  uint8_t *p;
  uint8_t *p_len;
  uint8_t i;
  uint16_t tx_flatk;
  p = rtl_vendor_init_config;
  p_len = rtl_vendor_init_config + 4;
  LE_STREAM_TO_UINT32(signature, p);
  LE_STREAM_TO_UINT16(payload_len, p);
  if (signature != BT_CONFIG_SIGNATURE)
    {
      /* Invalid Signature */

      return -EIO;
    }

  if (payload_len != sizeof(rtl_vendor_init_config) - BT_CONFIG_HEADER_LEN)
    {
      /* Fix the len, just avoid the length is not corect */

      LE_UINT16_TO_STREAM(p_len,
                    sizeof(rtl_vendor_init_config) - BT_CONFIG_HEADER_LEN);
    }

  p_entry = rtl_vendor_init_config + BT_CONFIG_HEADER_LEN;
  while (p_entry < rtl_vendor_init_config + sizeof(rtl_vendor_init_config))
    {
      p = p_entry;
      LE_STREAM_TO_UINT16(entry_offset, p);
      LE_STREAM_TO_UINT8(entry_len, p);
      p_entry = p + entry_len;
      switch (entry_offset)
        {
        case 0x000c:

          /* FIXME: MP (If use mp, Set badurate 115200
           * in rtl_vendor_init_config)
           */

          LE_STREAM_TO_UINT32(hci_cfg_baudrate, p);
          break;
        case 0x0018:

          /* FIXME: MP (If use mp, Clear flowctl in rtl_vendor_init_config) */

          break;
        case 0x0030:

          /* FIXME: Customer use Wi-Fi MAC - 1 as BT ADDR,
           * so ignore action here
           */

#if 0
          if (entry_len == 6)
            {
              if ((hci_lgc_efuse[0] != 0xff) && (hci_lgc_efuse[1] != 0xff))
                {
                  for (uint8_t i = 0 ; i < 6; i++)
                    {
                      p[i] = hci_lgc_efuse[5 - i];
                    }
                }
            }

#endif
          break;
        case 0x194:
          if (hci_lgc_efuse[LEFUSE(0x196)] == 0xff)
            {
              if (!(hci_phy_efuse[2] & BIT0))
                {
                  tx_flatk = hci_phy_efuse[0xa] | hci_phy_efuse[0xb] << 8;
                  bt_flatk_8710c(tx_flatk);
                }

              break;
            }

          else
            {
              p[0] = hci_lgc_efuse[LEFUSE(0x196)];
              if (hci_lgc_efuse[LEFUSE(0x196)] & BIT1)
                {
                  p[1] = hci_lgc_efuse[LEFUSE(0x197)];
                }

              if (hci_lgc_efuse[LEFUSE(0x196)] & BIT2)
                {
                  p[2] = hci_lgc_efuse[LEFUSE(0x198)];
                  p[3] = hci_lgc_efuse[LEFUSE(0x199)];
                  tx_flatk = hci_lgc_efuse[LEFUSE(0x198)]
                             | hci_lgc_efuse[LEFUSE(0x199)] << 8;
                  bt_flatk_8710c(tx_flatk);
                }

              else
                {
                  if (!(hci_phy_efuse[2] & BIT0))
                    {
                      tx_flatk = hci_phy_efuse[0xa]
                                 | hci_phy_efuse[0xb] << 8;
                      bt_flatk_8710c(tx_flatk);
                    }
                }

              if (hci_lgc_efuse[LEFUSE(0x196)] & BIT5)
                {
                  p[4] = hci_lgc_efuse[LEFUSE(0x19a)];
                  p[5] = hci_lgc_efuse[LEFUSE(0x19b)];
                }
            }

          break;
        case 0x19f:
          for (i = 0; i < entry_len; i ++)
            {
              if (hci_lgc_efuse[LEFUSE(0x19c + i)] != 0xff)
                {
                  p[i] = hci_lgc_efuse[LEFUSE(0x19c + i)];
                }
            }

          break;
        case 0x1a4:
          for (i = 0; i < entry_len; i ++)
            {
              if (hci_lgc_efuse[LEFUSE(0x1a2 + i)] != 0xff)
                {
                  p[i] = hci_lgc_efuse[LEFUSE(0x1a2 + i)];
                }
            }

          break;
        default:
          break;
        }
    }

  return 0;
}

int hci_board_init(void)
{
  /* FIXME: Wi-Fi Coexist, MP, Trace_Setting */

  hci_read_efuse();
  if (!CHECK_SW(EFUSE_SW_BT_FW_LOG))
    {
      set_reg_val(0x400000cc, BIT2 | BIT1 | BIT0, 6);
      rtw_msleep_os(5);
      set_reg_val(0x400000cc, BIT8, 1);
      rtw_msleep_os(5);
    }

  if (hci_parse_config())
    {
      return -EIO;
    }

  return 0;
}

int hci_board_init_done(void)
{
  /* FIXME: MP */

  return 0;
}

