/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_coex.c
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

#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include "amebaz_coex.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_VENDOR_MAILBOX_CMD 0xfc8f

/* static net_buf_simple *Xiaomi_buf; */

struct rtl_btinfo
{
  uint8_t cmd;
  uint8_t len;
  uint8_t data[6];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

unsigned int send_coex_mailbox_to_wifi_from_btapp(uint8_t state)
{
  uint8_t para[8];
  para[0] = 0x45; /* Mailbox ID */

  para[1] = state; /* Data0 */

  para[2] = 0; /* Data1 */

  para[3] = 0; /* Data2 */

  para[4] = 0; /* Data3 */

  para[5] = 0; /* Data4 */

  para[6] = 0; /* Data5 */

  para[7] = 0; /* Data6 */

  rltk_coex_mailbox_to_wifi(para, 8);
  return 1;
}

static void rtk_notify_info_to_wifi(uint8_t length, uint8_t *report_info)
{
  struct rtl_btinfo *report = (struct rtl_btinfo *)(report_info);
  if (length)
    {
      printf("bt info: cmd %2.2X", report->cmd);
      printf("bt info: len %2.2X", report->len);
      printf("bt info: data %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X",
             report->data[0], report->data[1], report->data[2],
             report->data[3], report->data[4], report->data[5]);
    }

  rltk_coex_mailbox_to_wifi(report_info, report->len + 2);

  /* send BT INFO to Wi-Fi driver */
}

void bt_coex_handle_cmd_complete_evt(uint16_t opcode, uint16_t cause,
                                     uint8_t total_len, uint8_t *p)
{
  if (opcode == HCI_VENDOR_MAILBOX_CMD)
    {
      status = *p++; /* jump the double subcmd */

      total_len--;
      if (total_len <= 1)
        {
          printf("bt_coex_handle_cmd_complete_evt: not report to wifi");
          return ;
        }

      rltk_coex_mailbox_to_wifi(p, total_len);

      /* rtk_parse_vendor_mailbox_cmd_evt(p, total_len, status); */
    }
}

void bt_coex_handle_specific_evt(uint8_t *p, uint8_t len)
{
  rltk_coex_mailbox_to_wifi(p, len);
}

static const char *bt_hex_real(const void *buf, size_t len)
{
  static const char hex[] = "0123456789abcdef";
  static char str[129];
  const uint8_t *b = buf;
  size_t i;
  len = MIN(len, (sizeof(str) - 1) / 2);
  for (i = 0; i < len; i++)
    {
      str[i * 2]     = hex[b[i] >> 4];
      str[i * 2 + 1] = hex[b[i] & 0xf];
    }

  str[i * 2] = '\0';
  return str;
}

static inline char *log_strdup(const char *str)
{
  return (char *)str;
}

#define bt_hex(buf, len) log_strdup(bt_hex_real(buf, len))
static void bt_coex_dump_buf(net_buf_simple *tmp_buf)
{
  printf("\n\r[%s] len=%d, buf = % s\n\r", __func__,
                                           tmp_buf->len,
                                           bt_hex(tmp_buf->data,
                                           tmp_buf->len));
}

static int bt_coex_unpack_xiaomi_vendor_cmd(net_buf_simple *tmp_buf)
{
  if (tmp_buf->data[0] == 0x25 && tmp_buf->data[1] == 0x00)
    {
      tmp_buf->data += 2;
      tmp_buf->len -= 2;
      return 1;
    }

  else
    {
      printf("[rtk_coex]Xiaomi vendor header not match.\n\r");
      return -1;
    }
}

bool bt_coex_handle_xiaomi_evt(net_buf_simple *xiaomi_buf)
{
  /* bt_coex_dump_buf(xiaomi_buf); */

  bt_coex_unpack_xiaomi_vendor_cmd(xiaomi_buf);

  /* bt_coex_dump_buf(xiaomi_buf); */

  rltk_coex_mailbox_to_wifi(xiaomi_buf->data, xiaomi_buf->len);
  return true;
}

typedef struct bt_sw_mailbox_info_s
{
  uint8_t data[8];
}bt_sw_mailbox_info_t;

static bt_sw_mailbox_info_t scan_enable;

unsigned int bt_coex_sw_mailbox_set(unsigned int mailbox_control)
{
#if 0 /* This function need to be removed */

  uint8_t mailbox_len = 8;
  memset(&scan_enable, 0, sizeof(scan_enable));
  switch (mailbox_control)
    {
    case BT_SW_MAILBOX_SCAN_OFF:
      scan_enable.data[0] = 0x27;
      scan_enable.data[1] = 6;
      rtk_notify_info_to_wifi(mailbox_len, scan_enable.data);
      break;
    case BT_SW_MAILBOX_SCAN_ON:
      scan_enable.data[0] = 0x27;
      scan_enable.data[1] = 6;
      scan_enable.data[5] = (0x0 | 0x1 << 5); /* BT scan EN bit */

      rtk_notify_info_to_wifi(mailbox_len, scan_enable.data);
      break;
    default:
      printf("[Err %s]No such sw mailbox command.\n\r", __func__);
      break;
    }

  return true;
#else
  return true;
#endif
}

void bt_coex_init(void)
{
  vendor_cmd_init(NULL);
}

