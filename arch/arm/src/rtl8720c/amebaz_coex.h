/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_coex.h
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
/*  READ_ME
 * <b>Example usage</b>
 * \code{ble_app_main.c}
 *
 * void app_le_gap_init(void)
 * {
 *    ....
 *    bt_coex_init();
 * }
 *
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_SW_MAILBOX_SCAN_OFF  0x00
#define BT_SW_MAILBOX_SCAN_ON   0x01

typedef struct net_buf_simple_s
{
  /* * Pointer to the start of data in the buffer. */

  uint8_t *data;

  /* * Length of the data behind the data pointer. */

  uint16_t len;

  /* * Amount of data that this buffer can store. */

  uint16_t size;
  /** Start of the data storage. Not to be accessed directly
   *  (the data pointer should be used instead).
   */

  uint8_t *__buf;
} net_buf_simple;

void bt_coex_handle_cmd_complete_evt(uint16_t opcode,
                                              uint16_t cause,
                                              uint8_t total_len,
                                              uint8_t *p);
void bt_coex_handle_specific_evt(uint8_t *p, uint8_t len);
bool bt_coex_handle_xiaomi_evt(net_buf_simple *xiaomi_buf);
unsigned int bt_coex_sw_mailbox_set(unsigned int mailbox_control);
void bt_coex_init(void);
unsigned int send_coex_mailbox_to_wifi_from_btapp(uint8_t state);

