/****************************************************************************
 * wireless/bluetooth/bt_keys.h
 * Bluetooth key handling
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS
 * ; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum
{
  BT_KEYS_SLAVE_LTK   = (1 << 0),
  BT_KEYS_IRK         = (1 << 1),
  BT_KEYS_LTK         = (1 << 2),
  BT_KEYS_LOCAL_CSRK  = (1 << 3),
  BT_KEYS_REMOTE_CSRK = (1 << 4),

  BT_KEYS_ALL         = (BT_KEYS_SLAVE_LTK | BT_KEYS_IRK | BT_KEYS_LTK |
                         BT_KEYS_LOCAL_CSRK | BT_KEYS_REMOTE_CSRK),
};

struct bt_ltk_s
{
  uint64_t rand;
  uint16_t ediv;
  uint8_t val[16];
  FAR struct bt_keys_s *next;
};

struct bt_irk_s
{
  uint8_t val[16];
  bt_addr_t rpa;
  FAR struct bt_keys_s *next;
};

struct bt_csrk_s
{
  uint8_t val[16];
  uint32_t cnt;
  FAR struct bt_keys_s *next;
};

struct bt_keys_s
{
  bt_addr_le_t addr;
  int keys;
  struct bt_ltk_s slave_ltk;
  struct bt_ltk_s ltk;
  struct bt_irk_s irk;
  struct bt_csrk_s local_csrk;
  struct bt_csrk_s remote_csrk;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct bt_keys_s *bt_keys_get_addr(FAR const bt_addr_le_t *addr);
FAR struct bt_keys_s *bt_keys_get_type(int type,
                                       FAR const bt_addr_le_t *addr);
void bt_keys_add_type(FAR struct bt_keys_s *keys, int type);
void bt_keys_clear(FAR struct bt_keys_s *keys, int type);
FAR struct bt_keys_s *bt_keys_find(int type, FAR const bt_addr_le_t *addr);
FAR struct bt_keys_s *bt_keys_find_irk(FAR const bt_addr_le_t *addr);
