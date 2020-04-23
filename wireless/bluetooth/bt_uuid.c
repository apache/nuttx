/****************************************************************************
 * wireless/bluetooth/bt_att.c
 *  Bluetooth UUID handling.
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

#include <string.h>
#include <errno.h>

#include <nuttx/wireless/bluetooth/bt_uuid.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* TODO: Decide whether to continue using BLE format or switch to RFC 4122 */

static const struct bt_uuid_s g_uuid128_base =
{
  .type   = BT_UUID_128,
  .u.u128 =
  {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void uuid_to_uuid128(FAR const struct bt_uuid_s *src,
                            FAR struct bt_uuid_s *dst)
{
  switch (src->type)
    {
    case BT_UUID_16:
      *dst = g_uuid128_base;
      memcpy(&dst->u.u128[2], &src->u.u16, sizeof(src->u.u16));
      return;

    case BT_UUID_128:
      memcpy(dst, src, sizeof(*dst));
      return;
    }
}

static int uuid128_cmp(FAR const struct bt_uuid_s *u1,
                       FAR const struct bt_uuid_s *u2)
{
  struct bt_uuid_s uuid1;
  struct bt_uuid_s uuid2;

  uuid_to_uuid128(u1, &uuid1);
  uuid_to_uuid128(u2, &uuid2);

  return memcmp(uuid1.u.u128, uuid2.u.u128, sizeof(uuid1.u.u128));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_uuid_cmp(FAR const struct bt_uuid_s *u1,
                FAR const struct bt_uuid_s *u2)
{
  /* Convert to 128 bit if types don't match */

  if (u1->type != u2->type)
    {
      return uuid128_cmp(u1, u2);
    }

  switch (u1->type)
    {
    case BT_UUID_16:
      return (int)u1->u.u16 - (int)u2->u.u16;

    case BT_UUID_128:
      return memcmp(u1->u.u128, u2->u.u128, sizeof(u1->u.u128));
    }

  return -EINVAL;
}
