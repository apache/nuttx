/****************************************************************************
 * net/bt/host/classic/did.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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
 ****************************************************************************/

/* Copyright 2025 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/classic/sdp.h>

#include "did_internal.h"

#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_did);

#define DID_VER_1_3 (0x0103u)

#define BLUETOOTH_DEVICE_IDENTIFY_SPEC_VERSION 0x0103
#define BLUETOOTH_DEVICE_IDENTIFY_ATTR_VERSION 0x0100

#define BLUETOOTH_DEVICE_VENDOR_ID_SOURCE_BTSIG 0x0001 /* Bluetooth SIG assigned */
#define BLUETOOTH_DEVICE_VENDOR_ID_SOURCE_USBIF 0x0002 /* USB Implementer's Forum */

static struct bt_sdp_attribute did_attrs[] =
{
  BT_SDP_NEW_SERVICE,
  BT_SDP_LIST(BT_SDP_ATTR_SVCLASS_ID_LIST,
    BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 3),
    BT_SDP_DATA_ELEM_LIST(
        {
          BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
          BT_SDP_ARRAY_16(BT_SDP_PNP_INFO_SVCLASS)
        },

    )
  ),

  BT_SDP_LIST(BT_SDP_ATTR_SPECIFICATION_ID,
    BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
    BT_SDP_ARRAY_16(BLUETOOTH_DEVICE_IDENTIFY_SPEC_VERSION)),
  BT_SDP_LIST(BT_SDP_ATTR_VENDOR_ID, BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
    BT_SDP_ARRAY_16(CONFIG_BT_DEVICE_VENDOR_ID)),
  BT_SDP_LIST(BT_SDP_ATTR_PRODUCT_ID, BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
    BT_SDP_ARRAY_16(CONFIG_BT_DEVICE_PRODUCT_ID)),
  BT_SDP_LIST(BT_SDP_ATTR_VERSION, BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
    BT_SDP_ARRAY_16(CONFIG_BT_DEVICE_VERSION)),
  BT_SDP_LIST(BT_SDP_ATTR_PRIMARY_RECORD,
    BT_SDP_TYPE_SIZE(BT_SDP_BOOL),
    BT_SDP_ARRAY_8(1)),
  BT_SDP_LIST(BT_SDP_ATTR_VENDOR_ID_SOURCE,
    BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
    BT_SDP_ARRAY_16(BLUETOOTH_DEVICE_VENDOR_ID_SOURCE_BTSIG)),
};

static struct bt_sdp_record did_rec = BT_SDP_RECORD(did_attrs);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_did_init(void)
{
  __maybe_unused int err;

  static bool initialized;

  if (initialized)
    {
      return;
    }

  err = bt_sdp_register_service(&did_rec);
  if (err != 0)
    {
      LOG_ERR("Failed to register DID SDP record (err %d)", err);
      return;
    }

  initialized = true;
}
