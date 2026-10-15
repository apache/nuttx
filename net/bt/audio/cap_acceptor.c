/****************************************************************************
 * net/bt/audio/cap_acceptor.c
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

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/audio/tbs.h>
#include <nuttx/net/bluetooth/audio/csip.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>

#include "cap_internal.h"

LOG_MODULE_REGISTER(bt_cap_acceptor, CONFIG_BT_CAP_ACCEPTOR_LOG_LEVEL);

#if defined(CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER)

static struct bt_gatt_attr svc_attrs[] =
{
        BT_GATT_PRIMARY_SERVICE(BT_UUID_CAS), BT_GATT_INCLUDE_SERVICE(NULL) /* To be overwritten */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_cap_acceptor_register(
  const struct bt_csip_set_member_register_param *param,
  struct bt_csip_set_member_svc_inst **svc_inst)
{
  static struct bt_gatt_service cas;
  int err;

  if (param == NULL)
    {
      LOG_DBG("param is NULL");

      return -EINVAL;
    }

  if (param->set_size == 0U)
    {
      LOG_DBG("param->set_size shall be non-zero");
      return -EINVAL;
    }

  if (param->rank == 0U)
    {
      LOG_DBG("param->rank shall be non-zero");
      return -EINVAL;
    }

  if (svc_inst == NULL)
    {
      LOG_DBG("svc_inst is NULL");

      return -EINVAL;
    }

  err = bt_csip_set_member_register(param, svc_inst);
  if (err != 0)
    {
      LOG_DBG("Failed to register CSIP");
      return err;
    }

  cas = (struct bt_gatt_service)BT_GATT_SERVICE(svc_attrs);

  /* Overwrite the include definition with the CSIP */

  cas.attrs[1].user_data = bt_csip_set_member_svc_decl_get(*svc_inst);

  err = bt_gatt_service_register(&cas);
  if (err != 0)
    {
      const int csip_err = bt_csip_set_member_unregister(*svc_inst);

      if (csip_err)
        {
          LOG_ERR("Failed to unregister CSIS: %d", csip_err);
        }

      cas.attrs[1].user_data = NULL;
      LOG_DBG("Failed to register CAS");

      return err;
    }

  return 0;
}

#else /* CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER */

BT_GATT_SERVICE_DEFINE(cas_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_CAS)

/* TODO: Add support for included CSIP */

);

#endif /* CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER */

bool bt_cap_acceptor_ccid_exist(const struct bt_conn *conn, uint8_t ccid)
{
  if (IS_ENABLED(CONFIG_BT_TBS_CLIENT_CCID) &&
          bt_tbs_client_get_by_ccid(conn,
          ccid) != NULL)
    {
      return true;
    }

  /* TODO: check mcs */

  return false;
}

bool bt_cap_acceptor_ccids_exist(const struct bt_conn *conn,
                                 const uint8_t ccids[],
                                 uint8_t ccid_cnt)
{
  for (uint8_t i = 0U; i < ccid_cnt; i++)
    {
      const uint8_t ccid = ccids[i];

      if (!bt_cap_acceptor_ccid_exist(conn, ccid))
        {
          LOG_DBG("CCID %u is unknown", ccid);

          /* TBD:
           * Should we reject the Metadata?
           *
           * Should unknown CCIDs trigger a discovery procedure for TBS or
           * MCS?
           *
           * Or should we just accept as is, and then let the application
           * decide?
           */

          return false;
        }
    }

  /* This will also return true if the ccid_cnt is 0 which is intended */

  return true;
}
