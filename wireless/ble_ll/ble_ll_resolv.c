/****************************************************************************
 * wireless/ble_ll/ble_ll_resolv.c
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll_resolv.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_ll.h"
#include "ble_ll_resolv.h"
#include "ble_ll_hci.h"
#include "ble_ll_scan.h"
#include "ble_ll_adv.h"
#include "ble_ll_sync.h"
#include "ble_hw.h"
#include "ble_ll_conn_priv.h"
#include "ble_ll_priv.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
struct ble_ll_resolv_data
{
  uint8_t addr_res_enabled;
  uint8_t rl_size;
  uint8_t rl_cnt_hw;
  uint8_t rl_cnt;
  clock_t rpa_tmo;
  struct ble_ll_event_s rpa_timer;
};

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
struct ble_ll_resolv_data g_ble_ll_resolv_data;

__attribute__((aligned(4))) struct ble_ll_resolv_entry
    g_ble_ll_resolv_list[CONFIG_BLE_LL_RESOLV_LIST_SIZE];

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: ble_ll_resolv_list_chg_allowed
 *
 * Description:
 *   Called to determine if a change is allowed to the resolving list at this
 *   time. We are not allowed to modify the resolving list if address
 * translation is enabled and we are either scanning, advertising, or
 * attempting to create a connection.
 *
 * Returned Value:
 *   int 0: not allowed. 1: allowed.
 *
 ****************************************************************************/

static int ble_ll_resolv_list_chg_allowed(void)
{
  int rc;

  if (g_ble_ll_resolv_data.addr_res_enabled &&
      ble_ll_is_busy(BLE_LL_BUSY_EXCLUDE_CONNECTIONS))
    {
      rc = 0;
    }
  else
    {
      rc = 1;
    }

  return rc;
}

/****************************************************************************
 * Name: generate_rpa
 *
 ****************************************************************************/

static void generate_rpa(const uint8_t *irk, uint8_t *rpa)
{
  uint8_t *prand;
  struct ble_encryption_block ecb;

  /* Get prand */

  prand = rpa + 3;
  ble_ll_rand_prand_get(prand);

  /* Calculate hash, hash = ah(local IRK, prand) */

  memcpy(ecb.key, irk, 16);
  memset(ecb.plain_text, 0, 13);
  ecb.plain_text[13] = prand[2];
  ecb.plain_text[14] = prand[1];
  ecb.plain_text[15] = prand[0];

  /* Calculate hash */

  ble_hw_encrypt_block(&ecb);

  rpa[0] = ecb.cipher_text[15];
  rpa[1] = ecb.cipher_text[14];
  rpa[2] = ecb.cipher_text[13];
}

/****************************************************************************
 * Name: ble_ll_resolv_gen_priv_addr
 *
 * Description:
 *   Called to generate a resolvable private address in rl structure
 *
 * Input Parameters:
 *   rl
 *   local
 *
 ****************************************************************************/

static void ble_ll_resolv_gen_priv_addr(struct ble_ll_resolv_entry *rl,
                                        int local)
{
  uint8_t *irk;
  uint8_t *addr;

  DEBUGASSERT(rl != NULL);

  if (local)
    {
      addr = rl->rl_local_rpa;
      irk = rl->rl_local_irk;
    }
  else
    {
      addr = rl->rl_peer_rpa;
      irk = rl->rl_peer_irk;
    }

  generate_rpa(irk, addr);
}

/****************************************************************************
 * Name: ble_ll_resolv_rpa_timer_cb
 *
 * Description:
 *   Called when the Resolvable private address timer expires. This timer
 *   is used to regenerate local and peers RPA's in the resolving list.
 *
 ****************************************************************************/

static void ble_ll_resolv_rpa_timer_cb(FAR void *arg)
{
  int i;
  irqstate_t sr;
  struct ble_ll_resolv_entry *rl;

  rl = &g_ble_ll_resolv_list[0];
  for (i = 0; i < g_ble_ll_resolv_data.rl_cnt; ++i)
    {
      if (rl->rl_has_local)
        {
          sr = enter_critical_section();
          ble_ll_resolv_gen_priv_addr(rl, 1);
          leave_critical_section(sr);
        }

      if (rl->rl_has_peer)
        {
          sr = enter_critical_section();
          ble_ll_resolv_gen_priv_addr(rl, 0);
          leave_critical_section(sr);
        }

      ++rl;
    }

  ble_ll_callout_reset(&g_ble_ll_resolv_data.rpa_timer,
                        g_ble_ll_resolv_data.rpa_tmo);

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
  ble_ll_adv_rpa_timeout();
#endif
}

/****************************************************************************
 * Name: ble_ll_resolv_irk_nonzero
 *
 * Description:
 *   Called to determine if the IRK is all zero.
 *
 * Input Parameters:
 *   irk
 *
 * Returned Value:
 *   int 0: IRK is zero . 1: IRK has non-zero value.
 *
 ****************************************************************************/

static int ble_ll_resolv_irk_nonzero(const uint8_t *irk)
{
  int i;
  int rc;

  rc = 0;
  for (i = 0; i < 16; ++i)
    {
      if (*irk != 0)
        {
          rc = 1;
          break;
        }

      ++irk;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_is_on_resolv_list
 *
 * Description:
 *   Used to determine if the device is on the resolving list.
 *
 * Input Parameters:
 *   addr
 *   addr_type - Public address (0) or random address (1)
 *
 * Returned Value:
 *   int 0: device is not on resolving list; otherwise the return value
 *   is the 'position' of the device in the resolving list (the index of the
 *   element plus 1).
 *
 ****************************************************************************/

static int ble_ll_is_on_resolv_list(const uint8_t *addr, uint8_t addr_type)
{
  int i;
  struct ble_ll_resolv_entry *rl;

  rl = &g_ble_ll_resolv_list[0];
  for (i = 0; i < g_ble_ll_resolv_data.rl_cnt; ++i)
    {
      if ((rl->rl_addr_type == addr_type) &&
          (!memcmp(&rl->rl_identity_addr[0], addr, BLE_DEV_ADDR_LEN)))
        {
          return i + 1;
        }

      ++rl;
    }

  return 0;
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: ble_ll_resolv_list_clr
 *
 * Description:
 *   Clear the resolving list
 *
 * Returned Value:
 *   int 0: success, BLE error code otherwise
 *
 ****************************************************************************/

int ble_ll_resolv_list_clr(void)
{
  /* Check proper state */

  if (!ble_ll_resolv_list_chg_allowed())
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  /* Sets total on list to 0. Clears HW resolve list */

  g_ble_ll_resolv_data.rl_cnt_hw = 0;
  g_ble_ll_resolv_data.rl_cnt = 0;
  ble_hw_resolv_list_clear();

  /* stop RPA timer when clearing RL */

  ble_ll_callout_stop(&g_ble_ll_resolv_data.rpa_timer);

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_resolv_list_read_size
 *
 * Description:
 *   Read the size of the resolving list. This is the total number of
 * resolving list entries allowed by the controller.
 *
 * Input Parameters:
 *   rspbuf - Pointer to response buffer
 *
 * Returned Value:
 *   int 0: success.
 *
 ****************************************************************************/

int ble_ll_resolv_list_read_size(uint8_t *rspbuf, uint8_t *rsplen)
{
  struct ble_hci_le_rd_resolv_list_size_rp *rsp = (void *)rspbuf;

  rsp->size = g_ble_ll_resolv_data.rl_size;

  *rsplen = sizeof(*rsp);
  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_resolv_list_find
 *
 * Description:
 *   Used to determine if the device is on the resolving list.
 *
 * Input Parameters:
 *   addr
 *   addr_type - Public address (0) or random address (1)
 *
 * Returned Value:
 *   Pointer to resolving list entry or NULL if no entry found.
 *
 ****************************************************************************/

struct ble_ll_resolv_entry *ble_ll_resolv_list_find(const uint8_t *addr,
                                                    uint8_t addr_type)
{
  int i;
  struct ble_ll_resolv_entry *rl;

  rl = &g_ble_ll_resolv_list[0];
  for (i = 0; i < g_ble_ll_resolv_data.rl_cnt; ++i)
    {
      if ((rl->rl_addr_type == addr_type) &&
          (!memcmp(&rl->rl_identity_addr[0], addr, BLE_DEV_ADDR_LEN)))
        {
          return rl;
        }

      ++rl;
    }

  return NULL;
}

/****************************************************************************
 * Name: ble_ll_resolv_list_add
 *
 * Description:
 *   Add a device to the resolving list
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_resolv_list_add(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_add_resolv_list_cp *cmd = (const void *)cmdbuf;
  struct ble_ll_resolv_entry *rl;
  int rc = BLE_ERR_SUCCESS;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  /* Must be in proper state */

  if (!ble_ll_resolv_list_chg_allowed())
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  /* Check if we have any open entries */

  if (g_ble_ll_resolv_data.rl_cnt >= g_ble_ll_resolv_data.rl_size)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  /* spec is not clear on how to handle this but make sure host is aware
   * that new keys are not used in that case
   */

  if (ble_ll_is_on_resolv_list(cmd->peer_id_addr, cmd->peer_addr_type))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  /* we keep this sorted in a way that entries with peer_irk are first */

  if (ble_ll_resolv_irk_nonzero(cmd->peer_irk))
    {
      memmove(
          &g_ble_ll_resolv_list[g_ble_ll_resolv_data.rl_cnt_hw + 1],
          &g_ble_ll_resolv_list[g_ble_ll_resolv_data.rl_cnt_hw],
          (g_ble_ll_resolv_data.rl_cnt - g_ble_ll_resolv_data.rl_cnt_hw) *
              sizeof(g_ble_ll_resolv_list[0]));
      rl = &g_ble_ll_resolv_list[g_ble_ll_resolv_data.rl_cnt_hw];
    }
  else
    {
      rl = &g_ble_ll_resolv_list[g_ble_ll_resolv_data.rl_cnt];
    }

  memset(rl, 0, sizeof(*rl));
  rl->rl_addr_type = cmd->peer_addr_type;
  memcpy(rl->rl_identity_addr, cmd->peer_id_addr, BLE_DEV_ADDR_LEN);

  if (ble_ll_resolv_irk_nonzero(cmd->peer_irk))
    {
      swap_buf(rl->rl_peer_irk, cmd->peer_irk, 16);
      rl->rl_has_peer = 1;

      /* generate peer RPA now, those will be updated by timer when
       * resolution is enabled
       */

      ble_ll_resolv_gen_priv_addr(rl, 0);
    }

  if (ble_ll_resolv_irk_nonzero(cmd->local_irk))
    {
      swap_buf(rl->rl_local_irk, cmd->local_irk, 16);
      rl->rl_has_local = 1;

      /* generate local RPA now, those will be updated by timer when
       * resolution is enabled
       */

      ble_ll_resolv_gen_priv_addr(rl, 1);
    }

  /* By default use privacy network mode */

  rl->rl_priv_mode = BLE_HCI_PRIVACY_NETWORK;

  /* Add peers IRKs to HW resolving list. Should always succeed since we
   * already checked if there is room for it.
   */

  if (rl->rl_has_peer)
    {
      rc = ble_hw_resolv_list_add(rl->rl_peer_irk);
      DEBUGASSERT(rc == BLE_ERR_SUCCESS);
      g_ble_ll_resolv_data.rl_cnt_hw++;
    }

  g_ble_ll_resolv_data.rl_cnt++;

  /* start RPA timer if this was first element added to RL */

  if (g_ble_ll_resolv_data.rl_cnt == 1)
    {
      ble_ll_callout_reset(&g_ble_ll_resolv_data.rpa_timer,
                            g_ble_ll_resolv_data.rpa_tmo);
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_resolv_list_rmv
 *
 * Description:
 *   Remove a device from the resolving list
 *
 * Input Parameters:
 *   cmdbuf
 *
 * Returned Value:
 *   int 0: success, BLE error code otherwise
 *
 ****************************************************************************/

int ble_ll_resolv_list_rmv(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_rmv_resolve_list_cp *cmd = (const void *)cmdbuf;
  int position;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  /* Must be in proper state */

  if (!ble_ll_resolv_list_chg_allowed())
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  /* Remove from IRK records */

  position =
      ble_ll_is_on_resolv_list(cmd->peer_id_addr, cmd->peer_addr_type);
  if (position)
    {
      DEBUGASSERT(position <= g_ble_ll_resolv_data.rl_cnt);

      memmove(&g_ble_ll_resolv_list[position - 1],
              &g_ble_ll_resolv_list[position],
              (g_ble_ll_resolv_data.rl_cnt - position) *
                  sizeof(g_ble_ll_resolv_list[0]));
      g_ble_ll_resolv_data.rl_cnt--;

      /* Remove from HW list */

      if (position <= g_ble_ll_resolv_data.rl_cnt_hw)
        {
          ble_hw_resolv_list_rmv(position - 1);
          g_ble_ll_resolv_data.rl_cnt_hw--;
        }

      /* stop RPA timer if list is empty */

      if (g_ble_ll_resolv_data.rl_cnt == 0)
        {
          ble_ll_callout_stop(&g_ble_ll_resolv_data.rpa_timer);
        }

      return BLE_ERR_SUCCESS;
    }

  return BLE_ERR_UNK_CONN_ID;
}

/****************************************************************************
 * Name: ble_ll_resolv_enable_cmd
 *
 * Description:
 *   Called to enable or disable address resolution in the controller
 *
 * Input Parameters:
 *   cmdbuf
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_resolv_enable_cmd(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_set_addr_res_en_cp *cmd = (const void *)cmdbuf;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (ble_ll_is_busy(BLE_LL_BUSY_EXCLUDE_CONNECTIONS))
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (cmd->enable > 1)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  g_ble_ll_resolv_data.addr_res_enabled = cmd->enable;

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_resolv_peer_addr_rd
 *
 ****************************************************************************/

int ble_ll_resolv_peer_addr_rd(const uint8_t *cmdbuf, uint8_t len,
                               uint8_t *rspbuf, uint8_t *rsplen)
{
  const struct ble_hci_le_rd_peer_resolv_addr_cp *cmd = (const void *)cmdbuf;
  struct ble_hci_le_rd_peer_resolv_addr_rp *rsp = (void *)rspbuf;
  struct ble_ll_resolv_entry *rl;
  int rc;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  rl = ble_ll_resolv_list_find(cmd->peer_id_addr, cmd->peer_addr_type);
  if (rl)
    {
      memcpy(rsp->rpa, rl->rl_peer_rpa, BLE_DEV_ADDR_LEN);
      rc = BLE_ERR_SUCCESS;
    }
  else
    {
      memset(rsp->rpa, 0, BLE_DEV_ADDR_LEN);
      rc = BLE_ERR_UNK_CONN_ID;
    }

  *rsplen = sizeof(*rsp);
  return rc;
}

/****************************************************************************
 * Name: ble_ll_resolv_local_addr_rd
 *
 ****************************************************************************/

int ble_ll_resolv_local_addr_rd(const uint8_t *cmdbuf, uint8_t len,
                                uint8_t *rspbuf, uint8_t *rsplen)
{
  const struct ble_hci_le_rd_local_resolv_addr_cp *cmd =
      (const void *)cmdbuf;
  struct ble_hci_le_rd_local_resolv_addr_rp *rsp = (void *)rspbuf;
  struct ble_ll_resolv_entry *rl;
  int rc;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  rl = ble_ll_resolv_list_find(cmd->peer_id_addr, cmd->peer_addr_type);
  if (rl)
    {
      memcpy(rsp->rpa, rl->rl_local_rpa, BLE_DEV_ADDR_LEN);
      rc = BLE_ERR_SUCCESS;
    }
  else
    {
      memset(rsp->rpa, 0, BLE_DEV_ADDR_LEN);
      rc = BLE_ERR_UNK_CONN_ID;
    }

  *rsplen = sizeof(*rsp);
  return rc;
}

/****************************************************************************
 * Name: ble_ll_resolv_set_rpa_tmo
 *
 * Description:
 *   Set the resolvable private address timeout.
 *
 * Input Parameters:
 *   cmdbuf
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_resolv_set_rpa_tmo(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_set_rpa_tmo_cp *cmd = (const void *)cmdbuf;
  uint16_t tmo_secs;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  tmo_secs = le16toh(cmd->rpa_timeout);
  if (!((tmo_secs > 0) && (tmo_secs <= 0xa1b8)))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  g_ble_ll_resolv_data.rpa_tmo = MSEC2TICK(tmo_secs * 1000);

  /* restart timer if there is something on RL */

  if (g_ble_ll_resolv_data.rl_cnt)
    {
      ble_ll_callout_reset(&g_ble_ll_resolv_data.rpa_timer,
                            g_ble_ll_resolv_data.rpa_tmo);
    }

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_resolve_set_priv_mode
 *
 ****************************************************************************/

int ble_ll_resolve_set_priv_mode(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_set_privacy_mode_cp *cmd = (const void *)cmdbuf;
  struct ble_ll_resolv_entry *rl;

  if (ble_ll_is_busy(BLE_LL_BUSY_EXCLUDE_CONNECTIONS))
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  rl = ble_ll_resolv_list_find(cmd->peer_id_addr, cmd->peer_id_addr_type);
  if (!rl)
    {
      return BLE_ERR_UNK_CONN_ID;
    }

  if (cmd->mode > BLE_HCI_PRIVACY_DEVICE)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  rl->rl_priv_mode = cmd->mode;

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_resolv_get_rpa_tmo
 *
 * Description:
 *   Returns the Resolvable Private address timeout, in os ticks
 *
 * Returned Value:
 *   uint32_t
 *
 ****************************************************************************/

uint32_t ble_ll_resolv_get_rpa_tmo(void)
{
  return g_ble_ll_resolv_data.rpa_tmo;
}

/****************************************************************************
 * Name: ble_ll_resolv_get_priv_addr
 *
 ****************************************************************************/

void ble_ll_resolv_get_priv_addr(struct ble_ll_resolv_entry *rl, int local,
                                 uint8_t *addr)
{
  irqstate_t sr;

  DEBUGASSERT(rl != NULL);
  DEBUGASSERT(addr != NULL);

  sr = enter_critical_section();
  if (local)
    {
      DEBUGASSERT(rl->rl_has_local);
      memcpy(addr, rl->rl_local_rpa, BLE_DEV_ADDR_LEN);
    }
  else
    {
      DEBUGASSERT(rl->rl_has_peer);
      memcpy(addr, rl->rl_peer_rpa, BLE_DEV_ADDR_LEN);
    }

  leave_critical_section(sr);
}

/****************************************************************************
 * Name: ble_ll_resolv_set_peer_rpa
 *
 ****************************************************************************/

void ble_ll_resolv_set_peer_rpa(int index, uint8_t *rpa)
{
  irqstate_t sr;
  struct ble_ll_resolv_entry *rl;

  sr = enter_critical_section();
  rl = &g_ble_ll_resolv_list[index];
  memcpy(rl->rl_peer_rpa, rpa, BLE_DEV_ADDR_LEN);
  leave_critical_section(sr);
}

/****************************************************************************
 * Name: ble_ll_resolv_set_local_rpa
 *
 ****************************************************************************/

void ble_ll_resolv_set_local_rpa(int index, uint8_t *rpa)
{
  irqstate_t sr;
  struct ble_ll_resolv_entry *rl;

  sr = enter_critical_section();
  rl = &g_ble_ll_resolv_list[index];
  memcpy(rl->rl_local_rpa, rpa, BLE_DEV_ADDR_LEN);
  leave_critical_section(sr);
}

/****************************************************************************
 * Name: ble_ll_resolv_gen_rpa
 *
 * Description:
 *   Generate a resolvable private address.
 *
 * Input Parameters:
 *   addr
 *   addr_type
 *   rpa
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_resolv_gen_rpa(uint8_t *addr, uint8_t addr_type, uint8_t *rpa,
                          int local)
{
  struct ble_ll_resolv_entry *rl;

  rl = ble_ll_resolv_list_find(addr, addr_type);
  if (rl)
    {
      if ((local && rl->rl_has_local) || (!local && rl->rl_has_peer))
        {
          ble_ll_resolv_get_priv_addr(rl, local, rpa);
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_resolv_rpa
 *
 * Description:
 *   Resolve a Resolvable Private Address
 *
 * Input Parameters:
 *   rpa
 *   index
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_resolv_rpa(const uint8_t *rpa, const uint8_t *irk)
{
  int rc;
  const uint32_t *irk32;
  uint32_t *key32;
  uint32_t *pt32;
  struct ble_encryption_block ecb;

  irk32 = (const uint32_t *)irk;
  key32 = (uint32_t *)&ecb.key[0];

  key32[0] = irk32[0];
  key32[1] = irk32[1];
  key32[2] = irk32[2];
  key32[3] = irk32[3];

  pt32 = (uint32_t *)&ecb.plain_text[0];
  pt32[0] = 0;
  pt32[1] = 0;
  pt32[2] = 0;
  pt32[3] = 0;

  ecb.plain_text[15] = rpa[3];
  ecb.plain_text[14] = rpa[4];
  ecb.plain_text[13] = rpa[5];

  ble_hw_encrypt_block(&ecb);
  if ((ecb.cipher_text[15] == rpa[0]) && (ecb.cipher_text[14] == rpa[1]) &&
      (ecb.cipher_text[13] == rpa[2]))
    {
      rc = 1;
    }
  else
    {
      rc = 0;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_resolv_peer_rpa_any
 *
 ****************************************************************************/

int ble_ll_resolv_peer_rpa_any(const uint8_t *rpa)
{
  int i;

  for (i = 0; i < g_ble_ll_resolv_data.rl_cnt_hw; i++)
    {
      if (ble_ll_resolv_rpa(rpa, g_ble_ll_resolv_list[i].rl_peer_irk))
        {
          return i;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: ble_ll_resolv_enabled
 *
 * Description:
 *   Returns whether or not address resolution is enabled.
 *
 * Returned Value:
 *   uint8_t
 *
 ****************************************************************************/

uint8_t ble_ll_resolv_enabled(void)
{
  return g_ble_ll_resolv_data.addr_res_enabled;
}

/****************************************************************************
 * Name: ble_ll_resolv_list_reset
 *
 * Description:
 *   Called to reset private address resolution module.
 *
 ****************************************************************************/

void ble_ll_resolv_list_reset(void)
{
  g_ble_ll_resolv_data.addr_res_enabled = 0;
  ble_ll_callout_stop(&g_ble_ll_resolv_data.rpa_timer);
  ble_ll_resolv_list_clr();
  ble_ll_resolv_init();
}

/****************************************************************************
 * Name: ble_ll_resolv_init
 *
 ****************************************************************************/

void ble_ll_resolv_init(void)
{
  uint8_t hw_size;

  /* Default is 15 minutes */

  g_ble_ll_resolv_data.rpa_tmo = MSEC2TICK(15 * 60 * 1000);

  hw_size = ble_hw_resolv_list_size();
  if (hw_size > CONFIG_BLE_LL_RESOLV_LIST_SIZE)
    {
      hw_size = CONFIG_BLE_LL_RESOLV_LIST_SIZE;
    }

  g_ble_ll_resolv_data.rl_size = hw_size;

  ble_ll_event_init(&g_ble_ll_resolv_data.rpa_timer,
                    ble_ll_resolv_rpa_timer_cb, NULL);
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY */
