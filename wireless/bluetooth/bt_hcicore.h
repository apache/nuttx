/****************************************************************************
 * wireless/bluetooth/bt_att.c
 * HCI core Bluetooth handling.
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
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLUETOOTH_BT_HDICORE_H
#define __WIRELESS_BLUETOOTH_BT_HDICORE_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <semaphore.h>
#include <mqueue.h>

#include <nuttx/wireless/bt_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enabling debug increases stack size requirement considerably */

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
#  define BT_STACK_DEBUG_EXTRA  512
#else
#  define BT_STACK_DEBUG_EXTRA  0
#endif

#define BT_STACK(name, size) \
  char __stack name[(size) + BT_STACK_DEBUG_EXTRA]
#define BT_STACK_NOINIT(name, size) \
  char __noinit __stack name[(size) + BT_STACK_DEBUG_EXTRA]

/* LMP feature helpers */

#define lmp_bredr_capable(dev)  (!((dev).features[4] & BT_LMP_NO_BREDR))
#define lmp_le_capable(dev)     ((dev).features[4] & BT_LMP_LE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* State tracking for the local Bluetooth controller */

struct bt_dev_s
{
  /* Local Bluetooth Device Address */

  bt_addr_t bdaddr;

  /* Controller version & manufacturer information */

  uint8_t hci_version;
  uint16_t hci_revision;
  uint16_t manufacturer;

  /* BR/EDR features page 0 */

  uint8_t features[8];

  /* LE features */

  uint8_t le_features[8];

  /* Advertising state */

  uint8_t adv_enable;

  /* Scanning state */

  uint8_t scan_enable;
  uint8_t scan_filter;

  /* Controller buffer information */

  uint8_t le_pkts;
  uint16_t le_mtu;
  sem_t le_pkts_sem;

  /* Number of commands controller can accept */

  uint8_t ncmd;
  sem_t ncmd_sem;

  /* Last sent HCI command */

  FAR struct bt_buf_s *sent_cmd;

  /* Queue for incoming HCI events and ACL data */

  mqd_t rx_queue;

  /* Queue for outgoing HCI commands */

  mqd_t tx_queue;

  /* Registered HCI driver */

  FAR const struct bt_driver_s *dev;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct bt_dev_s g_btdev;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline int bt_addr_cmp(FAR const bt_addr_t *a, FAR const bt_addr_t *b)
{
  return memcmp(a, b, sizeof(*a));
}

static inline int bt_addr_le_cmp(FAR const bt_addr_le_t *a, FAR const bt_addr_le_t *b)
{
  return memcmp(a, b, sizeof(*a));
}

static inline void bt_addr_copy(FAR bt_addr_t *dst, FAR const bt_addr_t *src)
{
  memcpy(dst, src, sizeof(*dst));
}

static inline void bt_addr_le_copy(FAR bt_addr_le_t *dst, FAR const bt_addr_le_t *src)
{
  memcpy(dst, src, sizeof(*dst));
}

static inline bool bt_addr_le_is_rpa(FAR const bt_addr_le_t *addr)
{
  if (addr->type != BT_ADDR_LE_RANDOM)
    {
      return false;
    }

  if ((addr->val[5] & 0xc0) == 0x40)
    {
      return true;
    }

  return false;
}

static inline bool bt_addr_le_is_identity(FAR const bt_addr_le_t *addr)
{
  if (addr->type == BT_ADDR_LE_PUBLIC)
    {
      return true;
    }

  /* Check for Random Static address type */

  if ((addr->val[5] & 0xc0) == 0xc0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct bt_buf_s *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len);
int bt_hci_cmd_send(uint16_t opcode, FAR struct bt_buf_s *buf);
int bt_hci_cmd_send_sync(uint16_t opcode, FAR struct bt_buf_s *buf,
                         FAR struct bt_buf_s **rsp);
int bt_le_scan_update(void);

/* The helper is only safe to be called from internal kernel threads as it's
 * not multi-threading safe
 */

#ifdef CONFIG_DEBUG_WIRELESS_INFO
FAR const char *bt_addr_str(FAR const bt_addr_t *addr);
FAR const char *bt_addr_le_str(FAR const bt_addr_le_t *addr);
#endif

#endif /* __WIRELESS_BLUETOOTH_BT_HDICORE_H */
