/****************************************************************************
 * wireless/bluetooth/bt_hcicore.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_HDICORE_H
#define __WIRELESS_BLUETOOTH_BT_HDICORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/mqueue.h>

#include <nuttx/semaphore.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LMP feature helpers */

#define lmp_bredr_capable(btdev)  (!((btdev).features[4] & BT_LMP_NO_BREDR))
#define lmp_le_capable(btdev)     ((btdev).features[4] & BT_LMP_LE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* State tracking for the local Bluetooth controller */

struct bt_dev_s
{
    /* Local Bluetooth Device Address */

    bt_addr_t bdaddr;

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
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
#endif

  /* TX thread status */

  int tx_status;

  /* Number of commands controller can accept */

  uint8_t ncmd;
  sem_t ncmd_sem;

  /* Last sent HCI command */

  FAR struct bt_buf_s *sent_cmd;

  /* Queue for incoming HCI events and ACL data */

  struct file rx_queue;

  /* Queue for outgoing HCI commands */

  struct file tx_queue;

  /* Registered HCI driver */

  FAR struct bt_driver_s *btdev;
};

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/* Connection callback structure */

struct bt_conn_s; /* Forward reference */
struct bt_conn_cb_s
{
  FAR struct bt_conn_cb_s *flink;
  FAR void *context;

  CODE void (*connected)(FAR struct bt_conn_s *conn, FAR void *context);
  CODE void (*disconnected)(FAR struct bt_conn_s *conn, FAR void *context);
};
#else
/* RAW HCI packets callbacks */

struct bt_hci_cb_s
{
  FAR void *context;

  CODE void (*received)(FAR struct bt_buf_s *buf, FAR void *context);
};
#endif

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/****************************************************************************
 * Name: bt_le_scan_cb_t
 *
 * Description:
 *   A function of this type will be called back when user application
 *   triggers active LE scan. The caller will populate all needed
 *   parameters based on data coming from scan result.
 *   Such function can be set by user when LE active scan API is used.
 *
 * Input Parameters:
 *  addr     - Advertiser LE address and type.
 *  rssi     - Strength of advertiser signal.
 *  adv_type - Type of advertising response from advertiser.
 *  adv_data - Address of buffer containing advertiser data.
 *  len      - Length of advertiser data contained in buffer.
 *
 ****************************************************************************/

typedef CODE void bt_le_scan_cb_t(FAR const bt_addr_le_t *addr, int8_t rssi,
                                  uint8_t adv_type,
                                  FAR const uint8_t *adv_data, uint8_t len);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct bt_dev_s g_btdev;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
static inline int bt_addr_cmp(FAR const bt_addr_t *a, FAR const bt_addr_t *b)
{
  return memcmp(a, b, sizeof(*a));
}

static inline int bt_addr_le_cmp(FAR const bt_addr_le_t *a,
                                 FAR const bt_addr_le_t *b)
{
  return memcmp(a, b, sizeof(*a));
}

static inline void bt_addr_copy(FAR bt_addr_t *dst, FAR const bt_addr_t *src)
{
  memcpy(dst, src, sizeof(*dst));
}

static inline void bt_addr_le_copy(FAR bt_addr_le_t *dst,
                                   FAR const bt_addr_le_t *src)
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

struct bt_eir_s; /* Forward reference */

#endif

/****************************************************************************
 * Name: bt_initialize
 *
 * Description:
 *   Initialize Bluetooth. Must be the called before anything else.
 *
 * Returned Value:
 *    Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_initialize(void);

/****************************************************************************
 * Name: bt_deinitialize
 *
 * Description:
 *   Deinitialize Bluetooth.
 *
 * Returned Value:
 *    Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_deinitialize(void);

/****************************************************************************
 * Name: bt_driver_register
 *
 * Description:
 *   Register the Bluetooth low-level driver with the Bluetooth stack.
 *   This is called from the low-level driver and is part of the driver
 *   interface prototyped in include/nuttx/wireless/bluetooth/bt_driver.h
 *
 *   This function associates the Bluetooth driver with the Bluetooth stack.
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *  Zero is returned on success; a negated errno value is returned on any
 *  failure.
 *
 ****************************************************************************/

int bt_driver_register(FAR struct bt_driver_s *btdev);

/****************************************************************************
 * Name: bt_driver_unregister
 *
 * Description:
 *   Unregister a Bluetooth low-level driver previously registered with
 *   bt_driver_register.  This may be called from the low-level driver and
 *   is part of the driver interface prototyped in
 *   include/nuttx/wireless/bluetooth/bt_driver.h
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void bt_driver_unregister(FAR struct bt_driver_s *btdev);

/****************************************************************************
 * Name: bt_send
 *
 * Description:
 *   Add the provided buffer 'buf' to the head selected buffer list 'list'
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *   buf   - The buffer to be sent by the driver
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bt_send(FAR struct bt_driver_s *btdev,
            FAR struct bt_buf_s *buf);

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
/****************************************************************************
 * Name: bt_hci_cmd_create
 *
 * Description:
 *   Allocate and initialize a buffer for a command
 *
 * Returned Value:
 *   A reference to the allocated buffer.  NULL could possibly be returned
 *   on any failure to allocate.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len);

/* Send HCI commands */

int bt_hci_cmd_send(uint16_t opcode, FAR struct bt_buf_s *buf);
int bt_hci_cmd_send_sync(uint16_t opcode, FAR struct bt_buf_s *buf,
                         FAR struct bt_buf_s **rsp);

/* The helper is only safe to be called from internal kernel threads as it's
 * not multi-threading safe
 */

FAR const char *bt_addr_str(FAR const bt_addr_t *addr);
FAR const char *bt_addr_le_str(FAR const bt_addr_le_t *addr);

/****************************************************************************
 * Name: bt_start_advertising
 *
 * Description:
 *   Set advertisement data, scan response data, advertisement parameters
 *   and start advertising.
 *
 * Input Parameters:
 *   type - Advertising type.
 *   ad   - Data to be used in advertisement packets.
 *   sd   - Data to be used in scan response packets.
 *
 * Returned Value:
 *   Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_start_advertising(uint8_t type, FAR const struct bt_eir_s *ad,
                         FAR const struct bt_eir_s *sd);

/****************************************************************************
 * Name: bt_stop_advertising
 *
 * Description:
 *   Stops ongoing advertising.
 *
 * Returned Value:
 *   Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_stop_advertising(void);

/****************************************************************************
 * Name: bt_start_scanning
 *
 * Description:
 *   Start LE scanning with and provide results through the specified
 *   callback.
 *
 * Input Parameters:
 *   filter_dups - Enable duplicate filtering (or not).
 *   cb          - Callback to notify scan results.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_start_scanning(uint8_t filter_dups, bt_le_scan_cb_t cb);

/****************************************************************************
 * Name: bt_stop_scanning
 *
 * Description:
 *   Stops ongoing LE scanning.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_stop_scanning(void);

/****************************************************************************
 * Name: bt_le_scan_update
 *
 * Description:
 *   Used to determine whether to start scan and which scan type should be
 *   used.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_le_scan_update(void);

/****************************************************************************
 * Name: bt_conn_cb_register
 *
 * Description:
 *   Register callbacks to monitor the state of connections.
 *
 * Input Parameters:
 *   cb - Instance of the callback structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_cb_register(FAR struct bt_conn_cb_s *cb);
#else

/****************************************************************************
 * Name: bt_hci_cb_register
 *
 * Description:
 *   Register callbacks to handle RAW HCI packets
 *
 * Input Parameters:
 *   cb - Instance of the callback structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_hci_cb_register(FAR struct bt_hci_cb_s *cb);

#endif

/****************************************************************************
 * Name: bt_receive
 *
 * Description:
 *   Called by the Bluetooth low-level driver when new data is received from
 *   the radio.  This may be called from the low-level driver and is part of
 *   the driver interface
 *
 ****************************************************************************/

int bt_receive(FAR struct bt_driver_s *btdev, enum bt_buf_type_e type,
               FAR void *data, size_t len);

#endif /* __WIRELESS_BLUETOOTH_BT_HDICORE_H */
