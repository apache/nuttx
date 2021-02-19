/****************************************************************************
 * wireless/bluetooth/bt_ioctl.c
 * Bluetooth network IOCTL handler
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_ioctl.h>

#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_ioctl.h"

#ifdef CONFIG_NETDEV_IOCTL  /* Not optional! */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* These structures encapsulate all globals used by the IOCTL logic. */

/* Scan state variables */

struct btnet_scanstate_s
{
  sem_t bs_exclsem;                 /* Manages exclusive access */
  volatile bool bs_scanning;        /* True:  Scanning in progress */
  volatile uint8_t bs_head;         /* Head of circular list (for removal) */
  uint8_t bs_tail;                  /* Tail of circular list (for addition) */

  struct bt_scanresponse_s bs_rsp[CONFIG_BLUETOOTH_MAXSCANRESULT];
};

/* Discovery state variables.  NOTE:  This function must be cast-compatible
 * with struct bt_gatt_discover_params_s.
 */

struct btnet_discoverstate_s
{
  struct bt_gatt_discover_params_s bd_params;
  struct bt_uuid_s bd_uuid;         /* Discovery UUID */
  sem_t bd_donesem;                 /* Manages exclusive access */
};

/* GATT read state variables. */

struct btnet_rdstate_s
{
  struct btreq_s *rd_btreq;
  uint8_t rd_result;                /* The result of the read */
  sem_t rd_donesem;                 /* Manages exclusive access */
};

/* GATT write state variables. */

struct btnet_wrstate_s
{
  struct btreq_s *wr_btreq;
  uint8_t wr_result;                /* The result of the read */
  sem_t wr_donesem;                 /* Manages exclusive access */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* At present only a single Bluetooth device is supported.  So we can simply
 * maintain the pending scan, discovery, MTU exchange, read and write states
 * as globals.
 *
 * NOTE: This limits to a single Bluetooth device with one concurrent scan
 * action, one concurrent MTU exchange, one concurrent discovery action,
 * etc.
 *
 * REVISIT: A fix might be to (1) allocate instances on each IOCTL command
 * the starts an operation, keeping the allocated structures in a list.  (2)
 * Return a reference number with each such command that starts an
 * operation.  (3) That reference number would then be used in each IOCTL
 * command that gets the result of the requested operation.  (4) The
 * allocated instance would be freed when either: (a) the result has been
 * returned or (b) it has expired without being harvested.  This implies
 * a timer that runs while there are pending operations in order to expire
 * the unharvested results.
 */

static struct btnet_scanstate_s     g_scanstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btnet_scan_callback
 *
 * Description:
 *   This is an HCI callback function.  HCI provides scan result data via
 *   this callback function.  The scan result data will be added to the
 *   cached scan results.
 *
 * Input Parameters:
 *   Scan result data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btnet_scan_callback(FAR const bt_addr_le_t *addr,
                                int8_t rssi, uint8_t adv_type,
                                FAR const uint8_t *adv_data, uint8_t len)
{
  FAR struct bt_scanresponse_s *rsp;
  uint8_t nexttail;
  uint8_t head;
  uint8_t tail;
  int ret;

  if (!g_scanstate.bs_scanning)
    {
      wlerr("ERROR:  Results received while not scanning\n");
      return;
    }

  if (len > CONFIG_BLUETOOTH_MAXSCANDATA)
    {
      wlerr("ERROR: Scan result is too big:  %u\n", len);
      return;
    }

  /* Get exclusive access to the scan data */

  ret = nxsem_wait_uninterruptible(&g_scanstate.bs_exclsem);
  if (ret < 0)
    {
      wlerr("nxsem_wait_uninterruptible() failed: %d\n", ret);
      return;
    }

  /* Add the scan data to the cache */

  tail     = g_scanstate.bs_tail;
  nexttail = tail + 1;

  if (nexttail >= CONFIG_BLUETOOTH_MAXSCANRESULT)
    {
      nexttail = 0;
    }

  /* Is the circular buffer full? */

  head = g_scanstate.bs_head;
  if (nexttail == head)
    {
      wlerr("ERROR: Too many scan results\n");

      if (++head >= CONFIG_BLUETOOTH_MAXSCANRESULT)
        {
          head = 0;
        }

      g_scanstate.bs_head = head;
    }

  /* Save the new scan result */

  rsp = &g_scanstate.bs_rsp[tail];
  memcpy(&rsp->sr_addr, addr, sizeof(bt_addr_le_t));
  rsp->sr_rssi = rssi;
  rsp->sr_type = adv_type;
  rsp->sr_len  = len;
  memcpy(&rsp->sr_data, adv_data, len);

  g_scanstate.bs_tail = nexttail;
  nxsem_post(&g_scanstate.bs_exclsem);
}

/****************************************************************************
 * Name: btnet_scan_result
 *
 * Description:
 *   This function implements the SIOCBTSCANGET IOCTL command.  It returns
 *   the current, buffered discovered handles.
 *
 * Input Parameters:
 *   result - Location to return the scan result data
 *   maxrsp - The maximum number of responses that can be returned.
 *
 * Returned Value:
 *   On success, the actual number of scan results obtain is returned.  A
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int btnet_scan_result(FAR struct bt_scanresponse_s *result,
                             uint8_t maxrsp)
{
  uint8_t head;
  uint8_t tail;
  uint8_t nrsp;
  bool scanning;
  int ret;

  scanning = g_scanstate.bs_scanning;
  wlinfo("Scanning? %s\n", scanning ? "YES" : "NO");

  /* Get exclusive access to the scan data while we are actively scanning.
   * The semaphore is uninitialized in other cases.
   */

  if (scanning)
    {
      /* Get exclusive access to the scan data */

      ret = nxsem_wait(&g_scanstate.bs_exclsem);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Copy all available results */

  head = g_scanstate.bs_head;
  tail = g_scanstate.bs_tail;

  for (nrsp = 0; nrsp < maxrsp && head != tail; nrsp++)
    {
      FAR const uint8_t *src;
      FAR uint8_t *dest;

      /* Copy data from the head index into the user buffer */

      src  = (FAR const uint8_t *)&g_scanstate.bs_rsp[head];
      dest = (FAR uint8_t *)&result[nrsp];
      memcpy(dest, src, sizeof(struct bt_scanresponse_s));

      /* Increment the head index */

      if (++head >= CONFIG_BLUETOOTH_MAXSCANRESULT)
        {
          head = 0;
        }
    }

  g_scanstate.bs_head = head;

  if (scanning)
    {
      nxsem_post(&g_scanstate.bs_exclsem);
    }

  return nrsp;
}

/****************************************************************************
 * Name: btnet_exchange_rsp
 *
 * Description:
 *   Result of MTU exchange.
 *
 * Input Parameters:
 *   conn   - The address of the peer in the MTU exchange
 *   result - The result of the MTU exchange
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btnet_exchange_rsp(FAR struct bt_conn_s *conn, uint8_t result)
{
  FAR struct btnet_wrstate_s *pstate = conn->p_iostate;
  FAR struct btreq_s *btreq = pstate->wr_btreq;

  wlinfo("%s\n", result == 0 ? "succeeded" : "failed");

  btreq->btr_exresult = result;
  nxsem_post(&pstate->wr_donesem);
}

/****************************************************************************
 * Name: btnet_discover_func
 *
 * Description:
 *   GATT discovery callback.  This function is called when a new handle is
 *   discovered
 *
 * Input Parameters:
 *   attr - The discovered attributes
 *   arg  - The original discovery parameters
 *
 * Returned Value:
 *   BT_GATT_ITER_CONTINUE meaning to continue the iteration.
 *
 ****************************************************************************/

static uint8_t btnet_discover_func(FAR const struct bt_gatt_attr_s *attr,
                                   FAR void *arg)
{
  FAR struct bt_gatt_discover_params_s *params = arg;
  FAR struct btreq_s *btreq = (FAR struct btreq_s *)(params->p_data);

  wlinfo("Discovered handle %x\n", attr->handle);

  if (btreq->btr_indx >= btreq->btr_gnrsp)
    {
      wlerr("ERROR: No space for results\n");
      return BT_GATT_ITER_STOP;
    }

  btreq->btr_grsp[btreq->btr_indx].dr_handle = attr->handle;
  btreq->btr_grsp[btreq->btr_indx].dr_perm   = attr->perm;
  btreq->btr_indx++;

  return BT_GATT_ITER_CONTINUE;
}

/****************************************************************************
 * Name: btnet_discover_destroy
 *
 * Description:
 *   GATT destroy callback.  This function is called when a discovery
 *   completes.
 *
 * Input Parameters:
 *   arg  - The original discovery parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btnet_discover_destroy(FAR void *arg)
{
  struct btnet_discoverstate_s *pstate = arg;

  wlinfo("Discover destroy\n");

  nxsem_post(&pstate->bd_donesem);
}

/****************************************************************************
 * Name: btnet_read_callback
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   conn   - Connection of read
 *   result - The result status of the read.
 *   data   - The data read
 *   length - The Number of bytes read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btnet_read_callback(FAR struct bt_conn_s *conn, int result,
                                FAR const void *data, uint16_t length)
{
  FAR struct btnet_rdstate_s *pstate = conn->p_iostate;
  FAR struct btreq_s *btreq = pstate->rd_btreq;

  wlinfo("Read complete: result %d length %u\n", result, length);

  if (length > HCI_GATTRD_DATA)
    {
      wlerr("ERROR: Unexpected length %u, result = %d\n", length, result);
      btreq->btr_rdresult = ENFILE;
    }
  else
    {
      DEBUGASSERT((unsigned int)result < UINT8_MAX);

      btreq->btr_rdresult = result;
      btreq->btr_rdsize   = length;
      memcpy(btreq->btr_rddata, data, length);
    }

  nxsem_post(&pstate->rd_donesem);
}

/****************************************************************************
 * Name: bnet_write_callback
 *
 * Description:
 *   Result of write operation.
 *
 * Input Parameters:
 *   conn   - The address of the peer in the MTU exchange
 *   result - The result of the MTU exchange
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bnet_write_callback(FAR struct bt_conn_s *conn, uint8_t result)
{
  FAR struct btnet_wrstate_s *pstate = conn->p_iostate;
  FAR struct btreq_s *btreq = pstate->wr_btreq;

  wlinfo("%s\n", result == 0 ? "succeeded" : "failed");

  btreq->btr_wrresult = result;
  nxsem_post(&pstate->wr_donesem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btnet_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   netdev - Reference to the NuttX driver state structure
 *   cmd    - The IOCTL command
 *   arg    - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int btnet_ioctl(FAR struct net_driver_s *netdev, int cmd, unsigned long arg)
{
  FAR struct btreq_s *btreq = (FAR struct btreq_s *)((uintptr_t)arg);
  int ret;

  wlinfo("cmd=%04x arg=%lu\n", cmd, arg);
  DEBUGASSERT(netdev != NULL && netdev->d_private != NULL);

  if (btreq == NULL)
    {
      return -EINVAL;
    }

  wlinfo("ifname: %s\n", btreq->btr_name);

  /* Process the command */

  switch (cmd)
    {
      case SIOCBTCONNECT:
        {
          FAR struct bt_conn_s *conn;
          conn = bt_conn_create_le(&btreq->btr_rmtpeer);

          if (!conn)
            {
              wlerr("Connection failed\n");
              ret = -ENOTCONN;
            }
          else
            {
              wlinfo("Connection pending\n");
              ret = OK;
            }
        }
        break;

      case SIOCBTDISCONNECT:
        {
          FAR struct bt_conn_s *conn;

          conn = bt_conn_lookup_addr_le(&btreq->btr_rmtpeer);
          if (!conn)
            {
              wlerr("Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              ret = bt_conn_disconnect(conn,
                                       BT_HCI_ERR_REMOTE_USER_TERM_CONN);
              if (ret == -ENOTCONN)
                {
                  wlerr("Already disconnected\n");
                }
              else
                {
                  /* Release reference taken in bt_conn_create_le */

                  bt_conn_release(conn);
                  ret = OK;
                }

              /* Release reference taken in bt_conn_lookup_addr_le */

              bt_conn_release(conn);
            }
        }
        break;

       /* SIOCGBTINFO:  Get Bluetooth device Info.  Given the device name,
        * fill in the btreq_s structure.
        *
        * REVISIT:  Little more than a stub at present.  It does return the
        * device address associated with the device name which in itself is
        * important.
        */

       case SIOCGBTINFO:
         {
           memset(&btreq->btru.btri, 0, sizeof(btreq->btru.btri));
           BLUETOOTH_ADDRCOPY(btreq->btr_bdaddr.val, g_btdev.bdaddr.val);
           btreq->btr_num_cmd = CONFIG_BLUETOOTH_BUFFER_PREALLOC;
           btreq->btr_num_acl = CONFIG_BLUETOOTH_BUFFER_PREALLOC;
           btreq->btr_acl_mtu = BLUETOOTH_MAX_MTU;
           btreq->btr_sco_mtu = BLUETOOTH_MAX_MTU;
           btreq->btr_max_acl = CONFIG_IOB_NBUFFERS;
           ret                = OK;
         }
         break;

       /* SIOCGBTFEAT
        *   Get Bluetooth BR/BDR device Features.  This returns the cached
        *   basic (page 0) and extended (page 1 & 2) features.  Only page 0
        *   is valid.
        * SIOCGBTLEFEAT
        *   Get Bluetooth LE device Features.  This returns the cached page
        *   0-2  features.  Only page 0 is value.
        */

       case SIOCGBTFEAT:
       case SIOCGBTLEFEAT:
         {
           FAR const uint8_t *src;

           memset(&btreq->btru.btrf, 0, sizeof(btreq->btru.btrf));
           if (cmd == SIOCGBTFEAT)
             {
               src = g_btdev.features;
             }
           else
             {
               src = g_btdev.le_features;
             }

           memcpy(btreq->btr_features0, src, 8);
           ret = OK;
         }
         break;

      /* SIOCBTADVSTART:  Set advertisement data, scan response data,
       * advertisement parameters and start advertising.
       */

      case SIOCBTADVSTART:
        {
          ret = bt_start_advertising(btreq->btr_advtype,
                                     btreq->btr_advad,
                                     btreq->btr_advsd);
          wlinfo("Start advertising: %d\n", ret);
        }
        break;

      /* SIOCBTADVSTOP:   Stop advertising. */

      case SIOCBTADVSTOP:
        {
          wlinfo("Stop advertising\n");
          bt_stop_advertising();
          ret = OK;
        }
        break;

      /* SIOCBTSCANSTART:  Start LE scanning.  Buffered scan results may be
       * obtained via SIOCBTSCANGET
       */

      case SIOCBTSCANSTART:
        {
          /* Are we already scanning? */

          if (g_scanstate.bs_scanning)
            {
              wlwarn("WARNING: Already scanning\n");
              ret = -EBUSY;
            }
          else
            {
              /* Initialize scan state */

              nxsem_init(&g_scanstate.bs_exclsem, 0, 1);
              g_scanstate.bs_scanning = true;
              g_scanstate.bs_head     = 0;
              g_scanstate.bs_tail     = 0;

              ret = bt_start_scanning(btreq->btr_dupenable,
                                      btnet_scan_callback);
              wlinfo("Start scan: %d\n", ret);

              if (ret < 0)
                {
                  nxsem_destroy(&g_scanstate.bs_exclsem);
                  g_scanstate.bs_scanning = false;
                }
            }
        }
        break;

      /* SIOCBTSCANGET:  Return scan results buffered since the call time
       * that the SIOCBTSCANGET command was invoked.
       */

      case SIOCBTSCANGET:
        {
          ret = btnet_scan_result(btreq->btr_rsp, btreq->btr_nrsp);
          wlinfo("Get scan results: %d\n", ret);

          if (ret >= 0)
            {
              btreq->btr_nrsp = ret;
              ret = OK;
            }
        }
        break;

      /* SIOCBTSCANSTOP:
       *  Stop LE scanning and discard any buffered results.
       */

      case SIOCBTSCANSTOP:
        {
          /* Stop scanning */

          ret = bt_stop_scanning();
          wlinfo("Stop scanning: %d\n", ret);

          nxsem_destroy(&g_scanstate.bs_exclsem);
          g_scanstate.bs_scanning = false;
        }
        break;

      /* SIOCBTSECURITY:  Enable security for a connection. */

      case SIOCBTSECURITY:
        {
          FAR struct bt_conn_s *conn;

          /* Get the connection associated with the provided LE address */

          conn = bt_conn_lookup_addr_le(&btreq->btr_secaddr);
          if (conn == NULL)
            {
              wlwarn("WARNING:  Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              ret = bt_conn_security(conn, btreq->btr_seclevel);
              if (ret < 0)
                {
                  wlerr("ERROR:  Security setting failed: %d\n", ret);
                }

              bt_conn_release(conn);
            }
        }
        break;

      /* SIOCBTEXCHANGE:  Exchange MTUs */

      case SIOCBTEXCHANGE:
        {
          FAR struct bt_conn_s *conn;

          /* Get the connection associated with the provided LE address */

          conn = bt_conn_lookup_addr_le(&btreq->btr_expeer);
          if (conn == NULL)
            {
              wlwarn("WARNING:  Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              struct btnet_wrstate_s wrstate;

              memset(&wrstate, 0, sizeof(wrstate));
              wrstate.wr_btreq = btreq;
              conn->p_iostate = &wrstate;
              nxsem_init(&wrstate.wr_donesem, 0, 0);

              btreq->btr_wrresult = EBUSY;

              ret = bt_gatt_exchange_mtu(conn, btnet_exchange_rsp);
              if (ret < 0)
                {
                  wlerr("ERROR: Exchange operation failed: %d\n", ret);
                }
              else
                {
                  /* Wait for callback to complete the exchange */

                  ret = nxsem_wait(&wrstate.wr_donesem);
                }

              nxsem_destroy(&wrstate.wr_donesem);

              bt_conn_release(conn);
            }
        }
        break;

      /* SIOCBTDISCOVER:  Starts GATT discovery */

      case SIOCBTDISCOVER:
        {
          FAR struct bt_conn_s *conn;

          /* Get the connection associated with the provided LE address */

          conn = bt_conn_lookup_addr_le(&btreq->btr_dpeer);
          if (conn == NULL)
            {
              wlwarn("WARNING:  Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              struct btnet_discoverstate_s dstate;
              FAR struct bt_gatt_discover_params_s *params;
              memset(&dstate, 0, sizeof(dstate));

              /* Set up the query */

              dstate.bd_uuid.type   = BT_UUID_16;
              dstate.bd_uuid.u.u16  = btreq->btr_duuid16;

              params                         = &dstate.bd_params;
              params->func                   = btnet_discover_func;
              params->destroy                = btnet_discover_destroy;
              params->start_handle           = btreq->btr_dstart;
              params->end_handle             = btreq->btr_dend;
              params->p_data                 = (void *)arg;
              btreq->btr_indx                = 0;

              if (btreq->btr_duuid16 == 0)
                {
                  params->uuid = NULL;
                }
              else
                {
                  params->uuid = &dstate.bd_uuid;
                }

              nxsem_init(&dstate.bd_donesem, 0, 0);

              /* Start the query */

              switch (btreq->btr_dtype)
                {
                  case GATT_DISCOVER:
                    ret = bt_gatt_discover(conn, params);
                    break;

                  case GATT_DISCOVER_DESC:
                    ret = bt_gatt_discover_descriptor(conn, params);
                    break;

                  case GATT_DISCOVER_CHAR:
                    ret = bt_gatt_discover_characteristic(conn, params);
                    break;

                  default:
                    wlerr("ERROR: Unrecognized GATT discover type: %u\n",
                          btreq->btr_dtype);
                    ret = -EINVAL;
                }

              if (ret < 0)
                {
                  wlerr("ERROR: Failed to start discovery: %d\n", ret);
                }
              else
                {
                  ret = nxsem_wait(&dstate.bd_donesem);
                  if (ret == 0)
                    {
                      btreq->btr_gnrsp = btreq->btr_indx;
                    }
                }

              nxsem_destroy(&dstate.bd_donesem);

              bt_conn_release(conn);
            }
        }
        break;

      /* SIOCBTGATTRD:  Read GATT data */

      case SIOCBTGATTRD:
        {
          FAR struct bt_conn_s *conn;

          /* Get the connection associated with the provided LE address */

          conn = bt_conn_lookup_addr_le(&btreq->btr_rdpeer);
          if (conn == NULL)
            {
              wlwarn("WARNING: Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              /* Set up for the read */

              struct btnet_rdstate_s rdstate;

              memset(&rdstate, 0, sizeof(rdstate));
              rdstate.rd_btreq = btreq;
              conn->p_iostate = &rdstate;
              nxsem_init(&rdstate.rd_donesem, 0, 0);

              /* Initiate read.. single or multiple? */

              if (btreq->btr_rdnhandles == 1)
                {
                  /* Read single */

                  DEBUGASSERT(conn->p_iostate != NULL);
                  ret = bt_gatt_read(conn, btreq->btr_rdhandles[0],
                                     btreq->btr_rdoffset,
                                     btnet_read_callback);
                }
              else if (btreq->btr_rdnhandles < HCI_GATT_MAXHANDLES)
                {
                  /* Read multiple */

                  DEBUGASSERT(conn->p_iostate != NULL);
                  DEBUGASSERT(btreq->btr_rdnhandles > 0);
                  ret = bt_gatt_read_multiple(conn, btreq->btr_rdhandles,
                                              btreq->btr_rdnhandles,
                                              btnet_read_callback);
                }
              else
                {
                  ret = -ENFILE;
                }

              if (ret < 0)
                {
                  wlerr("ERROR:  Read operation failed: %d\n", ret);
                }
              else
                {
                  /* Wait for callback to complete the transfer */

                  ret = nxsem_wait(&rdstate.rd_donesem);
                }

              nxsem_destroy(&rdstate.rd_donesem);

              bt_conn_release(conn);
            }
        }
        break;

      /* SIOCBTGATTWR:  Write GATT data */

      case SIOCBTGATTWR:
        {
          DEBUGASSERT(btreq->btr_wrdata != NULL);

          FAR struct bt_conn_s *conn;

          /* Get the connection associated with the provided LE address */

          conn = bt_conn_lookup_addr_le(&btreq->btr_wrpeer);
          if (conn == NULL)
            {
              wlwarn("WARNING:  Peer not connected\n");
              ret = -ENOTCONN;
            }
          else
            {
              /* Set up for the write */

              struct btnet_wrstate_s wrstate;

              memset(&wrstate, 0, sizeof(wrstate));
              wrstate.wr_btreq = btreq;
              conn->p_iostate = &wrstate;
              nxsem_init(&wrstate.wr_donesem, 0, 0);

              btreq->btr_wrresult = EBUSY;

              /* Initiate write */

              ret = bt_gatt_write(conn, btreq->btr_wrhandle,
                                  btreq->btr_wrdata,
                                  btreq->btr_wrnbytes,
                                  bnet_write_callback);
              if (ret < 0)
                {
                  wlerr("ERROR:  Write operation failed: %d\n", ret);
                }
              else
                {
                  /* Wait for callback to complete the transfer */

                  ret = nxsem_wait(&wrstate.wr_donesem);
                }

              nxsem_destroy(&wrstate.wr_donesem);

              bt_conn_release(conn);
            }
        }
        break;

      default:
        wlwarn("WARNING: Unrecognized IOCTL command: %02x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#endif /* CONFIG_NETDEV_IOCTL */
