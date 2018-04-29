/****************************************************************************
 * wireless/bluetooth/bt_ioctl.c
 * Bluetooth network IOCTL handler
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Public Types
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
  sem_t bd_exclsem;                 /* Manages exclusive access */
  volatile bool bd_discovering;     /* True:  Discovery in progress */
  volatile uint8_t bd_head;         /* Head of circular list (for removal) */
  volatile uint8_t bd_tail;         /* Tail of circular list (for addition) */

  struct bt_discresonse_s bd_rsp[CONFIG_BLUETOOTH_MAXDISCOVER];
};

/* GATT read state variables. */

struct btnet_rdstate_s
{
  bool rd_pending;                  /* True: result not yet received */
  uint8_t rd_result;                /* The result of the read */
  uint8_t rd_head;                  /* Start of data to read */
  uint8_t rd_tail;                  /* End data to read */
  uint8_t rd_data[HCI_GATTRD_DATA]; /* Data read */
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
static struct btnet_discoverstate_s g_discoverstate;
static struct bt_result_s           g_exchangeresult;
static struct btnet_rdstate_s       g_rdstate;
static struct bt_result_s           g_gattwrresult;

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

static void btnet_scan_callback(FAR const bt_addr_le_t *addr, int8_t rssi,
                                uint8_t adv_type, FAR const uint8_t *adv_data,
                                uint8_t len)
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

  while ((ret = nxsem_wait(&g_scanstate.bs_exclsem)) < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      if (ret != -EINTR)
        {
          return;
        }
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
          DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
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
  wlinfo("Exchange %s\n", result == 0 ? "succeeded" : "failed");
  g_exchangeresult.br_pending = false;
  g_exchangeresult.br_result  = result;
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
  uint8_t nexttail;
  uint8_t head;
  uint8_t tail;
  int ret;

  wlinfo("Discovered handle %u\n", attr->handle);

  if (!g_discoverstate.bd_discovering)
    {
      wlerr("ERROR:  Results received while not discovering\n");
      return BT_GATT_ITER_STOP;
    }

  /* Get exclusive access to the discovered data */

  while ((ret = nxsem_wait(&g_discoverstate.bd_exclsem)) < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      if (ret != -EINTR)
        {
          return BT_GATT_ITER_STOP;
        }
    }

  /* Add the discovered data to the cache */

  tail     = g_discoverstate.bd_tail;
  nexttail = tail + 1;

  if (nexttail >= CONFIG_BLUETOOTH_MAXSCANRESULT)
    {
      nexttail = 0;
    }

  /* Is the circular buffer full? */

  head = g_discoverstate.bd_head;
  if (nexttail == head)
    {
      wlerr("ERROR: Too many handles discovered.  Data lost.\n");

      if (++head >= CONFIG_BLUETOOTH_MAXSCANRESULT)
        {
          head = 0;
        }

      g_discoverstate.bd_head = head;
    }

  /* Save the newly discovered handle */

  g_discoverstate.bd_rsp[tail].dr_handle = attr->handle;
  g_discoverstate.bd_rsp[tail].dr_perm   = attr->perm;
  g_discoverstate.bd_tail                = nexttail;

  nxsem_post(&g_discoverstate.bd_exclsem);
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
  FAR struct bt_gatt_discover_params *params = arg;

  /* There is nothing that needs to be down here.  The parameters were
   * allocated on the stack and are long gone.
   */

  wlinfo("Discover destroy.  params %p\n", params);
  DEBUGASSERT(params != NULL && g_discoverstate.bd_discovering);
  UNUSED(params);

  memset(&g_discoverstate.bd_params, 0, sizeof(struct btnet_discoverstate_s));
  nxsem_destroy(&g_discoverstate.bd_exclsem);
  g_discoverstate.bd_discovering = false;
}

/****************************************************************************
 * Name: btnet_discover_result
 *
 * Description:
 *   This function implements the SIOCBTDISCGET IOCTL command.  It returns
 *   the current, buffered discovered handles.
 *
 * Input Parameters:
 *   result - Location to return the discovery result data
 *   maxrsp - The maximum number of responses that can be returned.
 *
 * Returned Value:
 *   On success, the actual number of discovery results obtain is returned.  A
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int btnet_discover_result(FAR struct bt_discresonse_s *result,
                                 uint8_t maxrsp)
{
  uint8_t head;
  uint8_t tail;
  uint8_t nrsp;
  bool discovering;
  int ret;

  discovering = g_discoverstate.bd_discovering;
  wlinfo("Discovering? %s\n", discovering ? "YES" : "NO");

  /* Get exclusive access to the discovery data while we are actively
   * discovering. The semaphore is uninitialized in other cases.
   */

  if (discovering)
   {
      ret = nxsem_wait(&g_discoverstate.bd_exclsem);
      if (ret < 0)
        {
          DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
          return ret;
        }
    }

  /* Copy all available results */

  head = g_discoverstate.bd_head;
  tail = g_discoverstate.bd_tail;

  for (nrsp = 0; nrsp < maxrsp && head != tail; nrsp++)
    {
      /* Copy data from the head index into the user buffer */

      result[nrsp].dr_handle = g_discoverstate.bd_rsp[head].dr_handle;
      result[nrsp].dr_perm   = g_discoverstate.bd_rsp[head].dr_perm;

      /* Increment the head index */

      if (++head >= CONFIG_BLUETOOTH_MAXDISCOVER)
        {
          head = 0;
        }
    }

  g_discoverstate.bd_head = head;

  if (discovering)
   {
      nxsem_post(&g_discoverstate.bd_exclsem);
   }

  return nrsp;
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
  wlinfo("Read complete: result %d length %u\n", result, length);

  DEBUGASSERT(conn != NULL && g_rdstate.rd_pending);

  if (length > HCI_GATTRD_DATA)
    {
      g_rdstate.rd_result = ENFILE;
    }
  else
    {
      DEBUGASSERT((unsigned int)result < UINT8_MAX);

      g_rdstate.rd_result = result;
      g_rdstate.rd_head   = 0;
      g_rdstate.rd_tail   = length;
      memcpy(g_rdstate.rd_data, data, length);
    }

  g_rdstate.rd_pending    = false;
}

/****************************************************************************
 * Name: btnet_read_result
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   btreq - IOCTL command data
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int btnet_read_result(FAR struct btreq_s *btreq)
{
  int head;
  int rdlen;

  DEBUGASSERT(btreq != NULL && btreq->btr_rddata != NULL);

  /* Is the read complete? */

  btreq->btr_rdpending = g_rdstate.rd_pending;
  if (!g_rdstate.rd_pending)
    {
      return -EBUSY;
    }

  /* Yes... return the read data */

  head  = g_rdstate.rd_head;
  if (head >= g_rdstate.rd_tail)
    {
      return -ENODATA;
    }

  rdlen = btreq->btr_rdsize;
  if (rdlen + head >= g_rdstate.rd_tail)
    {
      rdlen = g_rdstate.rd_tail - head;
    }

  btreq->btr_rdresult = g_rdstate.rd_result;
  btreq->btr_rdsize   = rdlen;
  memcpy(btreq->btr_rddata, &g_rdstate.rd_data[head], rdlen);

  g_rdstate.rd_head = head + rdlen;
  return OK;
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
  wlinfo("Exchange %s\n", result == 0 ? "succeeded" : "failed");
  g_gattwrresult.br_pending = false;
  g_gattwrresult.br_result  = result;
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

  wlinfo("cmd=%04x arg=%ul\n", cmd, arg);
  DEBUGASSERT(netdev != NULL && netdev->d_private != NULL);

  if (btreq == NULL)
    {
      return -EINVAL;
    }

  wlinfo("ifname: %s\n", btreq->btr_name);

  /* Process the command */

  switch (cmd)
    {
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
                                     btreq->btr_advad);
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

      /* SIOCBTSCANSTOP:  Stop LE scanning and discard any buffered results. */

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
          /* Check if we are still waiting for the result of the last exchange */

          if (g_exchangeresult.br_pending)
            {
              wlwarn("WARNING:  Last exchange not yet complete\n");
              ret = -EBUSY;
            }
          else
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
                  ret = bt_gatt_exchange_mtu(conn, btnet_exchange_rsp);
                  if (ret == OK)
                    {
                      g_exchangeresult.br_pending = true;
                      g_exchangeresult.br_result  = EBUSY;
                    }

                  bt_conn_release(conn);
                }
            }
        }
        break;

      /* SIOCBTEXRESULT: Get the result of the MTU exchange */

      case SIOCBTEXRESULT:
        {
          btreq->btr_expending = g_exchangeresult.br_pending;
          btreq->btr_exresult  = g_exchangeresult.br_result;
          ret                  = OK;
        }
        break;

      /* SIOCBTDISCOVER:  Starts GATT discovery */

      case SIOCBTDISCOVER:
        {
          FAR struct bt_conn_s *conn;

          /* Check if discovery is already in progress */

          if (g_discoverstate.bd_discovering)
            {
              wlwarn("WARNING:  Discovery is already in progress\n");
              ret = -EBUSY;
            }
          else
            {
              /* Get the connection associated with the provided LE address */

              conn = bt_conn_lookup_addr_le(&btreq->btr_dpeer);
              if (conn == NULL)
                {
                  wlwarn("WARNING:  Peer not connected\n");
                  ret = -ENOTCONN;
                }
              else
                {
                  FAR struct bt_gatt_discover_params_s *params;

                  /* Set up the query */

                  g_discoverstate.bd_uuid.type   = BT_UUID_16;
                  g_discoverstate.bd_uuid.u.u16  = btreq->btr_duuid16;

                  params                         = &g_discoverstate.bd_params;
                  params->uuid                   = &g_discoverstate.bd_uuid;
                  params->func                   = btnet_discover_func;
                  params->destroy                = btnet_discover_destroy;
                  params->start_handle           = btreq->btr_dstart;
                  params->end_handle             = btreq->btr_dend;

                  nxsem_init(&g_discoverstate.bd_exclsem, 0, 1);
                  g_discoverstate.bd_discovering = true;
                  g_discoverstate.bd_head        = 0;
                  g_discoverstate.bd_tail        = 0;

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
                      btnet_discover_destroy(params);
                    }

                  bt_conn_release(conn);
                }
            }
        }
        break;

      /* SIOCBTDISCGET:  Return discovered results buffered since the call time
       * that the SIOCBTDISCGET command was invoked.
       */

      case SIOCBTDISCGET:
        {
          ret = btnet_discover_result(btreq->btr_grsp, btreq->btr_gnrsp);
          wlinfo("Get discovery results: %d\n", ret);

          if (ret >= 0)
            {
              btreq->btr_nrsp = ret;
              ret = OK;
            }
        }
        break;

      /* SIOCBTGATTRD:  Initiate read of GATT data */

      case SIOCBTGATTRD:
        {
          /* Is there already a read in progress?  The current
           * implementation can support only one read at a time.
           * REVISIT.. see suggested design improvement above.
           */

          if (g_rdstate.rd_pending)
            {
              wlwarn("WARNING:  Read pending\n");
              ret = -EBUSY;
            }
          else
            {
              FAR struct bt_conn_s *conn;

              /* Get the connection associated with the provided LE address */

              conn = bt_conn_lookup_addr_le(&btreq->btr_rdpeer);
              if (conn == NULL)
                {
                  wlwarn("WARNING:  Peer not connected\n");
                  ret = -ENOTCONN;
                }
              else
                {
                  /* Set up for the read */

                  g_rdstate.rd_pending = true;
                  g_rdstate.rd_head    = 0;
                  g_rdstate.rd_tail    = 0;

                  /* Initiate read.. single or multiple? */

                  if (btreq->btr_rdnhandles == 1)
                    {
                      /* Read single */

                      ret = bt_gatt_read(conn, btreq->btr_rdhandles[0],
                                         btreq->btr_rdoffset,
                                         btnet_read_callback);
                    }
                  else if (btreq->btr_rdnhandles < HCI_GATT_MAXHANDLES)
                    {
                      /* Read multiple */

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

                  bt_conn_release(conn);
                }
            }
        }
        break;

      /* SIOCBTGATTRDGET:  Get the result of the GATT data read */

      case SIOCBTGATTRDGET:
        {
          ret = btnet_read_result(btreq);
          wlinfo("Get read results: %d\n", ret);
        }
        break;

      /* SIOCBTGATTWR:  Write GATT data */

      case SIOCBTGATTWR:
        {
          DEBUGASSERT(btreq->btr_wrdata != NULL);

          /* Is there already a write in progress?  The current
           * implementation can support only one write at a time.
           * REVISIT.. see suggested design improvement above.
           */

          if (g_gattwrresult.br_pending)
            {
              wlwarn("WARNING:  Read pending\n");
              ret = -EBUSY;
            }
          else
            {
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

                  g_gattwrresult.br_pending = true;
                  g_gattwrresult.br_result  = EBUSY;

                  /* Initiate write */

                  ret = bt_gatt_write(conn, btreq->btr_wrhandle,
                                      btreq->btr_wrdata,
                                      btreq->btr_wrnbytes,
                                      bnet_write_callback);
                  if (ret < 0)
                    {
                      wlerr("ERROR:  Write operation failed: %d\n", ret);
                    }

                  bt_conn_release(conn);
                }
            }
        }
        break;

      /* SIOCBTGATTWRGET:  Get the result of the GATT data write */

      case SIOCBTGATTWRGET:
        {
          btreq->btr_wrpending = g_gattwrresult.br_pending;
          btreq->btr_wrresult  = g_gattwrresult.br_result;
          ret                  = OK;
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
