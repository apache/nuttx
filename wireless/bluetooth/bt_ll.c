/****************************************************************************
 * /home/v01d/coding/nuttx_nrf51/nuttx/wireless/bluetooth/bt_ll.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_ll.h>
#include <errno.h>
#include <debug.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: move to Kconfig */

#define CONFIG_BLUETOOTH_LINKLAYER_WHITELIST_SIZE 4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bt_ll_s
{
  struct bt_driver_s bt_driver;
  const struct bt_ll_lowerhalf_s *lower;    /* Lower-half of Link Layer */

  struct bt_hci_cp_set_event_mask_s eventmask;
  struct bt_hci_cp_set_event_mask_s le_eventmask;

  struct bt_hci_cp_host_buffer_size_s hostbufsize;
  struct bt_hci_cp_set_ctl_to_host_flow_s flowctrl;

  struct bt_addr_le_s white_list[CONFIG_BLUETOOTH_LINKLAYER_WHITELIST_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/** HCI layer functions *****************************************************/

static int bt_ll_hci_open(FAR const struct bt_driver_s *btdev);
static int bt_ll_hci_send(FAR const struct bt_driver_s *btdev,
                          FAR struct bt_buf_s *buf);

/** Internal Link Layer functions *******************************************/

static int bt_ll_adv_report(struct bt_buf_s *outbuf, uint8_t pdutype,
                            bool random_addr, uint8_t *addr,
                            uint8_t *data, size_t data_len, int8_t rssi);

static int bt_ll_cmd_complete(struct bt_buf_s *buf, uint16_t opcode,
                              size_t retsize);
static int bt_ll_cmd_status(struct bt_buf_s *outbuf, uint16_t opcode,
                            uint8_t status, uint8_t pending_commands);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct bt_ll_s g_ll;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_ll_hci_open
 *
 * Description:
 *   This function is called by the host layer to "open" the device.
 *
 ****************************************************************************/

static int bt_ll_hci_open(FAR const struct bt_driver_s *btdev)
{
  /* Nothing to do */

  return OK;
}

/****************************************************************************
 * Name: bt_ll_hci_send
 *
 * Description:
 *   This function is called by the host layer whenever it needs to send
 *   an HCI packet to the controller.
 *
 ****************************************************************************/

static int bt_ll_hci_send(FAR const struct bt_driver_s *btdev,
                          FAR struct bt_buf_s *buf)
{
  int ret = OK;

  if (buf->type == BT_CMD)
    {
      FAR struct bt_hci_cmd_hdr_s *hdr =
          (struct bt_hci_cmd_hdr_s *)buf->data;
      uint16_t opcode = hdr->opcode;
      void *hci_params =
          bt_buf_consume(buf, sizeof(struct bt_hci_cmd_hdr_s));
      struct bt_buf_s *outbuf;

      outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

      wlinfo("Received HCI CMD: opcode 0x%x 0x%x\n", opcode >> 10,
             opcode & ((1 << 10) - 1));

      switch (opcode)
        {
          case BT_HCI_OP_RESET:
            {
              uint8_t *retval;

              wlinfo("RESET request\n");

              g_ll.lower->reset();

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));

              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_SCAN_PARAMS:
            {
              struct bt_hci_cp_le_set_scan_params_s *scan_params =
                  hci_params;

              if (g_ll.lower->state() == BT_LL_STATE_SCANNING)
                {
                  bt_ll_cmd_status(outbuf, opcode,
                                   BT_HCI_ERR_COMMAND_DISALLOWED, 1);
                  bt_hci_receive(outbuf);
                }
              else
                {
                  if (scan_params->window > scan_params->interval)
                    {
                      bt_ll_cmd_status(outbuf, opcode,
                                       BT_HCI_ERR_INVALID_PARAMETERS, 1);
                      bt_hci_receive(outbuf);
                    }
                  else
                    {
                      uint8_t *retval;

                      g_ll.lower->setup_scan(scan_params);

                      bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
                      retval = bt_buf_extend(outbuf, sizeof(uint8_t));
                      *retval = BT_HCI_SUCCESS;

                      bt_hci_receive(outbuf);
                    }
                }
            }
            break;
          case BT_HCI_OP_LE_SET_SCAN_ENABLE:
            {
              uint8_t *retval;
              struct bt_hci_cp_le_set_scan_enable_s *scan_enable =
                  hci_params;

              /* TODO: handle filter dup */

              g_ll.lower->scan(scan_enable->enable == BT_LE_SCAN_ENABLE);

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_ADV_DATA:
            {
              uint8_t *retval;

              struct bt_hci_cp_le_set_adv_data_s *advdata = hci_params;

              g_ll.lower->set_advdata(advdata);

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_SCAN_RSP_DATA:
            {
              uint8_t *retval;

              struct bt_hci_cp_le_set_scan_rsp_data_s *rspdata = hci_params;

              g_ll.lower->set_rspdata(rspdata);

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_ADV_PARAMETERS:
            {
              struct bt_hci_cp_le_set_adv_parameters_s *params = hci_params;

              if (g_ll.lower->state() == BT_LL_STATE_ADVERTISING)
                {
                  bt_ll_cmd_status(outbuf, opcode,
                                   BT_HCI_ERR_COMMAND_DISALLOWED, 1);
                  bt_hci_receive(outbuf);
                }
              else
                {
                  uint8_t *retval;

                  bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
                  retval = bt_buf_extend(outbuf, sizeof(uint8_t));

                  if (params->min_interval > params->max_interval)
                    {
                      *retval = BT_HCI_ERR_INVALID_PARAMETERS;
                    }
                  else if ((params->type == BT_LE_ADV_SCAN_IND ||
                            params->type == BT_LE_ADV_NONCONN_IND) &&
                           (params->min_interval < 100))
                    {
                      *retval = BT_HCI_ERR_INVALID_PARAMETERS;
                    }
                  else if (params->channel_map == 0 ||
                           params->channel_map > 7)
                    {
                      *retval = BT_HCI_ERR_INVALID_PARAMETERS;
                    }
                  else
                    {
                      ret = g_ll.lower->setup_advertise(params);

                      *retval = BT_HCI_SUCCESS;
                    }

                  bt_hci_receive(outbuf);
                }
            }
            break;
          case BT_HCI_OP_LE_SET_ADV_ENABLE:
            {
              uint8_t *retval;
              struct bt_hci_cp_le_set_adv_enable_s *adv = hci_params;

              ret = g_ll.lower->advertise(adv->enable);

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_RAND_ADDR:
            {
              uint8_t *retval;

              struct bt_addr_s *addr = hci_params;

              g_ll.lower->set_randaddr(addr);

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_READ_LOCAL_FEATURES:
          case BT_HCI_OP_LE_READ_LOCAL_FEATURES:
            {
              struct bt_hci_rp_le_read_local_features_s features;

              wlinfo("bt_ll: read local features\n");

              memset(&features, 0, sizeof(features));

              if (opcode == BT_HCI_OP_READ_LOCAL_FEATURES)
                {
                  features.features[4] = BT_LMP_LE | BT_LMP_NO_BREDR;
                }
              else
                {
                  /* TODO: inform encryption here when supported */
                }

              bt_ll_cmd_complete(outbuf, opcode, sizeof(features));

              memcpy(bt_buf_extend(outbuf, sizeof(features)), &features,
                     sizeof(features));

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_READ_REMOTE_FEATURES:
            {
              struct bt_hci_le_read_remote_features_s *features = hci_params;

              /* We have to respond this always */

              bt_ll_cmd_status(outbuf, opcode, BT_HCI_PENDING, 1);
              bt_hci_receive(outbuf);

              /* If features were already received,
               * this call will send them
               */

              g_ll.lower->get_remote_features(features->conn);
            }
            break;
          case BT_HCI_OP_LE_RAND:
            {
              struct bt_hci_rp_le_rand_s *randrsp;

              bt_ll_cmd_complete(outbuf, opcode, sizeof(*randrsp));

              randrsp = bt_buf_extend(outbuf, sizeof(*randrsp));
              randrsp->status = BT_HCI_SUCCESS;

              g_ll.lower->rand(randrsp->rand, sizeof(randrsp->rand));

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_READ_BD_ADDR:
            {
              struct bt_hci_rp_read_bd_addr_s bdaddr;

              wlinfo("bt_ll: read bd addr\n");

              bdaddr.status = 0;

              if (g_ll.lower->getaddress(&bdaddr.bdaddr) < 0)
                {
                  wlwarn("Public broadcast address not set, "
                         "returning fake value\n");

                  /* no public address, return 0xAAAAAAAAAAAA to keep host
                   * layer happy.
                   */

                  memset(&bdaddr.bdaddr, 0xaa, sizeof(struct bt_addr_s));
                }

              bt_ll_cmd_complete(outbuf, opcode, sizeof(bdaddr));

              memcpy(bt_buf_extend(outbuf, sizeof(bdaddr)), &bdaddr,
                     sizeof(bdaddr));

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_READ_BUFFER_SIZE:
            {
              struct bt_hci_rp_le_read_buffer_size_s bufsize;

              wlinfo("bt_ll: read buffer size\n");

              bufsize.status = 0;
              bufsize.le_max_len = BLUETOOTH_LE_MAX_DATA_PAYLOAD;
              bufsize.le_max_num = 1;

              bt_ll_cmd_complete(outbuf, opcode, sizeof(bufsize));

              memcpy(bt_buf_extend(outbuf, sizeof(bufsize)), &bufsize,
                     sizeof(bufsize));

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_READ_LOCAL_VERSION_INFO:
            {
              struct bt_hci_rp_read_local_version_info_s version;

              wlinfo("bt_ll: read local version\n");

              /* TODO: get from lower layer */

              version.status = BT_HCI_SUCCESS;
              version.manufacturer = BT_LL_COMPID_NORDIC;
              version.hci_version = BT_LL_VERSNUM_4_0;
              version.lmp_subversion = 0;

              bt_ll_cmd_complete(outbuf, opcode, sizeof(version));

              memcpy(bt_buf_extend(outbuf, sizeof(version)), &version,
                     sizeof(version));

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_SET_EVENT_MASK:
          case BT_HCI_OP_SET_EVENT_MASK:
            {
              uint8_t *retval;
              struct bt_hci_cp_set_event_mask_s *mask = hci_params;

              if (opcode == BT_HCI_OP_SET_EVENT_MASK)
                {
                  memcpy(&g_ll.eventmask, mask, sizeof(*mask));
                }
              else
                {
                  memcpy(&g_ll.le_eventmask, mask, sizeof(*mask));
                }

              bt_ll_cmd_complete(outbuf, opcode, 1);

              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = 0;

              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_HOST_BUFFER_SIZE:
            {
              uint8_t *retval;
              struct bt_hci_cp_host_buffer_size_s *hostbufsize = hci_params;

              memcpy(&g_ll.hostbufsize, hostbufsize, sizeof(*hostbufsize));

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;
              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_SET_CTL_TO_HOST_FLOW:
            {
              uint8_t *retval;
              struct bt_hci_cp_set_ctl_to_host_flow_s *flowctrl = hci_params;

              memcpy(&g_ll.flowctrl, flowctrl, sizeof(*flowctrl));

              bt_ll_cmd_complete(outbuf, opcode, sizeof(uint8_t));
              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;
              bt_hci_receive(outbuf);
            }
            break;
          case BT_HCI_OP_LE_LTK_REQ_NEG_REPLY:
            {
              /* TODO */
            }
            break;
          case BT_HCI_OP_LE_LTK_REQ_REPLY:
            {
              struct bt_hci_cp_le_ltk_req_reply_s *repl = hci_params;
              uint8_t *retval;
              uint16_t *handle;

              g_ll.lower->set_ltk(repl->handle, repl->ltk);

              bt_ll_cmd_complete(outbuf, BT_HCI_OP_LE_LTK_REQ_REPLY,
                                 sizeof(uint8_t) + sizeof(uint16_t));

              retval = bt_buf_extend(outbuf, sizeof(uint8_t));
              *retval = BT_HCI_SUCCESS;

              handle = bt_buf_extend(outbuf, sizeof(uint16_t));
              *handle = repl->handle;

              bt_hci_receive(outbuf);
            }
            break;
          default:
            {
              wlerr("unhandled HCI opcode 0x%x 0x%x\n", opcode >> 10,
                    opcode & ((1 << 10) - 1));

              bt_ll_cmd_status(outbuf, opcode,
                               BT_HCI_ERR_UNKNOWN_COMMAND, 1);
              bt_hci_receive(outbuf);
            }
            break;
        }
    }
  else if (buf->type == BT_ACL_OUT)
    {
      FAR struct bt_hci_acl_hdr_s *hdr = (FAR void *)buf->data;
      uint8_t *data = bt_buf_consume(buf, sizeof(struct bt_hci_acl_hdr_s));

      g_ll.lower->send_data(data, hdr->len, hdr->packet_boundary,
                            hdr->broadcast);
    }

  /* Release input buffer */

  bt_buf_release(buf);

  if (ret < 0)
    {
      return ret;
    }
  else
    {
      return buf->len;
    }
}

/****************************************************************************
 * Name: bt_ll_cmd_complete
 *
 * Description:
 *   Write a CMD_COMPLETE HCI event into the output buffer for the given
 *   command opcode.
 *
 ****************************************************************************/

static int bt_ll_cmd_complete(struct bt_buf_s *outbuf, uint16_t opcode,
                              size_t retsize)
{
  struct bt_hci_evt_hdr_s *evt;
  struct hci_evt_cmd_complete_s *cmd;

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  /* copy header */

  evt->evt = BT_HCI_EVT_CMD_COMPLETE;
  evt->len = sizeof(struct hci_evt_cmd_complete_s) + retsize;

  cmd = bt_buf_extend(outbuf, sizeof(*cmd));

  /* copy content */

  cmd->ncmd   = 1;
  cmd->opcode = opcode;

  /* return parameters will be completed by caller */

  return OK;
}

/****************************************************************************
 * Name: bt_ll_cmd_status
 *
 * Description:
 *   Write a CMD_STATUS HCI event into the output buffer for the given
 *   command opcode.
 *
 ****************************************************************************/

static int bt_ll_cmd_status(struct bt_buf_s *outbuf, uint16_t opcode,
                            uint8_t status, uint8_t pending_commands)
{
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_cmd_status_s *cmd;

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  /* copy header */

  evt->evt = BT_HCI_EVT_CMD_STATUS;
  evt->len = sizeof(struct bt_hci_evt_cmd_status_s);

  cmd = bt_buf_extend(outbuf, sizeof(*cmd));

  /* copy content */

  cmd->ncmd   = pending_commands;
  cmd->opcode = opcode;
  cmd->status = status;

  /* return parameters will be completed by caller */

  return OK;
}

/****************************************************************************
 * Name: bt_ll_adv_report
 *
 * Description:
 *   Write an HCI LE ADVERTISING REPORT into the output buffer using the
 *   values read from the LL received PDU.
 *
 ****************************************************************************/

static int bt_ll_adv_report(struct bt_buf_s *outbuf, uint8_t pdutype,
                            bool random_addr, uint8_t *addr,
                            uint8_t *data, size_t data_len, int8_t rssi)
{
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_le_meta_event_s *metaevt;
  uint8_t *num_reports;
  struct bt_hci_ev_le_advertising_report_s *report;

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  /* Generate header. The length of the report itself is built from the
   * report struct size which already includes one byte of data, the data
   * length detected in the PDU and the RSSI byte which is not included in
   * the struct (thus data_len -1 (extra byte) + 1 (rssi) = data_len)
   */

  evt->evt = BT_HCI_EVT_LE_META_EVENT;
  evt->len = sizeof(struct bt_hci_evt_le_meta_event_s) +
             sizeof(uint8_t) +
             sizeof(struct bt_hci_ev_le_advertising_report_s) + data_len;

  /* Meta event header */

  metaevt = bt_buf_extend(outbuf, sizeof(*metaevt));

  metaevt->subevent = BT_HCI_EVT_LE_ADVERTISING_REPORT;

  num_reports = bt_buf_extend(outbuf, sizeof(uint8_t));

  *num_reports = 1;

  /* Write the actual report */

  report = bt_buf_extend(outbuf, sizeof(*report) + data_len);

  BLUETOOTH_ADDRCOPY(report->addr.val, addr);
  report->addr.type = (random_addr ? BT_ADDR_LE_RANDOM : BT_ADDR_LE_PUBLIC);
  report->length = data_len;

  if (data_len > 0)
    {
      memcpy(report->data, data, data_len);
    }

  report->data[data_len] = rssi;

  switch (pdutype)
    {
      case BT_LE_PDUTYPE_ADV_SCAN_IND:
        report->evt_type = BT_LE_ADV_SCAN_IND;
        break;
      case BT_LE_PDUTYPE_SCAN_RSP:
        report->evt_type = BT_LE_ADV_SCAN_RSP;
        break;
      case BT_LE_PDUTYPE_ADV_NONCON_IND:
        report->evt_type = BT_LE_ADV_NONCONN_IND;
        break;
      case BT_LE_PDUTYPE_ADV_IND:
        report->evt_type = BT_LE_ADV_IND;
        break;
      case BT_LE_PDUTYPE_ADV_DIRECT_IND:
        report->evt_type = BT_LE_ADV_DIRECT_IND;
        break;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_ll_handle_adv_pdu
 *
 * Description:
 *   This function is to be used by the lower-half to delegate the processing
 *   of an advertisment PDU to the upper-half. This will generate the HCI
 *   advertising report which will sent back to the host layer.
 *
 ****************************************************************************/

void bt_ll_handle_adv_pdu(struct bt_adv_pdu_hdr_s *hdr, uint8_t *payload_ptr)
{
  struct bt_buf_s *outbuf;
  size_t data_length;

  /* Decode PDU */

  wlinfo("handling advertisement PDU: %i\n", hdr->type);

  switch (hdr->type)
    {
      case BT_LE_PDUTYPE_ADV_IND:
      case BT_LE_PDUTYPE_ADV_DIRECT_IND:
      case BT_LE_PDUTYPE_ADV_NONCON_IND:
      case BT_LE_PDUTYPE_ADV_SCAN_IND:
      case BT_LE_PDUTYPE_SCAN_RSP:
        break;

      default:
        return;  /* Discard non ADV PDUs */
        break;
    }

  /* actual length of data within payload, excluding address field */

  data_length = hdr->length - BLUETOOTH_ADDRSIZE;

  if (hdr->type == BT_LE_PDUTYPE_ADV_DIRECT_IND)
    {
      /* TODO: this should only be considered if the intended address is
       * our own address
       */
    }

  /* prepare HCI output buffer */

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  if (hdr->type == BT_LE_PDUTYPE_ADV_DIRECT_IND)
    {
      struct bt_adv_direct_ind_pdu_payload_s *payload =
          (struct bt_adv_direct_ind_pdu_payload_s *)payload_ptr;

      bt_ll_adv_report(outbuf, hdr->type, hdr->txadd, payload->adv_addr,
                       NULL, 0, 127);
    }
  else
    {
      bt_adv_indirect_pdu_payload_t *payload =
          (bt_adv_indirect_pdu_payload_t *)payload_ptr;

      bt_ll_adv_report(outbuf, hdr->type, hdr->txadd, payload->adv_addr,
                       payload->adv_data, data_length, 127);
    }

  /* TODO: report real RSSI */

  /* send out HCI buffer */

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_whitelisted
 *
 * Description:
 *   Allows to check if a given address is in the white list or not.
 *
 ****************************************************************************/

bool bt_ll_whitelisted(struct bt_addr_le_s addr)
{
  int i;

  for (i = 0; i < CONFIG_BLUETOOTH_LINKLAYER_WHITELIST_SIZE; i++)
    {
      if (!BLUETOOTH_ADDRCMP(g_ll.white_list[i].val, BT_ADDR_ANY) &&
          BLUETOOTH_ADDRCMP(g_ll.white_list[i].val, addr.val) &&
          g_ll.white_list[i].type == addr.type)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: bt_ll_advtype_to_pdutype
 *
 * Description:
 *   Utility function to translate from a given HCI advertising type constant
 *   to a PDU type constant to be used by the link-layer.
 *
 ****************************************************************************/

int bt_ll_advtype_to_pdutype(int advtype)
{
  switch (advtype)
    {
      case BT_LE_ADV_SCAN_IND:
        return BT_LE_PDUTYPE_ADV_SCAN_IND;
        break;
      case BT_LE_ADV_SCAN_RSP:
        return BT_LE_PDUTYPE_SCAN_RSP;
        break;
      case BT_LE_ADV_NONCONN_IND:
        return BT_LE_PDUTYPE_ADV_NONCON_IND;
        break;
      case BT_LE_ADV_IND:
        return BT_LE_PDUTYPE_ADV_IND;
        break;
      case BT_LE_ADV_DIRECT_IND:
        return BT_LE_PDUTYPE_ADV_DIRECT_IND;
        break;
      default:

        /* should not get here */

        ASSERT(false);
        break;
    }
}

/****************************************************************************
 * Name: bt_ll_sca_to_ppm
 *
 * Description:
 *   Utility function to convert a slave-clock accuracy (SCA) value, as
 *   defined by standard, to a number in PPM (parts per million). In other
 *   words, this returns an accuracy measured in microseconds.
 *
 ****************************************************************************/

uint32_t bt_ll_sca_to_ppm(uint8_t sca)
{
  switch (sca)
    {
      case BT_LE_0_20_PPM:
        return 20;
        break;
      case BT_LE_21_30_PPM:
        return 30;
        break;
      case BT_LE_31_50_PPM:
        return 50;
        break;
      case BT_LE_51_75_PPM:
        return 75;
        break;
      case BT_LE_76_100_PPM:
        return 100;
        break;
      case BT_LE_101_150_PPM:
        return 150;
        break;
      case BT_LE_151_250_PPM:
        return 250;
        break;
      case BT_LE_251_500_PPM:
        return 500;
        break;
    }

  /* should not reach here */

  PANIC();
}

/****************************************************************************
 * Name: bt_ll_on_data
 *
 * Description:
 *   Called by the lower half on establishing a new connection. It will
 *   generate a "connection complete" event to be sent to the host layer.
 *
 ****************************************************************************/

void bt_ll_on_connection(struct bt_adv_pdu_hdr_s *hdr,
                         struct bt_connect_req_pdu_payload_s *payload,
                         uint8_t status, uint8_t conn_id, uint8_t role)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_le_meta_event_s *metaevt;
  struct bt_hci_evt_le_conn_complete_s *conn;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  /* Generate header */

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  evt->evt = BT_HCI_EVT_LE_META_EVENT;
  evt->len = sizeof(struct bt_hci_evt_le_meta_event_s) +
             sizeof(struct bt_hci_evt_le_conn_complete_s);

  /* Describe meta event */

  metaevt = bt_buf_extend(outbuf, sizeof(*metaevt));

  metaevt->subevent = BT_HCI_EVT_LE_CONN_COMPLETE;

  /* Connection complete */

  conn = bt_buf_extend(outbuf, sizeof(*conn));
  conn->status = status;
  conn->handle = conn_id;
  conn->role = role;
  conn->peer_addr.type = hdr->txadd;
  BLUETOOTH_ADDRCOPY(conn->peer_addr.val, payload->init_addr);
  conn->interval = payload->interval;
  conn->latency = payload->latency;
  conn->supv_timeout = payload->timeout;
  conn->clock_accuracy = payload->sca;    /* TODO: should be master's */

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_on_connection_closed
 *
 * Description:
 *   Called by the lower half when the connection is closed gracefully
 *
 ****************************************************************************/

void bt_ll_on_connection_closed(uint8_t conn_id, uint8_t reason)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_disconn_complete_s *disc;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  /* Generate header */

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  evt->evt = BT_HCI_EVT_DISCONN_COMPLETE;
  evt->len = sizeof(struct bt_hci_evt_disconn_complete_s);

  disc = bt_buf_extend(outbuf, sizeof(*disc));

  disc->handle = conn_id;
  disc->reason = reason;
  disc->status = BT_HCI_SUCCESS;

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_update_conn_params
 *
 * Description:
 *   Called by the lower half when the connection params are updated
 *
 ****************************************************************************/

void bt_ll_update_conn_params(uint8_t conn, uint16_t interval,
                              uint16_t latency, uint16_t timeout)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_le_meta_event_s *metaevt;
  struct bt_hci_evt_le_conn_update_s *params;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  /* Generate header */

  evt = bt_buf_extend(outbuf, sizeof(*evt));

  evt->evt = BT_HCI_EVT_LE_META_EVENT;
  evt->len = sizeof(struct bt_hci_evt_le_meta_event_s) +
             sizeof(struct bt_hci_evt_le_conn_update_s);

  /* Describe meta event */

  metaevt = bt_buf_extend(outbuf, sizeof(*metaevt));

  metaevt->subevent = BT_HCI_EVT_LE_CONN_UPDATE;

  /* Connection complete */

  params = bt_buf_extend(outbuf, sizeof(*params));
  params->status = BT_HCI_SUCCESS;
  params->handle = conn;
  params->interval = interval;
  params->latency = latency;
  params->timeout = timeout;

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_ack_packets
 *
 * Description:
 *   Called by the lower half to report the number of packets to acknowledge
 *   (may be zero, since the controller is expected to keep the host up to
 *   date on outstanding packets.
 *
 ****************************************************************************/

void bt_ll_ack_packets(uint8_t conn_id, size_t num_packets)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *evt;
  struct bt_hci_evt_num_completed_packets_s *numpackets;
  struct bt_hci_handle_count_s *handlecount;

  /* Inform of completed data packets */

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  evt = bt_buf_extend(outbuf, sizeof(*evt));
  evt->evt = BT_HCI_EVT_NUM_COMPLETED_PACKETS;
  evt->len = sizeof(*numpackets) + sizeof(*handlecount);

  numpackets = bt_buf_extend(outbuf, sizeof(*numpackets) +
                             sizeof(*handlecount));

  /* Report either one packet sent or zero */

  numpackets->num_handles = 1;
  handlecount = numpackets->h;
  handlecount->count = num_packets;
  handlecount->handle = conn_id;

  /* send to host layer */

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_on_data
 *
 * Description:
 *   Called by the lower half on reception of a DATA PDU. This will create
 *   the correponding ACL packet to be sent to the host layer.
 *
 ****************************************************************************/

void bt_ll_on_data(const struct bt_data_pdu_hdr_s *hdr,
                   const uint8_t *payload,
                   uint8_t conn_id)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_acl_hdr_s *acl;
  uint8_t *data;

  if (hdr->llid != BT_LE_DATA_PDU1 || hdr->length != 0)
    {
      /* Send current data, if any */

      outbuf = bt_buf_alloc(BT_ACL_IN, NULL, BLUETOOTH_H4_HDRLEN);

      acl = bt_buf_extend(outbuf, sizeof(*acl));

      /* write ACL header */

      acl->handle = conn_id;

      if (hdr->llid == BT_LE_DATA_PDU1)
        {
          /* L2CAP fragment */

          acl->packet_boundary = BT_HCI_ACL_CONTINUATION;
        }
      else if (hdr->llid == BT_LE_DATA_PDU2)
        {
          /* New or self-contained L2CAP message */

          acl->packet_boundary = BT_HCI_ACL_NEW;
        }

      acl->len = hdr->length;

      acl->broadcast = 0; /* TODO: ? */

      /* copy payload data */

      data = bt_buf_extend(outbuf, acl->len);
      memcpy(data, payload, acl->len);

      /* send to host layer */

      bt_hci_receive(outbuf);
    }
}

/****************************************************************************
 * Name: bt_ll_send_rem_features
 *
 * Description:
 *   Called by the lower half on reception when remote features are available
 *
 ****************************************************************************/

void bt_ll_send_rem_features(uint8_t conn, const uint8_t *features)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *hdr;
  struct bt_hci_evt_le_meta_event_s *meta;
  struct bt_hci_rp_le_read_remote_features_s *rem;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  hdr = bt_buf_extend(outbuf, sizeof(struct bt_hci_evt_hdr_s));

  hdr->evt = BT_HCI_EVT_LE_META_EVENT;
  hdr->len = sizeof(*meta) + sizeof(*rem);

  meta = bt_buf_extend(outbuf, sizeof(struct bt_hci_evt_le_meta_event_s));

  meta->subevent = BT_HCI_EVT_LE_READ_REM_FEAT_COMPLETE;

  rem = bt_buf_extend(outbuf, sizeof(*rem));

  rem->conn = conn;
  rem->status = BT_HCI_SUCCESS;
  memcpy(rem->features, features, sizeof(rem->features));

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_request_ltk
 *
 * Description:
 *   Called by the lower half to request the LTK handled by the host
 *
 ****************************************************************************/

void bt_ll_request_ltk(uint8_t conn, uint16_t ediv, uint64_t rand)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *hdr;
  struct bt_hci_evt_le_meta_event_s *meta;
  struct bt_hci_evt_le_ltk_request_s *req;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  hdr = bt_buf_extend(outbuf, sizeof(struct bt_hci_evt_hdr_s));

  hdr->evt = BT_HCI_EVT_LE_META_EVENT;
  hdr->len = sizeof(*meta) + sizeof(*req);

  meta = bt_buf_extend(outbuf, sizeof(struct bt_hci_evt_le_meta_event_s));

  meta->subevent = BT_HCI_EVT_LE_LTK_REQUEST;

  req = bt_buf_extend(outbuf, sizeof(*req));

  req->handle = conn;
  req->ediv = ediv;
  req->rand = rand;

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_encryption_changed
 *
 * Description:
 *   Called by the lower half to report that encryption was enabled/disabled
 *
 ****************************************************************************/

void bt_ll_encryption_changed(uint8_t conn, bool enabled)
{
  struct bt_buf_s *outbuf;
  struct bt_hci_evt_hdr_s *hdr;
  struct bt_hci_evt_encrypt_change_s *enc;

  outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);

  hdr = bt_buf_extend(outbuf, sizeof(struct bt_hci_evt_hdr_s));

  hdr->evt = BT_HCI_EVT_ENCRYPT_CHANGE;
  hdr->len = sizeof(*enc);

  enc = bt_buf_extend(outbuf, sizeof(*enc));

  enc->encrypt = enabled ? 1 : 0;
  enc->handle = conn;
  enc->status = BT_HCI_SUCCESS;

  bt_hci_receive(outbuf);
}

/****************************************************************************
 * Name: bt_ll_initialize
 *
 * Description:
 *   Initialize the Link Layer. This will receive a pointer to the
 *   architecture dependant lower-half and register itself with the
 *   networking layer using the HCI protocol for communication.
 *
 ****************************************************************************/

int bt_ll_initialize(const struct bt_ll_lowerhalf_s *lower)
{
  memset(&g_ll, 0, sizeof(g_ll));

  g_ll.lower = lower;

  g_ll.bt_driver.head_reserve = 0;
  g_ll.bt_driver.open = bt_ll_hci_open;
  g_ll.bt_driver.send = bt_ll_hci_send;

  bt_netdev_register(&g_ll.bt_driver);

  return OK;
}
