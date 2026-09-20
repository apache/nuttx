/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_sdc_hci.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/debug.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <sdc_hci_cmd_controller_baseband.h>
#include <sdc_hci_cmd_info_params.h>
#include <sdc_hci_cmd_le.h>
#include <sdc_hci_cmd_link_control.h>
#include <sdc_hci_cmd_status_params.h>

#include "nrf54l_sdc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_sdc_cmd_s
{
  uint16_t opcode;
  uint8_t params;
  uint8_t response;
  bool status;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HCI wire lengths exclude the command header and return status byte.
 * Commands completing asynchronously return Command Status instead.
 */

static const struct nrf54l_sdc_cmd_s g_commands[] =
{
  {0x0406,  3,  0, true},  /* Disconnect */
  {0x041d,  2,  0, true},  /* Read Remote Version Information */
  {0x0c01,  8,  0, false}, /* Set Event Mask */
  {0x0c03,  0,  0, false}, /* Reset */
  {0x0c2d,  3,  3, false}, /* Read Transmit Power Level */
  {0x0c31,  1,  0, false}, /* Set Controller To Host Flow Control */
  {0x0c33,  7,  0, false}, /* Host Buffer Size */
  {0x0c35,  1,  0, false}, /* Host Number Of Completed Packets (variable) */
  {0x0c63,  8,  0, false}, /* Set Event Mask Page 2 */
  {0x1001,  0,  8, false}, /* Read Local Version Information */
  {0x1002,  0, 64, false}, /* Read Local Supported Commands */
  {0x1003,  0,  8, false}, /* Read Local Supported Features */
  {0x1009,  0,  6, false}, /* Read BD_ADDR */
  {0x1405,  2,  3, false}, /* Read RSSI */
  {0x2001,  8,  0, false}, /* LE Set Event Mask */
  {0x2002,  0,  3, false}, /* LE Read Buffer Size */
  {0x2003,  0,  8, false}, /* LE Read Local Supported Features */
  {0x2005,  6,  0, false}, /* LE Set Random Address */
  {0x2006, 15,  0, false}, /* LE Set Advertising Parameters */
  {0x2007,  0,  1, false}, /* LE Read Advertising Channel TX Power */
  {0x2008, 32,  0, false}, /* LE Set Advertising Data */
  {0x2009, 32,  0, false}, /* LE Set Scan Response Data */
  {0x200a,  1,  0, false}, /* LE Set Advertising Enable */
  {0x200b,  7,  0, false}, /* LE Set Scan Parameters */
  {0x200c,  2,  0, false}, /* LE Set Scan Enable */
  {0x200d, 25,  0, true},  /* LE Create Connection */
  {0x200e,  0,  0, false}, /* LE Create Connection Cancel */
  {0x200f,  0,  1, false}, /* LE Read Filter Accept List Size */
  {0x2010,  0,  0, false}, /* LE Clear Filter Accept List */
  {0x2011,  7,  0, false}, /* LE Add Device To Filter Accept List */
  {0x2012,  7,  0, false}, /* LE Remove Device From Filter Accept List */
  {0x2013, 14,  0, true},  /* LE Connection Update */
  {0x2014,  5,  0, false}, /* LE Set Host Channel Classification */
  {0x2015,  2,  7, false}, /* LE Read Channel Map */
  {0x2016,  2,  0, true},  /* LE Read Remote Features */
  {0x2017, 32, 16, false}, /* LE Encrypt */
  {0x2018,  0,  8, false}, /* LE Rand */
  {0x2019, 28,  0, true},  /* LE Enable Encryption */
  {0x201a, 18,  2, false}, /* LE Long Term Key Request Reply */
  {0x201b,  2,  2, false}, /* LE Long Term Key Request Negative Reply */
  {0x201c,  0,  8, false}, /* LE Read Supported States */
  {0x2022,  6,  2, false}, /* LE Set Data Length */
  {0x2023,  0,  4, false}, /* LE Read Suggested Default Data Length */
  {0x2024,  4,  0, false}, /* LE Write Suggested Default Data Length */
  {0x202f,  0,  8, false}, /* LE Read Maximum Data Length */
  {0x2030,  2,  4, false}, /* LE Read PHY */
  {0x2031,  3,  0, false}, /* LE Set Default PHY */
  {0x2032,  7,  0, true},  /* LE Set PHY */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_sdc_supported_commands
 *
 * Description:
 *   Report only commands implemented by this HCI transport.
 *
 ****************************************************************************/

static void nrf54l_sdc_supported_commands(void *response)
{
  sdc_hci_ip_supported_commands_t *cmd = response;

  memset(cmd, 0, sizeof(*cmd));
  cmd->hci_disconnect = 1;
  cmd->hci_read_remote_version_information = 1;
  cmd->hci_set_event_mask = 1;
  cmd->hci_reset = 1;
  cmd->hci_read_transmit_power_level = 1;
  cmd->hci_set_controller_to_host_flow_control = 1;
  cmd->hci_host_buffer_size = 1;
  cmd->hci_host_number_of_completed_packets = 1;
  cmd->hci_set_event_mask_page_2 = 1;
  cmd->hci_read_local_version_information = 1;
  cmd->hci_read_local_supported_features = 1;
  cmd->hci_read_bd_addr = 1;
  cmd->hci_read_rssi = 1;
  cmd->hci_le_set_event_mask = 1;
  cmd->hci_le_read_buffer_size_v1 = 1;
  cmd->hci_le_read_local_supported_features = 1;
  cmd->hci_le_set_random_address = 1;
#ifdef CONFIG_NRF54L_SDC_ADVERTISING
  cmd->hci_le_set_advertising_parameters = 1;
  cmd->hci_le_read_advertising_physical_channel_tx_power = 1;
  cmd->hci_le_set_advertising_data = 1;
  cmd->hci_le_set_scan_response_data = 1;
  cmd->hci_le_set_advertising_enable = 1;
#endif
#ifdef CONFIG_NRF54L_SDC_SCANNING
  cmd->hci_le_set_scan_parameters = 1;
  cmd->hci_le_set_scan_enable = 1;
#endif
#if NRF54L_SDC_CENTRAL_COUNT > 0
  cmd->hci_le_create_connection = 1;
  cmd->hci_le_create_connection_cancel = 1;
  cmd->hci_le_connection_update = 1;
  cmd->hci_le_set_host_channel_classification = 1;
  cmd->hci_le_enable_encryption = 1;
#endif
  cmd->hci_le_read_filter_accept_list_size = 1;
  cmd->hci_le_clear_filter_accept_list = 1;
  cmd->hci_le_add_device_to_filter_accept_list = 1;
  cmd->hci_le_remove_device_from_filter_accept_list = 1;
  cmd->hci_le_read_channel_map = 1;
  cmd->hci_le_read_remote_features = 1;
  cmd->hci_le_encrypt = 1;
  cmd->hci_le_rand = 1;
#if CONFIG_NRF54L_SDC_PERIPHERAL_COUNT > 0
  cmd->hci_le_long_term_key_request_reply = 1;
  cmd->hci_le_long_term_key_request_negative_reply = 1;
#endif
  cmd->hci_le_read_supported_states = 1;
#ifdef CONFIG_NRF54L_SDC_DLE
  cmd->hci_le_set_data_length = 1;
  cmd->hci_le_read_suggested_default_data_length = 1;
  cmd->hci_le_write_suggested_default_data_length = 1;
  cmd->hci_le_read_maximum_data_length = 1;
#endif
#if defined(CONFIG_NRF54L_SDC_LE_2M_PHY) || \
    defined(CONFIG_NRF54L_SDC_LE_CODED_PHY)
  cmd->hci_le_read_phy = 1;
  cmd->hci_le_set_default_phy = 1;
  cmd->hci_le_set_phy = 1;
#endif
}

/****************************************************************************
 * Name: nrf54l_sdc_dispatch
 ****************************************************************************/

static uint8_t nrf54l_sdc_dispatch(uint16_t opcode, const void *params,
                                 void *response)
{
  switch (opcode)
    {
      case SDC_HCI_OPCODE_CMD_LC_DISCONNECT:
        return sdc_hci_cmd_lc_disconnect(params);
      case SDC_HCI_OPCODE_CMD_LC_READ_REMOTE_VERSION_INFORMATION:
        return sdc_hci_cmd_lc_read_remote_version_information(params);
      case SDC_HCI_OPCODE_CMD_CB_SET_EVENT_MASK:
        return sdc_hci_cmd_cb_set_event_mask(params);
      case SDC_HCI_OPCODE_CMD_CB_RESET:
        return nrf54l_sdc_reset();
      case SDC_HCI_OPCODE_CMD_CB_READ_TRANSMIT_POWER_LEVEL:
        return sdc_hci_cmd_cb_read_transmit_power_level(params, response);
      case SDC_HCI_OPCODE_CMD_CB_SET_CONTROLLER_TO_HOST_FLOW_CONTROL:
        return sdc_hci_cmd_cb_set_controller_to_host_flow_control(params);
      case SDC_HCI_OPCODE_CMD_CB_HOST_BUFFER_SIZE:
        return sdc_hci_cmd_cb_host_buffer_size(params);
      case SDC_HCI_OPCODE_CMD_CB_HOST_NUMBER_OF_COMPLETED_PACKETS:
        return sdc_hci_cmd_cb_host_number_of_completed_packets(params);
      case SDC_HCI_OPCODE_CMD_CB_SET_EVENT_MASK_PAGE_2:
        return sdc_hci_cmd_cb_set_event_mask_page_2(params);
      case SDC_HCI_OPCODE_CMD_IP_READ_LOCAL_VERSION_INFORMATION:
        return sdc_hci_cmd_ip_read_local_version_information(response);
      case SDC_HCI_OPCODE_CMD_IP_READ_LOCAL_SUPPORTED_COMMANDS:
        nrf54l_sdc_supported_commands(response);
        return 0;
      case SDC_HCI_OPCODE_CMD_IP_READ_LOCAL_SUPPORTED_FEATURES:
        return sdc_hci_cmd_ip_read_local_supported_features(response);
      case SDC_HCI_OPCODE_CMD_IP_READ_BD_ADDR:
        return sdc_hci_cmd_ip_read_bd_addr(response);
      case SDC_HCI_OPCODE_CMD_SP_READ_RSSI:
        return sdc_hci_cmd_sp_read_rssi(params, response);
      case SDC_HCI_OPCODE_CMD_LE_SET_EVENT_MASK:
        return sdc_hci_cmd_le_set_event_mask(params);
      case SDC_HCI_OPCODE_CMD_LE_READ_BUFFER_SIZE:
        return sdc_hci_cmd_le_read_buffer_size(response);
      case SDC_HCI_OPCODE_CMD_LE_READ_LOCAL_SUPPORTED_FEATURES:
        return sdc_hci_cmd_le_read_local_supported_features(response);
      case SDC_HCI_OPCODE_CMD_LE_SET_RANDOM_ADDRESS:
        return sdc_hci_cmd_le_set_random_address(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_ADV_PARAMS:
        return sdc_hci_cmd_le_set_adv_params(params);
      case SDC_HCI_OPCODE_CMD_LE_READ_ADV_PHYSICAL_CHANNEL_TX_POWER:
        return sdc_hci_cmd_le_read_adv_physical_channel_tx_power(response);
      case SDC_HCI_OPCODE_CMD_LE_SET_ADV_DATA:
        return sdc_hci_cmd_le_set_adv_data(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_SCAN_RESPONSE_DATA:
        return sdc_hci_cmd_le_set_scan_response_data(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_ADV_ENABLE:
        return sdc_hci_cmd_le_set_adv_enable(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_SCAN_PARAMS:
        return sdc_hci_cmd_le_set_scan_params(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_SCAN_ENABLE:
        return sdc_hci_cmd_le_set_scan_enable(params);
      case SDC_HCI_OPCODE_CMD_LE_CREATE_CONN:
        return sdc_hci_cmd_le_create_conn(params);
      case SDC_HCI_OPCODE_CMD_LE_CREATE_CONN_CANCEL:
        return sdc_hci_cmd_le_create_conn_cancel();
      case SDC_HCI_OPCODE_CMD_LE_READ_FILTER_ACCEPT_LIST_SIZE:
        return sdc_hci_cmd_le_read_filter_accept_list_size(response);
      case SDC_HCI_OPCODE_CMD_LE_CLEAR_FILTER_ACCEPT_LIST:
        return sdc_hci_cmd_le_clear_filter_accept_list();
      case SDC_HCI_OPCODE_CMD_LE_ADD_DEVICE_TO_FILTER_ACCEPT_LIST:
        return sdc_hci_cmd_le_add_device_to_filter_accept_list(params);
      case SDC_HCI_OPCODE_CMD_LE_REMOVE_DEVICE_FROM_FILTER_ACCEPT_LIST:
        return sdc_hci_cmd_le_remove_device_from_filter_accept_list(params);
      case SDC_HCI_OPCODE_CMD_LE_CONN_UPDATE:
        return sdc_hci_cmd_le_conn_update(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_HOST_CHANNEL_CLASSIFICATION:
        return sdc_hci_cmd_le_set_host_channel_classification(params);
      case SDC_HCI_OPCODE_CMD_LE_READ_CHANNEL_MAP:
        return sdc_hci_cmd_le_read_channel_map(params, response);
      case SDC_HCI_OPCODE_CMD_LE_READ_REMOTE_FEATURES:
        return sdc_hci_cmd_le_read_remote_features(params);
      case SDC_HCI_OPCODE_CMD_LE_ENCRYPT:
        return sdc_hci_cmd_le_encrypt(params, response);
      case SDC_HCI_OPCODE_CMD_LE_RAND:
        return sdc_hci_cmd_le_rand(response);
      case SDC_HCI_OPCODE_CMD_LE_ENABLE_ENCRYPTION:
        return sdc_hci_cmd_le_enable_encryption(params);
      case SDC_HCI_OPCODE_CMD_LE_LONG_TERM_KEY_REQUEST_REPLY:
        return sdc_hci_cmd_le_long_term_key_request_reply(params, response);
      case SDC_HCI_OPCODE_CMD_LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY:
        return sdc_hci_cmd_le_long_term_key_request_negative_reply(params,
                                                                 response);
      case SDC_HCI_OPCODE_CMD_LE_READ_SUPPORTED_STATES:

        /* Advertise the enabled individual Link Layer states. */

        memset(response, 0, 8);
#ifdef CONFIG_NRF54L_SDC_ADVERTISING
        *(uint8_t *)response |= 0x03;
#  if CONFIG_NRF54L_SDC_PERIPHERAL_COUNT > 0
        *(uint8_t *)response |= 0x8c;
#  endif
#endif
#ifdef CONFIG_NRF54L_SDC_SCANNING
        *(uint8_t *)response |= 0x30;
#endif
#if NRF54L_SDC_CENTRAL_COUNT > 0
        *(uint8_t *)response |= 0x40;
#endif
        return 0;
      case SDC_HCI_OPCODE_CMD_LE_SET_DATA_LENGTH:
        return sdc_hci_cmd_le_set_data_length(params, response);
      case SDC_HCI_OPCODE_CMD_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH:
        return sdc_hci_cmd_le_read_suggested_default_data_length(response);
      case SDC_HCI_OPCODE_CMD_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH:
        return sdc_hci_cmd_le_write_suggested_default_data_length(params);
      case SDC_HCI_OPCODE_CMD_LE_READ_MAX_DATA_LENGTH:
        return sdc_hci_cmd_le_read_max_data_length(response);
      case SDC_HCI_OPCODE_CMD_LE_READ_PHY:
        return sdc_hci_cmd_le_read_phy(params, response);
      case SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_PHY:
        return sdc_hci_cmd_le_set_default_phy(params);
      case SDC_HCI_OPCODE_CMD_LE_SET_PHY:
        return sdc_hci_cmd_le_set_phy(params);
      default:
        return 0x01; /* Unknown HCI Command */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_sdc_command
 *
 * Description:
 *   Execute a validated HCI command in the serialized SDC context.  Return
 *   the length of its Command Complete/Status event, or zero for successful
 *   Host Number Of Completed Packets, which must not generate an event.
 *   The caller supplies an HCI event buffer of at least 70 bytes.
 *
 ****************************************************************************/

size_t nrf54l_sdc_command(const uint8_t *command, uint8_t *event)
{
  const struct nrf54l_sdc_cmd_s *desc = NULL;
  uint16_t opcode = command[0] | (command[1] << 8);
  uint8_t status = 0x01;
  unsigned int length;
  unsigned int i;

  memset(event, 0, 70);
  for (i = 0; i < nitems(g_commands); i++)
    {
      if (g_commands[i].opcode == opcode)
        {
          desc = &g_commands[i];
          break;
        }
    }

  if (desc != NULL)
    {
      length = desc->params;
      if (opcode == SDC_HCI_OPCODE_CMD_CB_HOST_NUMBER_OF_COMPLETED_PACKETS &&
          command[2] >= 1)
        {
          length += command[3] * 4;
        }

      if (command[2] == length)
        {
          status = nrf54l_sdc_dispatch(opcode, &command[3], &event[6]);
        }
      else
        {
          status = 0x12; /* Invalid HCI Command Parameters */
        }
    }

  if (opcode == SDC_HCI_OPCODE_CMD_CB_HOST_NUMBER_OF_COMPLETED_PACKETS &&
      status == 0)
    {
      return 0;
    }

  if (status != 0)
    {
      wlwarn("HCI command 0x%04x failed: 0x%02x\n", opcode, status);
    }

  if (desc != NULL && desc->status)
    {
      event[0] = 0x0f; /* Command Status */
      event[1] = 4;
      event[2] = status;
      event[3] = 1;
      event[4] = command[0];
      event[5] = command[1];
    }
  else
    {
      event[0] = 0x0e; /* Command Complete */
      event[1] = 4 + (desc != NULL ? desc->response : 0);
      event[2] = 1;
      event[3] = command[0];
      event[4] = command[1];
      event[5] = status;
    }

  return event[1] + 2;
}
