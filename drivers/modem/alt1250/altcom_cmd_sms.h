/****************************************************************************
 * drivers/modem/alt1250/altcom_cmd_sms.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SMS_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/net/sms.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_SMS_MSG_VALID_UD         (0x01 << 0)
#define ALTCOM_SMS_MSG_VALID_SRR        (0x01 << 1)
#define ALTCOM_SMS_MSG_VALID_CONCAT_HDR (0x01 << 3)
#define ALTCOM_SMS_MSG_VALID_TOA        (0x01 << 4)

#define ALTCOM_SMS_DELIVERY_STAT_CAT_MASK (0x60)
#define ALTCOM_SMS_DELIVERY_STAT_CAT_OK   (0x00)
#define ALTCOM_SMS_DELIVERY_STAT_CAT_PEND (0x01 << 6)
#define ALTCOM_SMS_DELIVERY_STAT_CAT_FAIL (0x01 << 7)

#define ALTCOM_SMS_MSG_TYPE_SEND (0x01 << 0)
#define ALTCOM_SMS_MSG_TYPE_RECV (0x01 << 1)
#define ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT (0x01 << 2)

#define ALTCOM_SMS_CHSET_GSM7   0
#define ALTCOM_SMS_CHSET_BINARY 1
#define ALTCOM_SMS_CHSET_UCS2   2

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct apicmd_sms_time_s
{
  uint8_t year; /* Years (0-99) */
  uint8_t mon;  /* Month (1-12) */
  uint8_t mday; /* Day of the month (1-31) */
  uint8_t hour; /* Hours (0-23) */
  uint8_t min;  /* Minutes (0-59) */
  uint8_t sec;  /* Seconds (0-59) */
  int8_t  tz;   /* Time zone in hour (-24 - +24) */
} end_packed_struct;

begin_packed_struct struct apicmd_sms_addr_s
{
  /* Type of address is reserved. */

  uint8_t toa;
  uint8_t length;
  uint16_t address[SMS_MAX_ADDRLEN];
} end_packed_struct;

begin_packed_struct struct apicmd_sms_concat_hdr_s
{
  /* Contain a modulo 256 counter indicating the reference number
   * for a particular concatenated short message.
   */

  uint8_t ref_num;

  /* Contain a value in the range 0 to 255 indicating the total number of
   * short messages within the concatenated short message.
   */

  uint8_t max_num;

  /* Contain a value in the range 0 to 255 indicating the sequence number
   * of a particular short message within the concatenated short message.
   */

  uint8_t seq_num;
} end_packed_struct;

begin_packed_struct struct apicmd_sms_userdata_s
{
  /* Set the character set used for SMS */

  uint8_t chset;

  /* Buffer length of User-Data field. */

  uint16_t data_len;

  /* User data in utf-8 format. */

  uint8_t *data;
} end_packed_struct;

begin_packed_struct struct apicmd_sms_msg_s
{
  uint8_t  type;

  union
    {
      /* SMS-DELIVER */

      begin_packed_struct struct
        {
          uint8_t valid_indicator;
          struct apicmd_sms_addr_s src_addr;
          struct apicmd_sms_time_s sc_time;
          struct apicmd_sms_concat_hdr_s concat_hdr;
          struct apicmd_sms_userdata_s userdata;

          /* Variable length array */

          uint16_t user_data[0];
        }
      end_packed_struct recv;

      /* SMS-STATUS-REPORT */

      begin_packed_struct struct
        {
          uint8_t status;
          struct apicmd_sms_time_s sc_time;
          uint8_t ref_id;
          struct apicmd_sms_time_s discharge_time;
        }
      end_packed_struct delivery_report;
    } u;
} end_packed_struct;

/* Command format for these response
 * APICMDID_SMS_INIT, APICMDID_SMS_FIN, APICMDID_SMS_REPORT_RECV,
 * APICMDID_SMS_DELETE
 */

begin_packed_struct struct apicmd_sms_res_s
{
  int32_t result;
} end_packed_struct;

/* Command format for APICMDID_SMS_INIT request */

begin_packed_struct struct apicmd_sms_init_req_s
{
  uint8_t types;
  uint8_t storage_use;
} end_packed_struct;

/* Command format for APICMDID_SMS_SEND request */

begin_packed_struct struct apicmd_sms_send_req_s
{
  struct apicmd_sms_addr_s sc_addr;

  /* SMS-SUBMIT */

  uint8_t valid_indicator;
  struct apicmd_sms_addr_s dest_addr;
  struct apicmd_sms_userdata_s userdata;

  /* Variable length array */

  uint16_t user_data[0];
} end_packed_struct;

/* Command format for APICMDID_SMS_SEND response */

begin_packed_struct struct apicmd_sms_sendres_s
{
  int32_t result;
  uint8_t mr_num;
  uint8_t mr_list[SMS_CONCATENATE_MAX];
} end_packed_struct;

/* Command format for APICMDID_SMS_REPORT_RECV request */

begin_packed_struct struct apicmd_sms_reprecv_s
{
  uint16_t index;
  struct apicmd_sms_addr_s sc_addr;
  struct apicmd_sms_msg_s msg;
} end_packed_struct;

/* Command format for APICMDID_SMS_DELETE request */

begin_packed_struct struct apicmd_sms_delete_s
{
  uint16_t index;
  uint8_t types;
} end_packed_struct;

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SMS_H */
