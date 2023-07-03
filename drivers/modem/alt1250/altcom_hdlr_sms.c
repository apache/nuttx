/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_sms.c
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

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>

#include "altcom_cmd_sms.h"
#include "altcom_errno.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_smsinit_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_init_req_s *out =
    (FAR struct apicmd_sms_init_req_s *)pktbuf;

  *altcid = APICMDID_SMS_INIT;

  out->types = ALTCOM_SMS_MSG_TYPE_SEND | ALTCOM_SMS_MSG_TYPE_RECV |
               ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT;
  out->storage_use = 1; /* always use storage */

  return sizeof(struct apicmd_sms_init_req_s);
}

int32_t altcom_smsfin_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  *altcid = APICMDID_SMS_FIN;

  return 0;
}

int32_t altcom_smssend_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int i;
  FAR struct apicmd_sms_send_req_s *out =
    (FAR struct apicmd_sms_send_req_s *)pktbuf;
  FAR struct sms_send_msg_s *msg = (FAR struct sms_send_msg_s *)arg[0];
  uint16_t msglen = *((FAR uint16_t *)arg[1]);
  bool en_status_report = *((FAR bool *)arg[2]);
  FAR struct sms_sc_addr_s *scaddr = (FAR struct sms_sc_addr_s *)arg[3];
  uint8_t chset = *((FAR uint8_t *)arg[4]);
  FAR uint8_t *dest_toa = (FAR uint8_t *)arg[5];

  if (msglen > pktsz)
    {
      return -ENOBUFS;
    }

  if ((msg->header.destaddrlen % 2) || (msg->header.datalen % 2) ||
      (msg->header.destaddrlen > (SMS_MAX_ADDRLEN * 2)) ||
      (msg->header.datalen > (SMS_MAX_DATALEN * 2)) ||
      (scaddr->addrlen % 2) || (scaddr->addrlen > (SMS_MAX_ADDRLEN * 2)))
    {
      /* destaddrlen and datalen must be even numbers */

      return -EINVAL;
    }

  out->sc_addr.toa = scaddr->toa;
  out->sc_addr.length = scaddr->addrlen;

  /* Swap sc address */

  for (i = 0; i < (scaddr->addrlen / 2); i++)
    {
      out->sc_addr.address[i] = htons(scaddr->addr[i]);
    }

  out->valid_indicator = ALTCOM_SMS_MSG_VALID_UD;
  out->valid_indicator |= en_status_report ? ALTCOM_SMS_MSG_VALID_SRR : 0;

  out->dest_addr.toa = (dest_toa == NULL) ? 0 : *dest_toa;
  out->dest_addr.length = msg->header.destaddrlen;

  /* Swap destination address */

  for (i = 0; i < (msg->header.destaddrlen / 2); i++)
    {
      out->dest_addr.address[i] = htons(msg->header.destaddr[i]);
    }

  switch (chset)
    {
      case SMS_CHSET_UCS2:
        chset = ALTCOM_SMS_CHSET_UCS2;
        break;
      case SMS_CHSET_GSM7:
        chset = ALTCOM_SMS_CHSET_GSM7;
        break;
      case SMS_CHSET_BINARY:
        chset = ALTCOM_SMS_CHSET_BINARY;
        break;
      default:
        return -EINVAL;
    }

  out->userdata.chset = ALTCOM_SMS_CHSET_UCS2;
  out->userdata.data_len = htons(msg->header.datalen);

  /* Swap data */

  for (i = 0; i < (msg->header.datalen / 2); i++)
    {
      out->user_data[i] = htons(msg->data[i]);
    }

  *altcid = APICMDID_SMS_SEND;

  size = sizeof(struct apicmd_sms_send_req_s) + msg->header.datalen;

  return size;
}

int32_t altcom_smsdelete_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_delete_s *out =
    (FAR struct apicmd_sms_delete_s *)pktbuf;
  uint16_t msg_index = *(FAR uint16_t *)arg[0];

  *altcid = APICMDID_SMS_DELETE;

  out->index = htons(msg_index);
  out->types = 0;

  return sizeof(struct apicmd_sms_delete_s);
}

int32_t altcom_smsreportrecv_pkt_compose(FAR void **arg, size_t arglen,
                                         uint8_t altver, FAR uint8_t *pktbuf,
                                         const size_t pktsz,
                                         FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_res_s *out =
    (FAR struct apicmd_sms_res_s *)pktbuf;

  *altcid = APICMDID_SMS_REPORT_RECV | ALTCOM_CMDID_REPLY_BIT;

  out->result = htonl(0); /* always success */

  return sizeof(struct apicmd_sms_res_s);
}

int32_t altcom_smscommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_sms_res_s *in =
    (FAR struct apicmd_sms_res_s *)pktbuf;

  return altcom_geterrcode(in->result);
}

int32_t altcom_smssend_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_sms_sendres_s *in =
    (FAR struct apicmd_sms_sendres_s *)pktbuf;
  FAR struct sms_refids_s *refid = (FAR struct sms_refids_s *)arg[0];
  uint16_t msglen = *((FAR uint16_t *)arg[1]);
  int32_t sendresult = altcom_geterrcode(in->result);

  if (sendresult >= 0)
    {
      refid->nrefid = in->mr_num;
      memcpy(refid->refid, in->mr_list, sizeof(refid->refid));
    }

  return (sendresult < 0) ? sendresult : msglen;
}

int32_t altcom_smsreportrecv_pkt_parse(FAR struct alt1250_dev_s *dev,
                                       FAR uint8_t *pktbuf, size_t pktsz,
                                       uint8_t altver, FAR void **arg,
                                       size_t arglen, FAR uint64_t *bitmap)
{
  int i;
  FAR struct apicmd_sms_reprecv_s *in =
    (FAR struct apicmd_sms_reprecv_s *)pktbuf;
  FAR uint16_t *msg_index = (FAR uint16_t *)arg[0];
  FAR uint16_t *msg_sz = (FAR uint16_t *)arg[1];
  FAR uint8_t *maxnum = (FAR uint8_t *)arg[2];
  FAR uint8_t *seqnum = (FAR uint8_t *)arg[3];
  FAR struct sms_recv_msg_header_s *msgheader =
    (FAR struct sms_recv_msg_header_s *)arg[4];

  *msg_index = ntohs(in->index);
  *maxnum = 0;
  *seqnum = 0;

  if (in->msg.type == ALTCOM_SMS_MSG_TYPE_RECV)
    {
      FAR struct sms_deliver_msg_s *deliver =
        (FAR struct sms_deliver_msg_s *)arg[4];

      *msg_sz = sizeof(struct sms_deliver_msg_s) +
        ntohs(in->msg.u.recv.userdata.data_len);

      if (in->msg.u.recv.valid_indicator & ALTCOM_SMS_MSG_VALID_CONCAT_HDR)
        {
          *maxnum = in->msg.u.recv.concat_hdr.max_num;
          *seqnum = in->msg.u.recv.concat_hdr.seq_num;
        }

      msgheader->msgtype = SMS_MSG_TYPE_DELIVER;
      memcpy(&msgheader->send_time, &in->msg.u.recv.sc_time,
             sizeof(msgheader->send_time));
      msgheader->srcaddrlen = in->msg.u.recv.src_addr.length;
      if (msgheader->srcaddrlen > SMS_MAX_ADDRLEN * 2)
        {
          m_err("Unexpected src addrlen: %u\n", msgheader->srcaddrlen);
          return -EINVAL;
        }

      /* Swap source address */

      for (i = 0; i < (msgheader->srcaddrlen / 2); i++)
        {
          msgheader->srcaddr[i] = ntohs(in->msg.u.recv.src_addr.address[i]);
        }

      msgheader->datalen = ntohs(in->msg.u.recv.userdata.data_len);
      if (msgheader->datalen > (SMS_MAX_DATALEN * 2))
        {
          m_err("Unexpected datalen: %u\n", msgheader->datalen);
          return -EINVAL;
        }

      /* Swap data */

      for (i = 0; i < (msgheader->datalen / 2); i++)
        {
          deliver->data[i] = ntohs(in->msg.u.recv.user_data[i]);
        }

      m_info("[recv msg] msg size: %u\n", *msg_sz);
      m_info("           maxnum: %u, seqnum: %u\n", *maxnum, *seqnum);
      m_info("           msgtype: %u\n", msgheader->msgtype);
      m_info("           srcaddrlen: %u\n", msgheader->srcaddrlen);
      m_info("           datalen: %u\n", msgheader->datalen);
    }
  else if (in->msg.type == ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT)
    {
      FAR struct sms_status_report_msg_s *report =
        (FAR struct sms_status_report_msg_s *)arg[4];

      *msg_sz = sizeof(struct sms_status_report_msg_s);

      msgheader->msgtype = SMS_MSG_TYPE_STATUS_REPORT;
      memcpy(&msgheader->send_time, &in->msg.u.delivery_report.sc_time,
             sizeof(msgheader->send_time));
      msgheader->srcaddrlen = 0;
      memset(msgheader->srcaddr, 0, sizeof(msgheader->srcaddr));
      msgheader->datalen = sizeof(struct sms_status_report_s);
      report->status_report.refid = in->msg.u.delivery_report.ref_id;
      report->status_report.status = in->msg.u.delivery_report.status;
      memcpy(&report->status_report.discharge_time,
             &in->msg.u.delivery_report.discharge_time,
             sizeof(report->status_report.discharge_time));

      m_info("[staus report] msg size: %u\n", *msg_sz);
      m_info("           msgtype: %u\n", msgheader->msgtype);
      m_info("           datalen: %u\n", msgheader->datalen);
      m_info("           refid: %u\n", report->status_report.refid);
      m_info("           status: %u\n", report->status_report.status);
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}
