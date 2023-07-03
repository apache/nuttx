/****************************************************************************
 * drivers/modem/alt1250/altcom_pkt.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_PKT_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <arpa/inet.h>

#include <nuttx/modem/alt1250.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_HDR_MAGICNUMBER (0xFEEDBAC5)

#define ALTCOM_CMDOPT_CHECKSUM_EN  (1 << 0)

#define ALTCOM_PAYLOAD_SIZE_MAX    (4112)
#define ALTCOM_PAYLOAD_SIZE_MAX_V4 (3092)
#define ALTCOM_PKT_SIZE_MAX        (ALTCOM_PAYLOAD_SIZE_MAX \
                                    + sizeof(struct altcom_cmdhdr_s) \
                                    + sizeof(struct altcom_cmdfooter_s))
#define ALTCOM_RX_PKT_SIZE_MAX     (ALTCOM_PAYLOAD_SIZE_MAX_V4 \
                                    + sizeof(struct altcom_cmdhdr_s))

#define LTE_RESULT_OK     (0)      /**< Result code on success */
#define LTE_RESULT_ERROR  (1)      /**< Result code on failure */
#define LTE_RESULT_CANCEL (2)      /**< Result code on cancel */

#define ALTCOM_CMD_POWER_ON_REPLY_SIZE  (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct altcom_cmdhdr_s
{
  uint32_t magic;
  uint8_t  ver;
  uint8_t  seqid;
  uint16_t cmdid;
  uint16_t transid;
  uint16_t datalen;
  union
    {
      uint16_t v1_options;
      uint16_t v4_hdr_cksum;
    };
  union
    {
      uint16_t v1_checksum;
      uint16_t v4_data_cksum;
    };
  uint8_t payload[0];
} end_packed_struct;

begin_packed_struct struct altcom_cmdfooter_s
{
  uint16_t reserve;
  uint16_t checksum;
} end_packed_struct;

begin_packed_struct struct altcom_errind_s
{
  uint8_t ver;
  uint8_t seqid;
  uint16_t cmdid;
  uint16_t transid;
  uint16_t dtlen;
  union
    {
      uint16_t v1_options;
      uint16_t v4_hdr_cksum;
    };
  union
    {
      uint16_t v1_checksum;
      uint16_t v4_data_cksum;
    };
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline int is_errind(uint16_t cmdid)
{
  return (cmdid == ALTCOM_CMDID_ERRIND);
}

static inline uint16_t parse_cid(FAR struct altcom_cmdhdr_s *hdr)
{
  return hdr->cmdid;
}

static inline uint16_t parse_tid(FAR struct altcom_cmdhdr_s *hdr)
{
  return hdr->transid;
}

static inline uint16_t parse_cid4errind(FAR struct altcom_cmdhdr_s *hdr)
{
  FAR struct altcom_errind_s *pkt =
    (FAR struct altcom_errind_s *)&hdr->payload[0];

  return ntohs(pkt->cmdid);
}

static inline uint16_t parse_tid4errind(FAR struct altcom_cmdhdr_s *hdr)
{
  FAR struct altcom_errind_s *pkt =
    (FAR struct altcom_errind_s *)&hdr->payload[0];

  return ntohs(pkt->transid);
}

static inline FAR uint8_t *get_payload(FAR struct altcom_cmdhdr_s *hdr)
{
  return &hdr->payload[0];
}

static inline uint16_t get_payload_len(FAR struct altcom_cmdhdr_s *hdr)
{
  return hdr->datalen;
}

static inline FAR uint8_t get_altver(FAR struct altcom_cmdhdr_s *hdr)
{
  return hdr->ver;
}

static inline uint16_t get_pktlen(uint8_t ver, uint16_t payloadlen)
{
  if (ver == ALTCOM_VER1)
    {
      payloadlen += sizeof(struct altcom_cmdhdr_s) +
        sizeof(struct altcom_cmdfooter_s);
    }
  else if (ver == ALTCOM_VER4)
    {
      payloadlen += sizeof(struct altcom_cmdhdr_s);
    }
  else
    {
      DEBUGASSERT(0);
    }

  return payloadlen;
}

static inline uint16_t convert_cid2v1(uint16_t cid)
{
  if ((cid >= APICMDID_SOCK_ACCEPT) && (cid <= APICMDID_SOCK_SETSOCKOPT))
    {
      return cid;
    }

  if ((cid >= APICMDID_SMS_INIT) && (cid <= APICMDID_SMS_DELETE))
    {
      return cid;
    }

  if ((cid >= APICMDID_GET_VERSION_V4) && (cid <= APICMDID_SET_PSM_V4))
    {
      return (cid + 1);
    }

  if ((cid >= APICMDID_RADIO_ON_V4) && (cid <= APICMDID_ACTIVATE_PDN_V4))
    {
      return (cid - 0x2ec);
    }

  if ((cid >= APICMDID_DEACTIVATE_PDN_V4) &&
      (cid <= APICMDID_REPORT_NETINFO_V4))
    {
      return (cid - 0x2ed);
    }

  if (cid == (APICMDID_ERRINFO_V4 & ~ALTCOM_CMDID_REPLY_BIT))
    {
      return APICMDID_ERRINFO;
    }

  if (cid == APICMDID_GET_SIMINFO_V4)
    {
      return APICMDID_GET_SIMINFO;
    }

  if (cid == APICMDID_GET_QUALITY_V4)
    {
      return APICMDID_GET_QUALITY;
    }

  if (cid == APICMDID_ACTIVATE_PDN_CANCEL_V4)
    {
      return APICMDID_ACTIVATE_PDN_CANCEL;
    }

  if (cid == APICMDID_GET_CELLINFO_V4)
    {
      return APICMDID_GET_CELLINFO;
    }

  if (cid == APICMDID_GET_RAT_V4)
    {
      return APICMDID_GET_RAT;
    }

  if (cid == APICMDID_SET_RAT_V4)
    {
      return APICMDID_SET_RAT;
    }

  if (cid == APICMDID_SEND_ATCMD_V4)
    {
      return APICMDID_SEND_ATCMD;
    }

  if (cid == APICMDID_URC_EVENT_V4)
    {
      return APICMDID_URC_EVENT;
    }

  if ((cid >= APICMDID_FW_INJECTDELTAIMG_V4) &&
      (cid <= APICMDID_FW_GETUPDATERESULT_V4))
    {
      return (cid + 0xdd0);
    }

  if (cid == APICMDID_TLS_CONFIG_VERIFY_CALLBACK_V4)
    {
      return APICMDID_TLS_CONFIG_VERIFY_CALLBACK;
    }

  if (cid == APICMDID_CLOGS_V4)
    {
      return APICMDID_CLOGS;
    }

  if (cid == APICMDID_LOGLIST_V4)
    {
      return APICMDID_LOGLIST;
    }

#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
  if ((cid >= APICMDID_LOGOPEN_V4) && (cid <= APICMDID_LOGLSEEK_V4))
    {
      return cid + (APICMDID_LOGOPEN - APICMDID_LOGOPEN_V4);
    }
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

  return APICMDID_UNKNOWN;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_make_poweron_cmd_v1
 *
 * Description:
 *   Create an ALTCOM command for POWER_ON_REQ of protocol version 1.
 *
 * Input Parameters:
 *   sz         - Variable to put the length of the command created.
 *
 * Returned Value:
 *   Returns the buffer pointer of the created command.
 *
 ****************************************************************************/

FAR void *altcom_make_poweron_cmd_v1(FAR int *sz);

/****************************************************************************
 * Name: altcom_make_poweron_cmd_v4
 *
 * Description:
 *   Create an ALTCOM command for POWER_ON_REQ of protocol version 4.
 *
 * Input Parameters:
 *   sz         - Variable to put the length of the command created.
 *
 * Returned Value:
 *   Returns the buffer pointer of the created command.
 *
 ****************************************************************************/

FAR void *altcom_make_poweron_cmd_v4(FAR int *sz);

/****************************************************************************
 * Name: altcom_is_v1pkt_ok
 *
 * Description:
 *   Check if the POWER_ON_RES command of protocol version 1 is OK.
 *
 * Input Parameters:
 *   cmdhdr      - Pointer to the received packet.
 *
 * Returned Value:
 *   Returns true if the POWER_ON_RES command of protocol version 1 is OK,
 *   false otherwise.
 *
 ****************************************************************************/

bool altcom_is_v1pkt_ok(FAR struct altcom_cmdhdr_s *cmdhdr);

/****************************************************************************
 * Name: altcom_is_v4pkt_ok
 *
 * Description:
 *   Check if the POWER_ON_RES command of protocol version 4 is OK.
 *
 * Input Parameters:
 *   cmdhdr      - Pointer to the received packet.
 *
 * Returned Value:
 *   Returns true if the POWER_ON_RES command of protocol version 4 is OK,
 *   false otherwise.
 *
 ****************************************************************************/

bool altcom_is_v4pkt_ok(FAR struct altcom_cmdhdr_s *cmdhdr);

/****************************************************************************
 * Name: altcom_is_pkt_ok
 *
 * Description:
 *   Check the validity of the ALTCOM command received from ALT1250.
 *   In the case of a fragmented packet, returns the length of the remaining
 *   fragmented packets.
 *
 * Input Parameters:
 *   pkt      - Pointer to the received packet.
 *   sz       - Size of received packets.
 *
 * Returned Value:
 *   If the received packet is valid, 0 is returned. If the received packet
 *   is a fragmented packet, the length of the remaining fragmented packet is
 *   returned. If the received packet is not valid, a negative value is
 *   returned.
 *
 ****************************************************************************/

int altcom_is_pkt_ok(FAR uint8_t *pkt, int sz);

/****************************************************************************
 * Name: altcom_set_header_top
 *
 * Description:
 *   Set the version and command ID among the ALTCOM command headers.
 *
 * Input Parameters:
 *   hdr      - Pointer to ALTCOM command header.
 *   ver      - ALTCOM protocol version.
 *   cid      - ALTCOM command ID.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void altcom_set_header_top(FAR struct altcom_cmdhdr_s *hdr,
  uint8_t ver, uint16_t cid);

/****************************************************************************
 * Name: altcom_make_header
 *
 * Description:
 *   Create ALTCOM command header.
 *
 * Input Parameters:
 *   hdr      - Pointer to ALTCOM command header.
 *   ver      - ALTCOM protocol version.
 *   cid      - ALTCOM command ID.
 *   sz       - Body size of the ALTCOM command.
 *
 * Returned Value:
 *   Returns the transaction ID of the ALTCOM command.
 *
 ****************************************************************************/

uint16_t altcom_make_header(FAR struct altcom_cmdhdr_s *hdr,
  uint8_t ver, uint16_t cid, uint16_t sz);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_PKT_H */
