/****************************************************************************
 * drivers/modem/alt1250/altcom_pkt.c
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
#include <nuttx/modem/alt1250.h>
#include <assert.h>

#include "altcom_pkt.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct calculated_checksum
{
  uint16_t header_checksum;
  uint16_t body_checksum;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_poweron_cmd[sizeof(struct altcom_cmdhdr_s) +
  sizeof(struct altcom_cmdfooter_s)];
static uint8_t g_seqid;
static uint16_t g_transid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t calc_checksum_v1(FAR uint8_t *ptr, uint16_t len)
{
  uint32_t ret = 0x00;
  uint16_t calctmp = 0x00;
  uint16_t i;
  int is_odd = len & 0x01;

  for (i = 0; i < (len & 0xfffe); i += sizeof(uint16_t))
    {
      calctmp = *((uint16_t *)(ptr + i));
      ret += ntohs(calctmp);
    }

  if (is_odd)
    {
      ret += *(ptr + i) << 8;
    }

  ret = ~((ret & 0xffff) + (ret >> 16));

  return (uint16_t)ret;
}

static uint16_t calc_checksum_v4(FAR uint8_t *ptr, uint16_t len)
{
  uint32_t ret = 0x00;
  uint32_t calctmp = 0x00;
  uint16_t i;

  /* Data accumulating */

  for (i = 0; i < len; i++)
    {
      calctmp += ptr[i];
    }

  ret = ~((calctmp & 0xffff) + (calctmp >> 16));

  return (uint16_t)ret;
}

static inline void set_header_top(FAR struct altcom_cmdhdr_s *hdr,
  uint8_t ver, uint16_t cid)
{
  hdr->magic = htonl(ALTCOM_HDR_MAGICNUMBER);
  hdr->ver = ver;
  hdr->seqid = g_seqid++;
  hdr->cmdid = htons(cid);
  hdr->transid = htons(g_transid++);
  hdr->datalen = 0;
}

static struct calculated_checksum convert_ntoh(
  FAR struct altcom_cmdhdr_s *hdr)
{
  struct calculated_checksum checksum;

  hdr->cmdid = ntohs(hdr->cmdid);
  hdr->transid = ntohs(hdr->transid);
  hdr->datalen = ntohs(hdr->datalen);

  if (hdr->ver == ALTCOM_VER1)
    {
      /* Only V1 has footer */

      hdr->v1_options = ntohs(hdr->v1_options);
      hdr->v1_checksum = ntohs(hdr->v1_checksum);

      struct altcom_cmdfooter_s *footer
        = (struct altcom_cmdfooter_s *)&hdr->payload[hdr->datalen];

      checksum.body_checksum = calc_checksum_v1(&hdr->payload[0],
            sizeof(struct altcom_cmdfooter_s)-2 + hdr->datalen);

      footer->reserve = ntohs(footer->reserve);
      footer->checksum = ntohs(footer->checksum);
    }
  else
    {
      hdr->v4_hdr_cksum = ntohs(hdr->v4_hdr_cksum);

      checksum.body_checksum = calc_checksum_v4(&hdr->payload[0],
        hdr->datalen);

      hdr->v4_data_cksum = ntohs(hdr->v4_data_cksum);
    }

  return checksum;
}

static int check_valid_pkt(FAR struct altcom_cmdhdr_s *hdr,
  uint16_t body_checksum)
{
  int valid_version = ALTCOM_VERX;

  if (hdr->ver == ALTCOM_VER1)
    {
      struct altcom_cmdfooter_s *footer
        = (struct altcom_cmdfooter_s *)&hdr->payload[hdr->datalen];

      if (footer->checksum == body_checksum)
        {
          valid_version = ALTCOM_VER1;
        }
      else
        {
          m_err("[V1] cmdid: 0x%04x cmdlen: %d, "
            "body checksum: 0x%04x/0x%04x\n",
            hdr->cmdid,  hdr->datalen,
            footer->checksum, body_checksum);
        }
    }
  else if(hdr->ver == ALTCOM_VER4)
    {
      if (hdr->v4_data_cksum == body_checksum)
        {
          valid_version = ALTCOM_VER4;
        }
      else
        {
          m_err("[V4] cmdid: 0x%04x cmdlen: %d, "
            "body checksum: 0x%04x/0x%04x\n",
            hdr->cmdid,  hdr->datalen,
            hdr->v4_data_cksum, body_checksum);
        }
    }

  return valid_version;
}

static int check_valid_reply(FAR struct altcom_cmdhdr_s *hdr,
  uint16_t body_checksum)
{
  int valid_version = ALTCOM_VERX;

  valid_version = check_valid_pkt(hdr, body_checksum);
  if (valid_version == ALTCOM_VER1)
    {
      if ((hdr->cmdid == (ALTCOM_CMDID_POWER_ON_V1 | ALTCOM_CMDID_REPLY_BIT))
          && (hdr->datalen == ALTCOM_CMD_POWER_ON_REPLY_SIZE))
        {
          valid_version = ALTCOM_VER1;
        }
      else
        {
          valid_version = ALTCOM_VERX;
        }
    }
  else if(valid_version == ALTCOM_VER4)
    {
      if ((hdr->cmdid == (ALTCOM_CMDID_POWER_ON_V4 | ALTCOM_CMDID_REPLY_BIT))
          && (hdr->datalen == ALTCOM_CMD_POWER_ON_REPLY_SIZE))
        {
          valid_version = ALTCOM_VER4;
        }
      else
        {
          valid_version = ALTCOM_VERX;
        }
    }

  return valid_version;
}

static bool is_header_ok(FAR struct altcom_cmdhdr_s *hdr)
{
  uint16_t checksum;

  if (hdr->ver == ALTCOM_VER1)
    {
      checksum = calc_checksum_v1((uint8_t *)hdr,
        sizeof(struct altcom_cmdhdr_s) - sizeof(hdr->v1_checksum));
      if (ntohs(hdr->v1_checksum) == checksum)
        {
          if (ntohs(hdr->datalen) + sizeof(struct altcom_cmdhdr_s)
              + sizeof(struct altcom_cmdfooter_s) <= ALTCOM_RX_PKT_SIZE_MAX)
            {
              return true;
            }
          else
            {
              m_err("[V1] Data length exceeding the buffer length: %d\n",
                    ntohs(hdr->datalen));
            }
        }
      else
        {
          m_err("[V1] cmdid: 0x%04x cmdlen: %d, "
               "header checksum: 0x%04x/0x%04x\n",
                ntohs(hdr->cmdid),  ntohs(hdr->datalen),
                ntohs(hdr->v1_checksum), checksum);
        }
    }
  else if (hdr->ver == ALTCOM_VER4)
    {
      checksum = calc_checksum_v4((uint8_t *)hdr,
        sizeof(struct altcom_cmdhdr_s) - sizeof(hdr->v4_hdr_cksum) -
        sizeof(hdr->v4_data_cksum));
      if (ntohs(hdr->v4_hdr_cksum) == checksum)
        {
          if (ntohs(hdr->datalen) + sizeof(struct altcom_cmdhdr_s) <=
              ALTCOM_RX_PKT_SIZE_MAX)
            {
              return true;
            }
          else
            {
              m_err("[V4] Data length exceeding the buffer length: %d\n",
                    ntohs(hdr->datalen));
            }
        }
      else
        {
          m_err("[V4] cmdid: 0x%04x cmdlen: %d, "
               "header checksum: 0x%04x/0x%04x\n",
                ntohs(hdr->cmdid),  ntohs(hdr->datalen),
                ntohs(hdr->v4_hdr_cksum), checksum);
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool altcom_is_v1pkt_ok(struct altcom_cmdhdr_s *cmdhdr)
{
  struct calculated_checksum checksum;

  if (!is_header_ok(cmdhdr))
    {
      return false;
    }

  checksum = convert_ntoh(cmdhdr);
  return (check_valid_reply(cmdhdr, checksum.body_checksum) == ALTCOM_VER1)
    && (cmdhdr->payload[0] == LTE_RESULT_OK);
}

bool altcom_is_v4pkt_ok(struct altcom_cmdhdr_s *cmdhdr)
{
  struct calculated_checksum checksum;

  if (!is_header_ok(cmdhdr))
    {
      return false;
    }

  checksum = convert_ntoh(cmdhdr);
  return (check_valid_reply(cmdhdr, checksum.body_checksum) == ALTCOM_VER4)
    && (cmdhdr->payload[0] == LTE_RESULT_OK);
}

FAR void *altcom_make_poweron_cmd_v1(int *sz)
{
  struct altcom_cmdhdr_s *hdr = (struct altcom_cmdhdr_s *)g_poweron_cmd;
  struct altcom_cmdfooter_s *footer
    = (struct altcom_cmdfooter_s *)&hdr->payload[0];

  set_header_top(hdr, ALTCOM_VER1, ALTCOM_CMDID_POWER_ON_V1);
  hdr->v1_options = htons(ALTCOM_CMDOPT_CHECKSUM_EN);
  hdr->v1_checksum =
    htons(calc_checksum_v1((uint8_t *)hdr,
          sizeof(struct altcom_cmdhdr_s) - 2));

  footer->reserve = 0;
  footer->checksum =
    htons(calc_checksum_v1(&hdr->payload[0],
          sizeof(struct altcom_cmdfooter_s) - 2));

  /* No payload of this altcom command.
   * So sending size is just header and footer size
   */

  *sz = sizeof(struct altcom_cmdhdr_s) + sizeof(struct altcom_cmdfooter_s);

  return g_poweron_cmd;
}

FAR void *altcom_make_poweron_cmd_v4(int *sz)
{
  struct altcom_cmdhdr_s *hdr = (struct altcom_cmdhdr_s *)g_poweron_cmd;

  set_header_top(hdr, ALTCOM_VER4, ALTCOM_CMDID_POWER_ON_V4);
  hdr->v4_hdr_cksum = htons(calc_checksum_v4((uint8_t *)hdr,
        sizeof(struct altcom_cmdhdr_s)-4));
  hdr->v4_data_cksum = htons(calc_checksum_v4(&hdr->payload[0], 0));

  /* No payload of this altcom command. So sending size is just header size */

  *sz = sizeof(struct altcom_cmdhdr_s);

  return g_poweron_cmd;
}

int altcom_is_pkt_ok(FAR uint8_t *pkt, int sz)
{
  struct calculated_checksum checksum;
  int ver;
  int ret = OK;
  FAR struct altcom_cmdhdr_s *hdr = (FAR struct altcom_cmdhdr_s *)pkt;
  int remlen;

  if (!is_header_ok(hdr))
    {
      return -EPROTO;
    }

  remlen = get_pktlen(hdr->ver, ntohs(hdr->datalen)) - sz;
  if (remlen > 0)
    {
      /* Cases in which fragmented packets are received. */

      return remlen;
    }
  else if (remlen < 0)
    {
      /* The case where the received data length becomes
       * larger than the payload length set in the header.
       */

      return -EPROTO;
    }

  /* Whole packets are received. So check the validity of the packets. */

  checksum = convert_ntoh((FAR struct altcom_cmdhdr_s *)pkt);

  ver = check_valid_pkt((FAR struct altcom_cmdhdr_s *)pkt,
                        checksum.body_checksum);
  if (ver == ALTCOM_VERX)
    {
      ret = -EPROTO;
    }

  return ret;
}

uint16_t altcom_make_header(FAR struct altcom_cmdhdr_s *hdr,
  uint8_t ver, uint16_t cid, uint16_t sz)
{
  uint16_t tid = g_transid;

  hdr->magic = htonl(ALTCOM_HDR_MAGICNUMBER);
  hdr->ver = ver;
  hdr->seqid = g_seqid++;
  hdr->cmdid = htons(cid);
  hdr->transid = htons(g_transid++);
  hdr->datalen = htons(sz);

  if (ver == ALTCOM_VER1)
    {
      struct altcom_cmdfooter_s *footer
        = (struct altcom_cmdfooter_s *)&hdr->payload[sz];

      hdr->v1_options = htons(ALTCOM_CMDOPT_CHECKSUM_EN);
      hdr->v1_checksum =
        htons(calc_checksum_v1((uint8_t *)hdr,
          sizeof(struct altcom_cmdhdr_s) - 2));

      footer->reserve = 0;
      footer->checksum =
        htons(calc_checksum_v1(&hdr->payload[0],
          sizeof(struct altcom_cmdfooter_s) - 2 + sz));
    }
  else if (ver == ALTCOM_VER4)
    {
      hdr->v4_hdr_cksum = htons(calc_checksum_v4((uint8_t *)hdr,
        sizeof(struct altcom_cmdhdr_s)-4));
      hdr->v4_data_cksum = htons(calc_checksum_v4(&hdr->payload[0], sz));
    }
  else
    {
      DEBUGASSERT(0);
    }

  return tid;
}
