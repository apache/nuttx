/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_sim.c
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
#include <nuttx/modem/alt1250.h>
#include <nuttx/wireless/lte/lte.h>

#include "altcom_cmd.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void getsiminfo_parse_response(
  FAR struct apicmd_cmddat_getsiminfo_res_s *resp,
  FAR lte_siminfo_t *siminfo)
{
  siminfo->option = ntohl(resp->option);

  if (siminfo->option & LTE_SIMINFO_GETOPT_MCCMNC)
    {
      if (LTE_MNC_DIGIT_MAX < resp->mnc_digit)
        {
          resp->mnc_digit = LTE_MNC_DIGIT_MAX;
        }

      memcpy(siminfo->mcc, resp->mcc, LTE_MCC_DIGIT);
      siminfo->mnc_digit = resp->mnc_digit;
      memcpy(siminfo->mnc, resp->mnc, resp->mnc_digit);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_SPN)
    {
      if (LTE_SIMINFO_SPN_LEN < resp->spn_len)
        {
          resp->spn_len = LTE_SIMINFO_SPN_LEN;
        }

      siminfo->spn_len = resp->spn_len;
      memcpy(siminfo->spn, resp->spn, resp->spn_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_ICCID)
    {
      if (LTE_SIMINFO_ICCID_LEN < resp->iccid_len)
        {
          resp->iccid_len = LTE_SIMINFO_ICCID_LEN;
        }

      siminfo->iccid_len = resp->iccid_len;
      memcpy(siminfo->iccid, resp->iccid, resp->iccid_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_IMSI)
    {
      if (LTE_SIMINFO_IMSI_LEN < resp->imsi_len)
        {
          resp->imsi_len = LTE_SIMINFO_IMSI_LEN;
        }

      siminfo->imsi_len = resp->imsi_len;
      memcpy(siminfo->imsi, resp->imsi, resp->imsi_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID1)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid1_len)
        {
          resp->gid1_len = LTE_SIMINFO_GID_LEN;
        }

      siminfo->gid1_len = resp->gid1_len;
      memcpy(siminfo->gid1, resp->gid1, resp->gid1_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID2)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid2_len)
        {
          resp->gid2_len = LTE_SIMINFO_GID_LEN;
        }

      siminfo->gid2_len = resp->gid2_len;
      memcpy(siminfo->gid2, resp->gid2, resp->gid2_len);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getphone_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_PHONENO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_PHONENO_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getimsi_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMSI;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMSI_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getsiminfo_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR uint32_t *option = (FAR uint32_t *)arg[0];

  FAR struct apicmd_cmddat_getsiminfo_s *out =
    (FAR struct apicmd_cmddat_getsiminfo_s *)pktbuf;

  out->option = htonl(*option);
  size = sizeof(struct apicmd_cmddat_getsiminfo_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_SIMINFO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_SIMINFO_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getphone_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *errcause = (FAR uint8_t *)arg[1];
  FAR char *phoneno = (FAR char *)arg[2];
  FAR struct apicmd_cmddat_phonenores_s *in =
    (FAR struct apicmd_cmddat_phonenores_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  *errcause = in->errcause;
  if (0 == *ret)
    {
      if (arglen > 3)
        {
          FAR size_t *len = (FAR size_t *)arg[3];

          /* Is it enough length to include Null terminate?
           * The length of LTE_PHONENO_LEN includes the
           * null terminator.
           */

          if (*len < strnlen((FAR const char *)in->phoneno,
            LTE_PHONENO_LEN))
            {
              return -ENOBUFS;
            }
        }

      strncpy(phoneno, (FAR const char *)in->phoneno, LTE_PHONENO_LEN);
    }

  return 0;
}

int32_t altcom_getimsi_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *errcause = (FAR uint8_t *)arg[1];
  FAR char *imsi = (FAR char *)arg[2];
  FAR struct apicmd_cmddat_getimsires_s *in =
    (FAR struct apicmd_cmddat_getimsires_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  *errcause = in->errcause;
  if (0 == *ret)
    {
      if (arglen > 3)
        {
          FAR size_t *len = (FAR size_t *)arg[3];

          /* Is it enough length to include Null terminate?
           * The length of LTE_IMSI_LEN includes the null terminator.
           */

          if (*len < strnlen((FAR const char *)in->imsi, LTE_IMSI_LEN))
            {
              return -ENOBUFS;
            }
        }

      strncpy(imsi, (FAR const char *)in->imsi, LTE_IMSI_LEN);
    }

  return 0;
}

int32_t altcom_getsiminfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_siminfo_t *siminfo = (FAR lte_siminfo_t *)arg[1];
  FAR struct apicmd_cmddat_getsiminfo_res_s *in =
    (FAR struct apicmd_cmddat_getsiminfo_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  if (0 == *ret)
    {
      /* Parse SIM information */

      getsiminfo_parse_response(in, siminfo);
    }

  return 0;
}
