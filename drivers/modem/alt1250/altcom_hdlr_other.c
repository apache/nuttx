/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_other.c
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

#include "alt1250.h"
#include "altcom_cmd.h"
#include "altcom_lwm2m_hdlr.h"
#include "altcom_errno.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATCMD_HEADER     "AT"
#define ATCMD_HEADER_LEN (2)
#define ATCMD_FOOTER     '\r'
#define ATCMD_FOOTER_LEN (1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getimei_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMEI;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMEI_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_sendatcmd_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const char *cmd = (FAR const char *)arg[0];
  int cmdlen = (int)arg[1];

  size = cmdlen - ATCMD_HEADER_LEN;
  memcpy(pktbuf, cmd + ATCMD_HEADER_LEN, size);
  pktbuf[size - ATCMD_FOOTER_LEN] = '\0';

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      size = -ENOTSUP;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SEND_ATCMD_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_errinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR lte_errinfo_t *info = (FAR lte_errinfo_t *)arg[0];
  FAR struct apicmd_cmddat_errinfo_s *in =
    (FAR struct apicmd_cmddat_errinfo_s *)pktbuf;

  info->err_indicator = in->indicator;
  info->err_result_code = ntohl(in->err_code);
  info->err_no = altcom_errno2nuttx(ntohl(in->err_no));
  memcpy(info->err_string, in->err_str, LTE_ERROR_STRING_MAX_LEN);
  info->err_string[LTE_ERROR_STRING_MAX_LEN - 1] = '\0';

  return 0;
}

int32_t altcom_getimei_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR char *imei = (FAR char *)arg[1];
  FAR struct apicmd_cmddat_getimeires_s *in =
    (FAR struct apicmd_cmddat_getimeires_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      if (arglen > 2)
        {
          FAR size_t *len = (FAR size_t *)arg[2];

          /* Is it enough length to include Null terminate?
           * The length of LTE_IMEI_LEN includes the
           * null terminator.
           */

          if (*len < strnlen((FAR const char *)in->imei, LTE_IMEI_LEN))
            {
              return -ENOBUFS;
            }
        }

      strncpy(imei, (FAR const char *)in->imei, LTE_IMEI_LEN);
    }

  return 0;
}

int32_t altcom_sendatcmd_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR char *respbuff = (FAR char *)arg[0];
  FAR int respbufflen = (int)arg[1];
  FAR int *resplen = (FAR int *)arg[2];

  if (respbufflen < pktsz)
    {
      return -ENOBUFS;
    }

  memcpy(respbuff, (FAR char *)pktbuf, pktsz);
  *resplen = pktsz;

  return 0;
}

int32_t altcom_urc_event_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = -ENOTSUP;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  uint32_t lcmdid = 0;
  alt_evtbuf_inst_t *inst;
  lwm2mstub_hndl_t lwm2m_urc_handler;
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      pktbuf[pktsz] = '\0';
      m_info("===== URC =====\n%s"
             "===============\n", pktbuf);
      lwm2m_urc_handler = lwm2mstub_get_handler(&pktbuf, &pktsz, &lcmdid);
      *bitmap = get_event_lapibuffer(dev, lcmdid, &inst);

      if (*bitmap != 0ULL)
        {
          ret = lwm2m_urc_handler(pktbuf, pktsz,
                                       inst->outparam, inst->outparamlen);
        }
    }
#endif

  return ret;
}
