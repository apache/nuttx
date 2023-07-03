/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_firmware.c
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
#include "altcom_errno.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void getver_parse_response(FAR struct apicmd_cmddat_getverres_s *resp,
                                  FAR lte_version_t *version)
{
  memset(version, 0, sizeof(*version));
  strncpy(version->bb_product,
          (FAR const char *)resp->bb_product, LTE_VER_BB_PRODUCT_LEN - 1);
  strncpy(version->np_package,
          (FAR const char *)resp->np_package, LTE_VER_NP_PACKAGE_LEN - 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getver_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_VERSION;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_VERSION_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_injectimage_pkt_compose(FAR void **arg, size_t arglen,
                                       uint8_t altver, FAR uint8_t *pktbuf,
                                       const size_t pktsz,
                                       FAR uint16_t *altcid)
{
  int32_t size = 0;

  FAR uint8_t *sending_data = (uint8_t *)arg[0];
  int len = *(int *)arg[1];
  bool mode = *(bool *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_fw_injectdeltaimg_s *out =
       (FAR struct apicmd_cmddat_fw_injectdeltaimg_s *)pktbuf;

      *altcid = APICMDID_FW_INJECTDELTAIMG;

      len = (len > APICMD_FW_INJECTDATA_MAXLEN) ?
        APICMD_FW_INJECTDATA_MAXLEN : len;

      out->data_len = htonl(len);
      out->inject_mode = mode ? LTEFW_INJECTION_MODE_NEW
        : LTEFW_INJECTION_MODE_APPEND;
      memcpy(out->data, sending_data, len);
      size = sizeof(struct apicmd_cmddat_fw_injectdeltaimg_s);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_fw_injectdeltaimg_v4_s *out =
       (FAR struct apicmd_cmddat_fw_injectdeltaimg_v4_s *)pktbuf;

      *altcid = APICMDID_FW_INJECTDELTAIMG_V4;
      len = (len > APICMD_FW_INJECTDATA_MAXLEN_V4) ?
        APICMD_FW_INJECTDATA_MAXLEN_V4 : len;

      out->data_len = htonl(len);
      out->inject_mode = mode ? LTEFW_INJECTION_MODE_NEW
        : LTEFW_INJECTION_MODE_APPEND;
      memcpy(out->data, sending_data, len);
      size = sizeof(struct apicmd_cmddat_fw_injectdeltaimg_v4_s) - 1 + len;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getimagelen_pkt_compose(FAR void **arg, size_t arglen,
                                       uint8_t altver, FAR uint8_t *pktbuf,
                                       const size_t pktsz,
                                       FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_GETDELTAIMGLEN;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_GETDELTAIMGLEN_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_execupdate_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_EXECDELTAUPDATE;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_EXECDELTAUPDATE_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getupdateres_pkt_compose(FAR void **arg, size_t arglen,
                                        uint8_t altver, FAR uint8_t *pktbuf,
                                        const size_t pktsz,
                                        FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_GETUPDATERESULT;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_GETUPDATERESULT_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getver_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_version_t *version = (FAR lte_version_t *)arg[1];
  FAR struct apicmd_cmddat_getverres_s *in =
    (FAR struct apicmd_cmddat_getverres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (*ret == 0)
    {
      /* Parse version information */

      getver_parse_response(in, version);
    }

  return 0;
}

int32_t altcom_fwcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  int32_t result_cmd;
  int16_t injection_retcode;

  FAR struct apicmd_cmddat_fw_deltaupcommres_s *in =
    (FAR struct apicmd_cmddat_fw_deltaupcommres_s *)pktbuf;

  /* Negative value in result_cmd means an error is occured.
   * Zero indicates command successed or size of injected data
   */

  result_cmd = altcom_geterrcode(in->api_result);
  injection_retcode = ntohs(in->ltefw_result);

  return (injection_retcode != 0) ? -injection_retcode : result_cmd;
}
