/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_pin.c
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

static void getpinset_parse_response(
  FAR struct apicmd_cmddat_getpinsetres_s *resp,
  FAR lte_getpin_t *pinset)
{
  pinset->enable            = resp->active;
  pinset->status            = resp->status;
  pinset->pin_attemptsleft  = resp->pin_attemptsleft;
  pinset->puk_attemptsleft  = resp->puk_attemptsleft;
  pinset->pin2_attemptsleft = resp->pin2_attemptsleft;
  pinset->puk2_attemptsleft = resp->puk2_attemptsleft;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getpinset_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
{
  return -ENOTSUP;
}

int32_t altcom_setpinlock_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  return -ENOTSUP;
}

int32_t altcom_setpincode_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  return -ENOTSUP;
}

int32_t altcom_enterpin_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  return -ENOTSUP;
}

int32_t altcom_getpinset_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_getpin_t *pinset = (FAR lte_getpin_t *)arg[1];
  FAR struct apicmd_cmddat_getpinsetres_s *in =
    (FAR struct apicmd_cmddat_getpinsetres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Parse PIN settings */

      getpinset_parse_response(in, pinset);
    }

  return 0;
}

int32_t altcom_setpinlock_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *attemptsleft = (FAR uint8_t *)arg[1];
  FAR struct apicmd_cmddat_setpinlockres_s *in =
    (FAR struct apicmd_cmddat_setpinlockres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (*ret != 0)
    {
      *attemptsleft = in->attemptsleft;
    }

  return 0;
}

int32_t altcom_setpincode_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *attemptsleft = (FAR uint8_t *)arg[1];
  FAR struct apicmd_cmddat_setpincoderes_s *in =
    (FAR struct apicmd_cmddat_setpincoderes_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (*ret != 0)
    {
      *attemptsleft = in->attemptsleft;
    }

  return 0;
}

int32_t altcom_enterpin_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *simstat = (FAR uint8_t *)arg[1];
  FAR uint8_t *attemptsleft = (FAR uint8_t *)arg[2];
  FAR struct apicmd_cmddat_enterpinres_s *in =
    (FAR struct apicmd_cmddat_enterpinres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  *simstat      = in->simstat;
  *attemptsleft = in->attemptsleft;

  return 0;
}
