/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_radio.c
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
 * Public Functions
 ****************************************************************************/

int32_t altcom_radioon_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_RADIO_ON;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_RADIO_ON_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_radiooff_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_RADIO_OFF;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_RADIO_OFF_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_radioon_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_radioonres_s *in =
    (FAR struct apicmd_cmddat_radioonres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

int32_t altcom_radiooff_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_radiooffres_s *in =
    (FAR struct apicmd_cmddat_radiooffres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}
