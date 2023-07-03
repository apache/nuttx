/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_pin.c
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
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_PINSET;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_PINSET_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setpinlock_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR bool *enable = (FAR bool *)arg[0];
  FAR char *pincode = (FAR char *)arg[1];

  FAR struct apicmd_cmddat_setpinlock_s *out =
    (FAR struct apicmd_cmddat_setpinlock_s *)pktbuf;

  out->mode = *enable;
  strncpy((FAR char *)out->pincode, pincode, sizeof(out->pincode));

  size = sizeof(struct apicmd_cmddat_setpinlock_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PIN_LOCK;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PIN_LOCK_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setpincode_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int8_t *target_pin = (FAR int8_t *)arg[0];
  FAR char *pincode = (FAR char *)arg[1];
  FAR char *new_pincode = (FAR char *)arg[2];

  FAR struct apicmd_cmddat_setpincode_s *out =
    (FAR struct apicmd_cmddat_setpincode_s *)pktbuf;

  if (LTE_TARGET_PIN == *target_pin)
    {
      out->chgtype = APICMD_SETPINCODE_CHGTYPE_PIN;
    }
  else
    {
      out->chgtype = APICMD_SETPINCODE_CHGTYPE_PIN2;
    }

  strncpy((FAR char *)out->pincode, pincode, sizeof(out->pincode));

  strncpy((FAR char *)out->newpincode, new_pincode, sizeof(out->newpincode));

  size = sizeof(struct apicmd_cmddat_setpincode_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PIN_CODE;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PIN_CODE_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_enterpin_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR char *pincode = (FAR char *)arg[0];
  FAR char *new_pincode = (FAR char *)arg[1];

  FAR struct apicmd_cmddat_enterpin_s *out =
    (FAR struct apicmd_cmddat_enterpin_s *)pktbuf;

  strncpy((FAR char *)out->pincode, pincode, sizeof(out->pincode));
  if (new_pincode)
    {
      out->newpincodeuse = APICMD_ENTERPIN_NEWPINCODE_USE;
      strncpy((FAR char *)out->newpincode,
              new_pincode, sizeof(out->newpincode));
    }
  else
    {
      out->newpincodeuse = APICMD_ENTERPIN_NEWPINCODE_UNUSE;
    }

  size = sizeof(struct apicmd_cmddat_enterpin_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ENTER_PIN;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ENTER_PIN_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
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
