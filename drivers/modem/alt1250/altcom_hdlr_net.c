/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_net.c
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
#include <stdlib.h>
#include <errno.h>
#include <arpa/inet.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/wireless/lte/lte.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include "altcom_cmd.h"
#include "alt1250.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOMBS_BASE_HEX 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
static uint16_t g_set_repevt = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool altcombs_check_arrydigitnum(FAR uint8_t number[], uint8_t digit)
{
  uint8_t cnt;

  for (cnt = 0; cnt < digit; cnt++)
    {
      if (number[cnt] < APICMD_CELLINFO_DIGIT_NUM_MIN ||
        APICMD_CELLINFO_DIGIT_NUM_MAX < number[cnt])
        {
          return false;
        }
    }

  return true;
}

static int32_t altcombs_set_quality(FAR lte_quality_t *data,
  FAR struct apicmd_cmddat_quality_s *cmd_quality)
{
  data->valid = APICMD_QUALITY_ENABLE == cmd_quality->enability ?
                         LTE_VALID : LTE_INVALID;
  if (data->valid)
    {
      data->rsrp  = ntohs(cmd_quality->rsrp);
      data->rsrq  = ntohs(cmd_quality->rsrq);
      data->sinr  = ntohs(cmd_quality->sinr);
      data->rssi  = ntohs(cmd_quality->rssi);
      if (data->rsrp < APICMD_QUALITY_RSRP_MIN ||
        APICMD_QUALITY_RSRP_MAX < data->rsrp)
        {
          m_err("data.rsrp error:%d\n", data->rsrp);
          data->valid = LTE_INVALID;
        }
      else if (data->rsrq < APICMD_QUALITY_RSRQ_MIN ||
          APICMD_QUALITY_RSRQ_MAX < data->rsrq)
        {
          m_err("data.rsrq error:%d\n", data->rsrq);
          data->valid = LTE_INVALID;
        }
      else if (data->sinr < APICMD_QUALITY_SINR_MIN ||
          APICMD_QUALITY_SINR_MAX < data->sinr)
        {
          m_err("data->sinr error:%d\n", data->sinr);
          data->valid = LTE_INVALID;
        }
      else
        {
          /* Do nothing. */
        }
    }

  return 0;
}

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
static void altcombs_set_cellinfo(
  FAR struct apicmd_cmddat_cellinfo_s *cmd_cellinfo,
  FAR lte_cellinfo_t *api_cellinfo)
{
  if (cmd_cellinfo->valid == LTE_VALID)
    {
      if (ntohl(cmd_cellinfo->cell_id) > APICMD_CELLINFO_CELLID_MAX)
        {
          m_err("cmd_cellinfo->cell_id error:%ld\n",
                           ntohl(cmd_cellinfo->cell_id));
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (ntohl(cmd_cellinfo->earfcn) > APICMD_CELLINFO_EARFCN_MAX)
        {
          m_err("cmd_cellinfo->earfcn error:%ld\n",
                           ntohl(cmd_cellinfo->earfcn));
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mcc,
        LTE_MCC_DIGIT))
        {
          m_err("cmd_cellinfo->mcc error\n");
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (
        cmd_cellinfo->mnc_digit < APICMD_CELLINFO_MNC_DIGIT_MIN ||
        cmd_cellinfo->mnc_digit > LTE_MNC_DIGIT_MAX)
        {
          m_err("cmd_cellinfo->mnc_digit error:%d\n",
                           cmd_cellinfo->mnc_digit);
          api_cellinfo->valid = LTE_INVALID;
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mnc,
               cmd_cellinfo->mnc_digit))
        {
          m_err("cmd_cellinfo->mnc error\n");
          api_cellinfo->valid = LTE_INVALID;
        }
      else
        {
          api_cellinfo->valid = LTE_VALID;
          api_cellinfo->phycell_id = ntohl(cmd_cellinfo->cell_id);
          api_cellinfo->earfcn     = ntohl(cmd_cellinfo->earfcn);
          memcpy(api_cellinfo->mcc, cmd_cellinfo->mcc, LTE_MCC_DIGIT);
          api_cellinfo->mnc_digit  = cmd_cellinfo->mnc_digit;
          memcpy(api_cellinfo->mnc, cmd_cellinfo->mnc,
                 cmd_cellinfo->mnc_digit);
        }
    }
  else
    {
      api_cellinfo->valid = LTE_INVALID;
    }

  api_cellinfo->option = 0;
}
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
static void altcombs_set_cellinfo_v4(
  FAR struct apicmd_cmddat_cellinfo_v4_s *cmd_cellinfo,
  FAR lte_cellinfo_t *api_cellinfo)
{
  int i;

  api_cellinfo->valid = LTE_INVALID;
  api_cellinfo->option = 0;

  if (cmd_cellinfo->enability != LTE_INVALID)
    {
      if (ntohl(cmd_cellinfo->cell_id) > APICMD_CELLINFO_CELLID_MAX)
        {
          m_err("cmd_cellinfo->cell_id error:%ld\n",
                           ntohl(cmd_cellinfo->cell_id));
        }
      else if (ntohl(cmd_cellinfo->earfcn) > APICMD_CELLINFO_EARFCN_MAX)
        {
          m_err("cmd_cellinfo->earfcn error:%ld\n",
                           ntohl(cmd_cellinfo->earfcn));
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mcc,
        LTE_MCC_DIGIT))
        {
          m_err("cmd_cellinfo->mcc error\n");
        }
      else if (
        cmd_cellinfo->mnc_digit < APICMD_CELLINFO_MNC_DIGIT_MIN ||
        cmd_cellinfo->mnc_digit > LTE_MNC_DIGIT_MAX)
        {
          m_err("cmd_cellinfo->mnc_digit error:%d\n",
                           cmd_cellinfo->mnc_digit);
        }
      else if (!altcombs_check_arrydigitnum(cmd_cellinfo->mnc,
               cmd_cellinfo->mnc_digit))
        {
          m_err("cmd_cellinfo->mnc error\n");
        }
      else if (strlen((const char *)cmd_cellinfo->cgid) >
       APICMD_CELLINFO_GCID_MAX)
        {
          m_err("cmd_cellinfo->cgid error\n");
        }
      else
        {
          api_cellinfo->valid = LTE_VALID;
          api_cellinfo->phycell_id = ntohl(cmd_cellinfo->cell_id);
          api_cellinfo->earfcn     = ntohl(cmd_cellinfo->earfcn);
          memcpy(api_cellinfo->mcc, cmd_cellinfo->mcc, LTE_MCC_DIGIT);
          api_cellinfo->mnc_digit  = cmd_cellinfo->mnc_digit;
          memcpy(api_cellinfo->mnc, cmd_cellinfo->mnc,
                 cmd_cellinfo->mnc_digit);

          api_cellinfo->option |= LTE_CELLINFO_OPT_GCID;
          api_cellinfo->gcid = strtoul((FAR const char *)cmd_cellinfo->cgid,
                                       NULL, ALTCOMBS_BASE_HEX);

          api_cellinfo->option |= LTE_CELLINFO_OPT_AREACODE;
          api_cellinfo->area_code = ntohs(cmd_cellinfo->tac);

          if (cmd_cellinfo->enability & APICMD_CELLINFO_VALID_SFN)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_SFN;
              api_cellinfo->sfn = ntohs(cmd_cellinfo->sfn);
            }

          if (cmd_cellinfo->enability & APICMD_CELLINFO_VALID_RSRP)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_RSRP;
              api_cellinfo->rsrp = ntohs(cmd_cellinfo->rsrp);
            }

          if (cmd_cellinfo->enability & APICMD_CELLINFO_VALID_RSRQ)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_RSRQ;
              api_cellinfo->rsrq = ntohs(cmd_cellinfo->rsrq);
            }

          if (cmd_cellinfo->enability & APICMD_CELLINFO_VALID_TIMEDIFFIDX)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_TIMEDIFFIDX;
              api_cellinfo->time_diffidx = ntohs(cmd_cellinfo->time_diffidx);
            }

          if (cmd_cellinfo->enability & APICMD_CELLINFO_VALID_TA)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_TA;
              api_cellinfo->ta = ntohs(cmd_cellinfo->ta);
            }

          if (api_cellinfo->nr_neighbor > cmd_cellinfo->neighbor_num)
            {
              api_cellinfo->nr_neighbor = cmd_cellinfo->neighbor_num;
            }

          for (i = 0; i < api_cellinfo->nr_neighbor; i++)
            {
              api_cellinfo->option |= LTE_CELLINFO_OPT_NEIGHBOR;
              api_cellinfo->neighbors[i].option = 0;
              api_cellinfo->neighbors[i].phycell_id =
                ntohl(cmd_cellinfo->neighbor_cell[i].cell_id);
              api_cellinfo->neighbors[i].earfcn =
                ntohl(cmd_cellinfo->neighbor_cell[i].earfcn);
              if (cmd_cellinfo->neighbor_cell[i].valid &
                  APICMD_CELLINFO_VALID_SFN)
                {
                  api_cellinfo->neighbors[i].option |= LTE_CELLINFO_OPT_SFN;
                  api_cellinfo->neighbors[i].sfn =
                    ntohs(cmd_cellinfo->neighbor_cell[i].sfn);
                }

              if (cmd_cellinfo->neighbor_cell[i].valid &
                  APICMD_CELLINFO_VALID_RSRP)
                {
                  api_cellinfo->neighbors[i].option |= LTE_CELLINFO_OPT_RSRP;
                  api_cellinfo->neighbors[i].rsrp =
                    ntohs(cmd_cellinfo->neighbor_cell[i].rsrp);
                }

              if (cmd_cellinfo->neighbor_cell[i].valid &
                  APICMD_CELLINFO_VALID_RSRQ)
                {
                  api_cellinfo->neighbors[i].option |= LTE_CELLINFO_OPT_RSRQ;
                  api_cellinfo->neighbors[i].rsrq =
                    ntohs(cmd_cellinfo->neighbor_cell[i].rsrq);
                }
            }
        }
    }
}
#endif

static void getltime_parse_response(
  FAR struct apicmd_cmddat_getltimeres_s *resp,
  FAR lte_localtime_t *localtime)
{
  localtime->year   = resp->ltime.year;
  localtime->mon    = resp->ltime.month;
  localtime->mday   = resp->ltime.day;
  localtime->hour   = resp->ltime.hour;
  localtime->min    = resp->ltime.minutes;
  localtime->sec    = resp->ltime.seconds;
  localtime->tz_sec = ntohl(resp->ltime.timezone);
}

static void parse_ltime(FAR struct apicmd_cmddat_ltime_s *from,
                        FAR lte_localtime_t *to)
{
  to->year = from->year;
  to->mon = from->month;
  to->mday = from->day;
  to->hour = from->hour;
  to->min = from->minutes;
  to->sec = from->seconds;
  to->tz_sec = ntohl(from->timezone);
}

static int parse_simd(FAR struct apicmd_cmddat_repevt_simd_s *simd,
                      FAR uint32_t *simstat)
{
  int ret = 0;

  switch (simd->status)
    {
      case APICMD_REPORT_EVT_SIMD_REMOVAL:
        {
          *simstat = LTE_SIMSTAT_REMOVAL;
        }
        break;

      case APICMD_REPORT_EVT_SIMD_INSERTION:
        {
          *simstat = LTE_SIMSTAT_INSERTION;
        }
        break;

      default:
        {
          m_err("Unsupport SIMD status. status:%d\n", simd->status);
          return -EILSEQ;
        }
        break;
    }

  return ret;
}

static int parse_simstate(
  FAR struct apicmd_cmddat_repevt_simstate_s *simstate,
  FAR uint32_t *simstat)
{
  int ret = 0;

  switch (simstate->state)
    {
      case APICMD_REPORT_EVT_SIMSTATE_SIM_INIT_WAIT_PIN_UNLOCK:
        {
          *simstat = LTE_SIMSTAT_WAIT_PIN_UNLOCK;
        }
        break;

      case APICMD_REPORT_EVT_SIMSTATE_PERSONALIZATION_FAILED:
        {
          *simstat = LTE_SIMSTAT_PERSONAL_FAILED;
        }
        break;

      case APICMD_REPORT_EVT_SIMSTATE_ACTIVATION_COMPLETED:
        {
          *simstat = LTE_SIMSTAT_ACTIVATE;
        }
        break;

      case APICMD_REPORT_EVT_SIMSTATE_DEACTIVATED:
        {
          *simstat = LTE_SIMSTAT_DEACTIVATE;
        }
        break;

      default:
        {
          m_err("Unsupport SIM state. status:%d\n", simstate->state);
          ret = -EILSEQ;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getltime_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_LTIME;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_LTIME_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getoper_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_OPERATOR;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_OPERATOR_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getqual_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_QUALITY;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_QUALITY_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getcell_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_CELLINFO_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_getrat_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_RAT_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_setrat_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR uint8_t *rat = (FAR uint8_t *)arg[0];
  FAR bool *persistent = (FAR bool *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setrat_s *out =
        (FAR struct apicmd_cmddat_setrat_s *)pktbuf;

      out->rat = *rat;
      out->persistency = *persistent;

      size = sizeof(struct apicmd_cmddat_setrat_s);

      *altcid = APICMDID_SET_RAT_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_getimscap_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMS_CAP;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMS_CAP_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setrepqual_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR cellinfo_report_cb_t *callback = (FAR cellinfo_report_cb_t *)arg[0];
  FAR uint32_t *period = (FAR uint32_t *)arg[1];

  FAR struct apicmd_cmddat_setrepquality_s *out =
    (FAR struct apicmd_cmddat_setrepquality_s *)pktbuf;

  if (callback == NULL)
    {
      out->enability = APICMD_SET_REPQUALITY_DISABLE;
    }
  else
    {
      out->enability = APICMD_SET_REPQUALITY_ENABLE;
    }

  out->interval = htonl(*period);
  size = sizeof(struct apicmd_cmddat_setrepquality_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_REP_QUALITY;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_REP_QUALITY_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setrepcell_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR cellinfo_report_cb_t *callback = (FAR cellinfo_report_cb_t *)arg[0];
  FAR uint32_t *period = (FAR uint32_t *)arg[1];

  FAR struct apicmd_cmddat_setrepcellinfo_s *out =
    (FAR struct apicmd_cmddat_setrepcellinfo_s *)pktbuf;

  if (callback == NULL)
    {
      out->enability = LTE_DISABLE;
    }
  else
    {
      out->enability = LTE_ENABLE;
    }

  out->interval = htonl(*period);
  size = sizeof(struct apicmd_cmddat_setrepcellinfo_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_REP_CELLINFO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_REP_CELLINFO_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setrepevt_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR simstat_report_cb_t *callback = (FAR simstat_report_cb_t *)arg[0];
  FAR int32_t *id = (FAR int32_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_setrepevt_s *out =
        (FAR struct apicmd_cmddat_setrepevt_s *)pktbuf;

      if (*id == LTE_CMDID_REPSIMSTAT)
        {
          if (callback)
            {
              g_set_repevt |=
                (APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
            }
          else
            {
              g_set_repevt &=
                ~(APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
            }
        }
      else if(*id == LTE_CMDID_REPLTIME)
        {
          if (callback)
            {
              g_set_repevt |= APICMD_SET_REP_EVT_LTIME;
            }
          else
            {
              g_set_repevt &= ~APICMD_SET_REP_EVT_LTIME;
            }
        }
      else
        {
          return -ENOSYS;
        }

      out->event = g_set_repevt;
      size = sizeof(struct apicmd_cmddat_setrepevt_s);
      *altcid = APICMDID_SET_REP_EVT;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setrepevt_v4_s *out =
        (FAR struct apicmd_cmddat_setrepevt_v4_s *)pktbuf;

      if (*id == LTE_CMDID_REPSIMSTAT)
        {
          out->event = htons(APICMD_SET_REP_EVT_SIMD |
                             APICMD_SET_REP_EVT_SIMSTATE);
        }
      else if(*id == LTE_CMDID_REPLTIME)
        {
          out->event = htons(APICMD_SET_REP_EVT_LTIME);
        }
      else
        {
          return -ENOSYS;
        }

      out->enability = callback ? APICMD_SET_REP_EVT_ENABLE :
                                  APICMD_SET_REP_EVT_DISABLE;
      size = sizeof(struct apicmd_cmddat_setrepevt_v4_s);
      *altcid = APICMDID_SET_REP_EVT_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getltime_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_localtime_t *localtime = (FAR lte_localtime_t *)arg[1];
  FAR struct apicmd_cmddat_getltimeres_s *in =
    (FAR struct apicmd_cmddat_getltimeres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Parse local time */

      getltime_parse_response(in, localtime);
    }

  return 0;
}

int32_t altcom_getoper_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR char *oper = (FAR char *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_getoperatorres_s *in =
        (FAR struct apicmd_cmddat_getoperatorres_s *)pktbuf;

      *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
      if (0 == *ret)
        {
          if (arglen > 2)
            {
              FAR size_t *len = (FAR size_t *)arg[2];

              /* Is it enough length to include Null terminate?
               * The length of LTE_OPERATOR_LEN includes the
               * null terminator.
               */

              if (*len < strnlen((FAR const char *)in->oper,
                LTE_OPERATOR_LEN))
                {
                  return -ENOBUFS;
                }
            }

          strncpy(oper, (FAR const char *)in->oper, LTE_OPERATOR_LEN);
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getoperatorres_v4_s *in =
        (FAR struct apicmd_cmddat_getoperatorres_v4_s *)pktbuf;

      *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
      if (0 == *ret)
        {
          if (arglen > 2)
            {
              FAR size_t *len = (FAR size_t *)arg[2];

              /* Is it enough length to include Null terminate?
               * The length of LTE_OPERATOR_LEN includes the
               * null terminator.
               */

              if (*len < strnlen((FAR const char *)in->oper,
                LTE_OPERATOR_LEN))
                {
                  return -ENOBUFS;
                }
            }

          strncpy(oper, (FAR const char *)in->oper, LTE_OPERATOR_LEN);
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_getqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_quality_t *quality = (FAR lte_quality_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_getqualityres_s *in =
        (FAR struct apicmd_cmddat_getqualityres_s *)pktbuf;

      *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
      if (0 == *ret)
        {
          /* Parse quality information */

          altcombs_set_quality(quality, &in->quality);
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getqualityres_v4_s *in =
        (FAR struct apicmd_cmddat_getqualityres_v4_s *)pktbuf;

      *ret = 0;

      /* Parse quality information */

      altcombs_set_quality(quality, &in->quality);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_getcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_cellinfo_t *cellinfo = (FAR lte_cellinfo_t *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getcellinfores_v4_s *in =
        (FAR struct apicmd_cmddat_getcellinfores_v4_s *)pktbuf;

      /* Parse LTE network cell information */

      altcombs_set_cellinfo_v4(&in->cellinfo, cellinfo);

      *ret = 0;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_getrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_ratinfo_t *ratinfo = (FAR lte_ratinfo_t *)arg[1];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getratres_s *in =
        (FAR struct apicmd_cmddat_getratres_s *)pktbuf;

      *ret = ntohl(in->result);

      if (0 > *ret)
        {
          m_err("Modem returned an error [%ld].\n", *ret);
        }
      else
        {
          ratinfo->rat               = in->rat;
          ratinfo->multi_rat_support = (bool)in->rat_mode;
          ratinfo->source            = in->source;
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_setrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  FAR int *ret = (FAR int *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setratres_s *in =
        (FAR struct apicmd_cmddat_setratres_s *)pktbuf;

      *ret = ntohl(in->result);

      if (0 > *ret)
        {
          m_err("Modem returned an error [%ld].\n", *ret);
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_getimscap_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR bool *imscap = (FAR bool *)arg[1];
  FAR struct apicmd_cmddat_getimscapres_s *in =
    (FAR struct apicmd_cmddat_getimscapres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      *imscap = (in->ims_cap == APICMD_IMSCAP_ENABLE) ?
                LTE_ENABLE : LTE_DISABLE;
    }

  return 0;
}

int32_t altcom_setrepqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setrepquality_res_s *in =
    (FAR struct apicmd_cmddat_setrepquality_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EIO;

  return 0;
}

int32_t altcom_setrepcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setrepcellinfo_res_s *in =
    (FAR struct apicmd_cmddat_setrepcellinfo_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EIO;

  return 0;
}

int32_t altcom_repqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR lte_quality_t *quality = (FAR lte_quality_t *)arg[0];
  int ret = 0;

  FAR struct apicmd_cmddat_quality_s *in =
    (FAR struct apicmd_cmddat_quality_s *)pktbuf;

  ret = altcombs_set_quality(quality, in);
  if (0 > ret)
    {
      return -EFAULT;
    }

  return 0;
}

int32_t altcom_repcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  struct cellinfo_helper_s
    {
      lte_cellinfo_t info;
      lte_neighbor_cell_t neighbors[LTE_NEIGHBOR_CELL_MAX];
    };

  FAR lte_cellinfo_t *cellinfo = (FAR lte_cellinfo_t *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_cellinfo_s *in =
        (FAR struct apicmd_cmddat_cellinfo_s *)pktbuf;

      cellinfo->nr_neighbor = 0;

      altcombs_set_cellinfo(in, cellinfo);
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_cellinfo_v4_s *in =
        (FAR struct apicmd_cmddat_cellinfo_v4_s *)pktbuf;

      cellinfo->nr_neighbor = LTE_NEIGHBOR_CELL_MAX;

      altcombs_set_cellinfo_v4(in, cellinfo);
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_setrepevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_setrepevtres_s *in =
        (FAR struct apicmd_cmddat_setrepevtres_s *)pktbuf;

      *ret = (APICMD_SET_REP_EVT_RES_OK == in->result) ? 0 : -EIO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setrepevtres_v4_s *in =
        (FAR struct apicmd_cmddat_setrepevtres_v4_s *)pktbuf;

      *ret = (APICMD_SET_REP_EVT_RES_OK == in->result) ? 0 : -EIO;
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_repevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = -ENOTSUP;
  alt_evtbuf_inst_t *inst;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_repevt_s *in =
        (FAR struct apicmd_cmddat_repevt_s *)pktbuf;

      switch (in->type)
        {
          case APICMD_REPORT_EVT_TYPE_LTIME:
            {
              *bitmap = get_event_lapibuffer(dev, LTE_CMDID_REPLTIME, &inst);
              if (*bitmap != 0ULL)
                {
                  FAR lte_localtime_t *ltime = inst->outparam[0];
                  parse_ltime(&in->u.ltime, ltime);
                  ret = 0;
                }
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMD:
          case APICMD_REPORT_EVT_TYPE_SIMSTATE:
            {
              *bitmap = get_event_lapibuffer(dev, LTE_CMDID_REPSIMSTAT,
                                             &inst);
              if (*bitmap != 0ULL)
                {
                  FAR uint32_t *simstat = inst->outparam[0];
                  if (in->type == APICMD_REPORT_EVT_TYPE_SIMD)
                    {
                      ret = parse_simd(&in->u.simd, simstat);
                    }
                  else
                    {
                      ret = parse_simstate(&in->u.simstate, simstat);
                    }
                }
            }
            break;

          default:
            {
              m_err("Unsupport event type. type:%d\n", in->type);
              ret = -EILSEQ;
            }
            break;
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_repevt_v4_s *in =
        (FAR struct apicmd_cmddat_repevt_v4_s *)pktbuf;

      switch (ntohs(in->type))
        {
          case APICMD_REPORT_EVT_TYPE_LTIME:
            {
              *bitmap = get_event_lapibuffer(dev, LTE_CMDID_REPLTIME, &inst);
              if (*bitmap != 0ULL)
                {
                  FAR lte_localtime_t *ltime = inst->outparam[0];
                  parse_ltime(&in->u.ltime, ltime);
                  ret = 0;
                }
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMD:
          case APICMD_REPORT_EVT_TYPE_SIMSTATE:
            {
              *bitmap = get_event_lapibuffer(dev, LTE_CMDID_REPSIMSTAT,
                                             &inst);
              if (*bitmap != 0ULL)
                {
                  FAR uint32_t *simstat = inst->outparam[0];
                  if (ntohs(in->type) == APICMD_REPORT_EVT_TYPE_SIMD)
                    {
                      ret = parse_simd(&in->u.simd, simstat);
                    }
                  else
                    {
                      ret = parse_simstate(&in->u.simstate, simstat);
                    }
                }
            }
            break;

          default:
            {
              m_err("Unsupport event type. type:%d\n",
                ntohs(in->type));
              ret = -EILSEQ;
            }
            break;
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return ret;
}
