/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_psave.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

#define ALTCOM_GETEDRX_TYPE_UE         0
#define ALTCOM_GETEDRX_TYPE_NEGOTIATED 1
#define ALTCOM_GETPSM_TYPE_UE          0
#define ALTCOM_GETPSM_TYPE_NEGOTIATED  1

#define ALTCOMBS_EDRX_CYCLE_WBS1_MIN      (LTE_EDRX_CYC_512)
#define ALTCOMBS_EDRX_CYCLE_WBS1_MAX      (LTE_EDRX_CYC_262144)
#define ALTCOMBS_EDRX_CYCLE_NBS1_MIN      (LTE_EDRX_CYC_2048)
#define ALTCOMBS_EDRX_CYCLE_NBS1_MAX      (LTE_EDRX_CYC_1048576)
#define ALTCOMBS_EDRX_PTW_WBS1_MIN        (0)
#define ALTCOMBS_EDRX_PTW_WBS1_MAX        (15)
#define ALTCOMBS_EDRX_PTW_NBS1_MIN        (0)
#define ALTCOMBS_EDRX_PTW_NBS1_MAX        (15)
#define ALTCOMBS_PSM_UNIT_T3324_MIN       (LTE_PSM_T3324_UNIT_2SEC)
#define ALTCOMBS_PSM_UNIT_T3324_MAX       (LTE_PSM_T3324_UNIT_6MIN)
#define ALTCOMBS_PSM_UNIT_T3412_MIN       (LTE_PSM_T3412_UNIT_2SEC)
#define ALTCOMBS_PSM_UNIT_T3412_MAX       (LTE_PSM_T3412_UNIT_320HOUR)

#define ALTCOMBS_EDRX_INVALID             (255)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_edrx_ptw_nbs1_table[] =
{
  LTE_EDRX_PTW_256,
  LTE_EDRX_PTW_512,
  LTE_EDRX_PTW_768,
  LTE_EDRX_PTW_1024,
  LTE_EDRX_PTW_1280,
  LTE_EDRX_PTW_1536,
  LTE_EDRX_PTW_1792,
  LTE_EDRX_PTW_2048,
  LTE_EDRX_PTW_2304,
  LTE_EDRX_PTW_2560,
  LTE_EDRX_PTW_2816,
  LTE_EDRX_PTW_3072,
  LTE_EDRX_PTW_3328,
  LTE_EDRX_PTW_3584,
  LTE_EDRX_PTW_3840,
  LTE_EDRX_PTW_4096,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t altcombs_convert_api_edrx_value(
  FAR lte_edrx_setting_t *api_edrx,
  FAR struct apicmd_cmddat_setedrx_s *cmd_edrx, uint8_t altver)
{
  int           i;
  int           table_size = 0;

  if (!cmd_edrx || !api_edrx)
    {
      m_err("null param\n");
      return -EINVAL;
    }

  /* act_type check for version V4 or later */

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      if (api_edrx->act_type == LTE_EDRX_ACTTYPE_WBS1)
        {
          cmd_edrx->acttype = APICMD_EDRX_ACTTYPE_WBS1;
        }
      else if (api_edrx->act_type == LTE_EDRX_ACTTYPE_NOTUSE)
        {
          cmd_edrx->acttype = APICMD_EDRX_ACTTYPE_NOTUSE;
        }
       else
        {
          m_err("Operation is not allowed[act_type : %d].\n",
                api_edrx->act_type);
          return -EPERM;
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      if (api_edrx->act_type == LTE_EDRX_ACTTYPE_WBS1)
        {
          cmd_edrx->acttype = APICMD_EDRX_ACTTYPE_WBS1;
        }
      else if (api_edrx->act_type == LTE_EDRX_ACTTYPE_NBS1)
        {
          cmd_edrx->acttype = APICMD_EDRX_ACTTYPE_NBS1;
        }
      else if (api_edrx->act_type == LTE_EDRX_ACTTYPE_NOTUSE)
        {
          cmd_edrx->acttype = APICMD_EDRX_ACTTYPE_NOTUSE;
        }
      else
        {
          m_err("Operation is not allowed[act_type : %d].\n",
                api_edrx->act_type);
          return -EPERM;
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  if (api_edrx->enable)
    {
      cmd_edrx->enable = LTE_ENABLE;

      if (APICMD_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          if (api_edrx->edrx_cycle <= LTE_EDRX_CYC_262144)
            {
              cmd_edrx->edrx_cycle = api_edrx->edrx_cycle;
            }
          else
            {
              m_err("Invalid cycle :%ld\n", api_edrx->edrx_cycle);
              return -EINVAL;
            }

          if (api_edrx->ptw_val <= LTE_EDRX_PTW_2048)
            {
              cmd_edrx->ptw_val = api_edrx->ptw_val;
            }
          else
            {
              m_err("Invalid PTW :%ld\n", api_edrx->ptw_val);
              return -EINVAL;
            }
        }
      else if (APICMD_EDRX_ACTTYPE_NBS1 == cmd_edrx->acttype)
        {
          if (api_edrx->edrx_cycle <= LTE_EDRX_CYC_1048576 &&
              (api_edrx->edrx_cycle == LTE_EDRX_CYC_2048 ||
               api_edrx->edrx_cycle == LTE_EDRX_CYC_4096 ||
               api_edrx->edrx_cycle == LTE_EDRX_CYC_8192 ||
               api_edrx->edrx_cycle >= LTE_EDRX_CYC_16384))
            {
              cmd_edrx->edrx_cycle = api_edrx->edrx_cycle;
            }
          else
            {
              m_err("Invalid cycle :%ld\n", api_edrx->edrx_cycle);
              return -EINVAL;
            }

          table_size = ARRAY_SZ(g_edrx_ptw_nbs1_table);

          for (i = 0; i < table_size; i++)
            {
              if (api_edrx->ptw_val == g_edrx_ptw_nbs1_table[i])
                {
                  cmd_edrx->ptw_val = (uint8_t)i;
                  break;
                }
            }

          if (i == table_size)
            {
              m_err("Invalid PTW :%ld\n", api_edrx->ptw_val);
              return -EINVAL;
            }
        }
    }
  else
    {
      cmd_edrx->enable = LTE_DISABLE;
    }

  return 0;
}

static int32_t altcombs_convert_apicmd_edrx_value(
  FAR struct apicmd_edrxset_s *cmd_edrx, FAR lte_edrx_setting_t *api_edrx)
{
  if (!cmd_edrx || !api_edrx)
    {
      m_err("null param\n");
      return -EINVAL;
    }

  if (cmd_edrx->enable == LTE_ENABLE)
    {
      api_edrx->enable = LTE_ENABLE;

      if (cmd_edrx->acttype == APICMD_EDRX_ACTTYPE_NOTUSE)
        {
          api_edrx->act_type = LTE_EDRX_ACTTYPE_NOTUSE;
        }
      else if (cmd_edrx->acttype == APICMD_EDRX_ACTTYPE_WBS1)
        {
          api_edrx->act_type = LTE_EDRX_ACTTYPE_WBS1;
          if (cmd_edrx->edrx_cycle >= ALTCOMBS_EDRX_CYCLE_WBS1_MIN &&
              cmd_edrx->edrx_cycle <= ALTCOMBS_EDRX_CYCLE_WBS1_MAX)
            {
              api_edrx->edrx_cycle = cmd_edrx->edrx_cycle;
            }
          else
            {
              m_err("Invalid cycle :%d\n", cmd_edrx->edrx_cycle);
              return -EINVAL;
            }

          if (cmd_edrx->ptw_val >= ALTCOMBS_EDRX_PTW_WBS1_MIN &&
              cmd_edrx->ptw_val <= ALTCOMBS_EDRX_PTW_WBS1_MAX)
            {
              api_edrx->ptw_val = cmd_edrx->ptw_val;
            }
          else
            {
              m_err("Invalid PTW :%d\n", cmd_edrx->ptw_val);
              return -EINVAL;
            }
        }
      else if (cmd_edrx->acttype == APICMD_EDRX_ACTTYPE_NBS1)
        {
          api_edrx->act_type = LTE_EDRX_ACTTYPE_NBS1;
          if (cmd_edrx->edrx_cycle <= LTE_EDRX_CYC_1048576 &&
              (cmd_edrx->edrx_cycle == LTE_EDRX_CYC_2048 ||
               cmd_edrx->edrx_cycle == LTE_EDRX_CYC_4096 ||
               cmd_edrx->edrx_cycle == LTE_EDRX_CYC_8192 ||
               cmd_edrx->edrx_cycle >= LTE_EDRX_CYC_16384))
            {
              api_edrx->edrx_cycle = cmd_edrx->edrx_cycle;
            }
          else
            {
              m_err("Invalid cycle :%d\n", cmd_edrx->edrx_cycle);
              return -EINVAL;
            }

          if (cmd_edrx->ptw_val >= ALTCOMBS_EDRX_PTW_NBS1_MIN &&
              cmd_edrx->ptw_val <= ALTCOMBS_EDRX_PTW_NBS1_MAX)
            {
              api_edrx->ptw_val = g_edrx_ptw_nbs1_table[cmd_edrx->ptw_val];
            }
          else
            {
              m_err("Invalid PTW :%d\n", cmd_edrx->ptw_val);
              return -EINVAL;
            }
        }
      else
        {
          m_err("Invalid acttype :%d\n", cmd_edrx->acttype);
          return -EINVAL;
        }
    }
  else if (cmd_edrx->enable == LTE_DISABLE)
    {
      api_edrx->enable = LTE_DISABLE;
    }
  else
    {
      m_err("Invalid enable :%d\n", cmd_edrx->enable);
      return -EINVAL;
    }

  return 0;
}

static int32_t altcombs_set_psm(FAR struct apicmd_cmddat_psm_set_s *cmd_set,
                                FAR lte_psm_setting_t *api_set)
{
  if (!cmd_set || !api_set)
    {
      return -EINVAL;
    }

  api_set->enable                         = cmd_set->enable;
  api_set->req_active_time.unit           = cmd_set->rat_time.unit;
  api_set->req_active_time.time_val       = cmd_set->rat_time.time_val;
  api_set->ext_periodic_tau_time.unit     = cmd_set->tau_time.unit;
  api_set->ext_periodic_tau_time.time_val = cmd_set->tau_time.time_val;

  return 0;
}

static void getce_parse_response(FAR struct apicmd_cmddat_getceres_s *resp,
                                 FAR lte_ce_setting_t *ce)
{
  ce->mode_a_enable = resp->mode_a_enable;
  ce->mode_b_enable = resp->mode_b_enable;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_getedrx_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_EDRX;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getedrx_v4_s *out =
        (FAR struct apicmd_cmddat_getedrx_v4_s *)pktbuf;

      out->type = ALTCOM_GETEDRX_TYPE_UE;
      size = sizeof(struct apicmd_cmddat_getedrx_v4_s);
      *altcid = APICMDID_GET_EDRX_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_setedrx_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR lte_edrx_setting_t *settings = (FAR lte_edrx_setting_t *)arg[0];
  int ret = 0;

  FAR struct apicmd_cmddat_setedrx_s *out =
    (FAR struct apicmd_cmddat_setedrx_s *)pktbuf;

  size = sizeof(struct apicmd_cmddat_setedrx_s);
  ret = altcombs_convert_api_edrx_value(settings, out, altver);
  if (ret < 0)
    {
      size = ret;
    }

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_EDRX;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_EDRX_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getdedrx_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_DYNAMICEDRX;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getedrx_v4_s *out =
        (FAR struct apicmd_cmddat_getedrx_v4_s *)pktbuf;

      out->type = ALTCOM_GETEDRX_TYPE_NEGOTIATED;
      size = sizeof(struct apicmd_cmddat_getedrx_v4_s);
      *altcid = APICMDID_GET_EDRX_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_getpsm_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_PSM;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getpsm_v4_s *out =
        (FAR struct apicmd_cmddat_getpsm_v4_s *)pktbuf;

      out->type = ALTCOM_GETPSM_TYPE_UE;

      size = sizeof(struct apicmd_cmddat_getpsm_v4_s);
      *altcid = APICMDID_GET_PSM_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_setpsm_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR lte_psm_setting_t *settings = (FAR lte_psm_setting_t *)arg[0];

  FAR struct apicmd_cmddat_setpsm_s *out =
    (FAR struct apicmd_cmddat_setpsm_s *)pktbuf;

  out->set.enable            = settings->enable;
  out->set.rat_time.unit     = settings->req_active_time.unit;
  out->set.rat_time.time_val = settings->req_active_time.time_val;
  out->set.tau_time.unit     = settings->ext_periodic_tau_time.unit;
  out->set.tau_time.time_val = settings->ext_periodic_tau_time.time_val;

  size = sizeof(struct apicmd_cmddat_setpsm_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PSM;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PSM_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getdpsm_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_DYNAMICPSM;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getpsm_v4_s *out =
        (FAR struct apicmd_cmddat_getpsm_v4_s *)pktbuf;

      out->type = ALTCOM_GETPSM_TYPE_NEGOTIATED;

      size = sizeof(struct apicmd_cmddat_getpsm_v4_s);
      *altcid = APICMDID_GET_PSM_V4;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_getce_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_CE;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_setce_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  FAR lte_ce_setting_t *settings = (FAR lte_ce_setting_t *)arg[0];
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_setce_s *out =
        (FAR struct apicmd_cmddat_setce_s *)pktbuf;

      out->mode_a_enable = (uint8_t)settings->mode_a_enable;
      out->mode_b_enable = (uint8_t)settings->mode_b_enable;

      size = sizeof(struct apicmd_cmddat_setce_s);
      *altcid = APICMDID_SET_CE;
    }
  else
#endif
    {
      size = -ENOTSUP;
    }

  return size;
}

int32_t altcom_getedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_edrx_setting_t *settings = (FAR lte_edrx_setting_t *)arg[1];
  FAR struct apicmd_cmddat_getedrxres_s *in =
    (FAR struct apicmd_cmddat_getedrxres_s *)pktbuf;
  FAR bool *is_getedrxevt = (FAR bool *)arg[2];

  *is_getedrxevt = true;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Checks and converts the eDRX value of the API command response. */

      *ret = altcombs_convert_apicmd_edrx_value(&in->set, settings);
      if (0 > *ret)
        {
          m_err("altcombs_convert_apicmd_edrx_value() failed: %ld\n", *ret);
          *ret = -EFAULT;
        }
    }

  return 0;
}

int32_t altcom_setedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setedrxres_s *in =
    (FAR struct apicmd_cmddat_setedrxres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

int32_t altcom_getdedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_edrx_setting_t *settings = (FAR lte_edrx_setting_t *)arg[1];
  FAR struct apicmd_cmddat_getdynamicedrxres_s *in =
    (FAR struct apicmd_cmddat_getdynamicedrxres_s *)pktbuf;
  FAR bool *is_getdedrxevt = (FAR bool *)arg[2];

  *is_getdedrxevt = true;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Checks and converts the eDRX value of the API command response. */

      *ret = altcombs_convert_apicmd_edrx_value(&in->set, settings);
      if (0 > *ret)
        {
          m_err("altcombs_convert_apicmd_edrx_value() failed: %ld\n", *ret);
          *ret = -EFAULT;
        }
    }

  return 0;
}

int32_t altcom_getpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_psm_setting_t *settings = (FAR lte_psm_setting_t *)arg[1];
  FAR bool *is_getpsmevt =  (FAR bool *)arg[2];

  FAR struct apicmd_cmddat_getpsmres_s *in =
    (FAR struct apicmd_cmddat_getpsmres_s *)pktbuf;

  *is_getpsmevt = true;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Parse PSM settings */

      *ret = altcombs_set_psm(&in->set, settings);
      if (0 > *ret)
        {
          m_err("altcombs_set_psm() failed: %ld\n", *ret);
          *ret = -EFAULT;
        }
    }

  return 0;
}

int32_t altcom_setpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setpsmres_s *in =
    (FAR struct apicmd_cmddat_setpsmres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

int32_t altcom_getdpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_psm_setting_t *settings = (FAR lte_psm_setting_t *)arg[1];
  FAR struct apicmd_cmddat_getdynamicpsmres_s *in =
    (FAR struct apicmd_cmddat_getdynamicpsmres_s *)pktbuf;
  FAR bool *is_getcpsmevt = (FAR bool *)arg[2];

  *is_getcpsmevt = true;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Parse PSM settings */

      *ret = altcombs_set_psm(&in->set, settings);
      if (0 > *ret)
        {
          m_err("altcombs_set_psm() failed: %ld\n", *ret);
          *ret = -EFAULT;
        }
    }

  return 0;
}

int32_t altcom_getce_pkt_parse(FAR struct alt1250_dev_s *dev,
                               FAR uint8_t *pktbuf, size_t pktsz,
                               uint8_t altver, FAR void **arg,
                               size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_ce_setting_t *settings = (FAR lte_ce_setting_t *)arg[1];
  FAR struct apicmd_cmddat_getceres_s *in =
    (FAR struct apicmd_cmddat_getceres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  if (0 == *ret)
    {
      /* Parse CE settings */

      getce_parse_response(in, settings);
    }

  return 0;
}

int32_t altcom_setce_pkt_parse(FAR struct alt1250_dev_s *dev,
                               FAR uint8_t *pktbuf, size_t pktsz,
                               uint8_t altver, FAR void **arg,
                               size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setceres_s *in =
    (FAR struct apicmd_cmddat_setceres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}
