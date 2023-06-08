/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_pdn.c
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

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
static int32_t altcombs_set_pdninfo(struct apicmd_pdnset_s *cmd_pdn,
                                    lte_pdn_t *lte_pdn)
{
  int32_t i;

  if (!cmd_pdn || !lte_pdn)
    {
      return -EINVAL;
    }

  lte_pdn->session_id = cmd_pdn->session_id;
  lte_pdn->active = cmd_pdn->activate;
  lte_pdn->apn_type = htonl(cmd_pdn->apntype);
  lte_pdn->ipaddr_num = cmd_pdn->ipaddr_num;
  for (i = 0; i < lte_pdn->ipaddr_num; i++)
    {
      lte_pdn->address[i].ip_type = cmd_pdn->ip_address[i].iptype;
      strncpy(lte_pdn->address[i].address,
              (FAR char *)cmd_pdn->ip_address[i].address,
              LTE_IPADDR_MAX_LEN - 1);
    }

  lte_pdn->ims_register = cmd_pdn->imsregister == APICMD_PDN_IMS_REG ?
    LTE_IMS_REGISTERED : LTE_IMS_NOT_REGISTERED;
  lte_pdn->data_allow = cmd_pdn->dataallow ==
    APICMD_PDN_DATAALLOW_ALLOW ?
    LTE_DATA_ALLOW : LTE_DATA_DISALLOW;
  lte_pdn->data_roaming_allow = cmd_pdn->dararoamingallow ==
    APICMD_PDN_DATAROAMALLOW_ALLOW ?
    LTE_DATA_ALLOW : LTE_DATA_DISALLOW;

  return 0;
}
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
static int32_t altcombs_set_pdninfo_v4(
  FAR struct apicmd_pdnset_v4_s *cmd_pdn,
  FAR lte_pdn_t *lte_pdn, FAR uint8_t *ndnsaddrs,
  FAR struct sockaddr_storage *dnsaddrs)
{
  int32_t i;
  int af;

  if (!cmd_pdn || !lte_pdn)
    {
      return -EINVAL;
    }

  lte_pdn->session_id = cmd_pdn->session_id;
  lte_pdn->active = cmd_pdn->activate;
  lte_pdn->apn_type = htonl(cmd_pdn->apntype);
  lte_pdn->ipaddr_num = cmd_pdn->ipaddr_num;
  for (i = 0; i < lte_pdn->ipaddr_num; i++)
    {
      lte_pdn->address[i].ip_type = cmd_pdn->ip_address[i].iptype;
      strncpy(lte_pdn->address[i].address,
              (FAR char *)cmd_pdn->ip_address[i].address,
              LTE_IPADDR_MAX_LEN - 1);
    }

  lte_pdn->ims_register = cmd_pdn->imsregister;
  lte_pdn->data_allow = cmd_pdn->dataallow;
  lte_pdn->data_roaming_allow = cmd_pdn->dararoamingallow;

  if (ndnsaddrs && dnsaddrs)
    {
      *ndnsaddrs = (cmd_pdn->dnsaddr_num > APICMD_PDN_DNSCOUNT_MAX) ?
        APICMD_PDN_DNSCOUNT_MAX : cmd_pdn->dnsaddr_num;
      for (i = 0; (i < APICMD_PDN_DNSCOUNT_MAX) && (i < *ndnsaddrs); i++)
        {
          af = (cmd_pdn->dns_address[i].iptype == LTE_IPTYPE_V4) ?
            AF_INET : AF_INET6;
          if (af == AF_INET)
            {
              FAR struct sockaddr_in *addr =
                (FAR struct sockaddr_in *)&dnsaddrs[i];

              addr->sin_family = AF_INET;
              addr->sin_port = 0;
              if (1 != inet_pton(af,
                (FAR const char *)cmd_pdn->dns_address[i].address,
                (FAR void *)&addr->sin_addr))
                {
                  /* inet_pton() failed, then force break */

                  *ndnsaddrs = i;
                  break;
                }
            }
          else
            {
              FAR struct sockaddr_in6 *addr =
                (FAR struct sockaddr_in6 *)&dnsaddrs[i];

              addr->sin6_family = AF_INET6;
              addr->sin6_port = 0;
              if (1 != inet_pton(af,
                (FAR const char *)cmd_pdn->dns_address[i].address,
                (FAR void *)&addr->sin6_addr))
                {
                  /* inet_pton() failed, then force break */

                  *ndnsaddrs = i;
                  break;
                }
            }
        }
    }

  return 0;
}
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
static void getnetinfo_parse_response(
  FAR struct apicmd_cmddat_getnetinfores_s *resp,
  FAR lte_netinfo_t *netinfo, uint8_t pdn_num)
{
  uint8_t i;

  netinfo->nw_stat         = resp->netinfo.nw_stat;
  netinfo->nw_err.err_type = resp->netinfo.err_info.err_type;
  netinfo->nw_err.reject_cause.category =
    resp->netinfo.err_info.reject_cause.category;
  netinfo->nw_err.reject_cause.value =
    resp->netinfo.err_info.reject_cause.value;
  netinfo->pdn_num = resp->netinfo.pdn_count;

  if ((0 < resp->netinfo.pdn_count) && (netinfo->pdn_stat))
    {
      for (i = 0; (i < resp->netinfo.pdn_count) && (i < pdn_num); i++)
        {
          altcombs_set_pdninfo(&resp->netinfo.pdn[i],
                               &netinfo->pdn_stat[i]);
        }
    }
}
#endif

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
static void getnetinfo_parse_response_v4(
  FAR struct apicmd_cmddat_getnetinfores_v4_s *resp,
  FAR lte_netinfo_t *netinfo, uint8_t pdn_num)
{
  uint8_t i;

  netinfo->nw_stat         = resp->netinfo.nw_stat;
  netinfo->nw_err.err_type = resp->netinfo.err_info.err_type;
  netinfo->nw_err.reject_cause.category =
    resp->netinfo.err_info.reject_cause.category;
  netinfo->nw_err.reject_cause.value =
    resp->netinfo.err_info.reject_cause.value;
  netinfo->pdn_num = resp->netinfo.pdn_count;

  if ((0 < resp->netinfo.pdn_count) && (netinfo->pdn_stat))
    {
      for (i = 0; (i < resp->netinfo.pdn_count) && (i < pdn_num); i++)
        {
          altcombs_set_pdninfo_v4(&resp->netinfo.pdn[i],
                                  &netinfo->pdn_stat[i], NULL, NULL);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_actpdn_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR lte_apn_setting_t *apn = (FAR lte_apn_setting_t *)arg[0];

  FAR struct apicmd_cmddat_activatepdn_s *out =
    (FAR struct apicmd_cmddat_activatepdn_s *)pktbuf;

  out->apntype = htonl(apn->apn_type);
  if (apn->ip_type == LTE_IPTYPE_NON && altver == ALTCOM_VER1)
    {
      return -ENOTSUP;
    }

  out->iptype  = apn->ip_type;
  out->authtype  = apn->auth_type;

  strncpy((FAR char *)out->apnname, (FAR const char *)apn->apn,
    sizeof(out->apnname));
  if (apn->auth_type != LTE_APN_AUTHTYPE_NONE)
    {
      strncpy((FAR char *)out->username, (FAR const char *)apn->user_name,
        sizeof(out->username));
      strncpy((FAR char *)out->password, (FAR const char *)apn->password,
        sizeof(out->password));
    }

  size = sizeof(struct apicmd_cmddat_activatepdn_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ACTIVATE_PDN;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ACTIVATE_PDN_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_deactpdn_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR uint8_t *session_id = (FAR uint8_t *)arg[0];

  FAR struct apicmd_cmddat_deactivatepdn_s *out =
    (FAR struct apicmd_cmddat_deactivatepdn_s *)pktbuf;

  out->session_id = *session_id;

  size = sizeof(struct apicmd_cmddat_deactivatepdn_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_DEACTIVATE_PDN;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_DEACTIVATE_PDN_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_actpdncancel_pkt_compose(FAR void **arg, size_t arglen,
                                        uint8_t altver, FAR uint8_t *pktbuf,
                                        const size_t pktsz,
                                        FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ACTIVATE_PDN_CANCEL;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ACTIVATE_PDN_CANCEL_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_getnetinfo_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_NETINFO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_NETINFO_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_setrepnet_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR struct apicmd_cmddat_set_repnetinfo_s *out =
    (FAR struct apicmd_cmddat_set_repnetinfo_s *)pktbuf;

  out->report = APICMD_REPNETINFO_REPORT_ENABLE;

  size = sizeof(struct apicmd_cmddat_set_repnetinfo_s);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SETREP_NETINFO;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SETREP_NETINFO_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_actpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_pdn_t *pdn = (FAR lte_pdn_t *)arg[1];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_activatepdnres_s *in =
        (FAR struct apicmd_cmddat_activatepdnres_s *)pktbuf;

      if (LTE_RESULT_OK == in->result)
        {
          /* Parse PDN information */

          altcombs_set_pdninfo(&in->pdnset, pdn);
          *ret = in->result;
        }
      else
        {
          *ret = (LTE_RESULT_CANCEL == in->result) ? -ECANCELED : -EPROTO;
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_activatepdnres_v4_s *in =
        (FAR struct apicmd_cmddat_activatepdnres_v4_s *)pktbuf;

      if (LTE_RESULT_OK == in->result)
        {
          /* Parse PDN information */

          altcombs_set_pdninfo_v4(&in->pdnset, pdn, NULL, NULL);
          *ret = in->result;
        }
      else
        {
          *ret = (LTE_RESULT_CANCEL == in->result) ? -ECANCELED : -EPROTO;
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_deactpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_deactivatepdnres_s *in =
    (FAR struct apicmd_cmddat_deactivatepdnres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

int32_t altcom_actpdncancel_pkt_parse(FAR struct alt1250_dev_s *dev,
                                      FAR uint8_t *pktbuf, size_t pktsz,
                                      uint8_t altver, FAR void **arg,
                                      size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_activatepdn_cancel_res_s *in =
    (FAR struct apicmd_cmddat_activatepdn_cancel_res_s *)pktbuf;

  *ret = 0;

  if (LTE_RESULT_OK != in->result)
    {
      m_err("API command response is err.\n");
      *ret = -EIO;
    }

  return 0;
}

int32_t altcom_getnetinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_netinfo_t *info = (FAR lte_netinfo_t *)arg[1];
  FAR uint8_t *pdn_num = (FAR uint8_t *)arg[2];

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_getnetinfores_s *in =
        (FAR struct apicmd_cmddat_getnetinfores_s *)pktbuf;

      *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
      if (0 == *ret)
        {
          /* Parse network information */

          getnetinfo_parse_response(in, info, *pdn_num);
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getnetinfores_v4_s *in =
        (FAR struct apicmd_cmddat_getnetinfores_v4_s *)pktbuf;

      *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
      if (0 == *ret)
        {
          /* Parse network information */

          getnetinfo_parse_response_v4(in, info, *pdn_num);
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}

int32_t altcom_setrepnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_set_repnetinfores_s *in =
    (FAR struct apicmd_cmddat_set_repnetinfores_s *)pktbuf;

  *ret = (APICMD_REPNETINFO_RES_OK == in->result ? 0 : -EIO);

  return 0;
}

int32_t altcom_repnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR lte_netinfo_t *netinfo = (FAR lte_netinfo_t *)arg[0];
  FAR uint8_t *ndnsaddrs = (FAR uint8_t *)arg[1];
  FAR struct sockaddr_storage *dnsaddrs =
    (FAR struct sockaddr_storage *)arg[2];
  int i = 0;

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_rep_netinfo_s *in =
        (FAR struct apicmd_cmddat_rep_netinfo_s *)pktbuf;

      netinfo->nw_stat = in->netinfo.nw_stat;
      netinfo->nw_err.err_type = in->netinfo.err_info.err_type;
      netinfo->nw_err.reject_cause.category = in->netinfo.
       err_info.reject_cause.category;
      netinfo->nw_err.reject_cause.value = in->netinfo.
        err_info.reject_cause.value;
      netinfo->pdn_num = in->netinfo.pdn_count;
      if (0 < in->netinfo.pdn_count)
        {
          for (i = 0; i < in->netinfo.pdn_count; i++)
            {
              altcombs_set_pdninfo(&in->netinfo.pdn[i],
                     &netinfo->pdn_stat[i]);
            }
        }

      /* parse DNS address if exists */

      if (pktsz == (sizeof(struct apicmd_cmddat_rep_netinfo_s)))
        {
          *ndnsaddrs = 0;

          /* parse IPv4 DNS address */

          if (*(uint32_t *)in->dnsaddrv4 != 0)
            {
              FAR struct sockaddr_in *v4addr =
                (FAR struct sockaddr_in *)&dnsaddrs[0];

              v4addr->sin_family = AF_INET;
              v4addr->sin_port = 0;
              memcpy(&v4addr->sin_addr, in->dnsaddrv4,
                sizeof(v4addr->sin_addr));
              (*ndnsaddrs)++;
            }

          /* parse IPv6 DNS address */

          if (!((*(uint32_t *)&in->dnsaddrv6[0] == 0) &&
               (*(uint32_t *)&in->dnsaddrv6[4] == 0) &&
               (*(uint32_t *)&in->dnsaddrv6[8] == 0) &&
               (*(uint32_t *)&in->dnsaddrv6[12] == 0)))
            {
              FAR struct sockaddr_in6 *v6addr =
                (FAR struct sockaddr_in6 *)&dnsaddrs[*ndnsaddrs];

              v6addr->sin6_family = AF_INET6;
              v6addr->sin6_port = 0;
              memcpy(&v6addr->sin6_addr, in->dnsaddrv6,
                sizeof(v6addr->sin6_addr));
              (*ndnsaddrs)++;
            }
        }
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_rep_netinfo_v4_s *in =
        (FAR struct apicmd_cmddat_rep_netinfo_v4_s *)pktbuf;

      netinfo->nw_stat = in->netinfo.nw_stat;
      netinfo->nw_err.err_type = in->netinfo.err_info.err_type;
      netinfo->nw_err.reject_cause.category = in->netinfo.
        err_info.reject_cause.category;
      netinfo->nw_err.reject_cause.value = in->netinfo.
        err_info.reject_cause.value;
      netinfo->pdn_num = in->netinfo.pdn_count;
      if (0 < in->netinfo.pdn_count)
        {
          for (i = 0; i < in->netinfo.pdn_count; i++)
            {
              if (i == 0)
                {
                  altcombs_set_pdninfo_v4(&in->netinfo.pdn[i],
                    &netinfo->pdn_stat[i], ndnsaddrs, dnsaddrs);
                }
              else
                {
                  altcombs_set_pdninfo_v4(&in->netinfo.pdn[i],
                    &netinfo->pdn_stat[i], NULL, NULL);
                }
            }
        }
    }
  else
#endif
    {
      return -ENOSYS;
    }

  return 0;
}
