/****************************************************************************
 * drivers/modem/alt1250/altcom_cmd.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/wireless/lte/lte.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_CELLINFO_CELLID_MAX    (503)
#define APICMD_CELLINFO_EARFCN_MAX    (262143)
#define APICMD_CELLINFO_DIGIT_NUM_MIN (0)
#define APICMD_CELLINFO_DIGIT_NUM_MAX (9)
#define APICMD_CELLINFO_MNC_DIGIT_MIN         (2)
#define APICMD_CELLINFO_GCID_MAX              (16)
#define APICMD_CELLINFO_VALID_TIMEDIFFIDX     (1 << 1)
#define APICMD_CELLINFO_VALID_TA              (1 << 2)
#define APICMD_CELLINFO_VALID_SFN             (1 << 3)
#define APICMD_CELLINFO_VALID_RSRP            (1 << 4)
#define APICMD_CELLINFO_VALID_RSRQ            (1 << 5)

#define APICMD_EDRX_ACTTYPE_NOTUSE   (0) /* eDRX is not running */
#define APICMD_EDRX_ACTTYPE_ECGSMIOT (1) /* EC-GSM-IoT (A/Gb mode) */
#define APICMD_EDRX_ACTTYPE_GSM      (2) /* GSM (A/Gb mode) */
#define APICMD_EDRX_ACTTYPE_IU       (3) /* UTRAN (Iu mode) */
#define APICMD_EDRX_ACTTYPE_WBS1     (4) /* E-UTRAN (WB-S1 mode) */
#define APICMD_EDRX_ACTTYPE_NBS1     (5) /* E-UTRAN (NB-S1 mode) */

#define APICMD_ENTERPIN_NEWPINCODE_UNUSE         (0)
#define APICMD_ENTERPIN_NEWPINCODE_USE           (1)
#define APICMD_ENTERPIN_PINCODE_LEN               9

#define APICMD_ERRINFO_ERRSTR_MAX_LEN            (64)

#define APICMD_IMSCAP_ENABLE                     (0)
#define APICMD_IMSCAP_DISABLE                    (1)

#define APICMD_NETINFO_PDNCOUNT_MAX              (5)

#define APICMD_OPERATOR_LEN_V4  33

#define APICMD_PDN_IMS_REG                (0)
#define APICMD_PDN_IMS_UNREG              (1)

#define APICMD_PDN_DATAALLOW_ALLOW        (0)
#define APICMD_PDN_DATAALLOW_DISALLOW     (1)

#define APICMD_PDN_DATAROAMALLOW_ALLOW    (0)
#define APICMD_PDN_DATAROAMALLOW_DISALLOW (1)

#define APICMD_PDN_DNSCOUNT_MAX           (4)

#define APICMD_QUALITY_DISABLE  (0)
#define APICMD_QUALITY_ENABLE   (1)
#define APICMD_QUALITY_RSRP_MIN (-140)
#define APICMD_QUALITY_RSRP_MAX (0)
#define APICMD_QUALITY_RSRQ_MIN (-60)
#define APICMD_QUALITY_RSRQ_MAX (0)
#define APICMD_QUALITY_SINR_MIN (-128)
#define APICMD_QUALITY_SINR_MAX (40)

#define APICMD_SET_REP_EVT_DISABLE                          (0)
#define APICMD_SET_REP_EVT_ENABLE                           (1)

#define APICMD_SET_REP_EVT_LTIME                            (1 << 0)
#define APICMD_SET_REP_EVT_SIMD                             (1 << 1)
#define APICMD_SET_REP_EVT_SIMSTATE                         (1 << 2)

#define APICMD_SET_REP_EVT_RES_OK                           (0)
#define APICMD_SET_REP_EVT_RES_ERR                          (1)

#define APICMD_REPORT_EVT_TYPE_LTIME                        (0)
#define APICMD_REPORT_EVT_TYPE_SIMD                         (1)
#define APICMD_REPORT_EVT_TYPE_SIMSTATE                     (2)

#define APICMD_REPORT_EVT_SIMD_REMOVAL                      (0)
#define APICMD_REPORT_EVT_SIMD_INSERTION                    (1)

#define APICMD_REPORT_EVT_SIMSTATE_DEACTIVATED              (0)
#define APICMD_REPORT_EVT_SIMSTATE_SIM_INIT_WAIT_PIN_UNLOCK (1)
#define APICMD_REPORT_EVT_SIMSTATE_PERSONALIZATION_FAILED   (2)
#define APICMD_REPORT_EVT_SIMSTATE_ACTIVATION_COMPLETED     (3)

#define APICMD_REPNETINFO_RES_OK                            (0)
#define APICMD_REPNETINFO_RES_ERR                           (1)

#define APICMD_REPNETINFO_REPORT_ENABLE                     (0)
#define APICMD_REPNETINFO_REPORT_DISABLE                    (1)

#define APICMD_SET_REPQUALITY_DISABLE                       (0)
#define APICMD_SET_REPQUALITY_ENABLE                        (1)

#define APICMD_SETPINCODE_PINCODE_LEN                        9

#define APICMD_SETPINCODE_CHGTYPE_PIN                       (1)
#define APICMD_SETPINCODE_CHGTYPE_PIN2                      (2)

#define APICMD_SETPINLOCK_PINCODE_LEN                        9

#define APICMD_FW_INJECTDATA_MAXLEN                         (4096)
#define APICMD_FW_INJECTDATA_MAXLEN_V4                      (3000)

#define APICMD_IPV4_LEN                                     (4)
#define APICMD_IPV6_LEN                                     (16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* structure for common */

begin_packed_struct struct apicmd_ipaddr_s
{
  uint8_t iptype;
  uint8_t address[LTE_IPADDR_MAX_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_pdnset_s
{
  uint8_t session_id;
  uint8_t activate;
  uint32_t apntype;
  uint8_t ipaddr_num;
  struct apicmd_ipaddr_s
    ip_address[LTE_PDN_IPADDR_MAX_COUNT];
  uint8_t imsregister;
  uint8_t dataallow;
  uint8_t dararoamingallow;
} end_packed_struct;

begin_packed_struct struct apicmd_pdnset_v4_s
{
  uint8_t session_id;
  uint8_t activate;
  uint32_t apntype;
  uint8_t ipaddr_num;
  struct apicmd_ipaddr_s
    ip_address[LTE_PDN_IPADDR_MAX_COUNT];
  uint8_t imsregister;
  uint8_t dataallow;
  uint8_t dararoamingallow;
  uint8_t dnsaddr_num;
  struct apicmd_ipaddr_s
    dns_address[APICMD_PDN_DNSCOUNT_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_neighbor_cell_s
{
  uint8_t  valid;
  uint32_t cell_id;
  uint32_t earfcn;
  uint16_t sfn;
  int16_t  rsrp;
  int16_t  rsrq;
} end_packed_struct;

begin_packed_struct struct apicmd_edrxset_s
{
  uint8_t acttype;
  uint8_t enable;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_ltime_s
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minutes;
  uint8_t seconds;
  int32_t timezone;
} end_packed_struct;

begin_packed_struct struct apicmd_netinfo_rejectcause_s
{
  uint8_t category;
  uint8_t value;
} end_packed_struct;

begin_packed_struct struct apicmd_netinfo_nwerrinfo_s
{
  uint8_t err_type;
  struct apicmd_netinfo_rejectcause_s reject_cause;
} end_packed_struct;

begin_packed_struct struct apicmd_netinfo_s
{
  uint8_t nw_stat;
  uint8_t pdn_count;
  struct apicmd_netinfo_nwerrinfo_s err_info;
  struct apicmd_pdnset_s pdn[APICMD_NETINFO_PDNCOUNT_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_netinfo_v4_s
{
  uint8_t nw_stat;
  uint8_t pdn_count;
  struct apicmd_netinfo_nwerrinfo_s err_info;
  struct apicmd_pdnset_v4_s pdn[APICMD_NETINFO_PDNCOUNT_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_psm_timeval_s
{
  uint8_t unit;
  uint8_t time_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_psm_set_s
{
  uint8_t enable;
  struct apicmd_cmddat_psm_timeval_s rat_time;
  struct apicmd_cmddat_psm_timeval_s tau_time;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_quality_s
{
  uint8_t enability;
  int16_t rsrp;
  int16_t rsrq;
  int16_t sinr;
  int16_t rssi;
} end_packed_struct;

/* structure for APICMDID_ACTIVATE_PDN */

begin_packed_struct struct apicmd_cmddat_activatepdn_s
{
  uint32_t apntype;
  uint8_t iptype;
  uint8_t apnname[LTE_APN_LEN];
  uint8_t authtype;
  uint8_t username[LTE_APN_USER_NAME_LEN];
  uint8_t password[LTE_APN_PASSWD_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_activatepdnres_s
{
  uint8_t result;
  struct apicmd_pdnset_s pdnset;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_activatepdnres_v4_s
{
  uint8_t result;
  struct apicmd_pdnset_v4_s pdnset;
} end_packed_struct;

/* structure for APICMDID_ACTIVATE_PDN_CANCEL */

begin_packed_struct struct apicmd_cmddat_activatepdn_cancel_res_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_GET_CELLINFO */

begin_packed_struct struct apicmd_cmddat_cellinfo_s
{
  uint8_t  valid;
  uint32_t cell_id;
  uint32_t earfcn;
  uint8_t  mcc[LTE_MCC_DIGIT];
  uint8_t  mnc_digit;
  uint8_t  mnc[LTE_MNC_DIGIT_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_cellinfo_v4_s
{
  uint8_t enability;
  uint32_t cell_id;
  uint32_t earfcn;
  uint8_t mcc[LTE_MCC_DIGIT];
  uint8_t mnc_digit;
  uint8_t mnc[LTE_MNC_DIGIT_MAX];
  uint8_t cgid[APICMD_CELLINFO_GCID_MAX + 1];
  uint16_t tac;
  uint16_t time_diffidx;
  uint16_t ta;
  uint16_t sfn;
  int16_t rsrp;
  int16_t rsrq;
  uint8_t neighbor_num;
  struct apicmd_cmddat_neighbor_cell_s
           neighbor_cell[LTE_NEIGHBOR_CELL_MAX];
} end_packed_struct;

/* structure for APICMDID_DATA_ALLOW */

begin_packed_struct struct apicmd_cmddat_dataallow_s
{
  uint8_t session_id;
  uint8_t data_allow;
  uint8_t dataroam_allow;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_dataallowres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_DEACTIVATE_PDN */

begin_packed_struct struct apicmd_cmddat_deactivatepdn_s
{
  uint8_t session_id;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_deactivatepdnres_s
{
  uint8_t result;
  uint8_t errcause;
} end_packed_struct;

/* structure for APICMDID_ENTER_PIN */

begin_packed_struct struct apicmd_cmddat_enterpin_s
{
  uint8_t pincode[APICMD_ENTERPIN_PINCODE_LEN];
  uint8_t newpincodeuse;
  uint8_t newpincode[APICMD_ENTERPIN_PINCODE_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_enterpinres_s
{
  uint8_t result;
  uint8_t simstat;
  uint8_t attemptsleft;
} end_packed_struct;

/* structure for APICMDID_ERRINFO */

begin_packed_struct struct apicmd_cmddat_errinfo_s
{
  uint8_t indicator;
  int32_t err_code;
  int32_t err_no;
  uint8_t err_str[APICMD_ERRINFO_ERRSTR_MAX_LEN];
} end_packed_struct;

/* structure for APICMDID_GET_CE */

begin_packed_struct struct apicmd_cmddat_getceres_s
{
  uint8_t result;
  uint8_t mode_a_enable;
  uint8_t mode_b_enable;
} end_packed_struct;

/* structure for APICMDID_GET_CELLINFO */

begin_packed_struct struct apicmd_cmddat_getcellinfores_v4_s
{
  struct apicmd_cmddat_cellinfo_v4_s cellinfo;
} end_packed_struct;

/* structure for APICMDID_GET_DYNAMICEDRX */

begin_packed_struct struct apicmd_cmddat_getdynamicedrxres_s
{
  uint8_t result;
  struct apicmd_edrxset_s set;
} end_packed_struct;

/* structure for APICMDID_GET_DYNAMICPSM */

begin_packed_struct struct apicmd_cmddat_getdynamicpsmres_s
{
  uint8_t result;
  struct apicmd_cmddat_psm_set_s set;
} end_packed_struct;

/* structure for APICMDID_GET_EDRX */

begin_packed_struct struct apicmd_cmddat_getedrx_v4_s
{
  uint8_t type;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getedrxres_s
{
  uint8_t result;
  struct apicmd_edrxset_s set;
} end_packed_struct;

/* structure for APICMDID_GET_IMS_CAP */

begin_packed_struct struct apicmd_cmddat_getimscapres_s
{
  uint8_t result;
  uint8_t ims_cap;
} end_packed_struct;

/* structure for APICMDID_GET_NETINFO */

begin_packed_struct struct apicmd_cmddat_getnetinfores_s
{
  uint8_t result;
  struct apicmd_netinfo_s netinfo;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getnetinfores_v4_s
{
  uint8_t result;
  struct apicmd_netinfo_v4_s netinfo;
} end_packed_struct;

/* structure for APICMDID_GET_PINSET */

begin_packed_struct struct apicmd_cmddat_getpinsetres_s
{
  uint8_t result;
  uint8_t active;
  uint8_t status;
  uint8_t pin_attemptsleft;
  uint8_t puk_attemptsleft;
  uint8_t pin2_attemptsleft;
  uint8_t puk2_attemptsleft;
} end_packed_struct;

/* structure for APICMDID_GET_PSM */

begin_packed_struct struct apicmd_cmddat_getpsm_v4_s
{
  uint8_t type;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getpsmres_s
{
  uint8_t result;
  struct apicmd_cmddat_psm_set_s set;
} end_packed_struct;

/* structure for APICMDID_GET_QUALITY */

begin_packed_struct struct apicmd_cmddat_getqualityres_s
{
  uint8_t result;
  struct apicmd_cmddat_quality_s quality;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getqualityres_v4_s
{
  struct apicmd_cmddat_quality_s quality;
} end_packed_struct;

/* structure for APICMDID_GET_SIMINFO */

begin_packed_struct struct apicmd_cmddat_getsiminfo_s
{
  uint32_t option;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getsiminfo_res_s
{
  uint8_t  result;
  uint32_t option;
  uint8_t  mcc[LTE_MCC_DIGIT];
  uint8_t  mnc_digit;
  uint8_t  mnc[LTE_MNC_DIGIT_MAX];
  uint8_t  spn_len;
  uint8_t  spn[LTE_SIMINFO_SPN_LEN];
  uint8_t  iccid_len;
  uint8_t  iccid[LTE_SIMINFO_ICCID_LEN];
  uint8_t  imsi_len;
  uint8_t  imsi[LTE_SIMINFO_IMSI_LEN];
  uint8_t  gid1_len;
  uint8_t  gid1[LTE_SIMINFO_GID_LEN];
  uint8_t  gid2_len;
  uint8_t  gid2[LTE_SIMINFO_GID_LEN];
} end_packed_struct;

/* structure for APICMDID_GET_IMEI */

begin_packed_struct struct apicmd_cmddat_getimeires_s
{
  uint8_t result;
  uint8_t imei[LTE_IMEI_LEN];
} end_packed_struct;

/* structure for APICMDID_GET_IMSI */

begin_packed_struct struct apicmd_cmddat_getimsires_s
{
  uint8_t result;
  uint8_t errcause;
  uint8_t imsi[LTE_IMSI_LEN];
} end_packed_struct;

/* structure for APICMDID_GET_LTIME */

begin_packed_struct struct apicmd_cmddat_getltimeres_s
{
  uint8_t result;
  struct apicmd_cmddat_ltime_s ltime;
} end_packed_struct;

/* structure for APICMDID_GET_OPERATOR */

begin_packed_struct struct apicmd_cmddat_getoperatorres_s
{
  uint8_t result;
  uint8_t oper[LTE_OPERATOR_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getoperatorres_v4_s
{
  uint8_t result;
  uint8_t oper[APICMD_OPERATOR_LEN_V4];
} end_packed_struct;

/* structure for APICMDID_GET_PHONENO */

begin_packed_struct struct apicmd_cmddat_phonenores_s
{
  uint8_t result;
  uint8_t errcause;
  uint8_t phoneno[LTE_PHONENO_LEN];
} end_packed_struct;

/* structure for APICMDID_RADIO_OFF */

begin_packed_struct struct apicmd_cmddat_radiooffres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_RADIO_ON */

begin_packed_struct struct apicmd_cmddat_radioonres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_SET_RAT */

begin_packed_struct struct apicmd_cmddat_setrat_s
{
  uint8_t rat;
  uint8_t persistency;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setratres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_GET_RAT */

begin_packed_struct struct apicmd_cmddat_getratres_s
{
  uint8_t result;
  uint8_t rat;
  uint8_t rat_mode;
  uint8_t source;
} end_packed_struct;

/* structure for APICMDID_SET_REP_CELLINFO */

begin_packed_struct struct apicmd_cmddat_setrepcellinfo_s
{
  uint8_t enability;
  uint32_t interval;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepcellinfo_res_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_SET_REP_EVT */

begin_packed_struct struct apicmd_cmddat_setrepevt_s
{
  uint8_t event;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevt_v4_s
{
  uint16_t event;
  uint8_t enability;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevtres_s
{
  uint8_t result;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevtres_v4_s
{
  uint8_t result;
  uint16_t event;
} end_packed_struct;

/* structure for APICMDID_REPORT_EVT */

begin_packed_struct struct apicmd_cmddat_repevt_simd_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_simstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repregstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_reppsmstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repantitamper_s
{
  uint8_t data;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repdynpsm_s
{
  struct apicmd_cmddat_psm_timeval_s at_val;
  struct apicmd_cmddat_psm_timeval_s tau_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repdynedrx_s
{
  uint8_t acttype;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repconnphase_s
{
  uint8_t state;
  uint8_t rat;
  uint8_t scantype;
  uint8_t scanreason;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_s
{
  uint8_t type;
  union
    {
      struct apicmd_cmddat_ltime_s ltime;
      struct apicmd_cmddat_repevt_simd_s simd;
      struct apicmd_cmddat_repevt_simstate_s simstate;
    } u;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_v4_s
{
  uint16_t type;
  union
    {
      struct apicmd_cmddat_ltime_s ltime;
      struct apicmd_cmddat_repevt_simd_s simd;
      struct apicmd_cmddat_repevt_simstate_s simstate;
      struct apicmd_cmddat_repregstate_s regstate;
      struct apicmd_cmddat_reppsmstate_s psmstate;
      struct apicmd_cmddat_repdynpsm_s dynpsm;
      struct apicmd_cmddat_repdynedrx_s dynedrx;
      struct apicmd_cmddat_repconnphase_s connphase;
      struct apicmd_cmddat_repantitamper_s antitamper;
    } u;
} end_packed_struct;

/* structure for APICMDID_SETREP_NETINFO */

begin_packed_struct struct apicmd_cmddat_set_repnetinfo_s
{
  uint8_t report;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_set_repnetinfores_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_REPORT_NETINFO */

begin_packed_struct struct apicmd_cmddat_rep_netinfo_s
{
  struct apicmd_netinfo_s netinfo;
  uint8_t dnsaddrv4[APICMD_IPV4_LEN];
  uint8_t dnsaddrv6[APICMD_IPV6_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_rep_netinfo_v4_s
{
  struct apicmd_netinfo_v4_s netinfo;
} end_packed_struct;

/* structure for APICMDID_SET_REP_QUALITY */

begin_packed_struct struct apicmd_cmddat_setrepquality_s
{
  uint8_t enability;
  uint32_t interval;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepquality_res_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_SET_CE */

begin_packed_struct struct apicmd_cmddat_setce_s
{
  uint8_t mode_a_enable;
  uint8_t mode_b_enable;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setceres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_SET_EDRX */

begin_packed_struct struct apicmd_cmddat_setedrx_s
{
  uint8_t acttype;
  uint8_t enable;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setedrxres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_SET_PIN_CODE */

begin_packed_struct struct apicmd_cmddat_setpincode_s
{
  uint8_t chgtype;
  uint8_t pincode[APICMD_SETPINCODE_PINCODE_LEN];
  uint8_t newpincode[APICMD_SETPINCODE_PINCODE_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setpincoderes_s
{
  uint8_t result;
  uint8_t attemptsleft;
} end_packed_struct;

/* structure for APICMDID_SET_PIN_LOCK */

begin_packed_struct struct apicmd_cmddat_setpinlock_s
{
  uint8_t mode;
  uint8_t pincode[APICMD_SETPINLOCK_PINCODE_LEN];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setpinlockres_s
{
  uint8_t result;
  uint8_t attemptsleft;
} end_packed_struct;

/* structure for APICMDID_SET_PSM */

begin_packed_struct struct apicmd_cmddat_setpsm_s
{
  struct apicmd_cmddat_psm_set_s set;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setpsmres_s
{
  uint8_t result;
} end_packed_struct;

/* structure for APICMDID_GET_VERSION */

begin_packed_struct struct apicmd_cmddat_getverres_s
{
  uint8_t result;
  uint8_t bb_product[LTE_VER_BB_PRODUCT_LEN];
  uint8_t np_package[LTE_VER_NP_PACKAGE_LEN];
} end_packed_struct;

/* structure for APICMDID_FW_INJECTDELTAIMG */

begin_packed_struct struct apicmd_cmddat_fw_injectdeltaimg_s
{
  uint8_t  data[APICMD_FW_INJECTDATA_MAXLEN];
  uint32_t data_len;
  uint8_t  inject_mode;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_fw_injectdeltaimg_v4_s
{
  uint8_t  inject_mode;
  uint32_t data_len;

  /* Variable length array */

  uint8_t  data[1];
} end_packed_struct;

/* structure for APICMDID_FW_INJECTDELTAIMG_RES,
 * APICMDID_FW_GETDELTAIMGLEN_RES, APICMDID_FW_EXECDELTAUPDATE_RES,
 * APICMDID_FW_GETUPDATERESULT_RES
 */

begin_packed_struct struct apicmd_cmddat_fw_deltaupcommres_s
{
  int32_t  api_result;
  uint16_t ltefw_result;
} end_packed_struct;

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_H */
