/****************************************************************************
 * include/nuttx/wireless/lte/lte.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LTE_LTE_H
#define __INCLUDE_NUTTX_WIRELESS_LTE_LTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_RESULT_OK     (0)      /* Result code on success */
#define LTE_RESULT_ERROR  (1)      /* Result code on failure */
#define LTE_RESULT_CANCEL (2)      /* Result code on cancel */

#define LTE_VALID   (true)         /* Valid */
#define LTE_INVALID (false)        /* Invalid */

#define LTE_ENABLE  (true)         /* Enable */
#define LTE_DISABLE (false)        /* Disable */

#define LTE_ERR_WAITENTERPIN (1)   /* Waiting for PIN enter */
#define LTE_ERR_REJECT       (2)   /* Rejected from the network */
#define LTE_ERR_MAXRETRY     (3)   /* No response from the network */
#define LTE_ERR_BARRING      (4)   /* Network barring */
#define LTE_ERR_DETACHED     (5)   /* Network detached */
#define LTE_ERR_UNEXPECTED   (255) /* Unexpected cause */

#define LTE_SESSION_ID_MIN (1)     /* Minimum value of session ID */
#define LTE_SESSION_ID_MAX (5)     /* Maximum value of session ID */

/* Network status: Not registered, MT is not currently searching
 *                 a new operator to register to
 */

#define LTE_NETSTAT_NOT_REG_NOT_SEARCHING     (0)

/* Network status: Registered, home network */

#define LTE_NETSTAT_REG_HOME                  (1)

/* Network status: Not registered, but MT is currently searching
 *                 a new operator to register to
 */

#define LTE_NETSTAT_NOT_REG_SEARCHING         (2)

/* Network status: Registration denied */

#define LTE_NETSTAT_REG_DENIED                (3)

/* Network status: Unknown */

#define LTE_NETSTAT_UNKNOWN                   (4)

/* Network status: Registered, roaming */

#define LTE_NETSTAT_REG_ROAMING               (5)

/* Network status: Registered for "SMS only", home network */

#define LTE_NETSTAT_REG_SMS_ONLY_HOME         (6)

/* Network status: Registered for "SMS only", roaming */

#define LTE_NETSTAT_REG_SMS_ONLY_ROAMING      (7)

/* Network status: Attached for emergency bearer services only */

#define LTE_NETSTAT_NOT_REG_EMERGENCY         (8)

/* Network status: Registered for "CSFB not preferred", home network */

#define LTE_NETSTAT_REG_CSFB_NOT_PREF_HOME    (9)

/* Network status: Registered for "CSFB not preferred", roaming */

#define LTE_NETSTAT_REG_CSFB_NOT_PREF_ROAMING (10)

/* The maximum string length of the APN name */

#define LTE_APN_LEN           (101)

/* The maximum string length of the APN user name */

#define LTE_APN_USER_NAME_LEN (64)

/* The maximum string length of the APN password */

#define LTE_APN_PASSWD_LEN    (32)

#define LTE_IPTYPE_V4         (0) /* Internet protocol type: IPv4 */
#define LTE_IPTYPE_V6         (1) /* Internet protocol type: IPv6 */
#define LTE_IPTYPE_V4V6       (2) /* Internet protocol type: IPv4/v6 */
#define LTE_IPTYPE_NON        (3) /* Internet protocol type:
                                   * Non-IP Data Delivery
                                   */

/* Internet protocol type: IP
 * deprecated. Use LTE_IPTYPE_V4 instead.
 */

#define LTE_APN_IPTYPE_IP     LTE_IPTYPE_V4

/* Internet protocol type: IPv6
 * deprecated. Use LTE_IPTYPE_V6 instead.
 */

#define LTE_APN_IPTYPE_IPV6   LTE_IPTYPE_V6

/* Internet protocol type: IPv4/v6
 * deprecated. Use LTE_IPTYPE_V4V6 instead.
 */

#define LTE_APN_IPTYPE_IPV4V6 LTE_IPTYPE_V4V6

#define LTE_APN_AUTHTYPE_NONE (0) /* PPP authentication type: NONE */
#define LTE_APN_AUTHTYPE_PAP  (1) /* PPP authentication type: PAP */
#define LTE_APN_AUTHTYPE_CHAP (2) /* PPP authentication type: CHAP */

/* APN type: Unknown */

#define LTE_APN_TYPE_UNKNOWN   (0x01)

/* APN type: Default data traffic */

#define LTE_APN_TYPE_DEFAULT   (0x02)

/* APN type: MMS traffic(Multimedia Messaging Service) */

#define LTE_APN_TYPE_MMS       (0x04)

/* APN type: SUPL assisted GPS */

#define LTE_APN_TYPE_SUPL      (0x08)

/* APN type: DUN traffic(Dial Up Networking bridge ) */

#define LTE_APN_TYPE_DUN       (0x10)

/* APN type: HiPri traffic(High Priority Mobile data) */

#define LTE_APN_TYPE_HIPRI     (0x20)

/* APN type: FOTA(Firmware On The Air) */

#define LTE_APN_TYPE_FOTA      (0x40)

/* APN type: IMS(IP Multimedia Subsystem) */

#define LTE_APN_TYPE_IMS       (0x80)

/* APN type: CBS(Carrier Branded Services) */

#define LTE_APN_TYPE_CBS       (0x100)

/* APN type: IA(Initial Attach APN) */

#define LTE_APN_TYPE_IA        (0x200)

/* APN type: Emergency PDN */

#define LTE_APN_TYPE_EMERGENCY (0x400)

/* Network error type: MAX_RETRY */

#define LTE_NETERR_MAXRETRY    (0)

/* Network error type: REJECT */

#define LTE_NETERR_REJECT      (1)

/* Network error type: Network Detach */

#define LTE_NETERR_NWDTCH      (2)

/* Network reject category: NAS-EMM */

#define LTE_REJECT_CATEGORY_EMM  (0)

/* Network reject category: NAS-ESM */

#define LTE_REJECT_CATEGORY_ESM  (1)

/* Length of character string for BB product */

#define LTE_VER_BB_PRODUCT_LEN (5)

/* Length of character string for NP package */

#define LTE_VER_NP_PACKAGE_LEN (32)

/* Enable setting of PIN lock
 * deprecated. Use LTE_ENABLE instead.
 */

#define LTE_PIN_ENABLE  LTE_ENABLE

/* Disable setting of PIN lock
 * deprecated. Use LTE_DISABLE instead.
 */

#define LTE_PIN_DISABLE LTE_DISABLE

/* PIN status: Not pending for any password */

#define LTE_PINSTAT_READY         (0)

/* PIN status: Waiting SIM PIN to be given */

#define LTE_PINSTAT_SIM_PIN       (1)

/* PIN status: Waiting SIM PUK to be given */

#define LTE_PINSTAT_SIM_PUK       (2)

/* PIN status: Waiting phone to SIM card password to be given */

#define LTE_PINSTAT_PH_SIM_PIN    (3)

/* PIN status: Waiting phone-to-very first SIM card password to be given */

#define LTE_PINSTAT_PH_FSIM_PIN   (4)

/* PIN status: Waiting phone-to-very first SIM card unblocking
 *             password to be given
 */

#define LTE_PINSTAT_PH_FSIM_PUK   (5)

/* PIN status: Waiting SIM PIN2 to be given */

#define LTE_PINSTAT_SIM_PIN2      (6)

/* PIN status: Waiting SIM PUK2 to be given */

#define LTE_PINSTAT_SIM_PUK2      (7)

/* PIN status: Waiting network personalization password to be given */

#define LTE_PINSTAT_PH_NET_PIN    (8)

/* PIN status: Waiting network personalization unblocking password
 *             to be given
 */

#define LTE_PINSTAT_PH_NET_PUK    (9)

/* PIN status: Waiting network subset personalization password to be given */

#define LTE_PINSTAT_PH_NETSUB_PIN (10)

/* PIN status: Waiting network subset personalization unblocking password
 *             to be given
 */

#define LTE_PINSTAT_PH_NETSUB_PUK (11)

/* PIN status: Waiting service provider personalization password
 *             to be given
 */

#define LTE_PINSTAT_PH_SP_PIN     (12)

/* PIN status: Waiting service provider personalization unblocking password
 *             to be given
 */

#define LTE_PINSTAT_PH_SP_PUK     (13)

/* PIN status: Waiting corporate personalization password to be given */

#define LTE_PINSTAT_PH_CORP_PIN   (14)

/* PIN status: Waiting corporate personalization unblocking password
 *             to be given
 */

#define LTE_PINSTAT_PH_CORP_PUK   (15)

#define LTE_TARGET_PIN  (0)  /* Select of PIN change */
#define LTE_TARGET_PIN2 (1)  /* Select of PIN2 change */

/* SIM status: SIM removal signal detected */

#define LTE_SIMSTAT_REMOVAL         (0)

/* SIM status: SIM insertion signal detected */

#define LTE_SIMSTAT_INSERTION       (1)

/* SIM status: SIM init passed, wait for PIN unlock */

#define LTE_SIMSTAT_WAIT_PIN_UNLOCK (2)

/* SIM status: Personalization failed, wait for run-time depersonalization */

#define LTE_SIMSTAT_PERSONAL_FAILED (3)

/* SIM status: Activation completed. Event is sent always
 *             at any SIM activation completion
 */

#define LTE_SIMSTAT_ACTIVATE        (4)

/* SIM status: SIM is deactivated */

#define LTE_SIMSTAT_DEACTIVATE      (5)

#define LTE_MCC_DIGIT     (3)  /* Digit number of Mobile Country Code */
#define LTE_MNC_DIGIT_MAX (3)  /* Max digit number of Mobile Network Code */

/* Digit number of mcc
 * deprecated. Use LTE_MCC_DIGIT instead.
 */

#define LTE_CELLINFO_MCC_DIGIT     LTE_MCC_DIGIT

/* Max digit number of mnc
 * deprecated. Use LTE_MNC_DIGIT_MAX instead.
 */

#define LTE_CELLINFO_MNC_DIGIT_MAX LTE_MNC_DIGIT_MAX

#define LTE_EDRX_ACTTYPE_WBS1     (0) /* E-UTRAN (WB-S1 mode)   */
#define LTE_EDRX_ACTTYPE_NBS1     (1) /* E-UTRAN (NB-S1 mode)   */
#define LTE_EDRX_ACTTYPE_ECGSMIOT (2) /* EC-GSM-IoT (A/Gb mode) */
#define LTE_EDRX_ACTTYPE_GSM      (3) /* GSM (A/Gb mode)        */
#define LTE_EDRX_ACTTYPE_IU       (4) /* UTRAN (Iu mode)        */
#define LTE_EDRX_ACTTYPE_NOTUSE   (5) /* eDRX is not running    */

#define LTE_EDRX_CYC_512      (0) /* eDRX cycle:    5.12 sec */
#define LTE_EDRX_CYC_1024     (1) /* eDRX cycle:   10.24 sec */
#define LTE_EDRX_CYC_2048     (2) /* eDRX cycle:   20.48 sec */
#define LTE_EDRX_CYC_4096     (3) /* eDRX cycle:   40.96 sec */
#define LTE_EDRX_CYC_6144     (4) /* eDRX cycle:   61.44 sec */
#define LTE_EDRX_CYC_8192     (5) /* eDRX cycle:   81.92 sec */
#define LTE_EDRX_CYC_10240    (6) /* eDRX cycle:  102.40 sec */
#define LTE_EDRX_CYC_12288    (7) /* eDRX cycle:  122.88 sec */
#define LTE_EDRX_CYC_14336    (8) /* eDRX cycle:  143.36 sec */
#define LTE_EDRX_CYC_16384    (9) /* eDRX cycle:  163.84 sec */
#define LTE_EDRX_CYC_32768   (10) /* eDRX cycle:  327.68 sec */
#define LTE_EDRX_CYC_65536   (11) /* eDRX cycle:  655.36 sec */
#define LTE_EDRX_CYC_131072  (12) /* eDRX cycle: 1310.72 sec */
#define LTE_EDRX_CYC_262144  (13) /* eDRX cycle: 2621.44 sec */
#define LTE_EDRX_CYC_524288  (14) /* eDRX cycle: 5242.88 sec */
#define LTE_EDRX_CYC_1048576 (15) /* eDRX cycle: 10485.76 sec */
#define LTE_EDRX_PTW_128      (0) /* Paging time window:  1.28 sec */
#define LTE_EDRX_PTW_256      (1) /* Paging time window:  2.56 sec */
#define LTE_EDRX_PTW_384      (2) /* Paging time window:  3.84 sec */
#define LTE_EDRX_PTW_512      (3) /* Paging time window:  5.12 sec */
#define LTE_EDRX_PTW_640      (4) /* Paging time window:  6.40 sec */
#define LTE_EDRX_PTW_768      (5) /* Paging time window:  7.68 sec */
#define LTE_EDRX_PTW_896      (6) /* Paging time window:  8.96 sec */
#define LTE_EDRX_PTW_1024     (7) /* Paging time window: 10.24 sec */
#define LTE_EDRX_PTW_1152     (8) /* Paging time window: 11.52 sec */
#define LTE_EDRX_PTW_1280     (9) /* Paging time window: 12.80 sec */
#define LTE_EDRX_PTW_1408    (10) /* Paging time window: 14.08 sec */
#define LTE_EDRX_PTW_1536    (11) /* Paging time window: 15.36 sec */
#define LTE_EDRX_PTW_1664    (12) /* Paging time window: 16.64 sec */
#define LTE_EDRX_PTW_1792    (13) /* Paging time window: 17.92 sec */
#define LTE_EDRX_PTW_1920    (14) /* Paging time window: 19.20 sec */
#define LTE_EDRX_PTW_2048    (15) /* Paging time window: 20.48 sec */
#define LTE_EDRX_PTW_2304    (16) /* Paging time window: 23.04 sec */
#define LTE_EDRX_PTW_2560    (17) /* Paging time window: 25.60 sec */
#define LTE_EDRX_PTW_2816    (18) /* Paging time window: 28.16 sec */
#define LTE_EDRX_PTW_3072    (19) /* Paging time window: 30.72 sec */
#define LTE_EDRX_PTW_3328    (20) /* Paging time window: 33.28 sec */
#define LTE_EDRX_PTW_3584    (21) /* Paging time window: 35.84 sec */
#define LTE_EDRX_PTW_3840    (22) /* Paging time window: 38.40 sec */
#define LTE_EDRX_PTW_4096    (23) /* Paging time window: 40.96 sec */

/* Unit of request active time(T3324): 2 sec */

#define LTE_PSM_T3324_UNIT_2SEC    (0)

/* Unit of request active time(T3324): 1 min */

#define LTE_PSM_T3324_UNIT_1MIN    (1)

/* Unit of request active time(T3324): 6 min */

#define LTE_PSM_T3324_UNIT_6MIN    (2)

/* Unit of request active time(T3324): The value indicates that
 * the timer is deactivated.
 */

#define LTE_PSM_T3324_UNIT_DEACT   (3)

/* Unit of extended periodic TAU time(T3412): 2 sec */

#define LTE_PSM_T3412_UNIT_2SEC    (0)

/* Unit of extended periodic TAU time(T3412): 30 sec */

#define LTE_PSM_T3412_UNIT_30SEC   (1)

/* Unit of extended periodic TAU time(T3412): 1 min */

#define LTE_PSM_T3412_UNIT_1MIN    (2)

/* Unit of extended periodic TAU time(T3412): 10 min */

#define LTE_PSM_T3412_UNIT_10MIN   (3)

/* Unit of extended periodic TAU time(T3412): 1 hour */

#define LTE_PSM_T3412_UNIT_1HOUR   (4)

/* Unit of extended periodic TAU time(T3412): 10 hour */

#define LTE_PSM_T3412_UNIT_10HOUR  (5)

/* Unit of extended periodic TAU time(T3412): 320 hour */

#define LTE_PSM_T3412_UNIT_320HOUR (6)

/* Unit of extended periodic TAU time(T3412): The value indicates that
 * the timer is deactivated.
 */

#define LTE_PSM_T3412_UNIT_DEACT   (7)

/* The minimum timer value used by PSM related timers */

#define LTE_PSM_TIMEVAL_MIN        (0)

/* The maximum timer value used by PSM related timers */

#define LTE_PSM_TIMEVAL_MAX        (31)

#define LTE_IPADDR_MAX_LEN (40) /* Maximum length of the IP address */

/* Invalid Session ID */

#define LTE_PDN_SESSIONID_INVALID_ID (0)

/* Minimum value of Session ID */

#define LTE_PDN_SESSIONID_MIN        (LTE_PDN_SESSIONID_INVALID_ID)

/* Maximum value of Session ID */

#define LTE_PDN_SESSIONID_MAX        (255)

#define LTE_PDN_DEACTIVE         (0) /* PDN status: Not active */
#define LTE_PDN_ACTIVE           (1) /* PDN status: Active */

#define LTE_PDN_IPADDR_MAX_COUNT (2) /* Maximum number of IP addresses */

#define LTE_IMS_NOT_REGISTERED   (0) /* IMS status: Not registered */
#define LTE_IMS_REGISTERED       (1) /* IMS status: Registered */
#define LTE_DATA_DISALLOW        (0) /* Data communication: Not allow */
#define LTE_DATA_ALLOW           (1) /* Data communication: Allow */

/* Modem restart cause: User initiated */

#define LTE_RESTART_USER_INITIATED  (0)

/* Modem restart cause: Modem initiated */

#define LTE_RESTART_MODEM_INITIATED (1)

/* Error indicator for error code */

#define LTE_ERR_INDICATOR_ERRCODE  (0x01)

/* Error indicator for error number */

#define LTE_ERR_INDICATOR_ERRNO    (0x02)

/* Error indicator for error string */

#define LTE_ERR_INDICATOR_ERRSTR   (0x04)

/* Maximum length of the error string */

#define LTE_ERROR_STRING_MAX_LEN   (64)

/* Indicates to get for Mobile Country Code/Mobile Network Code of SIM */

#define LTE_SIMINFO_GETOPT_MCCMNC (1 << 0)

/* Indicates to get for SPN of SIM */

#define LTE_SIMINFO_GETOPT_SPN    (1 << 1)

/* Indicates to get for ICCID of SIM */

#define LTE_SIMINFO_GETOPT_ICCID  (1 << 2)

/* Indicates to get for IMSI of SIM */

#define LTE_SIMINFO_GETOPT_IMSI   (1 << 3)

/* Indicates to get for GID1(Group Identifier Level 1) of SIM */

#define LTE_SIMINFO_GETOPT_GID1   (1 << 4)

/* Indicates to get for GID2(Group Identifier Level 2) of SIM */

#define LTE_SIMINFO_GETOPT_GID2   (1 << 5)

/* Digit number of mcc
 * deprecated. Use LTE_MCC_DIGIT instead.
 */

#define LTE_SIMINFO_MCC_DIGIT      LTE_MCC_DIGIT

/* Max digit number of mnc
 * deprecated. Use LTE_MNC_DIGIT_MAX instead.
 */

#define LTE_SIMINFO_MNC_DIGIT_MAX  LTE_MNC_DIGIT_MAX

#define LTE_SIMINFO_SPN_LEN   (16)  /* Maximum length of SPN */
#define LTE_SIMINFO_ICCID_LEN (10)  /* Maximum length of ICCCID */
#define LTE_SIMINFO_IMSI_LEN  (15)  /* Maximum length of IMSI */
#define LTE_SIMINFO_GID_LEN   (128) /* Maximum length of GID */

/* Maximum length of phone number
 * that includes a null terminater
 */

#define LTE_PHONENO_LEN  (41)

/* Maximum length of IMEI
 * that includes a null terminater
 */

#define LTE_IMEI_LEN     (16)

/* Maximum length of network operator
 * that includes a null terminater
 */

#define LTE_OPERATOR_LEN (17)

/* Maximum length of IMSI
 * that includes a null terminater
 */

#define LTE_IMSI_LEN (LTE_SIMINFO_IMSI_LEN + 1)

/* Indicates that the global cell ID can be referenced */

#define LTE_CELLINFO_OPT_GCID        (1 << 0)

/* Indicates that the tracking area code can be referenced */

#define LTE_CELLINFO_OPT_AREACODE    (1 << 1)

/* Indicates that the sub frame number can be referenced */

#define LTE_CELLINFO_OPT_SFN         (1 << 2)

/* Indicates that the RSRP can be referenced */

#define LTE_CELLINFO_OPT_RSRP        (1 << 3)

/* Indicates that the RSRQ can be referenced */

#define LTE_CELLINFO_OPT_RSRQ        (1 << 4)

/* Indicates that the time difference index can be referenced */

#define LTE_CELLINFO_OPT_TIMEDIFFIDX (1 << 5)

/* Indicates that the timing advance can be referenced */

#define LTE_CELLINFO_OPT_TA          (1 << 6)

/* Indicates that the neighbor cell can be referenced */

#define LTE_CELLINFO_OPT_NEIGHBOR    (1 << 7)

#define LTE_NEIGHBOR_CELL_MAX (32) /* Maximum number of neighbor cells */

#define LTE_RAT_CATM  (2)     /* RAT type: Cat-M */
#define LTE_RAT_NBIOT (3)     /* RAT type: NB-IoT */

#define LTE_RAT_MODE_SINGLE   (0) /* Modem only supports single RAT  */
#define LTE_RAT_MODE_MULTIPLE (1) /* Modem supports multiple RAT */

/* RAT has not changed since the last modem boot. */

#define LTE_RAT_SOURCE_DEFAULT (0)

/* The current RAT was determined by host. */

#define LTE_RAT_SOURCE_HOST    (1)

/* The current RAT was determined by LWM2M. */

#define LTE_RAT_SOURCE_LWM2M   (2)

/* Maximum length of AT command */

#define LTE_AT_COMMAND_MAX_LEN (2048)

/* Inject delta image from the beginning. */

#define LTEFW_INJECTION_MODE_NEW    (0)

/* Inject delta image from the continuation. */

#define LTEFW_INJECTION_MODE_APPEND (1)

/* LTEFW result code */

/* OK */

#define LTEFW_RESULT_OK                            (0x0000)

/* Not enough space for storage for injection */

#define LTEFW_RESULT_NOT_ENOUGH_INJECTSTORAGE      (-1)

/* CRC check error in header part of delta image */

#define LTEFW_RESULT_DELTAIMAGE_HDR_CRC_ERROR      (-2)

/* Unsupported header type of delta image */

#define LTEFW_RESULT_DELTAIMAGE_HDR_UNSUPPORTED    (-3)

/* Failed to set delta image */

#define LTEFW_RESULT_PRECHK_SET_DELTAIMAGE_FAILED  (-4)

/* Failed to delta update */

#define LTEFW_RESULT_DELTAUPDATE_FAILED            (-5)

/* Not found delta image */

#define LTEFW_RESULT_PRECHK_DELTAIMAGE_MISSING     (-6)

/* Out of memory that prepare for update */

#define LTEFW_RESULT_PRECHK_OOM                    (-7)

/* Invalid size of delta image */

#define LTEFW_RESULT_PRECHK_SIZE_ERROR             (-8)

/* Wrong delta image package */

#define LTEFW_RESULT_PRECHK_PKG_ERROR              (-9)

/* CRC check error in delta image */

#define LTEFW_RESULT_PRECHK_CRC_ERROR              (-10)

/* There is no update result */

#define LTEFW_RESULT_DELTAUPDATE_NORESULT          (-11)

/* Length of LTE modem log file name */

#define LTE_LOG_NAME_LEN  32

/* Number of LTE modem logs saved */

#define LTE_LOG_LIST_SIZE 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Definition of version information of the modem.
 * This is notified by get_ver_cb_t
 */

typedef struct lte_version
{
  /* BB product version. It is terminated with '\0'. */

  char bb_product[LTE_VER_BB_PRODUCT_LEN];

  /* NP package version. It is terminated with '\0'. */

  char np_package[LTE_VER_NP_PACKAGE_LEN];
} lte_version_t;

/* Definition of PIN setting information.
 * This is notified by get_pinset_cb_t
 */

typedef struct lte_getpin
{
  /* PIN enable. Definition is as below.
   *  - LTE_ENABLE
   *  - LTE_DISABLE
   */

  uint8_t enable;

  /* PIN status. Refer to the this parameter only
   * when enable is @ref LTE_ENABLE.
   */

  uint8_t status;

  /* PIN attempts left */

  uint8_t pin_attemptsleft;

  /* PUK attempts left */

  uint8_t puk_attemptsleft;

  /* PIN2 attempts left */

  uint8_t pin2_attemptsleft;

  /* PUK2 attempts left */

  uint8_t puk2_attemptsleft;
} lte_getpin_t;

/* Definition of local time. This is notified by
 * get_localtime_cb_t and localtime_report_cb_t
 */

typedef struct lte_localtime
{
  int32_t year;   /* Years (0-99) */
  int32_t mon;    /* Month (1-12) */
  int32_t mday;   /* Day of the month (1-31) */
  int32_t hour;   /* Hours (0-23) */
  int32_t min;    /* Minutes (0-59) */
  int32_t sec;    /* Seconds (0-59) */
  int32_t tz_sec; /* Time zone in seconds (-86400-86400) */
} lte_localtime_t;

/* Definition of parameters for quality information.
 * This is reported by quality_report_cb_t
 * and notified by get_quality_cb_t
 */

typedef struct lte_quality
{
  /* Valid flag. Definition is as below.
   *  - LTE_VALID
   *  - LTE_INVALID
   *
   *  Refer to the following parameters only when this flag
   *  is LTE_VALID. This is because valid parameters can not be
   *  acquired when RF function is OFF and so on
   */

  bool    valid;

  /* RSRP in dBm (-140-0) */

  int16_t rsrp;

  /* RSRQ in dBm (-60-0) */

  int16_t rsrq;

  /* SINR in dBm (-128-40) */

  int16_t sinr;

  /* RSSI in dBm */

  int16_t rssi;
} lte_quality_t;

/* Definition of parameters for neighbor cell information.
 * This is reported by cellinfo_report_cb_t
 */

typedef struct lte_neighbor_cell
{
  /* Indicates which parameters can be referenced.
   *  Bit setting definition is as below.
   *  - LTE_CELLINFO_OPT_SFN
   *  - LTE_CELLINFO_OPT_RSRP
   *  - LTE_CELLINFO_OPT_RSRQ
   */

  uint8_t option;

  /* Physical cell ID (1-503) */

  uint32_t phycell_id;

  /* EARFCN (1-262143) */

  uint32_t earfcn;

  /* Sub Frame Number (0-1023). It can be referenced when
   * LTE_CELLINFO_OPT_SFN is set in option field.
   */

  uint16_t sfn;

  /* RSRP in dBm (-140-0). It can be referenced when
   * LTE_CELLINFO_OPT_RSRP is set in option field.
   */

  int16_t rsrp;

  /* RSRQ in dBm (-60-0). It can be referenced when
   * LTE_CELLINFO_OPT_RSRQ is set in option field.
   */

  int16_t rsrq;
} lte_neighbor_cell_t;

/* Definition of parameters for cell information.
 * This is reported by cellinfo_report_cb_t
 */

typedef struct lte_cellinfo
{
  /* Valid flag. Definition is as below.
   * - LTE_VALID
   * - LTE_INVALID
   *
   *  Refer to the following parameters only when this flag
   *  is LTE_VALID. This is because valid parameters can not be
   *  acquired when RF function is OFF and so on
   */

  bool     valid;

  /* Physical cell ID (1-503) */

  uint32_t phycell_id;

  /* EARFCN (1-262143) */

  uint32_t earfcn;

  /* Mobile Country Code (000-999) */

  uint8_t  mcc[LTE_MCC_DIGIT];

  /* Digit number of Mobile Network Code(2-3) */

  uint8_t  mnc_digit;

  /* Mobile Network Code (00-999) */

  uint8_t  mnc[LTE_MNC_DIGIT_MAX];

  /* Indicates which parameters can be referenced.
   *  Bit setting definition is as below.
   *  - LTE_CELLINFO_OPT_GCID
   *  - LTE_CELLINFO_OPT_AREACODE
   *  - LTE_CELLINFO_OPT_SFN
   *  - LTE_CELLINFO_OPT_RSRP
   *  - LTE_CELLINFO_OPT_RSRQ
   *  - LTE_CELLINFO_OPT_TIMEDIFFIDX
   *  - LTE_CELLINFO_OPT_TA
   *  - LTE_CELLINFO_OPT_NEIGHBOR
   */

  uint8_t option;

  /* Global Cell ID. It can be referenced when
   * LTE_CELLINFO_OPT_GCID is set in option field.
   */

  uint32_t gcid;

  /* Tracking Area Code (1-65535). It can be referenced when
   *  LTE_CELLINFO_OPT_AREACODE is set in option field.
   */

  uint16_t area_code;

  /* Sub Frame Number (0-1023). It can be referenced when
   * LTE_CELLINFO_OPT_SFN is set in option field.
   */

  uint16_t sfn;

  /* RSRP in dBm (-140-0). It can be referenced when
   * LTE_CELLINFO_OPT_RSRP is set in option field.
   */

  int16_t rsrp;

  /* RSRQ in dBm (-60-0). It can be referenced when
   * LTE_CELLINFO_OPT_RSRQ is set in option field.
   */

  int16_t rsrq;

  /* Time difference index (0-4095). It can be referenced when
   * LTE_CELLINFO_OPT_TIMEDIFFIDX is set in option field.
   */

  uint16_t time_diffidx;

  /* Timing Advance (0-1282). It can be referenced when
   * LTE_CELLINFO_OPT_TA is set in option field.
   */

  uint16_t ta;

  /* Number of neighbor cells (0-32). It can be referenced when
   * LTE_CELLINFO_OPT_NEIGHBOR is set in option field.
   *
   * When using lte_get_cellinfo_sync, need to specify the number
   * of neighbor cells to get. Then, prepare a memory area for
   * the specified number of neighbor cells and set it to
   * neighbors.
   */

  uint8_t nr_neighbor;

  /* Pointer to neighbor cells. It can be referenced when
   * LTE_CELLINFO_OPT_NEIGHBOR is set in option field.
   *
   * When using the lte_get_cellinfo_sync, prepare a memory
   * area for the specified number of neighbor cells.
   */

  lte_neighbor_cell_t *neighbors;
} lte_cellinfo_t;

/* Definition of eDRX settings used in lte_set_edrx().
 * This is notified by get_edrx_cb_t
 */

typedef struct lte_edrx_setting
{
  /* eDRX act type. Definition is as below.
   * - LTE_EDRX_ACTTYPE_WBS1
   * - LTE_EDRX_ACTTYPE_NBS1
   * - LTE_EDRX_ACTTYPE_ECGSMIOT
   * - LTE_EDRX_ACTTYPE_GSM
   * - LTE_EDRX_ACTTYPE_IU
   * - LTE_EDRX_ACTTYPE_NOTUSE
   */

  uint8_t  act_type;

  /* eDRX enable. Definition is as below.
   * - LTE_ENABLE
   * - LTE_DISABLE
   */

  bool     enable;

  /* eDRX cycle.
   * This variable is not vaild when LTE_EDRX_ACTTYPE_NOTUSE
   * is set to act_type.
   * Definitions are below:
   * - LTE_EDRX_CYC_512
   * - LTE_EDRX_CYC_1024
   * - LTE_EDRX_CYC_2048
   * - LTE_EDRX_CYC_4096
   * - LTE_EDRX_CYC_6144
   * - LTE_EDRX_CYC_8192
   * - LTE_EDRX_CYC_10240
   * - LTE_EDRX_CYC_12288
   * - LTE_EDRX_CYC_14336
   * - LTE_EDRX_CYC_16384
   * - LTE_EDRX_CYC_32768
   * - LTE_EDRX_CYC_65536
   * - LTE_EDRX_CYC_131072
   * - LTE_EDRX_CYC_262144
   */

  uint32_t edrx_cycle;

  /* Paging time window.
   * This variable is not vaild when LTE_EDRX_ACTTYPE_NOTUSE
   * is set to act_type.
   * Definitions are below:
   * - LTE_EDRX_PTW_128
   * - LTE_EDRX_PTW_256
   * - LTE_EDRX_PTW_384
   * - LTE_EDRX_PTW_512
   * - LTE_EDRX_PTW_640
   * - LTE_EDRX_PTW_768
   * - LTE_EDRX_PTW_896
   * - LTE_EDRX_PTW_1024
   * - LTE_EDRX_PTW_1152
   * - LTE_EDRX_PTW_1280
   * - LTE_EDRX_PTW_1408
   * - LTE_EDRX_PTW_1536
   * - LTE_EDRX_PTW_1664
   * - LTE_EDRX_PTW_1792
   * - LTE_EDRX_PTW_1920
   * - LTE_EDRX_PTW_2048
   */

  uint32_t ptw_val;
} lte_edrx_setting_t;

/* Definition of timer information for PSM
 */

typedef struct lte_psm_timeval
{
  /* Unit of timer value. Definition is as below.
   * - When kind of timer is Requested Active Time
   *  - LTE_PSM_T3324_UNIT_2SEC
   *  - LTE_PSM_T3324_UNIT_1MIN
   *  - LTE_PSM_T3324_UNIT_6MIN
   *  - LTE_PSM_T3324_UNIT_DEACT
   *
   * - When kind of timer is Extended periodic TAU Time
   *  - LTE_PSM_T3412_UNIT_2SEC
   *  - LTE_PSM_T3412_UNIT_30SEC
   *  - LTE_PSM_T3412_UNIT_1MIN
   *  - LTE_PSM_T3412_UNIT_10MIN
   *  - LTE_PSM_T3412_UNIT_1HOUR
   *  - LTE_PSM_T3412_UNIT_10HOUR
   *  - LTE_PSM_T3412_UNIT_320HOUR
   *  - LTE_PSM_T3412_UNIT_DEACT
   */

  uint8_t unit;

  /* Timer value (0-31) */

  uint8_t time_val;
} lte_psm_timeval_t;

/* Definition of PSM settings used in lte_set_psm().
 * This is notified by get_psm_cb_t
 */

typedef struct lte_psm_setting
{
  /* PSM enable. Definition is as below.
   * - LTE_ENABLE
   * - LTE_DISABLE
   */

  bool              enable;

  /* Requested Active Time value(T3324). See lte_psm_timeval_t */

  lte_psm_timeval_t req_active_time;

  /* Extended periodic TAU value(T3412). See lte_psm_timeval_t */

  lte_psm_timeval_t ext_periodic_tau_time;
} lte_psm_setting_t;

/* Definition of APN setting used in lte_activate_pdn().
 */

typedef struct lte_apn_setting
{
  /* Access point name. It is terminated with '\0'.
   * Maximum length is LTE_APN_LEN including '\0'.
   */

  char   *apn;

  /* Type of IP for APN. Definition is as below.
   * - LTE_IPTYPE_V4
   * - LTE_IPTYPE_V6
   * - LTE_IPTYPE_V4V6
   * - LTE_IPTYPE_NON
   */

  uint8_t  ip_type;

  /* Type of Authentication. Definition is as below.
   * - LTE_APN_AUTHTYPE_NONE
   * - LTE_APN_AUTHTYPE_PAP
   * - LTE_APN_AUTHTYPE_CHAP
   */

  uint8_t  auth_type;

  /* Type of APN. Bit setting definition is as below.
   * - LTE_APN_TYPE_UNKNOWN
   * - LTE_APN_TYPE_DEFAULT
   * - LTE_APN_TYPE_MMS
   * - LTE_APN_TYPE_SUPL
   * - LTE_APN_TYPE_DUN
   * - LTE_APN_TYPE_HIPRI
   * - LTE_APN_TYPE_FOTA
   * - LTE_APN_TYPE_IMS
   * - LTE_APN_TYPE_CBS
   * - LTE_APN_TYPE_IA
   * - LTE_APN_TYPE_EMERGENCY
   */

  uint32_t apn_type;

  /* User name. It is terminated with '\0'.
   * Maximum length is LTE_APN_USER_NAME_LEN including '\0'.
   */

  char   *user_name;

  /* Password. It is terminated with '\0'.
   * Maximum length is LTE_APN_PASSWD_LEN including '\0'.
   */

  char   *password;
} lte_apn_setting_t;

/* Definition of ip address used in lte_pdn_t.
 */

typedef struct lte_ipaddr
{
  /* Type of IP address. Definition is as below.
   * - LTE_IPTYPE_V4
   * - LTE_IPTYPE_V6
   */

  uint8_t ip_type;

  /* IP address. It is terminated with '\0'.
   * eg. (IPv4) 192.0.2.1, (IPv6) 2001:db8:85a3:0:0:8a2e:370:7334
   */

  char  address[LTE_IPADDR_MAX_LEN];
} lte_ipaddr_t;

/* Definition of pdn information used in activate_pdn_cb_t.
 */

typedef struct lte_pdn
{
  /* PDN session id. The range is from
   * LTE_PDN_SESSIONID_MIN to LTE_PDN_SESSIONID_MAX.
   */

  uint8_t      session_id;

  /* PDN active status. Definition is as below.
   * - LTE_PDN_ACTIVE
   * - LTE_PDN_DEACTIVE
   */

  uint8_t      active;

  /* APN type of PDN. Bit setting definition is as below.
   * - LTE_APN_TYPE_UNKNOWN
   * - LTE_APN_TYPE_DEFAULT
   * - LTE_APN_TYPE_MMS
   * - LTE_APN_TYPE_SUPL
   * - LTE_APN_TYPE_DUN
   * - LTE_APN_TYPE_HIPRI
   * - LTE_APN_TYPE_FOTA
   * - LTE_APN_TYPE_IMS
   * - LTE_APN_TYPE_CBS
   * - LTE_APN_TYPE_IA
   * - LTE_APN_TYPE_EMERGENCY
   */

  uint32_t     apn_type;

  /* Number of valid ip addresses */

  uint8_t      ipaddr_num;

  /* IP address information. See @ref lte_ipaddr_t */

  lte_ipaddr_t address[LTE_PDN_IPADDR_MAX_COUNT];

  /* IMS registored status.
   * This is valid when LTE_APN_TYPE_IMS is set in apn_type.
   * Definition is as below.
   * - LTE_IMS_NOT_REGISTERED
   * - LTE_IMS_REGISTERED
   */

  uint8_t      ims_register;

  /* Status of data communication enability. Definition is as below.
   * - LTE_DATA_ALLOW
   * - LTE_DATA_DISALLOW
   */

  uint8_t      data_allow;

  /* Status of roaming data communication enability.
   * Definition is as below.
   * - LTE_DATA_ALLOW
   * - LTE_DATA_DISALLOW
   */

  uint8_t      data_roaming_allow;
} lte_pdn_t;

/* Definition of LTE network reject cause used in lte_nw_err_info_t.
 */

typedef struct lte_reject_cause
{
  /* Category of reject cause. Definition is as below.
   * - LTE_REJECT_CATEGORY_EMM
   * - LTE_REJECT_CATEGORY_ESM
   */

  uint8_t category;

  /* Value of LTE network reject cause.
   * Definition is See 3GPP TS 24.008 13.7.0
   */

  uint8_t value;
} lte_reject_cause_t;

/* Definition of LTE network error information used in lte_netinfo_t.
 */

typedef struct lte_nw_err_info
{
  /* Type of LTE network error. Definition is as below.
   * - LTE_NETERR_MAXRETRY
   * - LTE_NETERR_REJECT
   * - LTE_NETERR_NWDTCH
   */

  uint8_t            err_type;

  /* LTE network attach request reject cause. It can be referenced when
   * LTE_NETERR_REJECT is see in err_type field
   * See lte_reject_cause_t
   */

  lte_reject_cause_t reject_cause;
} lte_nw_err_info_t;

/* Definition of lte network information used in get_netinfo_cb_t.
 */

typedef struct lte_netinfo
{
  /* LTE network status. Definition is as below.
   * - LTE_NETSTAT_NOT_REG_NOT_SEARCHING
   * - LTE_NETSTAT_REG_HOME
   * - LTE_NETSTAT_NOT_REG_SEARCHING
   * - LTE_NETSTAT_REG_DENIED
   * - LTE_NETSTAT_UNKNOWN
   * - LTE_NETSTAT_REG_ROAMING
   * - LTE_NETSTAT_REG_SMS_ONLY_HOME
   * - LTE_NETSTAT_REG_SMS_ONLY_ROAMING
   * - LTE_NETSTAT_NOT_REG_EMERGENCY
   * - LTE_NETSTAT_REG_CSFB_NOT_PREF_HOME
   * - LTE_NETSTAT_REG_CSFB_NOT_PREF_ROAMING
   */

  uint8_t           nw_stat;

  /* LTE network error information. It can be referenced when
   * LTE_NETSTAT_REG_DENIED is set in nw_stat field.
   * See lte_nw_err_info_t
   */

  lte_nw_err_info_t nw_err;

  /* Number of PDN status informations.
   * The maximum number of PDNs is LTE_SESSION_ID_MAX.
   */

  uint8_t           pdn_num;

  /* List of PDN status. See lte_pdn_t
   *
   * When using the lte_getnetinfo,
   * the maximum number of PDNs status areas must be allocated.
   */

  lte_pdn_t         *pdn_stat;
} lte_netinfo_t;

/* Definition of error information used in lte_get_errinfo().
 */

typedef struct lte_error_info
{
  /* Enable error indicator. Bit setting definition is as below.
   * - LTE_ERR_INDICATOR_ERRCODE
   * - LTE_ERR_INDICATOR_ERRNO
   * - LTE_ERR_INDICATOR_ERRSTR
   */

  uint8_t err_indicator;

  /* Last error code. See 3GPP TS 27.007 9.2 */

  int32_t err_result_code;

  /* Last error no. See <errno.h> */

  int32_t err_no;

  /* Error string use debug only */

  char err_string[LTE_ERROR_STRING_MAX_LEN];
} lte_errinfo_t;

/* Definition of CE settings used in lte_set_ce().
 * This is notified by get_ce_cb_t
 */

typedef struct lte_ce_setting
{
  /* Mode A enable. Definition is as below.
   * - LTE_ENABLE
   * - LTE_DISABLE
   */

  bool mode_a_enable;

  /* Mode B enable. Definition is as below.
   * - LTE_ENABLE
   * - LTE_DISABLE
   */

  bool mode_b_enable;
} lte_ce_setting_t;

/* Definition of parameters for SIM information.
 * This is notified by get_siminfo_cb_t
 */

typedef struct lte_siminfo
{
  /* Indicates which parameter to get.
   * Bit setting definition is as below.
   * - LTE_SIMINFO_GETOPT_MCCMNC
   * - LTE_SIMINFO_GETOPT_SPN
   * - LTE_SIMINFO_GETOPT_ICCID
   * - LTE_SIMINFO_GETOPT_IMSI
   * - LTE_SIMINFO_GETOPT_GID1
   * - LTE_SIMINFO_GETOPT_GID2
   */

  uint32_t option;

  /* Mobile Country Code (000-999). It can be referenced when
   * LTE_SIMINFO_GETOPT_MCCMNC is set in option field.
   */

  char  mcc[LTE_MCC_DIGIT];

  /* Digit number of Mobile Network Code(2-3). It can be referenced when
   * LTE_SIMINFO_GETOPT_MCCMNC is set in option field.
   */

  uint8_t  mnc_digit;

  /* Mobile Network Code (00-999). It can be referenced when
   * LTE_SIMINFO_GETOPT_MCCMNC is set in option field.
   */

  char  mnc[LTE_MNC_DIGIT_MAX];

  /* Length of Service provider name. It can be referenced when
   * LTE_SIMINFO_GETOPT_SPN is set in option field.
   */

  uint8_t  spn_len;

  /* Service provider name. It can be referenced when
   * LTE_SIMINFO_GETOPT_SPN is set in option field.
   */

  char  spn[LTE_SIMINFO_SPN_LEN];

  /* Length of ICCID. It can be referenced when
   * LTE_SIMINFO_GETOPT_ICCID is set in option field.
   */

  uint8_t  iccid_len;

  /* ICCID. It can be referenced when
   * LTE_SIMINFO_GETOPT_ICCID is set in option field.
   * If the ICCID is 19 digits, "F" is set to the 20th digit.
   */

  uint8_t  iccid[LTE_SIMINFO_ICCID_LEN];

  /* Length of IMSI. It can be referenced when
   * LTE_SIMINFO_GETOPT_IMSI is set in option field.
   */

  uint8_t  imsi_len;

  /* International Mobile Subscriber Identity. It can be referenced when
   * LTE_SIMINFO_GETOPT_IMSI is set in option field.
   */

  char  imsi[LTE_SIMINFO_IMSI_LEN];

  /* Length of GID1. It can be referenced when
   * LTE_SIMINFO_GETOPT_GID1 is set in option field.
   */

  uint8_t  gid1_len;

  /* Group Identifier Level 1. It can be referenced when
   * LTE_SIMINFO_GETOPT_GID1 is set in option field.
   */

  char  gid1[LTE_SIMINFO_GID_LEN];

  /* Length of GID2. It can be referenced when
   * LTE_SIMINFO_GETOPT_GID2 is set in option field.
   */

  uint8_t  gid2_len;

  /* Group Identifier Level 1. It can be referenced when
   * LTE_SIMINFO_GETOPT_GID2 is set in option field.
   */

  char  gid2[LTE_SIMINFO_GID_LEN];
} lte_siminfo_t;

/* Definition of parameters for RAT information.
 */

typedef struct lte_ratinfo
{
  /* RAT type. Definition is as below.
   * - LTE_RAT_CATM
   * - LTE_RAT_NBIOT
   */

  uint8_t rat;

  /* Flag that indicates whether the modem supports multiple RATs.
   * Definition is as below.
   * - LTE_ENABLE
   * - LTE_DISABLE
   */

  bool multi_rat_support;

  /* Source that determined the current RAT.
   * Definition is as below.
   * - LTE_RAT_SOURCE_DEFAULT
   * - LTE_RAT_SOURCE_HOST
   * - LTE_RAT_SOURCE_LWM2M
   */

  uint8_t source;
} lte_ratinfo_t;

/* Definition of delta image setting to inject.
 */

struct ltefw_injectdata_s
{
  /* The pointer of delta image to inject */

  uint8_t  *data;

  /* The length of image to inject. */

  uint32_t data_len;

  /* Delta image injection mode.
   *  As below value stored.
   *  - LTEFW_INJECTION_MODE_NEW
   *  - LTEFW_INJECTION_MODE_APPEND
   */

  uint8_t  inject_mode;
};

/* Definition of callback function.
 *
 * Since lte_get_version() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_version().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] version : The version information of the modem.
 *                See @ref lte_version_t
 */

typedef void (*get_ver_cb_t)(uint32_t result, lte_version_t *version);

/* Definition of callback function.
 *
 * Since lte_get_phoneno() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_phoneno().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] errcause : Error cause. It is set only if the result is
 *                 not successful. As below value stored.
 * - LTE_ERR_WAITENTERPIN
 * - LTE_ERR_UNEXPECTED
 *
 * [in] phoneno : A character string indicating phone number.
 *                It is terminated with '\0'
 */

typedef void (*get_phoneno_cb_t)(uint32_t result, uint8_t errcause,
                                 char *phoneno);

/* Definition of callback function.
 *
 * Since lte_get_imsi() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_imsi().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] errcause : Error cause. It is set only if the result is
 *                 not successful. As below value stored.
 * - LTE_ERR_WAITENTERPIN
 * - LTE_ERR_UNEXPECTED
 * [in] imsi : A character string indicating IMSI.
 *             It is terminated with '\0'
 */

typedef void (*get_imsi_cb_t)(uint32_t result, uint8_t errcause,
                              char *imsi);

/* Definition of callback function.
 *
 * Since lte_get_imei() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_imei().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] imei : A character string indicating IMEI.
 *             It is terminated with '\0'
 */

typedef void (*get_imei_cb_t)(uint32_t result, char *imei);

/* Definition of callback function.
 *
 * Since lte_get_pinset() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_pinset().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] pinset : PIN settings information.
 *               See lte_getpin_t
 */

typedef void (*get_pinset_cb_t)(uint32_t result, lte_getpin_t *pinset);

/* Definition of callback function.
 *
 * Since lte_set_pinenable() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_set_pinenable().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] attemptsleft : Number of attempts left.
 *                     It is set only if the result is not successful.
 */

typedef void (*set_pinenable_cb_t)(uint32_t result, uint8_t attemptsleft);

/* Definition of callback function.
 *
 * Since lte_change_pin() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_change_pin().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] attemptsleft : Number of attempts left.
 *                     It is set only if the result is not successful.
 */

typedef void (*change_pin_cb_t)(uint32_t result, uint8_t attemptsleft);

/* Definition of callback function.
 *
 * Since lte_enter_pin() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_enter_pin().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] simstat : State after PIN enter.
 *                As below value stored.
 * - LTE_PINSTAT_READY
 * - LTE_PINSTAT_SIM_PIN
 * - LTE_PINSTAT_SIM_PUK
 * - LTE_PINSTAT_PH_SIM_PIN
 * - LTE_PINSTAT_PH_FSIM_PIN
 * - LTE_PINSTAT_PH_FSIM_PUK
 * - LTE_PINSTAT_SIM_PIN2
 * - LTE_PINSTAT_SIM_PUK2
 * - LTE_PINSTAT_PH_NET_PIN
 * - LTE_PINSTAT_PH_NET_PUK
 * - LTE_PINSTAT_PH_NETSUB_PIN
 * - LTE_PINSTAT_PH_NETSUB_PUK
 * - LTE_PINSTAT_PH_SP_PIN
 * - LTE_PINSTAT_PH_SP_PUK
 * - LTE_PINSTAT_PH_CORP_PIN
 * - LTE_PINSTAT_PH_CORP_PUK
 *
 * [in] attemptsleft : Number of attempts left.
 *                     It is set only if the result is not successful.
 *                     If simstat is other than PIN, PUK, PIN2, PUK2,
 *                     set the number of PIN.
 */

typedef void (*enter_pin_cb_t)(uint32_t result,
                               uint8_t simstat,
                               uint8_t attemptsleft);

/* Definition of callback function.
 *
 * Since lte_get_localtime() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_localtime().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] localtime : Local time. See lte_localtime_t
 */

typedef void (*get_localtime_cb_t)(uint32_t result,
                                   lte_localtime_t *localtime);

/* Definition of callback function.
 *
 * Since lte_get_operator() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_operator().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] oper : A character string indicating network operator.
 *             It is terminated with '\0' If it is not connected,
 *             the first character is '\0'.
 */

typedef void (*get_operator_cb_t)(uint32_t result, char *oper);

/* Definition of callback function.
 *
 * Since lte_get_edrx() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_edrx().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] settings : eDRX settings. See lte_edrx_setting_t
 */

typedef void (*get_edrx_cb_t)(uint32_t result, lte_edrx_setting_t *settings);

/* Definition of callback function.
 *
 * Since lte_set_edrx() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_set_edrx().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*set_edrx_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * Since lte_get_psm() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_psm().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] settings : PSM settings. See lte_psm_setting_t
 */

typedef void (*get_psm_cb_t)(uint32_t result, lte_psm_setting_t *settings);

/* Definition of callback function.
 *
 * Since lte_set_psm() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_set_psm().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*set_psm_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * Since lte_get_ce() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_ce().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] settings : CE settings. See lte_ce_setting_t
 */

typedef void (*get_ce_cb_t)(uint32_t result, lte_ce_setting_t *settings);

/* Definition of callback function.
 *
 * Since lte_set_ce() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_set_ce().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*set_ce_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * When the SIM state changes, the SIM state is
 * reported by this function.
 *
 * [in] simstat : The SIM state.
 *                As below value stored.
 * - LTE_SIMSTAT_REMOVAL
 * - LTE_SIMSTAT_INSERTION
 * - LTE_SIMSTAT_WAIT_PIN_UNLOCK
 * - LTE_SIMSTAT_PERSONAL_FAILED
 * - LTE_SIMSTAT_ACTIVATE
 * - LTE_SIMSTAT_DEACTIVATE
 */

typedef void (*simstat_report_cb_t)(uint32_t simstat);

/* Definition of callback function.
 *
 * When the local time changes, the local time is
 * reported by this function.
 *
 * [in] localtime : Local time. See lte_localtime_t
 */

typedef void (*localtime_report_cb_t)(lte_localtime_t *localtime);

/* Definition of callback function.
 *
 * The quality information is reported by this function. It is reported
 * at intervals of the set report period.
 *
 * [in] quality : Quality information. See lte_quality_t
 */

typedef void (*quality_report_cb_t)(lte_quality_t *quality);

/* Definition of callback function.
 *
 * The cell information is reported by this function. It is reported
 * at intervals of the set report period.
 *
 * [in] cellinfo : Cell information. See lte_cellinfo_t
 */

typedef void (*cellinfo_report_cb_t)(lte_cellinfo_t *cellinfo);

/* Definition of callback function.
 *
 * Since lte_radio_on() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_radio_on().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*radio_on_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * Since lte_radio_off() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_radio_off().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*radio_off_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * Since lte_get_netinfo() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_netinfo().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] info : Pointer of LTE network information.
 *             See lte_netinfo_t
 */

typedef void (*get_netinfo_cb_t)(uint32_t result, lte_netinfo_t *info);

/* Definition of callback function.
 *
 * Since lte_get_imscap() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_imscap.
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] imscap : The IMS capability.
 *               As below value stored.
 * - LTE_ENABLE
 * - LTE_DISABLE
 */

typedef void (*get_imscap_cb_t)(uint32_t result, bool imscap);

/* Definition of callback function.
 *
 * Since lte_activate_pdn() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_activate_pdn.
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 * - LTE_RESULT_CANCEL
 *
 * [in] pdn : The connect pdn information. See lte_pdn_t
 */

typedef void (*activate_pdn_cb_t)(uint32_t result, lte_pdn_t *pdn);

/* Definition of callback function.
 *
 * Since lte_deactivate_pdn() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_deactivate_pdn.
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*deactivate_pdn_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * Since lte_dataallow() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_dataallow.
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 */

typedef void (*data_allow_cb_t)(uint32_t result);

/* Definition of callback function.
 *
 * The modem restart is reported by this function. It is reported
 * at modem reset.
 *
 * [in] reason : Reason of modem restart.
 *               As below value stored.
 * - LTE_RESTART_USER_INITIATED
 * - LTE_RESTART_MODEM_INITIATED
 */

typedef void (*restart_report_cb_t)(uint32_t reason);

/* Definition of callback function.
 *
 * The change LTE network information is reported by this function.
 * It is reported at LTE network connection status.
 *
 * [in] info : Pointer of LTE network information.
 *             See lte_netinfo_t
 */

typedef void (*netinfo_report_cb_t)(lte_netinfo_t *info);

/* Definition of callback function.
 *
 * Since lte_get_siminfo() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_siminfo().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] siminfo : SIM information. See lte_siminfo_t
 */

typedef void (*get_siminfo_cb_t)(uint32_t result, lte_siminfo_t *siminfo);

/* Definition of callback function.
 *
 * Since lte_get_dynamic_edrx_param() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_dynamic_edrx_param().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] param : eDRX dynamic parameter. See lte_edrx_setting_t.
 */

typedef void (*get_dynamic_edrx_param_cb_t)(uint32_t result,
                                            lte_edrx_setting_t *param);

/* Definition of callback function.
 *
 * Since lte_get_dynamic_psm_param() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_dynamic_psm_param().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] param : PSM dynamic parameter. See lte_psm_setting_t
 */

typedef void (*get_dynamic_psm_param_cb_t)(uint32_t result,
                                           lte_psm_setting_t *param);

/* Definition of callback function.
 *
 * Since lte_get_current_edrx() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_current_edrx().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] settings : Current eDRX settings. See lte_edrx_setting_t.
 */

typedef void (*get_current_edrx_cb_t)(uint32_t result,
                                      lte_edrx_setting_t *settings);

/* Definition of callback function.
 *
 * Since lte_get_current_psm() is an asynchronous API,
 * the result is notified by this function.
 *
 * [in] result : The result of lte_get_current_psm().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] settings : Current PSM settings. See lte_psm_setting_t
 */

typedef void (*get_current_psm_cb_t)(uint32_t result,
                                     lte_psm_setting_t *settings);

/* Definition of callback function.
 *
 * Since lte_get_quality() is an asynchronous API,
 * the quality information is notified by this function.
 *
 * [in] result : The result of lte_get_quality().
 *               As below value stored.
 * - LTE_RESULT_OK
 * - LTE_RESULT_ERROR
 *
 * [in] quality : Quality information. See lte_quality_t
 */

typedef void (*get_quality_cb_t)(uint32_t result,
                                 lte_quality_t *quality);

#endif /* __INCLUDE_NUTTX_WIRELESS_LTE_LTE_H */
