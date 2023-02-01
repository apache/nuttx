/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr.c
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
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <sys/types.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/wireless/lte/lte.h>
#include <netinet/tcp.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/sms.h>

#include "alt1250.h"
#include "altcom_pkt.h"
#include "altcom_cmd.h"
#include "altcom_cmd_sock.h"
#include "altcom_cmd_sms.h"
#include "altcom_cmd_log.h"
#include "altcom_lwm2m_hdlr.h"

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
#define ALTCOMBS_BASE_HEX                 16
#define ALTCOMBS_EDRX_INVALID             (255)

#define READSET_BIT   (1 << 0)
#define WRITESET_BIT  (1 << 1)
#define EXCEPTSET_BIT (1 << 2)

#define ATCMD_HEADER     "AT"
#define ATCMD_HEADER_LEN (2)
#define ATCMD_FOOTER     '\r'
#define ATCMD_FOOTER_LEN (1)

#define ALTCOM_SO_SETMODE_8BIT   (1)
#define ALTCOM_SO_SETMODE_32BIT  (2)
#define ALTCOM_SO_SETMODE_LINGER (3)
#define ALTCOM_SO_SETMODE_INADDR (4)
#define ALTCOM_SO_SETMODE_IPMREQ (5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct compose_inst_s
{
  uint32_t cmdid;
  compose_handler_t hdlr;
} compose_inst_t;

typedef struct parse_inst_s
{
  uint16_t altcid;
  parse_handler_t hdlr;
} parse_inst_t;

static uint16_t g_set_repevt = 0;

static uint8_t g_edrx_acttype_table[] =
{
  LTE_EDRX_ACTTYPE_NOTUSE,
  LTE_EDRX_ACTTYPE_ECGSMIOT,
  LTE_EDRX_ACTTYPE_GSM,
  LTE_EDRX_ACTTYPE_IU,
  LTE_EDRX_ACTTYPE_WBS1,
  LTE_EDRX_ACTTYPE_NBS1
};

static const uint8_t g_edrx_cycle_wbs1_table[] =
{
  LTE_EDRX_CYC_512,
  LTE_EDRX_CYC_1024,
  LTE_EDRX_CYC_2048,
  LTE_EDRX_CYC_4096,
  LTE_EDRX_CYC_6144,
  LTE_EDRX_CYC_8192,
  LTE_EDRX_CYC_10240,
  LTE_EDRX_CYC_12288,
  LTE_EDRX_CYC_14336,
  LTE_EDRX_CYC_16384,
  LTE_EDRX_CYC_32768,
  LTE_EDRX_CYC_65536,
  LTE_EDRX_CYC_131072,
  LTE_EDRX_CYC_262144,
};

static const uint8_t g_edrx_cycle_nbs1_table[] =
{
  ALTCOMBS_EDRX_INVALID,
  ALTCOMBS_EDRX_INVALID,
  LTE_EDRX_CYC_2048,
  LTE_EDRX_CYC_4096,
  ALTCOMBS_EDRX_INVALID,
  LTE_EDRX_CYC_8192,
  ALTCOMBS_EDRX_INVALID,
  ALTCOMBS_EDRX_INVALID,
  ALTCOMBS_EDRX_INVALID,
  LTE_EDRX_CYC_16384,
  LTE_EDRX_CYC_32768,
  LTE_EDRX_CYC_65536,
  LTE_EDRX_CYC_131072,
  LTE_EDRX_CYC_262144,
  LTE_EDRX_CYC_524288,
  LTE_EDRX_CYC_1048576,
};

static const uint8_t g_edrx_ptw_wbs1_table[] =
{
  LTE_EDRX_PTW_128,
  LTE_EDRX_PTW_256,
  LTE_EDRX_PTW_384,
  LTE_EDRX_PTW_512,
  LTE_EDRX_PTW_640,
  LTE_EDRX_PTW_768,
  LTE_EDRX_PTW_896,
  LTE_EDRX_PTW_1024,
  LTE_EDRX_PTW_1152,
  LTE_EDRX_PTW_1280,
  LTE_EDRX_PTW_1408,
  LTE_EDRX_PTW_1536,
  LTE_EDRX_PTW_1664,
  LTE_EDRX_PTW_1792,
  LTE_EDRX_PTW_1920,
  LTE_EDRX_PTW_2048,
};

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
 * Private Function Prototypes
 ****************************************************************************/

static int32_t getver_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getphone_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getimsi_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getimei_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getpinset_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setpinlock_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setpincode_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t enterpin_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getltime_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getoper_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setrepqual_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setrepcell_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setrepevt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getce_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setce_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t radioon_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t radiooff_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t actpdn_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t deactpdn_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getnetinfo_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getimscap_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setrepnet_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getsiminfo_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getdedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getdpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getqual_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t actpdncancel_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getcell_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getrat_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setrat_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t socket_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t close_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t accept_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t bind_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t connect_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t fcntl_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getsockname_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getsockopt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t listen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t recvfrom_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t sendto_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t setsockopt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t select_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t sendatcmd_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t injectimage_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getimagelen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t execupdate_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t getupdateres_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t smsinit_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t smsfin_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t smssend_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t smsdelete_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t smsreportrecv_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t logsave_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t loglist_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
static int32_t logopen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t logclose_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t logread_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t loglseek_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
static int32_t logremove_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid);
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

static int32_t errinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getver_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getphone_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getimsi_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getimei_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getpinset_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setpinlock_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setpincode_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t enterpin_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getltime_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getoper_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setrepqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setrepcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setrepevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t repevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t repqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t repcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getce_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setce_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t radioon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t radiooff_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t actpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t deactpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getnetinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getimscap_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setrepnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t repnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getsiminfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getdedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getdpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t actpdncancel_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t setrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t sockcomm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t scokaddr_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t getsockopt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t recvfrom_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t select_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t sendatcmd_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t fwcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t smscommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t smssend_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t smsreportrecv_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t urc_event_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t logsave_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t loglist_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
static int32_t logcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
static int32_t logread_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap);
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define CTABLE_CONTENT(cmdid, hdlrname) \
    { LTE_CMDID_##cmdid, hdlrname##_pkt_compose }
#define PTABLE_CONTENT(altcid, hdlrname) \
    { APICMDID_##altcid, hdlrname##_pkt_parse }

static compose_inst_t g_composehdlrs[] =
{
  CTABLE_CONTENT(GETVER, getver),
  CTABLE_CONTENT(GETPHONE, getphone),
  CTABLE_CONTENT(GETIMSI, getimsi),
  CTABLE_CONTENT(GETIMEI, getimei),
  CTABLE_CONTENT(GETPINSET, getpinset),
  CTABLE_CONTENT(PINENABLE, setpinlock),
  CTABLE_CONTENT(CHANGEPIN, setpincode),
  CTABLE_CONTENT(ENTERPIN, enterpin),
  CTABLE_CONTENT(GETLTIME, getltime),
  CTABLE_CONTENT(GETOPER, getoper),
  CTABLE_CONTENT(REPQUAL, setrepqual),
  CTABLE_CONTENT(REPCELL, setrepcell),
  CTABLE_CONTENT(REPSIMSTAT, setrepevt),
  CTABLE_CONTENT(REPLTIME, setrepevt),
  CTABLE_CONTENT(GETEDRX, getedrx),
  CTABLE_CONTENT(SETEDRX, setedrx),
  CTABLE_CONTENT(GETPSM, getpsm),
  CTABLE_CONTENT(SETPSM, setpsm),
  CTABLE_CONTENT(GETCE, getce),
  CTABLE_CONTENT(SETCE, setce),
  CTABLE_CONTENT(RADIOON, radioon),
  CTABLE_CONTENT(RADIOOFF, radiooff),
  CTABLE_CONTENT(ACTPDN, actpdn),
  CTABLE_CONTENT(DEACTPDN, deactpdn),
  CTABLE_CONTENT(GETNETINFO, getnetinfo),
  CTABLE_CONTENT(IMSCAP, getimscap),
  CTABLE_CONTENT(REPNETINFO, setrepnet),
  CTABLE_CONTENT(GETSIMINFO, getsiminfo),
  CTABLE_CONTENT(GETCEDRX, getdedrx),
  CTABLE_CONTENT(GETCPSM, getdpsm),
  CTABLE_CONTENT(GETQUAL, getqual),
  CTABLE_CONTENT(ACTPDNCAN, actpdncancel),
  CTABLE_CONTENT(GETCELL, getcell),
  CTABLE_CONTENT(GETRAT, getrat),
  CTABLE_CONTENT(GETRATINFO, getrat),
  CTABLE_CONTENT(SETRAT, setrat),
  CTABLE_CONTENT(ACCEPT, accept),
  CTABLE_CONTENT(BIND, bind),
  CTABLE_CONTENT(CLOSE, close),
  CTABLE_CONTENT(CONNECT, connect),
  CTABLE_CONTENT(FCNTL, fcntl),
  CTABLE_CONTENT(GETSOCKNAME, getsockname),
  CTABLE_CONTENT(GETSOCKOPT, getsockopt),
  CTABLE_CONTENT(LISTEN, listen),
  CTABLE_CONTENT(RECVFROM, recvfrom),
  CTABLE_CONTENT(SELECT, select),
  CTABLE_CONTENT(SENDTO, sendto),
  CTABLE_CONTENT(SOCKET, socket),
  CTABLE_CONTENT(SETSOCKOPT, setsockopt),
  CTABLE_CONTENT(SENDATCMD, sendatcmd),
  CTABLE_CONTENT(INJECTIMAGE, injectimage),
  CTABLE_CONTENT(GETIMAGELEN, getimagelen),
  CTABLE_CONTENT(EXEUPDATE, execupdate),
  CTABLE_CONTENT(GETUPDATERES, getupdateres),
  CTABLE_CONTENT(SMS_INIT, smsinit),
  CTABLE_CONTENT(SMS_FIN,  smsfin),
  CTABLE_CONTENT(SMS_SEND, smssend),
  CTABLE_CONTENT(SMS_DELETE, smsdelete),
  CTABLE_CONTENT(SMS_REPORT_RECV, smsreportrecv),
  CTABLE_CONTENT(SAVE_LOG, logsave),
  CTABLE_CONTENT(GET_LOGLIST, loglist),
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
  CTABLE_CONTENT(LOGOPEN, logopen),
  CTABLE_CONTENT(LOGCLOSE, logclose),
  CTABLE_CONTENT(LOGREAD, logread),
  CTABLE_CONTENT(LOGLSEEK, loglseek),
  CTABLE_CONTENT(LOGREMOVE, logremove),
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */
};

static parse_inst_t g_parsehdlrs[] =
{
  PTABLE_CONTENT(ERRINFO, errinfo),
  PTABLE_CONTENT(GET_VERSION, getver),
  PTABLE_CONTENT(GET_PHONENO, getphone),
  PTABLE_CONTENT(GET_IMSI, getimsi),
  PTABLE_CONTENT(GET_IMEI, getimei),
  PTABLE_CONTENT(GET_PINSET, getpinset),
  PTABLE_CONTENT(SET_PIN_LOCK, setpinlock),
  PTABLE_CONTENT(SET_PIN_CODE, setpincode),
  PTABLE_CONTENT(ENTER_PIN, enterpin),
  PTABLE_CONTENT(GET_LTIME, getltime),
  PTABLE_CONTENT(GET_OPERATOR, getoper),
  PTABLE_CONTENT(SET_REP_QUALITY, setrepqual),
  PTABLE_CONTENT(SET_REP_CELLINFO, setrepcell),
  PTABLE_CONTENT(SET_REP_EVT, setrepevt),
  PTABLE_CONTENT(REPORT_EVT, repevt),
  PTABLE_CONTENT(REPORT_QUALITY, repqual),
  PTABLE_CONTENT(REPORT_CELLINFO, repcell),
  PTABLE_CONTENT(GET_EDRX, getedrx),
  PTABLE_CONTENT(SET_EDRX, setedrx),
  PTABLE_CONTENT(GET_PSM, getpsm),
  PTABLE_CONTENT(SET_PSM, setpsm),
  PTABLE_CONTENT(GET_CE, getce),
  PTABLE_CONTENT(SET_CE, setce),
  PTABLE_CONTENT(RADIO_ON, radioon),
  PTABLE_CONTENT(RADIO_OFF, radiooff),
  PTABLE_CONTENT(ACTIVATE_PDN, actpdn),
  PTABLE_CONTENT(DEACTIVATE_PDN, deactpdn),
  PTABLE_CONTENT(GET_NETINFO, getnetinfo),
  PTABLE_CONTENT(GET_IMS_CAP, getimscap),
  PTABLE_CONTENT(SETREP_NETINFO, setrepnet),
  PTABLE_CONTENT(REPORT_NETINFO, repnet),
  PTABLE_CONTENT(GET_SIMINFO, getsiminfo),
  PTABLE_CONTENT(GET_DYNAMICEDRX, getdedrx),
  PTABLE_CONTENT(GET_DYNAMICPSM, getdpsm),
  PTABLE_CONTENT(GET_QUALITY, getqual),
  PTABLE_CONTENT(ACTIVATE_PDN_CANCEL, actpdncancel),
  PTABLE_CONTENT(GET_CELLINFO, getcell),
  PTABLE_CONTENT(GET_RAT, getrat),
  PTABLE_CONTENT(SET_RAT, setrat),
  PTABLE_CONTENT(SOCK_ACCEPT, scokaddr),
  PTABLE_CONTENT(SOCK_BIND, sockcomm),
  PTABLE_CONTENT(SOCK_CLOSE, sockcomm),
  PTABLE_CONTENT(SOCK_CONNECT, sockcomm),
  PTABLE_CONTENT(SOCK_FCNTL, sockcomm),
  PTABLE_CONTENT(SOCK_GETSOCKNAME, scokaddr),
  PTABLE_CONTENT(SOCK_GETSOCKOPT, getsockopt),
  PTABLE_CONTENT(SOCK_LISTEN, sockcomm),
  PTABLE_CONTENT(SOCK_RECVFROM, recvfrom),
  PTABLE_CONTENT(SOCK_SELECT, select),
  PTABLE_CONTENT(SOCK_SENDTO, sockcomm),
  PTABLE_CONTENT(SOCK_SOCKET, sockcomm),
  PTABLE_CONTENT(SOCK_SETSOCKOPT, sockcomm),
  PTABLE_CONTENT(SEND_ATCMD, sendatcmd),
  PTABLE_CONTENT(FW_INJECTDELTAIMG, fwcommon),
  PTABLE_CONTENT(FW_GETDELTAIMGLEN, fwcommon),
  PTABLE_CONTENT(FW_EXECDELTAUPDATE, fwcommon),
  PTABLE_CONTENT(FW_GETUPDATERESULT, fwcommon),
  PTABLE_CONTENT(SMS_INIT, smscommon),
  PTABLE_CONTENT(SMS_FIN, smscommon),
  PTABLE_CONTENT(SMS_SEND, smssend),
  PTABLE_CONTENT(SMS_DELETE, smscommon),
  PTABLE_CONTENT(SMS_REPORT_RECV, smsreportrecv),
  PTABLE_CONTENT(URC_EVENT, urc_event),
  PTABLE_CONTENT(CLOGS, logsave),
  PTABLE_CONTENT(LOGLIST, loglist),
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
  PTABLE_CONTENT(LOGOPEN, logcommon),
  PTABLE_CONTENT(LOGCLOSE, logcommon),
  PTABLE_CONTENT(LOGREAD, logread),
  PTABLE_CONTENT(LOGLSEEK, logcommon),
  PTABLE_CONTENT(LOGREMOVE, logcommon),
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int32_t altcombs_set_pdninfo(struct apicmd_pdnset_s *cmd_pdn,
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

int32_t altcombs_set_pdninfo_v4(FAR struct apicmd_pdnset_v4_s *cmd_pdn,
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

int32_t altcombs_convert_api_edrx_value(lte_edrx_setting_t *api_edrx,
  struct apicmd_cmddat_setedrx_s *cmd_edrx, uint8_t altver)
{
  int           i;
  int           table_size = 0;

  if (!cmd_edrx || !api_edrx)
    {
      m_err("null param\n");
      return -EINVAL;
    }

  /* act_type check for version V4 or later */

  if (altver == ALTCOM_VER1)
    {
      if (api_edrx->act_type != LTE_EDRX_ACTTYPE_WBS1 &&
          api_edrx->act_type != LTE_EDRX_ACTTYPE_NOTUSE)
        {
          m_err("Operation is not allowed[act_type : %d].\n",
                      api_edrx->act_type);
          return -EPERM;
        }
    }
  else if (altver == ALTCOM_VER4)
    {
      if (api_edrx->act_type != LTE_EDRX_ACTTYPE_WBS1 &&
          api_edrx->act_type != LTE_EDRX_ACTTYPE_NBS1 &&
          api_edrx->act_type != LTE_EDRX_ACTTYPE_NOTUSE)
        {
          m_err("Operation is not allowed[act_type : %d].\n",
                          api_edrx->act_type);
          return -EPERM;
        }
    }
  else
    {
      return -ENOSYS;
    }

  table_size = ARRAY_SZ(g_edrx_acttype_table);
  for (i = 0; i < table_size; i++)
    {
      if (api_edrx->act_type == g_edrx_acttype_table[i])
        {
          cmd_edrx->acttype = (uint8_t)i;
          break;
        }
    }

  if (LTE_ENABLE == api_edrx->enable)
    {
      cmd_edrx->enable = LTE_ENABLE;

      if (APICMD_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          table_size = ARRAY_SZ(g_edrx_cycle_wbs1_table);

          for (i = 0; i < table_size; i++)
            {
              if (api_edrx->edrx_cycle == g_edrx_cycle_wbs1_table[i])
                {
                  cmd_edrx->edrx_cycle = (uint8_t)i;
                  break;
                }
            }

          if (i == table_size)
            {
              m_err("Invalid cycle :%ld\n", api_edrx->edrx_cycle);
              return -EINVAL;
            }

          table_size = ARRAY_SZ(g_edrx_ptw_wbs1_table);

          for (i = 0; i < table_size; i++)
            {
              if (api_edrx->ptw_val == g_edrx_ptw_wbs1_table[i])
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
      else if (APICMD_EDRX_ACTTYPE_NBS1 == cmd_edrx->acttype)
        {
          table_size = ARRAY_SZ(g_edrx_cycle_nbs1_table);

          for (i = 0; i < table_size; i++)
            {
              if (api_edrx->edrx_cycle == g_edrx_cycle_nbs1_table[i])
                {
                  cmd_edrx->edrx_cycle = (uint8_t)i;
                  break;
                }
            }

          if (i == table_size)
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

static void getnetinfo_parse_response(
  FAR struct apicmd_cmddat_getnetinfores_s *resp,
  FAR lte_netinfo_t *netinfo,
  uint8_t pdn_num)
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

static void getnetinfo_parse_response_v4(
  FAR struct apicmd_cmddat_getnetinfores_v4_s *resp,
  FAR lte_netinfo_t *netinfo,
  uint8_t pdn_num)
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

static int32_t altcombs_convert_apicmd_edrx_value(
  struct apicmd_edrxset_s *cmd_edrx, lte_edrx_setting_t *api_edrx)
{
  if (!cmd_edrx || !api_edrx)
    {
      m_err("null param\n");
      return -EINVAL;
    }

  if (LTE_DISABLE > cmd_edrx->enable ||
      LTE_ENABLE < cmd_edrx->enable)
    {
      m_err("Invalid enable :%d\n", cmd_edrx->enable);
      return -EINVAL;
    }

  if (LTE_ENABLE == cmd_edrx->enable)
    {
      if (APICMD_EDRX_ACTTYPE_NOTUSE != cmd_edrx->acttype &&
          APICMD_EDRX_ACTTYPE_WBS1   != cmd_edrx->acttype &&
          APICMD_EDRX_ACTTYPE_NBS1   != cmd_edrx->acttype)
        {
          m_err("Invalid acttype :%d\n", cmd_edrx->acttype);
          return -EINVAL;
        }

      if (LTE_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          if (cmd_edrx->edrx_cycle < ALTCOMBS_EDRX_CYCLE_WBS1_MIN ||
              cmd_edrx->edrx_cycle > ALTCOMBS_EDRX_CYCLE_WBS1_MAX)
            {
              m_err("Invalid cycle :%d\n", cmd_edrx->edrx_cycle);
              return -EINVAL;
            }

          if (ALTCOMBS_EDRX_PTW_WBS1_MIN > cmd_edrx->ptw_val ||
              ALTCOMBS_EDRX_PTW_WBS1_MAX < cmd_edrx->ptw_val)
            {
              m_err("Invalid PTW :%d\n", cmd_edrx->ptw_val);
              return -EINVAL;
            }
        }
      else if (LTE_EDRX_ACTTYPE_NBS1 == cmd_edrx->acttype)
        {
          if (cmd_edrx->edrx_cycle < ALTCOMBS_EDRX_CYCLE_NBS1_MIN ||
              cmd_edrx->edrx_cycle > ALTCOMBS_EDRX_CYCLE_NBS1_MAX)
            {
              m_err("Invalid cycle :%d\n", cmd_edrx->edrx_cycle);
              return -EINVAL;
            }

          if (cmd_edrx->ptw_val < ALTCOMBS_EDRX_PTW_NBS1_MIN ||
              cmd_edrx->ptw_val > ALTCOMBS_EDRX_PTW_NBS1_MAX)
            {
              m_err("Invalid PTW :%d\n", cmd_edrx->ptw_val);
              return -EINVAL;
            }
        }

      api_edrx->enable = LTE_ENABLE;
      api_edrx->act_type = g_edrx_acttype_table[cmd_edrx->acttype];
      if (APICMD_EDRX_ACTTYPE_WBS1 == cmd_edrx->acttype)
        {
          api_edrx->edrx_cycle =
            g_edrx_cycle_wbs1_table[cmd_edrx->edrx_cycle];
          api_edrx->ptw_val = g_edrx_ptw_wbs1_table[cmd_edrx->ptw_val];
        }
      else if (APICMD_EDRX_ACTTYPE_NBS1 == cmd_edrx->acttype)
        {
          api_edrx->edrx_cycle =
            g_edrx_cycle_nbs1_table[cmd_edrx->edrx_cycle];
          api_edrx->ptw_val = g_edrx_ptw_nbs1_table[cmd_edrx->ptw_val];
        }
    }
  else
    {
      api_edrx->enable = LTE_DISABLE;
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

static void getver_parse_response(FAR struct apicmd_cmddat_getverres_s *resp,
                                  FAR lte_version_t *version)
{
  memset(version, 0, sizeof(*version));
  strncpy(version->bb_product,
          (FAR const char *)resp->bb_product, LTE_VER_BB_PRODUCT_LEN - 1);
  strncpy(version->np_package,
          (FAR const char *)resp->np_package, LTE_VER_NP_PACKAGE_LEN - 1);
}

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

static void getsiminfo_parse_response(
  FAR struct apicmd_cmddat_getsiminfo_res_s *resp,
  FAR lte_siminfo_t *siminfo)
{
  siminfo->option = ntohl(resp->option);

  if (siminfo->option & LTE_SIMINFO_GETOPT_MCCMNC)
    {
      if (LTE_MNC_DIGIT_MAX < resp->mnc_digit)
        {
          resp->mnc_digit = LTE_MNC_DIGIT_MAX;
        }

      memcpy(siminfo->mcc, resp->mcc, LTE_MCC_DIGIT);
      siminfo->mnc_digit = resp->mnc_digit;
      memcpy(siminfo->mnc, resp->mnc, resp->mnc_digit);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_SPN)
    {
      if (LTE_SIMINFO_SPN_LEN < resp->spn_len)
        {
          resp->spn_len = LTE_SIMINFO_SPN_LEN;
        }

      siminfo->spn_len = resp->spn_len;
      memcpy(siminfo->spn, resp->spn, resp->spn_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_ICCID)
    {
      if (LTE_SIMINFO_ICCID_LEN < resp->iccid_len)
        {
          resp->iccid_len = LTE_SIMINFO_ICCID_LEN;
        }

      siminfo->iccid_len = resp->iccid_len;
      memcpy(siminfo->iccid, resp->iccid, resp->iccid_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_IMSI)
    {
      if (LTE_SIMINFO_IMSI_LEN < resp->imsi_len)
        {
          resp->imsi_len = LTE_SIMINFO_IMSI_LEN;
        }

      siminfo->imsi_len = resp->imsi_len;
      memcpy(siminfo->imsi, resp->imsi, resp->imsi_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID1)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid1_len)
        {
          resp->gid1_len = LTE_SIMINFO_GID_LEN;
        }

      siminfo->gid1_len = resp->gid1_len;
      memcpy(siminfo->gid1, resp->gid1, resp->gid1_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID2)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid2_len)
        {
          resp->gid2_len = LTE_SIMINFO_GID_LEN;
        }

      siminfo->gid2_len = resp->gid2_len;
      memcpy(siminfo->gid2, resp->gid2, resp->gid2_len);
    }
}

static void parse_ltime(FAR struct apicmd_cmddat_ltime_s *from,
  lte_localtime_t *to)
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

static void sockaddr2altstorage(FAR const struct sockaddr *from,
                          FAR struct altcom_sockaddr_storage *to)
{
  FAR struct sockaddr_in         *inaddr_from;
  FAR struct sockaddr_in6        *in6addr_from;
  FAR struct altcom_sockaddr_in  *inaddr_to;
  FAR struct altcom_sockaddr_in6 *in6addr_to;

  if (from->sa_family == AF_INET)
    {
      inaddr_from = (FAR struct sockaddr_in *)from;
      inaddr_to   = (FAR struct altcom_sockaddr_in *)to;

      inaddr_to->sin_len    = sizeof(struct altcom_sockaddr_in);
      inaddr_to->sin_family = ALTCOM_AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
        sizeof(struct altcom_in_addr));
    }
  else if (from->sa_family == AF_INET6)
    {
      in6addr_from = (FAR struct sockaddr_in6 *)from;
      in6addr_to   = (FAR struct altcom_sockaddr_in6 *)to;

      in6addr_to->sin6_len    = sizeof(struct altcom_sockaddr_in6);
      in6addr_to->sin6_family = ALTCOM_AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
        sizeof(struct altcom_in6_addr));
    }
}

static void altstorage2sockaddr(
  FAR const struct altcom_sockaddr_storage *from, FAR struct sockaddr *to)
{
  FAR struct altcom_sockaddr_in  *inaddr_from;
  FAR struct altcom_sockaddr_in6 *in6addr_from;
  FAR struct sockaddr_in         *inaddr_to;
  FAR struct sockaddr_in6        *in6addr_to;

  if (from->ss_family == ALTCOM_AF_INET)
    {
      inaddr_from = (FAR struct altcom_sockaddr_in *)from;
      inaddr_to   = (FAR struct sockaddr_in *)to;

      inaddr_to->sin_family = AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
        sizeof(struct in_addr));

      /* LwIP does not use this member, so it should be set to 0 */

      memset(inaddr_to->sin_zero, 0, sizeof(inaddr_to->sin_zero));
    }
  else if (from->ss_family == ALTCOM_AF_INET6)
    {
      in6addr_from = (FAR struct altcom_sockaddr_in6 *)from;
      in6addr_to   = (FAR struct sockaddr_in6 *)to;

      in6addr_to->sin6_family = AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
        sizeof(struct in6_addr));

      /* LwIP does not use thease members, so it should be set to 0 */

      in6addr_to->sin6_flowinfo = 0;
      in6addr_to->sin6_scope_id = 0;
    }
}

static int flags2altflags(int32_t flags, int32_t *altflags)
{
  if (flags & (MSG_DONTROUTE | MSG_CTRUNC | MSG_PROXY | MSG_TRUNC |
               MSG_EOR | MSG_FIN | MSG_SYN | MSG_CONFIRM |
               MSG_RST | MSG_ERRQUEUE | MSG_NOSIGNAL))
    {
      return -ENOPROTOOPT;
    }

  *altflags = 0;

  if (flags & MSG_PEEK)
    {
      *altflags |= ALTCOM_MSG_PEEK;
    }

  if (flags & MSG_WAITALL)
    {
      *altflags |= ALTCOM_MSG_WAITALL;
    }

  if (flags & MSG_OOB)
    {
      *altflags |= ALTCOM_MSG_OOB;
    }

  if (flags & MSG_DONTWAIT)
    {
      *altflags |= ALTCOM_MSG_DONTWAIT;
    }

  if (flags & MSG_MORE)
    {
      *altflags |= ALTCOM_MSG_MORE;
    }

  return 0;
}

static int get_so_setmode(uint16_t level, uint16_t option)
{
  int setmode = 0;

  switch (level)
    {
      case SOL_SOCKET:
        switch (option)
          {
            case SO_ACCEPTCONN:
            case SO_ERROR:
            case SO_BROADCAST:
            case SO_KEEPALIVE:
            case SO_REUSEADDR:
            case SO_TYPE:
            case SO_RCVBUF:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
#ifdef CONFIG_NET_SOLINGER
            case SO_LINGER:
              setmode = ALTCOM_SO_SETMODE_LINGER;
              break;
#endif
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_IP:
        switch (option)
          {
            case IP_TOS:
            case IP_TTL:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            case IP_MULTICAST_TTL:
            case IP_MULTICAST_LOOP:
              setmode = ALTCOM_SO_SETMODE_8BIT;
              break;
            case IP_MULTICAST_IF:
              setmode = ALTCOM_SO_SETMODE_INADDR;
              break;
            case IP_ADD_MEMBERSHIP:
            case IP_DROP_MEMBERSHIP:
              setmode = ALTCOM_SO_SETMODE_IPMREQ;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_TCP:
        switch (option)
          {
            case TCP_NODELAY:
            case TCP_KEEPIDLE:
            case TCP_KEEPINTVL:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_IPV6:
        switch (option)
          {
            case IPV6_V6ONLY:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      default:
        m_err("Not support level: %u\n", level);
        setmode = -EILSEQ;
        break;
    }

  return setmode;
}

static int copy_logfilename(FAR char *filename, size_t fnamelen,
                            FAR char *path)
{
  int ret = OK;
  size_t pathlen = strnlen(path, ALTCOM_PATH_LEN_MAX);

  if ((ALTCOM_PATH_LEN_MAX > pathlen) &&
      (strncmp(path, ALTCOM_LOGSPATH, strlen(ALTCOM_LOGSPATH)) == 0))
    {
      path += strlen(ALTCOM_LOGSPATH);
      pathlen -= strlen(ALTCOM_LOGSPATH);

      if (pathlen <= fnamelen)
        {
          strncpy(filename, path, fnamelen);
        }
      else
        {
          ret = -ENOBUFS;
        }
    }
  else
    {
      ret = -EILSEQ;
    }

  return ret;
}

#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS

static int create_logpath(FAR char *filename, FAR char *path)
{
  if (strlen(filename) + strlen(ALTCOM_LOGSPATH) >=
      ALTCOM_LOG_ACCESS_PATH_LEN_MAX)
    {
      return -ENAMETOOLONG;
    }

  snprintf(path, ALTCOM_LOG_ACCESS_PATH_LEN_MAX, "%s%s", ALTCOM_LOGSPATH,
           filename);

  return OK;
}

#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

static int32_t getver_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
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

static int32_t getphone_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_PHONENO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_PHONENO_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getimsi_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMSI;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMSI_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getimei_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMEI;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMEI_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getpinset_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_PINSET;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_PINSET_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setpinlock_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR bool *enable = (FAR bool *)arg[0];
  FAR char *pincode = (FAR char *)arg[1];

  FAR struct apicmd_cmddat_setpinlock_s *out =
    (FAR struct apicmd_cmddat_setpinlock_s *)pktbuf;

  out->mode = *enable;
  strncpy((FAR char *)out->pincode, pincode, sizeof(out->pincode));

  size = sizeof(struct apicmd_cmddat_setpinlock_s);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PIN_LOCK;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PIN_LOCK_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setpincode_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PIN_CODE;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PIN_CODE_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t enterpin_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ENTER_PIN;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ENTER_PIN_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getltime_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_LTIME;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_LTIME_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getoper_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_OPERATOR;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_OPERATOR_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setrepqual_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_REP_QUALITY;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_REP_QUALITY_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setrepcell_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_REP_CELLINFO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_REP_CELLINFO_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setrepevt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR simstat_report_cb_t *callback = (FAR simstat_report_cb_t *)arg[0];
  FAR int32_t *id = (FAR int32_t *)arg[1];

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
          size = -ENOSYS;
        }

      out->event = g_set_repevt;
      size = sizeof(struct apicmd_cmddat_setrepevt_s);
      *altcid = APICMDID_SET_REP_EVT;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setrepevt_v4_s *out =
        (FAR struct apicmd_cmddat_setrepevt_v4_s *)pktbuf;

      if (*id == LTE_CMDID_REPSIMSTAT)
        {
          if (callback)
            {
              g_set_repevt |=
                (APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
              out->enability = APICMD_SET_REP_EVT_ENABLE;
            }
          else
            {
              g_set_repevt &=
                ~(APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
              out->enability = APICMD_SET_REP_EVT_DISABLE;
            }
        }
      else if(*id == LTE_CMDID_REPLTIME)
        {
          if (callback)
            {
              g_set_repevt |= APICMD_SET_REP_EVT_LTIME;
              out->enability = APICMD_SET_REP_EVT_ENABLE;
            }
          else
            {
              g_set_repevt &= ~APICMD_SET_REP_EVT_LTIME;
              out->enability = APICMD_SET_REP_EVT_DISABLE;
            }
        }
      else
        {
          size = -ENOSYS;
        }

      out->event = htons(g_set_repevt);
      size = sizeof(struct apicmd_cmddat_setrepevt_v4_s);
      *altcid = APICMDID_SET_REP_EVT_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_EDRX;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getedrx_v4_s *out =
        (FAR struct apicmd_cmddat_getedrx_v4_s *)pktbuf;

      out->type = ALTCOM_GETEDRX_TYPE_UE;
      size = sizeof(struct apicmd_cmddat_getedrx_v4_s);
      *altcid = APICMDID_GET_EDRX_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t setedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_EDRX;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_EDRX_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_PSM;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getpsm_v4_s *out =
        (FAR struct apicmd_cmddat_getpsm_v4_s *)pktbuf;

      out->type = ALTCOM_GETPSM_TYPE_UE;

      size = sizeof(struct apicmd_cmddat_getpsm_v4_s);
      *altcid = APICMDID_GET_PSM_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t setpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SET_PSM;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SET_PSM_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getce_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_CE;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t setce_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR lte_ce_setting_t *settings = (FAR lte_ce_setting_t *)arg[0];

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
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t radioon_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_RADIO_ON;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_RADIO_ON_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t radiooff_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_RADIO_OFF;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_RADIO_OFF_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t actpdn_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
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

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ACTIVATE_PDN;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ACTIVATE_PDN_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t deactpdn_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR uint8_t *session_id = (FAR uint8_t *)arg[0];

  FAR struct apicmd_cmddat_deactivatepdn_s *out =
    (FAR struct apicmd_cmddat_deactivatepdn_s *)pktbuf;

  out->session_id = *session_id;

  size = sizeof(struct apicmd_cmddat_deactivatepdn_s);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_DEACTIVATE_PDN;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_DEACTIVATE_PDN_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getnetinfo_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_NETINFO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_NETINFO_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getimscap_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_IMS_CAP;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_IMS_CAP_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t setrepnet_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR struct apicmd_cmddat_set_repnetinfo_s *out =
    (FAR struct apicmd_cmddat_set_repnetinfo_s *)pktbuf;

  out->report = APICMD_REPNETINFO_REPORT_ENABLE;

  size = sizeof(struct apicmd_cmddat_set_repnetinfo_s);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_SETREP_NETINFO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SETREP_NETINFO_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getsiminfo_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR uint32_t *option = (FAR uint32_t *)arg[0];

  FAR struct apicmd_cmddat_getsiminfo_s *out =
    (FAR struct apicmd_cmddat_getsiminfo_s *)pktbuf;

  out->option = htonl(*option);
  size = sizeof(struct apicmd_cmddat_getsiminfo_s);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_SIMINFO;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_SIMINFO_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getdedrx_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_DYNAMICEDRX;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getedrx_v4_s *out =
        (FAR struct apicmd_cmddat_getedrx_v4_s *)pktbuf;

      out->type = ALTCOM_GETEDRX_TYPE_NEGOTIATED;
      size = sizeof(struct apicmd_cmddat_getedrx_v4_s);
      *altcid = APICMDID_GET_EDRX_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t getdpsm_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      size = 0;
      *altcid = APICMDID_GET_DYNAMICPSM;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getpsm_v4_s *out =
        (FAR struct apicmd_cmddat_getpsm_v4_s *)pktbuf;

      out->type = ALTCOM_GETPSM_TYPE_NEGOTIATED;

      size = sizeof(struct apicmd_cmddat_getpsm_v4_s);
      *altcid = APICMDID_GET_PSM_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t getqual_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_GET_QUALITY;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_QUALITY_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t actpdncancel_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_ACTIVATE_PDN_CANCEL;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_ACTIVATE_PDN_CANCEL_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getcell_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_CELLINFO_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t getrat_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_GET_RAT_V4;
    }
  else
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t setrat_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR uint8_t *rat = (FAR uint8_t *)arg[0];
  FAR bool *persistent = (FAR bool *)arg[1];

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
    {
      size = -ENOTSUP;
    }

  return size;
}

static int32_t sockaddrlen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];
  FAR int16_t *addrlen = (FAR int16_t *)arg[1];

  FAR struct altmdmpkt_sockaddrlen_s *out =
    (FAR struct altmdmpkt_sockaddrlen_s *)pktbuf;

  out->sockfd = htonl(*usockid);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->addrlen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->addrlen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      size = sizeof(struct altmdmpkt_sockaddrlen_s);
    }

  return size;
}

static int32_t sockaddr_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];
  FAR int16_t *addrlen = (FAR int16_t *)arg[1];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[2];

  FAR struct altmdmpkt_sockaddr_s *out =
    (FAR struct altmdmpkt_sockaddr_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  out->sockfd = htonl(*usockid);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->namelen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->namelen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else
    {
      size = -EINVAL;
    }

  sockaddr2altstorage((struct sockaddr *)sa, &altsa);
  memcpy(&out->name, &altsa, sizeof(struct altcom_sockaddr_storage));

  if (size == 0)
    {
      size = sizeof(struct altmdmpkt_sockaddr_s);
    }

  return size;
}

static int32_t socket_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int16_t *domain = (FAR int16_t *)arg[0];
  FAR int16_t *type = (FAR int16_t *)arg[1];
  FAR int16_t *protocol = (FAR int16_t *)arg[2];

  FAR struct apicmd_socket_s *out =
    (FAR struct apicmd_socket_s *)pktbuf;

  /* convert domain */

  if (*domain == PF_UNSPEC)
    {
      out->domain = htonl(ALTCOM_PF_UNSPEC);
    }
  else if (*domain == PF_INET)
    {
      out->domain = htonl(ALTCOM_PF_INET);
    }
  else if (*domain == PF_INET6)
    {
      out->domain = htonl(ALTCOM_PF_INET6);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  /* convert type */

  if (*type == SOCK_STREAM)
    {
      out->type = htonl(ALTCOM_SOCK_STREAM);
    }
  else if (*type == SOCK_DGRAM)
    {
      out->type = htonl(ALTCOM_SOCK_DGRAM);
    }
  else if (*type == SOCK_RAW)
    {
      out->type = htonl(ALTCOM_SOCK_RAW);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  /* convert protocol */

  if (*protocol == IPPROTO_IP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_IP);
    }
  else if (*protocol == IPPROTO_ICMP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_ICMP);
    }
  else if (*protocol == IPPROTO_TCP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_TCP);
    }
  else if (*protocol == IPPROTO_UDP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_UDP);
    }
  else if (*protocol == IPPROTO_IPV6)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_IPV6);
    }
  else if (*protocol == IPPROTO_ICMP6)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_ICMPV6);
    }
  else if (*protocol == IPPROTO_UDPLITE)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_UDPLITE);
    }
  else if (*protocol == IPPROTO_RAW)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_RAW);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_socket_s);
    }

  *altcid = APICMDID_SOCK_SOCKET;

  return size;
}

static int32_t close_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];

  FAR struct apicmd_close_s *out =
    (FAR struct apicmd_close_s *)pktbuf;

  out->sockfd = htonl(*usockid);

  size = sizeof(struct apicmd_close_s);

  *altcid = APICMDID_SOCK_CLOSE;

  return size;
}

static int32_t accept_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddrlen_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_ACCEPT;

  return size;
}

static int32_t bind_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddr_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_BIND;

  return size;
}

static int32_t connect_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddr_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_CONNECT;

  return size;
}

static int32_t fcntl_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *cmd = (FAR int32_t *)arg[1];
  FAR int32_t *val = (FAR int32_t *)arg[2];

  FAR struct apicmd_fcntl_s *out =
    (FAR struct apicmd_fcntl_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  out->cmd = htonl(*cmd);
  out->val = htonl(*val);

  if (size == 0)
    {
      size = sizeof(struct apicmd_fcntl_s);
    }

  *altcid = APICMDID_SOCK_FCNTL;

  return size;
}

static int32_t getsockname_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddrlen_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_GETSOCKNAME;

  return size;
}

static int32_t getsockopt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int16_t *level = (FAR int16_t *)arg[1];
  FAR int16_t *option = (FAR int16_t *)arg[2];
  FAR uint16_t *max_valuelen = (FAR uint16_t *)arg[3];

  FAR struct apicmd_getsockopt_s *out =
    (FAR struct apicmd_getsockopt_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  if (*level == SOL_SOCKET)
    {
      out->level = htonl(ALTCOM_SOL_SOCKET);
      out->optlen = htonl(*max_valuelen);

      if (*option == SO_ACCEPTCONN)
        {
          out->optname = htonl(ALTCOM_SO_ACCEPTCONN);
        }
      else if (*option == SO_ERROR)
        {
          out->optname = htonl(ALTCOM_SO_ERROR);
        }
      else if (*option == SO_BROADCAST)
        {
          out->optname = htonl(ALTCOM_SO_BROADCAST);
        }
      else if (*option == SO_KEEPALIVE)
        {
          out->optname = htonl(ALTCOM_SO_KEEPALIVE);
        }
      else if (*option == SO_REUSEADDR)
        {
          out->optname = htonl(ALTCOM_SO_REUSEADDR);
        }
      else if (*option == SO_TYPE)
        {
          out->optname = htonl(ALTCOM_SO_TYPE);
        }
      else if (*option == SO_RCVBUF)
        {
          out->optname = htonl(ALTCOM_SO_RCVBUF);
        }
#ifdef CONFIG_NET_SOLINGER
      else if (*option == SO_LINGER)
        {
          out->optname = htonl(ALTCOM_SO_LINGER);
        }
#endif
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IP)
    {
      out->level = htonl(ALTCOM_IPPROTO_IP);
      out->optlen = htonl(*max_valuelen);

      if (*option == IP_TOS)
        {
          out->optname = htonl(ALTCOM_IP_TOS);
        }
      else if (*option == IP_TTL)
        {
          out->optname = htonl(ALTCOM_IP_TTL);
        }
      else if (*option == IP_MULTICAST_TTL)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_TTL);
        }
      else if (*option == IP_MULTICAST_LOOP)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_LOOP);
        }
      else if (*option == IP_MULTICAST_IF)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_IF);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_TCP)
    {
      out->level = htonl(ALTCOM_IPPROTO_TCP);
      out->optlen = htonl(*max_valuelen);
      if (*option == TCP_NODELAY)
        {
          out->optname = htonl(ALTCOM_TCP_NODELAY);
        }
      else if (*option == TCP_KEEPIDLE)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPIDLE);
        }
      else if (*option == TCP_KEEPINTVL)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPINTVL);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IPV6)
    {
      out->level = htonl(ALTCOM_IPPROTO_IPV6);
      out->optlen = htonl(*max_valuelen);
      if (*option == IPV6_V6ONLY)
        {
          out->optname = htonl(ALTCOM_IPV6_V6ONLY);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else
    {
      size = -ENOPROTOOPT;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_getsockopt_s);
    }

  *altcid = APICMDID_SOCK_GETSOCKOPT;

  return size;
}

static int32_t listen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR uint16_t *backlog = (FAR uint16_t *)arg[1];

  FAR struct apicmd_listen_s *out =
    (FAR struct apicmd_listen_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  out->backlog = htonl(*backlog);

  size = sizeof(struct apicmd_listen_s);

  *altcid = APICMDID_SOCK_LISTEN;

  return size;
}

static int32_t recvfrom_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *flags = (FAR int32_t *)arg[1];
  FAR uint16_t *max_buflen = (FAR uint16_t *)arg[2];
  FAR uint16_t *max_addrlen = (FAR uint16_t *)arg[3];

  FAR struct apicmd_recvfrom_s *out =
    (FAR struct apicmd_recvfrom_s *)pktbuf;
  int32_t flg;

  out->sockfd = htonl(*sockfd);
  if (*max_buflen > APICMD_DATA_LENGTH)
    {
      /* Truncate the length to the maximum transfer size */

      *max_buflen = APICMD_DATA_LENGTH;
    }

  out->recvlen = htonl(*max_buflen);
  size = flags2altflags(*flags, &flg);
  out->flags = htonl(flg);
  if (*max_addrlen == sizeof(struct sockaddr_in))
    {
      out->fromlen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*max_addrlen == sizeof(struct sockaddr_in6))
    {
      out->fromlen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else if (*max_addrlen == 0)
    {
      out->fromlen = htonl(0);
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_recvfrom_s);
    }

  *altcid = APICMDID_SOCK_RECVFROM;

  return size;
}

static int32_t sendto_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *flags = (FAR int32_t *)arg[1];
  FAR uint16_t *addrlen = (FAR uint16_t *)arg[2];
  FAR uint16_t *buflen = (FAR uint16_t *)arg[3];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[4];
  FAR uint8_t *buf = (FAR uint8_t *)arg[5];

  FAR struct apicmd_sendto_s *out =
    (FAR struct apicmd_sendto_s *)pktbuf;
  int32_t flg;
  struct altcom_sockaddr_storage altsa;

  if (*buflen > APICMD_DATA_LENGTH)
    {
      /* Truncate the length to the maximum transfer size */

      *buflen = APICMD_DATA_LENGTH;
    }
  else if (*buflen < 0)
    {
      size = -EINVAL;
      goto err_out;
    }

  if (*buflen > 0 && !buf)
    {
      size = -EINVAL;
      goto err_out;
    }

  if (sa && !(*addrlen))
    {
      size = -EINVAL;
      goto err_out;
    }

  out->sockfd = htonl(*sockfd);
  size = flags2altflags(*flags, &flg);
  if (size != 0)
    {
      goto err_out;
    }

  out->flags = htonl(flg);
  out->datalen = htonl(*buflen);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->tolen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->tolen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else if (*addrlen == 0)
    {
      out->tolen = htonl(0);
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      memset(&altsa, 0, sizeof(struct altcom_sockaddr_storage));
      sockaddr2altstorage((struct sockaddr *)sa, &altsa);
      memcpy(&out->to, &altsa, *addrlen);
      memcpy(out->senddata, buf, *buflen);
      size = sizeof(struct apicmd_sendto_s) - sizeof(out->senddata) +
        *buflen;
    }

err_out:
  *altcid = APICMDID_SOCK_SENDTO;

  return size;
}

static int32_t setsockopt_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int16_t *level = (FAR int16_t *)arg[1];
  FAR int16_t *option = (FAR int16_t *)arg[2];
  FAR uint16_t *valuelen = (FAR uint16_t *)arg[3];
  FAR uint8_t *value = (FAR uint8_t *)arg[4];

  FAR struct apicmd_setsockopt_s *out =
    (FAR struct apicmd_setsockopt_s *)pktbuf;

  int setmode = 0;

  out->sockfd = htonl(*sockfd);
  if (*level == SOL_SOCKET)
    {
      out->level = htonl(ALTCOM_SOL_SOCKET);
      out->optlen = htonl(*valuelen);

      if (*option == SO_BROADCAST)
        {
          out->optname = htonl(ALTCOM_SO_BROADCAST);
        }
      else if (*option == SO_REUSEADDR)
        {
          out->optname = htonl(ALTCOM_SO_REUSEADDR);
        }
      else if (*option == SO_KEEPALIVE)
        {
          out->optname = htonl(ALTCOM_SO_KEEPALIVE);
        }
      else if (*option == SO_RCVBUF)
        {
          out->optname = htonl(ALTCOM_SO_RCVBUF);
        }
#ifdef CONFIG_NET_SOLINGER
      else if (*option == SO_LINGER)
        {
          out->optname = htonl(ALTCOM_SO_LINGER);
        }
#endif
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IP)
    {
      out->level = htonl(ALTCOM_IPPROTO_IP);
      out->optlen = htonl(*valuelen);

      if (*option == IP_TOS)
        {
          out->optname = htonl(ALTCOM_IP_TOS);
        }
      else if (*option == IP_TTL)
        {
          out->optname = htonl(ALTCOM_IP_TTL);
        }
      else if (*option == IP_MULTICAST_TTL)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_TTL);
        }
      else if (*option == IP_MULTICAST_LOOP)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_LOOP);
        }
      else if (*option == IP_MULTICAST_IF)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_IF);
        }
      else if (*option == IP_ADD_MEMBERSHIP)
        {
          out->optname = htonl(ALTCOM_IP_ADD_MEMBERSHIP);
        }
      else if (*option == IP_DROP_MEMBERSHIP)
        {
          out->optname = htonl(ALTCOM_IP_DROP_MEMBERSHIP);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_TCP)
    {
      out->level = htonl(ALTCOM_IPPROTO_TCP);
      out->optlen = htonl(*valuelen);
      if (*option == TCP_NODELAY)
        {
          out->optname = htonl(ALTCOM_TCP_NODELAY);
        }
      else if (*option == TCP_KEEPIDLE)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPIDLE);
        }
      else if (*option == TCP_KEEPINTVL)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPINTVL);
        }
      else if (*option == TCP_KEEPCNT)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPCNT);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IPV6)
    {
      out->level = htonl(ALTCOM_IPPROTO_IPV6);
      out->optlen = htonl(*valuelen);
      if (*option == IPV6_V6ONLY)
        {
          out->optname = htonl(ALTCOM_IPV6_V6ONLY);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else
    {
      size = -ENOPROTOOPT;
    }

  if (size < 0)
    {
      goto exit;
    }

  setmode = get_so_setmode(*level, *option);

  switch (setmode)
    {
      case ALTCOM_SO_SETMODE_8BIT:
        if (*valuelen < sizeof(int8_t))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(int8_t));
            size = -EINVAL;
            break;
          }

        *out->optval = value[0];
        break;
      case ALTCOM_SO_SETMODE_32BIT:
        if (*valuelen < sizeof(int32_t))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(int32_t));
            size = -EINVAL;
            break;
          }

        *((FAR int32_t *)out->optval) =
          htonl(*((FAR int32_t *)value));
        break;
      case ALTCOM_SO_SETMODE_LINGER:
        if (*valuelen < sizeof(struct linger))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct linger));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_linger *)out->optval)->l_onoff =
          htonl(((FAR struct linger *)value)->l_onoff);
        ((FAR struct altcom_linger *)out->optval)->l_linger =
          htonl(((FAR struct linger *)value)->l_linger);
        break;
      case ALTCOM_SO_SETMODE_INADDR:
        if (*valuelen < sizeof(struct in_addr))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct in_addr));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_in_addr *)out->optval)->s_addr =
          htonl(((FAR struct in_addr *)value)->s_addr);
        break;
      case ALTCOM_SO_SETMODE_IPMREQ:
        if (*valuelen < sizeof(struct ip_mreq))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct ip_mreq));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_ip_mreq *)out->optval)->imr_multiaddr.s_addr =
          htonl(((FAR struct ip_mreq *)value)->imr_multiaddr.s_addr);
        ((FAR struct altcom_ip_mreq *)out->optval)->imr_interface.s_addr =
          htonl(((FAR struct ip_mreq *)value)->imr_interface.s_addr);
        break;
      default:
        size = -EINVAL;
        break;
    }

exit:
  if (size == 0)
    {
      size = sizeof(struct apicmd_setsockopt_s);
    }

  *altcid = APICMDID_SOCK_SETSOCKOPT;

  return size;
}

static int32_t sendatcmd_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR const char *cmd = (FAR const char *)arg[0];
  int cmdlen = (int)arg[1];

  size = cmdlen - ATCMD_HEADER_LEN;
  memcpy(pktbuf, cmd + ATCMD_HEADER_LEN, size);
  pktbuf[size - ATCMD_FOOTER_LEN] = '\0';

  if (altver == ALTCOM_VER1)
    {
      size = -ENOTSUP;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_SEND_ATCMD_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t injectimage_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  FAR uint8_t *sending_data = (uint8_t *)arg[0];
  int len = *(int *)arg[1];
  bool mode = *(bool *)arg[2];

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
  else if (altver == ALTCOM_VER4)
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
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getimagelen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_GETDELTAIMGLEN;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_GETDELTAIMGLEN_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t execupdate_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_EXECDELTAUPDATE;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_EXECDELTAUPDATE_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t getupdateres_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_FW_GETUPDATERESULT;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_FW_GETUPDATERESULT_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t select_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *request = (FAR int32_t *)arg[0];
  FAR int32_t *id = (FAR int32_t *)arg[1];
  FAR int32_t *maxfds = (FAR int32_t *)arg[2];
  FAR uint16_t *used_setbit = (FAR uint16_t *)arg[3];
  FAR altcom_fd_set *readset = (FAR altcom_fd_set *)arg[4];
  FAR altcom_fd_set *writeset = (FAR altcom_fd_set *)arg[5];
  FAR altcom_fd_set *exceptset = (FAR altcom_fd_set *)arg[6];

  FAR struct apicmd_select_s *out =
    (FAR struct apicmd_select_s *)pktbuf;

  out->request = htonl(*request);
  out->id = htonl(*id);
  out->maxfds = htonl(*maxfds);
  out->used_setbit = htons(*used_setbit);
  memcpy(&out->readset, readset, sizeof(altcom_fd_set));
  memcpy(&out->writeset, writeset, sizeof(altcom_fd_set));
  memcpy(&out->exceptset, exceptset, sizeof(altcom_fd_set));

  size = sizeof(struct apicmd_select_s);

  *altcid = APICMDID_SOCK_SELECT;

  return size;
}

static int32_t smsinit_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_init_req_s *out =
    (FAR struct apicmd_sms_init_req_s *)pktbuf;

  *altcid = APICMDID_SMS_INIT;

  out->types = ALTCOM_SMS_MSG_TYPE_SEND | ALTCOM_SMS_MSG_TYPE_RECV |
               ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT;
  out->storage_use = 1; /* always use storage */

  return sizeof(struct apicmd_sms_init_req_s);
}

static int32_t smsfin_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  *altcid = APICMDID_SMS_FIN;

  return 0;
}

static int32_t smssend_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  int i;
  FAR struct apicmd_sms_send_req_s *out =
    (FAR struct apicmd_sms_send_req_s *)pktbuf;
  FAR struct sms_send_msg_s *msg = (FAR struct sms_send_msg_s *)arg[0];
  uint16_t msglen = *((FAR uint16_t *)arg[1]);
  bool en_status_report = *((FAR bool *)arg[2]);
  FAR struct sms_sc_addr_s *scaddr = (FAR struct sms_sc_addr_s *)arg[3];
  uint8_t chset = *((FAR uint8_t *)arg[4]);
  FAR uint8_t *dest_toa = (FAR uint8_t *)arg[5];

  if (msglen > pktsz)
    {
      return -ENOBUFS;
    }

  if ((msg->header.destaddrlen % 2) || (msg->header.datalen % 2) ||
      (msg->header.destaddrlen > (SMS_MAX_ADDRLEN * 2)) ||
      (msg->header.datalen > (SMS_MAX_DATALEN * 2)))
    {
      /* destaddrlen and datalen must be even numbers */

      return -EINVAL;
    }

  out->sc_addr.toa = scaddr->toa;
  out->sc_addr.length = scaddr->addrlen;

  /* Swap sc address */

  for (i = 0; i < (scaddr->addrlen / 2); i++)
    {
      out->sc_addr.address[i] = htons(scaddr->addr[i]);
    }

  out->valid_indicator = ALTCOM_SMS_MSG_VALID_UD;
  out->valid_indicator |= en_status_report ? ALTCOM_SMS_MSG_VALID_SRR : 0;

  out->dest_addr.toa = (dest_toa == NULL) ? 0 : *dest_toa;
  out->dest_addr.length = msg->header.destaddrlen;

  /* Swap destination address */

  for (i = 0; i < (msg->header.destaddrlen / 2); i++)
    {
      out->dest_addr.address[i] = htons(msg->header.destaddr[i]);
    }

  switch (chset)
    {
      case SMS_CHSET_UCS2:
        chset = ALTCOM_SMS_CHSET_UCS2;
        break;
      case SMS_CHSET_GSM7:
        chset = ALTCOM_SMS_CHSET_GSM7;
        break;
      case SMS_CHSET_BINARY:
        chset = ALTCOM_SMS_CHSET_BINARY;
        break;
      default:
        return -EINVAL;
    }

  out->userdata.chset = ALTCOM_SMS_CHSET_UCS2;
  out->userdata.data_len = htons(msg->header.datalen);

  /* Swap data */

  for (i = 0; i < (msg->header.datalen / 2); i++)
    {
      out->user_data[i] = htons(msg->data[i]);
    }

  *altcid = APICMDID_SMS_SEND;

  size = sizeof(struct apicmd_sms_send_req_s) + msg->header.datalen;

  return size;
}

static int32_t smsdelete_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_delete_s *out =
    (FAR struct apicmd_sms_delete_s *)pktbuf;
  uint16_t msg_index = *(FAR uint16_t *)arg[0];

  *altcid = APICMDID_SMS_DELETE;

  out->index = htons(msg_index);
  out->types = 0;

  return sizeof(struct apicmd_sms_delete_s);
}

static int32_t smsreportrecv_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_sms_res_s *out =
    (FAR struct apicmd_sms_res_s *)pktbuf;

  *altcid = APICMDID_SMS_REPORT_RECV | ALTCOM_CMDID_REPLY_BIT;

  out->result = htonl(0); /* always success */

  return sizeof(struct apicmd_sms_res_s);
}

static int32_t logsave_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_cmddat_clogs_s *out =
    (FAR struct apicmd_cmddat_clogs_s *)pktbuf;
  size_t len = *(FAR size_t *)arg[0];
  int32_t size = sizeof(struct apicmd_cmddat_clogs_s);

  out->pathlen = len + strlen(ALTCOM_LOGSPATH);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_CLOGS;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_CLOGS_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t loglist_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmddbg_getloglist_s *out =
    (FAR struct apicmddbg_getloglist_s *)pktbuf;
  size_t len = (size_t)arg[0];
  int32_t size = sizeof(struct apicmddbg_getloglist_s);

  out->listsize = LTE_LOG_LIST_SIZE;
  out->pathlen = len + strlen(ALTCOM_LOGSPATH);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGLIST;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGLIST_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS

static int32_t logopen_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logopen_s *out = (FAR struct apicmd_logopen_s *)pktbuf;
  FAR char *filename = (FAR char *)arg[0];
  int32_t size = sizeof(struct apicmd_logopen_s);
  int ret;

  ret = create_logpath(filename, out->path);
  if (ret < 0)
    {
      return ret;
    }

  out->flags = htonl(ALTCOM_LOG_OPEN_FLAGS);
  out->mode = htonl(0);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGOPEN;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGOPEN_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t logclose_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logclose_s *out = (FAR struct apicmd_logclose_s *)pktbuf;
  int fd = (int)arg[0];
  int32_t size = sizeof(struct apicmd_logclose_s);

  out->fd = htonl(fd);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGCLOSE;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGCLOSE_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t logread_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logread_s *out = (FAR struct apicmd_logread_s *)pktbuf;
  int fd = (int)arg[0];
  size_t rlen = (size_t)arg[1];
  int32_t size = sizeof(struct apicmd_logread_s);

  out->fd = htonl(fd);
  out->readlen = (rlen > ALTCOM_LOG_READ_LEN_MAX) ?
                  htonl(ALTCOM_LOG_READ_LEN_MAX) : htonl(rlen);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGREAD;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGREAD_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t loglseek_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_loglseek_s *out = (FAR struct apicmd_loglseek_s *)pktbuf;
  int fd = (int)arg[0];
  off_t offset = *(FAR off_t *)arg[1];
  int whence = (int)arg[2];
  int32_t size = sizeof(struct apicmd_loglseek_s);

  switch (whence)
    {
      case SEEK_SET:
        out->whence = htonl(ALTCOM_LOG_SEEK_SET);
        break;

       case SEEK_CUR:
        out->whence = htonl(ALTCOM_LOG_SEEK_CUR);
        break;

      case SEEK_END:
        out->whence = htonl(ALTCOM_LOG_SEEK_END);
        break;

      default:
        return -EINVAL;
        break;
    }

  out->fd = htonl(fd);
  out->offset = htonl(offset);

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGLSEEK;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGLSEEK_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

static int32_t logremove_pkt_compose(FAR void **arg,
                          size_t arglen, uint8_t altver, FAR uint8_t *pktbuf,
                          const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logremove_s *out =
    (FAR struct apicmd_logremove_s *)pktbuf;
  FAR char *filename = (FAR char *)arg[0];
  int32_t size = sizeof(struct apicmd_logremove_s);
  int ret;

  ret = create_logpath(filename, out->path);
  if (ret < 0)
    {
      return ret;
    }

  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGREMOVE;
    }
  else if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGREMOVE_V4;
    }
  else
    {
      size = -ENOSYS;
    }

  return size;
}

#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int32_t errinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR lte_errinfo_t *info = (FAR lte_errinfo_t *)arg[0];
  FAR struct apicmd_cmddat_errinfo_s *in =
    (FAR struct apicmd_cmddat_errinfo_s *)pktbuf;

  info->err_indicator = in->indicator;
  info->err_result_code = ntohl(in->err_code);
  info->err_no = ntohl(in->err_no);
  memcpy(info->err_string, in->err_str, LTE_ERROR_STRING_MAX_LEN);
  info->err_string[LTE_ERROR_STRING_MAX_LEN - 1] = '\0';

  return 0;
}

static int32_t getver_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getphone_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *errcause = (FAR uint8_t *)arg[1];
  FAR char *phoneno = (FAR char *)arg[2];
  FAR struct apicmd_cmddat_phonenores_s *in =
    (FAR struct apicmd_cmddat_phonenores_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  *errcause = in->errcause;
  if (0 == *ret)
    {
      if (arglen > 3)
        {
          FAR size_t *len = (FAR size_t *)arg[3];

          /* Is it enough length to include Null terminate?
           * The length of LTE_PHONENO_LEN includes the
           * null terminator.
           */

          if (*len < strnlen((FAR const char *)in->phoneno,
            LTE_PHONENO_LEN))
            {
              return -ENOBUFS;
            }
        }

      strncpy(phoneno, (FAR const char *)in->phoneno, LTE_PHONENO_LEN);
    }

  return 0;
}

static int32_t getimsi_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR uint8_t *errcause = (FAR uint8_t *)arg[1];
  FAR char *imsi = (FAR char *)arg[2];
  FAR struct apicmd_cmddat_getimsires_s *in =
    (FAR struct apicmd_cmddat_getimsires_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;
  *errcause = in->errcause;
  if (0 == *ret)
    {
      if (arglen > 3)
        {
          FAR size_t *len = (FAR size_t *)arg[3];

          /* Is it enough length to include Null terminate?
           * The length of APICMD_IMSI_LEN includes the
           * null terminator.
           */

          if (*len < strnlen((FAR const char *)in->imsi,
            APICMD_IMSI_LEN))
            {
              return -ENOBUFS;
            }
        }

      strncpy(imsi, (FAR const char *)in->imsi, APICMD_IMSI_LEN);
    }

  return 0;
}

static int32_t getimei_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getpinset_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setpinlock_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setpincode_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t enterpin_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getltime_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getoper_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR char *oper = (FAR char *)arg[1];

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
  else if (altver == ALTCOM_VER4)
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

  return 0;
}

static int32_t setrepqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setrepquality_res_s *in =
    (FAR struct apicmd_cmddat_setrepquality_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EIO;

  return 0;
}

static int32_t setrepcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setrepcellinfo_res_s *in =
    (FAR struct apicmd_cmddat_setrepcellinfo_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EIO;

  return 0;
}

static int32_t setrepevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_setrepevtres_s *in =
        (FAR struct apicmd_cmddat_setrepevtres_s *)pktbuf;

      *ret = (APICMD_SET_REP_EVT_RES_OK == in->result) ? 0 : -EIO;
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_setrepevtres_v4_s *in =
        (FAR struct apicmd_cmddat_setrepevtres_v4_s *)pktbuf;

      *ret = (APICMD_SET_REP_EVT_RES_OK == in->result) ? 0 : -EIO;
    }

  return 0;
}

static int32_t repevt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = 0;
  FAR uint8_t *flag  = (FAR uint8_t *)arg[0];
  FAR uint32_t *simstat = (FAR uint32_t *)arg[1];
  FAR lte_localtime_t *ltime = (FAR lte_localtime_t *)arg[2];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_repevt_s *in =
        (FAR struct apicmd_cmddat_repevt_s *)pktbuf;

      switch (in->type)
        {
          case APICMD_REPORT_EVT_TYPE_LTIME:
            {
              parse_ltime(&in->u.ltime, ltime);
              *flag |= ALTCOM_REPEVT_FLAG_LTIME;
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMD:
            {
              ret = parse_simd(&in->u.simd, simstat);
              if (ret == 0)
                {
                  *flag |= ALTCOM_REPEVT_FLAG_SIMSTAT;
                }
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMSTATE:
            {
              ret = parse_simstate(&in->u.simstate, simstat);
              if (ret == 0)
                {
                  *flag |= ALTCOM_REPEVT_FLAG_SIMSTAT;
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
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_repevt_v4_s *in =
        (FAR struct apicmd_cmddat_repevt_v4_s *)pktbuf;

      switch (ntohs(in->type))
        {
          case APICMD_REPORT_EVT_TYPE_LTIME:
            {
              parse_ltime(&in->u.ltime, ltime);
              *flag |= ALTCOM_REPEVT_FLAG_LTIME;
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMD:
            {
              ret = parse_simd(&in->u.simd, simstat);
              if (ret == 0)
                {
                  *flag |= ALTCOM_REPEVT_FLAG_SIMSTAT;
                }
            }
            break;

          case APICMD_REPORT_EVT_TYPE_SIMSTATE:
            {
              ret = parse_simstate(&in->u.simstate, simstat);
              if (ret == 0)
                {
                  *flag |= ALTCOM_REPEVT_FLAG_SIMSTAT;
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

  return ret;
}

static int32_t repqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t repcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  struct cellinfo_helper_s
    {
      lte_cellinfo_t info;
      lte_neighbor_cell_t neighbors[LTE_NEIGHBOR_CELL_MAX];
    };

  FAR lte_cellinfo_t *cellinfo = (FAR lte_cellinfo_t *)arg[0];

  if (altver == ALTCOM_VER1)
    {
      FAR struct apicmd_cmddat_cellinfo_s *in =
        (FAR struct apicmd_cmddat_cellinfo_s *)pktbuf;

      cellinfo->nr_neighbor = 0;

      altcombs_set_cellinfo(in, cellinfo);
    }
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_cellinfo_v4_s *in =
        (FAR struct apicmd_cmddat_cellinfo_v4_s *)pktbuf;

      cellinfo->nr_neighbor = LTE_NEIGHBOR_CELL_MAX;

      altcombs_set_cellinfo_v4(in, cellinfo);
    }

  return 0;
}

static int32_t getedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setedrxres_s *in =
    (FAR struct apicmd_cmddat_setedrxres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t getpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setpsmres_s *in =
    (FAR struct apicmd_cmddat_setpsmres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t getce_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setce_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_setceres_s *in =
    (FAR struct apicmd_cmddat_setceres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t radioon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_radioonres_s *in =
    (FAR struct apicmd_cmddat_radioonres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t radiooff_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_radiooffres_s *in =
    (FAR struct apicmd_cmddat_radiooffres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t actpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_pdn_t *pdn = (FAR lte_pdn_t *)arg[1];

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
  else if (altver == ALTCOM_VER4)
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

  return 0;
}

static int32_t deactpdn_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_deactivatepdnres_s *in =
    (FAR struct apicmd_cmddat_deactivatepdnres_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  return 0;
}

static int32_t getnetinfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_netinfo_t *info = (FAR lte_netinfo_t *)arg[1];
  FAR uint8_t *pdn_num = (FAR uint8_t *)arg[2];

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
  else if (altver == ALTCOM_VER4)
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

  return 0;
}

static int32_t getimscap_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t setrepnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR struct apicmd_cmddat_set_repnetinfores_s *in =
    (FAR struct apicmd_cmddat_set_repnetinfores_s *)pktbuf;

  *ret = (APICMD_REPNETINFO_RES_OK == in->result ? 0 : -EIO);

  return 0;
}

static int32_t repnet_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR lte_netinfo_t *netinfo = (FAR lte_netinfo_t *)arg[0];
  FAR uint8_t *ndnsaddrs = (FAR uint8_t *)arg[1];
  FAR struct sockaddr_storage *dnsaddrs =
    (FAR struct sockaddr_storage *)arg[2];
  int i = 0;

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
  else if (altver == ALTCOM_VER4)
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

  return 0;
}

static int32_t getsiminfo_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_siminfo_t *siminfo = (FAR lte_siminfo_t *)arg[1];
  FAR struct apicmd_cmddat_getsiminfo_res_s *in =
    (FAR struct apicmd_cmddat_getsiminfo_res_s *)pktbuf;

  *ret = (LTE_RESULT_OK == in->result) ? 0 : -EPROTO;

  if (0 == *ret)
    {
      /* Parse SIM information */

      getsiminfo_parse_response(in, siminfo);
    }

  return 0;
}

static int32_t getdedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getdpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getqual_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_quality_t *quality = (FAR lte_quality_t *)arg[1];

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
  else if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getqualityres_v4_s *in =
        (FAR struct apicmd_cmddat_getqualityres_v4_s *)pktbuf;

      *ret = 0;

      /* Parse quality information */

      altcombs_set_quality(quality, &in->quality);
    }

  return 0;
}

static int32_t actpdncancel_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t getcell_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_cellinfo_t *cellinfo = (FAR lte_cellinfo_t *)arg[1];

  if (altver == ALTCOM_VER4)
    {
      FAR struct apicmd_cmddat_getcellinfores_v4_s *in =
        (FAR struct apicmd_cmddat_getcellinfores_v4_s *)pktbuf;

      /* Parse LTE network cell information */

      altcombs_set_cellinfo_v4(&in->cellinfo, cellinfo);

      *ret = 0;
    }

  return 0;
}

static int32_t getrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];
  FAR lte_ratinfo_t *ratinfo = (FAR lte_ratinfo_t *)arg[1];

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

  return 0;
}

static int32_t setrat_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int *ret = (FAR int *)arg[0];

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

  return 0;
}

static int32_t sockcomm_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];

  FAR struct altmdmpktr_sockcomm_s *in =
    (FAR struct altmdmpktr_sockcomm_s *)pktbuf;

  *ret = ntohl(in->ret_code);
  *errcode = ntohl(in->err_code);

  return 0;
}

static int32_t scokaddr_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *addrlen = (FAR uint32_t *)arg[2];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[3];

  FAR struct altmdmpktr_sockaddr_s *in =
    (FAR struct altmdmpktr_sockaddr_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  *ret = ntohl(in->ret_code);
  *errcode = ntohl(in->err_code);

  if (*ret >= 0)
    {
      *addrlen = ntohl(in->addrlen);
      if (*addrlen == sizeof(struct altcom_sockaddr_in))
        {
          *addrlen = sizeof(struct sockaddr_in);
        }
      else if (*addrlen == sizeof(struct altcom_sockaddr_in6))
        {
          *addrlen = sizeof(struct sockaddr_in6);
        }
      else
        {
          rc = -EILSEQ;
        }

      memcpy(&altsa, &in->address, sizeof(struct altcom_sockaddr_storage));
      altstorage2sockaddr(&altsa, (FAR struct sockaddr *)sa);
    }

  return rc;
}

static int32_t getsockopt_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t  *ret     = (FAR int32_t *)arg[0];
  FAR int32_t  *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *optlen  = (FAR uint32_t *)arg[2];
  FAR int8_t   *optval  = (FAR int8_t *)arg[3];
  FAR uint16_t *level   = (FAR uint16_t *)arg[4];
  FAR uint16_t *option  = (FAR uint16_t *)arg[5];

  FAR struct apicmd_getsockoptres_s *in =
    (FAR struct apicmd_getsockoptres_s *)pktbuf;

  int setmode = 0;

  *ret = ntohl(in->ret_code);
  *errcode = ntohl(in->err_code);

  if (*ret >= 0)
    {
      *optlen = ntohl(in->optlen);
      if (*optlen > APICMD_OPTVAL_LENGTH)
        {
          rc = -EILSEQ;
        }
      else
        {
          setmode = get_so_setmode(*level, *option);

          switch (setmode)
            {
              case ALTCOM_SO_SETMODE_8BIT:
                if (*optlen < sizeof(int8_t))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(int8_t));
                    rc = -EILSEQ;
                    break;
                  }

                *optval = in->optval[0];
                break;
              case ALTCOM_SO_SETMODE_32BIT:
                if (*optlen < sizeof(int32_t))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(int32_t));
                    rc = -EILSEQ;
                    break;
                  }

                *((FAR int32_t *)optval) =
                  ntohl(*((FAR int32_t *)in->optval));
                break;
              case ALTCOM_SO_SETMODE_LINGER:
                if (*optlen < sizeof(struct linger))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(struct linger));
                    rc = -EILSEQ;
                    break;
                  }

                FAR struct altcom_linger *plinger;

                plinger = (FAR struct altcom_linger *)&in->optval[0];
                ((FAR struct linger *)optval)->l_onoff =
                  ntohl(plinger->l_onoff);
                ((FAR struct linger *)optval)->l_linger =
                  ntohl(plinger->l_linger);
                break;
              case ALTCOM_SO_SETMODE_INADDR:
                if (*optlen < sizeof(struct in_addr))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(struct in_addr));
                    rc = -EILSEQ;
                    break;
                  }

                FAR struct altcom_in_addr *pinaddr;

                pinaddr = (FAR struct altcom_in_addr *)&in->optval[0];
                ((FAR struct in_addr *)optval)->s_addr =
                  ntohl(pinaddr->s_addr);
                break;
              default:
                break;
            }
        }
    }

  return rc;
}

static int32_t recvfrom_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *fromlen = (FAR uint32_t *)arg[2];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[3];
  FAR int8_t *buf = (FAR int8_t *)arg[4];

  FAR struct apicmd_recvfromres_s *in =
    (FAR struct apicmd_recvfromres_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  *ret = ntohl(in->ret_code);
  *errcode = ntohl(in->err_code);

  if (*ret >= 0)
    {
      *fromlen = ntohl(in->fromlen);
      if (*fromlen == sizeof(struct altcom_sockaddr_in))
        {
          *fromlen = sizeof(struct sockaddr_in);
        }
      else if (*fromlen == sizeof(struct altcom_sockaddr_in6))
        {
          *fromlen = sizeof(struct sockaddr_in6);
        }
      else if (*fromlen != 0)
        {
          rc = -EILSEQ;
        }

      if ((rc == OK) && (*fromlen != 0))
        {
          memcpy(&altsa, &in->from, *fromlen);
          altstorage2sockaddr(&altsa, (FAR struct sockaddr *)sa);
        }

      if (*ret > APICMD_DATA_LENGTH)
        {
          rc = -EILSEQ;
        }
      else
        {
          memcpy(buf, in->recvdata, *ret);
        }
    }

  return rc;
}

static int32_t select_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR int32_t *id = (FAR int32_t *)arg[2];
  FAR altcom_fd_set *readset = (FAR altcom_fd_set *)arg[3];
  FAR altcom_fd_set *writeset = (FAR altcom_fd_set *)arg[4];
  FAR altcom_fd_set *exceptset = (FAR altcom_fd_set *)arg[5];

  FAR struct apicmd_selectres_s *in =
    (FAR struct apicmd_selectres_s *)pktbuf;
  uint16_t used_setbit;

  *ret = ntohl(in->ret_code);
  *errcode = ntohl(in->err_code);

  if (*ret >= 0)
    {
      *id = ntohl(in->id);
      used_setbit = ntohs(in->used_setbit);
      memset(readset, 0, sizeof(altcom_fd_set));
      memset(writeset, 0, sizeof(altcom_fd_set));
      memset(exceptset, 0, sizeof(altcom_fd_set));
      if (used_setbit & READSET_BIT)
        {
          memcpy(readset, &in->readset, sizeof(altcom_fd_set));
        }

      if (used_setbit & WRITESET_BIT)
        {
          memcpy(writeset, &in->writeset, sizeof(altcom_fd_set));
        }

      if (used_setbit & EXCEPTSET_BIT)
        {
          memcpy(exceptset, &in->exceptset, sizeof(altcom_fd_set));
        }
    }

  return 0;
}

static int32_t sendatcmd_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
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

static int32_t fwcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t result_cmd;
  int16_t injection_retcode;

  FAR struct apicmd_cmddat_fw_deltaupcommres_s *in =
    (FAR struct apicmd_cmddat_fw_deltaupcommres_s *)pktbuf;

  /* Negative value in result_cmd means an error is occured.
   * Zero indicates command successed or size of injected data
   */

  result_cmd = ntohl(in->api_result);
  injection_retcode = ntohs(in->ltefw_result);

  return (injection_retcode != 0) ? -injection_retcode : result_cmd;
}

static int32_t smscommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_sms_res_s *in =
    (FAR struct apicmd_sms_res_s *)pktbuf;

  return ntohl(in->result);
}

static int32_t smssend_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_sms_sendres_s *in =
    (FAR struct apicmd_sms_sendres_s *)pktbuf;
  FAR struct sms_refids_s *refid = (FAR struct sms_refids_s *)arg[0];
  uint16_t msglen = *((FAR uint16_t *)arg[1]);
  int32_t sendresult = ntohl(in->result);

  if (sendresult >= 0)
    {
      refid->nrefid = in->mr_num;
      memcpy(refid->refid, in->mr_list, sizeof(refid->refid));
    }

  return (sendresult < 0) ? sendresult : msglen;
}

static int32_t smsreportrecv_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int i;
  FAR struct apicmd_sms_reprecv_s *in =
    (FAR struct apicmd_sms_reprecv_s *)pktbuf;
  FAR uint16_t *msg_index = (FAR uint16_t *)arg[0];
  FAR uint16_t *msg_sz = (FAR uint16_t *)arg[1];
  FAR uint8_t *maxnum = (FAR uint8_t *)arg[2];
  FAR uint8_t *seqnum = (FAR uint8_t *)arg[3];
      FAR struct sms_recv_msg_header_s *msgheader =
        (FAR struct sms_recv_msg_header_s *)arg[4];

  *msg_index = ntohs(in->index);
  *maxnum = 0;
  *seqnum = 0;

  if (in->msg.type == ALTCOM_SMS_MSG_TYPE_RECV)
    {
      FAR struct sms_deliver_msg_s *deliver =
        (FAR struct sms_deliver_msg_s *)arg[4];

      *msg_sz = sizeof(struct sms_deliver_msg_s) +
        ntohs(in->msg.u.recv.userdata.data_len);

      if (in->msg.u.recv.valid_indicator & ALTCOM_SMS_MSG_VALID_CONCAT_HDR)
        {
          *maxnum = in->msg.u.recv.concat_hdr.max_num;
          *seqnum = in->msg.u.recv.concat_hdr.seq_num;
        }

      msgheader->msgtype = SMS_MSG_TYPE_DELIVER;
      memcpy(&msgheader->send_time, &in->msg.u.recv.sc_time,
             sizeof(msgheader->send_time));
      msgheader->srcaddrlen = in->msg.u.recv.src_addr.length;
      if (msgheader->srcaddrlen > SMS_MAX_ADDRLEN * 2)
        {
          m_err("Unexpected src addrlen: %u\n", msgheader->srcaddrlen);
          return -EINVAL;
        }

      /* Swap source address */

      for (i = 0; i < (msgheader->srcaddrlen / 2); i++)
        {
          msgheader->srcaddr[i] = ntohs(in->msg.u.recv.src_addr.address[i]);
        }

      msgheader->datalen = ntohs(in->msg.u.recv.userdata.data_len);
      if (msgheader->datalen > (SMS_MAX_DATALEN * 2))
        {
          m_err("Unexpected datalen: %u\n", msgheader->datalen);
          return -EINVAL;
        }

      /* Swap data */

      for (i = 0; i < (msgheader->datalen / 2); i++)
        {
          deliver->data[i] = ntohs(in->msg.u.recv.user_data[i]);
        }

      m_info("[recv msg] msg size: %u\n", *msg_sz);
      m_info("           maxnum: %u, seqnum: %u\n", *maxnum, *seqnum);
      m_info("           msgtype: %u\n", msgheader->msgtype);
      m_info("           srcaddrlen: %u\n", msgheader->srcaddrlen);
      m_info("           datalen: %u\n", msgheader->datalen);
    }
  else if (in->msg.type == ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT)
    {
      FAR struct sms_status_report_msg_s *report =
        (FAR struct sms_status_report_msg_s *)arg[4];

      *msg_sz = sizeof(struct sms_status_report_msg_s);

      msgheader->msgtype = SMS_MSG_TYPE_STATUS_REPORT;
      memcpy(&msgheader->send_time, &in->msg.u.delivery_report.sc_time,
             sizeof(msgheader->send_time));
      msgheader->srcaddrlen = 0;
      memset(msgheader->srcaddr, 0, sizeof(msgheader->srcaddr));
      msgheader->datalen = sizeof(struct sms_status_report_s);
      report->status_report.refid = in->msg.u.delivery_report.ref_id;
      report->status_report.status = in->msg.u.delivery_report.status;
      memcpy(&report->status_report.discharge_time,
             &in->msg.u.delivery_report.discharge_time,
             sizeof(report->status_report.discharge_time));

      m_info("[staus report] msg size: %u\n", *msg_sz);
      m_info("           msgtype: %u\n", msgheader->msgtype);
      m_info("           datalen: %u\n", msgheader->datalen);
      m_info("           refid: %u\n", report->status_report.refid);
      m_info("           status: %u\n", report->status_report.status);
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}

static int32_t urc_event_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  int32_t ret = -ENOTSUP;
  uint32_t lcmdid = 0;
  alt_evtbuf_inst_t *inst;
  lwm2mstub_hndl_t lwm2m_urc_handler;

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

  return ret;
}

static int32_t logsave_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_cmddat_clogsres_s *out =
    (FAR struct apicmd_cmddat_clogsres_s *)pktbuf;
  FAR char *fname = (FAR char *)arg[0];
  size_t fnamelen = *(FAR size_t *)arg[1];

  int32_t ret = ntohl(out->altcom_result);

  if ((ret == 0) && (fname != NULL))
    {
      if (ALTCOM_PATH_LEN_MAX > out->pathlen)
        {
          ret = copy_logfilename(fname, fnamelen, out->path);
        }
      else
        {
          ret = -EILSEQ;
        }
    }

  return ret;
}

static int32_t loglist_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmddbg_getloglistres_s *out =
    (FAR struct apicmddbg_getloglistres_s *)pktbuf;
  FAR char *list = (FAR char *)arg[0];
  size_t nlists = (size_t)arg[1];
  size_t fnamelen = (size_t)arg[2];
  int32_t ret = ntohl(out->altcom_result);
  int i;

  if (fnamelen != LTE_LOG_NAME_LEN)
    {
      return -ENOBUFS;
    }

  if (ret == 0)
    {
      if ((out->listsize > LTE_LOG_LIST_SIZE) ||
          ((out->listsize != 0) && (ALTCOM_PATH_LEN_MAX < out->pathlen)))
        {
          return -EILSEQ;
        }

      for (i = 0; i < MIN(nlists, out->listsize); i++)
        {
          ret = copy_logfilename(&list[i * fnamelen], fnamelen,
                                 &out->list[i * out->pathlen]);
          if (ret != OK)
            {
              break;
            }
        }

      ret = (i == MIN(nlists, out->listsize)) ? i : ret;
    }
  else if (ret == -EPROTO)
    {
      ret = 0;
    }

  return ret;
}

#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS

static int32_t logcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_logcommonres_s *out =
    (FAR struct apicmd_logcommonres_s *)pktbuf;

  return ntohl(out->altcom_result);
}

static int32_t logread_pkt_parse(FAR struct alt1250_dev_s *dev,
                          FAR uint8_t *pktbuf,
                          size_t pktsz, uint8_t altver, FAR void **arg,
                          size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_logreadres_s *out =
    (FAR struct apicmd_logreadres_s *)pktbuf;
  FAR void *buf = arg[0];
  size_t len = (size_t)arg[1];
  int32_t ret = ntohl(out->altcom_result);

  if (ret > 0)
    {
      if (ret <= len)
        {
          memcpy(buf, out->readdata, ret);
        }
      else
        {
          ret = -EILSEQ;
        }
    }

  return ret;
}

#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

compose_handler_t alt1250_composehdlr(uint32_t cmdid)
{
  int i;
  compose_handler_t ret = NULL;

  for (i = 0; i < ARRAY_SZ(g_composehdlrs); i++)
    {
      if (g_composehdlrs[i].cmdid == cmdid)
        {
          ret = g_composehdlrs[i].hdlr;
        }
    }

  return ret;
}

parse_handler_t alt1250_parsehdlr(uint16_t altcid, uint8_t altver)
{
  int i;
  parse_handler_t ret = NULL;

  altcid &= ~ALTCOM_CMDID_REPLY_BIT;

  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1 */

      altcid = convert_cid2v1(altcid);
      if (altcid == APICMDID_UNKNOWN)
        {
          return NULL;
        }
    }

  for (i = 0; i < ARRAY_SZ(g_parsehdlrs); i++)
    {
      if (g_parsehdlrs[i].altcid == altcid)
        {
          ret = g_parsehdlrs[i].hdlr;
        }
    }

  return ret;
}
