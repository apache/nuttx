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
#include <stddef.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/wireless/lte/lte_ioctl.h>

#include "altcom_pkt.h"
#include "altcom_hdlr_pdn.h"
#include "altcom_hdlr_radio.h"
#include "altcom_hdlr_net.h"
#include "altcom_hdlr_psave.h"
#include "altcom_hdlr_sim.h"
#include "altcom_hdlr_pin.h"
#include "altcom_hdlr_socket.h"
#include "altcom_hdlr_sms.h"
#include "altcom_hdlr_firmware.h"
#include "altcom_hdlr_log.h"
#include "altcom_hdlr_other.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define CTABLE_CONTENT(cmdid, hdlrname) \
    { LTE_CMDID_##cmdid, hdlrname##_pkt_compose }
#define PTABLE_CONTENT(altcid, hdlrname) \
    { APICMDID_##altcid, hdlrname##_pkt_parse }

static compose_inst_t g_composehdlrs[] =
{
  CTABLE_CONTENT(GETVER, altcom_getver),
  CTABLE_CONTENT(GETPHONE, altcom_getphone),
  CTABLE_CONTENT(GETIMSI, altcom_getimsi),
  CTABLE_CONTENT(GETIMEI, altcom_getimei),
  CTABLE_CONTENT(GETPINSET, altcom_getpinset),
  CTABLE_CONTENT(PINENABLE, altcom_setpinlock),
  CTABLE_CONTENT(CHANGEPIN, altcom_setpincode),
  CTABLE_CONTENT(ENTERPIN, altcom_enterpin),
  CTABLE_CONTENT(GETLTIME, altcom_getltime),
  CTABLE_CONTENT(GETOPER, altcom_getoper),
  CTABLE_CONTENT(REPQUAL, altcom_setrepqual),
  CTABLE_CONTENT(REPCELL, altcom_setrepcell),
  CTABLE_CONTENT(REPSIMSTAT, altcom_setrepevt),
  CTABLE_CONTENT(REPLTIME, altcom_setrepevt),
  CTABLE_CONTENT(GETEDRX, altcom_getedrx),
  CTABLE_CONTENT(SETEDRX, altcom_setedrx),
  CTABLE_CONTENT(GETPSM, altcom_getpsm),
  CTABLE_CONTENT(SETPSM, altcom_setpsm),
  CTABLE_CONTENT(GETCE, altcom_getce),
  CTABLE_CONTENT(SETCE, altcom_setce),
  CTABLE_CONTENT(RADIOON, altcom_radioon),
  CTABLE_CONTENT(RADIOOFF, altcom_radiooff),
  CTABLE_CONTENT(ACTPDN, altcom_actpdn),
  CTABLE_CONTENT(DEACTPDN, altcom_deactpdn),
  CTABLE_CONTENT(GETNETINFO, altcom_getnetinfo),
  CTABLE_CONTENT(IMSCAP, altcom_getimscap),
  CTABLE_CONTENT(REPNETINFO, altcom_setrepnet),
  CTABLE_CONTENT(GETSIMINFO, altcom_getsiminfo),
  CTABLE_CONTENT(GETCEDRX, altcom_getdedrx),
  CTABLE_CONTENT(GETCPSM, altcom_getdpsm),
  CTABLE_CONTENT(GETQUAL, altcom_getqual),
  CTABLE_CONTENT(ACTPDNCAN, altcom_actpdncancel),
  CTABLE_CONTENT(GETCELL, altcom_getcell),
  CTABLE_CONTENT(GETRAT, altcom_getrat),
  CTABLE_CONTENT(GETRATINFO, altcom_getrat),
  CTABLE_CONTENT(SETRAT, altcom_setrat),
  CTABLE_CONTENT(ACCEPT, altcom_accept),
  CTABLE_CONTENT(BIND, altcom_bind),
  CTABLE_CONTENT(CLOSE, altcom_close),
  CTABLE_CONTENT(CONNECT, altcom_connect),
  CTABLE_CONTENT(FCNTL, altcom_fcntl),
  CTABLE_CONTENT(GETSOCKNAME, altcom_getsockname),
  CTABLE_CONTENT(GETSOCKOPT, altcom_getsockopt),
  CTABLE_CONTENT(LISTEN, altcom_listen),
  CTABLE_CONTENT(RECVFROM, altcom_recvfrom),
  CTABLE_CONTENT(SELECT, altcom_select),
  CTABLE_CONTENT(SENDTO, altcom_sendto),
  CTABLE_CONTENT(SOCKET, altcom_socket),
  CTABLE_CONTENT(SETSOCKOPT, altcom_setsockopt),
  CTABLE_CONTENT(SENDATCMD, altcom_sendatcmd),
  CTABLE_CONTENT(INJECTIMAGE, altcom_injectimage),
  CTABLE_CONTENT(GETIMAGELEN, altcom_getimagelen),
  CTABLE_CONTENT(EXEUPDATE, altcom_execupdate),
  CTABLE_CONTENT(GETUPDATERES, altcom_getupdateres),
  CTABLE_CONTENT(SMS_INIT, altcom_smsinit),
  CTABLE_CONTENT(SMS_FIN,  altcom_smsfin),
  CTABLE_CONTENT(SMS_SEND, altcom_smssend),
  CTABLE_CONTENT(SMS_DELETE, altcom_smsdelete),
  CTABLE_CONTENT(SMS_REPORT_RECV, altcom_smsreportrecv),
  CTABLE_CONTENT(SAVE_LOG, altcom_logsave),
  CTABLE_CONTENT(GET_LOGLIST, altcom_loglist),
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
  CTABLE_CONTENT(LOGOPEN, altcom_logopen),
  CTABLE_CONTENT(LOGCLOSE, altcom_logclose),
  CTABLE_CONTENT(LOGREAD, altcom_logread),
  CTABLE_CONTENT(LOGLSEEK, altcom_loglseek),
  CTABLE_CONTENT(LOGREMOVE, altcom_logremove),
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */
};

static parse_inst_t g_parsehdlrs[] =
{
  PTABLE_CONTENT(ERRINFO, altcom_errinfo),
  PTABLE_CONTENT(GET_VERSION, altcom_getver),
  PTABLE_CONTENT(GET_PHONENO, altcom_getphone),
  PTABLE_CONTENT(GET_IMSI, altcom_getimsi),
  PTABLE_CONTENT(GET_IMEI, altcom_getimei),
  PTABLE_CONTENT(GET_PINSET, altcom_getpinset),
  PTABLE_CONTENT(SET_PIN_LOCK, altcom_setpinlock),
  PTABLE_CONTENT(SET_PIN_CODE, altcom_setpincode),
  PTABLE_CONTENT(ENTER_PIN, altcom_enterpin),
  PTABLE_CONTENT(GET_LTIME, altcom_getltime),
  PTABLE_CONTENT(GET_OPERATOR, altcom_getoper),
  PTABLE_CONTENT(SET_REP_QUALITY, altcom_setrepqual),
  PTABLE_CONTENT(SET_REP_CELLINFO, altcom_setrepcell),
  PTABLE_CONTENT(SET_REP_EVT, altcom_setrepevt),
  PTABLE_CONTENT(REPORT_EVT, altcom_repevt),
  PTABLE_CONTENT(REPORT_QUALITY, altcom_repqual),
  PTABLE_CONTENT(REPORT_CELLINFO, altcom_repcell),
  PTABLE_CONTENT(GET_EDRX, altcom_getedrx),
  PTABLE_CONTENT(SET_EDRX, altcom_setedrx),
  PTABLE_CONTENT(GET_PSM, altcom_getpsm),
  PTABLE_CONTENT(SET_PSM, altcom_setpsm),
  PTABLE_CONTENT(GET_CE, altcom_getce),
  PTABLE_CONTENT(SET_CE, altcom_setce),
  PTABLE_CONTENT(RADIO_ON, altcom_radioon),
  PTABLE_CONTENT(RADIO_OFF, altcom_radiooff),
  PTABLE_CONTENT(ACTIVATE_PDN, altcom_actpdn),
  PTABLE_CONTENT(DEACTIVATE_PDN, altcom_deactpdn),
  PTABLE_CONTENT(GET_NETINFO, altcom_getnetinfo),
  PTABLE_CONTENT(GET_IMS_CAP, altcom_getimscap),
  PTABLE_CONTENT(SETREP_NETINFO, altcom_setrepnet),
  PTABLE_CONTENT(REPORT_NETINFO, altcom_repnet),
  PTABLE_CONTENT(GET_SIMINFO, altcom_getsiminfo),
  PTABLE_CONTENT(GET_DYNAMICEDRX, altcom_getdedrx),
  PTABLE_CONTENT(GET_DYNAMICPSM, altcom_getdpsm),
  PTABLE_CONTENT(GET_QUALITY, altcom_getqual),
  PTABLE_CONTENT(ACTIVATE_PDN_CANCEL, altcom_actpdncancel),
  PTABLE_CONTENT(GET_CELLINFO, altcom_getcell),
  PTABLE_CONTENT(GET_RAT, altcom_getrat),
  PTABLE_CONTENT(SET_RAT, altcom_setrat),
  PTABLE_CONTENT(SOCK_ACCEPT, altcom_scokaddr),
  PTABLE_CONTENT(SOCK_BIND, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_CLOSE, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_CONNECT, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_FCNTL, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_GETSOCKNAME, altcom_scokaddr),
  PTABLE_CONTENT(SOCK_GETSOCKOPT, altcom_getsockopt),
  PTABLE_CONTENT(SOCK_LISTEN, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_RECVFROM, altcom_recvfrom),
  PTABLE_CONTENT(SOCK_SELECT, altcom_select),
  PTABLE_CONTENT(SOCK_SENDTO, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_SOCKET, altcom_sockcomm),
  PTABLE_CONTENT(SOCK_SETSOCKOPT, altcom_sockcomm),
  PTABLE_CONTENT(SEND_ATCMD, altcom_sendatcmd),
  PTABLE_CONTENT(FW_INJECTDELTAIMG, altcom_fwcommon),
  PTABLE_CONTENT(FW_GETDELTAIMGLEN, altcom_fwcommon),
  PTABLE_CONTENT(FW_EXECDELTAUPDATE, altcom_fwcommon),
  PTABLE_CONTENT(FW_GETUPDATERESULT, altcom_fwcommon),
  PTABLE_CONTENT(SMS_INIT, altcom_smscommon),
  PTABLE_CONTENT(SMS_FIN, altcom_smscommon),
  PTABLE_CONTENT(SMS_SEND, altcom_smssend),
  PTABLE_CONTENT(SMS_DELETE, altcom_smscommon),
  PTABLE_CONTENT(SMS_REPORT_RECV, altcom_smsreportrecv),
  PTABLE_CONTENT(URC_EVENT, altcom_urc_event),
  PTABLE_CONTENT(CLOGS, altcom_logsave),
  PTABLE_CONTENT(LOGLIST, altcom_loglist),
#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS
  PTABLE_CONTENT(LOGOPEN, altcom_logcommon),
  PTABLE_CONTENT(LOGCLOSE, altcom_logcommon),
  PTABLE_CONTENT(LOGREAD, altcom_logread),
  PTABLE_CONTENT(LOGLSEEK, altcom_logcommon),
  PTABLE_CONTENT(LOGREMOVE, altcom_logcommon),
#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */
};

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
