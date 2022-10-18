/****************************************************************************
 * drivers/modem/alt1250/altcom_lwm2m_hdlr.c
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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/wireless/lte/lte_ioctl.h>

#include "altcom_lwm2m_hdlr.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t read_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t write_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t exec_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t start_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t stop_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t fwupdate_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t server_op_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct urc_hdltbl_s
{
  FAR const char *head;
  uint32_t lcmdid;
  lwm2mstub_hndl_t hdlr;
};

struct lwm2mstub_instance_s
{
  int object_id;
  int object_inst;
  int res_id;
  int res_inst;
};

struct lwm2mstub_ovcondition_s
{
    uint8_t valid_mask;
    unsigned int min_period;
    unsigned int max_period;
    double gt_cond;
    double lt_cond;
    double step_val;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct urc_hdltbl_s urc_idhandles[] =
{
  { "%LWM2MOBJCMDU: \"READ\",",
    LTE_CMDID_LWM2M_READ_EVT, read_request_hndl },

  { "%LWM2MOBJCMDU: \"WRITE\",",
    LTE_CMDID_LWM2M_WRITE_EVT, write_request_hndl },

  { "%LWM2MOBJCMDU: \"EXE\",",
    LTE_CMDID_LWM2M_EXEC_EVT, exec_request_hndl },

  { "%LWM2MOBJCMDU: \"OBSERVE_START\",",
    LTE_CMDID_LWM2M_OVSTART_EVT, start_ov_request_hndl },

  { "%LWM2MOBJCMDU: \"OBSERVE_STOP\",",
    LTE_CMDID_LWM2M_OVSTOP_EVT, stop_ov_request_hndl },

  { "%LWM2MOPEV: ",
    LTE_CMDID_LWM2M_SERVEROP_EVT, server_op_notice_hndl },

  { "%LWM2MEV: ",
    LTE_CMDID_LWM2M_FWUP_EVT, fwupdate_notice_hndl },

  {
    NULL, 0, NULL
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * name: skip_until
 ****************************************************************************/

static FAR uint8_t *skip_until(FAR uint8_t *stream, FAR char *delim)
{
  for (; *stream && !strchr(delim, *stream); stream++);
  for (; *stream && strchr(delim, *stream); stream++);

  if (*stream == '\0')
    {
      stream = NULL;
    }

  return stream;
}

/****************************************************************************
 * name: strcpy_until
 ****************************************************************************/

static FAR char strcpy_until(char *dst, int n, char **src, char *delim)
{
  char *tmp = *src;

  if (dst)
    {
      dst[n - 1] = '\0';
      n--;
    }

  while (*tmp && !strchr(delim, *tmp))
    {
      if (dst && (n > 0))
        {
          *dst++ = *tmp;
          n--;
        }

      tmp++;
    }

  if (dst && (n > 0))
    {
      *dst = '\0';
    }

  *src = tmp + 1;

  return *tmp;
}

/****************************************************************************
 * name: parse_instance
 ****************************************************************************/

static FAR uint8_t *parse_instance(uint8_t *pktbuf, int *seq_no, int *srv_id,
                           struct lwm2mstub_instance_s *inst)
{
  *seq_no = atoi((char *)pktbuf); /* for seq_no */
  pktbuf = skip_until(pktbuf, ",");
  *srv_id = atoi((char *)pktbuf); /* for srv_id */
  pktbuf = skip_until(pktbuf, ",");

  /* Parse URI like /objid/objinst/resid */

  pktbuf = skip_until(pktbuf, "/");
  inst->object_id = atoi((char *)pktbuf);
  pktbuf = skip_until(pktbuf, "/");
  inst->object_inst = atoi((char *)pktbuf);
  pktbuf = skip_until(pktbuf, "/");
  inst->res_id = atoi((char *)pktbuf);

  inst->res_inst = -1;
  if (skip_until(pktbuf, "/") != NULL)
    {
      pktbuf = skip_until(pktbuf, "/");
      inst->res_inst = atoi((char *)pktbuf);
    }

  pktbuf = skip_until(pktbuf, ",/\r\n");

  return pktbuf;
}

/****************************************************************************
 * name: read_request_hndl
 ****************************************************************************/

static int32_t read_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  parse_instance(pktbuf, (int *)&cb_args[0], (int *)&cb_args[1],
                  (struct lwm2mstub_instance_s *)(cb_args[2]));

  return 0;
}

/****************************************************************************
 * name: write_request_hndl
 ****************************************************************************/

static int32_t write_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  if (!cb_args[3] && ((int)cb_args[5]) <= 0)
    {
      return -1;
    }

  pktbuf = parse_instance(pktbuf, (int *)&cb_args[0], (int *)&cb_args[1],
                          (struct lwm2mstub_instance_s *)(cb_args[2]));

  if (*pktbuf == '\"')
    {
      pktbuf++;
    }

  strcpy_until((char *)cb_args[3], (int)cb_args[5], (char **)&pktbuf,
               "\",\r\n");

  cb_args[4] = (void *)strlen((char *)cb_args[3]);

  return 0;
}

/****************************************************************************
 * name: exec_request_hndl
 ****************************************************************************/

static int32_t exec_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  parse_instance(pktbuf, (int *)&cb_args[0], (int *)&cb_args[1],
                  (struct lwm2mstub_instance_s *)(cb_args[2]));

  return 0;
}

/****************************************************************************
 * name: parse_observe
 ****************************************************************************/

static uint8_t *parse_observe(uint8_t *pktbuf, int *seq_no, int *srv_id,
                           int tksize, char *token,
                           struct lwm2mstub_instance_s *inst)
{
  *seq_no = atoi((char *)pktbuf); /* for seq_no */
  pktbuf = skip_until(pktbuf, ",");
  *srv_id = atoi((char *)pktbuf); /* for server id */
  pktbuf = skip_until(pktbuf, "\"");
  strcpy_until(token, tksize, (char **)&pktbuf, "\"");
  pktbuf = skip_until(pktbuf, ",");
  pktbuf = skip_until(pktbuf, "/");

  inst->object_id   = -1;
  inst->object_inst = -1;
  inst->res_id      = -1;
  inst->res_inst    = -1;

  inst->object_id = atoi((char *)pktbuf);
  if (strcpy_until(NULL, 0, (char **)&pktbuf, "/\",") == '/')
    {
      inst->object_inst = atoi((char *)pktbuf);
      if (strcpy_until(NULL, 0, (char **)&pktbuf, "/\",") == '/')
        {
          inst->res_id = atoi((char *)pktbuf);
          if (strcpy_until(NULL, 0, (char **)&pktbuf, "/\",") == '/')
            {
              inst->res_inst = atoi((char *)pktbuf);
            }
        }
    }

  return pktbuf;
}

/****************************************************************************
 * name: start_ov_request_hndl
 ****************************************************************************/

static int32_t start_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  struct lwm2mstub_ovcondition_s *cond
    = (struct lwm2mstub_ovcondition_s *)cb_args[5];

  parse_observe(pktbuf,
                (int *)&cb_args[0], (int *)&cb_args[1],
                (int)cb_args[4], (char *)cb_args[3],
                (struct lwm2mstub_instance_s *)cb_args[2]);

  cond->valid_mask = 0;

  return 0;
}

/****************************************************************************
 * name: stop_ov_request_hndl
 ****************************************************************************/

static int32_t stop_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  parse_observe(pktbuf,
                (int *)&cb_args[0], (int *)&cb_args[1],
                (int)cb_args[4], (char *)cb_args[3],
                (struct lwm2mstub_instance_s *)cb_args[2]);

  return 0;
}

/****************************************************************************
 * name: fwup_srvop_handle
 ****************************************************************************/

static int32_t fwup_srvop_handle(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  uint8_t *ep;
  FAR int *event = (FAR int *)&cb_args[0];

  /* Expected unsolicited event
   *    %LWM2MOPEV: <event>[,....
   *    %LWM2MEV: <event>[,....
   */

  *event = strtol((const char *)pktbuf, (char **)&ep, 10);
  if ((*event == 0) && (pktbuf == ep))
    {
      return -1;
    }

  return 1;
}

/****************************************************************************
 * name: fwupdate_notice_hndl
 ****************************************************************************/

static int32_t fwupdate_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return fwup_srvop_handle(pktbuf, pktsz, cb_args, arglen);
}

/****************************************************************************
 * name: server_op_notice_hndl
 ****************************************************************************/

static int32_t server_op_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return fwup_srvop_handle(pktbuf, pktsz, cb_args, arglen);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: lwm2mstub_get_handler
 ****************************************************************************/

lwm2mstub_hndl_t lwm2mstub_get_handler(FAR uint8_t **pktbuf, size_t *pktsz,
                                      uint32_t *lcmdid)
{
  char *head_pos;
  struct urc_hdltbl_s *tbl;
  size_t shift_size = 0;

  *lcmdid = 0;
  tbl = urc_idhandles;

  while (tbl->head)
    {
      head_pos = strstr((char *)*pktbuf, tbl->head);
      if (head_pos)
        {
          shift_size = head_pos - (char *)*pktbuf + strlen(tbl->head);

          /* Follow shift_size to advance them */

          *pktbuf += shift_size;
          *pktsz -= shift_size;

          *lcmdid = tbl->lcmdid;
          return tbl->hdlr;
        }

      tbl++;
    }

  return NULL;
}
