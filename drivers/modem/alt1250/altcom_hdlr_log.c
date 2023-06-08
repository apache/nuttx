/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_log.c
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
#include <stdio.h>
#include <errno.h>
#include <arpa/inet.h>

#include "altcom_cmd_log.h"
#include "altcom_errno.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b)  (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_logsave_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_cmddat_clogs_s *out =
    (FAR struct apicmd_cmddat_clogs_s *)pktbuf;
  size_t len = *(FAR size_t *)arg[0];
  int32_t size = sizeof(struct apicmd_cmddat_clogs_s);

  out->pathlen = len + strlen(ALTCOM_LOGSPATH);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_CLOGS;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_CLOGS_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_loglist_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmddbg_getloglist_s *out =
    (FAR struct apicmddbg_getloglist_s *)pktbuf;
  size_t len = (size_t)arg[0];
  int32_t size = sizeof(struct apicmddbg_getloglist_s);

  out->listsize = LTE_LOG_LIST_SIZE;
  out->pathlen = len + strlen(ALTCOM_LOGSPATH);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGLIST;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGLIST_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

#ifdef CONFIG_MODEM_ALT1250_LOG_ACCESS

int32_t altcom_logopen_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
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

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGOPEN;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGOPEN_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_logclose_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logclose_s *out = (FAR struct apicmd_logclose_s *)pktbuf;
  int fd = (int)arg[0];
  int32_t size = sizeof(struct apicmd_logclose_s);

  out->fd = htonl(fd);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGCLOSE;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGCLOSE_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_logread_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  FAR struct apicmd_logread_s *out = (FAR struct apicmd_logread_s *)pktbuf;
  int fd = (int)arg[0];
  size_t rlen = (size_t)arg[1];
  int32_t size = sizeof(struct apicmd_logread_s);

  out->fd = htonl(fd);
  out->readlen = (rlen > ALTCOM_LOG_READ_LEN_MAX) ?
                  htonl(ALTCOM_LOG_READ_LEN_MAX) : htonl(rlen);

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGREAD;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGREAD_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_loglseek_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
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

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGLSEEK;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGLSEEK_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

int32_t altcom_logremove_pkt_compose(FAR void **arg, size_t arglen,
                                     uint8_t altver, FAR uint8_t *pktbuf,
                                     const size_t pktsz,
                                     FAR uint16_t *altcid)
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

#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV1
  if (altver == ALTCOM_VER1)
    {
      *altcid = APICMDID_LOGREMOVE;
    }
  else
#endif
#ifndef CONFIG_MODEM_ALT1250_DISABLE_PV4
  if (altver == ALTCOM_VER4)
    {
      *altcid = APICMDID_LOGREMOVE_V4;
    }
  else
#endif
    {
      size = -ENOSYS;
    }

  return size;
}

#endif /* CONFIG_MODEM_ALT1250_LOG_ACCESS */

int32_t altcom_logsave_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_cmddat_clogsres_s *out =
    (FAR struct apicmd_cmddat_clogsres_s *)pktbuf;
  FAR char *fname = (FAR char *)arg[0];
  size_t fnamelen = *(FAR size_t *)arg[1];

  int32_t ret = altcom_geterrcode(out->altcom_result);

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

int32_t altcom_loglist_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmddbg_getloglistres_s *out =
    (FAR struct apicmddbg_getloglistres_s *)pktbuf;
  FAR char *list = (FAR char *)arg[0];
  size_t nlists = (size_t)arg[1];
  size_t fnamelen = (size_t)arg[2];
  int32_t ret = altcom_geterrcode(out->altcom_result);
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

int32_t altcom_logcommon_pkt_parse(FAR struct alt1250_dev_s *dev,
                                   FAR uint8_t *pktbuf, size_t pktsz,
                                   uint8_t altver, FAR void **arg,
                                   size_t arglen, FAR uint64_t *bitmap)
{
  FAR struct apicmd_logcommonres_s *out =
    (FAR struct apicmd_logcommonres_s *)pktbuf;

  return ntohl(out->altcom_result);
}

int32_t altcom_logread_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 int8_t altver, FAR void **arg,
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
