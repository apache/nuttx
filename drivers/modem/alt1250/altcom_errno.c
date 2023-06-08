/****************************************************************************
 * drivers/modem/alt1250/altcom_errno.c
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

#include <errno.h>
#include <arpa/inet.h>
#include "altcom_errno.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SZ
#  define ARRAY_SZ(array) (sizeof(array)/sizeof(array[0]))
#endif

#define TABLE_CONTENT(errno_name) { ALTCOM_##errno_name, errno_name }

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct converrno_s
{
  int altcom_errno;
  int nuttx_errno;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct converrno_s g_converrno_tbl[] =
{
  TABLE_CONTENT(E2BIG),
  TABLE_CONTENT(EACCES),
  TABLE_CONTENT(EADDRINUSE),
  TABLE_CONTENT(EADDRNOTAVAIL),
  TABLE_CONTENT(EADV),
  TABLE_CONTENT(EAFNOSUPPORT),
  TABLE_CONTENT(EAGAIN),
  TABLE_CONTENT(EALREADY),
  TABLE_CONTENT(EBADE),
  TABLE_CONTENT(EBADF),
  TABLE_CONTENT(EBADFD),
  TABLE_CONTENT(EBADMSG),
  TABLE_CONTENT(EBADR),
  TABLE_CONTENT(EBADRQC),
  TABLE_CONTENT(EBADSLT),
  TABLE_CONTENT(EBFONT),
  TABLE_CONTENT(EBUSY),
  TABLE_CONTENT(ECANCELED),
  TABLE_CONTENT(ECASECLASH),
  TABLE_CONTENT(ECHILD),
  TABLE_CONTENT(ECHRNG),
  TABLE_CONTENT(ECOMM),
  TABLE_CONTENT(ECONNABORTED),
  TABLE_CONTENT(ECONNREFUSED),
  TABLE_CONTENT(ECONNRESET),
  TABLE_CONTENT(EDEADLK),
  TABLE_CONTENT(EDEADLOCK),
  TABLE_CONTENT(EDESTADDRREQ),
  TABLE_CONTENT(EDOM),
  TABLE_CONTENT(EDOTDOT),
  TABLE_CONTENT(EDQUOT),
  TABLE_CONTENT(EEXIST),
  TABLE_CONTENT(EFAULT),
  TABLE_CONTENT(EFBIG),
  TABLE_CONTENT(EFTYPE),
  TABLE_CONTENT(EHOSTDOWN),
  TABLE_CONTENT(EHOSTUNREACH),
  TABLE_CONTENT(EIDRM),
  TABLE_CONTENT(EILSEQ),
  TABLE_CONTENT(EINPROGRESS),
  TABLE_CONTENT(EINTR),
  TABLE_CONTENT(EINVAL),
  TABLE_CONTENT(EIO),
  TABLE_CONTENT(EISCONN),
  TABLE_CONTENT(EISDIR),
  TABLE_CONTENT(EL2HLT),
  TABLE_CONTENT(EL2NSYNC),
  TABLE_CONTENT(EL3HLT),
  TABLE_CONTENT(EL3RST),
  TABLE_CONTENT(ELBIN),
  TABLE_CONTENT(ELIBACC),
  TABLE_CONTENT(ELIBBAD),
  TABLE_CONTENT(ELIBEXEC),
  TABLE_CONTENT(ELIBMAX),
  TABLE_CONTENT(ELIBSCN),
  TABLE_CONTENT(ELNRNG),
  TABLE_CONTENT(ELOOP),
  TABLE_CONTENT(EMFILE),
  TABLE_CONTENT(EMLINK),
  TABLE_CONTENT(EMSGSIZE),
  TABLE_CONTENT(EMULTIHOP),
  TABLE_CONTENT(ENAMETOOLONG),
  TABLE_CONTENT(ENETDOWN),
  TABLE_CONTENT(ENETRESET),
  TABLE_CONTENT(ENETUNREACH),
  TABLE_CONTENT(ENFILE),
  TABLE_CONTENT(ENMFILE),
  TABLE_CONTENT(ENOANO),
  TABLE_CONTENT(ENOBUFS),
  TABLE_CONTENT(ENOCSI),
  TABLE_CONTENT(ENODATA),
  TABLE_CONTENT(ENODEV),
  TABLE_CONTENT(ENOENT),
  TABLE_CONTENT(ENOEXEC),
  TABLE_CONTENT(ENOLCK),
  TABLE_CONTENT(ENOLINK),
  TABLE_CONTENT(ENOMEDIUM),
  TABLE_CONTENT(ENOMEM),
  TABLE_CONTENT(ENOMSG),
  TABLE_CONTENT(ENONET),
  TABLE_CONTENT(ENOPKG),
  TABLE_CONTENT(ENOPROTOOPT),
  TABLE_CONTENT(ENOSHARE),
  TABLE_CONTENT(ENOSPC),
  TABLE_CONTENT(ENOSR),
  TABLE_CONTENT(ENOSTR),
  TABLE_CONTENT(ENOSYS),
  TABLE_CONTENT(ENOTBLK),
  TABLE_CONTENT(ENOTCONN),
  TABLE_CONTENT(ENOTDIR),
  TABLE_CONTENT(ENOTEMPTY),
  TABLE_CONTENT(ENOTRECOVERABLE),
  TABLE_CONTENT(ENOTSOCK),
  TABLE_CONTENT(ENOTSUP),
  TABLE_CONTENT(ENOTTY),
  TABLE_CONTENT(ENOTUNIQ),
  TABLE_CONTENT(ENXIO),
  TABLE_CONTENT(EOPNOTSUPP),
  TABLE_CONTENT(EOVERFLOW),
  TABLE_CONTENT(EOWNERDEAD),
  TABLE_CONTENT(EPERM),
  TABLE_CONTENT(EPFNOSUPPORT),
  TABLE_CONTENT(EPIPE),
  TABLE_CONTENT(EPROCLIM),
  TABLE_CONTENT(EPROTO),
  TABLE_CONTENT(EPROTONOSUPPORT),
  TABLE_CONTENT(EPROTOTYPE),
  TABLE_CONTENT(ERANGE),
  TABLE_CONTENT(EREMCHG),
  TABLE_CONTENT(EREMOTE),
  TABLE_CONTENT(EROFS),
  TABLE_CONTENT(ESHUTDOWN),
  TABLE_CONTENT(ESOCKTNOSUPPORT),
  TABLE_CONTENT(ESPIPE),
  TABLE_CONTENT(ESRCH),
  TABLE_CONTENT(ESRMNT),
  TABLE_CONTENT(ESTALE),
  TABLE_CONTENT(ESTRPIPE),
  TABLE_CONTENT(ETIME),
  TABLE_CONTENT(ETIMEDOUT),
  TABLE_CONTENT(ETOOMANYREFS),
  TABLE_CONTENT(ETXTBSY),
  TABLE_CONTENT(EUNATCH),
  TABLE_CONTENT(EUSERS),
  TABLE_CONTENT(EXDEV),
  TABLE_CONTENT(EXFULL)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int altcom_errno2nuttx(int altcom_errno)
{
  int i;

  for (i = 0; i < ARRAY_SZ(g_converrno_tbl); i++)
    {
      if (g_converrno_tbl[i].altcom_errno == altcom_errno)
        {
          return g_converrno_tbl[i].nuttx_errno;
        }
    }

  return altcom_errno;
}

int altcom_geterrcode(int32_t err_code)
{
  int err;

  err = ntohl(err_code);
  if (err < 0)
    {
      err = altcom_errno2nuttx(-err);
      err = -err;
    }

  return err;
}

