/****************************************************************************
 * libs/libc/misc/lib_err.c
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

#include <err.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VA(call)           \
do                         \
{                          \
    va_list ap;            \
    va_start(ap, fmt);     \
    call;                  \
    va_end(ap);            \
} while(0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vwarn
 ****************************************************************************/

void vwarn(FAR const char *fmt, va_list ap)
{
  int error = errno;
  struct va_format vaf;

#ifdef va_copy
  va_list copy;

  va_copy(copy, ap);

  vaf.fmt = fmt;
  vaf.va  = &copy;
#else
  vaf.fmt = fmt;
  vaf.va  = &ap;
#endif

#ifdef CONFIG_FILE_STREAM
  fprintf(stderr, "%d: %pV: %s\n", _SCHED_GETTID(), &vaf, strerror(error));
#else
  dprintf(STDERR_FILENO, "%d: %pV: %s\n", _SCHED_GETTID(),
                                          &vaf, strerror(error));
#endif

#ifdef va_copy
  va_end(copy);
#endif
}

/****************************************************************************
 * Name: vwarnx
 ****************************************************************************/

void vwarnx(FAR const char *fmt, va_list ap)
{
  struct va_format vaf;

#ifdef va_copy
  va_list copy;

  va_copy(copy, ap);

  vaf.fmt = fmt;
  vaf.va  = &copy;
#else
  vaf.fmt = fmt;
  vaf.va  = &ap;
#endif

#ifdef CONFIG_FILE_STREAM
  fprintf(stderr, "%d: %pV\n", _SCHED_GETTID(), &vaf);
#else
  dprintf(STDERR_FILENO, "%d: %pV\n", _SCHED_GETTID(), &vaf);
#endif

#ifdef va_copy
  va_end(copy);
#endif
}

/****************************************************************************
 * Name: warn
 ****************************************************************************/

void warn(FAR const char *fmt, ...)
{
  VA(vwarn(fmt, ap));
}

/****************************************************************************
 * Name: warnx
 ****************************************************************************/

void warnx(FAR const char *fmt, ...)
{
  VA(vwarnx(fmt, ap));
}

/****************************************************************************
 * Name: verr
 ****************************************************************************/

void verr(int status, FAR const char *fmt, va_list ap)
{
  vwarn(fmt, ap);
  exit(status);
}

/****************************************************************************
 * Name: verrx
 ****************************************************************************/

void verrx(int status, FAR const char *fmt, va_list ap)
{
  vwarnx(fmt, ap);
  exit(status);
}

/****************************************************************************
 * Name: err
 ****************************************************************************/

void err(int status, FAR const char *fmt, ...)
{
  VA(verr(status, fmt, ap));
}

/****************************************************************************
 * Name: errx
 ****************************************************************************/

void errx(int status, FAR const char *fmt, ...)
{
  VA(verrx(status, fmt, ap));
}
