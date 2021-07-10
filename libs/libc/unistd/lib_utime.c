/****************************************************************************
 * libs/libc/unistd/lib_utime.c
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

#include <sys/time.h>
#include <utime.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int utime(FAR const char *path, FAR const struct utimbuf *times)
{
  struct timeval timbuf[2];

  if (times == NULL)
    {
      return utimes(path, NULL);
    }

  timbuf[0].tv_sec  = times->actime;
  timbuf[0].tv_usec = 0;
  timbuf[1].tv_sec  = times->modtime;
  timbuf[1].tv_usec = 0;

  return utimes(path, timbuf);
}
