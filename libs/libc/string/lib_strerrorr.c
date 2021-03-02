/****************************************************************************
 * libs/libc/string/lib_strerrorr.c
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

#include <sys/types.h>
#include <string.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strerror_r
 *
 * Description:
 *   The strerror_r() function is similar to strerror(), but is thread safe.
 *   It returns the error string in the user-supplied buffer 'buf' of length
 *  'buflen'.
 *
 * Returned Value:
 *  strerror_r() returns 0 on success. On error, a (positive) error number is
 *  returned.
 *
 * Portability:
 *   Specified in POSIX.1-2001
 *
 ****************************************************************************/

int strerror_r(int errnum, FAR char *buf, size_t buflen)
{
  FAR const char *errstr = strerror(errnum);

  DEBUGASSERT(buf != NULL);
  strncpy(buf, errstr, buflen);
  return OK;
}
