/****************************************************************************
 * libs/libc/netdb/lib_herror.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdio.h>
#include <unistd.h>
#include <netdb.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hstrerror
 *
 * Description:
 *   Give the message for an h_errno value: HOST_NOT_FOUND, NO_DATA,
 *   NO_RECOVERY or TRY_AGAIN.  The messages are the same as in other C
 *   libraries.
 *
 * Input Parameters:
 *   errnum - The h_errno value.
 *
 * Returned Value:
 *   A pointer to a constant string.  It is never NULL.
 *
 ****************************************************************************/

FAR const char *hstrerror(int errnum)
{
  switch (errnum)
    {
      case 0:
        return "Resolver Error 0 (no error)";

      case HOST_NOT_FOUND:
        return "Unknown host";

      case NO_DATA:
        return "No address associated with name";

      case NO_RECOVERY:
        return "Unknown server error";

      case TRY_AGAIN:
        return "Host name lookup failure";

      default:
        return "Unknown resolver error";
    }
}

/****************************************************************************
 * Name: herror
 *
 * Description:
 *   Write the message for the current h_errno value to stderr.  When s is
 *   not NULL and not empty, the message starts with s and ": ".
 *
 * Input Parameters:
 *   s - The text in front of the message, or NULL.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void herror(FAR const char *s)
{
#ifdef CONFIG_FILE_STREAM
  if (s != NULL && *s != '\0')
    {
      fprintf(stderr, "%s: %s\n", s, hstrerror(h_errno));
    }
  else
    {
      fprintf(stderr, "%s\n", hstrerror(h_errno));
    }
#else
  if (s != NULL && *s != '\0')
    {
      dprintf(STDERR_FILENO, "%s: %s\n", s, hstrerror(h_errno));
    }
  else
    {
      dprintf(STDERR_FILENO, "%s\n", hstrerror(h_errno));
    }
#endif
}
