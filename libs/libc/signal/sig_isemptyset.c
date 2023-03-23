/****************************************************************************
 * libs/libc/signal/sig_isemptyset.c
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

#include <stdbool.h>
#include <signal.h>
#include <errno.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigisemptyset
 *
 * Description:
 *   This function returns 1 if set contains no signals, and 0 otherwise.
 *
 *   This is a non-standard function that may be provided by glibc if
 *   _GNU_SOURCE is defined.
 *
 * Input Parameters:
 *   set - Signal set to test
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    true - The set is empty.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigisemptyset(FAR sigset_t *set)
{
  int ndx;

  /* Add the signal to the set */

  for (ndx = 0; ndx < _SIGSET_NELEM; ndx++)
    {
      if (set->_elem[ndx] != _NO_SIGNALS)
        {
          return 0;
        }
    }

  return 1;
}
