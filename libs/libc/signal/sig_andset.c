/****************************************************************************
 * libs/libc/signal/sig_andset.c
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

#include <signal.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigandset
 *
 * Description:
 *   This function returns the intersection of right and left in dest.
 *
 *   This is a non-standard function that may be provided by glibc if
 *   _GNU_SOURCE is defined.
 *
 * Input Parameters:
 *   dest        - Location to store the intersection
 *   left, right - The two sets to use in the intersection
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *     0 on success and -1 on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigandset(FAR sigset_t *dest, FAR sigset_t *left, FAR sigset_t *right)
{
  int ndx;

  /* Add the signal to the dest set */

  for (ndx = 0; ndx < _SIGSET_NELEM; ndx++)
    {
      dest->_elem[ndx] = left->_elem[ndx] & right->_elem[ndx];
    }

  return OK;
}
