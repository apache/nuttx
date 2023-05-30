/****************************************************************************
 * libs/libc/signal/sig_xorset.c
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

#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_xorset
 *
 * Description:
 *   This function returns the xor of right and left in dest.
 *
 * Input Parameters:
 *   dest        - Location to store the union
 *   left, right - The two sets to use in the union
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *     0 on successor or -1 on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_xorset(FAR sigset_t *dest, FAR const sigset_t *left,
                 FAR const sigset_t *right)
{
  int ndx;

  /* Add the signal sets */

  for (ndx = 0; ndx < _SIGSET_NELEM; ndx++)
    {
      dest->_elem[ndx] = left->_elem[ndx] ^ right->_elem[ndx];
    }

  return OK;
}
