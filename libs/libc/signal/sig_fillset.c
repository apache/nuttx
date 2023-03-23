/****************************************************************************
 * libs/libc/signal/sig_fillset.c
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
#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigfillset
 *
 * Description:
 *   This function initializes the signal set specified by set such that all
 *   signals are included.
 *
 * Input Parameters:
 *   set - Signal set to initialize
 *
 * Returned Value:
 *   0 (OK), or -1 (ERROR) if the signal set cannot be initialized.
 *
 ****************************************************************************/

int sigfillset(FAR sigset_t *set)
{
  int ndx;

  /* Add sll signals to the set */

  for (ndx = 0; ndx < _SIGSET_NELEM; ndx++)
    {
      set->_elem[ndx] = _ALL_SIGNALS;
    }

  return OK;
}
