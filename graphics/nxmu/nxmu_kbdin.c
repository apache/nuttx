/****************************************************************************
 * graphics/nxmu/nxmu_kbdin.c
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
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nx.h>

#include "nxmu.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_kbdin
 *
 * Description:
 *   New keyboard data has been received from the thread or interrupt
 *   handler that manages some kind of keyboard/keypad hardware.  Route that
 *   positional data to the appropriate window client.
 *
 ****************************************************************************/

void nxmu_kbdin(FAR struct nxmu_state_s *nxmu, uint8_t nch, FAR uint8_t *ch)
{
  FAR struct nxclimsg_kbdin_s *outmsg;
  int size;
  int i;

  /* Allocate a bigger message to account for the variable amount of
   * character data.
   */

  size   = sizeof(struct nxclimsg_kbdin_s) + nch - 1;
  outmsg = (FAR struct nxclimsg_kbdin_s *)kmm_malloc(size);
  if (outmsg)
    {
      /* Give the keypad input only to the top child */

      outmsg->msgid = NX_CLIMSG_KBDIN;
      outmsg->wnd   = nxmu->be.topwnd;
      outmsg->nch   = nch;

      for (i = 0; i < nch; i++)
        {
          outmsg->ch[i] = ch[i];
        }

      nxmu_sendclientwindow(nxmu->be.topwnd, outmsg, size);
      kmm_free(outmsg);
    }
}

#endif /* CONFIG_NX_KBD */
