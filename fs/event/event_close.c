/****************************************************************************
 * fs/event/event_close.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/event.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "notify/notify.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxevent_close
 *
 * Description:
 *   This function is called to indicate that the calling task is finished
 *   with the specified named event group. The event_close() deallocates
 *   any system resources allocated by the system for this named event.
 *
 * Input Parameters:
 *  event - event descriptor
 *
 * Returned Value:
 *  0 (OK), or negated errno if unsuccessful.
 *
 * Assumptions:
 *   - Care must be taken to avoid risking the deletion of a event that
 *     another calling task has already locked.
 *   - event_close must not be called for an un-named event
 *
 ****************************************************************************/

int nxevent_close(FAR nxevent_t *event)
{
  FAR struct nevent_inode_s *nevent;
  struct inode *inode;

  DEBUGASSERT(event);

  /* Upcast to get back to out internal representation */

  nevent = (FAR struct nevent_inode_s *)event;
  DEBUGASSERT(nevent->ne_inode);
  inode = nevent->ne_inode;

  /* If the event group was previously unlinked and the reference count has
   * decremented to zero, then release the event group and delete the inode
   * now.
   */

  if (atomic_fetch_sub(&inode->i_crefs, 1) <= 1)
    {
      nxevent_destroy(&nevent->ne_event);
      group_free(NULL, nevent);

      /* Release and free the inode container.  If it has been properly
       * unlinked, then the peer pointer should be NULL.
       */

#ifdef CONFIG_FS_NOTIFY
      notify_close2(inode);
#endif
      DEBUGASSERT(inode->i_peer == NULL);
      inode_free(inode);
    }

  return OK;
}
