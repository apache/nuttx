/****************************************************************************
 * fs/event/event_open.c
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

#include <sys/stat.h>
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/event.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "notify/notify.h"
#include "event/event.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_open
 *
 * Description:
 *   This function establishes a connection between named event groups and a
 *   task. the task may reference the event group associated with name using
 *   the address returned by this call. The event group may be used in a
 *   subsequent calls to nxevent_wait(), or nxevent_post(). And the event
 *   group remains usable until the event group is closed by a successful
 *   call to nxevent_close().
 *
 *   If a task makes multiple calls to event_open() with the same name, then
 *   the same event group address is returned.
 *
 * Input Parameters:
 *   event  - Location to return the event group reference.
 *   name   - event group name.
 *   oflags - event group creation options.  This may either or both of the
 *     following bit settings.
 *     oflags = 0:  Connect to the event group only if it already exists.
 *     oflags = O_CREAT:  Connect to the event group if it exists, otherwise
 *        create the event group.
 *     oflags = O_CREAT|O_EXCL:  Create a new event group unless
 *        already exists.
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *     parameters are expected:
 *     1. mode_t mode, is required but not used in the present
 *     implementation.
 *     2. unsigned events. The event group is created with an initial
 *     value of events.
 *
 * Returned Value:
 *   0 (OK), or negated errno if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxevent_open(FAR nxevent_t **event, FAR const char *name,
                 int oflags, ...)
{
  FAR struct inode *inode;
  FAR struct nevent_inode_s *nevent;
  struct inode_search_s desc;
  char fullpath[MAX_EVENTPATH];
  mode_t mode;
  nxevent_mask_t events = 0;
  int ret;

  /* Get the full path to the  */

  snprintf(fullpath, MAX_EVENTPATH,
           CONFIG_FS_NAMED_EVENTS_VFS_PATH "/%s", name);

  /* Get the inode for this event group.  This should succeed if the
   * event group has already been created.  In this case, inode_find()
   * will have incremented the reference count on the inode.
   */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at this path.  Get the search results */

      inode = desc.node;

      /* Verify that the inode is a event group */

      if (!INODE_IS_NAMEDEVENT(inode))
        {
          ret = -ENXIO;
          goto errout_with_inode;
        }

      /* It exists and is a event group.  Check if the caller wanted to
       * create a new event group with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          ret = -EEXIST;
          goto errout_with_inode;
        }

      /* Return a reference to the event group, retaining the reference
       * count on the inode.
       */

      *event = &inode->u.i_nevent->ne_event;
    }
  else
    {
      va_list ap;

      /* The event group does not exists.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The event group does not exist and O_CREAT is not set */

          ret = -ENOENT;
          goto errout_with_search;
        }

      /* Create the event group.  First we have to extract the additional
       * parameters from the variable argument list.
       * REVISIT:  Mode parameter is not currently used.
       */

      va_start(ap, oflags);
      mode  = va_arg(ap, mode_t) & ~getumask();
      events = va_arg(ap, nxevent_mask_t);
      va_end(ap);

      /* Create an inode in the pseudo-filesystem at this path.  The new
       * inode will be created with a reference count of zero.
       */

      inode_lock();
      ret = inode_reserve(fullpath, mode, &inode);
      inode_unlock();

      if (ret < 0)
        {
          goto errout_with_search;
        }

      /* Allocate the event group structure (using the appropriate allocator
       * for the group)
       */

      nevent = group_malloc(NULL, sizeof(struct nevent_inode_s));
      if (!nevent)
        {
          ret = -ENOMEM;
          goto errout_with_inode;
        }

      /* Link to the inode */

      inode->u.i_nevent = nevent;
      nevent->ne_inode  = inode;

      /* Initialize the inode */

      INODE_SET_NAMEDEVENT(inode);
      atomic_fetch_add(&inode->i_crefs, 1);

      /* Initialize the event groups */

      nxevent_init(&nevent->ne_event, events);

      /* Return a reference to the event groups */

      *event = &nevent->ne_event;
    }

  RELEASE_SEARCH(&desc);
#ifdef CONFIG_FS_NOTIFY
  notify_open(fullpath, oflags);
#endif
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}
