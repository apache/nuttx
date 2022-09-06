/****************************************************************************
 * drivers/ipcc/ipcc_priv.h
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
#include <nuttx/ipcc.h>
#include <nuttx/mutex.h>
#include <stdio.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Driver state structure */

struct ipcc_driver_s
{
  FAR struct pollfd        *fds[CONFIG_IPCC_NPOLLWAITERS];
  FAR struct ipcc_lower_s  *ipcc;     /* Lower half driver state */
  int                       crefs;    /* Count number of open references */
  int                       unlinked; /* 1 - driver has been unlinked */
  mutex_t                   lock;     /* Mutual exclusion for driver */
  sem_t                     rxsem;    /* Data ready to read semaphore */
  sem_t                     txsem;    /* Data ready to send semaphore */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int ipcc_open(FAR struct file *filep);
int ipcc_close(FAR struct file *filep);
int ipcc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
ssize_t ipcc_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
ssize_t ipcc_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
int ipcc_poll(FAR struct file *filep, FAR struct pollfd *fds,
                  bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
int ipcc_unlink(FAR struct inode *inode);
#endif

void ipcc_pollnotify(FAR struct ipcc_driver_s *ipcc, pollevent_t eventset);
void ipcc_cleanup(FAR struct ipcc_driver_s *priv);
