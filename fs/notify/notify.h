/****************************************************************************
 * fs/notify/notify.h
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

#ifndef __FS_NOTIFY_NOTIFY_H
#define __FS_NOTIFY_NOTIFY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* These are internal OS interface and are not available to applications */

void notify_open(FAR const char *path, int oflags);
void notify_close(FAR const char *path, int oflags);
void notify_close2(FAR struct inode *inode);
void notify_read(FAR struct file *filep);
void notify_write(FAR struct file *filep);
void notify_chstat(FAR struct file *filep);
void notify_unlink(FAR const char *path);
void notify_unmount(FAR const char *path);
void notify_mkdir(FAR const char *path);
void notify_create(FAR const char *path);
void notify_rename(FAR const char *oldpath, bool oldisdir,
                   FAR const char *newpath, bool newisdir);
void notify_initialize(void);

#endif
