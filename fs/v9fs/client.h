/****************************************************************************
 * fs/v9fs/client.h
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

#ifndef __FS_V9FS_CLIENT_H
#define __FS_V9FS_CLIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/idr.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/uio.h>

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct v9fs_payload_s
{
  FAR struct iovec *wiov;
  FAR struct iovec *riov;
  struct list_node  node;
  size_t            wcount;
  size_t            rcount;
  sem_t             resp;
  uint16_t          tag;
  int               ret;
};

struct v9fs_transport_s
{
  FAR const struct v9fs_transport_ops_s *ops;

  /* The remainder of the structure is used by the "lower-half" driver
   * for whatever state storage that it may need.
   */
};

struct v9fs_transport_ops_s
{
  CODE int (*create)(FAR struct v9fs_transport_s **transport,
                     FAR const char *args);
  CODE int (*request)(FAR struct v9fs_transport_s *transport,
                      FAR struct v9fs_payload_s *payload);
  CODE void (*destroy)(FAR struct v9fs_transport_s *transport);
};

struct v9fs_client_s
{
  FAR struct v9fs_transport_s *transport;
  FAR struct idr_s            *fids;
  unsigned int                 msize;
  uint32_t                     root_fid;
  uint32_t                     tag_id;
  mutex_t                      lock;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int v9fs_client_statfs(FAR struct v9fs_client_s *client,
                       FAR struct statfs *buf);
int v9fs_client_stat(FAR struct v9fs_client_s *client, uint32_t fid,
                     FAR struct stat *buf);
off_t v9fs_client_getsize(FAR struct v9fs_client_s *client, uint32_t fid);
int v9fs_client_chstat(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const struct stat *buf, int flags);
ssize_t v9fs_client_read(FAR struct v9fs_client_s *client, uint32_t fid,
                         FAR void *buffer, off_t offset, size_t buflen);
ssize_t v9fs_client_convertdir(FAR const uint8_t *buffer, size_t bufsize,
                               off_t head, FAR off_t *offset,
                               FAR struct dirent *entry);
ssize_t v9fs_client_readdir(FAR struct v9fs_client_s *client, uint32_t fid,
                            FAR void *buffer, off_t offset, size_t buflen);
ssize_t v9fs_client_write(FAR struct v9fs_client_s *client, uint32_t fid,
                          FAR const void *buffer, off_t offset,
                          size_t buflen);
int v9fs_client_fsync(FAR struct v9fs_client_s *client, uint32_t fid);
int v9fs_client_rename(FAR struct v9fs_client_s *client, uint32_t fid,
                       uint32_t newfid, FAR const char *name);
int v9fs_client_remove(FAR struct v9fs_client_s *client, uint32_t fid);
int v9fs_client_unlink(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const char *name, bool isdir);
int v9fs_client_mkdir(FAR struct v9fs_client_s *client, uint32_t fid,
                      FAR const char *name, int mode);
int v9fs_client_create(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const char *name, int oflags, int mode);
int v9fs_client_open(FAR struct v9fs_client_s *client,
                      uint32_t fid, int oflags);
int v9fs_client_getname(FAR struct v9fs_client_s *client, uint32_t fid,
                        FAR char *path);
int v9fs_client_walk(FAR struct v9fs_client_s *client, FAR const char *path,
                     FAR const char **childname);
int v9fs_client_init(FAR struct v9fs_client_s *client, FAR const char *data);
int v9fs_client_uninit(FAR struct v9fs_client_s *client);
int v9fs_transport_create(FAR struct v9fs_transport_s **transport,
                          FAR const char *trans_type, FAR const char *data);
int v9fs_transport_request(FAR struct v9fs_transport_s *transport,
                           FAR struct v9fs_payload_s *payload);
void v9fs_transport_destroy(FAR struct v9fs_transport_s *transport);
void v9fs_transport_done(FAR struct v9fs_payload_s *cookie, int ret);
int v9fs_fid_put(FAR struct v9fs_client_s *client, uint32_t fid);
int v9fs_fid_get(FAR struct v9fs_client_s *client, uint32_t fid);

#endif /* __FS_V9FS_CLIENT_H */
