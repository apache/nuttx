/****************************************************************************
 * fs/v9fs/client.c
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

#include <sys/param.h>
#include <fcntl.h>

#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lib/lib.h>

#include "client.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 9P Magic Numbers */

#define V9FS_NOTAG             ((uint16_t)(~0))
#define V9FS_NOFID             ((uint32_t)(~0))
#define V9FS_HDRSZ             7          /* size[4] type[1] tag[2] */
#define V9FS_IOHDRSZ           24         /* Reserve enough space for
                                           * TWRITE/TREAD
                                           */
#define V9FS_READDIRHDRSZ      24         /* Reserve enough space for
                                           * READDIR
                                           */
#define V9FS_BIT8SZ            1          /* uint8  -> V9FS_BIT8SZ  */
#define V9FS_BIT16SZ           2          /* uint16 -> V9FS_BIT16SZ */
#define V9FS_BIT32SZ           4          /* uint32 -> V9FS_BIT32SZ */
#define V9FS_BIT64SZ           8          /* uint64 -> V9FS_BIT64SZ */
#define V9FS_DEFAULT_VERSION   "9P2000.L" /* Current implementations are
                                           * based on the "9P2000L"
                                           * protocol.
                                           */

/* 9P setattr valid bits */

#define V9FS_SETATTR_MODE      (1 << 0)
#define V9FS_SETATTR_UID       (1 << 1)
#define V9FS_SETATTR_GID       (1 << 2)
#define V9FS_SETATTR_SIZE      (1 << 3)
#define V9FS_SETATTR_ATIME     (1 << 4)
#define V9FS_SETATTR_MTIME     (1 << 5)
#define V9FS_SETATTR_CTIME     (1 << 6)
#define V9FS_SETATTR_ATIMESET  (1 << 7)
#define V9FS_SETATTR_MTIMESET  (1 << 8)

/* 9P2000.L open flags */

#define V9FS_RDONLY            00000000
#define V9FS_WRONLY            00000001
#define V9FS_RDWR              00000002
#define V9FS_CREATE            00000100
#define V9FS_EXCL              00000200
#define V9FS_NOCTTY            00000400
#define V9FS_TRUNC             00001000
#define V9FS_APPEND            00002000
#define V9FS_NONBLOCK          00004000
#define V9FS_DSYNC             00010000
#define V9FS_FASYNC            00020000
#define V9FS_DIRECT            00040000
#define V9FS_DIRECTORY         00200000
#define V9FS_NOFOLLOW          00400000
#define V9FS_NOATIME           01000000
#define V9FS_CLOEXEC           02000000
#define V9FS_SYNC              04000000

/* 9P2000.L stat flags */

#define V9FS_STATS_ALL         0x00003fff

/* 9P2000.L unlink flags */

#define V9FS_REMOVEDIR         0x200

/* QID = uint8_t (type) + uint32_t (version) + uint64_t (path) */

#define V9FS_QIDSZ             (V9FS_BIT8SZ + V9FS_BIT32SZ + V9FS_BIT64SZ)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum v9fs_flags_e
{
  V9FS_TLERROR = 6,
  V9FS_RLERROR,
  V9FS_TSTATFS = 8,
  V9FS_RSTATFS,
  V9FS_TLOPEN = 12,
  V9FS_RLOPEN,
  V9FS_TLCREATE = 14,
  V9FS_RLCREATE,
  V9FS_TRENAME = 20,
  V9FS_RRENAME,
  V9FS_TGETATTR = 24,
  V9FS_RGETATTR,
  V9FS_TSETATTR = 26,
  V9FS_RSETATTR,
  V9FS_TREADDIR = 40,
  V9FS_RREADDIR,
  V9FS_TFSYNC = 50,
  V9FS_RFSYNC,
  V9FS_TMKDIR = 72,
  V9FS_RMKDIR,
  V9FS_TRENAMEAT = 74,
  V9FS_RRENAMEAT,
  V9FS_TUNLINKAT = 76,
  V9FS_RUNLINKAT,
  V9FS_TVERSION = 100,
  V9FS_RVERSION,
  V9FS_TATTACH = 104,
  V9FS_RATTACH,
  V9FS_TERROR = 106,
  V9FS_RERROR,
  V9FS_TWALK = 110,
  V9FS_RWALK,
  V9FS_TREAD = 116,
  V9FS_RREAD,
  V9FS_TWRITE = 118,
  V9FS_RWRITE,
  V9FS_TCLUNK = 120,
  V9FS_RCLUNK,
  V9FS_TREMOVE = 122,
  V9FS_RREMOVE,
  V9FS_TSTAT = 124,
  V9FS_RSTAT,
  V9FS_TWSTAT = 126,
  V9FS_RWSTAT,
};

begin_packed_struct struct v9fs_qid_s
{
  uint8_t  type;
  uint32_t version;
  uint64_t path;
} end_packed_struct;

begin_packed_struct struct v9fs_header_s
{
  uint32_t size;
  uint8_t type;
  uint16_t tag;
} end_packed_struct;

begin_packed_struct struct v9fs_attach_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t afid;
  uint8_t buffer[2 * (V9FS_BIT16SZ + NAME_MAX) + V9FS_BIT32SZ];
} end_packed_struct;

begin_packed_struct struct v9fs_rattach_s
{
  struct v9fs_header_s header;
  struct v9fs_qid_s qid;
} end_packed_struct;

begin_packed_struct struct v9fs_clunk_s
{
  struct v9fs_header_s header;
  uint32_t fid;
} end_packed_struct;

#define v9fs_remove_s v9fs_clunk_s

begin_packed_struct struct v9fs_version_s
{
  struct v9fs_header_s header;
  uint32_t msize;
  uint16_t version_len;
  char version[NAME_MAX];
} end_packed_struct;

begin_packed_struct struct v9fs_setattr_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t valid;
  uint32_t mode;
  uint32_t uid;
  uint32_t gid;
  uint64_t size;
  uint64_t atime_sec;
  uint64_t atime_nsec;
  uint64_t mtime_sec;
  uint64_t mtime_nsec;
} end_packed_struct;

begin_packed_struct struct v9fs_mkdir_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint16_t name_len;
  uint8_t buffer[NAME_MAX + V9FS_BIT32SZ * 2];
} end_packed_struct;

#define v9fs_rmkdir_s v9fs_rattach_s

begin_packed_struct struct v9fs_unlink_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint16_t name_len;
  uint8_t buffer[NAME_MAX + V9FS_BIT32SZ];
} end_packed_struct;

begin_packed_struct struct v9fs_lerror_s
{
  struct v9fs_header_s header;
  uint32_t ecode;
} end_packed_struct;

begin_packed_struct struct v9fs_open_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t flags;
} end_packed_struct;

begin_packed_struct struct v9fs_ropen_s
{
  struct v9fs_header_s header;
  struct v9fs_qid_s qid;
  uint32_t iounit;
} end_packed_struct;

begin_packed_struct struct v9fs_create_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint16_t name_len;
  uint8_t buffer[NAME_MAX + V9FS_BIT32SZ * 3];
} end_packed_struct;

#define v9fs_rcreate_s v9fs_ropen_s

begin_packed_struct struct v9fs_walk_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t newfid;
  uint16_t nwname;
} end_packed_struct;

begin_packed_struct struct v9fs_rwalk_s
{
  struct v9fs_header_s header;
  uint16_t nwqid;
  struct v9fs_qid_s wqid[1];
} end_packed_struct;

begin_packed_struct struct v9fs_write_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint64_t offset;
  uint32_t count;
} end_packed_struct;

begin_packed_struct struct v9fs_rwrite_s
{
  struct v9fs_header_s header;
  uint32_t count;
} end_packed_struct;

#define v9fs_read_s v9fs_write_s
#define v9fs_rread_s v9fs_rwrite_s

#define v9fs_readdir_s v9fs_write_s
#define v9fs_rreaddir_s v9fs_rwrite_s

begin_packed_struct struct v9fs_fsync_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t datasync;
} end_packed_struct;

begin_packed_struct struct v9fs_rename_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint32_t newfid;
  uint16_t name_len;
  uint8_t buffer[NAME_MAX];
} end_packed_struct;

begin_packed_struct struct v9fs_stat_s
{
  struct v9fs_header_s header;
  uint32_t fid;
  uint64_t mask;
} end_packed_struct;

begin_packed_struct struct v9fs_rstat_s
{
  struct v9fs_header_s header;
  uint64_t vaild;
  struct v9fs_qid_s qid;
  uint32_t mode;
  uint32_t uid;
  uint32_t gid;
  uint64_t nlink;
  uint64_t rdev;
  uint64_t size;
  uint64_t blksize;
  uint64_t blocks;
  uint64_t atime_sec;
  uint64_t atime_nsec;
  uint64_t mtime_sec;
  uint64_t mtime_nsec;
  uint64_t ctime_sec;
  uint64_t ctime_nsec;
  uint64_t btime_sec;
  uint64_t btime_nsec;
  uint64_t gen;
  uint64_t data_version;
} end_packed_struct;

#define v9fs_statfs_s v9fs_clunk_s

begin_packed_struct struct v9fs_rstatfs_s
{
  struct v9fs_header_s header;
  uint32_t type;    /* Type of file system (see below) */
  uint32_t bsize;   /* Optimal transfer block size */
  uint64_t blocks;  /* Total data blocks in file system */
  uint64_t bfree;   /* Free blocks in fs */
  uint64_t bavail;  /* Free blocks avail to non-superuser */
  uint64_t files;   /* Total file nodes in file system */
  uint64_t ffree;   /* Free file nodes in fs */
  uint64_t fsid;    /* File system id */
  uint32_t namelen; /* Maximum length of filenames */
} end_packed_struct;

struct v9fs_map_s
{
  int original;
  int conversion;
};

struct v9fs_fid_s
{
  uint32_t iounit;
  uint32_t refcount;
  char relpath[1];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * v9fs_get_tagid
 ****************************************************************************/

static inline uint16_t v9fs_get_tagid(FAR struct v9fs_client_s *client)
{
  uint16_t tag;

  nxmutex_lock(&client->lock);
  tag = client->tag_id;
  client->tag_id++;
  if (client->tag_id == V9FS_NOTAG)
    {
      /* Bypassing V9FS_NOTAG */

      client->tag_id = 1;
    }

  nxmutex_unlock(&client->lock);
  return tag;
}

/****************************************************************************
 * v9fs_map_setattr_flags
 ****************************************************************************/

static uint32_t v9fs_map_setattr_flags(int flags)
{
  uint32_t rflags = 0;
  size_t i;
  static const struct v9fs_map_s flags_map[] =
    {
      { CH_STAT_MODE,  V9FS_SETATTR_MODE },
      { CH_STAT_UID,   V9FS_SETATTR_UID },
      { CH_STAT_GID,   V9FS_SETATTR_GID },
      { CH_STAT_SIZE,  V9FS_SETATTR_SIZE },
      { CH_STAT_ATIME, V9FS_SETATTR_ATIMESET },
      { CH_STAT_MTIME, V9FS_SETATTR_MTIMESET },
    };

  for (i = 0; i < nitems(flags_map); i++)
    {
      if (flags & flags_map[i].original)
        {
          rflags |= flags_map[i].conversion;
        }
    }

  return rflags;
}

/****************************************************************************
 * v9fs_map_open_flags
 ****************************************************************************/

static uint32_t v9fs_map_open_flags(int flags)
{
  uint32_t rflags = 0;
  size_t i;
  static const struct v9fs_map_s flags_map[] =
    {
      { O_CREAT,     V9FS_CREATE },
      { O_NOCTTY,    V9FS_NOCTTY },
      { O_TRUNC,     V9FS_TRUNC},
      { O_APPEND,    V9FS_APPEND },
      { O_NONBLOCK,  V9FS_NONBLOCK },
      { O_DSYNC,     V9FS_DSYNC },
      { FASYNC,      V9FS_FASYNC },
      { O_DIRECT,    V9FS_DIRECT },
      { O_DIRECTORY, V9FS_DIRECTORY },
      { O_NOFOLLOW,  V9FS_NOFOLLOW },
      { O_NOATIME,   V9FS_NOATIME },
      { O_CLOEXEC,   V9FS_CLOEXEC },
      { O_EXCL,      V9FS_EXCL },
      { O_SYNC,      V9FS_SYNC},
    };

  switch (flags & 3)
    {
      case O_RDONLY:
        rflags |= V9FS_RDONLY;
        break;

      case O_WRONLY:
        rflags |= V9FS_WRONLY;
        break;

      case O_RDWR:
        rflags |= V9FS_RDWR;
        break;
    }

  for (i = 0; i < nitems(flags_map); i++)
    {
      if (flags & flags_map[i].original)
        {
          rflags |= flags_map[i].conversion;
        }
    }

  return rflags;
}

/****************************************************************************
 * v9fs_fid_create
 ****************************************************************************/

static int v9fs_fid_create(FAR struct v9fs_client_s *client,
                           FAR const char *relpath)
{
  FAR struct v9fs_fid_s *fid;
  size_t len;
  int ret;

  len = strlen(relpath);
  fid = fs_heap_zalloc(sizeof(struct v9fs_fid_s) + len);
  if (fid == NULL)
    {
      return -ENOMEM;
    }

  fid->refcount = 1;
  nxmutex_lock(&client->lock);
  ret = idr_alloc(client->fids, fid, 0, 0);
  nxmutex_unlock(&client->lock);
  if (ret >= 0)
    {
      memcpy(fid->relpath, relpath, len + 1);
      return ret;
    }

  /* Failed to initialize fid */

  fs_heap_free(fid);
  return ret;
}

/****************************************************************************
 * v9fs_fid_destroy
 ****************************************************************************/

static void v9fs_fid_destroy(FAR struct v9fs_client_s *client,
                             uint32_t fid)
{
  FAR struct v9fs_fid_s *fidp;

  nxmutex_lock(&client->lock);
  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      nxmutex_unlock(&client->lock);
      return;
    }

  idr_remove(client->fids, fid);
  nxmutex_unlock(&client->lock);
  fs_heap_free(fidp);
}

/****************************************************************************
 * v9fs_client_rpc
 ****************************************************************************/

static int v9fs_client_rpc(FAR struct v9fs_transport_s *transport,
                           FAR struct iovec *wiov, size_t wcount,
                           FAR struct iovec *riov, size_t rcount,
                           uint16_t tag)
{
  struct v9fs_payload_s payload;
  int ret;

  nxsem_init(&payload.resp, 0, 0);
  payload.wiov = wiov;
  payload.riov = riov;
  payload.wcount = wcount;
  payload.rcount = rcount;
  payload.tag = tag;
  payload.ret = -EIO;

  ret = v9fs_transport_request(transport, &payload);
  if (ret < 0)
    {
      nxsem_destroy(&payload.resp);
      return ret;
    }

  nxsem_wait(&payload.resp);
  nxsem_destroy(&payload.resp);

  return payload.ret;
}

/****************************************************************************
 * v9fs_client_clunk
 ****************************************************************************/

static int v9fs_client_clunk(FAR struct v9fs_client_s *client,
                             uint32_t fid)
{
  struct v9fs_clunk_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int ret;

  /* size[4] Tclunk tag[2] fid[4]
   * size[4] Rclunk tag[2]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ;
  request.header.type = V9FS_TCLUNK;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;

  /* A free buffer is reserved at the time of receipt to handle whether
   * there is an error number message
   */

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);

  /* Even if the clunk returns an error, the fid is no longer valid for
   * the server, so we need to remove the fid on the client side.
   */

  v9fs_fid_destroy(client, fid);
  return ret;
}

/****************************************************************************
 * v9fs_client_attach
 ****************************************************************************/

static int v9fs_client_attach(FAR struct v9fs_client_s *client, uint32_t fid,
                              FAR const char *uname, FAR const char *aname)
{
  struct v9fs_attach_s request;
  struct v9fs_rattach_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  uint16_t uname_len = strlen(uname);
  uint16_t aname_len = strlen(aname);
  uint32_t uid = getuid();
  off_t offset = 0;
  uint32_t newfid;
  int ret;

  /* size[4] Tattach tag[2] fid[4] afid[4] uname[s] aname[s] n_uname[4]
   * size[4] Rattach tag[2] qid[13]
   */

  ret = v9fs_fid_create(client, "");
  if (ret < 0)
    {
      return ret;
    }

  newfid = ret;
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 3 + V9FS_BIT16SZ * 2 +
                        uname_len + aname_len;
  request.header.type = V9FS_TATTACH;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = newfid;
  request.afid = fid;

  memcpy(&request.buffer[offset], &uname_len, V9FS_BIT16SZ);
  offset += V9FS_BIT16SZ;
  memcpy(&request.buffer[offset], uname, uname_len);
  offset += uname_len;
  memcpy(&request.buffer[offset], &aname_len, V9FS_BIT16SZ);
  offset += V9FS_BIT16SZ;
  memcpy(&request.buffer[offset], aname, aname_len);
  offset += aname_len;
  memcpy(&request.buffer[offset], &uid, V9FS_BIT32SZ);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 3 + V9FS_BIT16SZ * 2 +
                    uname_len + aname_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_QIDSZ;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      v9fs_fid_destroy(client, newfid);
      return ret;
    }

  return newfid;
}

/****************************************************************************
 * v9fs_client_version
 ****************************************************************************/

static int v9fs_client_version(FAR struct v9fs_client_s *client)
{
  struct v9fs_version_s request;
  struct v9fs_version_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int len = strlen(V9FS_DEFAULT_VERSION);
  int ret;

  /* size[4] Tversion tag[2] msize[4] version[s]
   * size[4] Rversion tag[2] msize[4] version[s]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT16SZ + len;
  request.header.type = V9FS_TVERSION;
  request.header.tag = V9FS_NOTAG;
  request.msize = client->msize;
  request.version_len = len;
  memcpy(request.version, V9FS_DEFAULT_VERSION, len);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT16SZ + len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT16SZ + len;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  if (memcmp(response.version, V9FS_DEFAULT_VERSION, len))
    {
      return -EREMOTEIO;
    }

  if (response.msize < client->msize)
    {
      client->msize = response.msize;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * v9fs_client_statfs
 ****************************************************************************/

int v9fs_client_statfs(FAR struct v9fs_client_s *client,
                       FAR struct statfs *buf)
{
  struct v9fs_statfs_s request;
  struct v9fs_rstatfs_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int ret;

  /* size[4] Tstatfs tag[2] fid[4]
   * size[4] Rstatfs tag[2] type[4] bsize[4] blocks[8] bfree[8] bavail[8]
   *                        files[8] ffree[8] fsid[8] namelen[4]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ;
  request.header.type = V9FS_TSTATFS;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = client->root_fid;

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;
  riov[0].iov_base = &response;
  riov[0].iov_len = sizeof(response);

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  buf->f_bsize   = response.bsize;
  buf->f_blocks  = response.blocks;
  buf->f_bfree   = response.bfree;
  buf->f_bavail  = response.bavail;
  buf->f_files   = response.files;
  buf->f_ffree   = response.ffree;
  buf->f_namelen = response.namelen;
  buf->f_type    = V9FS_MAGIC;

  return 0;
}

/****************************************************************************
 * v9fs_client_stat
 ****************************************************************************/

int v9fs_client_stat(FAR struct v9fs_client_s *client, uint32_t fid,
                     FAR struct stat *buf)
{
  struct v9fs_stat_s request;
  struct v9fs_rstat_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int ret;

  /* size[4] Tgetattr tag[2] fid[4] request_mask[8]
   * size[4] Rgetattr tag[2] valid[8] qid[13] mode[4] uid[4] gid[4] nlink[8]
   *         rdev[8] size[8] blksize[8] blocks[8]
   *         atime_sec[8] atime_nsec[8] mtime_sec[8] mtime_nsec[8]
   *         ctime_sec[8] ctime_nsec[8] btime_sec[8] btime_nsec[8]
   *         gen[8] data_version[8]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ;
  request.header.type = V9FS_TGETATTR;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;
  request.mask = V9FS_STATS_ALL;

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ;
  riov[0].iov_base = &response;
  riov[0].iov_len = sizeof(response);

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  buf->st_mode  = response.mode;
  buf->st_uid   = response.uid;
  buf->st_gid   = response.gid;
  buf->st_nlink = response.nlink;
  buf->st_rdev  = response.rdev;
  buf->st_size  = response.size;
  buf->st_blksize = response.blksize;
  buf->st_blocks  = response.blocks;
  buf->st_atim.tv_sec  = response.atime_sec;
  buf->st_atim.tv_nsec = response.atime_nsec;
  buf->st_mtim.tv_sec  = response.mtime_sec;
  buf->st_mtim.tv_nsec = response.mtime_nsec;
  buf->st_ctim.tv_sec = response.ctime_sec;
  buf->st_ctim.tv_nsec = response.ctime_nsec;

  return 0;
}

/****************************************************************************
 * v9fs_client_getsize
 ****************************************************************************/

off_t v9fs_client_getsize(FAR struct v9fs_client_s *client, uint32_t fid)
{
  struct stat buf;
  int ret;

  ret = v9fs_client_stat(client, fid, &buf);
  if (ret < 0)
    {
      return ret;
    }

  return buf.st_size;
}

/****************************************************************************
 * v9fs_client_chstat
 ****************************************************************************/

int v9fs_client_chstat(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const struct stat *buf, int flags)
{
  struct v9fs_setattr_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];

  /* size[4] Tsetattr tag[2] fid[4] valid[4] mode[4] uid[4] gid[4] size[8]
   *                  atime_sec[8] atime_nsec[8] mtime_sec[8] mtime_nsec[8]
   * size[4] Rsetattr tag[2]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 5 + V9FS_BIT64SZ * 5;
  request.header.type = V9FS_TSETATTR;
  request.header.tag = v9fs_get_tagid(client);

  request.fid = fid;
  request.valid = v9fs_map_setattr_flags(flags);
  request.mode = v9fs_map_open_flags(buf->st_mode);
  request.uid = buf->st_uid;
  request.gid = buf->st_gid;
  request.size = buf->st_size;
  request.atime_sec = buf->st_atim.tv_sec;
  request.atime_nsec = buf->st_atim.tv_nsec;
  request.mtime_sec = buf->st_mtim.tv_sec;
  request.mtime_nsec = buf->st_mtim.tv_nsec;

  /* A free buffer is reserved at the time of receipt to handle whether
   * there is an error number message
   */

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 5 + V9FS_BIT64SZ * 5;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  return v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                         request.header.tag);
}

/****************************************************************************
 * v9fs_client_read
 ****************************************************************************/

ssize_t v9fs_client_read(FAR struct v9fs_client_s *client, uint32_t fid,
                         FAR void *buffer, off_t offset, size_t buflen)
{
  FAR struct v9fs_fid_s *fidp;
  struct v9fs_read_s request;
  struct v9fs_rread_s response;
  struct iovec wiov[1];
  struct iovec riov[2];
  size_t nread = 0;
  int ret = 0;

  /* size[4] Tread tag[2] fid[4] offset[8] count[4]
   * size[4] Rread tag[2] count[4] data[count]
   */

  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      return -ENOENT;
    }

  while (buflen > 0)
    {
      request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ +
                            V9FS_BIT32SZ;
      request.header.type = V9FS_TREAD;
      request.header.tag = v9fs_get_tagid(client);
      request.fid = fid;
      request.offset = offset;
      request.count = buflen > fidp->iounit ? fidp->iounit : buflen;

      wiov[0].iov_base = &request;
      wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ +
                        V9FS_BIT32SZ;
      riov[0].iov_base = &response;
      riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;
      riov[1].iov_base = (FAR void *)buffer;
      riov[1].iov_len = request.count;

      ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 2,
                            request.header.tag);
      if (ret < 0)
        {
          break;
        }

      if (response.count == 0)
        {
          break;
        }

      nread += response.count;
      offset += response.count;
      buffer += response.count;
      buflen -= response.count;
    }

  return nread ? nread : ret;
}

/****************************************************************************
 * v9fs_client_readdir
 ****************************************************************************/

ssize_t v9fs_client_readdir(FAR struct v9fs_client_s *client, uint32_t fid,
                            FAR void *buffer, off_t offset, size_t buflen)
{
  FAR struct v9fs_fid_s *fidp;
  struct v9fs_readdir_s request;
  struct v9fs_rreaddir_s response;
  struct iovec wiov[1];
  struct iovec riov[2];
  int ret;

  /* size[4] Treaddir tag[2] fid[4] offset[8] count[4]
   * size[4] Rreaddir tag[2] count[4] data[count]
   */

  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      return -ENOENT;
    }

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ +
                        V9FS_BIT32SZ;
  request.header.type = V9FS_TREADDIR;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;
  request.offset = offset;
  request.count = buflen > fidp->iounit ? fidp->iounit : buflen;

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ + V9FS_BIT32SZ;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;
  riov[1].iov_base = buffer;
  riov[1].iov_len = request.count;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 2,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  return response.count;
}

/****************************************************************************
 * v9fs_client_convertdir
 ****************************************************************************/

ssize_t v9fs_client_convertdir(FAR const uint8_t *buffer, size_t bufsize,
                               off_t head, FAR off_t *offset,
                               FAR struct dirent *entry)
{
  uint64_t off;
  uint16_t name_len;
  off_t next = head;

  /* Ensure there is enough data for the fixed parts
   * (qid, offset, type, name_len)
   */

  if (bufsize < V9FS_QIDSZ + V9FS_BIT64SZ + V9FS_BIT8SZ + V9FS_BIT16SZ)
    {
      return -EIO;
    }

  /* Skip the qid part (13 Bytes) */

  next += V9FS_QIDSZ;

  /* Read the offset (8 bytes) */

  memcpy(&off, buffer + next, sizeof(uint64_t));
  next += sizeof(uint64_t);

  /* Read the type (1 byte) */

  memcpy(&entry->d_type, buffer + next, sizeof(uint8_t));
  next += sizeof(uint8_t);

  /* Read the name_len (2 bytes) */

  memcpy(&name_len, buffer + next, sizeof(uint16_t));
  next += sizeof(uint16_t);

  /* Ensure there is enough data for the name */

  if (bufsize - next < name_len)
    {
      return -EIO;
    }

  /* Copy the file name to entry->d_name and null-terminate it */

  if (name_len > NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  memcpy(entry->d_name, buffer + next, name_len);
  entry->d_name[name_len] = '\0';
  next += name_len;
  *offset = off;

  return next - head;
}

/****************************************************************************
 * v9fs_client_write
 ****************************************************************************/

ssize_t v9fs_client_write(FAR struct v9fs_client_s *client, uint32_t fid,
                          FAR const void *buffer, off_t offset,
                          size_t buflen)
{
  FAR struct v9fs_fid_s *fidp;
  struct v9fs_write_s request;
  struct v9fs_rwrite_s response;
  struct iovec wiov[2];
  struct iovec riov[1];
  size_t nwrite = 0;
  int ret = 0;

  /* size[4] Twrite tag[2] fid[4] offset[8] count[4] data[count]
   * size[4] Rwrite tag[2] count[4]
   */

  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      return -ENOENT;
    }

  while (buflen > 0)
    {
      request.count = buflen > fidp->iounit ? fidp->iounit : buflen;
      request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ +
                            V9FS_BIT32SZ + request.count;
      request.header.type = V9FS_TWRITE;
      request.header.tag = v9fs_get_tagid(client);
      request.fid = fid;
      request.offset = offset;

      wiov[0].iov_base = &request;
      wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ + V9FS_BIT64SZ +
                        V9FS_BIT32SZ;
      wiov[1].iov_base = (FAR void *)buffer;
      wiov[1].iov_len = request.count;
      riov[0].iov_base = &response;
      riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

      ret = v9fs_client_rpc(client->transport, wiov, 2, riov, 1,
                            request.header.tag);
      if (ret < 0)
        {
          break;
        }

      nwrite += response.count;
      offset += response.count;
      buffer += response.count;
      buflen -= response.count;
    }

  return nwrite ? nwrite : ret;
}

/****************************************************************************
 * v9fs_client_fsync
 ****************************************************************************/

int v9fs_client_fsync(FAR struct v9fs_client_s *client, uint32_t fid)
{
  struct v9fs_fsync_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];

  /* size[4] Tfsync tag[2] fid[4] datasync[4]
   * size[4] Rfsync tag[2]
   * 0x01 - Sync data only
   * 0x00 - Complete update data (including timestamps and permissions)
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 2;
  request.header.type = V9FS_TFSYNC;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;
  request.datasync = 0;

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 2;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  return v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                         request.header.tag);
}

/****************************************************************************
 * v9fs_client_rename
 ****************************************************************************/

int v9fs_client_rename(FAR struct v9fs_client_s *client, uint32_t fid,
                       uint32_t newfid, FAR const char *name)
{
  struct v9fs_rename_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];

  /* size[4] Trename tag[2] fid[4] dfid[4] name[s]
   * size[4] Rrename tag[2]
   */

  request.name_len = strlen(name);
  request.fid = fid;
  request.newfid = newfid;
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ +
                         request.name_len;
  request.header.type = V9FS_TRENAME;
  request.header.tag = v9fs_get_tagid(client);
  memcpy(&request.buffer, name, request.name_len);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ +
                    request.name_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  return v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                         request.header.tag);
}

/****************************************************************************
 * v9fs_client_remove
 ****************************************************************************/

int v9fs_client_remove(FAR struct v9fs_client_s *client, uint32_t fid)
{
  struct v9fs_remove_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];

  /* size[4] Tremove tag[2] fid[4]
   * size[4] Rremove tag[2]
   */

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ;
  request.header.type = V9FS_TREMOVE;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;

  /* A free buffer is reserved at the time of receipt to handle whether
   * there is an error number message
   */

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  return v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                         request.header.tag);
}

/****************************************************************************
 * v9fs_client_unlink
 ****************************************************************************/

int v9fs_client_unlink(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const char *name, bool isdir)
{
  struct v9fs_unlink_s request;
  struct v9fs_lerror_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int flags = isdir ? V9FS_REMOVEDIR : 0;
  int ret;

  /* size[4] Tunlinkat tag[2] dirfd[4] name[s] flags[4]
   * size[4] Runlinkat tag[2]
   */

  request.name_len = strlen(name);
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ +
                        request.name_len;
  request.header.type = V9FS_TUNLINKAT;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = fid;
  memcpy(&request.buffer, name, request.name_len);
  memcpy(&request.buffer[request.name_len], &flags, V9FS_BIT32SZ);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ +
                    request.name_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  return v9fs_client_clunk(client, fid);
}

/****************************************************************************
 * v9fs_client_mkdir
 ****************************************************************************/

int v9fs_client_mkdir(FAR struct v9fs_client_s *client, uint32_t fid,
                      FAR const char *name, int mode)
{
  struct v9fs_mkdir_s request;
  struct v9fs_rmkdir_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  uint32_t gid = getgid();
  off_t offset = 0;

  /* size[4] Tmkdir tag[2] dfid[4] name[s] mode[4] gid[4]
   * size[4] Rmkdir tag[2] qid[13]
   */

  request.fid = fid;
  request.name_len = strlen(name);
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 3 + V9FS_BIT16SZ +
                        request.name_len;
  request.header.type = V9FS_TMKDIR;
  request.header.tag = v9fs_get_tagid(client);
  memcpy(&request.buffer[offset], name, request.name_len);
  offset += request.name_len;
  memcpy(&request.buffer[offset], &mode, V9FS_BIT32SZ);
  offset += V9FS_BIT32SZ;
  memcpy(&request.buffer[offset], &gid, V9FS_BIT32SZ);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 3 + V9FS_BIT16SZ +
                    request.name_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_QIDSZ;

  return v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                         request.header.tag);
}

/****************************************************************************
 * v9fs_client_create
 ****************************************************************************/

int v9fs_client_create(FAR struct v9fs_client_s *client, uint32_t fid,
                       FAR const char *name, int oflags, int mode)
{
  FAR struct v9fs_fid_s *fidp;
  struct v9fs_create_s request;
  struct v9fs_rcreate_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  uint32_t flags = v9fs_map_open_flags(oflags);
  uint32_t gid = getgid();
  off_t offset = 0;
  int ret;

  /* size[4] Tlcreate tag[2] fid[4] name[s] flags[4] mode[4] gid[4]
   * size[4] Rlcreate tag[2] qid[13] iounit[4]
   */

  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      return -ENOENT;
    }

  request.fid = fid;
  request.name_len = strlen(name);
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 4 + V9FS_BIT16SZ +
                        request.name_len;
  request.header.type = V9FS_TLCREATE;
  request.header.tag = v9fs_get_tagid(client);
  memcpy(&request.buffer[offset], name, request.name_len);
  offset += request.name_len;
  memcpy(&request.buffer[offset], &flags, V9FS_BIT32SZ);
  offset += V9FS_BIT32SZ;
  memcpy(&request.buffer[offset], &mode, V9FS_BIT32SZ);
  offset += V9FS_BIT32SZ;
  memcpy(&request.buffer[offset], &gid, V9FS_BIT32SZ);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 4 + V9FS_BIT16SZ +
                    request.name_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_QIDSZ + V9FS_BIT32SZ;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  fidp->iounit = response.iounit;
  if (!fidp->iounit || fidp->iounit > client->msize - V9FS_IOHDRSZ)
    {
      fidp->iounit = client->msize - V9FS_IOHDRSZ;
    }

  return 0;
}

/****************************************************************************
 * v9fs_client_open
 ****************************************************************************/

int v9fs_client_open(FAR struct v9fs_client_s *client,
                     uint32_t fid, int oflags)
{
  FAR struct v9fs_fid_s *fidp;
  struct v9fs_open_s request;
  struct v9fs_ropen_s response;
  struct iovec wiov[1];
  struct iovec riov[1];
  int ret;

  /* size[4] Tlopen tag[2] fid[4] flags[4]
   * size[4] Rlopen tag[2] qid[13] iounit[4]
   */

  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      return -ENOENT;
    }

  request.fid = fid;
  request.flags = v9fs_map_open_flags(oflags);
  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 2;
  request.header.type = V9FS_TLOPEN;
  request.header.tag = v9fs_get_tagid(client);

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 2;
  riov[0].iov_base = &response;
  riov[0].iov_len = V9FS_HDRSZ + V9FS_QIDSZ + V9FS_BIT32SZ;

  ret = v9fs_client_rpc(client->transport, wiov, 1, riov, 1,
                        request.header.tag);
  if (ret < 0)
    {
      return ret;
    }

  fidp->iounit = response.iounit;
  if (!fidp->iounit || fidp->iounit > client->msize - V9FS_IOHDRSZ)
    {
      fidp->iounit = client->msize - V9FS_IOHDRSZ;
    }

  return 0;
}

/****************************************************************************
 * v9fs_client_getname
 ****************************************************************************/

int v9fs_client_getname(FAR struct v9fs_client_s *client, uint32_t fid,
                        FAR char *path)
{
  FAR struct v9fs_fid_s *fidp;

  nxmutex_lock(&client->lock);
  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      nxmutex_unlock(&client->lock);
      return -ENOENT;
    }

  strlcat(path, fidp->relpath, PATH_MAX);
  nxmutex_unlock(&client->lock);
  return 0;
}

/****************************************************************************
 * v9fs_client_walk
 ****************************************************************************/

int v9fs_client_walk(FAR struct v9fs_client_s *client, FAR const char *path,
                     FAR const char **childname)
{
  struct v9fs_walk_s request;
  struct v9fs_rwalk_s response;
  FAR char *request_payload;
  FAR char *response_payload;
  FAR const char *start;
  FAR const char *end;
  struct iovec wiov[2];
  struct iovec riov[2];
  uint16_t nwname = 0;
  uint32_t newfid;
  size_t total_len = 0;
  size_t offset = 0;
  size_t name_len;
  int ret;

  /* size[4] Twalk tag[2] fid[4] newfid[4] nwname[2] nwname*(wname[s])
   * size[4] Rwalk tag[2] nwqid[2] nwqid*(wqid[13])
   */

  /* Parse path info, We need to skip parsing the root path */

  start = path;
  while (*start != '\0')
    {
      end = strchr(start, '/');
      if (end == NULL)
        {
          if (childname == NULL)
            {
              nwname++;
              total_len += strlen(start);
            }

          break;
        }
      else if (end != start)
        {
          nwname++;
          total_len += end - start;
        }

      start = end + 1;
    }

  /* Request = sizeof(struct v9fs_walk_s) + nwname * sizeof(uint16_t) +
   *           total_len
   * Response = sizeof(struct v9fs_rwalk_s) + V9FS_QIDSZ * (nwname + 1)
   * When PATH_MAX = 255, we support a maximum recursive depth of
   * (255 - 7 - 2) / 13 = 18 layers, PATH MAX size affects recursion depth
   */

  total_len += V9FS_BIT16SZ * nwname;
  if (total_len > PATH_MAX || nwname * V9FS_QIDSZ > PATH_MAX)
    {
      return -ENAMETOOLONG;
    }

  request_payload = lib_get_pathbuffer();
  if (request_payload == NULL)
    {
      return -ENOMEM;
    }

  response_payload = lib_get_pathbuffer();
  if (response_payload == NULL)
    {
      lib_put_pathbuffer(request_payload);
      return -ENOMEM;
    }

  newfid = v9fs_fid_create(client, path);
  if (newfid < 0)
    {
      goto err;
    }

  request.header.size = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ +
                        total_len;
  request.header.type = V9FS_TWALK;
  request.header.tag = v9fs_get_tagid(client);
  request.fid = client->root_fid;
  request.newfid = newfid;
  request.nwname = nwname;
  start = path;
  while (*start != '\0')
    {
      end = strchr(start, '/');
      if (end == start)
        {
          start++;
          continue;
        }

      if (end == NULL)
        {
          if (childname != NULL)
            {
              *childname = start;
              break;
            }

          name_len = strlen(start);
          end = start + name_len - 1;
        }
      else
        {
          name_len = end - start;
        }

      memcpy(&request_payload[offset], &name_len, sizeof(uint16_t));
      offset += sizeof(uint16_t);
      memcpy(&request_payload[offset], start, name_len);
      offset += name_len;
      start = end + 1;
    }

  wiov[0].iov_base = &request;
  wiov[0].iov_len = V9FS_HDRSZ + V9FS_BIT32SZ * 2 + V9FS_BIT16SZ;
  wiov[1].iov_base = request_payload;
  wiov[1].iov_len = total_len;
  riov[0].iov_base = &response;
  riov[0].iov_len = sizeof(response);
  riov[1].iov_base = response_payload;
  riov[1].iov_len = V9FS_QIDSZ * (nwname + 1);

  ret = v9fs_client_rpc(client->transport, wiov, total_len ? 2 : 1,
                        riov, 2, request.header.tag);
  if (ret < 0)
    {
      v9fs_fid_destroy(client, newfid);
      newfid = ret;
    }

  /* There are differences in different server implementations, so it is
   * necessary to check whether the returned nwqid satisfies the requested
   * number.
   */

  if (response.nwqid != nwname)
    {
      newfid = -ENOENT;
    }

err:
  lib_put_pathbuffer(request_payload);
  lib_put_pathbuffer(response_payload);
  return newfid;
}

/****************************************************************************
 * v9fs_client_init
 ****************************************************************************/

int v9fs_client_init(FAR struct v9fs_client_s *client,
                     FAR const char *data)
{
  FAR const char *options;
  FAR char *aname;
  char transport[NAME_MAX];
  char uname[NAME_MAX] =
    {
      0
    };

  size_t length;
  int ret;

  aname = lib_get_pathbuffer();
  if (aname == NULL)
    {
      return -ENOMEM;
    }

  aname[0] = '\0';

  /* Parse commandline */

  options = data;
  while (*options != '\0')
    {
      FAR const char *sep = strchr(options, ',');
      length = sep ? sep - options : strlen(options);

      if (strncmp(options, "uname=", 6) == 0)
        {
          strlcpy(uname, options + 6, length - 5);
        }
      else if (strncmp(options, "aname=", 6) == 0)
        {
          strlcpy(aname, options + 6, length - 5);
        }
      else if (strncmp(options, "trans=", 6) == 0)
        {
          strlcpy(transport, options + 6, length - 5);
        }
      else if (strncmp(options, "msize=", 6) == 0)
        {
          client->msize = atoi(options + 6);
        }

      options += length + 1;
    }

  if (client->msize == 0)
    {
      /* Set default size */

      client->msize = (CONFIG_V9FS_DEFAULT_MSIZE + V9FS_IOHDRSZ);
    }

  /* Initialize the client function */

  ret = v9fs_transport_create(&client->transport, transport, data);
  if (ret < 0)
    {
      lib_put_pathbuffer(aname);
      return ret;
    }

  client->fids = idr_init_base(1);
  if (client->fids == NULL)
    {
      v9fs_transport_destroy(client->transport);
      lib_put_pathbuffer(aname);
      return -ENOMEM;
    }

  nxmutex_init(&client->lock);

  /* Do Version */

  ret = v9fs_client_version(client);
  if (ret < 0)
    {
      goto out;
    }

  /* Do Attach */

  ret = v9fs_client_attach(client, V9FS_NOFID, uname, aname);
  if (ret < 0)
    {
      goto out;
    }

  client->root_fid = ret;
  client->tag_id = 1;
  lib_put_pathbuffer(aname);
  return 0;

out:
  v9fs_transport_destroy(client->transport);
  nxmutex_destroy(&client->lock);
  idr_destroy(client->fids);
  lib_put_pathbuffer(aname);
  return ret;
}

/****************************************************************************
 * v9fs_client_uninit
 ****************************************************************************/

int v9fs_client_uninit(FAR struct v9fs_client_s *client)
{
  int ret;

  ret = v9fs_client_clunk(client, client->root_fid);
  if (ret < 0)
    {
      return ret;
    }

  v9fs_transport_destroy(client->transport);
  nxmutex_destroy(&client->lock);
  idr_destroy(client->fids);
  return 0;
}

/****************************************************************************
 * v9fs_transport_done
 ****************************************************************************/

void v9fs_transport_done(FAR struct v9fs_payload_s *cookie, int ret)
{
  FAR struct v9fs_lerror_s *error = cookie->riov[0].iov_base;

  /* Recive message = riov[0] (Header) + iov[1] (Payload) + ...
   * So we first check if the type on the rheader is RLERROR. If it is,
   * it means that payload[1] is ecode.
   */

  if (error->header.type == V9FS_RLERROR)
    {
      /* Therefore, we assign the error code to the ecode of the cookie
       * and check it in the next process
       */

      cookie->ret = -error->ecode;
    }
  else
    {
      cookie->ret = ret;
    }

  nxsem_post(&cookie->resp);
}

/****************************************************************************
 * v9fs_fid_put
 ****************************************************************************/

int v9fs_fid_put(FAR struct v9fs_client_s *client, uint32_t fid)
{
  FAR struct v9fs_fid_s *fidp;

  nxmutex_lock(&client->lock);
  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      nxmutex_unlock(&client->lock);
      return -ENOENT;
    }

  fidp->refcount--;
  if (fidp->refcount > 0)
    {
      nxmutex_unlock(&client->lock);
      return fidp->refcount;
    }

  nxmutex_unlock(&client->lock);
  return v9fs_client_clunk(client, fid);
}

/****************************************************************************
 * v9fs_fid_get
 ****************************************************************************/

int v9fs_fid_get(FAR struct v9fs_client_s *client, uint32_t fid)
{
  FAR struct v9fs_fid_s *fidp;

  nxmutex_lock(&client->lock);
  fidp = idr_find(client->fids, fid);
  if (fidp == NULL)
    {
      nxmutex_unlock(&client->lock);
      return -ENOENT;
    }

  fidp->refcount++;
  nxmutex_unlock(&client->lock);
  return 0;
}
