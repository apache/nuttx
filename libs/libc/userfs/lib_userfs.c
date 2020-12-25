/****************************************************************************
 * libs/libc/userfs/lib_userfs.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/socket.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/fs/userfs.h>
#include <nuttx/semaphore.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct userfs_info_s
{
  FAR const struct userfs_operations_s *userops;
  FAR void *volinfo;          /* Data that accompanies the user callbacks */
  struct sockaddr_in client;  /* Client to send response back to */
  int16_t sockfd;             /* Server socket */
  uint16_t iolen;             /* Size of I/O buffer */
  uint16_t mxwrite;           /* The max size of a write data */
  uint8_t iobuffer[1];        /* I/O buffer.  Actual size is iolen. */
};

#define SIZEOF_USERFS_INFO_S(n) (sizeof(struct userfs_info_s) + (n) - 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* REVISIT:  This is insufficient in the KERNEL build.  In that build mode,
 * there will be multiple instances of these variables and the logic will
 * not generate unique instance numbers.
 */

static sem_t   g_userfs_exclsem = SEM_INITIALIZER(1);
static uint8_t g_userfs_next_instance;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userfs_server_portno
 ****************************************************************************/

static inline uint16_t userfs_server_portno(void)
{
  int ret;

  ret = _SEM_WAIT(&g_userfs_exclsem);
  if (ret >= 0)
    {
      /* Get the next instance number.
       *
       * REVISIT: Here we really should verify that other UserFs
       * exists with the same instance number.  That could
       * happen if g_userfs_next_instance were to wrap around.
       */

      ret = USERFS_SERVER_PORTBASE | g_userfs_next_instance++;
      _SEM_POST(&g_userfs_exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: userfs_*_dispatch
 ****************************************************************************/

static inline int userfs_open_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_open_request_s *req, size_t reqlen)
{
  struct userfs_open_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;
  int ret;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_OPEN_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_OPEN_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request.
   *
   * REVISIT: In the kernel build openinfo will be valid only in the
   * context of this process.
   */

  DEBUGASSERT(info->userops != NULL && info->userops->open != NULL);
  resp.ret  = info->userops->open(info->volinfo, req->relpath, req->oflags,
                                  req->mode, &resp.openinfo);

  /* Send the response */

  resp.resp = USERFS_RESP_OPEN;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_open_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ret = -errno;
      ferr("ERROR: Send open response failed: %d\n", ret);
      return ret;
    }

  /* REVISIT: Partial sends are not supported */

  DEBUGASSERT(nsent == sizeof(struct userfs_open_response_s));
  return OK;
}

static inline int userfs_close_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_close_request_s *req, size_t reqlen)
{
  struct userfs_close_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_close_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->close != NULL);
  resp.ret  = info->userops->close(info->volinfo, req->openinfo);

  /* Send the response */

  resp.resp = USERFS_RESP_CLOSE;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_close_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_read_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_read_request_s *req, size_t reqlen)
{
  FAR struct userfs_read_response_s *resp;
  size_t readlen;
  size_t resplen;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_read_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  readlen = req->readlen;
  if (readlen > info->mxwrite)
    {
      readlen = info->mxwrite;
    }

  resp = (FAR struct userfs_read_response_s *)info->iobuffer;

  DEBUGASSERT(info->userops != NULL && info->userops->read != NULL);
  resp->nread = info->userops->read(info->volinfo, req->openinfo,
                                    resp->rddata, readlen);

  /* Send the response */

  resp->resp  = USERFS_RESP_READ;
  resplen     = SIZEOF_USERFS_READ_RESPONSE_S(resp->nread);
  nsent       = sendto(info->sockfd, resp, resplen, 0,
                       (FAR struct sockaddr *)&info->client,
                       sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_write_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_write_request_s *req, size_t reqlen)
{
  struct userfs_write_response_s resp;
  size_t writelen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_WRITE_REQUEST_S(0))
    {
      return -EINVAL;
    }

  writelen = req->writelen;
  if (writelen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_WRITE_REQUEST_S(writelen);
  if (expected != reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->write != NULL);
  resp.nwritten = info->userops->write(info->volinfo, req->openinfo,
                                       req->wrdata, writelen);

  /* Send the response */

  resp.resp     = USERFS_RESP_WRITE;
  nsent         = sendto(info->sockfd, &resp,
                         sizeof(struct userfs_write_response_s),
                         0, (FAR struct sockaddr *)&info->client,
                         sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_seek_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_seek_request_s *req, size_t reqlen)
{
  struct userfs_seek_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_seek_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->seek != NULL);
  resp.ret  = info->userops->seek(info->volinfo, req->openinfo, req->offset,
                                  req->whence);

  /* Send the response */

  resp.resp = USERFS_RESP_SEEK;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_seek_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_ioctl_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_ioctl_request_s *req, size_t reqlen)
{
  struct userfs_ioctl_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_ioctl_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->ioctl != NULL);
  resp.ret  = info->userops->ioctl(info->volinfo, req->openinfo, req->cmd,
                                   req->arg);

  /* Send the response */

  resp.resp = USERFS_RESP_IOCTL;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_ioctl_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_sync_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_sync_request_s *req, size_t reqlen)
{
  struct userfs_sync_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_sync_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->sync != NULL);
  resp.ret  = info->userops->sync(info->volinfo, req->openinfo);

  /* Send the response */

  resp.resp = USERFS_RESP_SYNC;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_sync_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_dup_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_dup_request_s *req, size_t reqlen)
{
  struct userfs_dup_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_dup_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->dup != NULL);
  resp.ret  = info->userops->dup(info->volinfo,
                                 req->openinfo, &resp.openinfo);

  /* Send the response */

  resp.resp = USERFS_RESP_DUP;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_dup_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_fstat_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_fstat_request_s *req, size_t reqlen)
{
  struct userfs_fstat_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_fstat_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->fstat != NULL);
  resp.ret  = info->userops->fstat(info->volinfo, req->openinfo, &resp.buf);

  /* Send the response */

  resp.resp = USERFS_RESP_FSTAT;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_fstat_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_truncate_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_truncate_request_s *req, size_t reqlen)
{
  struct userfs_truncate_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_truncate_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->truncate != NULL);
  resp.ret  = info->userops->truncate(info->volinfo,
                                      req->openinfo, req->length);

  /* Send the response */

  resp.resp = USERFS_RESP_FSTAT;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_truncate_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_opendir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_opendir_request_s *req, size_t reqlen)
{
  struct userfs_opendir_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_OPENDIR_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_OPENDIR_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->opendir != NULL);
  resp.ret  = info->userops->opendir(info->volinfo, req->relpath, &resp.dir);

  /* Send the response */

  resp.resp = USERFS_RESP_OPENDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_opendir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_closedir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_closedir_request_s *req, size_t reqlen)
{
  struct userfs_closedir_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_closedir_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->closedir != NULL);
  resp.ret  = info->userops->closedir(info->volinfo, req->dir);

  /* Send the response */

  resp.resp = USERFS_RESP_CLOSEDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_closedir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_readdir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_readdir_request_s *req, size_t reqlen)
{
  struct userfs_readdir_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_readdir_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->readdir != NULL);
  resp.ret  = info->userops->readdir(info->volinfo, req->dir, &resp.entry);

  /* Send the response */

  resp.resp = USERFS_RESP_READDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_readdir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_rewinddir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_rewinddir_request_s *req, size_t reqlen)
{
  struct userfs_rewinddir_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_rewinddir_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->rewinddir != NULL);
  resp.ret  = info->userops->rewinddir(info->volinfo, req->dir);

  /* Send the response */

  resp.resp = USERFS_RESP_REWINDDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_rewinddir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_statfs_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_statfs_request_s *req, size_t reqlen)
{
  struct userfs_statfs_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_statfs_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->statfs != NULL);
  resp.ret  = info->userops->statfs(info->volinfo, &resp.buf);

  /* Send the response */

  resp.resp = USERFS_RESP_STATFS;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_statfs_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_unlink_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_unlink_request_s *req, size_t reqlen)
{
  struct userfs_unlink_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_UNLINK_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_UNLINK_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->unlink != NULL);
  resp.ret  = info->userops->unlink(info->volinfo, req->relpath);

  /* Send the response */

  resp.resp = USERFS_RESP_UNLINK;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_unlink_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_mkdir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_mkdir_request_s *req, size_t reqlen)
{
  struct userfs_mkdir_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_MKDIR_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_MKDIR_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->mkdir != NULL);
  resp.ret  = info->userops->mkdir(info->volinfo, req->relpath, req->mode);

  /* Send the response */

  resp.resp = USERFS_RESP_MKDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_mkdir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_rmdir_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_rmdir_request_s *req, size_t reqlen)
{
  struct userfs_rmdir_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_MKDIR_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_MKDIR_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->rmdir != NULL);
  resp.ret  = info->userops->rmdir(info->volinfo, req->relpath);

  /* Send the response */

  resp.resp = USERFS_RESP_RMDIR;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_rmdir_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_rename_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_rename_request_s *req, size_t reqlen)
{
  struct userfs_rename_response_s resp;
  FAR char *newrelpath;
  unsigned int newoffset;
  int oldpathlen;
  int newpathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_RENAME_REQUEST_S(0, 0))
    {
      return -EINVAL;
    }

  oldpathlen = strlen(req->oldrelpath);
  newoffset  = req->newoffset;

  if (oldpathlen >= newoffset)
    {
      return -EINVAL;
    }

  newrelpath = &req->oldrelpath[newoffset];
  newpathlen = strlen(newrelpath);

  if ((newpathlen + newoffset) > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_RENAME_REQUEST_S(newoffset, newpathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->rename != NULL);
  resp.ret  = info->userops->rename(info->volinfo, req->oldrelpath,
                                    newrelpath);

  /* Send the response */

  resp.resp = USERFS_RESP_RENAME;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_rename_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_stat_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_stat_request_s *req, size_t reqlen)
{
  struct userfs_stat_response_s resp;
  int pathlen;
  size_t expected;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen < SIZEOF_USERFS_STAT_REQUEST_S(0))
    {
      return -EINVAL;
    }

  pathlen = strlen(req->relpath);
  if (pathlen > info->mxwrite)
    {
      return -EINVAL;
    }

  expected = SIZEOF_USERFS_STAT_REQUEST_S(pathlen);
  if (expected >= reqlen)
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->stat != NULL);
  resp.ret  = info->userops->stat(info->volinfo, req->relpath, &resp.buf);

  /* Send the response */

  resp.resp = USERFS_RESP_STAT;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_stat_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  return nsent < 0 ? nsent : OK;
}

static inline int userfs_destroy_dispatch(FAR struct userfs_info_s *info,
                   FAR struct userfs_destroy_request_s *req, size_t reqlen)
{
  struct userfs_destroy_response_s resp;
  ssize_t nsent;

  /* Verify the request size */

  if (reqlen != sizeof(struct userfs_destroy_request_s))
    {
      return -EINVAL;
    }

  /* Dispatch the request */

  DEBUGASSERT(info->userops != NULL && info->userops->destroy != NULL);
  resp.ret  = info->userops->destroy(info->volinfo);

  /* Send the response */

  resp.resp = USERFS_RESP_DESTROY;
  nsent     = sendto(info->sockfd, &resp,
                     sizeof(struct userfs_destroy_response_s),
                     0, (FAR struct sockaddr *)&info->client,
                     sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      int ret = -errno;
      ferr("ERROR: sendto failed: %d\n", ret);
      return ret;
    }

  /* Speical case of resp.ret indicates an error, the destruction was
   * refused.  So we need to return success in this case so that we
   * continue processing requests.
   */

  return resp.ret < 0 ? OK : -ENOTCONN;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userfs_run
 *
 * Description:
 *   Start the UserFS server on the current thread.  This function will mount
 *   the UserFS file system and will not return until that file system has
 *   been unmounted.
 *
 *   userfs_run() implements the UserFS server. It performs there operations:
 *
 *   1. It configures and creates the UserFS file system and
 *   2. Mounts the user file system at the provide mount point path.
 *   2. Receives file system requests on the LocalHost socket with
 *      server port 0x83nn where nn is the same as above,
 *   3. Received file system requests are marshaled and dispatch to the
 *      user file system via callbacks to the operations provided by
 *      "userops", and
 *   3. Returns file system responses generated by the callbacks to the
 *      LocalHost client socket.
 *
 *   NOTE:  This is a user function that is implemented as part of the
 *   NuttX C library and is intended to be called by application logic.
 *
 * Input Parameters:
 *   mountpt  - Mountpoint path
 *   userops  - The caller operations that implement the file system
 *              interface.
 *   volinfo  - Private volume data that will be provided in all struct
 *              userfs_operations_s methods.
 *   mxwrite  - The max size of a write data
 *
 *  Returned Value:
 *    This function does not return unless the file system is unmounted (OK)
 *    or unless an error is encountered.  In the event of an error, the
 *    returned value is a negated errno value indicating the nature of the
 *    error.
 *
 ****************************************************************************/

int userfs_run(FAR const char *mountpt,
               FAR const struct userfs_operations_s *userops,
               FAR void *volinfo, size_t mxwrite)
{
  FAR struct userfs_info_s *info;
  FAR struct userfs_config_s config;
  struct sockaddr_in server;
  unsigned int iolen;
  socklen_t addrlen;
  ssize_t nread;
  int ret;

  DEBUGASSERT(mountpt != NULL && userops != NULL && mxwrite <= UINT16_MAX);
  DEBUGASSERT(mxwrite > 0 && mxwrite <= (UINT16_MAX - USERFS_REQ_MAXSIZE));

  /* Allocate a state structure with an I/O buffer to receive UserFS
   * requests
   */

  iolen = USERFS_REQ_MAXSIZE + mxwrite;
  info  = lib_zalloc(SIZEOF_USERFS_INFO_S(iolen));
  if (info == NULL)
    {
      ferr("ERROR: Failed to allocate state structure\n");
      return -ENOMEM;
    }

  /* Initialize the state structure */

  info->userops   = userops;
  info->volinfo   = volinfo;
  info->iolen     = iolen;
  info->mxwrite   = mxwrite;

  /* Create the UserFS configuration that will be provided as optional
   * data when the UserFS is mounted.
   */

  config.mxwrite  = mxwrite;
  config.portno   = userfs_server_portno();

  /* Mounts the user file system at the provided mount point path. */

  ret = mount(NULL, mountpt, "userfs", 0, (FAR const void *)&config);
  if (ret < 0)
    {
      ret = -get_errno();
      ferr("ERROR: mount() failed: %d\n", ret);
      goto errout_with_info;
    }

  /* Create a new LocalHost UDP server socket */

  info->sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (info->sockfd < 0)
    {
      ret = -get_errno();
      ferr("ERROR: socket() failed: %d\n", ret);
      goto errout_with_info;
    }

  /* Bind the socket to a server port number */

  server.sin_family      = AF_INET;
  server.sin_port        = htons(config.portno);
  server.sin_addr.s_addr = HTONL(INADDR_LOOPBACK);

  ret = bind(info->sockfd, (struct sockaddr *)&server,
             sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      ret = -get_errno();
      ferr("ERROR: bind() failed: %d\n", ret);
      goto errout_with_sockfd;
    }

  /* Receive file system requests on the POSIX message queue as long
   * as the mount persists.
   */

  do
    {
      /* Receive the next file system request */

      finfo("Receiving up %u bytes\n", info->iolen);
      addrlen = sizeof(struct sockaddr_in);
      nread   = recvfrom(info->sockfd, info->iobuffer, info->iolen, 0,
                         (FAR struct sockaddr *)&info->client,
                         &addrlen);
      if (nread < 0)
        {
          ret = -get_errno();
          ferr("ERROR: recvfrom failed: %d\n", ret);
          goto errout_with_sockfd;
        }

      DEBUGASSERT(addrlen == sizeof(struct sockaddr_in));

      /* Process the request according to its request ID */

      DEBUGASSERT(nread >= sizeof(uint8_t));
      switch (*info->iobuffer)
        {
          case USERFS_REQ_OPEN:
            ret = userfs_open_dispatch(info,
                   (FAR struct userfs_open_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_CLOSE:
            ret = userfs_close_dispatch(info,
                   (FAR struct userfs_close_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_READ:
            ret = userfs_read_dispatch(info,
                   (FAR struct userfs_read_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_WRITE:
            ret = userfs_write_dispatch(info,
                   (FAR struct userfs_write_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_SEEK:
            ret = userfs_seek_dispatch(info,
                   (FAR struct userfs_seek_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_IOCTL:
            ret = userfs_ioctl_dispatch(info,
                   (FAR struct userfs_ioctl_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_SYNC:
            ret = userfs_sync_dispatch(info,
                   (FAR struct userfs_sync_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_DUP:
            ret = userfs_dup_dispatch(info,
                   (FAR struct userfs_dup_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_FSTAT:
            ret = userfs_fstat_dispatch(info,
                   (FAR struct userfs_fstat_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_TRUNCATE:
            ret = userfs_truncate_dispatch(info,
                   (FAR struct userfs_truncate_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_OPENDIR:
            ret = userfs_opendir_dispatch(info,
                   (FAR struct userfs_opendir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_CLOSEDIR:
            ret = userfs_closedir_dispatch(info,
                   (FAR struct userfs_closedir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_READDIR:
            ret = userfs_readdir_dispatch(info,
                   (FAR struct userfs_readdir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_REWINDDIR:
            ret = userfs_rewinddir_dispatch(info,
                   (FAR struct userfs_rewinddir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_STATFS:
            ret = userfs_statfs_dispatch(info,
                   (FAR struct userfs_statfs_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_UNLINK:
            ret = userfs_unlink_dispatch(info,
                   (FAR struct userfs_unlink_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_MKDIR:
            ret = userfs_mkdir_dispatch(info,
                   (FAR struct userfs_mkdir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_RMDIR:
            ret = userfs_rmdir_dispatch(info,
                   (FAR struct userfs_rmdir_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_RENAME:
            ret = userfs_rename_dispatch(info,
                   (FAR struct userfs_rename_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_STAT:
            ret = userfs_stat_dispatch(info,
                   (FAR struct userfs_stat_request_s *)info->iobuffer,
                   nread);
            break;

          case USERFS_REQ_DESTROY:
            ret = userfs_destroy_dispatch(info,
                   (FAR struct userfs_destroy_request_s *)info->iobuffer,
                   nread);
            break;

          default:
            ferr("ERROR: Unrecognized request received: %u\n",
                 *info->iobuffer);
            ret = -EINVAL;
            break;
        }
    }
  while (ret == OK);

  /* Close the LocalHost socket */

errout_with_sockfd:
  close(info->sockfd);

  /* Free the IO Buffer */

errout_with_info:
  lib_free(info);
  return ret;
}
