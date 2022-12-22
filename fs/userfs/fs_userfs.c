/****************************************************************************
 * fs/userfs/fs_userfs.c
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

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/userfs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/net/net.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IOBUFFER_SIZE(p) (USERFS_REQ_MAXSIZE + (p)->mxwrite)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct userfs_dir_s
{
  struct fs_dirent_s base;
  FAR void *dir;
};

/* This structure holds the internal state of the UserFS proxy */

struct userfs_state_s
{
  /* Fields copied from struct userfs_config_s */

  size_t mxwrite;            /* The max size of a write data */

  /* Internal state */

  struct socket psock;       /* Client socket instance */
  struct sockaddr_in server; /* Server address */
  mutex_t lock;              /* Exclusive access for request-response sequence */

  /* I/O Buffer (actual size depends on USERFS_REQ_MAXSIZE and the configured
   * mxwrite).
   */

  uint8_t iobuffer[1];
};

#define SIZEOF_USERFS_STATE_S(n) (sizeof(struct userfs_state_s) + (n) - 1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     userfs_open(FAR struct file *filep, const char *relpath,
                 int oflags, mode_t mode);
static int     userfs_close(FAR struct file *filep);
static ssize_t userfs_read(FAR struct file *filep, char *buffer,
                 size_t buflen);
static ssize_t userfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t userfs_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static off_t   userfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     userfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

static int     userfs_sync(FAR struct file *filep);
static int     userfs_dup(FAR const struct file *oldp,
                          FAR struct file *newp);
static int     userfs_fstat(FAR const struct file *filep,
                 FAR struct stat *buf);
static int     userfs_truncate(FAR struct file *filep, off_t length);

static int     userfs_opendir(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct fs_dirent_s **dir);
static int     userfs_closedir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     userfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir,
                 FAR struct dirent *entry);
static int     userfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

static int     userfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                 FAR void **handle);
static int     userfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                 unsigned int flags);
static int     userfs_statfs(FAR struct inode *mountpt,
                 FAR struct statfs *buf);

static int     userfs_unlink(FAR struct inode *mountpt,
                 FAR const char *relpath);
static int     userfs_mkdir(FAR struct inode *mountpt,
                 FAR const char *relpath, mode_t mode);
static int     userfs_rmdir(FAR struct inode *mountpt,
                 FAR const char *relpath);
static int     userfs_rename(FAR struct inode *mountpt,
                 FAR const char *oldrelpath, FAR const char *newrelpath);
static int     userfs_stat(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct stat *buf);
static int     userfs_fchstat(FAR const struct file *filep,
                 FAR const struct stat *buf, int flags);
static int     userfs_chstat(FAR struct inode *mountpt,
                 FAR const char *relpath,
                 FAR const struct stat *buf, int flags);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations userfs_operations =
{
  userfs_open,       /* open */
  userfs_close,      /* close */
  userfs_read,       /* read */
  userfs_write,      /* write */
  userfs_seek,       /* seek */
  userfs_ioctl,      /* ioctl */
  userfs_truncate,   /* truncate */

  userfs_sync,       /* sync */
  userfs_dup,        /* dup */
  userfs_fstat,      /* fstat */
  userfs_fchstat,    /* fchstat */

  userfs_opendir,    /* opendir */
  userfs_closedir,   /* closedir */
  userfs_readdir,    /* readdir */
  userfs_rewinddir,  /* rewinddir */

  userfs_bind,       /* bind */
  userfs_unbind,     /* unbind */
  userfs_statfs,     /* statfs */

  userfs_unlink,     /* unlink */
  userfs_mkdir,      /* mkdir */
  userfs_rmdir,      /* rmdir */
  userfs_rename,     /* rename */
  userfs_stat,       /* stat */
  userfs_chstat      /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userfs_open
 ****************************************************************************/

static int userfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_open_request_s *req;
  FAR struct userfs_open_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  finfo("Open '%s'\n", relpath);

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req         = (FAR struct userfs_open_request_s *)priv->iobuffer;
  req->req    = USERFS_REQ_OPEN;
  req->oflags = oflags;
  req->mode   = mode;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_OPEN_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_open_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  /* Save the returned openinfo as the filep private data. */

  resp = (FAR struct userfs_open_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_OPEN)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  filep->f_priv = resp->openinfo;
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_close
 ****************************************************************************/

static int userfs_close(FAR struct file *filep)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_close_request_s *req;
  FAR struct userfs_close_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_close_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_CLOSE;
  req->openinfo = filep->f_priv;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_close_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_close_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_close_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_CLOSE)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  if (resp->ret >= 0)
    {
      filep->f_priv = NULL;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_read
 ****************************************************************************/

static ssize_t userfs_read(FAR struct file *filep, char *buffer,
                           size_t buflen)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_read_request_s *req;
  FAR struct userfs_read_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int respsize;
  int ret;

  finfo("Read %zu bytes from offset %jd\n", buflen, (intmax_t)filep->f_pos);

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_read_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_READ;
  req->openinfo = filep->f_priv;
  req->readlen  = buflen;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_read_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd < SIZEOF_USERFS_READ_RESPONSE_S(0))
    {
      ferr("ERROR: Response too small: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_read_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_READ)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  if (resp->nread > buflen)
    {
      ferr("ERROR: Response size too large: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  respsize = SIZEOF_USERFS_READ_RESPONSE_S(resp->nread);
  if (respsize != nrecvd)
    {
      ferr("ERROR: Incorrect response size: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  /* Copy the received data to the user buffer */

  memcpy(buffer, resp->rddata, resp->nread);
  return resp->nread;
}

/****************************************************************************
 * Name: userfs_write
 ****************************************************************************/

static ssize_t userfs_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_write_request_s *req;
  FAR struct userfs_write_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  finfo("Write %zu bytes to offset %jd\n", buflen, (intmax_t)filep->f_pos);

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Perform multiple writes if the write length exceeds the configured
   * maximum (mxwrite).
   */

  if (buflen > priv->mxwrite)
    {
      return -E2BIG; /* No implemented yet */
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_write_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_WRITE;
  req->openinfo = filep->f_priv;
  req->writelen = buflen;
  memcpy(req->wrdata, buffer, buflen);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_WRITE_REQUEST_S(buflen), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_write_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_write_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_WRITE)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->nwritten;
}

/****************************************************************************
 * Name: userfs_seek
 ****************************************************************************/

static off_t userfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_seek_request_s *req;
  FAR struct userfs_seek_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  finfo("Offset %lu bytes to whence=%d\n", (unsigned long)offset, whence);

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_seek_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_SEEK;
  req->openinfo = filep->f_priv;
  req->offset   = offset;
  req->whence   = whence;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_seek_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_seek_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_seek_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_SEEK)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_ioctl
 ****************************************************************************/

static int userfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_ioctl_request_s *req;
  FAR struct userfs_ioctl_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_ioctl_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_IOCTL;
  req->openinfo = filep->f_priv;
  req->cmd      = cmd;
  req->arg      = arg;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_ioctl_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_ioctl_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_ioctl_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_IOCTL)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_sync
 ****************************************************************************/

static int userfs_sync(FAR struct file *filep)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_sync_request_s *req;
  FAR struct userfs_sync_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_sync_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_SYNC;
  req->openinfo = filep->f_priv;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_sync_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_sync_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_sync_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_SYNC)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int userfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_dup_request_s *req;
  FAR struct userfs_dup_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  finfo("Dup %p->%p\n", oldp, newp);

  DEBUGASSERT(oldp != NULL &&
              oldp->f_inode != NULL &&
              oldp->f_inode->i_private != NULL);
  priv = oldp->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_dup_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_DUP;
  req->openinfo = oldp->f_priv;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_dup_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_dup_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_dup_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_DUP)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  newp->f_priv = resp->openinfo;
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int userfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_fstat_request_s *req;
  FAR struct userfs_fstat_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_fstat_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_FSTAT;
  req->openinfo = filep->f_priv;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_fstat_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_fstat_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_fstat_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_FSTAT)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Return the status of the directory entry */

  DEBUGASSERT(buf != NULL);
  memcpy(buf, &resp->buf, sizeof(struct stat));
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   descriptor 'filep'.
 *
 ****************************************************************************/

static int userfs_fchstat(FAR const struct file *filep,
                          FAR const struct stat *buf, int flags)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_fchstat_request_s *req;
  FAR struct userfs_fchstat_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_fchstat_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_FCHSTAT;
  req->openinfo = filep->f_priv;
  req->buf      = *buf;
  req->flags    = flags;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_fchstat_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %zd\n", nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %zd\n", nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_fchstat_response_s))
    {
      ferr("ERROR: Response size incorrect: %zd\n", nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_fchstat_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_FCHSTAT)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_truncate
 *
 * Description:
 *   Set the size of the regular file referred to by 'filep' to 'length'
 *
 ****************************************************************************/

static int userfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_truncate_request_s *req;
  FAR struct userfs_truncate_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(filep != NULL &&
              filep->f_inode != NULL &&
              filep->f_inode->i_private != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req           = (FAR struct userfs_truncate_request_s *)priv->iobuffer;
  req->req      = USERFS_REQ_TRUNCATE;
  req->openinfo = filep->f_priv;
  req->length   = length;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_truncate_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_truncate_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_truncate_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_FSTAT)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Return the result of truncate operation */

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_opendir
 *
 * Description:
 *   Open a directory
 *
 ****************************************************************************/

static int userfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s **dir)
{
  FAR struct userfs_dir_s *udir;
  FAR struct userfs_state_s *priv;
  FAR struct userfs_opendir_request_s *req;
  FAR struct userfs_opendir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_opendir_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_OPENDIR;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_OPENDIR_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_opendir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_opendir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_OPENDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Save the opaque dir reference in struct fs_dirent_s */

  DEBUGASSERT(dir != NULL);
  udir = kmm_zalloc(sizeof(struct userfs_dir_s));
  if (udir == NULL)
    {
      return -ENOMEM;
    }

  udir->dir = resp->dir;
  *dir = (FAR struct fs_dirent_s *)udir;
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_closedir
 *
 * Description:
 *   Close a directory
 *
 ****************************************************************************/

static int userfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct userfs_dir_s *udir;
  FAR struct userfs_state_s *priv;
  FAR struct userfs_closedir_request_s *req;
  FAR struct userfs_closedir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;
  udir = (FAR struct userfs_dir_s *)dir;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_closedir_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_CLOSEDIR;
  req->dir = udir->dir;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_closedir_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_closedir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_closedir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_CLOSEDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  kmm_free(udir);
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int userfs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry)
{
  FAR struct userfs_dir_s *udir;
  FAR struct userfs_state_s *priv;
  FAR struct userfs_readdir_request_s *req;
  FAR struct userfs_readdir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;
  udir = (FAR struct userfs_dir_s *)dir;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_readdir_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_READDIR;
  req->dir = udir->dir;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_readdir_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_readdir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_readdir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_READDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Return the dirent */

  DEBUGASSERT(dir != NULL);
  memcpy(entry, &resp->entry, sizeof(struct dirent));
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int userfs_rewinddir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct userfs_dir_s *udir;
  FAR struct userfs_state_s *priv;
  FAR struct userfs_rewinddir_request_s *req;
  FAR struct userfs_rewinddir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;
  udir = (FAR struct userfs_dir_s *)dir;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_rewinddir_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_REWINDDIR;
  req->dir = udir->dir;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_rewinddir_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_rewinddir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_rewinddir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_REWINDDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int userfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                       FAR void **handle)
{
  FAR struct userfs_state_s *priv;
  FAR const struct userfs_config_s *config;
  struct sockaddr_in client;
  unsigned int iolen;
  int ret;

  DEBUGASSERT(data != NULL && handle != NULL);
  config = (FAR const struct userfs_config_s *)data;

  /* Allocate an instance of the UserFS state structure */

  iolen = USERFS_REQ_MAXSIZE + config->mxwrite;
  priv  = kmm_malloc(SIZEOF_USERFS_STATE_S(iolen));
  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate state structure\n");
      return -ENOMEM;
    }

  /* Initialize the mutex that assures mutually exclusive access through
   * the entire request-response sequence.
   */

  nxmutex_init(&priv->lock);

  /* Copy the configuration data into the allocated structure.  Why?  First
   * we can't be certain of the life time of the memory underlying the config
   * reference.  Also, in the KERNEL build, the config data will like in
   * process-specific memory and cannot be shared across processes.
   */

  priv->mxwrite                = config->mxwrite;

  /* Preset the server address */

  priv->server.sin_family      = AF_INET;
  priv->server.sin_port        = HTONS(config->portno);
  priv->server.sin_addr.s_addr = HTONL(INADDR_LOOPBACK);

  /* Create a LocalHost UDP client socket */

  ret = psock_socket(PF_INET, SOCK_DGRAM, 0, &priv->psock);
  if (ret < 0)
    {
      ferr("ERROR: socket() failed: %d\n", ret);
      goto errout_with_alloc;
    }

  /* Bind the socket to the client address */

  client.sin_family      = AF_INET;
  client.sin_port        = 0;
  client.sin_addr.s_addr = HTONL(INADDR_LOOPBACK);

  ret = psock_bind(&priv->psock, (FAR struct sockaddr *)&client,
                   sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      ferr("ERROR: bind() failed: %d\n", ret);
      goto errout_with_psock;
    }

  /* Mounted! */

  *handle = (FAR void *)priv;
  return OK;

errout_with_psock:
  psock_close(&priv->psock);

errout_with_alloc:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: userfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int userfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags)
{
  FAR struct userfs_state_s *priv = (FAR struct userfs_state_s *)handle;
  FAR struct userfs_destroy_request_s *req;
  FAR struct userfs_destroy_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_destroy_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_DESTROY;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_destroy_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_destroy_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_destroy_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_DESTROY)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* If the destruction failed, then refuse to unmount at this time */

  if (resp->ret < 0)
    {
      return resp->ret;
    }

  /* Free resources and return success */

  psock_close(&priv->psock);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return OK;
}

/****************************************************************************
 * Name: userfs_statfs
 *
 * Description:
 *   Return filesystem statistics
 *
 ****************************************************************************/

static int userfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_statfs_request_s *req;
  FAR struct userfs_statfs_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_statfs_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_STATFS;

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       sizeof(struct userfs_statfs_request_s), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_statfs_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_statfs_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_STATFS)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Return the status of the file system */

  DEBUGASSERT(buf != NULL);
  memcpy(buf, &resp->buf, sizeof(struct statfs));
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_unlink
 *
 * Description:
 *   Remove a directory entry
 *
 ****************************************************************************/

static int userfs_unlink(FAR struct inode *mountpt,
                         FAR const char *relpath)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_unlink_request_s *req;
  FAR struct userfs_unlink_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen     = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_unlink_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_UNLINK;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_UNLINK_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_unlink_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_unlink_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_UNLINK)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_mkdir
 *
 * Description:
 *   Create a new directory
 *
 ****************************************************************************/

static int userfs_mkdir(FAR struct inode *mountpt,
                        FAR const char *relpath, mode_t mode)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_mkdir_request_s *req;
  FAR struct userfs_mkdir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req       = (FAR struct userfs_mkdir_request_s *)priv->iobuffer;
  req->req  = USERFS_REQ_MKDIR;
  req->mode = mode;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_MKDIR_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_mkdir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_mkdir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_MKDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_rmdir
 *
 * Description:
 *   Remove a directory
 *
 ****************************************************************************/

static int userfs_rmdir(FAR struct inode *mountpt,
                        FAR const char *relpath)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_rmdir_request_s *req;
  FAR struct userfs_rmdir_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_rmdir_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_RMDIR;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_RMDIR_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_rmdir_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_rmdir_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_RMDIR)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_rename
 *
 * Description:
 *   Rename a directory entry
 *
 ****************************************************************************/

static int userfs_rename(FAR struct inode *mountpt,
                         FAR const char *oldrelpath,
                         FAR const char *newrelpath)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_rename_request_s *req;
  FAR struct userfs_rename_response_s *resp;
  int oldpathlen;
  int newpathlen;
  ssize_t nsent;
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path lengths */

  DEBUGASSERT(oldrelpath != NULL && newrelpath != NULL);
  oldpathlen = strlen(oldrelpath) + 1;
  newpathlen = strlen(newrelpath) + 1;

  if ((oldpathlen + newpathlen) > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req            = (FAR struct userfs_rename_request_s *)priv->iobuffer;
  req->req       = USERFS_REQ_RENAME;
  req->newoffset = oldpathlen;

  strlcpy(req->oldrelpath, oldrelpath, oldpathlen);
  strlcpy(&req->oldrelpath[oldpathlen], newrelpath, newpathlen);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                      SIZEOF_USERFS_RENAME_REQUEST_S(oldpathlen, newpathlen),
                      0, (FAR struct sockaddr *)&priv->server,
                      sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_rename_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_rename_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_RENAME)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Name: userfs_stat
 *
 * Description:
 *   Return information about a file or directory
 *
 ****************************************************************************/

static int userfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_stat_request_s *req;
  FAR struct userfs_stat_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req      = (FAR struct userfs_stat_request_s *)priv->iobuffer;
  req->req = USERFS_REQ_STAT;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_STAT_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %d\n", (int)nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_stat_response_s))
    {
      ferr("ERROR: Response size incorrect: %u\n", (unsigned int)nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_stat_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_STAT)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  /* Return the directory entry status */

  DEBUGASSERT(buf != NULL);
  memcpy(buf, &resp->buf, sizeof(struct stat));
  return resp->ret;
}

/****************************************************************************
 * Name: userfs_chstat
 *
 * Description:
 *   Change information about a file or directory
 *
 ****************************************************************************/

static int userfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR const struct stat *buf, int flags)
{
  FAR struct userfs_state_s *priv;
  FAR struct userfs_chstat_request_s *req;
  FAR struct userfs_chstat_response_s *resp;
  ssize_t nsent;
  ssize_t nrecvd;
  int pathlen;
  int ret;

  DEBUGASSERT(mountpt != NULL &&
              mountpt->i_private != NULL);
  priv = mountpt->i_private;

  /* Check the path length */

  DEBUGASSERT(relpath != NULL);
  pathlen = strlen(relpath);
  if (pathlen > priv->mxwrite)
    {
      return -E2BIG;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Construct and send the request to the server */

  req        = (FAR struct userfs_chstat_request_s *)priv->iobuffer;
  req->req   = USERFS_REQ_CHSTAT;
  req->buf   = *buf;
  req->flags = flags;

  strlcpy(req->relpath, relpath, priv->mxwrite);

  nsent = psock_sendto(&priv->psock, priv->iobuffer,
                       SIZEOF_USERFS_CHSTAT_REQUEST_S(pathlen + 1), 0,
                       (FAR struct sockaddr *)&priv->server,
                       sizeof(struct sockaddr_in));
  if (nsent < 0)
    {
      ferr("ERROR: psock_sendto failed: %zd\n", nsent);
      nxmutex_unlock(&priv->lock);
      return (int)nsent;
    }

  /* Then get the response from the server */

  nrecvd = psock_recvfrom(&priv->psock, priv->iobuffer, IOBUFFER_SIZE(priv),
                          0, NULL, NULL);
  nxmutex_unlock(&priv->lock);

  if (nrecvd < 0)
    {
      ferr("ERROR: psock_recvfrom failed: %zd\n", nrecvd);
      return (int)nrecvd;
    }

  if (nrecvd != sizeof(struct userfs_chstat_response_s))
    {
      ferr("ERROR: Response size incorrect: %zd\n", nrecvd);
      return -EIO;
    }

  resp = (FAR struct userfs_chstat_response_s *)priv->iobuffer;
  if (resp->resp != USERFS_RESP_STAT)
    {
      ferr("ERROR: Incorrect response: %u\n", resp->resp);
      return -EIO;
    }

  return resp->ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
