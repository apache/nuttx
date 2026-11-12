/****************************************************************************
 * fs/nfs/nfs_util.c
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
#include <sys/types.h>
#include <sys/time.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <nuttx/debug.h>
#include <nuttx/lib/lib.h>

#include "rpc.h"
#include "nfs.h"
#include "nfs_proto.h"
#include "nfs_mount.h"
#include "nfs_node.h"
#include "xdr_subs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int nfs_pathsegment(FAR const char **path, FAR char *buffer,
                                  FAR char *terminator)
{
  FAR const char *src = *path;
  FAR char *dest = buffer;
  int nbytes = 0;
  char ch;

  /* Loop until the name is successfully parsed or an error occurs */

  for (; ; )
    {
      /* Get the next byte from the path */

      ch = *src++;

      /* Check if this the last byte in this segment name */

      if (ch == '\0' || ch == '/')
        {
          /* This logic just supports "//" sequences in the path name */

          if (ch == '\0' || nbytes > 0)
            {
              /* NULL terminate the parsed path segment */

              *dest        = '\0';

              /* Return next path and the terminating character */

              *terminator = ch;
              *path       = src;
              return OK;
            }

          /* Just skip over any leading '/' characters */
        }
      else if (nbytes >= NAME_MAX)
        {
          ferr("ERROR: File name segment is too long: %d\n", nbytes);
          return -EFBIG;
        }
      else
        {
          /* Save next character in the accumulated name */

          *dest++ = ch;
          nbytes++;
        }
    }
}

#ifdef CONFIG_FS_LINKS
/****************************************************************************
 * Name: nfs_readlink_internal
 *
 * Description:
 *   Read the target of a symbolic link.
 *
 * Input Parameters:
 *   nmp     - NFS mount point
 *   fhandle - File handle of the symbolic link
 *   buffer  - Buffer to store the link target
 *   bufsize - Size of the buffer
 *
 * Returned Value:
 *   On success, returns the length of the link target stored in 'buffer'.
 *   On failure, returns a negative errno value.
 *
 ****************************************************************************/

int nfs_readlink_internal(FAR struct nfsmount *nmp,
                          FAR struct file_handle *fhandle,
                          FAR char *buffer, size_t bufsize)
{
  FAR struct rpc_reply_readlink *readlink;
  FAR uint32_t *ptr;
  uint32_t length;
  int reqlen;
  int ret;

  /* Create the READLINK RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.readlink.readlink;
  reqlen = 0;

  /* Copy the variable-length file handle */

  *ptr++  = txdr_unsigned(fhandle->length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle->handle, fhandle->length);
  reqlen += uint32_alignup(fhandle->length);

  /* Perform the READLINK RPC */

  nfs_statistics(NFSPROC_READLINK);
  ret = nfs_request(nmp, NFSPROC_READLINK,
                    &nmp->nm_msgbuffer.readlink, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret != OK)
    {
      ferr("ERROR: nfs_request failed: %d\n", ret);
      return ret;
    }

  /* Parse the response */

  readlink = (FAR struct rpc_reply_readlink *)nmp->nm_iobuffer;
  ptr      = (FAR uint32_t *)&readlink->readlink;

  /* Skip attributes if present */

  if (*ptr++)
    {
      ptr += sizeof(struct nfs_fattr) / sizeof(uint32_t);
    }

  /* Get the target path length */

  length = fxdr_unsigned(uint32_t, *ptr);
  ptr++;

  /* POSIX readlink() behavior: if buffer is too small, truncate the result
   * and return the number of bytes placed in buffer (not an error).
   */

  if (length > bufsize)
    {
      length = bufsize;
    }

  /* Copy the target path */

  memcpy(buffer, ptr, length);
  return length;
}

/****************************************************************************
 * Name: nfs_followlink
 *
 * Description:
 *   Follow a symbolic link to its ultimate target. This function resolves
 *   a symbolic link by reading its target and resolving the path from
 *   the containing directory.
 *
 * Input Parameters:
 *   nmp           - NFS mount point
 *   dir_fhandle   - File handle of the directory containing the symlink
 *   fhandle       - File handle of the symbolic link (input/output)
 *   obj_attr      - Object attributes (input/output)
 *   symloop_count - Pointer to symlink resolution depth counter
 *
 * Returned Value:
 *   Zero on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int nfs_followlink(FAR struct nfsmount *nmp,
                          FAR struct file_handle *dir_fhandle,
                          FAR struct file_handle *fhandle,
                          FAR struct nfs_fattr *obj_attr,
                          FAR int *symloop_count)
{
  struct file_handle parent_fhandle;
  struct file_handle tmphandle;
  FAR const char *linkptr;
  FAR char *linkpath;
  FAR char *buffer;
  char terminator;
  uint32_t tmp;
  int ret;

  /* Limit the number of symbolic links that we pass through */

  if (*symloop_count >= SYMLOOP_MAX)
    {
      ferr("ERROR: Too many symbolic links (loop?)\n");
      return -ELOOP;
    }

  (*symloop_count)++;
  linkpath = lib_get_tempbuffer(PATH_MAX);
  if (linkpath == NULL)
    {
      ferr("ERROR: Failed to allocate buffer for symlink path\n");
      return -ENOMEM;
    }

  buffer = lib_get_tempbuffer(NAME_MAX + 1);
  if (buffer == NULL)
    {
      ferr("ERROR: Failed to allocate buffer for path segment\n");
      lib_put_tempbuffer(linkpath);
      return -ENOMEM;
    }

  /* Read the symlink target */

  ret = nfs_readlink_internal(nmp, fhandle, linkpath, PATH_MAX - 1);
  if (ret < 0)
    {
      goto errout;
    }

  linkpath[ret] = '\0';

  /* Our fs can only resolve relative path symbolic links.
   * Absolute paths are not supported.
   */

  if (linkpath[0] == '/')
    {
      ret = -ENOSYS;
      goto errout;
    }

  memcpy(&tmphandle, dir_fhandle, sizeof(tmphandle));

  /* Process each path component of the symlink target */

  linkptr = linkpath;
  for (; ; )
    {
      /* Set the parent directory which we will use for followlink
       * in this iteration.
       */

      memcpy(&parent_fhandle, &tmphandle, sizeof(parent_fhandle));

      /* Extract the next path component */

      ret = nfs_pathsegment(&linkptr, buffer, &terminator);
      if (ret != OK)
        {
          goto errout;
        }

      /* Lookup this component */

      ret = nfs_lookup(nmp, buffer, &tmphandle, obj_attr, NULL);
      if (ret != OK)
        {
          ferr("ERROR: nfs_lookup(\"%s\") failed: %d\n", buffer, ret);
          goto errout;
        }

      /* If this is a symlink, follow it recursively */

      tmp = fxdr_unsigned(uint32_t, obj_attr->fa_type);
      if (tmp == NFLNK)
        {
          ret = nfs_followlink(nmp, &parent_fhandle, &tmphandle,
                               obj_attr, symloop_count);
          if (ret != OK)
            {
              goto errout;
            }

          /* Update the type after following the symlink */

          tmp = fxdr_unsigned(uint32_t, obj_attr->fa_type);
        }

      if (terminator == '\0')
        {
          break;
        }

      /* Not the final component - verify it's a directory */

      if (tmp != NFDIR)
        {
          ferr("ERROR: Intermediate component \"%s\" is not a directory\n",
               buffer);
          ret = -ENOTDIR;
          goto errout;
        }
    }

  /* Success - copy the result back */

  memcpy(fhandle, &tmphandle, sizeof(*fhandle));

errout:
  lib_put_tempbuffer(buffer);
  lib_put_tempbuffer(linkpath);
  return ret;
}
#endif /* CONFIG_FS_LINKS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nfs_request
 *
 * Description:
 *   Perform the NFS request. On successful receipt, it verifies the NFS
 *   level of the returned values.
 *
 * Returned Value:
 *   Zero on success; a negative errno value on failure.
 *
 ****************************************************************************/

int nfs_request(FAR struct nfsmount *nmp, int procnum,
                FAR void *request, size_t reqlen,
                FAR void *response, size_t resplen)
{
  FAR struct rpcclnt *clnt = nmp->nm_rpcclnt;
  struct nfs_reply_header replyh;
  int error;

  error = rpcclnt_request(clnt, procnum, NFS_PROG, NFS_VER3,
                          request, reqlen, response, resplen);
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);

      if (error != -ENOTCONN && error != -EPIPE)
        {
          return error;
        }

      /* Reconnect */

      finfo("Reconnect due to timeout\n");

      error = rpcclnt_connect(nmp->nm_rpcclnt);

      if (error != 0)
        {
          return error;
        }

      /* Send the request again */

      error = rpcclnt_request(clnt, procnum, NFS_PROG, NFS_VER3,
                              request, reqlen, response, resplen);

      if (error != 0)
        {
          return error;
        }
    }

  memcpy(&replyh, response, sizeof(struct nfs_reply_header));

  if (replyh.nfs_status != 0)
    {
      /* NFS_ERRORS are the same as NuttX errno values */

      return -fxdr_unsigned(uint32_t, replyh.nfs_status);
    }

  if (replyh.rh.rpc_verfi.authtype != 0)
    {
      error = -EOPNOTSUPP;
      ferr("ERROR: NFS authtype %d from server\n",
           fxdr_unsigned(int, replyh.rh.rpc_verfi.authtype));
      return error;
    }

  finfo("NFS_SUCCESS\n");
  return OK;
}

/****************************************************************************
 * Name: nfs_lookup
 *
 * Description:
 *   Given a directory file handle, and the path to file in the directory,
 *   return the file handle of the path and attributes of both the file and
 *   the directory containing the file.
 *
 *   NOTE:  The LOOKUP call differs from other RPC messages in that the
 *   call message is variable length, depending upon the size of the path
 *   name.
 *
 ****************************************************************************/

int nfs_lookup(FAR struct nfsmount *nmp, FAR const char *filename,
               FAR struct file_handle *fhandle,
               FAR struct nfs_fattr *obj_attributes,
               FAR struct nfs_fattr *dir_attributes)
{
  FAR uint32_t *ptr;
  uint32_t value;
  int reqlen;
  int namelen;
  int error = 0;

  DEBUGASSERT(nmp && filename && fhandle);

  /* Get the length of the string to be sent */

  namelen = strlen(filename);
  if (namelen > NAME_MAX)
    {
      ferr("ERROR: Length of the string is too long: %d\n", namelen);
      return -E2BIG;
    }

  /* Initialize the request */

  ptr     = (FAR uint32_t *)&nmp->nm_msgbuffer.lookup.lookup;
  reqlen  = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle->length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle->handle, fhandle->length);
  reqlen += uint32_alignup(fhandle->length);
  ptr    += uint32_increment(fhandle->length);

  /* Copy the variable-length file name */

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  reqlen += uint32_alignup(namelen);

  /* Request LOOKUP from the server */

  nfs_statistics(NFSPROC_LOOKUP);
  error = nfs_request(nmp, NFSPROC_LOOKUP,
                      (FAR void *)&nmp->nm_msgbuffer.lookup, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);

  if (error)
    {
      ferr("ERROR: nfs_request failed: %d\n", error);
      return error;
    }

  /* Return the data to the caller's buffers.  NOTE:  Here we ignore the
   * the exact layout of the rpc_reply_lookup structure.  File handles
   * may differ in size whereas struct rpc_reply_lookup uses a fixed size.
   */

  ptr = (FAR uint32_t *)
    &((FAR struct rpc_reply_lookup *)nmp->nm_iobuffer)->lookup;

  /* Get the length of the file handle */

  value = *ptr++;
  value = fxdr_unsigned(uint32_t, value);
  if (value > NFSX_V3FHMAX)
    {
      ferr("ERROR: Bad file handle length: %" PRId32 "\n", value);
      return -EIO;
    }

  /* Return the file handle */

  fhandle->length = value;
  memcpy(&fhandle->handle, ptr, value);
  ptr += uint32_increment(value);

  /* Check if there are object attributes and, if so, copy them to the user
   * buffer
   */

  value = *ptr++;
  if (value)
    {
      if (obj_attributes)
        {
          memcpy(obj_attributes, ptr, sizeof(struct nfs_fattr));
        }

      ptr += uint32_increment(sizeof(struct nfs_fattr));
    }

  /* Check if there are directory attributes and, if so, copy them to the
   * user buffer
   */

  value = *ptr++;
  if (value && dir_attributes)
    {
      memcpy(dir_attributes, ptr, sizeof(struct nfs_fattr));
    }

  return OK;
}

/****************************************************************************
 * Name: nfs_findnode
 *
 * Description:
 *   Given a path to something that may or may not be in the file system,
 *   return the handle of the directory entry of the requested object.
 *
 * Returned Value:
 *   Zero on success; a negative errno value on failure.
 *
 ****************************************************************************/

int nfs_findnode(FAR struct nfsmount *nmp, FAR const char *relpath,
                 FAR struct file_handle *fhandle,
                 FAR struct nfs_fattr *obj_attributes,
                 FAR struct nfs_fattr *dir_attributes, bool nofollow)
{
  FAR const char    *path = relpath;
  FAR char          *buffer;
  char               terminator;
  uint32_t           tmp;
  int                error;
#ifdef CONFIG_FS_LINKS
  struct file_handle parent_fhandle;
  int                symloop_count = 0;
#endif

  buffer = lib_get_tempbuffer(NAME_MAX + 1);
  if (buffer == NULL)
    {
      ferr("ERROR: Failed to allocate buffer for path segment\n");
      return -ENOMEM;
    }

  /* Start with the file handle of the root directory.  */

  fhandle->length = nmp->nm_fhsize;
  memcpy(&fhandle->handle, nmp->nm_fh, nmp->nm_fhsize);

  /* If no path was provided, then the root directory must be exactly what
   * the caller is looking for.
   */

  if (*path == '\0')
    {
      /* Return the root directory attributes */

      if (obj_attributes)
        {
          memcpy(obj_attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));
        }

      if (dir_attributes)
        {
          memcpy(dir_attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));
        }

      lib_put_tempbuffer(buffer);
      return OK;
    }

  /* This is not the root directory. Loop until the directory entry
   * corresponding to the path is found.
   */

  for (; ; )
    {
#ifdef CONFIG_FS_LINKS
      /* Save the current directory handle before looking up
       * the next segment
       */

      memcpy(&parent_fhandle, fhandle, sizeof(parent_fhandle));
#endif

      /* Extract the next path segment name */

      error = nfs_pathsegment(&path, buffer, &terminator);
      if (error != OK)
        {
          /* The filename segment contains is too long. */

          ferr("ERROR: nfs_pathsegment of \"%s\" failed after \"%s\": %d\n",
               relpath, buffer, error);
          lib_put_tempbuffer(buffer);
          return error;
        }

      /* If we got an empty segment (e.g., from trailing slash like "a/b/"),
       * we're done. The previous lookup already resolved the directory.
       */

      if (buffer[0] == '\0')
        {
          lib_put_tempbuffer(buffer);
          return OK;
        }

      /* Look-up this path segment */

      error = nfs_lookup(nmp, buffer, fhandle, obj_attributes,
                         dir_attributes);
      if (error != OK)
        {
          ferr("ERROR: nfs_lookup of \"%s\" failed at \"%s\": %d\n",
               relpath, buffer, error);
          lib_put_tempbuffer(buffer);
          return error;
        }

#ifdef CONFIG_FS_LINKS
      /* Handle symbolic links based on position and nofollow flag.
       * For intermediate path components, always follow.
       * For final component, follow only if nofollow is false.
       */

      tmp = fxdr_unsigned(uint32_t, obj_attributes->fa_type);
      if (tmp == NFLNK && (terminator || !nofollow))
        {
          error = nfs_followlink(nmp, &parent_fhandle, fhandle,
                                 obj_attributes, &symloop_count);
          if (error != OK)
            {
              lib_put_tempbuffer(buffer);
              return error;
            }

          /* Update type after following symlink */

          tmp = fxdr_unsigned(uint32_t, obj_attributes->fa_type);
        }
#endif /* CONFIG_FS_LINKS */

      if (!terminator)
        {
          /* Return success meaning that the description the matching
           * directory entry is in fhandle, obj_attributes, and
           * dir_attributes.
           */

          lib_put_tempbuffer(buffer);
          return OK;
        }

      /* No.. then we have found one of the intermediate directories on
       * the way to the final path target.  In this case, make sure
       * the thing that we found is, indeed, a directory.
       */

#ifndef CONFIG_FS_LINKS
      tmp = fxdr_unsigned(uint32_t, obj_attributes->fa_type);
#endif
      if (tmp != NFDIR)
        {
          /* Ooops.. we found something else */

          ferr("ERROR: Intermediate segment \"%s\" of \'%s\" is not a "
                "directory\n",
                buffer, relpath);
          lib_put_tempbuffer(buffer);
          return -ENOTDIR;
        }
    }
}

/****************************************************************************
 * Name: nfs_finddir
 *
 * Description:
 *   Given a path to something that may or may not be in the file system,
 *   return the handle of the entry of the directory containing the requested
 *   object.
 *
 * Returned Value:
 *   Zero on success; a negative errno value on failure.
 *
 ****************************************************************************/

int nfs_finddir(FAR struct nfsmount *nmp, FAR const char *relpath,
                FAR struct file_handle *fhandle,
                FAR struct nfs_fattr *attributes, FAR char *filename)
{
  FAR const char    *path = relpath;
  uint32_t           tmp;
  char               terminator;
  int                error;
#ifdef CONFIG_FS_LINKS
  struct file_handle parent_fhandle;
  int                symloop_count = 0;
#endif

  /* Verify that a path was provided */

  if (*path == '\0')
    {
      return -ENOENT;
    }

  /* Start with the file handle of the root directory.  */

  fhandle->length = nmp->nm_fhsize;
  memcpy(&fhandle->handle, nmp->nm_fh, nmp->nm_fhsize);
  memcpy(attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));

  /* Loop until the directory entry containing the path is found. */

  for (; ; )
    {
#ifdef CONFIG_FS_LINKS
      /* Save the current directory handle before looking up the next
       * segment
       */

      memcpy(&parent_fhandle, fhandle, sizeof(parent_fhandle));
#endif

      /* Extract the next path segment name. */

      error = nfs_pathsegment(&path, filename, &terminator);
      if (error != OK)
        {
          /* The filename segment contains is too long. */

          ferr("ERROR: nfs_pathsegment of \"%s\" failed after \"%s\": %d\n",
               relpath, filename, error);
          return error;
        }

      /* If the terminator character in the path was the end of the string
       * then we have successfully found the directory that contains the name
       * of interest.
       */

      if (!terminator)
        {
          /* Return success meaning that the description of the directory
           * containing the object is in fhandle and attributes.
           */

          return OK;
        }

      /* Look-up the next path segment */

      error = nfs_lookup(nmp, filename, fhandle, attributes, NULL);
      if (error != OK)
        {
          ferr("ERROR: fs_lookup of \"%s\" failed at \"%s\": %d\n",
               relpath, filename, error);
          return error;
        }

      /* Make sure the thing that we found is, indeed, a directory. */

      tmp = fxdr_unsigned(uint32_t, attributes->fa_type);

#ifdef CONFIG_FS_LINKS
      if (tmp == NFLNK)
        {
          /* Intermediate segment is a symlink - must follow it */

          error = nfs_followlink(nmp, &parent_fhandle, fhandle, attributes,
                                 &symloop_count);
          if (error != OK)
            {
              return error;
            }

          /* Update type after following symlink */

          tmp = fxdr_unsigned(uint32_t, attributes->fa_type);
        }
#endif /* CONFIG_FS_LINKS */

      if (tmp != NFDIR)
        {
          /* Ooops.. we found something else */

          ferr("ERROR: Intermediate segment \"%s\" of \'%s\" is not a "
               "directory\n",
               filename, relpath);
          return -ENOTDIR;
        }
    }
}

/****************************************************************************
 * Name: nfs_attrupdate
 *
 * Description:
 *   Update file attributes on write or after the file is modified.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nfs_attrupdate(FAR struct nfsnode *np, FAR struct nfs_fattr *attributes)
{
  /* Save a few of the files attribute values in file structure (host
   * order).
   */

  np->n_type   = fxdr_unsigned(uint8_t, attributes->fa_type);
  np->n_mode   = fxdr_unsigned(uint16_t, attributes->fa_mode);
  np->n_size   = fxdr_hyper(&attributes->fa_size);

  fxdr_nfsv3time(&attributes->fa_atime, &np->n_atime);
  fxdr_nfsv3time(&attributes->fa_mtime, &np->n_mtime);
  fxdr_nfsv3time(&attributes->fa_ctime, &np->n_ctime);
}
