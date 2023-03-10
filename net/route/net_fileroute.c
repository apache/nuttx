/****************************************************************************
 * net/route/net_fileroute.c
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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>

#include "route/fileroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_FILEROUTE) || defined(CONFIG_ROUTE_IPv6_FILEROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Special "impossible" PID value used to indicate that there is no holder
 * of the lock.
 */

#define NO_HOLDER (INVALID_PROCESS_ID)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
/* Used to lock a routing table for exclusive write-only access */

static rmutex_t g_ipv4_lock = NXRMUTEX_INITIALIZER;
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
/* Used to lock a routing table for exclusive write-only access */

static rmutex_t g_ipv6_lock = NXRMUTEX_INITIALIZER;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_openroute_detached
 *
 * Description:
 *   Open and detach the routing table.
 *
 * Input Parameters:
 *   pathname - The full path to the routing table entry
 *   oflags   - Open flags
 *   filep    - Location in which to return the detached file instance.
 *
 * Returned Value:
 *   A non-negative file descriptor is returned on success.  A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int net_openroute_detached(FAR const char *pathname, int oflags,
                           FAR struct file *filep)
{
  int ret;

  /* Open the file for read/write access. */

  ret = file_open(filep, pathname, oflags, 0644);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open %s: %d\n", pathname, ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: net_routesize
 *
 * Description:
 *   Return the size of a routing table in terms of the number of entries in
 *   the routing table.
 *
 * Input Parameters:
 *   path      - The path to the routing table
 *   entrysize - The size of one entry in the routing table
 *
 * Returned Value:
 *   A non-negative count of the number of entries in the routing table is
 *   returned on success; a negated errno value is returned on and failure.
 *
 ****************************************************************************/

int net_routesize(FAR const char *path, size_t entrysize)
{
  struct stat buf;
  int ret;

  /* Get information about the file */

  ret = nx_stat(path, &buf, 1);
  if (ret < 0)
    {
      /* nx_stat() failed, but is that because the routing table has not been
       * created yet?
       */

      if (ret == -ENOENT)
        {
          /* The routing table file has not been created.  Return zero. */

          return 0;
        }

      /* Some other error */

      return ret;
    }

  /* The directory entry at this path must be a regular file */

  if (S_ISREG(buf.st_mode))
    {
      unsigned int nentries = buf.st_size / entrysize;

#ifdef CONFIG_DEBUG_NET_WARN
      if (nentries * entrysize != buf.st_size)
        {
          nwarn("WARNING: Size of routing table is not an even multiple of "
                "entries\n");
          nwarn("         %lu != %lu / %lu\n",
                (unsigned long)nentries,
                (unsigned long)buf.st_size,
                (unsigned long)entrysize);
        }
#endif

      return nentries;
    }

  /* It is probably a directory (should check for sure) */

  return -EISDIR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_openroute_ipv4/net_openroute_ipv6
 *
 * Description:
 *   Open the IPv4/IPv6 routing table with the specified access privileges.
 *
 * Input Parameters:
 *   oflags - Open flags
 *   filep  - Location in which to return the detached file instance.
 *
 * Returned Value:
 *   A non-negative file descriptor is returned on success.  A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_openroute_ipv4(int oflags, FAR struct file *filep)
{
  int ret;

  /* Lock the route.. we don't want to open it while it is subject to
   * modification.
   */

  ret = net_lockroute_ipv4();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv4() failed: %d\n", ret);
    }
  else
    {
      /* Open the file for read/write access. */

      ret = net_openroute_detached(IPv4_ROUTE_PATH, oflags, filep);
      if (ret < 0)
        {
          nerr("ERROR: net_openroute_detached() failed: %d\n", ret);
        }
    }

  net_unlockroute_ipv4();
  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_openroute_ipv6(int oflags, FAR struct file *filep)
{
  int ret;

  /* Lock the route.. we don't want to open it while it is subject to
   * modification.
   */

  ret = net_lockroute_ipv6();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv6() failed: %d\n", ret);
    }
  else
    {
      /* Open the file for read/write access. */

      ret = net_openroute_detached(IPv6_ROUTE_PATH, oflags, filep);
      if (ret < 0)
        {
          nerr("ERROR: net_openroute_detached() failed: %d\n", ret);
        }
    }

  net_unlockroute_ipv6();
  return ret;
}
#endif

/****************************************************************************
 * Name: net_readroute_ipv4/net_readroute_ipv6
 *
 * Description:
 *   Read one route entry from the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - File instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   route - Location to return the next route read from the file
 *
 * Returned Value:
 *   The number of bytes read on success.  The special return valud of zero
 *   indicates that the endof of file was encountered (and nothing was read).
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
ssize_t net_readroute_ipv4(FAR struct file *filep,
                           FAR struct net_route_ipv4_s *route)
{
  FAR uint8_t *dest;
  ssize_t ntotal;
  ssize_t ret;

  /* Lock the route.. we don't want to read from it while it is subject to
   * modification.
   */

  ret = (ssize_t)net_lockroute_ipv4();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv4() failed: %d\n", ret);
      return ret;
    }

  /* Read one record from the current position in the routing table */

  dest   = (FAR uint8_t *)route;
  ntotal = 0;

  do
    {
      size_t nbytes = sizeof(struct net_route_ipv4_s) - ntotal;
      ssize_t nread;

      nread = file_read(filep, dest, nbytes);
      if (nread <= 0)
        {
          /* Read error or end of file */

          ntotal = nread;
          break;
        }

      ntotal += nread;
      dest   += nread;
    }
  while (ntotal < sizeof(struct net_route_ipv4_s));

  net_unlockroute_ipv4();
  return ntotal;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
ssize_t net_readroute_ipv6(FAR struct file *filep,
                           FAR struct net_route_ipv6_s *route)
{
  FAR uint8_t *dest;
  ssize_t ntotal;
  ssize_t ret;

  /* Lock the route.. we don't want to read from it while it is subject to
   * modification.
   */

  ret = (ssize_t)net_lockroute_ipv6();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv6() failed: %d\n", ret);
      return ret;
    }

  /* Read one record from the current position in the routing table */

  dest   = (FAR uint8_t *)route;
  ntotal = 0;

  do
    {
      size_t nbytes = sizeof(struct net_route_ipv6_s) - ntotal;
      ssize_t nread;

      nread = file_read(filep, dest, nbytes);
      if (nread <= 0)
        {
          /* Read error or end of file */

          ret = nread;
          break;
        }

      ntotal += nread;
      dest   += nread;
    }
  while (ntotal < sizeof(struct net_route_ipv6_s));

  net_unlockroute_ipv6();
  return ntotal;
}
#endif

/****************************************************************************
 * Name: net_writeroute_ipv4/net_writeroute_ipv6
 *
 * Description:
 *   Write one route entry to the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - File instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   route - Location to return the next route read from the file
 *
 * Returned Value:
 *   The number of bytes written on success.  A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
ssize_t net_writeroute_ipv4(FAR struct file *filep,
                            FAR const struct net_route_ipv4_s *route)
{
  FAR const uint8_t *src;
  ssize_t ntotal;
  ssize_t ret;

  /* Lock the route.. we don't want to write to it while it is subject to
   * modification.
   */

  ret = (ssize_t)net_lockroute_ipv4();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv4() failed: %d\n", ret);
      return ret;
    }

  /* Write one record at the current position in the routing table */

  src    = (FAR const uint8_t *)route;
  ntotal = 0;

  do
    {
      size_t nbytes = sizeof(struct net_route_ipv4_s) - ntotal;
      ssize_t nwritten;

      nwritten = file_write(filep, src, nbytes);
      if (nwritten <= 0)
        {
          /* Write error (zero is not a valid return value for write) */

          ret = nwritten;
          break;
        }

      ntotal += nwritten;
      src    += nwritten;
    }
  while (ntotal < sizeof(struct net_route_ipv4_s));

  net_unlockroute_ipv4();
  return ntotal;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
ssize_t net_writeroute_ipv6(FAR struct file *filep,
                            FAR const struct net_route_ipv6_s *route)
{
  FAR const uint8_t *src;
  ssize_t ntotal;
  ssize_t ret;

  /* Lock the route.. we don't want to write to it while it is subject to
   * modification.
   */

  ret = (ssize_t)net_lockroute_ipv6();
  if (ret < 0)
    {
      nerr("ERROR: net_lockroute_ipv6() failed: %d\n", ret);
      return ret;
    }

  /* Write one record at the current position in the routing table */

  src    = (FAR const uint8_t *)route;
  ntotal = 0;

  do
    {
      size_t nbytes = sizeof(struct net_route_ipv6_s) - ntotal;
      ssize_t nwritten;

      nwritten = file_write(filep, src, nbytes);
      if (nwritten <= 0)
        {
          /* Write error (zero is not a valid return value for write) */

          ntotal = nwritten;
          break;
        }

      ntotal += nwritten;
      src    += nwritten;
    }
  while (ntotal < sizeof(struct net_route_ipv6_s));

  net_unlockroute_ipv6();
  return ret;
}
#endif

/****************************************************************************
 * Name: net_seekroute_ipv4/net_seekroute_ipv6
 *
 * Description:
 *   Seek to a specific entry entry to the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - File instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   index - The index of the routing table entry to seek to.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
off_t net_seekroute_ipv4(FAR struct file *filep, unsigned int index)
{
  off_t offset;
  off_t ret;

  /* Convert the index to a file offset */

  offset = (off_t)index * sizeof(struct net_route_ipv4_s);

  /* Then seek to that position */

  ret = file_seek(filep, offset, SEEK_SET);
  if (ret < 0)
    {
      nerr("ERROR: file_seek() failed: %ld\n", (long)ret);
      return (int)ret;
    }

  return OK;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
off_t net_seekroute_ipv6(FAR struct file *filep, unsigned int index)
{
  off_t offset;
  off_t ret;

  /* Convert the index to a file offset */

  offset = (off_t)index * sizeof(struct net_route_ipv6_s);

  /* Then seek to that position */

  ret = file_seek(filep, offset, SEEK_SET);
  if (ret < 0)
    {
      nerr("ERROR: file_seek() failed: %ld\n", (long)ret);
      return (int)ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: net_routesize_ipv4/net_routesize_ipv6
 *
 * Description:
 *   Return the size of a routing table in terms of the number of entries in
 *   the routing table.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-negative count of the number of entries in the routing table is
 *   returned on success; a negated errno value is returned on and failure.
 *
 * Assumptions:
 *   The size of the routing table may change after this size is returned
 *   unless the routing table is locked to prevent any modification to the
 *   routing table.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_routesize_ipv4(void)
{
  return net_routesize(IPv4_ROUTE_PATH, sizeof(struct net_route_ipv4_s));
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_routesize_ipv6(void)
{
  return net_routesize(IPv6_ROUTE_PATH, sizeof(struct net_route_ipv6_s));
}
#endif

/****************************************************************************
 * Name: net_lockroute_ipv4/net_lockroute_ipv6
 *
 * Description:
 *   Lock access to the routing table.  Necessary when a routing table is
 *   being reorganized due to deletion of a route.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_lockroute_ipv4(void)
{
  int ret = nxrmutex_lock(&g_ipv4_lock);
  if (ret < 0)
    {
      nerr("ERROR: nxrmutex_lock() failed: %d\n", ret);
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_lockroute_ipv6(void)
{
  int ret = nxrmutex_lock(&g_ipv6_lock);
  if (ret < 0)
    {
      nerr("ERROR: nxrmutex_lock() failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: net_unlockroute_ipv4/net_unlockroute_ipv6
 *
 * Description:
 *   Release the read lock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_unlockroute_ipv4(void)
{
  int ret = nxrmutex_unlock(&g_ipv4_lock);
  if (ret < 0)
    {
      nerr("ERROR: nxrmutex_unlock() failed: %d\n", ret);
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_unlockroute_ipv6(void)
{
  int ret = nxrmutex_unlock(&g_ipv6_lock);
  if (ret < 0)
    {
      nerr("ERROR: nxrmutex_unlock() failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: net_closeroute_ipv4/net_closeroute_ipv6
 *
 * Description:
 *   Close the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - File instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_closeroute_ipv4(FAR struct file *filep)
{
  return file_close(filep);
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_closeroute_ipv6(FAR struct file *filep)
{
  return file_close(filep);
}
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE */
