/****************************************************************************
 * net/route/net_fileroute.c
 *
 *   Copyright (C) 2017, 2020 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
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

#define NO_HOLDER ((pid_t)-1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
/* Semaphore used to lock a routing table for exclusive write-only access */

static sem_t g_ipv4_exclsem;
static pid_t g_ipv4_holder = NO_HOLDER;
static int g_ipv4_count;
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
/* Semaphore used to lock a routing table for exclusive write-only access */

static sem_t g_ipv6_exclsem;
static pid_t g_ipv6_holder = NO_HOLDER;
static int g_ipv6_count;
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

  ret = stat(path, &buf);
  if (ret < 0)
    {
      int errcode;

      /* stat() failed, but is that because the routing table has not been
       * created yet?
       */

      errcode = get_errno();
      if (errcode == ENOENT)
        {
          /* The routing table file has not been created.  Return size zero. */

          return 0;
        }

      /* Some other error */

      return -errcode;
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
 * Name: net_init_fileroute
 *
 * Description:
 *   Initialize the in-memory, RAM routing table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_init_fileroute(void)
{
  /* Initialize semaphores */

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
  nxsem_init(&g_ipv4_exclsem, 0, 1);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
  nxsem_init(&g_ipv6_exclsem, 0, 1);
#endif
}

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
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
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
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
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
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
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
  pid_t me = getpid();
  int ret;

  /* Are we already the holder of the lock? */

  if (g_ipv4_holder == me)
    {
      /* Yes.. just increment the count of locks held */

      g_ipv4_count++;
      ret = OK;
    }
  else
    {
      /* No.. wait to get the lock */

      ret = nxsem_wait(&g_ipv4_exclsem);
      if (ret < 0)
        {
          nerr("ERROR: nxsem_wait() failed: %d\n", ret);
        }
      else
        {
          DEBUGASSERT(g_ipv4_holder == NO_HOLDER && g_ipv4_count == 0);

          /* We are now the holder with one count */

          g_ipv4_holder = me;
          g_ipv4_count  = 1;
        }
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_lockroute_ipv6(void)
{
  pid_t me = getpid();
  int ret;

  /* Are we already the holder of the lock? */

  if (g_ipv6_holder == me)
    {
      /* Yes.. just increment the count of locks held */

      g_ipv6_count++;
      ret = OK;
    }
  else
    {
      /* No.. wait to get the lock */

      ret = nxsem_wait(&g_ipv6_exclsem);
      if (ret < 0)
        {
          nerr("ERROR: nxsem_wait() failed: %d\n", ret);
        }
      else
        {
          DEBUGASSERT(g_ipv6_holder == NO_HOLDER && g_ipv6_count == 0);

          /* We are now the holder with one count */

          g_ipv6_holder = me;
          g_ipv6_count  = 1;
        }
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
  pid_t me = getpid();
  int ret;

  /* If would be an error if we are called with on a thread that does not
   * hold the lock.
   */

  DEBUGASSERT(me == g_ipv4_holder && g_ipv4_count > 0);

  /* Release the count on the lock.  If this is the last count, then release
   * the lock.
   */

  if (g_ipv4_count > 1)
    {
      /* Not the last count... just decrement the count and return success */

      g_ipv4_count--;
      ret = OK;
    }
  else
    {
      /* This is the last count.  Release the lock */

      g_ipv4_holder = NO_HOLDER;
      g_ipv4_count  = 0;

      ret = nxsem_post(&g_ipv4_exclsem);
      if (ret < 0)
        {
          nerr("ERROR: nxsem_post() failed: %d\n", ret);
        }
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_unlockroute_ipv6(void)
{
  pid_t me = getpid();
  int ret;

  /* If would be an error if we are called with on a thread that does not
   * hold the lock.
   */

  DEBUGASSERT(me == g_ipv6_holder && g_ipv6_count > 0);

  /* Release the count on the lock.  If this is the last count, then release
   * the lock.
   */

  if (g_ipv6_count > 1)
    {
      /* Not the last count... just decrement the count and return success */

      g_ipv6_count--;
      ret = OK;
    }
  else
    {
      /* This is the last count.  Release the lock */

      g_ipv6_holder = NO_HOLDER;
      g_ipv6_count  = 0;

      ret = nxsem_post(&g_ipv6_exclsem);
      if (ret < 0)
        {
          nerr("ERROR: nxsem_post() failed: %d\n", ret);
        }
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
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
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
