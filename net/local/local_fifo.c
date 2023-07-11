/****************************************************************************
 * net/local/local_fifo.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <sys/stat.h>
#include <sys/ioctl.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "local/local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOCAL_CS_SUFFIX    "CS"  /* Name of the client-to-server FIFO */
#define LOCAL_SC_SUFFIX    "SC"  /* Name of the server-to-client FIFO */
#define LOCAL_HD_SUFFIX    "HD"  /* Name of the half duplex datagram FIFO */
#define LOCAL_SUFFIX_LEN   2

#define LOCAL_FULLPATH_LEN (strlen(CONFIG_NET_LOCAL_VFS_PATH) + \
                            UNIX_PATH_MAX + LOCAL_SUFFIX_LEN + 2)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_format_name
 *
 * Description:
 *   Format the name of the half duplex FIFO.
 *
 ****************************************************************************/

static void local_format_name(FAR const char *inpath, FAR char *outpath,
                              FAR const char *suffix, int32_t id)
{
  if (strncmp(inpath, CONFIG_NET_LOCAL_VFS_PATH,
              strlen(CONFIG_NET_LOCAL_VFS_PATH)) == 0)
    {
      inpath += strlen(CONFIG_NET_LOCAL_VFS_PATH);
    }

  if (id < 0)
    {
      snprintf(outpath, LOCAL_FULLPATH_LEN - 1,
               CONFIG_NET_LOCAL_VFS_PATH "/%s%s", inpath, suffix);
    }
  else
    {
      snprintf(outpath, LOCAL_FULLPATH_LEN - 1,
               CONFIG_NET_LOCAL_VFS_PATH "/%s%s%" PRIx32,
               inpath, suffix, id);
    }

  outpath[LOCAL_FULLPATH_LEN - 1] = '\0';
}

/****************************************************************************
 * Name: local_cs_name
 *
 * Description:
 *   Create the name of the client-to-server FIFO.
 *
 ****************************************************************************/

static void local_cs_name(FAR struct local_conn_s *conn, FAR char *path)
{
  local_format_name(conn->lc_path, path,
                    LOCAL_CS_SUFFIX, conn->lc_instance_id);
}

/****************************************************************************
 * Name: local_sc_name
 *
 * Description:
 *   Create the name of the server-to-client FIFO.
 *
 ****************************************************************************/

static void local_sc_name(FAR struct local_conn_s *conn, FAR char *path)
{
  local_format_name(conn->lc_path, path,
                    LOCAL_SC_SUFFIX, conn->lc_instance_id);
}

/****************************************************************************
 * Name: local_hd_name
 *
 * Description:
 *   Create the name of the half duplex, datagram FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
static void local_hd_name(FAR const char *inpath, FAR char *outpath)
{
  local_format_name(inpath, outpath, LOCAL_HD_SUFFIX, -1);
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_fifo_exists
 *
 * Description:
 *   Check if a FIFO exists.
 *
 ****************************************************************************/

static bool local_fifo_exists(FAR const char *path)
{
  struct stat buf;
  int ret;

  /* Create the client-to-server FIFO */

  ret = nx_stat(path, &buf, 1);
  if (ret < 0)
    {
      return false;
    }

  /* FIFOs are character devices in NuttX.  Return true if what we found
   * is a FIFO.  What if it is something else?  In that case, we will
   * return false and mkfifo() will fail.
   */

  return (bool)S_ISCHR(buf.st_mode);
}

/****************************************************************************
 * Name: local_create_fifo
 *
 * Description:
 *   Create the one FIFO.
 *
 ****************************************************************************/

static int local_create_fifo(FAR const char *path)
{
  int ret;

  /* Create the client-to-server FIFO if it does not already exist. */

  if (!local_fifo_exists(path))
    {
      ret = nx_mkfifo(path, 0644, CONFIG_DEV_FIFO_SIZE);
      if (ret < 0)
        {
          nerr("ERROR: Failed to create FIFO %s: %d\n", path, ret);
          return ret;
        }
    }

  /* The FIFO (or some character driver) exists at PATH or we successfully
   * created the FIFO at that location.
   */

  return OK;
}

/****************************************************************************
 * Name: local_release_fifo
 *
 * Description:
 *   Release a reference from one of the FIFOs used in a connection.
 *
 ****************************************************************************/

static int local_release_fifo(FAR const char *path)
{
  int ret;

  /* Unlink the client-to-server FIFO if it exists. */

  if (local_fifo_exists(path))
    {
      /* Un-linking the FIFO removes the FIFO from the namespace.  It will
       * also mark the FIFO device "unlinked".  When all of the open
       * references to the FIFO device are closed, the resources consumed
       * by the device instance will also be freed.
       */

      ret = nx_unlink(path);
      if (ret < 0)
        {
          nerr("ERROR: Failed to unlink FIFO %s: %d\n", path, ret);
          return ret;
        }
    }

  /* The FIFO does not exist or we successfully unlinked it. */

  return OK;
}

/****************************************************************************
 * Name: local_rx_open
 *
 * Description:
 *   Open a FIFO for read-only access.
 *
 ****************************************************************************/

static int local_rx_open(FAR struct local_conn_s *conn, FAR const char *path,
                         bool nonblock)
{
  int oflags = nonblock ? O_RDONLY | O_NONBLOCK : O_RDONLY;
  int ret;

  ret = file_open(&conn->lc_infile, path, oflags);
  if (ret < 0)
    {
      nerr("ERROR: Failed on open %s for reading: %d\n",
           path, ret);

      /* Map the error code to something consistent with the return
       * error codes from connect():
       *
       * If error is ENOENT, meaning that the FIFO does exist,
       * return EFAULT meaning that the socket structure address is
       * outside the user's address space.
       */

      return ret == -ENOENT ? -EFAULT : ret;
    }

  return OK;
}

/****************************************************************************
 * Name: local_tx_open
 *
 * Description:
 *   Open a FIFO for write-only access.
 *
 ****************************************************************************/

static int local_tx_open(FAR struct local_conn_s *conn, FAR const char *path,
                         bool nonblock)
{
  int ret;

  ret = file_open(&conn->lc_outfile, path, O_WRONLY | O_NONBLOCK);
  if (ret < 0)
    {
      nerr("ERROR: Failed on open %s for writing: %d\n",
           path, ret);

      /* Map the error code to something consistent with the return
       * error codes from connect():
       *
       * If error is ENOENT, meaning that the FIFO does exist,
       * return EFAULT meaning that the socket structure address is
       * outside the user's address space.
       */

      return ret == -ENOENT ? -EFAULT : ret;
    }

  /* Clear O_NONBLOCK if it's meant to be blocking */

  if (nonblock == false)
    {
      ret = file_fcntl(&conn->lc_outfile, F_SETFL, O_WRONLY);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: local_set_policy
 *
 * Description:
 *   Set the FIFO buffer policy:
 *
 *     0=Free FIFO resources when the last reference is closed
 *     1=Free FIFO resources when the buffer is empty.
 *
 ****************************************************************************/

static int local_set_policy(FAR struct file *filep, unsigned long policy)
{
  int ret;

  /* Set the buffer policy */

  ret = file_ioctl(filep, PIPEIOC_POLICY, policy);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set FIFO buffer policy: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: local_set_pollinthreshold
 *
 * Description:
 *   Set the local pollin threshold:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
static int local_set_pollinthreshold(FAR struct file *filep,
                                     unsigned long threshold)
{
  int ret;

  /* Set the buffer poll threshold */

  ret = file_ioctl(filep, PIPEIOC_POLLINTHRD, threshold);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set FIFO pollin threshold: %lu\n",
           threshold);
    }

  return ret;
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_set_polloutthreshold
 *
 * Description:
 *   Set the local pollout threshold:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
static int local_set_polloutthreshold(FAR struct file *filep,
                                      unsigned long threshold)
{
  int ret;

  /* Set the buffer poll threshold */

  ret = file_ioctl(filep, PIPEIOC_POLLOUTTHRD, threshold);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set FIFO pollout threshold: %lu\n",
           threshold);
    }

  return ret;
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_set_pollthreshold
 *
 * Description:
 *   Set the local pollin and pollout threshold:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_set_pollthreshold(FAR struct local_conn_s *conn,
                            unsigned long threshold)
{
  int ret;

  /* Set the buffer poll threshold */

  ret = local_set_pollinthreshold(&conn->lc_infile, threshold);
  if (ret > 0)
    {
       ret = local_set_polloutthreshold(&conn->lc_outfile, threshold);
    }

  return ret;
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_create_fifos
 *
 * Description:
 *   Create the FIFO pair needed for a SOCK_STREAM connection.
 *
 ****************************************************************************/

int local_create_fifos(FAR struct local_conn_s *conn)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Create the client-to-server FIFO if it does not already exist. */

  local_cs_name(conn, path);
  ret = local_create_fifo(path);
  if (ret >= 0)
    {
      /* Create the server-to-client FIFO if it does not already exist. */

      local_sc_name(conn, path);
      ret = local_create_fifo(path);
    }

  return ret;
}

/****************************************************************************
 * Name: local_create_halfduplex
 *
 * Description:
 *   Create the half-duplex FIFO needed for SOCK_DGRAM communication.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_create_halfduplex(FAR struct local_conn_s *conn,
                            FAR const char *path)
{
  char fullpath[LOCAL_FULLPATH_LEN];

  /* Create the half duplex FIFO if it does not already exist. */

  local_hd_name(path, fullpath);
  return local_create_fifo(fullpath);
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_release_fifos
 *
 * Description:
 *   Release references to the FIFO pair used for a SOCK_STREAM connection.
 *
 ****************************************************************************/

int local_release_fifos(FAR struct local_conn_s *conn)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret1;
  int ret2;

  /* Destroy the client-to-server FIFO if it exists. */

  local_sc_name(conn, path);
  ret1 = local_release_fifo(path);

  /* Destroy the server-to-client FIFO if it exists. */

  local_cs_name(conn, path);
  ret2 = local_release_fifo(path);

  /* Return a failure if one occurred. */

  return ret1 < 0 ? ret1 : ret2;
}

/****************************************************************************
 * Name: local_release_halfduplex
 *
 * Description:
 *   Release a reference to the FIFO used for SOCK_DGRAM communication
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_release_halfduplex(FAR struct local_conn_s *conn)
{
#if 1
  /* REVISIT: We need to think about this carefully.  Unlike the connection-
   * oriented Unix domain socket, we don't really know the best time to
   * release the FIFO resource.  It would be extremely inefficient to create
   * and destroy the FIFO on each packet. But, on the other hand, failing
   * to destroy the FIFO will leave the FIFO resources in place after the
   * communications have completed.
   *
   * I am thinking that there should be something like a timer.  The timer
   * would be started at the completion of each transfer and cancelled at
   * the beginning of each transfer.  If the timer expires, then the FIFO
   * would be destroyed.
   */

  /* #warning Missing logic */

  return OK;

#else
  char path[LOCAL_FULLPATH_LEN];

  /* Destroy the half duplex FIFO if it exists. */

  local_hd_name(conn->lc_path, path);
  return local_release_fifo(path);
#endif
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_open_client_rx
 *
 * Description:
 *   Open the client-side of the server-to-client FIFO.
 *
 ****************************************************************************/

int local_open_client_rx(FAR struct local_conn_s *client, bool nonblock)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the server-to-client path name */

  local_sc_name(client, path);

  /* Then open the file for read-only access */

  ret = local_rx_open(client, path, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the last reference is closed */

      ret = local_set_policy(&client->lc_infile, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: local_open_client_tx
 *
 * Description:
 *   Open the client-side of the client-to-server FIFO.
 *
 ****************************************************************************/

int local_open_client_tx(FAR struct local_conn_s *client, bool nonblock)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the client-to-server path name */

  local_cs_name(client, path);

  /* Then open the file for write-only access */

  ret = local_tx_open(client, path, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the last reference is closed */

      ret = local_set_policy(&client->lc_outfile, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: local_open_server_rx
 *
 * Description:
 *   Open the server-side of the client-to-server FIFO.
 *
 ****************************************************************************/

int local_open_server_rx(FAR struct local_conn_s *server, bool nonblock)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the client-to-server path name */

  local_cs_name(server, path);

  /* Then open the file for write-only access */

  ret = local_rx_open(server, path, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the last reference is closed */

      ret = local_set_policy(&server->lc_infile, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: local_open_server_tx
 *
 * Description:
 *   Only the server-side of the server-to-client FIFO.
 *
 ****************************************************************************/

int local_open_server_tx(FAR struct local_conn_s *server, bool nonblock)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the server-to-client path name */

  local_sc_name(server, path);

  /* Then open the file for read-only access */

  ret = local_tx_open(server, path, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the last reference is closed */

      ret = local_set_policy(&server->lc_outfile, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: local_open_receiver
 *
 * Description:
 *   Only the receiving side of the half duplex FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_open_receiver(FAR struct local_conn_s *conn, bool nonblock)
{
  char path[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the server-to-client path name */

  local_hd_name(conn->lc_path, path);

  /* Then open the file for read-only access */

  ret = local_rx_open(conn, path, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the buffer is empty. */

      ret = local_set_policy(&conn->lc_infile, 1);

      if (ret == 0)
        {
          /* Set POLLOUT threshold bigger than preamble len.
           * This is to avoid non-blocking read failed with -EAGAIN when
           * only preamble len is sent and read by reader.
           */

          ret = local_set_polloutthreshold(&conn->lc_infile,
                                           sizeof(uint16_t));
        }
    }

  return ret;
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

/****************************************************************************
 * Name: local_open_sender
 *
 * Description:
 *   Only the sending side of the half duplex FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_open_sender(FAR struct local_conn_s *conn, FAR const char *path,
                      bool nonblock)
{
  char fullpath[LOCAL_FULLPATH_LEN];
  int ret;

  /* Get the server-to-client path name */

  local_hd_name(path, fullpath);

  /* Then open the file for read-only access */

  ret = local_tx_open(conn, fullpath, nonblock);
  if (ret == OK)
    {
      /* Policy: Free FIFO resources when the buffer is empty. */

      ret = local_set_policy(&conn->lc_outfile, 1);
      if (ret == 0)
        {
          /* Set POLLIN threshold bigger than preamble len.
           * This is to avoid non-blocking read failed with -EAGAIN when
           * only preamble len is sent and read by reader.
           */

          ret = local_set_pollinthreshold(&conn->lc_outfile,
                                          sizeof(uint16_t));
        }
    }

  return ret;
}
#endif /* CONFIG_NET_LOCAL_DGRAM */

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
