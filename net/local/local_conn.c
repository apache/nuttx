/****************************************************************************
 * net/local/local_conn.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "local/local.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_initialize
 *
 * Description:
 *   Initialize the local, Unix domain connection structures.  Called once
 *   and only from the common network initialization logic.
 *
 ****************************************************************************/

void local_initialize(void)
{
#ifdef CONFIG_NET_LOCAL_STREAM
  dq_init(&g_local_listeners);
#endif
}

/****************************************************************************
 * Name: local_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized Unix domain socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   API
 *
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void)
{
  FAR struct local_conn_s *conn =
    (FAR struct local_conn_s *)kmm_zalloc(sizeof(struct local_conn_s));

  if (conn != NULL)
    {
      /* Initialize non-zero elements the new connection structure.  Since
       * Since the memory was allocated with kmm_zalloc(), it is not
       * necessary to zerio-ize any structure elements.
       */

#ifdef CONFIG_NET_LOCAL_STREAM
      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&conn->lc_waitsem, 0, 0);
      nxsem_set_protocol(&conn->lc_waitsem, SEM_PRIO_NONE);
#endif
    }

  return conn;
}

/****************************************************************************
 * Name: local_free()
 *
 * Description:
 *   Free a packet Unix domain connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void local_free(FAR struct local_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);

  /* Make sure that the read-only FIFO is closed */

  if (conn->lc_infile.f_inode != NULL)
    {
      file_close(&conn->lc_infile);
      conn->lc_infile.f_inode = NULL;
    }

  /* Make sure that the write-only FIFO is closed */

  if (conn->lc_outfile.f_inode != NULL)
    {
      file_close(&conn->lc_outfile);
      conn->lc_outfile.f_inode = NULL;
    }

#ifdef CONFIG_NET_LOCAL_STREAM
  /* Destroy all FIFOs associted with the connection */

  local_release_fifos(conn);
  nxsem_destroy(&conn->lc_waitsem);
#endif

  /* And free the connection structure */

  kmm_free(conn);
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
