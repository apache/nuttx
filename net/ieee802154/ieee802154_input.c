/****************************************************************************
 * net/ieee802154/ieee802154_input.c
 * Handle incoming IEEE 802.15.4 frame input
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/ieee802154.h>

#include "devif/devif.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_count_frames
 *
 * Description:
 *   Return the number of frames in the RX queue.
 *
 * Input Parameters:
 *   conn   - The socket connection structure.
 *
 * Returned Value:
 *   The number of frames in the queue.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_ASSERTIONS) && CONFIG_NET_IEEE802154_BACKLOG > 0
static int ieee802154_count_frames(FAR struct ieee802154_conn_s *conn)
{
  FAR struct ieee802154_container_s *container;
  int count;

  for (count = 0, container = conn->rxhead;
       container != NULL;
       count++, container = container->ic_flink)
    {
    }

  return count;
}
#endif

/****************************************************************************
 * Name: ieee802154_queue_frame
 *
 * Description:
 *   Add one frame to the connection's RX queue.
 *
 * Input Parameters:
 *   conn   - The socket connection structure.
 *   framel - A single frame to add to the RX queue.
 *   meta   - Meta data characterizing the received frame.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int ieee802154_queue_frame(FAR struct ieee802154_conn_s *conn,
                                  FAR struct iob_s *frame,
                                  FAR struct ieee802154_data_ind_s *meta)
{
  FAR struct ieee802154_container_s *container;

  /* Allocate a container for the frame */

  container = ieee802154_container_allocate();
  if (container == NULL)
    {
      nerr("ERROR: Failed to allocate a container\n");
      return -ENOMEM;
    }

  /* Initialize the container */

  memset(&container->ic_src, 0, sizeof(struct ieee802154_saddr_s));

  DEBUGASSERT(meta->src.mode != IEEE802154_ADDRMODE_NONE);
  container->ic_src.s_mode = meta->src.mode;
  IEEE802154_PANIDCOPY(container->ic_src.s_panid, meta->src.panid);

  if (meta->src.mode == IEEE802154_ADDRMODE_SHORT)
    {
      IEEE802154_SADDRCOPY(container->ic_src.s_saddr, meta->src.saddr);
    }
  else if (meta->src.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      IEEE802154_EADDRCOPY(container->ic_src.s_eaddr, meta->src.eaddr);
    }

  DEBUGASSERT(frame != NULL);
  container->ic_iob = frame;

  /* Add the container to the tail of the list of incoming frames */

  container->ic_flink = NULL;
  if (conn->rxtail == NULL)
    {
      conn->rxhead    = container;
    }
  else
    {
      conn->rxtail->ic_flink = container;
    }

#if CONFIG_NET_IEEE802154_BACKLOG > 0
  /* If incrementing the count would exceed the maximum backlog value, then
   * delete the oldest frame from the head of the RX queue.
   */

  if (conn->backlog >= CONFIG_NET_IEEE802154_BACKLOG)
    {
      DEBUGASSERT(conn->backlog == CONFIG_NET_IEEE802154_BACKLOG);

      /* Remove the container from the tail RX input queue. */

      container           = conn->rxhead;
      DEBUGASSERT(container != NULL);
      conn->rxhead        = container->ic_flink;
      container->ic_flink = NULL;

      /* Did the RX queue become empty? */

      if (conn->rxhead == NULL)
        {
          conn->rxtail = NULL;
        }

      DEBUGASSERT(container != NULL && container->ic_iob != NULL);

      /* Free both the IOB and the container */

      iob_free(container->ic_iob, IOBUSER_NET_SOCK_IEEE802154);
      ieee802154_container_free(container);
    }
  else
    {
      /* Increment the count of frames in the queue. */

      conn->backlog++;
    }

  DEBUGASSERT((int)conn->backlog == ieee802154_count_frames(conn));
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_input
 *
 * Description:
 *   Handle incoming IEEE 802.15.4 input
 *
 *   This function is called when the radio device driver has received an
 *   frame from the network.  The frame from the device driver must be
 *   provided in by the IOB frame argument of the  function call:
 *
 *   - The frame data is in the IOB io_data[] buffer,
 *   - The length of the frame is in the IOB io_len field, and
 *   - The offset past and radio MAC header is provided in the io_offset
 *     field.
 *
 *   The frame argument may refer to a single frame (a list of length one)
 *   or may it be the head of a list of multiple frames.
 *
 *   - The io_flink field points to the next frame in the list (if enable)
 *   - The last frame in the list will have io_flink == NULL.
 *
 * Input Parameters:
 *   radio       The radio network driver interface.
 *   framelist - The head of an incoming list of frames.  Normally this
 *               would be a single frame.  A list may be provided if
 *               appropriate, however.
 *   meta      - Meta data characterizing the received frame.
 *
 *               If there are multiple frames in the list, this metadata
 *               must apply to all of the frames in the list.
 *
 * Returned Value:
 *   OK    The IEEE 802.15.4 has been processed  and can be deleted
 *   ERROR Hold the IEEE 802.15.4 and try again later. There is a listening
 *         socket but no recv in place to catch the IEEE 802.15.4 yet.
 *         Useful when a packet arrives before a recv call is in place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

int ieee802154_input(FAR struct radio_driver_s *radio,
                     FAR struct iob_s *framelist,
                     FAR struct ieee802154_data_ind_s *meta)
{
  FAR struct ieee802154_conn_s *conn;
  FAR struct iob_s *frame;
  FAR struct iob_s *next;
  int ret = OK;

  /* Check if there is a connection that will accept this packet */

  conn = ieee802154_conn_active(meta);
  if (conn != NULL)
    {
      /* Setup for the application callback (NOTE:  These should not be
       * used by PF_IEEE802154 sockets).
       */

      radio->r_dev.d_appdata = radio->r_dev.d_buf;
      radio->r_dev.d_len     = 0;
      radio->r_dev.d_sndlen  = 0;

      /* The framelist probably contains only a single frame, but we will
       * process it as a list of frames.
       */

      for (frame = framelist; frame != NULL; frame = next)
        {
          /* Remove the frame from the list */

          next            = frame->io_flink;
          frame->io_flink = NULL;

          /* Add the frame to the RX queue */

          ret = ieee802154_queue_frame(conn, frame, meta);
          if (ret < 0)
            {
              nerr("ERROR: Failed to queue frame: %d\n", ret);
              iob_free(frame, IOBUSER_NET_SOCK_IEEE802154);
            }
        }

      /* Perform the application callback.  The frame may be processed now
       * if there is a user wait for an incoming frame.  Or it may pend in
       * the RX queue until some user process reads the frame.  NOTE:  The
       * return value from ieee802154_callback would distinguish these
       * cases:  IEEE802154_NEWDATA will still be processed if the frame
       * was not consumed.
       */

      ieee802154_callback(radio, conn, IEEE802154_NEWDATA);
    }
  else
    {
      nwarn("WARNING: No listener\n");
    }

  return ret;
}

#endif /* CONFIG_NET_IEEE802154 */
