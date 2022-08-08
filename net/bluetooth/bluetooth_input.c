/****************************************************************************
 * net/bluetooth/bluetooth_input.c
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
 * Handle incoming Bluetooth frame input
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
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "devif/devif.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_count_frames
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

#if CONFIG_NET_BLUETOOTH_BACKLOG > 0
static int bluetooth_count_frames(FAR struct bluetooth_conn_s *conn)
{
  FAR struct bluetooth_container_s *container;
  int count;

  for (count = 0, container = conn->bc_rxhead;
       container != NULL;
       count++, container = container->bn_flink)
    {
    }

  return count;
}
#endif

/****************************************************************************
 * Name: bluetooth_queue_frame
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

static int bluetooth_queue_frame(FAR struct bluetooth_conn_s *conn,
                                  FAR struct iob_s *frame,
                                  FAR struct bluetooth_frame_meta_s *meta)
{
  FAR struct bluetooth_container_s *container;

  /* Allocate a container for the frame */

  container = bluetooth_container_allocate();
  if (container == NULL)
    {
      nerr("ERROR: Failed to allocate a container\n");
      return -ENOMEM;
    }

  /* Initialize the container */

  memset(&container->bn_raddr, 0, sizeof(bt_addr_t));
  container->bn_channel = meta->bm_channel;
  BLUETOOTH_ADDRCOPY(&container->bn_raddr, &meta->bm_raddr);

  DEBUGASSERT(frame != NULL);
  container->bn_iob = frame;

  /* Add the container to the tail of the list of incoming frames */

  container->bn_flink = NULL;
  if (conn->bc_rxtail == NULL)
    {
      conn->bc_rxhead = container;
    }
  else
    {
      conn->bc_rxtail->bn_flink = container;
    }

  conn->bc_rxtail = container;

#if CONFIG_NET_BLUETOOTH_BACKLOG > 0
  /* If incrementing the count would exceed the maximum bc_backlog value,
   * then delete the oldest frame from the head of the RX queue.
   */

  if (conn->bc_backlog >= CONFIG_NET_BLUETOOTH_BACKLOG)
    {
      DEBUGASSERT(conn->bc_backlog == CONFIG_NET_BLUETOOTH_BACKLOG);

      /* Remove the container from the tail RX input queue. */

      container           = conn->bc_rxhead;
      DEBUGASSERT(container != NULL);
      conn->bc_rxhead     = container->bn_flink;
      container->bn_flink = NULL;

      /* Did the RX queue become empty? */

      if (conn->bc_rxhead == NULL)
        {
          conn->bc_rxtail = NULL;
        }

      DEBUGASSERT(container != NULL && container->bn_iob != NULL);

      /* Free both the IOB and the container */

      iob_free(container->bn_iob);
      bluetooth_container_free(container);
    }
  else
    {
      /* Increment the count of frames in the queue. */

      conn->bc_backlog++;
    }

  DEBUGASSERT((int)conn->bc_backlog == bluetooth_count_frames(conn));
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_input
 *
 * Description:
 *   Handle incoming Bluetooth input
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
 *   OK    The Bluetooth has been processed  and can be deleted
 *   ERROR Hold the Bluetooth and try again later. There is a listening
 *         socket but no recv in place to catch the Bluetooth yet.
 *         Useful when a packet arrives before a recv call is in place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

int bluetooth_input(FAR struct radio_driver_s *radio,
                     FAR struct iob_s *framelist,
                     FAR struct bluetooth_frame_meta_s *meta)
{
  FAR struct bluetooth_conn_s *conn;
  FAR struct iob_s *frame;
  FAR struct iob_s *next;
  int ret = OK;

  /* Check if there is a connection that will accept this packet */

  conn = bluetooth_conn_active(meta);
  if (conn != NULL)
    {
      /* Setup for the application callback (NOTE:  These should not be
       * used by PF_BLUETOOTH sockets).
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

          ret = bluetooth_queue_frame(conn, frame, meta);
          if (ret < 0)
            {
              nerr("ERROR: Failed to queue frame: %d\n", ret);
              iob_free(frame);
            }
        }

      /* Perform the application callback.  The frame may be processed now
       * if there is a user wait for an incoming frame.  Or it may pend in
       * the RX queue until some user process reads the frame.  NOTE:  The
       * return value from bluetooth_callback would distinguish these
       * cases:  BLUETOOTH_NEWDATA will still be processed if the frame
       * was not consumed.
       */

      bluetooth_callback(radio, conn, BLUETOOTH_NEWDATA);
    }
  else
    {
      nwarn("WARNING: No listener\n");
    }

  return ret;
}

#endif /* CONFIG_NET_BLUETOOTH */
