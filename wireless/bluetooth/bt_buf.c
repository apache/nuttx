/****************************************************************************
 * wireless/bluetooth/bt_buf_s.c
 * Bluetooth buffer management
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_buf.h>

#include "bt_hcicore.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_BLUETOOTH_BUFFER_PREALLOC) || \
    CONFIG_BLUETOOTH_BUFFER_PREALLOC < 1
#  undef CONFIG_BLUETOOTH_BUFFER_PREALLOC
#  define CONFIG_BLUETOOTH_BUFFER_PREALLOC 20
#endif

#if !defined(CONFIG_BLUETOOTH_BUFFER_IRQRESERVE) || \
    CONFIG_BLUETOOTH_BUFFER_IRQRESERVE < 0
#  undef CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
#  define CONFIG_BLUETOOTH_BUFFER_IRQRESERVE 0
#endif

#if CONFIG_BLUETOOTH_BUFFER_IRQRESERVE > CONFIG_BLUETOOTH_BUFFER_PREALLOC
#  undef CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
#  define CONFIG_BLUETOOTH_BUFFER_IRQRESERVE CONFIG_BLUETOOTH_BUFFER_PREALLOC
#endif

/* Memory Pools */

#define POOL_BUFFER_GENERAL  0
#define POOL_BUFFER_IRQ      1
#define POOL_BUFFER_DYNAMIC  2

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_BLUETOOTH_BUFFER_PREALLOC > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
/* g_buf_free is a list of buffers that are available for general use.  The
 * number of messages in this list is a system configuration item.
 */

static struct bt_buf_s *g_buf_free;
#endif

#if CONFIG_BLUETOOTH_BUFFER_IRQRESERVE > 0
/* The g_buf_free_irq is a list of buffer structures that are reserved for
 * use by only by interrupt handlers.
 */

static struct bt_buf_s *g_buf_free_irq;
#endif

/* Pool of pre-allocated buffer structures */

static struct bt_buf_s
  g_buf_pool[CONFIG_BLUETOOTH_BUFFER_PREALLOC];

static bool g_poolinit = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_buf_initialize
 *
 * Description:
 *   This function initializes the buffer allocator.  This function must
 *   be called early in the initialization sequence before any radios
 *   begin operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_buf_initialize(void)
{
  FAR struct bt_buf_s *pool = g_buf_pool;
  int remaining = CONFIG_BLUETOOTH_BUFFER_PREALLOC;

  /* Only allow the pool to be initialized once */

  if (g_poolinit)
    {
      return;
    }

  g_poolinit = true;

#if CONFIG_BLUETOOTH_BUFFER_PREALLOC > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
  /* Initialize g_buf_free, the list of buffer structures that are available
   * for general use.
   */

  g_buf_free = NULL;
  while (remaining > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE)
    {
      FAR struct bt_buf_s *buf = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      buf->flink = g_buf_free;
      g_buf_free = buf;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif

#if CONFIG_BLUETOOTH_BUFFER_IRQRESERVE > 0
  /* Initialize g_buf_free_irq is a list of buffer structures reserved for
   * use by only by interrupt handlers.
   */

  g_buf_free_irq = NULL;
  while (remaining > 0)
    {
      FAR struct bt_buf_s *buf = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      buf->flink     = g_buf_free_irq;
      g_buf_free_irq = buf;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif
}

/****************************************************************************
 * Name: bt_buf_alloc
 *
 * Description:
 *   The bt_buf_alloc function will get a free buffer for use by the
 *   Bluetooth stack with specified type and reserved headroom.  The
 *   reference count is initially set to one.
 *
 *   Interrupt handling logic will first attempt to allocate from the
 *   g_buf_free list.  If that list is empty, it will attempt to allocate
 *   from its reserve, g_buf_free_irq.  If that list is empty, then the
 *   allocation fails (NULL is returned).
 *
 *   Non-interrupt handler logic will attempt to allocate from g_buf_free
 *   list.  If that the list is empty, then the buffer structure will be
 *   allocated from the dynamic memory pool with some performance hit.
 *
 * Input Parameters:
 *   type         - Buffer type.
 *   iob          - The raw I/O buffer.  If NULL, then bt_buf_alloc() will
 *                  allocate the frame IOB.
 *   reserve_head - How much headroom to reserve.  Will be ignored if
 *                  a non-NULL iob is provided.  In that case, the head
 *                  room is determined by the actual data offset.
 *
 * Returned Value:
 *   A reference to the allocated buffer structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_buf_alloc(enum bt_buf_type_e type,
                                  FAR struct iob_s *iob,
                                  size_t reserve_head)
{
  FAR struct bt_buf_s *buf;
  irqstate_t flags;
  uint8_t pool;

  /* If we were called from an interrupt handler, then try to get the meta-
   * data structure from generally available list of messages. If this fails,
   * then try the list of messages reserved for interrupt handlers
   */

  flags = spin_lock_irqsave(); /* Always necessary in SMP mode */
  if (up_interrupt_context())
    {
#if CONFIG_BLUETOOTH_BUFFER_PREALLOC > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
      /* Try the general free list */

      if (g_buf_free != NULL)
        {
          buf            = g_buf_free;
          g_buf_free     = buf->flink;

          spin_unlock_irqrestore(flags);
          pool           = POOL_BUFFER_GENERAL;
        }
      else
#endif
#if CONFIG_BLUETOOTH_BUFFER_IRQRESERVE > 0
      /* Try the list list reserved for interrupt handlers */

      if (g_buf_free_irq != NULL)
        {
          buf            = g_buf_free_irq;
          g_buf_free_irq = buf->flink;

          spin_unlock_irqrestore(flags);
          pool           = POOL_BUFFER_IRQ;
        }
      else
#endif
        {
          spin_unlock_irqrestore(flags);
          return NULL;
        }
    }

  /* We were not called from an interrupt handler. */

  else
    {
#if CONFIG_BLUETOOTH_BUFFER_PREALLOC > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
      /* Try the general free list */

      if (g_buf_free != NULL)
        {
          buf           = g_buf_free;
          g_buf_free    = buf->flink;

          leave_critical_section(flags);
          pool          = POOL_BUFFER_GENERAL;
        }
      else
#endif
        {
          /* If we cannot get a buffer structure from the free list, then we
           * will have to allocate one from the kernel memory pool.
           */

          leave_critical_section(flags);
          buf = (FAR struct bt_buf_s *)kmm_malloc((sizeof (struct bt_buf_s)));

          /* Check if we successfully allocated the buffer structure */

          if (buf == NULL)
            {
              /* No..  memory not available */

              wlerr("ERROR: Failed to allocate buffer.\n");
              return NULL;
            }

          /* Remember that this buffer structure was dynamically allocated */

          pool = POOL_BUFFER_DYNAMIC;
        }
    }

  /* We have successfully allocated memory from some source.  Initialize the
   * allocated buffer structure.
   */

  memset(buf, 0, sizeof(struct bt_buf_s));
  buf->pool = pool;
  buf->ref  = 1;
  buf->type = type;

  /* Were we provided with an IOB? */

  if (iob != NULL)
    {
      /* Yes.. use that IOB */

      DEBUGASSERT(iob->io_len >= iob->io_offset &&
                  iob->io_len <= BLUETOOTH_MAX_FRAMELEN);

      buf->frame = iob;
      buf->data  = &iob->io_data[iob->io_offset];
      buf->len   = iob->io_len - iob->io_offset; /* IOB length includes offset */
    }
  else
    {
      /* No.. Allocate an IOB to hold the actual frame data.  This call will
       * normally block in the event that there is no available IOB memory.
       * It will return NULL is called from an interrupt handler with no
       * available buffers.
       */

      buf->frame = iob_alloc(false);
      if (!buf->frame)
        {
          wlerr("ERROR:  Failed to allocate an IOB\n");
          bt_buf_release(buf);
          return NULL;
        }

      buf->frame->io_offset = reserve_head;
      buf->frame->io_len    = reserve_head; /* IOB length includes offset */
      buf->frame->io_pktlen = reserve_head;

      buf->data = buf->frame->io_data + reserve_head;
    }

  wlinfo("buf %p type %d reserve %u\n", buf, buf->type, reserve_head);
  return buf;
}

/****************************************************************************
 * Name: bt_buf_release
 *
 * Description:
 *   The bt_buf_release function will return a buffer structure to
 *   the free pool of buffers if it was a pre-allocated buffer structure.
 *   If the buffer structure was allocated dynamically it will be
 *   deallocated.
 *
 * Input Parameters:
 *   buf - Buffer structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_buf_release(FAR struct bt_buf_s *buf)
{
  enum bt_buf_type_e type;
  irqstate_t flags;
  uint16_t handle;

  wlinfo("buf %p ref %u type %d\n", buf, buf->ref, buf->type);

  if (--buf->ref > 0)
    {
      wlinfo("Remaining references: %d\n", buf->ref);
      return;
    }

  handle = buf->u.acl.handle;
  type   = buf->type;

  /* Free the contained frame and return the container to the correct memory
   * pool.
   */

  if (buf->frame != NULL)
    {
      iob_free(buf->frame);
      buf->frame = NULL;
    }

#if CONFIG_BLUETOOTH_BUFFER_PREALLOC > CONFIG_BLUETOOTH_BUFFER_IRQRESERVE
  /* If this is a generally available pre-allocated buffer structure,
   * then just put it back in the free list.
   */

  if (buf->pool == POOL_BUFFER_GENERAL)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags      = spin_lock_irqsave();
      buf->flink = g_buf_free;
      g_buf_free = buf;
      spin_unlock_irqrestore(flags);
    }
  else
#endif

#if CONFIG_BLUETOOTH_BUFFER_IRQRESERVE > 0
  /* If this is a buffer structure pre-allocated for interrupts,
   * then put it back in the correct free list.
   */

  if (buf->pool == POOL_BUFFER_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags          = spin_lock_irqsave();
      buf->flink     = g_buf_free_irq;
      g_buf_free_irq = buf;
      spin_unlock_irqrestore(flags);
    }
  else
#endif

    {
      /* Otherwise, deallocate it. */

      DEBUGASSERT(buf->pool == POOL_BUFFER_DYNAMIC);
      sched_kfree(buf);
    }

  wlinfo("Buffer freed: %p\n", buf);

  if (type == BT_ACL_IN)
    {
      FAR struct bt_hci_cp_host_num_completed_packets_s *cp;
      FAR struct bt_hci_handle_count_s *hc;

      wlinfo("Reporting completed packet for handle %u\n", handle);

      buf = bt_hci_cmd_create(BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS,
                              sizeof(*cp) + sizeof(*hc));
      if (buf == NULL)
        {
          wlerr("ERROR: Unable to allocate new HCI command\n");
          return;
        }

      cp              = bt_buf_extend(buf, sizeof(*cp));
      cp->num_handles = BT_HOST2LE16(1);

      hc              = bt_buf_extend(buf, sizeof(*hc));
      hc->handle      = BT_HOST2LE16(handle);
      hc->count       = BT_HOST2LE16(1);

      bt_hci_cmd_send(BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS, buf);
    }
}

/****************************************************************************
 * Name: bt_buf_addref
 *
 * Description:
 *   Increment the reference count of a buffer.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_buf_addref(FAR struct bt_buf_s *buf)
{
  wlinfo("buf %p (old) ref %u type %d\n", buf, buf->ref, buf->type);

  buf->ref++;
  return buf;
}

/****************************************************************************
 * Name: bt_buf_extend
 *
 * Description:
 *   Increments the data length of a buffer to account for more data
 *   at the end of the buffer.
 *
 * Input Parameters:
 *   buf - Buffer to update.
 *   len - Number of bytes to increment the length with.
 *
 * Returned Value:
 *   The original tail of the buffer.
 *
 ****************************************************************************/

FAR void *bt_buf_extend(FAR struct bt_buf_s *buf, size_t len)
{
  FAR uint8_t *tail = bt_buf_tail(buf);

  wlinfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(bt_buf_tailroom(buf) >= len);

  buf->len += len;
  return tail;
}

/****************************************************************************
 * Name: bt_buf_put_le16
 *
 * Description:
 *   Adds 16-bit value in little endian format at the end of buffer.
 *   Increments the data length of a buffer to account for more data
 *   at the end.
 *
 * Input Parameters:
 *   buf   - Buffer to update.
 *   value - 16-bit value to be added.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_buf_put_le16(FAR struct bt_buf_s *buf, uint16_t value)
{
  wlinfo("buf %p value %u\n", buf, value);

  value = BT_HOST2LE16(value);
  memcpy(bt_buf_extend(buf, sizeof(value)), &value, sizeof(value));
}

/****************************************************************************
 * Name: bt_buf_provide
 *
 * Description:
 *   Modifies the data pointer and buffer length to account for more data
 *   in the beginning of the buffer.
 *
 * Input Parameters:
 *   buf - Buffer to update.
 *   len - Number of bytes to add to the beginning.
 *
 * Returned Value:
 *   The new beginning of the buffer data.
 *
 ****************************************************************************/

FAR void *bt_buf_provide(FAR struct bt_buf_s *buf, size_t len)
{
  wlinfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(buf != NULL && buf->frame != NULL &&
              bt_buf_headroom(buf) >= len);

  buf->data -= len;
  buf->len  += len;
  return buf->data;
}

/****************************************************************************
 * Name: bt_buf_consume
 *
 * Description:
 *   Removes data from the beginning of the buffer by modifying the data
 *   pointer and buffer length.
 *
 * Input Parameters:
 *   len - Number of bytes to remove.
 *
 * Returned Value:
 *   New beginning of the buffer data.
 *
 ****************************************************************************/

FAR void *bt_buf_consume(FAR struct bt_buf_s *buf, size_t len)
{
  wlinfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(buf->len >= len);

  buf->len -= len;
  return buf->data += len;
}

/****************************************************************************
 * Name: bt_buf_get_le16
 *
 * Description:
 *   Same idea as with bt_buf_pull(), but a helper for operating on
 *   16-bit little endian data.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 * Returned Value:
 *   16-bit value converted from little endian to host endian.
 *
 ****************************************************************************/

uint16_t bt_buf_get_le16(FAR struct bt_buf_s * buf)
{
  uint16_t value;

  value = BT_GETUINT16((FAR uint8_t *)buf->data);
  bt_buf_consume(buf, sizeof(value));

  return BT_LE162HOST(value);
}

/****************************************************************************
 * Name: bt_buf_headroom
 *
 * Description:
 *   Check how much free space there is in the beginning of the buffer.
 *
 * Returned Value:
 *   Number of bytes available in the beginning of the buffer.
 *
 ****************************************************************************/

size_t bt_buf_headroom(FAR struct bt_buf_s *buf)
{
  DEBUGASSERT(buf != NULL && buf->frame != NULL);
  return buf->data - buf->frame->io_data;
}

/****************************************************************************
 * Name: bt_buf_tailroom
 *
 * Description:
 *   Check how much free space there is at the end of the buffer.
 *
 * Returned Value:
 *   Number of bytes available at the end of the buffer.
 *
 ****************************************************************************/

size_t bt_buf_tailroom(FAR struct bt_buf_s * buf)
{
  return BLUETOOTH_MAX_FRAMELEN - bt_buf_headroom(buf) - buf->len;
}
