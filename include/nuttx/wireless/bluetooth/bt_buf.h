/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_buf.h
 * Bluetooth buffer management.
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
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BUF_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BUF_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Type of data contained in a buffer */

enum bt_buf_type_e
{
  BT_CMD,             /* HCI command */
  BT_EVT,             /* HCI event */
  BT_ACL_OUT,         /* Outgoing ACL data */
  BT_ACL_IN,          /* Incoming ACL data */
  BT_DUMMY = BT_CMD   /* Only used for waking up kernel threads */
};

/* HCI command specific information */

struct bt_buf_hci_data_s
{
  /* Used by bt_hci_cmd_send_sync. Initially contains the waiting
   * semaphore, as the semaphore is given back contains the bt_buf
   * for the return parameters.
   */

  FAR void *sync;

  /* The command opcode that the buffer contains */

  uint16_t opcode;
};

/* ACL data buffer specific information */

struct bt_buf_acl_data_s
{
  uint16_t handle;
};

/* This structure defines one buffer */

struct iob_s;  /* Forward reference */

struct bt_buf_s
{
  FAR struct bt_buf_s *flink;

  union
  {
    struct bt_buf_hci_data_s hci;
    struct bt_buf_acl_data_s acl;
  } u;

  FAR uint8_t *data;     /* Start of data in the buffer */
  uint8_t len;           /* Length of data in the buffer */
  uint8_t pool;          /* Memory pool */
  uint8_t ref;           /* Reference count */
  uint8_t type;          /* Type of data contained in the buffer */

  /* The full available buffer. */

  FAR struct iob_s *frame;
};

/* This structure defines a singly linked list of buffers */

struct bt_bufferlist_s
{
  FAR struct bt_buf_s *head;
  FAR struct bt_buf_s *tail;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 *   iob          - The raw I/O buffer.  If NULL, then bt_buf_alloc will
 *                  allocate.
 *   reserve_head - How much headroom to reserve.
 *
 * Returned Value:
 *   A reference to the allocated buffer structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

struct iob_s; /* Forward reference */

FAR struct bt_buf_s *bt_buf_alloc(enum bt_buf_type_e type,
                                  FAR struct iob_s *iob,
                                  size_t reserve_head);

/****************************************************************************
 * Name: bt_buf_release
 *
 * Description:
 *   Decrements the reference count of a buffer and returns the buffer to the
 *   memory pool if the count decrements zero.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_buf_release(FAR struct bt_buf_s *buf);

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

FAR struct bt_buf_s *bt_buf_addref(FAR struct bt_buf_s *buf);

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

FAR void *bt_buf_extend(FAR struct bt_buf_s *buf, size_t len);

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

void bt_buf_put_le16(FAR struct bt_buf_s *buf, uint16_t value);

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

FAR void *bt_buf_provide(FAR struct bt_buf_s *buf, size_t len);

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

FAR void *bt_buf_consume(FAR struct bt_buf_s *buf, size_t len);

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

uint16_t bt_buf_get_le16(FAR struct bt_buf_s *buf);

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

size_t bt_buf_tailroom(FAR struct bt_buf_s *buf);

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

size_t bt_buf_headroom(FAR struct bt_buf_s *buf);

/****************************************************************************
 * Name: bt_buf_tail
 *
 * Description:
 *   Get a pointer to the end of the data in a buffer.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 * Returned Value:
 *   Tail pointer for the buffer.
 *
 ****************************************************************************/

#define bt_buf_tail(buf) ((buf)->data + (buf)->len)

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BUF_H */
