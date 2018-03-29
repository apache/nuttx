/****************************************************************************
 * wireless/bluetooth/bt_att.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BT_BUF_H
#define __INCLUDE_NUTTX_WIRELESS_BT_BUF_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BT_BUF_MAX_DATA
 * Maximum amount of data that can fit in a buffer.
 *
 * The biggest foreseeable buffer size requirement right now comes from
 * the Bluetooth 4.2 SMP MTU which is 65. This then become 65 + 4 (L2CAP
 * header) + 4 (ACL header) + 1 (H4 header) = 74. This also covers the
 * biggest HCI commands and events which are a bit under the 70 byte
 * mark.
 */

#define BT_BUF_MAX_DATA 74

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

struct bt_buf_s
{
  FAR struct iob_s *iob; /* IOB container of the buffer */
  union
  {
    struct bt_buf_hci_data_s hci;
    struct bt_buf_acl_data_s acl;
  } u;

  FAR uint8_t *data;     /* Start of data in the buffer */
  uint8_t len;           /* Length of data in the buffer */
  uint8_t ref  : 5;      /* Reference count */
  uint8_t type : 3;      /* Type of data contained in the buffer */

  /* The full available buffer. */

  uint8_t buf[BT_BUF_MAX_DATA];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_buf_get
 *
 * Description:
 *   Get buffer from the available buffers pool with specified type and
 *   reserved headroom.
 *
 * Input Parameters:
 *   type         - Buffer type.
 *   reserve_head - How much headroom to reserve.
 *
 * Returned Value:
 *   New buffer or NULL if out of buffers.
 *
 *   WARNING: If there are no available buffers and the function is
 *   called from a task or thread the call will block until a buffer
 *   becomes available in the pool.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_buf_get(enum bt_buf_type_e type, size_t reserve_head);

/****************************************************************************
 * Name: bt_buf_put
 *
 * Description:
 *   Decrements the reference count of a buffer and puts it back into the
 *   pool if the count reaches zero.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 ****************************************************************************/

void bt_buf_put(FAR struct bt_buf_s *buf);

/****************************************************************************
 * Name: bt_buf_hold
 *
 * Description:
 *   Increment the reference count of a buffer.
 *
 * Input Parameters:
 *   buf - Buffer.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_buf_hold(FAR struct bt_buf_s *buf);

/****************************************************************************
 * Name: bt_buf_add
 *
 * Description:
 *   Increments the data length of a buffer to account for more data
 *   at the end.
 *
 * Input Parameters:
 *   buf - Buffer to update.
 *   len - Number of bytes to increment the length with.
 *
 * Returned Value:
 *   The original tail of the buffer.
 *
 ****************************************************************************/

FAR void *bt_buf_add(FAR struct bt_buf_s *buf, size_t len);

/****************************************************************************
 * Name: bt_buf_add_le16
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

void bt_buf_add_le16(FAR struct bt_buf_s *buf, uint16_t value);

/****************************************************************************
 * Name: bt_buf_push
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

FAR void *bt_buf_push(FAR struct bt_buf_s *buf, size_t len);

/****************************************************************************
 * Name: bt_buf_pull
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

FAR void *bt_buf_pull(FAR struct bt_buf_s *buf, size_t len);

/****************************************************************************
 * Name: bt_buf_pull_le16
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

uint16_t bt_buf_pull_le16(FAR struct bt_buf_s *buf);

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

/****************************************************************************
 * Name: bt_buf_init
 *
 * Description:
 *   Initialize the buffers with specified amount of incoming and outgoing
 *   ACL buffers. The HCI command and event buffers will be allocated from
 *   whatever is left over.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success or (negative) error code on failure.
 *
 ****************************************************************************/

int bt_buf_init(void);

#endif /* __INCLUDE_NUTTX_WIRELESS_BT_BUF_H */
