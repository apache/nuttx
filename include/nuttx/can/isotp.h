/****************************************************************************
 * include/nuttx/can/isotp.h
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

#ifndef __INCLUDE_NUTTX_CAN_ISOTP_H
#define __INCLUDE_NUTTX_CAN_ISOTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Protocol control information frame types */

#define ISOTP_PCI_SF         0x00  /* Single frame */
#define ISOTP_PCI_FF         0x01  /* First frame */
#define ISOTP_PCI_CF         0x02  /* Consecutive frame */
#define ISOTP_PCI_FC         0x03  /* Flow control frame */

/* PCI type field: upper nibble of the first frame byte */

#define ISOTP_PCI_TYPE_POS   4
#define ISOTP_PCI_TYPE_MASK  0xf0
#define ISOTP_PCI_TYPE(v)    ((v) & ISOTP_PCI_TYPE_MASK)
#define ISOTP_PCI_TYPE_SF    (ISOTP_PCI_SF << ISOTP_PCI_TYPE_POS)
#define ISOTP_PCI_TYPE_FF    (ISOTP_PCI_FF << ISOTP_PCI_TYPE_POS)
#define ISOTP_PCI_TYPE_CF    (ISOTP_PCI_CF << ISOTP_PCI_TYPE_POS)
#define ISOTP_PCI_TYPE_FC    (ISOTP_PCI_FC << ISOTP_PCI_TYPE_POS)

/* Lower-nibble field masks */

#define ISOTP_SF_DL_MASK     0x0f  /* SF data length */
#define ISOTP_FF_DL_MASK     0x0f  /* FF data length (upper nibble) */
#define ISOTP_FC_FS_MASK     0x0f  /* FC flow status */
#define ISOTP_CF_SN_MASK     0x0f  /* CF sequence number */

/* Flow control status */

#define ISOTP_FC_CTS         0     /* Continue to send */
#define ISOTP_FC_WAIT        1     /* Wait */
#define ISOTP_FC_OVFLW       2     /* Overflow */

/* Maximum ISO-TP message length carried by the classic 12-bit FF length
 * field.  Messages longer than this are sent with the 32-bit FF_DL escape
 * (FF_DL == 0, followed by a 32-bit length), so the absolute upper bound is
 * 2^32-1.
 */

#define ISOTP_MAX_MSG_LEN    4095

/* N_Cr timeout per ISO 15765-2: max time between FF/FC and next CF */

#define ISOTP_NCR_TIMEOUT_MS 1000

/* Minimum FC frame payload: FS byte + BS byte + STmin byte */

#define ISOTP_FC_LEN         3

/* CAN FD extended SF header size: {0x00, length_byte} */

#define ISOTP_ESF_HDR_LEN    2

/* Escaped First Frame header size: {0x10, 0x00, len[31:24], len[23:16],
 * len[15:8], len[7:0]} used when the message length exceeds the 12-bit
 * FF_DL field (ISOTP_MAX_MSG_LEN).
 */

#define ISOTP_ESC_FF_HDR_LEN 6

/* Build the first PCI byte for each frame type */

#define ISOTP_MAKE_SF(dl)    (ISOTP_PCI_TYPE_SF | ((dl) & ISOTP_SF_DL_MASK))
#define ISOTP_MAKE_FF(hi)    (ISOTP_PCI_TYPE_FF | ((hi) & ISOTP_FF_DL_MASK))
#define ISOTP_MAKE_CF(sn)    (ISOTP_PCI_TYPE_CF | ((sn) & ISOTP_CF_SN_MASK))
#define ISOTP_MAKE_FC(fs)    (ISOTP_PCI_TYPE_FC | ((fs) & ISOTP_FC_FS_MASK))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ISO-TP length type. Messages up to 4095 bytes fit the classic 12-bit
 * FF_DL field; longer ones need 32 bits and the FF_DL escape, which is only
 * compiled in when CONFIG_LIBC_CAN_ISOTP_LONG_PDU is selected. Classic-only
 * builds should keep 16-bit fields to avoid spending memory on long
 * messages.
 */

#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
typedef uint32_t isotp_len_t;
#else
typedef uint16_t isotp_len_t;
#endif

/* TX encode context */

struct isotp_encode_s
{
  FAR const uint8_t *buf;  /* Source data */
  isotp_len_t        len;  /* Total message length */
  isotp_len_t        done; /* Bytes encoded so far */
  uint8_t            sn;   /* Next consecutive frame sequence number */
};

/* RX decode context */

struct isotp_decode_s
{
  FAR uint8_t *buf;     /* Destination buffer */
  isotp_len_t  len;     /* Buffer capacity */
  isotp_len_t  total;   /* Expected total length (set after FF) */
  isotp_len_t  done;    /* Bytes received so far */
  uint8_t      sn;      /* Expected next sequence number */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: isotp_encode_frame
 *
 * Description:
 *   Encode the next CAN frame payload from the context.
 *
 *   When frame[0]>>4 == ISOTP_PCI_FF the caller must receive a FC frame
 *   before calling again.
 *
 * Input Parameters:
 *   ctx   - ISO-TP encode context
 *   frame - ISO-TP data
 *   dlen  - in: CAN data field size (CAN_MAX_DLEN=8 or CANFD_MAX_DLEN=64),
 *           out: actual encoded frame length.
 *
 * Returned Value:
 *   > 0 when more frames follow, 0 when encoding is complete
 *   (SF or last CF), or -errno on error.
 *
 ****************************************************************************/

int isotp_encode_frame(FAR struct isotp_encode_s *ctx,
                       FAR uint8_t *frame, FAR uint8_t *dlen);

/****************************************************************************
 * Name: isotp_decode_frame
 *
 * Description:
 *   Process one received CAN frame payload.
 *
 *   After FF: send FC, then keep calling until ctx->done >= ctx->total.
 *   After SF: ctx->done == ctx->total, message is complete.
 *
 * Input Parameters:
 *   ctx   - ISO-TP decode context
 *   frame - ISO-TP data
 *   dlen  - CAN data size
 *
 * Returned Value:
 *   > 0 when more frames follow (FF or non-final CF),
 *   0 when decoding is complete (SF or last CF),
 *   or -errno on error.
 *
 ****************************************************************************/

int isotp_decode_frame(FAR struct isotp_decode_s *ctx,
                       FAR const uint8_t *frame, uint8_t dlen);

#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isotp_build_fc
 *
 * Description:
 *   Build a flow control CAN frame payload.
 *
 ****************************************************************************/

static inline void isotp_build_fc(FAR uint8_t *frame, uint8_t flags,
                                  uint8_t bs, uint8_t stmin)
{
  frame[0] = ISOTP_MAKE_FC(flags);
  frame[1] = bs;
  frame[2] = stmin;
}

/****************************************************************************
 * Name: isotp_encode_init
 *
 * Description:
 *   Initialize a TX encode context for a message of 'len' bytes.
 *
 * Returned Value:
 *   0 on success, or -EMSGSIZE when the message is too long for the
 *   configured PDU support.
 *
 ****************************************************************************/

static inline int isotp_encode_init(FAR struct isotp_encode_s *ctx,
                                     FAR const uint8_t *buf, size_t len)
{
#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
  if (len > UINT32_MAX)
    {
      return -EMSGSIZE;
    }
#else
  /* Without long-PDU support the First Frame carries only a 12-bit length. */

  if (len > ISOTP_MAX_MSG_LEN)
    {
      return -EMSGSIZE;
    }
#endif

  ctx->buf  = buf;
  ctx->len  = len;
  ctx->done = 0;
  ctx->sn   = 1;
  return 0;
}

/****************************************************************************
 * Name: isotp_decode_init
 ****************************************************************************/

static inline void isotp_decode_init(FAR struct isotp_decode_s *ctx,
                                     FAR uint8_t *buf, isotp_len_t len)
{
  ctx->buf   = buf;
  ctx->len   = len;
  ctx->total = 0;
  ctx->done  = 0;
  ctx->sn    = 1;
}

#endif /* __INCLUDE_NUTTX_CAN_ISOTP_H */
