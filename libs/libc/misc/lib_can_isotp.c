/****************************************************************************
 * libs/libc/misc/lib_can_isotp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/can.h>
#include <nuttx/can/isotp.h>
#include <sys/param.h>

#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   >0 when more frames follow,
 *   0 when encoding is complete (SF or last CF), or
 *   -errno on error.
 *
 ****************************************************************************/

int isotp_encode_frame(FAR struct isotp_encode_s *ctx,
                       FAR uint8_t *frame, FAR uint8_t *dlen)
{
  uint8_t     max_dlen  = *dlen;
  isotp_len_t remaining = ctx->len - ctx->done;
  uint16_t    payload;

  if (max_dlen < CAN_MAX_DLEN || max_dlen > CANFD_MAX_DLEN ||
      ctx->done > ctx->len)
    {
      return -EINVAL;
    }

  if (remaining == 0)
    {
      *dlen = 0;
      return 0;
    }

  if (ctx->done == 0)
    {
      if (max_dlen > CAN_MAX_DLEN && ctx->len >= CAN_MAX_DLEN)
        {
          /* CAN FD extended SF {0x00, len, data...} for 8..max_dlen-2 bytes.
           * Only used when len > 7; shorter messages use classic SF so the
           * peer's ISO-TP socket (which checks cf->len <= 8 first) can
           * decode them correctly.
           */

          if (ctx->len <= max_dlen - 2)
            {
              frame[0] = ISOTP_MAKE_SF(0);
              frame[1] = ctx->len;
              memcpy(&frame[2], ctx->buf, ctx->len);
              ctx->done = ctx->len;
              *dlen = ctx->len + ISOTP_ESF_HDR_LEN;
              return 0;
            }
        }
      else if (ctx->len < max_dlen)
        {
          /* Classic SF {0x0N, data...} for len 1..7; used for both classic
           * CAN and short CAN FD messages so the peer decodes via the
           * classic sf_dl nibble path.
           */

          frame[0] = ISOTP_MAKE_SF(ctx->len);
          memcpy(&frame[1], ctx->buf, ctx->len);
          ctx->done = ctx->len;
          *dlen = ctx->len + 1;
          return 0;
        }

      /* First Frame (same for classic CAN and CAN FD) */

#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
      if (ctx->len > ISOTP_MAX_MSG_LEN)
        {
          /* Escaped FF: {0x10, 0x00, len[31:24..7:0]}, 32-bit FF_DL. */

          frame[0] = ISOTP_MAKE_FF(0);
          frame[1] = 0;
          frame[2] = (ctx->len >> 24) & 0xff;
          frame[3] = (ctx->len >> 16) & 0xff;
          frame[4] = (ctx->len >> 8) & 0xff;
          frame[5] = ctx->len & 0xff;
          payload  = max_dlen - ISOTP_ESC_FF_HDR_LEN;
          memcpy(&frame[ISOTP_ESC_FF_HDR_LEN], ctx->buf, payload);
        }
      else
#endif
        {
          /* Classic FF: {0x1L, LL}, 12-bit FF_DL. */

          frame[0] = ISOTP_MAKE_FF(ctx->len >> 8);
          frame[1] = ctx->len & 0xff;
          payload  = max_dlen - 2;
          memcpy(&frame[2], ctx->buf, payload);
        }

      ctx->done = payload;
      *dlen = max_dlen;
      return 1;
    }

  /* Consecutive Frame */

  frame[0] = ISOTP_MAKE_CF(ctx->sn);
  ctx->sn  = (ctx->sn + 1) & ISOTP_CF_SN_MASK;
  payload  = MIN(remaining, max_dlen - 1);
  memcpy(&frame[1], ctx->buf + ctx->done, payload);
  ctx->done += payload;

  *dlen = payload + 1;
  return ctx->done < ctx->len;
}

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
 *   >0 when more frames follow (FF or non-final CF),
 *   0 when decoding is complete (SF or last CF), or
 *   -errno on error.
 *
 ****************************************************************************/

int isotp_decode_frame(FAR struct isotp_decode_s *ctx,
                       FAR const uint8_t *frame, uint8_t dlen)
{
  uint8_t  pci_type;
  uint8_t  sf_len;
  uint16_t payload;

  if (dlen == 0 || dlen > CANFD_MAX_DLEN)
    {
      return -EINVAL;
    }

  pci_type = ISOTP_PCI_TYPE(frame[0]);
  switch (pci_type)
    {
      case ISOTP_PCI_TYPE_SF:
        {
          sf_len = frame[0] & ISOTP_SF_DL_MASK;
          if (sf_len == 0)
            {
              /* Extended SF (CAN FD): {0x00, len, data...} */

              if (dlen < 2)
                {
                  return -EINVAL;
                }

              sf_len = frame[1];
              if (sf_len == 0 || sf_len > dlen - 2)
                {
                  return -EINVAL;
                }

              if (sf_len > ctx->len)
                {
                  return -ENOBUFS;
                }

              memcpy(ctx->buf, &frame[2], sf_len);
              ctx->total = sf_len;
              ctx->done  = sf_len;
              return 0;
            }

          if (sf_len > dlen - 1)
            {
              return -EINVAL;
            }

          if (sf_len > ctx->len)
            {
              return -ENOBUFS;
            }

          memcpy(ctx->buf, &frame[1], sf_len);
          ctx->total = sf_len;
          ctx->done  = sf_len;

          return 0;
        }

      case ISOTP_PCI_TYPE_FF:
        {
          if (dlen < 2)
            {
              return -EINVAL;
            }

#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
          if ((frame[0] & ISOTP_FF_DL_MASK) == 0 && frame[1] == 0)
            {
              /* Escaped FF: 32-bit FF_DL follows the {0x10, 0x00} header. */

              if (dlen < ISOTP_ESC_FF_HDR_LEN)
                {
                  return -EINVAL;
                }

              ctx->total = ((uint32_t)frame[2] << 24) |
                           ((uint32_t)frame[3] << 16) |
                           ((uint32_t)frame[4] << 8) | frame[5];
              if (ctx->total == 0)
                {
                  return -EINVAL;
                }

              if (ctx->total > ctx->len)
                {
                  return -ENOBUFS;
                }

              payload = dlen - ISOTP_ESC_FF_HDR_LEN;
              if (ctx->total <= payload)
                {
                  return -EINVAL;
                }

              memcpy(ctx->buf, &frame[ISOTP_ESC_FF_HDR_LEN], payload);
              ctx->done = payload;

              return 1;
            }
#endif

          ctx->total = ((frame[0] & ISOTP_FF_DL_MASK) << 8) | frame[1];
          if (ctx->total == 0 || ctx->total > ctx->len)
            {
              return -ENOBUFS;
            }

          payload = dlen - 2;
          if (ctx->total <= payload)
            {
              return -EINVAL;
            }

          memcpy(ctx->buf, &frame[2], payload);
          ctx->done = payload;

          return 1;
        }

      case ISOTP_PCI_TYPE_CF:
        {
          if (dlen < 2 || ctx->done == 0 || ctx->done >= ctx->total ||
              ctx->total > ctx->len)
            {
              return -EINVAL;
            }

          if ((frame[0] & ISOTP_CF_SN_MASK) != ctx->sn)
            {
              return -EILSEQ;
            }

          ctx->sn = (ctx->sn + 1) & ISOTP_CF_SN_MASK;
          payload = MIN(dlen - 1, ctx->total - ctx->done);

          memcpy(ctx->buf + ctx->done, &frame[1], payload);
          ctx->done += payload;

          return ctx->done < ctx->total;
        }

      default:
        {
          return -EINVAL;
        }
    }
}
