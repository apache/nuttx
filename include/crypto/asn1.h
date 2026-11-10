/****************************************************************************
 * include/crypto/asn1.h
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

#ifndef __INCLUDE_CRYPTO_ASN1_H
#define __INCLUDE_CRYPTO_ASN1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <string.h>
#include <sys/endian.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ASN1_INTEGER           0x02
#define ASN1_SEQUENCE          0x10
#define ASN1_CONSTRUCTED       0x20

#define BYTE_0(x)              ((uint8_t)((x)        & 0xff))
#define BYTE_1(x)              ((uint8_t)(((x) >> 8) & 0xff))

#define ASN1_CHK_ADD(g, f)  \
  do                        \
    {                       \
      ssize_t _ret = (f);   \
      if (_ret < 0)         \
        {                   \
          return _ret;      \
        }                   \
      (g) += _ret;          \
    }                       \
  while (0)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function
ssize_t asn1_write_tag(FAR unsigned char **p,
                       FAR const unsigned char *start,
                       unsigned char tag)
{
  if (*p - start < 1)
    {
      return -EINVAL;
    }

  *--(*p) = tag;

  return 1;
}

static inline_function
ssize_t asn1_write_len(FAR unsigned char **p,
                       FAR const unsigned char *start,
                       size_t len)
{
  if (len < 0x80)
    {
      if (*p - start < 1)
        {
          return -EINVAL;
        }

      *--(*p) = (unsigned char)len;
      return 1;
    }

  if (len <= 0xff)
    {
      if (*p - start < 2)
        {
          return -EINVAL;
        }

      *--(*p) = (unsigned char)len;
      *--(*p) = 0x81;
      return 2;
    }

  if (len <= 0xffff)
    {
      if (*p - start < 3)
        {
          return -EINVAL;
        }

      *--(*p) = BYTE_0(len);
      *--(*p) = BYTE_1(len);
      *--(*p) = 0x82;
      return 3;
    }

  return -EINVAL;
}

static inline_function
ssize_t asn1_write_int(FAR unsigned char **p,
                       FAR const unsigned char *start,
                       int val)
{
  ssize_t len = 0;

  do
    {
      if (*p - start < 1)
        {
          return -EINVAL;
        }

      len += 1;
      *--(*p) = val & 0xff;
      val >>= 8;
    }
  while (val > 0);

  if (**p & 0x80)
    {
      if (*p - start < 1)
        {
          return -EINVAL;
        }

      *--(*p) = 0x00;
      len += 1;
    }

  ASN1_CHK_ADD(len, asn1_write_len(p, start, len));
  ASN1_CHK_ADD(len, asn1_write_tag(p, start, ASN1_INTEGER));
  return len;
}

static inline_function
ssize_t asn1_write_mpi(FAR unsigned char **p,
                       FAR const unsigned char *start,
                       FAR unsigned char *mpi, size_t mpilen)
{
  ssize_t len = mpilen;
  size_t i;

  /* DER represents 0 with a sign bit (0=nonnegative) and 7 value bits, not
   * as 0 digits. We need to end up with 020100, not with 0200.
   */

  if (len == 0)
    {
      len = 1;
    }

  if (*p < start || (size_t)(*p - start) < (size_t)len)
    {
      return -ENOMEM;
    }

  (*p) -= len;

  if (mpilen == 0)
    {
      (*p)[0] = 0x00;
    }
  else
    {
      /* convert to big-endian */

      for (i = 0; i < mpilen; i++)
        {
          (*p)[mpilen - i - 1] = mpi[i];
        }
    }

  if (**p & 0x80)
    {
      if (*p - start < 1)
        {
          return -ENOMEM;
        }

      (*p)--;
      *(*p) = 0x00;
      len++;
    }

  ASN1_CHK_ADD(len, asn1_write_len(p, start, len));
  ASN1_CHK_ADD(len, asn1_write_tag(p, start, ASN1_INTEGER));
  return len;
}

static inline_function
int asn1_get_len(FAR unsigned char **p,
                 FAR const unsigned char *end,
                 FAR size_t *len)
{
  if ((end - *p) < 1)
    {
      return -ENOMEM;
    }

  if ((**p & 0x80) == 0)
    {
      *len = *(*p)++;
    }
  else
    {
      switch (**p & 0x7f)
        {
          case 1:
            if ((end - *p) < 2)
              {
                return -ENOMEM;
              }

            *len = (*p)[1];
            (*p) += 2;
            break;
          case 2:
            if ((end - *p) < 3)
              {
                return -ENOMEM;
              }

            *len = ((size_t) (*p)[1] << 8) | (*p)[2];
            (*p) += 3;
            break;
          case 3:
            if ((end - *p) < 4)
              {
                return -ENOMEM;
              }

            *len = ((size_t) (*p)[1] << 16) |
                    ((size_t) (*p)[2] << 8) | (*p)[3];
            (*p) += 4;
            break;
          case 4:
            if ((end - *p) < 5)
              {
                return -ENOMEM;
              }

            *len = ((size_t) (*p)[1] << 24) | ((size_t) (*p)[2] << 16) |
                    ((size_t) (*p)[3] << 8) |           (*p)[4];
            (*p) += 5;
            break;
          default:
            return -EINVAL;
        }
    }

  if (*len > (size_t)(end - *p))
    {
      return -ENOMEM;
    }

  return 0;
}

static inline_function
int asn1_get_tag(FAR unsigned char **p,
                 FAR const unsigned char *end,
                 FAR size_t *len, int tag)
{
  if ((end - *p) < 1)
    {
      return -ENOMEM;
    }

  if (**p != tag)
    {
      return -EINVAL;
    }

  (*p)++;
  return asn1_get_len(p, end, len);
}

static inline_function
int asn1_get_int(FAR unsigned char **p,
                 FAR const unsigned char *end,
                 FAR int *val)
{
  size_t len;
  int ret;

  ret = asn1_get_tag(p, end, &len, ASN1_INTEGER);
  if (ret != 0)
    {
      return ret;
    }

  /* len==0 is malformed (0 must be represented as 020100 for INTEGER,
   * or 0A0100 for ENUMERATED tags)
   */

  if (len == 0)
    {
      return -EINVAL;
    }

  /* This is a cryptography library. Reject negative integers. */

  if ((**p & 0x80) != 0)
    {
      return -EINVAL;
    }

  /* Skip leading zeros. */

  while (len > 0 && **p == 0)
    {
      ++(*p);
      --len;
    }

  /* Reject integers that don't fit in an int. This code assumes that
   * the int type has no padding bit.
   */

  if (len > sizeof(int))
    {
      return -EINVAL;
    }

  if (len == sizeof(int) && (**p & 0x80) != 0)
    {
      return -EINVAL;
    }

  *val = 0;
  while (len-- > 0)
    {
      *val = (*val << 8) | **p;
      (*p)++;
    }

  return 0;
}

static inline_function
int asn1_get_mpi(FAR unsigned char **p,
                 FAR const unsigned char *end,
                 FAR unsigned char *mpi, size_t mpilen)
{
  uint32_t *left;
  uint32_t *right;
  uint32_t tmp;
  size_t overhead;
  size_t len;
  int ret;

  ret = asn1_get_tag(p, end, &len, ASN1_INTEGER);
  if (ret != 0)
    {
      return ret;
    }

  memset(mpi, 0, mpilen);

  if (len != 0)
    {
      overhead = mpilen - len;
      memcpy(mpi + overhead, *p, len);
    }

  for (left = (FAR uint32_t *)mpi,
       right = (FAR uint32_t *)(mpi + mpilen - sizeof(*right));
       left <= right; left++, right--)
    {
      tmp    = betoh32(*left);
      *left  = betoh32(*right);
      *right = tmp;
    }

  *p += len;

  return ret;
}

#endif /* __INCLUDE_CRYPTO_ASN1_H */
