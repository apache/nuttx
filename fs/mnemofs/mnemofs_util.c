/****************************************************************************
 * fs/mnemofs/mnemofs_util.c
 *
 * SPDX-License-Identifier: Apache-2.0 or BSD-3-Clause
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
 * Alternatively, the contents of this file may be used under the terms of
 * the BSD-3-Clause license:
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024 Saurav Pal
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "mnemofs.h"

#include <sys/param.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint8_t mfs_arrhash(FAR const char *arr, ssize_t len)
{
  ssize_t  l    = 0;
  ssize_t  r    = len - 1;
  uint16_t hash = 0;

  /* TODO: Change the array checksum to be 16 bit long. */

  while (l <= r)
    {
      hash += arr[l] * arr[r] * (l + 1) * (r + 1);
      l++;
      r--;
      hash %= (1 << 8);
    }

  finfo("Hash calculated for size %zd to be %d.", len, hash % (1 << 8));

  return hash % (1 << 8);
}

uint16_t mfs_hash(FAR const char *arr, ssize_t len)
{
  ssize_t  l    = 0;
  ssize_t  r    = len - 1;
  uint32_t hash = 0;

  /* TODO: Change the array checksum to be 16 bit long. */

  while (l <= r)
    {
      hash += arr[l] * arr[r] * (l + 1) * (r + 1);
      l++;
      r--;
      hash %= (1 << MFS_HASHSZ);
    }

  finfo("Hash calculated for size %zd to be %" PRIi32, len, hash);

  return hash;
}

FAR char *mfs_ser_8(const uint8_t n, FAR char * const out)
{
  *out = n;
  return out + 1;
}

FAR const char *mfs_deser_8(FAR const char * const in, FAR uint8_t *n)
{
  *n = in[0];
  return in + 1;
}

FAR char *mfs_ser_16(const uint16_t n, FAR char * const out)
{
  memcpy(out, &n, 2);
  return out + 2;
}

FAR const char *mfs_deser_16(FAR const char * const in, FAR uint16_t *n)
{
  memcpy(n, in, 2);
  return in + 2;
}

FAR char *mfs_ser_str(FAR const char * const str, const mfs_t len,
                      FAR char * const out)
{
  memcpy(out, str, len);
  return out + len;
}

FAR const char *mfs_deser_str(FAR const char * const in,
                              FAR char * const str, const mfs_t len)
{
  memcpy(str, in, len);
  str[len] = 0;
  return in + len;
}

FAR char *mfs_ser_mfs(const mfs_t n, FAR char * const out)
{
  memcpy(out, &n, 4);
  return out + 4;
}

FAR const char *mfs_deser_mfs(FAR const char * const in, FAR mfs_t * const n)
{
  memcpy(n, in, 4);
  return in + 4;
}

FAR char *mfs_ser_64(const uint64_t n, FAR char * const out)
{
  memcpy(out, &n, 8);
  return out + 8;
}

FAR const char *mfs_deser_64(FAR const char * const in,
                            FAR uint64_t * const n)
{
  memcpy(n, in, 8);
  return in + 8;
}

FAR char *mfs_ser_ctz(FAR const struct mfs_ctz_s * const x,
                      FAR char * const out)
{
  FAR char *o = out;

  o = mfs_ser_mfs(x->pg_e, o);
  o = mfs_ser_mfs(x->idx_e, o);
  return o;
}

FAR const char *mfs_deser_ctz(FAR const char * const in,
                              FAR struct mfs_ctz_s * const x)
{
  FAR const char *i = in;

  i = mfs_deser_mfs(i, &x->pg_e);
  i = mfs_deser_mfs(i, &x->idx_e);
  return i;
}

FAR char *mfs_ser_path(FAR const struct mfs_path_s * const x,
                      FAR char * const out)
{
  FAR char *o = out;

  o = mfs_ser_ctz(&x->ctz, o);
  o = mfs_ser_mfs(x->off, o);
  o = mfs_ser_mfs(x->sz, o);
  return o;
}

FAR const char *mfs_deser_path(FAR const char * const in,
                               FAR struct mfs_path_s * const x)
{
  FAR const char *i = in;

  i = mfs_deser_ctz(i, &x->ctz);
  i = mfs_deser_mfs(i, &x->off);
  i = mfs_deser_mfs(i, &x->sz);
  return i;
}

FAR char *mfs_ser_timespec(FAR const struct timespec * const x,
                           FAR char * const out)
{
  FAR char *o = out;

  o = mfs_ser_64(x->tv_sec, o);
  o = mfs_ser_64(x->tv_nsec, o);

  return o;
}

FAR const char *mfs_deser_timespec(FAR const char * const in,
                                   FAR struct timespec * const x)
{
  uint64_t tmp;
  FAR const char *i = in;

  i = mfs_deser_64(i, &tmp);
  x->tv_sec = tmp;
  i = mfs_deser_64(i, &tmp);
  x->tv_nsec = tmp;

  return i;
}

mfs_t mfs_v2n(mfs_t n)
{
  /* https://math.stackexchange.com/a/1835555 */

  return (n & (~(n - 1)));
}

mfs_t mfs_set_msb(mfs_t n)
{
  return 31 - mfs_clz(n);
}

bool mfs_ctz_eq(FAR const struct mfs_ctz_s * const a,
                FAR const struct mfs_ctz_s * const b)
{
  return a->idx_e == b->idx_e && a->pg_e == b->pg_e;
}

bool mfs_path_eq(FAR const struct mfs_path_s * const a,
                 FAR const struct mfs_path_s * const b)
{
  return mfs_ctz_eq(&a->ctz, &b->ctz) && (a->off == b->off)
         && (a->sz == b->sz);
}
