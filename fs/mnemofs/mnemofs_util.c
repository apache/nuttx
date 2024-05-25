/****************************************************************************
 * fs/mnemofs/mnemofs_util.c
 * Utilities for mnemofs
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

#include "mnemofs.h"

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

char mfs_arrhash(FAR const char *arr, ssize_t len)
{
  ssize_t l = 0;
  ssize_t r = len - 1;
  uint16_t hash = 0;

  while (l <= r)
    {
      hash += arr[l] * arr[r] * l * r;
      l++;
      r--;
      hash %= (1 << 8);
    }

  return hash % (1 << 8);
}

char *mfs_ser_8(const char n, char * const out)
{
  memcpy(out, &n, sizeof(n));
  return out + sizeof(n);
}

const char *mfs_deser_8(const char * const in, char *n)
{
  memcpy(&n, in, sizeof(n));
  return in + sizeof(n);
}

char *mfs_ser_str(const char * const str, const mfs_t len, char * const out)
{
  memcpy(out, str, len);
  return out + len;
}

const char *mfs_deser_str(const char * const in, char * const str,
                          const mfs_t len)
{
  memcpy(str, in, len);
  return in + len;
}

char *mfs_ser_mfs(const mfs_t n, char * const out)
{
  memcpy(out, &n, sizeof(n));
  return out + sizeof(n);
}

const char *mfs_deser_mfs(const char * const in, mfs_t * const n)
{
  memcpy(n, in, sizeof(*n));
  return in + sizeof(n);
}

char *mfs_ser_ctz_store(const struct mfs_ctz_store_s * const x,
                        char * const out)
{
  char *o = out;

  o = mfs_ser_mfs(x->pg_e, o);
  o = mfs_ser_mfs(x->idx_e, o);
  return o;
}

const char *mfs_deser_ctz_store(const char * const in,
                                struct mfs_ctz_store_s * const x)
{
  const char *i = in;

  i = mfs_deser_mfs(i, &x->pg_e);
  i = mfs_deser_mfs(i, &x->idx_e);
  return i;
}
