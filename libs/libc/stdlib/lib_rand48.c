/****************************************************************************
 * libs/libc/stdlib/lib_rand48.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>

#include <nuttx/lib/lib.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned short g_seed48[7] =
{
  0,
  0,
  0,
  0xe66d,
  0xdeec,
  0x5,
  0xb
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t rand48_step(FAR unsigned short *xi, FAR unsigned short *lc)
{
  uint64_t a, x;

  x = xi[0] | (xi[1] + (0u << 16)) | (xi[2] + (0ull << 32));
  a = lc[0] | (lc[1] + (0u << 16)) | (lc[2] + (0ull << 32));
  x = a * x + lc[3];

  xi[0] = x;
  xi[1] = x >> 16;
  xi[2] = x >> 32;
  return x & 0xffffffffffffull;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: srand48
 ****************************************************************************/

void srand48(long seed)
{
  unsigned short p[3];

  p[0] = 0x330e;
  p[1] = seed;
  p[2] = seed >> 16;
  seed48(p);
}

/****************************************************************************
 * Name: seed48
 ****************************************************************************/

FAR unsigned short *seed48(unsigned short seed16v[3])
{
  static unsigned short p[3];

  memcpy(p, g_seed48, sizeof p);
  memcpy(g_seed48, seed16v, sizeof p);
  return p;
}

/****************************************************************************
 * Name: lcong48
 ****************************************************************************/

void lcong48(unsigned short p[7])
{
  memcpy(g_seed48, p, sizeof(g_seed48));
}


/****************************************************************************
 * Name: jrand48
 *
 * Description:
 *   Return signed long integers uniformly distributed over the
 *   interval [-2^31, 2^31).
 *
 ****************************************************************************/

long jrand48(unsigned short s[3])
{
  return (long)(rand48_step(s, g_seed48 + 3) >> 16);
}

/****************************************************************************
 * Name: mrand48
 *
 * Description:
 *   Return signed long integers uniformly distributed over the
 *   interval [-2^31, 2^31).
 *
 ****************************************************************************/

long mrand48(void)
{
  return jrand48(g_seed48);
}

/****************************************************************************
 * Name: nrand48
 *
 * Description:
 *   Return nonnegative long integers  uniformly  distributed over the
 *   interval [0, 2^31).
 *
 ****************************************************************************/

long nrand48(unsigned short s[3])
{
  return rand48_step(s, g_seed48 + 3) >> 17;
}

/****************************************************************************
 * Name: lrand48
 *
 * Description:
 *   Return nonnegative long integers  uniformly  distributed over the
 *   interval [0, 2^31).
 *
 ****************************************************************************/

long lrand48(void)
{
  return nrand48(g_seed48);
}

/****************************************************************************
 * Name: erand48
 *
 * Description:
 *   Return nonnegative double-precision floating-point values uniformly
 *   distributed over the interval [0.0, 1.0).
 *
 ****************************************************************************/

double erand48(unsigned short s[3])
{
  union
    {
      uint64_t u;
      double f;
    } x;

  x.u = 0x3ff0000000000000ull | rand48_step(s, g_seed48 + 3) << 4;
  return x.f - 1.0;
}

/****************************************************************************
 * Name: drand48
 *
 * Description:
 *   Return nonnegative double-precision floating-point values uniformly
 *   distributed over the interval [0.0, 1.0).
 *
 ****************************************************************************/

double drand48(void)
{
  return erand48(g_seed48);
}
