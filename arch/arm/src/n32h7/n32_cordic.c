/****************************************************************************
 * arch/arm/src/n32h7/n32_cordic.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <nuttx/math/cordic.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/n32h7_cordic.h"
#include "n32_cordic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fixed precision (iterations = PRECISION * 4) */
#define N32_CORDIC_PRECISION    (6)

/* Use 32-bit Q1.31 format for input/output */
#define N32_CORDIC_INSIZE       (0)   /* 32-bit */
#define N32_CORDIC_OUTSIZE      (0)   /* 32-bit */
#define N32_CORDIC_FLOATIN      (0)   /* Fixed point */
#define N32_CORDIC_FLOATOUT     (0)   /* Fixed point */
#define N32_CORDIC_PHASELIMIT   (0)   /* Disable phase limit */
#define N32_CORDIC_CODINLIMIT   (0)   /* Disable coordinate limit */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n32_cordic_s
{
  const struct cordic_ops_s *ops;   /* Lower half operations */
  uint32_t                   base;  /* Base address of the CORDIC */
  bool                       inuse; /* True: driver is in-use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t cordic_getreg(struct n32_cordic_s *priv, int offset);
static void     cordic_putreg(struct n32_cordic_s *priv, int offset,
                              uint32_t value);
static int      cordic_calc(struct cordic_lowerhalf_s *lower,
                            struct cordic_calc_s *calc);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cordic_ops_s g_n32_cordic_ops =
{
  .calc = cordic_calc,
};

static struct n32_cordic_s g_n32_cordic_dev =
{
  .ops   = &g_n32_cordic_ops,
  .base  = N32_CORDIC_BASE,
  .inuse = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cordic_getreg
 ****************************************************************************/

static uint32_t cordic_getreg(struct n32_cordic_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: cordic_putreg
 ****************************************************************************/

static void cordic_putreg(struct n32_cordic_s *priv, int offset,
                          uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: cordic_calc
 ****************************************************************************/

static int cordic_calc(struct cordic_lowerhalf_s *lower,
                       struct cordic_calc_s *calc)
{
  struct n32_cordic_s *priv = (struct n32_cordic_s *)lower;
  int ret = OK;
  uint32_t ctrl = 0;
  bool arg2_inc = false;
  uint8_t scale = 0;

  DEBUGASSERT(lower && calc);

  /* Select function and determine number of arguments and scale factor */

  switch (calc->func)
    {
      case CORDIC_CALC_FUNC_COS:
        ctrl |= CORDIC_CTRLSTS_FUNC_COS;
        arg2_inc = true;
        scale = 0;
        break;

      case CORDIC_CALC_FUNC_SIN:
        ctrl |= CORDIC_CTRLSTS_FUNC_SIN;
        arg2_inc = true;
        scale = 0;
        break;

      case CORDIC_CALC_FUNC_PHASE:
        ctrl |= CORDIC_CTRLSTS_FUNC_PHASE;
        arg2_inc = true;
        scale = 0;
        break;

      case CORDIC_CALC_FUNC_MOD:
        ctrl |= CORDIC_CTRLSTS_FUNC_MODULUS;
        arg2_inc = true;
        scale = 0;
        break;

      case CORDIC_CALC_FUNC_ARCTAN:
        ctrl |= CORDIC_CTRLSTS_FUNC_ARCTAN;
        arg2_inc = true;
        scale = 0;
        break;

      case CORDIC_CALC_FUNC_HCOS:
        ctrl |= CORDIC_CTRLSTS_FUNC_HB_COS;
        arg2_inc = false;
        scale = 1;
        break;

      case CORDIC_CALC_FUNC_HSIN:
        ctrl |= CORDIC_CTRLSTS_FUNC_HB_SIN;
        arg2_inc = false;
        scale = 1;
        break;

      case CORDIC_CALC_FUNC_HARCTAN:
        ctrl |= CORDIC_CTRLSTS_FUNC_HB_ARCT;
        arg2_inc = false;
        scale = 1;
        break;

      case CORDIC_CALC_FUNC_LN:
        ctrl |= CORDIC_CTRLSTS_FUNC_NATL;
        arg2_inc = false;
        scale = 1;
        break;

      case CORDIC_CALC_FUNC_SQRT:
        ctrl |= CORDIC_CTRLSTS_FUNC_SQRT;
        arg2_inc = false;
        scale = 1;
        break;

      default:
        ret = -EINVAL;
        goto errout;
    }

  /* Set precision (iteration count = PRECISION * 4) */

  ctrl |= ((N32_CORDIC_PRECISION << CORDIC_CTRLSTS_PRECISION_SHIFT) &
           CORDIC_CTRLSTS_PRECISION_MASK);

  /* Set scale factor */

  ctrl |= ((scale << CORDIC_CTRLSTS_SCALE_SHIFT) &
           CORDIC_CTRLSTS_SCALE_MASK);

  /* Input/output width: fixed 32-bit */

  if (N32_CORDIC_INSIZE)
    {
      ctrl |= CORDIC_CTRLSTS_INSIZE;
    }

  if (N32_CORDIC_OUTSIZE)
    {
      ctrl |= CORDIC_CTRLSTS_OUTSIZE;
    }

  /* Floating point disabled (Q31) */

  if (N32_CORDIC_FLOATIN)
    {
      ctrl |= CORDIC_CTRLSTS_FLOATIN;
    }

  if (N32_CORDIC_FLOATOUT)
    {
      ctrl |= CORDIC_CTRLSTS_FLOATOUT;
    }

  /* Limit controls disabled */

  if (N32_CORDIC_PHASELIMIT)
    {
      ctrl |= CORDIC_CTRLSTS_PHASELIMIT;
    }

  if (N32_CORDIC_CODINLIMIT)
    {
      ctrl |= CORDIC_CTRLSTS_CODINLIMIT;
    }

  /* Number of arguments */

  if (arg2_inc)
    {
      ctrl |= CORDIC_CTRLSTS_NUMWRITE;
    }

  /* Number of results */

  if (calc->res2_incl)
    {
      ctrl |= CORDIC_CTRLSTS_NUMREAD;
    }

  /* Disable interrupts and DMA */

  /* (INTEN, DMAREN, DMAWEN remain 0) */

  /* Write control register */

  cordic_putreg(priv, N32_CORDIC_CTRLSTS_OFFSET, ctrl);

  /* Write arguments */

  cordic_putreg(priv, N32_CORDIC_WDATA_OFFSET, calc->arg1);
  if (arg2_inc)
    {
      cordic_putreg(priv, N32_CORDIC_WDATA_OFFSET, calc->arg2);
    }

  /* Read results. Hardware inserts wait states if RRF is not set. */

  calc->res1 = cordic_getreg(priv, N32_CORDIC_RDATA_OFFSET);
  if (calc->res2_incl)
    {
      calc->res2 = cordic_getreg(priv, N32_CORDIC_RDATA_OFFSET);
    }
  else
    {
      calc->res2 = 0;
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_cordicinitialize
 ****************************************************************************/

struct cordic_lowerhalf_s *n32_cordicinitialize(void)
{
  struct cordic_lowerhalf_s *lower = NULL;

  if (g_n32_cordic_dev.inuse)
    {
      _err("N32 CORDIC already in use\n");
      set_errno(EBUSY);
      goto errout;
    }

  lower = (struct cordic_lowerhalf_s *)&g_n32_cordic_dev;
  g_n32_cordic_dev.inuse = true;

errout:
  return lower;
}
