/****************************************************************************
 * arch/arm/src/stm32/stm32_cordic.c
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
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/math/cordic.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/stm32g4xxxx_cordic.h"

#include "stm32_cordic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_CORDIC_PRECISION (3)
#define STM32_CORDIC_ARGSIZE   (0) /* Argument size is 32-bit */
#define STM32_CORDIC_RESSIZE   (0) /* Result size is 32-bit */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct stm32_cordic_s
{
  const struct cordic_ops_s *ops;   /* Lower half operations */
  uint32_t                   base;  /* The base address of the CORDIC */
  bool                       inuse; /* True: driver is in-use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t cordic_getreg(struct stm32_cordic_s *priv, int offset);
static void cordic_putreg(struct stm32_cordic_s *priv, int offset,
                          uint32_t value);

/* Ops */

int cordic_calc(struct cordic_lowerhalf_s *lower,
                struct cordic_calc_s *calc);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* STM32 specific CORDIC ops */

struct cordic_ops_s g_stm32_cordic_ops =
{
  .calc = cordic_calc
};

/* STM32 CORDIC device */

struct stm32_cordic_s g_stm32_cordic_dev =
{
  .ops   = &g_stm32_cordic_ops,
  .base  = STM32_CORDIC_BASE,
  .inuse = false
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cordic_getreg
 ****************************************************************************/

static uint32_t cordic_getreg(struct stm32_cordic_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: cordic_putreg
 ****************************************************************************/

static void cordic_putreg(struct stm32_cordic_s *priv, int offset,
                          uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: cordic_calc
 ****************************************************************************/

int cordic_calc(struct cordic_lowerhalf_s *lower,
                struct cordic_calc_s *calc)
{
  struct stm32_cordic_s *priv     = (struct stm32_cordic_s *)lower;
  int                    ret      = OK;
  uint32_t               csr      = 0;
  bool                   arg2_inc = false;
  uint8_t                scale    = 0;

  DEBUGASSERT(lower);
  DEBUGASSERT(calc);

  /* Configure CORDIC function */

  switch (calc->func)
    {
      case CORDIC_CALC_FUNC_COS:
        {
          csr |= CORDIC_CSR_FUNC_COS;
          arg2_inc = true;
          scale = 0;
          break;
        }

      case CORDIC_CALC_FUNC_SIN:
        {
          csr |= CORDIC_CSR_FUNC_SIN;
          arg2_inc = true;
          scale = 0;
          break;
        }

      case CORDIC_CALC_FUNC_PHASE:
        {
          csr |= CORDIC_CSR_FUNC_PHASE;
          arg2_inc = true;
          scale = 0;
          break;
        }

      case CORDIC_CALC_FUNC_MOD:
        {
          csr |= CORDIC_CSR_FUNC_MOD;
          arg2_inc = true;
          scale = 0;
          break;
        }

      case CORDIC_CALC_FUNC_ARCTAN:
        {
          csr |= CORDIC_CSR_FUNC_ARCTAN;
          arg2_inc = true;
          scale = 0;
          break;
        }

      case CORDIC_CALC_FUNC_HCOS:
        {
          csr |= CORDIC_CSR_FUNC_HCOS;
          arg2_inc = false;
          scale = 1;
          break;
        }

      case CORDIC_CALC_FUNC_HSIN:
        {
          csr |= CORDIC_CSR_FUNC_HSIN;
          arg2_inc = false;
          scale = 1;
          break;
        }

      case CORDIC_CALC_FUNC_HARCTAN:
        {
          csr |= CORDIC_CSR_FUNC_HARCTAN;
          arg2_inc = false;
          scale = 1;
          break;
        }

      case CORDIC_CALC_FUNC_LN:
        {
          csr |= CORDIC_CSR_FUNC_LN;
          arg2_inc = false;
          scale = 1;
          break;
        }

      case CORDIC_CALC_FUNC_SQRT:
        {
          csr |= CORDIC_CSR_FUNC_SQRT;
          arg2_inc = false;
          scale = 1;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure precision */

  csr |= ((STM32_CORDIC_PRECISION << CORDIC_CSR_PRECISION_SHIFT) &
          CORDIC_CSR_PRECISION_MASK);

  /* Configure scale */

  csr |= ((scale << CORDIC_CSR_SCALE_SHIFT) & CORDIC_CSR_SCALE_MASK);

  /* Configure width of output data */

  csr |= STM32_CORDIC_RESSIZE;

  /* Configure width of input data */

  csr |= STM32_CORDIC_ARGSIZE;

  /* Include secondary argument */

  if (arg2_inc == true)
    {
      csr |= CORDIC_CSR_NARGS;
    }

  /* Include secondary result */

  if (calc->res2_incl == true)
    {
      csr |= CORDIC_CSR_NRES;
    }

  /* Write CSR */

  cordic_putreg(priv, STM32_CORDIC_CSR_OFFSET, csr);

  /* Write arguments */

  cordic_putreg(priv, STM32_CORDIC_WDATA_OFFSET, calc->arg1);

  if (arg2_inc == true)
    {
      cordic_putreg(priv, STM32_CORDIC_WDATA_OFFSET, calc->arg2);
    }

  /* Read results - blocking.
   * NOTE: We don't need to wait for RRDY flag as wait sates are
   *       inserted automatically on RDATA read.
   */

  calc->res1 = cordic_getreg(priv, STM32_CORDIC_RDATA_OFFSET);

  if (calc->res2_incl == true)
    {
      calc->res2 = cordic_getreg(priv, STM32_CORDIC_RDATA_OFFSET);
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
 * Name: stm32_cordicinitialize
 *
 * Description:
 *   Initialize a CORDIC device.  This function must be called
 *   from board-specific logic.
 *
 * Returned Value:
 *   On success, a pointer to the lower half CORDIC driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cordic_lowerhalf_s *stm32_cordicinitialize(void)
{
  struct cordic_lowerhalf_s *lower = NULL;

  if (g_stm32_cordic_dev.inuse == true)
    {
      _err("STM32 CORDIC device already in use\n");
      set_errno(EBUSY);
      goto errout;
    }

  /* Get lower-half device */

  lower = (struct cordic_lowerhalf_s *) &g_stm32_cordic_dev;

  /* The driver is now in-use */

  g_stm32_cordic_dev.inuse = true;

errout:
  return lower;
}
