/****************************************************************************
 * libs/libc/machine/arm/arm_fenv.c
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

#include <assert.h>
#include <fenv.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARM_FPSCR_TONEAREST   0x00000000
#define ARM_FPSCR_DOWNWARD    0x00400000
#define ARM_FPSCR_UPWARD      0x00800000
#define ARM_FPSCR_TOWARDZERO  0x00c00000

#define ARM_FPSCR_ROUNDMASK   0x00c00000

#define	SET_FPSCR(__r)	      __asm __volatile("vmrs %0, fpscr" : "=&r"(__r))
#define	GET_FPSCR(__r)	      __asm __volatile("vmsr fpscr, %0" : : "r"(__r))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fegetround(void)
{
  int ret = FE_TONEAREST;

#ifndef __SOFTFP__
  int fpscr = 0;

  GET_FPSCR(fpscr);

  fpscr &= ARM_FPSCR_ROUNDMASK;

  switch (fpscr)
  {
  case ARM_FPSCR_TONEAREST:
    ret = FE_TONEAREST;
    break;

  case ARM_FPSCR_DOWNWARD:
    ret = FE_DOWNWARD;
    break;

  case ARM_FPSCR_UPWARD:
    ret = FE_UPWARD;
    break;

  case ARM_FPSCR_TOWARDZERO:
    ret = FE_TOWARDZERO;
    break;

  default:
    assert(0);
    break;
  }

#endif

  return ret;
}

int fesetround(int round)
{
#ifndef __SOFTFP__
  int fpscr = 0;

  GET_FPSCR(fpscr);
  fpscr &= ~(ARM_FPSCR_ROUNDMASK);

  switch (fpscr)
  {
  case FE_TONEAREST:
    round = ARM_FPSCR_TONEAREST;
    break;

  case FE_DOWNWARD:
    round = ARM_FPSCR_DOWNWARD;
    break;

  case FE_UPWARD:
    round = ARM_FPSCR_UPWARD;
    break;

  case FE_TOWARDZERO:
    round = ARM_FPSCR_TOWARDZERO;
    break;

  default:
    assert(0);
    break;
  }

  fpscr |= round;
  SET_FPSCR(fpscr);
#endif
  return (0);
}
