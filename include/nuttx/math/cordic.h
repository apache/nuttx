/****************************************************************************
 * include/nuttx/math/cordic.h
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

#ifndef __INCLUDE_NUTTX_MATH_CORDIC_H
#define __INCLUDE_NUTTX_MATH_CORDIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MATH_CORDIC_USE_Q31

/* q1.31 conversion */

#  define ftoq31(f32_val)   ((f32_val) * 0x80000000)
#  define q31tof(q31_val)   ((float)(q31_val) / (float)0x80000000)
#  define b16toq31(b16_val) ((b16_val) * 0x8000)
#  define q31tob16(q31_val) ((q31_val) / 0x8000)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_MATH_CORDIC_USE_Q31
/* q1.31 as int32_t */

typedef int32_t cordic_num_t;
#endif

/* CORDIC functions */

enum cordic_calc_func_e
{
  CORDIC_CALC_FUNC_INVAL    = 0,
  CORDIC_CALC_FUNC_COS      = 1,  /* Primary result: cosine * m,
                                   * secondary result: sine * m,
                                   * primary argument: angle,
                                   * secondary argument: modulus m
                                   */
  CORDIC_CALC_FUNC_SIN      = 2,  /* Primary result: sine * m,
                                   * secondary result: cosine * m,
                                   * primary argument: angle,
                                   * secondary argument: modulus m
                                   */
  CORDIC_CALC_FUNC_PHASE    = 3,  /* Primary result: phase (atan2),
                                   * secondary result: modulus,
                                   * primary argument: x,
                                   * secondary argument: y
                                   */
  CORDIC_CALC_FUNC_MOD      = 4,  /* Primary result: modulus,
                                   * secondary result: phase (atan2),
                                   * primary argument: x,
                                   * secondary argument: y
                                   */
  CORDIC_CALC_FUNC_ARCTAN   = 5,  /* Primary result: arctangent,
                                   * secondary result: none,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_HCOS     = 6,  /* Primary result: hyperbolic cosine,
                                   * secondary result: hyperbolic sine,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_HSIN     = 7,  /* Primary result: hyperbolic sine,
                                   * secondary result: hyperbolic cosine,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_HARCTAN  = 8,  /* Primary result: hyperbolic arctangent,
                                   * secondary result: none,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_LN       = 9,  /* Primary result: natural logarithm,
                                   * secondary result: none,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_SQRT     = 10, /* Primary result: square root,
                                   * secondary result: none,
                                   * primary argument: x,
                                   * secondary argument: none
                                   */
  CORDIC_CALC_FUNC_LAST
};

/* CORDIC calculate request */

struct cordic_calc_s
{
  /* CORDIC request configuration */

  uint8_t func;                 /* CORDIC function */
  bool    res2_incl;            /* Include secondary result if available */

  /* Input data */

  cordic_num_t arg1;
  cordic_num_t arg2;

  /* Output data */

  cordic_num_t res1;
  cordic_num_t res2;
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct cordic_lowerhalf_s;
struct cordic_ops_s
{
  /* CORDIC calculate request */

  CODE int (*calc)(FAR struct cordic_lowerhalf_s *lower,
                   FAR struct cordic_calc_s *calc);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct cordic_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct cordic_ops_s  *ops;  /* Lower half operations */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: cordic_register
 *
 * Description:
 *   Register a CORDIC driver.
 *
 ****************************************************************************/

int cordic_register(FAR const char *path,
                    FAR struct cordic_lowerhalf_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MATH_CORDIC_H */
