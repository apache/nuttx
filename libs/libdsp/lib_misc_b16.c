/****************************************************************************
 * libs/libdsp/lib_misc_b16.c
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

#include <dspb16.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VECTOR2D_SATURATE_MAG_MIN (1)
#define FAST_ATAN2_SMALLNUM       (1)

#ifndef ABS
#  define ABS(a)   ((a) < 0 ? -(a) : (a))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: f_saturate_b16
 *
 * Description:
 *   Saturate b16_t number
 *
 * Input Parameters:
 *   val - pointer to b16_t number
 *   min - lower limit
 *   max - upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void f_saturate_b16(FAR b16_t *val, b16_t min, b16_t max)
{
  if (*val < min)
    {
      *val = min;
    }

  else if (*val > max)
    {
      *val = max;
    }
}

/****************************************************************************
 * Name: vector2d_mag_b16
 *
 * Description:
 *   Get 2D vector magnitude.
 *
 * Input Parameters:
 *   x   - (in) vector x component
 *   y   - (in) vector y component
 *
 * Returned Value:
 *   Return 2D vector magnitude
 *
 ****************************************************************************/

b16_t vector2d_mag_b16(b16_t x, b16_t y)
{
  b16_t t0 = 0;
  b16_t t1 = 0;
  b16_t t3 = 0;

  t0 = b16sqr(x);
  t1 = b16sqr(y);
  t3 = t0 + t1;

  /* TODO: move to fixedmath sqrt */

  if (t3 == 0)
    {
      return 0;
    }

#if CONFIG_LIBDSP_PRECISION == 0
  /* Use ub8 sqrt */

  return ub8toub16(ub16sqrtub8(t3));
#else
  /* Too slow ! */

  return ub16sqrtub16(t3);
#endif
}

/****************************************************************************
 * Name: vector2d_saturate_b16
 *
 * Description:
 *   Saturate 2D vector magnitude.
 *
 * Input Parameters:
 *   x   - (in/out) pointer to the vector x component
 *   y   - (in/out) pointer to the vector y component
 *   max - (in) maximum vector magnitude
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void vector2d_saturate_b16(FAR b16_t *x, FAR b16_t *y, b16_t max)
{
  b16_t mag = 0;
  b16_t tmp = 0;

  /* Get vector magnitude */

  mag = vector2d_mag_b16(*x, *y);

  if (mag < VECTOR2D_SATURATE_MAG_MIN)
    {
      mag = VECTOR2D_SATURATE_MAG_MIN;
    }

  if (mag > max)
    {
      /* Saturate vector */

      tmp = b16divb16(max, mag);
      *x  = b16mulb16(*x, tmp);
      *y  = b16mulb16(*x, tmp);
    }
}

/****************************************************************************
 * Name: dq_mag_b16
 *
 * Description:
 *   Get DQ vector magnitude.
 *
 * Input Parameters:
 *   dq  - (in/out) dq frame vector
 *
 * Returned Value:
 *  Return dq vector magnitude
 *
 ****************************************************************************/

b16_t dq_mag_b16(FAR dq_frame_b16_t *dq)
{
  return vector2d_mag_b16(dq->d, dq->q);
}

/****************************************************************************
 * Name: dq_saturate_b16
 *
 * Description:
 *   Saturate dq frame vector magnitude.
 *
 * Input Parameters:
 *   dq  - (in/out) dq frame vector
 *   max - (in) maximum vector magnitude
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void dq_saturate_b16(FAR dq_frame_b16_t *dq, b16_t max)
{
  vector2d_saturate_b16(&dq->d, &dq->q, max);
}

/****************************************************************************
 * Name: fast_sin_b16
 *
 * Description:
 *   Fast sin calculation
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated sine value
 *
 ****************************************************************************/

b16_t fast_sin_b16(b16_t angle)
{
  b16_t sin  = 0;
  b16_t n1   = ftob16(1.27323954f);
  b16_t n2   = ftob16(0.405284735f);
  b16_t tmp1 = 0;
  b16_t tmp2 = 0;

  /* Normalize angle */

  angle_norm_2pi_b16(&angle, -b16PI, b16PI);

  /* Get estiamte sine value from quadratic equation */

  if (angle < 0)
    {
      tmp1 = b16mulb16(n1, angle);
      tmp2 = b16mulb16(n2, b16sqr(angle));
      sin  = tmp1 + tmp2;
    }
  else
    {
      tmp1 = b16mulb16(n1, angle);
      tmp2 = b16mulb16(n2, b16sqr(angle));
      sin  = tmp1 - tmp2;
    }

  return sin;
}

/****************************************************************************
 * Name:fast_cos_b16
 *
 * Description:
 *   Fast cos calculation
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated cosine value
 *
 ****************************************************************************/

b16_t fast_cos_b16(b16_t angle)
{
  /* Get cosine value from sine sin(x + PI/2) = cos(x)  */

  return fast_sin_b16(angle + b16HALFPI);
}

/****************************************************************************
 * Name: fast_sin2_b16
 *
 * Description:
 *   Fast sin calculation with better accuracy (quadratic curve
 *   approximation)
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated sine value
 *
 ****************************************************************************/

b16_t fast_sin2_b16(b16_t angle)
{
  return b16sin(angle);
}

/****************************************************************************
 * Name:fast_cos2_b16
 *
 * Description:
 *   Fast cos calculation with better accuracy (quadratic curve
 *   approximation)
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated cosine value
 *
 ****************************************************************************/

b16_t fast_cos2_b16(b16_t angle)
{
  return b16cos(angle);
}

/****************************************************************************
 * Name: fast_atan2_b16
 *
 * Description:
 *   Fast atan2 calculation
 *
 * REFERENCE:
 * https://dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization/
 *
 * Input Parameters:
 *   x - (in)
 *   y - (in)
 *
 * Returned Value:
 *   Return estimated angle
 *
 ****************************************************************************/

b16_t fast_atan2_b16(b16_t y, b16_t x)
{
  b16_t angle = 0;
  b16_t abs_y = 0;
  b16_t rsq   = 0;
  b16_t r     = 0;
  b16_t n1    = ftob16(0.1963f);
  b16_t n2    = ftob16(0.9817f);
  b16_t tmp1 = 0;
  b16_t tmp2 = 0;
  b16_t tmp3 = 0;

  /* Get absolute value of y and add some small number to prevent 0/0 */

  abs_y = ABS(y) + FAST_ATAN2_SMALLNUM;

  /* Calculate angle */

  if (x >= 0)
    {
      r     = b16divb16((x - abs_y), (x + abs_y));
      rsq   = b16mulb16(r, r);
      tmp1  = b16mulb16(n1, rsq);
      tmp2  = b16mulb16((tmp1 - n2), r);
      tmp3  = b16mulb16(b16PI, ftob16(0.25f));
      angle = tmp2 + tmp3;
    }
  else
    {
      r     = b16divb16((x + abs_y), (abs_y - x));
      rsq   = b16mulb16(r, r);
      tmp1  = b16mulb16(n1, rsq);
      tmp2  = b16mulb16((tmp1 - n2), r);
      tmp3  = b16mulb16(b16PI, ftob16(0.75f));
      angle = tmp2 + tmp3;
    }

  /* Get angle sign */

  if (y < 0)
    {
      angle = -angle;
    }

  return angle;
}

/****************************************************************************
 * Name: angle_norm_b16
 *
 * Description:
 *   Normalize radians angle to a given boundary and a given period.
 *
 * Input Parameters:
 *   angle  - (in/out) pointer to the angle data
 *   per    - (in) angle period
 *   bottom - (in) lower limit
 *   top    - (in) upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void angle_norm_b16(FAR b16_t *angle, b16_t per, b16_t bottom, b16_t top)
{
  while (*angle > top)
    {
      /* Move the angle backwards by given period */

      *angle = *angle - per;
    }

  while (*angle < bottom)
    {
      /* Move the angle forwards by given period */

      *angle = *angle + per;
    }
}

/****************************************************************************
 * Name: angle_norm_2pi_b16
 *
 * Description:
 *   Normalize radians angle with period 2*PI to a given boundary.
 *
 * Input Parameters:
 *   angle  - (in/out) pointer to the angle data
 *   bottom - (in) lower limit
 *   top    - (in) upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void angle_norm_2pi_b16(FAR b16_t *angle, b16_t bottom, b16_t top)
{
  angle_norm_b16(angle, b16TWOPI, bottom, top);
}

/****************************************************************************
 * Name: phase_angle_update_b16
 *
 * Description:
 *   Update phase_angle_s structure:
 *     1. normalize angle value to <0.0, 2PI> range
 *     2. update angle value
 *     3. update sin/cos value for given angle
 *
 * Input Parameters:
 *   angle - (in/out) pointer to the angle data
 *   val   - (in) angle radian value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void phase_angle_update_b16(FAR struct phase_angle_b16_s *angle, b16_t val)
{
  LIBDSP_DEBUGASSERT(angle != NULL);

  /* Normalize angle to <0.0, 2PI> */

  angle_norm_2pi_b16(&val, 0, b16TWOPI);

  /* Update structure */

  angle->angle = val;

#if CONFIG_LIBDSP_PRECISION == 0
  angle->sin = fast_sin_b16(val);
  angle->cos = fast_cos_b16(val);
#else
  angle->sin = fast_sin2_b16(val);
  angle->cos = fast_cos2_b16(val);
#endif
}
