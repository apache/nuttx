/****************************************************************************
 * control/lib_misc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dsp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: f_saturate
 *
 * Description:
 *   Saturate float number
 *
 * Input Parameters:
 *   val - pointer to float number
 *   min - lower limit
 *   max - upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void f_saturate(FAR float *val, float min, float max)
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
 * Name: vector2d_mag
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

float vector2d_mag(float x, float y)
{
  return sqrtf(x * x + y * y);
}

/****************************************************************************
 * Name: vector2d_saturate
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

void vector2d_saturate(FAR float *x, FAR float *y, float max)
{
  float mag = 0.0f;
  float tmp = 0.0f;

  /* Get vector magnitude */

  mag = vector2d_mag(*x, *y);

  if (mag < 1e-10f)
    {
      mag = 1e-10f;
    }

  if (mag > max)
    {
      /* Saturate vector */

      tmp = max / mag;
      *x *= tmp;
      *y *= tmp;
    }
}

/****************************************************************************
 * Name: dq_mag
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

float dq_mag(FAR dq_frame_t *dq)
{
  return vector2d_mag(dq->d, dq->q);
}

/****************************************************************************
 * Name: dq_saturate
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

void dq_saturate(FAR dq_frame_t *dq, float max)
{
  vector2d_saturate(&dq->d, &dq->q, max);
}

/****************************************************************************
 * Name: fast_sin
 *
 * Description:
 *   Fast sin calculation
 *
 *   Reference: http://lab.polygonal.de/?p=205
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated sine value
 *
 ****************************************************************************/

float fast_sin(float angle)
{
  float sin = 0.0f;
  float n1  = 1.27323954f;
  float n2  = 0.405284735f;

  /* Normalize angle */

  angle_norm_2pi(&angle, -M_PI_F, M_PI_F);

  /* Get estiamte sine value from quadratic equation */

  if (angle < 0.0f)
    {
      sin = n1 * angle + n2 * angle * angle;
    }
  else
    {
      sin = n1 * angle - n2 * angle * angle;
    }

  return sin;
}

/****************************************************************************
 * Name:fast_cos
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

float fast_cos(float angle)
{
  /* Get cosine value from sine sin(x + PI/2) = cos(x)  */

  return fast_sin(angle + M_PI_2_F);
}

/****************************************************************************
 * Name: fast_sin2
 *
 * Description:
 *   Fast sin calculation with better accuracy
 *
 *   Reference: http://lab.polygonal.de/?p=205
 *
 * Input Parameters:
 *   angle
 *
 * Returned Value:
 *   Return estimated sine value
 *
 ****************************************************************************/

float fast_sin2(float angle)
{
  float sin = 0.0f;
  float n1 = 1.27323954f;
  float n2 = 0.405284735f;
  float n3 = 0.225f;

  /* Normalize angle */

  angle_norm_2pi(&angle, -M_PI_F, M_PI_F);

  /* Get estiamte sine value from quadratic equation and do more */

  if (angle < 0.0f)
    {
      sin = n1 * angle + n2 * angle * angle;

      if (sin < 0.0f)
        {
          sin = n3 * (sin *(-sin) - sin) + sin;
        }
      else
        {
          sin = n3 * (sin * sin - sin) + sin;
        }
    }
  else
    {
      sin = n1 * angle - n2 * angle * angle;

      if (sin < 0.0f)
        {
          sin = n3 * (sin *(-sin) - sin) + sin;
        }
      else
        {
          sin = n3 * (sin * sin - sin) + sin;
        }
    }

  return sin;
}

/****************************************************************************
 * Name:fast_cos2
 *
 * Description:
 *   Fast cos calculation with better accuracy
 *
 * Input Parameters:
 *   angle - (in)
 *
 * Returned Value:
 *   Return estimated cosine value
 *
 ****************************************************************************/

float fast_cos2(float angle)
{
  /* Get cosine value from sine sin(x + PI/2) = cos(x)  */

  return fast_sin2(angle + M_PI_2_F);
}

/****************************************************************************
 * Name: fast_atan2
 *
 * Description:
 *   Fast atan2 calculation
 *
 * REFERENCE:
 *   https://dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization/
 *
 * Input Parameters:
 *   x - (in)
 *   y - (in)
 *
 * Returned Value:
 *   Return estimated angle
 *
 ****************************************************************************/

float fast_atan2(float y, float x)
{
  float angle = 0.0f;
  float abs_y = 0.0f;
  float rsq   = 0.0f;
  float r     = 0.0f;
  float n1    = 0.1963f;
  float n2    = 0.9817f;

  /* Get absolute value of y and add some small number to prevent 0/0 */

  abs_y = fabsf(y)+1e-10f;

  /* Calculate angle */

  if (x >= 0.0f)
    {
      r = (x - abs_y) / (x + abs_y);
      rsq = r * r;
      angle = ((n1 * rsq) - n2) * r + (M_PI_F / 4.0f);
    }
  else
    {
      r = (x + abs_y) / (abs_y - x);
      rsq = r * r;
      angle = ((n1 * rsq) - n2) * r + (3.0f * M_PI_F / 4.0f);
    }

  /* Get angle sign */

  if (y < 0.0f)
    {
      angle = -angle;
    }
  else
    {
      angle = angle;
    }

  return angle;
}

/****************************************************************************
 * Name: angle_norm
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

void angle_norm(FAR float *angle, float per, float bottom, float top)
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
 * Name: angle_norm_2pi
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

void angle_norm_2pi(FAR float *angle, float bottom, float top)
{
  angle_norm(angle, 2.0f*M_PI_F, bottom, top);
}

/****************************************************************************
 * Name: phase_angle_update
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

void phase_angle_update(FAR struct phase_angle_s *angle, float val)
{
  DEBUGASSERT(angle != NULL);

  /* Normalize angle to <0.0, 2PI> */

  angle_norm_2pi(&val, 0.0f, 2.0f*M_PI_F);

  /* Update structure */

  angle->angle = val;

#if CONFIG_LIBDSP_PRECISION == 1
  angle->sin = fast_sin2(val);
  angle->cos = fast_cos2(val);
#elif CONFIG_LIBDSP_PRECISION == 2
  angle->sin = sin(val);
  angle->cos = cos(val);
#else
  angle->sin = fast_sin(val);
  angle->cos = fast_cos(val);
#endif
}
