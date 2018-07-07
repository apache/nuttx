/****************************************************************************
 * control/lib_transform.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Clarke transform (abc frame -> ab frame)
 *
 * Description:
 *   Transform the abc frame to the alpha-beta frame.
 *
 *   i_alpha = k*(i_a - 0.5*i_b - 0.5*i_c)
 *   i_beta  = k*sqrt(3)*0.5*(i_b - i_c)
 *
 *   We assume that:
 *     1) k = 2/3 for the non-power-invariant transformation
 *     2) balanced system: a + b + c = 0
 *
 * Input Parameters:
 *   abc - (in) pointer to the abc frame
 *   ab  - (out) pointer to the alpha-beta frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clarke_transform(FAR abc_frame_t *abc,
                      FAR ab_frame_t *ab)
{
  DEBUGASSERT(abc != NULL);
  DEBUGASSERT(ab != NULL);

  ab->a = abc->a;
  ab->b = ONE_BY_SQRT3_F*abc->a + TWO_BY_SQRT3_F*abc->b;
}

/****************************************************************************
 * Name: Inverse Clarke transform (ab frame -> abc frame)
 *
 * Description:
 *   Transform the alpha-beta frame to the abc frame.
 *
 * Input Parameters:
 *   ab  - (in) pointer to the alpha-beta frame
 *   abc - (out) pointer to the abc frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void inv_clarke_transform(FAR ab_frame_t *ab,
                          FAR abc_frame_t *abc)
{
  DEBUGASSERT(ab != NULL);
  DEBUGASSERT(abc != NULL);

  /* Assume non-power-invariant transform and balanced system */

  abc->a = ab->a;
  abc->b = -0.5f*ab->a + SQRT3_BY_TWO_F*ab->b;
  abc->c = -abc->a - abc->b;
}

/****************************************************************************
 * Name: Park transform (ab frame -> dq frame)
 *
 * Description:
 *   Transform the alpha-beta frame to the direct-quadrature frame.
 *
 * Input Parameters:
 *   angle - (in) pointer to the phase angle data
 *   ab    - (in) pointer to the alpha-beta frame
 *   dq    - (out) pointer to the direct-quadrature frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void park_transform(FAR phase_angle_t *angle,
                    FAR ab_frame_t *ab,
                    FAR dq_frame_t *dq)
{
  DEBUGASSERT(angle != NULL);
  DEBUGASSERT(ab != NULL);
  DEBUGASSERT(dq != NULL);

  dq->d = angle->cos * ab->a + angle->sin * ab->b;
  dq->q = angle->cos * ab->b - angle->sin * ab->a;
}

/****************************************************************************
 * Name: Inverse Park transform (dq frame -> ab frame)
 *
 * Description:
 *   Transform direct-quadrature frame to alpha-beta frame.
 *
 * Input Parameters:
 *   angle - (in) pointer to the phase angle data
 *   dq    - (in) pointer to the direct-quadrature frame
 *   ab    - (out) pointer to the alpha-beta frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void inv_park_transform(FAR phase_angle_t *angle,
                        FAR dq_frame_t *dq,
                        FAR ab_frame_t *ab)
{
  DEBUGASSERT(angle != NULL);
  DEBUGASSERT(dq != NULL);
  DEBUGASSERT(ab != NULL);

  ab->a = angle->cos * dq->d - angle->sin * dq->q;
  ab->b = angle->cos * dq->q + angle->sin * dq->d;
}
