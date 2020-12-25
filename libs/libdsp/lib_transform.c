/****************************************************************************
 * control/lib_transform.c
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

#include <dsp.h>

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
