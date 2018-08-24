/****************************************************************************
 * control/lib_svm.c
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

#include <assert.h>

#include <dsp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: svm3_sector_get
 *
 * Description:
 *   Get current sector for space vector modulation.
 *
 * Input Parameters:
 *   ijk - (in) pointer to the auxiliary ABC frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static uint8_t svm3_sector_get(FAR abc_frame_t *ijk)
{
  uint8_t sector = 0;
  float i = ijk->a;
  float j = ijk->b;
  float k = ijk->c;

  /* Identify the correct sector based on i,j,k frame:
   * 1. sector 1:
   *              i > 0.0
   *              j > 0.0
   *              k <= 0.0
   * 2. sector 2:
   *              i <= 0.0
   *              j >  0.0
   *              k <= 0.0
   * 3. sector 3:
   *              i <= 0.0
   *              j >  0.0
   *              k >  0.0
   * 4. sector 4:
   *              i <= 0.0
   *              j <= 0.0
   *              k >  0.0
   * 5. sector 5:
   *              i >  0.0
   *              j <= 0.0
   *              k >  0.0
   * 6. sector 6:
   *              i >  0.0
   *              j <=  0.0
   *              k <=  0.0
   */

  if (k <= 0.0)
    {
      if (i <= 0.0)
        {
          sector = 2;
        }
      else
        {
          if (j <= 0.0)
            {
              sector = 6;
            }
          else
            {
              sector = 1;
            }
        }
    }
  else
    {
      if (i <= 0.0)
        {
          if (j <= 0.0)
            {
              sector = 4;
            }
          else
            {
              sector = 3;
            }
        }
      else
        {
          sector = 5;
        }
    }

  /* Return SVM sector */

  return sector;
}

/****************************************************************************
 * Name: svm3_duty_calc
 *
 * Description:
 *   Calculate duty cycles for space vector modulation.
 *
 * Input Parameters:
 *   s   - (in/out) pointer to the SVM state data
 *   ijk - (in) pointer to the auxiliary ABC frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void svm3_duty_calc(FAR struct svm3_state_s *s, FAR abc_frame_t *ijk)
{
  float i = ijk->a;
  float j = ijk->b;
  float k = ijk->c;
  float T0 = 0.0f;
  float T1 = 0.0f;
  float T2 = 0.0f;

  /* Determine T1, T2 and T0 based on the sector */

  switch (s->sector)
    {
      case 1:
        {
          T1 = i;
          T2 = j;
          break;
        }
      case 2:
        {
          T1 = -k;
          T2 = -i;
          break;
        }
      case 3:
        {
          T1 = j;
          T2 = k;
          break;
        }
      case 4:
        {
          T1 = -i;
          T2 = -j;
          break;
        }
      case 5:
        {
          T1 = k;
          T2 = i;
          break;
        }
      case 6:
        {
          T1 = -j;
          T2 = -k;
          break;
        }
      default:
        {
          /* We should not get here */

          DEBUGASSERT(0);
          break;
        }
    }

  /* Get null vector time */

  T0 = 1.0f - T1 - T2;

  /* Calculate duty cycle for 3 phase */

  switch (s->sector)
    {
      case 1:
        {
          s->d_u = T1 + T2 + T0*0.5f;
          s->d_v = T2 + T0*0.5f;
          s->d_w = T0*0.5f;
          break;
        }
      case 2:
        {
          s->d_u = T1 + T0*0.5f;
          s->d_v = T1 + T2 + T0*0.5f;
          s->d_w = T0*0.5f;
          break;
        }
      case 3:
        {
          s->d_u = T0*0.5f;
          s->d_v = T1 + T2 + T0*0.5f;
          s->d_w = T2 + T0*0.5f;
          break;
        }
      case 4:
        {
          s->d_u = T0*0.5f;
          s->d_v = T1 + T0*0.5f;
          s->d_w = T1 + T2 + T0*0.5f;
          break;
        }
      case 5:
        {
          s->d_u = T2 + T0*0.5f;
          s->d_v = T0*0.5f;
          s->d_w = T1 + T2 + T0*0.5f;
          break;
        }
      case 6:
        {
          s->d_u = T1 + T2 + T0*0.5f;
          s->d_v = T0*0.5f;
          s->d_w = T1 + T0*0.5f;
          break;
        }
      default:
        {
          /* We should not get here */

          DEBUGASSERT(0);
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: svm3
 *
 * Description:
 *   One step of the space vector modulation.
 *   This is most common of SVM with alternate-reverse null vector.
 *
 *   Voltage vector definitions in 3-phase SVM:
 *
 *  |---------|-----------|--------------------|-----------------|
 *  | Voltage | swithcing | Line to neutral    | Line to line    |
 *  | vector  | vectors   | voltage            | voltage         |
 *  |         |-----------|--------------------|-----------------|
 *  |         | a | b | c | Van  | Vbn  | Vcn  | Vab | Vbe | Vca |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V0      | 0 | 0 | 0 |  0   |  0   |  0   |  0  |  0  |  0  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V1      | 1 | 0 | 0 |  2/3 | -1/3 | -1/3 |  1  |  0  | -1  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V2      | 1 | 1 | 0 |  1/3 |  1/3 | -2/3 |  0  |  1  | -1  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V3      | 0 | 1 | 0 | -1/3 |  2/3 | -1/3 | -1  |  1  |  0  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V4      | 0 | 1 | 1 | -2/3 |  1/3 |  1/3 | -1  |  0  |  1  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V5      | 0 | 0 | 1 | -1/3 | -1/3 |  2/3 |  0  | -1  |  1  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V6      | 1 | 0 | 1 |  1/3 | -2/3 |  1/3 |  1  | -1  |  0  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *  | V7      | 1 | 1 | 1 |  0   |  0   |  0   |  0  |  0  |  0  |
 *  |---------|---|---|---|------|------|------|-----|-----|-----|
 *
 *   Voltage values given in relation to the bus voltage (Vbus)/
 *
 * Input Parameters:
 *   s    - (out) pointer to the SVM data
 *   v_ab - (in) pointer to the modulation voltage vector in alpha-beta frame,
 *          normalized to magnitude (0.0 - 1.0)
 *
 * NOTE: v_ab vector magnitude must be in range <0.0, 1.0> to get correct
 *       SVM3 results.
 *
 * REVISIT: not sure how we should handle invalid data from user.
 *          For now we saturate output duty form SVM.
 *
 * REFERENCE:
 *   https://e2e.ti.com/group/motor/m/pdf_presentations/665547/download (32-34)
 *
 ****************************************************************************/

void svm3(FAR struct svm3_state_s *s, FAR ab_frame_t *v_ab)
{
  DEBUGASSERT(s != NULL);
  DEBUGASSERT(v_ab != NULL);

  abc_frame_t ijk;

  /* Perform modified inverse Clarke-transformation (alpha,beta) -> (i,j,k)
   * to obtain auxiliary frame which will be used in further calculations.
   */

  ijk.a = -0.5f*v_ab->b + SQRT3_BY_TWO_F*v_ab->a;
  ijk.b = v_ab->b;
  ijk.c = -ijk.b - ijk.a;

  /* Get vector sector */

  s->sector = svm3_sector_get(&ijk);

  /* Get duty cycle */

  svm3_duty_calc(s, &ijk);

  /* Saturate output from SVM */

  f_saturate(&s->d_u, s->d_min, s->d_max);
  f_saturate(&s->d_v, s->d_min, s->d_max);
  f_saturate(&s->d_w, s->d_min, s->d_max);
}

/****************************************************************************
 * Name: svm3_current_correct
 *
 * Description:
 *   Correct ADC samples (int32) according to SVM3 state.
 *   NOTE: This works only with 3 shunt resistors configuration.
 *
 ****************************************************************************/

void svm3_current_correct(FAR struct svm3_state_s *s,
                              int32_t *c0, int32_t *c1, int32_t *c2)
{
  /* Get best ADC samples according to SVM sector.
   *
   * In SVM phase current can be sampled only in v0 vector state, when lower
   * bridge transistors are turned on.
   *
   * We ignore sample from phase which has the shortest V0 state and
   * estimate its value with KCL for motor phases:
   *    i_a + i_b + i_c = 0
   */

  switch (s->sector)
    {
      case 1:
      case 6:
        {
          /* Sector 1-6: ignore phase 1 */

          *c0 = -(*c1 + *c2);

          break;
        }

      case 2:
      case 3:
        {
          /* Sector 2-3: ignore phase 2 */

          *c1 = -(*c0 + *c2);

          break;
        }

      case 4:
      case 5:
        {
          /* Sector 4-5: ignore phase 3 */

          *c2 = -(*c0 + *c1);

          break;
        }

      default:
        {
          /* We should not get here. */

          *c0 = 0;
          *c1 = 0;
          *c2 = 0;

          break;
        }
    }
}

/****************************************************************************
 * Name: svm3_init
 *
 * Description:
 *   Initialize 3-phase SVM data.
 *
 * Input Parameters:
 *   s - (in/out) pointer to the SVM state data
 *   sat - (in)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void svm3_init(FAR struct svm3_state_s *s, float min, float max)
{
  DEBUGASSERT(s != NULL);
  DEBUGASSERT(max > min);

  memset(s, 0, sizeof(struct svm3_state_s));

  s->d_max = max;
  s->d_min = min;
}
