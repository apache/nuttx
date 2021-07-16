/****************************************************************************
 * libs/libdsp/lib_svm_b16.c
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

#include <dspb16.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: svm3_sector_get_b16
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

static uint8_t svm3_sector_get_b16(FAR abc_frame_b16_t *ijk)
{
  uint8_t sector = 0;
  b16_t   i      = ijk->a;
  b16_t   j      = ijk->b;
  b16_t   k      = ijk->c;

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

  if (k <= 0)
    {
      if (i <= 0)
        {
          sector = 2;
        }
      else
        {
          if (j <= 0)
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
      if (i <= 0)
        {
          if (j <= 0)
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
 * Name: svm3_duty_calc_b16
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

static void svm3_duty_calc_b16(FAR struct svm3_state_b16_s *s,
                               FAR abc_frame_b16_t *ijk)
{
  b16_t i  = ijk->a;
  b16_t j  = ijk->b;
  b16_t k  = ijk->c;
  b16_t T0 = 0;
  b16_t T1 = 0;
  b16_t T2 = 0;

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

          LIBDSP_DEBUGASSERT(0);
          break;
        }
    }

  /* Get null vector time */

  T0 = b16ONE - T1 - T2;

  /* Calculate duty cycle for 3 phase */

  switch (s->sector)
    {
      case 1:
        {
          s->d_u = T1 + T2 + b16mulb16(T0, b16HALF);
          s->d_v = T2 + b16mulb16(T0, b16HALF);
          s->d_w = b16mulb16(T0, b16HALF);
          break;
        }

      case 2:
        {
          s->d_u = T1 + b16mulb16(T0, b16HALF);
          s->d_v = T1 + T2 + b16mulb16(T0, b16HALF);
          s->d_w = b16mulb16(T0, b16HALF);
          break;
        }

      case 3:
        {
          s->d_u = b16mulb16(T0, b16HALF);
          s->d_v = T1 + T2 + b16mulb16(T0, b16HALF);
          s->d_w = T2 + b16mulb16(T0, b16HALF);
          break;
        }

      case 4:
        {
          s->d_u = b16mulb16(T0, b16HALF);
          s->d_v = T1 + b16mulb16(T0, b16HALF);
          s->d_w = T1 + T2 + b16mulb16(T0, b16HALF);
          break;
        }

      case 5:
        {
          s->d_u = T2 + b16mulb16(T0, b16HALF);
          s->d_v = b16mulb16(T0, b16HALF);
          s->d_w = T1 + T2 + b16mulb16(T0, b16HALF);
          break;
        }

      case 6:
        {
          s->d_u = T1 + T2 + b16mulb16(T0, b16HALF);
          s->d_v = b16mulb16(T0, b16HALF);
          s->d_w = T1 + b16mulb16(T0, b16HALF);
          break;
        }

      default:
        {
          /* We should not get here */

          LIBDSP_DEBUGASSERT(0);
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: svm3_b16
 *
 * Description:
 *   One step of the space vector modulation.
 *   This is most common of SVM with alternate-reverse null vector.
 *
 *   Voltage vector definitions in 3-phase SVM:
 *
 *  |---------|-----------|--------------------|-----------------|
 *  | Voltage | switching | Line to neutral    | Line to line    |
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
 *   v_ab - (in) pointer to the modulation voltage vector in alpha-beta
 *          frame, normalized to magnitude (0.0 - 1.0)
 *
 * NOTE: v_ab vector magnitude must be in range <0.0, 1.0> to get correct
 *       SVM3 results.
 *
 * REFERENCE:
 *   https://e2e.ti.com/group/motor/m/pdf_presentations/665547/download
 *     pages 32-34
 *
 ****************************************************************************/

void svm3_b16(FAR struct svm3_state_b16_s *s, FAR ab_frame_b16_t *v_ab)
{
  LIBDSP_DEBUGASSERT(s != NULL);
  LIBDSP_DEBUGASSERT(v_ab != NULL);

  abc_frame_b16_t ijk;

  /* Perform modified inverse Clarke-transformation (alpha,beta) -> (i,j,k)
   * to obtain auxiliary frame which will be used in further calculations.
   */

  ijk.a = b16mulb16(-b16HALF, v_ab->b) + b16mulb16(SQRT3_BY_TWO_B16,
                                                   v_ab->a);
  ijk.b = v_ab->b;
  ijk.c = -ijk.b - ijk.a;

  /* Get vector sector */

  s->sector = svm3_sector_get_b16(&ijk);

  /* Get duty cycle */

  svm3_duty_calc_b16(s, &ijk);

  /* NOTE: we return not-saturated output. Duty-cycle saturation is
   *       board-specific characteristic and we have not access to this
   *       information here.
   */
}

/****************************************************************************
 * Name: svm3_current_correct_b16
 *
 * Description:
 *   Correct ADC samples (int32) according to SVM3 state.
 *   NOTE: This works only with 3 shunt resistors configuration.
 *
 ****************************************************************************/

void svm3_current_correct_b16(FAR struct svm3_state_b16_s *s,
                              b16_t *c0, b16_t *c1, b16_t *c2)
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
 * Name: svm3_init_b16
 *
 * Description:
 *   Initialize 3-phase SVM data.
 *
 * Input Parameters:
 *   s - (in/out) pointer to the SVM state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void svm3_init_b16(FAR struct svm3_state_b16_s *s)
{
  LIBDSP_DEBUGASSERT(s != NULL);

  memset(s, 0, sizeof(struct svm3_state_b16_s));
}
