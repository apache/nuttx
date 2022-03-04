/****************************************************************************
 * libs/libdsp/lib_motor_b16.c
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
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POLE_CNTR_THR (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_openloop_init_b16
 *
 * Description:
 *  Initialize open-loop data
 *
 * Input Parameters:
 *   op  - (in/out) pointer to the openloop data structure
 *   per - (in) period of the open-loop control
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_openloop_init_b16(FAR struct openloop_data_b16_s *op, b16_t per)
{
  LIBDSP_DEBUGASSERT(op != NULL);
  LIBDSP_DEBUGASSERT(per > 0);

  /* Reset openloop structure */

  memset(op, 0, sizeof(struct openloop_data_b16_s));

  /* Initialize data */

  op->per = per;
}

/****************************************************************************
 * Name: motor_openloop_b16
 *
 * Description:
 *   One step of the open-loop control
 *
 * Input Parameters:
 *   op    - (in/out) pointer to the open-loop data structure
 *   speed - (in) open-loop speed
 *   dir   - (in) rotation direction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_openloop_b16(FAR struct openloop_data_b16_s *op, b16_t speed,
                        b16_t dir)
{
  LIBDSP_DEBUGASSERT(op != NULL);
  LIBDSP_DEBUGASSERT(speed >= 0);
  LIBDSP_DEBUGASSERT(dir == DIR_CW_B16 || dir == DIR_CCW_B16);

  b16_t phase_step = 0;
  b16_t tmp = 0;

  /* Get phase step */

  tmp = b16mulb16(dir, speed);
  phase_step =  b16mulb16(tmp, op->per);

  /* Update open-loop angle */

  op->angle += phase_step;

  /* Normalize the open-loop angle to 0.0 - 2PI range */

  angle_norm_2pi_b16(&op->angle, MOTOR_ANGLE_E_MIN_B16,
                     MOTOR_ANGLE_E_MAX_B16);
}

/****************************************************************************
 * Name: motor_openloop_angle_get_b16
 *
 * Description:
 *   Get angle from open-loop controller
 *
 * Input Parameters:
 *   op    - (in/out) pointer to the open-loop data structure
 *
 * Returned Value:
 *   Return angle from open-loop controller
 *
 ****************************************************************************/

b16_t motor_openloop_angle_get_b16(FAR struct openloop_data_b16_s *op)
{
  LIBDSP_DEBUGASSERT(op != NULL);

  return op->angle;
}

/****************************************************************************
 * Name: motor_angle_e_update_b16
 *
 * Description:
 *   Update motor angle structure using electrical motor angle.
 *
 * Input Parameters:
 *   angle     - (in/out) pointer to the motor angle structure
 *   angle_new - (in) new motor electrical angle in range <0.0, 2PI>
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_angle_e_update_b16(FAR struct motor_angle_b16_s *angle,
                              b16_t angle_new, b16_t dir)
{
  LIBDSP_DEBUGASSERT(angle != NULL);
  LIBDSP_DEBUGASSERT(angle_new >= 0 && angle_new <= MOTOR_ANGLE_E_MAX_B16);
  LIBDSP_DEBUGASSERT(dir == DIR_CW_B16 || dir == DIR_CCW_B16);

  b16_t tmp1 = 0;
  b16_t tmp2 = 0;

  /* Check if we crossed electrical angle boundaries */

  if (dir == DIR_CW_B16)
    {
      /* For CW direction - previous angle is greater than current angle */

      if (angle_new - angle->angle_el.angle < -POLE_CNTR_THR)
        {
          angle->i += 1;
        }
    }

  else if (dir == DIR_CCW_B16)
    {
      /* For CCW direction - previous angle is lower than current angle */

      if (angle_new - angle->angle_el.angle > POLE_CNTR_THR)
        {
          angle->i -= 1;
        }
    }

  /* Reset pole counter if needed */

  if (angle->i >= angle->p)
    {
      angle->i = 0;
    }

  else if (angle->i < 0)
    {
      angle->i = angle->p - 1;
    }

  /* Update electrical angle structure */

  phase_angle_update_b16(&angle->angle_el, angle_new);

  /* Calculate mechanical angle.
   * One electrical angle rotation is equal to one mechanical rotation
   * divided by number of motor pole pairs.
   */

  tmp1 = b16mulb16(MOTOR_ANGLE_E_RANGE_B16, itob16(angle->i));
  tmp2 = tmp1 + angle->angle_el.angle;

  angle->anglem = b16mulb16(tmp2, angle->one_by_p);

  /* Normalize mechanical angle to <0, 2PI> and store */

  angle_norm_2pi_b16(&angle->anglem, MOTOR_ANGLE_M_MIN_B16,
                     MOTOR_ANGLE_M_MAX_B16);
}

/****************************************************************************
 * Name: motor_angle_m_update_b16
 *
 * Description:
 *   Update motor angle structure using mechanical motor angle
 *
 * Input Parameters:
 *   angle     - (in/out) pointer to the motor angle structure
 *   angle_new - (in) new motor mechanical angle in range <0.0, 2PI>
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_angle_m_update_b16(FAR struct motor_angle_b16_s *angle,
                              b16_t angle_new, b16_t dir)
{
  LIBDSP_DEBUGASSERT(angle != NULL);
  LIBDSP_DEBUGASSERT(angle_new >= 0 && angle_new <= MOTOR_ANGLE_E_MAX_B16);
  LIBDSP_DEBUGASSERT(dir == DIR_CW_B16 || dir == DIR_CCW_B16);

  b16_t angle_el = 0;
  b16_t tmp1     = 0;
  b16_t tmp2     = 0;

  /* Store new mechanical angle */

  angle->anglem = angle_new;

  /* Update pole counter */

  tmp1 = b16mulb16(angle->anglem, itob16(angle->p));

  angle->i = (uint8_t) b16toi(b16divb16(tmp1, MOTOR_ANGLE_M_MAX_B16));

  /* Get electrical angle */

  tmp1 = b16mulb16(angle->anglem, itob16(angle->p));
  tmp2 = b16mulb16(MOTOR_ANGLE_E_MAX_B16, itob16(angle->i));

  angle_el = (tmp1 - tmp2);

  /* Update electrical angle structure */

  phase_angle_update_b16(&angle->angle_el, angle_el);
}

/****************************************************************************
 * Name: motor_angle_m_get_b16
 *
 * Description:
 *   Get motor mechanical angle
 *
 * Input Parameters:
 *   angle - (in/out) pointer to the motor angle structure
 *
 * Returned Value:
 *   Return motor mechanical angle
 *
 ****************************************************************************/

b16_t motor_angle_m_get_b16(FAR struct motor_angle_b16_s *angle)
{
  LIBDSP_DEBUGASSERT(angle != NULL);

  return angle->anglem;
}

/****************************************************************************
 * Name: motor_angle_e_get_b16
 *
 * Description:
 *   Get motor electrical angle
 *
 * Input Parameters:
 *   angle - (in/out) pointer to the motor angle structure
 *
 * Returned Value:
 *   Return motor electrical angle
 *
 ****************************************************************************/

b16_t motor_angle_e_get_b16(FAR struct motor_angle_b16_s *angle)
{
  LIBDSP_DEBUGASSERT(angle != NULL);

  return angle->angle_el.angle;
}

/****************************************************************************
 * Name: motor_phy_params_init_b16
 *
 * Description:
 *   Initialize motor physical parameters
 *
 * Input Parameters:
 *   phy   - (in/out) pointer to the motor physical parameters
 *   poles - (in) number of the motor pole pairs
 *   res   - (in) average phase-to-neutral base motor resistance
 *                (without temperature compensation)
 *   ind   - (in) average phase-to-neutral motor inductance
 *   flux  - (in) flux linkage
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_phy_params_init_b16(FAR struct motor_phy_params_b16_s *phy,
                               uint8_t poles, b16_t res, b16_t ind,
                               b16_t flux_link)
{
  LIBDSP_DEBUGASSERT(phy != NULL);

  memset(phy, 0, sizeof(struct motor_phy_params_b16_s));

  phy->p          = poles;
  phy->flux_link  = flux_link;
  phy->res        = res;
  phy->ind        = ind;
  phy->one_by_ind = b16divb16(b16ONE, ind);
  phy->one_by_p   = b16divb16(b16ONE, poles);
}

/****************************************************************************
 * Name: pmsm_phy_params_init_b16
 *
 * Description:
 *   Initialize PMSM physical parameters
 *
 * Input Parameters:
 *   phy   - (in/out) pointer to the PMSM physical parameters
 *   poles - (in) number of the motor pole pairs
 *   res   - (in) average phase-to-neutral base motor resistance
 *                (without temperature compensation)
 *   ind   - (in) average phase-to-neutral motor inductance
 *   iner  - (in) rotor inertia (J)
 *   flux  - (in) flux linkage
 *   ind_d - (in) d-inductance
 *   ind_q - (in) q-inductance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pmsm_phy_params_init_b16(FAR struct pmsm_phy_params_b16_s *phy,
                              uint8_t poles, b16_t res, b16_t ind,
                              b16_t iner, b16_t flux,
                              b16_t ind_d, b16_t ind_q)
{
  LIBDSP_DEBUGASSERT(phy != NULL);

  /* Initialize motor phy */

  motor_phy_params_init_b16(&phy->motor, poles, res, ind, flux);

  /* Iniitalize PMSM specific data */

  phy->iner        = iner;
  phy->ind_d       = ind_d;
  phy->ind_q       = ind_q;
  phy->one_by_iner = b16divb16(b16ONE, iner);
  phy->one_by_indd = b16divb16(b16ONE, ind_d);
  phy->one_by_indq = b16divb16(b16ONE, ind_q);
}
