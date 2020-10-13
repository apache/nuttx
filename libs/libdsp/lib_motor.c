/****************************************************************************
 * control/lib_motor.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define POLE_CNTR_THR (0.0f)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_openloop_init
 *
 * Description:
 *  Initialize open-loop data
 *
 * Input Parameters:
 *   op  - (in/out) pointer to the openloop data structure
 *   max - (in) maximum openloop speed
 *   per - (in) period of the open-loop control
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_openloop_init(FAR struct openloop_data_s *op, float max,
                         float per)
{
  DEBUGASSERT(op != NULL);
  DEBUGASSERT(max > 0.0f);
  DEBUGASSERT(per > 0.0f);

  /* Reset openloop structure */

  memset(op, 0, sizeof(struct openloop_data_s));

  /* Initialize data */

  op->max = max;
  op->per = per;
}

/****************************************************************************
 * Name: motor_openloop
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

void motor_openloop(FAR struct openloop_data_s *op, float speed, float dir)
{
  DEBUGASSERT(op != NULL);
  DEBUGASSERT(speed >= 0.0f);
  DEBUGASSERT(dir == DIR_CW || dir == DIR_CCW);

  float phase_step = 0.0f;

  /* REVISIT: what should we do if speed is greater than max open-loop speed?
   *          Saturate speed or stop motor ?
   */

  if (speed > op->max)
    {
      speed = op->max;
    }

  /* Get phase step */

  phase_step = dir * speed * op->per;

  /* Update open-loop angle */

  op->angle += phase_step;

  /* Normalize the open-loop angle to 0.0 - 2PI range */

  angle_norm_2pi(&op->angle, MOTOR_ANGLE_E_MIN, MOTOR_ANGLE_E_MAX);
}

/****************************************************************************
 * Name: motor_openloop_angle_get
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

float motor_openloop_angle_get(FAR struct openloop_data_s *op)
{
  DEBUGASSERT(op != NULL);

  return op->angle;
}

/* In a multipolar electrical machines, mechanical angle is not equal
 * to mechanical angle which can be described as:
 *
 *   electrical angle = (p) * mechanical angle
 *   where p - number of poles pair
 *
 *
 * electrical
 * angle:
 *
 * poles = 4
 * i =                0    1    2    3    0    1    2    3    0
 *            2PI  ______________________________________________
 *                    /|   /|   /|   /|   /|   /|   /|   /|   /|
 *                   / |  / |  / |  / |  / |  / |  / |  / |  / |
 *                  /  | /  | /  | /  | /  | /  | /  | /  | /  |
 *            0    /___|/___|/___|/___|/___|/___|/___|/___|/___|/
 *                 .                  .                   .
 * mechanical      .                  .                   .
 * angle:          .                  .                   .
 *                 .                  .                   .
 *            2PI  .__________________.___________________.______
 *                 .                  o                   o
 *                 .             o    o              o    o
 *                 .        o         o         o         o
 *                 .   o              o    o              o     o
 *            0    o__________________o___________________o______
 *
 */

/****************************************************************************
 * Name: motor_angle_init
 *
 * Description:
 *   Initialize motor angle structure
 *
 * Input Parameters:
 *   angle - (in/out) pointer to the motor angle strucutre
 *   p     - (in) number of the motor pole pairs
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_angle_init(FAR struct motor_angle_s *angle, uint8_t p)
{
  DEBUGASSERT(angle != NULL);
  DEBUGASSERT(p > 0);

  /* Reset structure */

  memset(angle, 0, sizeof(struct motor_angle_s));

  /* Store pole pairs */

  angle->p = p;
  angle->one_by_p = (float)1.0f / p;

  /* Initialize angle with 0.0 */

  phase_angle_update(&angle->angle_el, 0.0f);
}

/****************************************************************************
 * Name: motor_angle_e_update
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

void motor_angle_e_update(FAR struct motor_angle_s *angle, float angle_new,
                          float dir)
{
  DEBUGASSERT(angle != NULL);
  DEBUGASSERT(angle_new >= 0.0f && angle_new <= MOTOR_ANGLE_E_MAX);
  DEBUGASSERT(dir == DIR_CW || dir == DIR_CCW);

  /* Check if we crossed electrical angle boundaries */

  if (dir == DIR_CW &&
      angle_new - angle->angle_el.angle < -POLE_CNTR_THR)
    {
      /* For CW direction - previous angle is greater than current angle */

      angle->i += 1;
    }

  else if (dir == DIR_CCW &&
           angle_new - angle->angle_el.angle > POLE_CNTR_THR)
    {
      /* For CCW direction - previous angle is lower than current angle */

      angle->i -= 1;
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

  phase_angle_update(&angle->angle_el, angle_new);

  /* Calculate mechanical angle.
   * One electrical angle rotation is equal to one mechanical rotation
   * divided by number of motor pole pairs.
   */

  angle->anglem = (MOTOR_ANGLE_E_RANGE * angle->i +
                   angle->angle_el.angle) * angle->one_by_p;

  /* Normalize mechanical angle to <0, 2PI> and store */

  angle_norm_2pi(&angle->anglem, MOTOR_ANGLE_M_MIN, MOTOR_ANGLE_M_MAX);
}

/****************************************************************************
 * Name: motor_angle_m_update
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

void motor_angle_m_update(FAR struct motor_angle_s *angle, float angle_new,
                          float dir)
{
  DEBUGASSERT(angle != NULL);
  DEBUGASSERT(angle_new >= 0.0f && angle_new <= MOTOR_ANGLE_E_MAX);
  DEBUGASSERT(dir == DIR_CW || dir == DIR_CCW);

  float angle_el = 0.0f;

  /* Store new mechanical angle */

  angle->anglem = angle_new;

  /* Update pole counter */

  angle->i = (uint8_t)(angle->anglem * angle->p / MOTOR_ANGLE_M_MAX);

  /* Get electrical angle */

  angle_el = angle->anglem * angle->p - MOTOR_ANGLE_E_MAX * angle->i;

  /* Update electrical angle structure */

  phase_angle_update(&angle->angle_el, angle_el);
}

/****************************************************************************
 * Name: motor_angle_m_get
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

float motor_angle_m_get(FAR struct motor_angle_s *angle)
{
  DEBUGASSERT(angle != NULL);

  return angle->anglem;
}

/****************************************************************************
 * Name: motor_angle_e_get
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

float motor_angle_e_get(FAR struct motor_angle_s *angle)
{
  DEBUGASSERT(angle != NULL);

  return angle->angle_el.angle;
}

/****************************************************************************
 * Name: motor_phy_params_init
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
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_phy_params_init(FAR struct motor_phy_params_s *phy, uint8_t poles,
                            float res, float ind)
{
  DEBUGASSERT(phy != NULL);

  phy->p          = poles;
  phy->res_base   = res;
  phy->ind        = ind;
  phy->one_by_ind = 1.0f / ind;

  /* Initialize with zeros */

  phy->res          = 0.0f;
  phy->res_alpha    = 0.0f;
  phy->res_temp_ref = 0.0f;
}

/****************************************************************************
 * Name: motor_phy_params_temp_set
 * Description:
 *   Initialize motor physical temperature parameters
 *
 * Input Parameters:
 *   phy          - (in/out) pointer to the motor physical parameters
 *   res_alpha    - (in) temperature coefficient of the winding resistance
 *   res_temp_ref - (in) reference temperature for alpha
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_phy_params_temp_set(FAR struct motor_phy_params_s *phy,
                               float res_alpha, float res_temp_ref)
{
  DEBUGASSERT(phy != NULL);

  phy->res_alpha    = res_alpha;
  phy->res_temp_ref = res_temp_ref;
}
