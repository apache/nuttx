/****************************************************************************
 * libs/libdsp/lib_pid_b16.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pid_controller_init_b16
 *
 * Description:
 *   Initialize PID controller. This function does not initialize saturation
 *   limits.
 *
 * Input Parameters:
 *   pid - (out) pointer to the PID controller data
 *   KP  - (in) proportional gain
 *   KI  - (in) integral gain
 *   KD  - (in) derivative gain
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pid_controller_init_b16(FAR pid_controller_b16_t *pid, b16_t KP,
                             b16_t KI, b16_t KD)
{
  LIBDSP_DEBUGASSERT(pid != NULL);

  /* Reset controller data */

  memset(pid, 0, sizeof(pid_controller_b16_t));

  /* Copy controller parameters */

  pid->KP = KP;
  pid->KI = KI;
  pid->KD = KD;
  pid->KC = 0;
}

/****************************************************************************
 * Name: pi_controller_init_b16
 *
 * Description:
 *   Initialize PI controller. This function does not initialize saturation
 *   limits.
 *
 * Input Parameters:
 *   pid - (out) pointer to the PID controller data
 *   KP  - (in) proportional gain
 *   KI  - (in) integral gain
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pi_controller_init_b16(FAR pid_controller_b16_t *pid, b16_t KP,
                            b16_t KI)
{
  LIBDSP_DEBUGASSERT(pid != NULL);

  /* Reset controller data */

  memset(pid, 0, sizeof(pid_controller_b16_t));

  /* Copy controller parameters */

  pid->KP = KP;
  pid->KI = KI;
  pid->KD = 0;
  pid->KC = 0;

  /* No windup-protection at default */

  pid->aw_en     = false;
  pid->ireset_en = false;
}

/****************************************************************************
 * Name: pid_saturation_set_b16
 *
 * Description:
 *   Set controller saturation limits. Sometimes we need change saturation
 *   configuration in the run-time, so this function is separate from
 *   pid_controller_init().
 *
 * Input Parameters:
 *   pid - (out) pointer to the PID controller data
 *   min - (in) lower limit
 *   max - (in) upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pid_saturation_set_b16(FAR pid_controller_b16_t *pid, b16_t min,
                            b16_t max)
{
  LIBDSP_DEBUGASSERT(pid != NULL);
  LIBDSP_DEBUGASSERT(min < max);

  pid->sat.max = max;
  pid->sat.min = min;

  /* Enable saturation in PID controller */

  pid->pidsat_en = true;
}

/****************************************************************************
 * Name: pi_saturation_set_b16
 *
 * Description:
 *
 * Input Parameters:
 *   pid - (out) pointer to the PID controller data
 *   min - (in) lower limit
 *   max - (in) upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pi_saturation_set_b16(FAR pid_controller_b16_t *pid, b16_t min,
                           b16_t max)
{
  LIBDSP_DEBUGASSERT(pid != NULL);
  LIBDSP_DEBUGASSERT(min < max);

  pid->sat.max = max;
  pid->sat.min = min;

  /* Enable saturation in PI controller */

  pid->pisat_en = true;
}

/****************************************************************************
 * Name: pid_antiwindup_enable_b16
 ****************************************************************************/

void pi_antiwindup_enable_b16(FAR pid_controller_b16_t *pid, b16_t KC,
                              bool enable)
{
  pid->aw_en = enable;
  pid->KC    = KC;
}

/****************************************************************************
 * Name: pid_ireset_enable_b16
 ****************************************************************************/

void pi_ireset_enable_b16(FAR pid_controller_b16_t *pid, bool enable)
{
  pid->ireset_en = enable;
}

/****************************************************************************
 * Name: pid_integral_reset_b16
 ****************************************************************************/

void pid_integral_reset_b16(FAR pid_controller_b16_t *pid)
{
  pid->part[1] = 0;
  pid->aw      = 0;
}

/****************************************************************************
 * Name: pi_integral_reset_b16
 ****************************************************************************/

void pi_integral_reset_b16(FAR pid_controller_b16_t *pid)
{
  pid_integral_reset_b16(pid);
}

/****************************************************************************
 * Name: pi_controller_b16
 *
 * Description:
 *   PI controller with output saturation and windup protection
 *
 * Input Parameters:
 *   pid - (in/out) pointer to the PI controller data
 *   err - (in) current controller error
 *
 * Returned Value:
 *   Return controller output.
 *
 ****************************************************************************/

b16_t pi_controller_b16(FAR pid_controller_b16_t *pid, b16_t err)
{
  LIBDSP_DEBUGASSERT(pid != NULL);

  b16_t tmp = 0;

  /* Store error in controller structure */

  pid->err = err;

  /* Get proportional part */

  pid->part[0] = b16mulb16(pid->KP, err);

  /* Get intergral part */

  pid->part[1] += b16mulb16(pid->KI, (err - pid->aw));

  /* Add proportional, integral */

  pid->out = pid->part[0] + pid->part[1];

  /* Store not saturated output */

  tmp = pid->out;

  /* Saturate output if enabled */

  if (pid->pisat_en == true)
    {
      if (pid->out > pid->sat.max)
        {
          if (pid->ireset_en == true)
            {
              /* Reset I part */

              if (err > 0)
                {
                  pi_integral_reset_b16(pid);
                }
            }

          /* Limit output to the upper limit */

          pid->out = pid->sat.max;
        }
      else if (pid->out < pid->sat.min)
        {
          if (pid->ireset_en == true)
            {
              /* Reset I part */

              if (err < 0)
                {
                  pi_integral_reset_b16(pid);
                }
            }

          /* Limit output to the lower limit */

          pid->out = pid->sat.min;
        }
    }

  /* Anti-windup I-part decay if enabled */

  if (pid->aw_en == true)
    {
      pid->aw = b16mulb16(pid->KC, (tmp - pid->out));
    }

  /* Return regulator output */

  return pid->out;
}

/****************************************************************************
 * Name: pid_controller_b16
 *
 * Description:
 *   PID controller with output saturation and windup protection
 *
 * Input Parameters:
 *   pid - (in/out) pointer to the PID controller data
 *   err - (in) current controller error
 *
 * Returned Value:
 *   Return controller output.
 *
 ****************************************************************************/

b16_t pid_controller_b16(FAR pid_controller_b16_t *pid, b16_t err)
{
  LIBDSP_DEBUGASSERT(pid != NULL);

  /* Get PI output */

  pi_controller_b16(pid, err);

  /* Get derivative part */

  pid->part[2] = b16mulb16(pid->KD, (err - pid->err_prev));

  /* Add derivative part to the PI part */

  pid->out += pid->part[2];

  /* Store current error */

  pid->err_prev = err;

  /* Saturate output if enabled */

  if (pid->pidsat_en == true)
    {
      if (pid->out > pid->sat.max)
        {
          /* Limit output to the upper limit */

          pid->out = pid->sat.max;
        }
      else if (pid->out < pid->sat.min)
        {
          /* Limit output to the lower limit */

          pid->out = pid->sat.min;
        }
    }

  /* Return regulator output */

  return pid->out;
}
