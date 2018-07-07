/****************************************************************************
 * control/lib_foc.c
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

#include <string.h>
#include <stdbool.h>

#include <dsp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_current_control
 *
 * Description:
 *   This function implements FOC current control algorithm.
 *
 * Input Parameters:
 *   foc - (in/out) pointer to the FOC data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_current_control(FAR struct foc_data_s *foc)
{
  FAR pid_controller_t *id_pid = &foc->id_pid;
  FAR pid_controller_t *iq_pid = &foc->iq_pid;
  FAR dq_frame_t       *v_dq  = &foc->v_dq;

  /* Get dq current error */

  foc->i_dq_err.d = foc->i_dq_ref.d - foc->i_dq.d;
  foc->i_dq_err.q = foc->i_dq_ref.q - foc->i_dq.q;

  /* NOTE: PI controllers saturation is updated in foc_vdq_mag_max_set() */

  /* PI controller for d-current (flux loop) */

  v_dq->d = pi_controller(id_pid, foc->i_dq_err.d);

  /* PI controller for q-current (torque loop) */

  v_dq->q = pi_controller(iq_pid, foc->i_dq_err.q);

  /* Saturate voltage DQ vector.
   * The maximum DQ voltage magnitude depends on the maximum possible
   * phase voltage and the maximum supported duty cycle.
   */

  dq_saturate(v_dq, foc->vdq_mag_max);
}

/****************************************************************************
 * Name: foc_vab_mod_scale_set
 *
 * Description:
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   scale - (in) scaling factor for alpha-beta voltage
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_vab_mod_scale_set(FAR struct foc_data_s *foc, float scale)
{
  foc->vab_mod_scale = scale;
}

/****************************************************************************
 * Name: foc_vdq_mag_max_set
 *
 * Description:
 *   Set maximum dq voltage vector magnitude
 *
 * Input Parameters:
 *   foc - (in/out) pointer to the FOC data
 *   max - (in) maximum dq voltage magnitude
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_vdq_mag_max_set(FAR struct foc_data_s *foc, float max)
{
  foc->vdq_mag_max = max;

  /* Update regulators saturation */

  pi_saturation_set(&foc->id_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
  pi_saturation_set(&foc->iq_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_init
 *
 * Description:
 *   Initialize FOC controller
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   id_kp - (in) KP for d current
 *   id_ki - (in) KI for d current
 *   iq_kp - (in) KP for q current
 *   iq_ki - (in) KI for q current
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_init(FAR struct foc_data_s *foc,
              float id_kp, float id_ki, float iq_kp, float iq_ki)
{
  /* Reset data */

  memset(foc, 0, sizeof(struct foc_data_s));

  /* Initialize PI current d component */

  pi_controller_init(&foc->id_pid, id_kp, id_ki);

  /* Initialize PI current q component */

  pi_controller_init(&foc->iq_pid, iq_kp, iq_ki);
}

/****************************************************************************
 * Name: foc_idq_ref_set
 *
 * Description:
 *   Set dq reference current vector
 *
 * Input Parameters:
 *   foc - (in/out) pointer to the FOC data
 *   d   - (in) reference d current
 *   q   - (in) reference q current
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_idq_ref_set(FAR struct foc_data_s *foc, float d, float q)
{
  foc->i_dq_ref.d = d;
  foc->i_dq_ref.q = q;
}

/****************************************************************************
 * Name: foc_vbase_update
 *
 * Description:
 *  Update base voltage for FOC controller
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   vbase - (in) base voltage for FOC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_vbase_update(FAR struct foc_data_s *foc, float vbase)
{
  float scale   = 0.0f;
  float mag_max = 0.0f;

  /* Only if voltage is valid */

  if (vbase >= 0.0f)
    {
      scale = 1.0f/vbase;
      mag_max = vbase;
    }

  foc_vab_mod_scale_set(foc, scale);
  foc_vdq_mag_max_set(foc, mag_max);
}

/****************************************************************************
 * Name: foc_process
 *
 * Description:
 *   Process FOC (Field Oriented Control)
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   i_abc - (in) pointer to the ABC current frame
 *   angle - (in) pointer to the phase angle data
 *
 * Returned Value:
 *   None
 *
 * TODO: add some reference and a brief description of the FOC
 *
 ****************************************************************************/

void foc_process(FAR struct foc_data_s *foc,
                 FAR abc_frame_t *i_abc,
                 FAR phase_angle_t *angle)
{
  DEBUGASSERT(foc != NULL);
  DEBUGASSERT(i_abc != NULL);
  DEBUGASSERT(angle != NULL);

  /* Copy ABC current to foc data */

  foc->i_abc.a = i_abc->a;
  foc->i_abc.b = i_abc->b;
  foc->i_abc.c = i_abc->c;

  /* Convert abc current to alpha-beta current */

  clarke_transform(&foc->i_abc, &foc->i_ab);

  /* Convert alpha-beta current to dq current */

  park_transform(angle, &foc->i_ab, &foc->i_dq);

  /* Run FOC current control (current dq -> voltage dq) */

  foc_current_control(foc);

  /* Inverse Park tranform (voltage dq -> voltage alpha-beta) */

  inv_park_transform(angle, &foc->v_dq, &foc->v_ab);

  /* Normalize the alpha-beta voltage to get the alpha-beta modulation voltage */

  foc->v_ab_mod.a = foc->v_ab.a * foc->vab_mod_scale;
  foc->v_ab_mod.b = foc->v_ab.b * foc->vab_mod_scale;
}
