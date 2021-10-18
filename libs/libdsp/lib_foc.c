/****************************************************************************
 * libs/libdsp/lib_foc.c
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

#include <string.h>
#include <stdbool.h>

#include <dsp.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_current_controller
 *
 * Description:
 *   This function implements FOC current controller algorithm.
 *
 * Input Parameters:
 *   foc      - (in/out) pointer to the FOC data
 *   v_dq_req - (in) pointer to the voltage DQ request frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_current_controller(FAR struct foc_data_f32_s *foc,
                                  FAR dq_frame_f32_t *v_dq_req)
{
  FAR pid_controller_f32_t *id_pid = &foc->id_pid;
  FAR pid_controller_f32_t *iq_pid = &foc->iq_pid;

  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(v_dq_req != NULL);

  /* Get dq current error */

  foc->i_dq_err.d = foc->i_dq_ref.d - foc->i_dq.d;
  foc->i_dq_err.q = foc->i_dq_ref.q - foc->i_dq.q;

  /* NOTE: PI controllers saturation is updated in foc_vdq_mag_max_set() */

  /* PI controller for d-current (flux loop) */

  v_dq_req->d = pi_controller(id_pid, foc->i_dq_err.d);

  /* PI controller for q-current (torque loop) */

  v_dq_req->q = pi_controller(iq_pid, foc->i_dq_err.q);
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

static void foc_vab_mod_scale_set(FAR struct foc_data_f32_s *foc,
                                  float scale)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

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

static void foc_vdq_mag_max_set(FAR struct foc_data_f32_s *foc, float max)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  foc->vdq_mag_max = max;

  /* Update regulators saturation */

  pi_saturation_set(&foc->id_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
  pi_saturation_set(&foc->iq_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
}

/****************************************************************************
 * Name: foc_vdq_ref_set
 *
 * Description:
 *   Set dq requested voltage vector
 *
 * Input Parameters:
 *   foc     - (in/out) pointer to the FOC data
 *   vdq_ref - (in) pointer to the requested idq voltage
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_vdq_ref_set(FAR struct foc_data_f32_s *foc,
                            FAR dq_frame_f32_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(vdq_ref != NULL);

  foc->v_dq.d = vdq_ref->d;
  foc->v_dq.q = vdq_ref->q;
}

/****************************************************************************
 * Name: foc_idq_ref_set
 *
 * Description:
 *   Set dq reference current vector
 *
 * Input Parameters:
 *   foc     - (in/out) pointer to the FOC data
 *   idq_ref - (in) pointer to the reference idq current
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void foc_idq_ref_set(FAR struct foc_data_f32_s *foc,
                            FAR dq_frame_f32_t *idq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(idq_ref != NULL);

  foc->i_dq_ref.d = idq_ref->d;
  foc->i_dq_ref.q = idq_ref->q;
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
 *   foc  - (in/out) pointer to the FOC data
 *   init - (in) pointer to the FOC initialization data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_init(FAR struct foc_data_f32_s *foc,
              FAR struct foc_initdata_f32_s *init)
{
  /* Reset data */

  memset(foc, 0, sizeof(struct foc_data_f32_s));

  /* Initialize PI current d component */

  pi_controller_init(&foc->id_pid, init->id_kp, init->id_ki);

  /* Initialize PI current q component */

  pi_controller_init(&foc->iq_pid, init->iq_kp, init->iq_ki);

  /* Disable PI intergral part reset when saturated */

  pi_ireset_enable(&foc->iq_pid, false);
  pi_ireset_enable(&foc->id_pid, false);

  /* Enable PI anti-windup protection */

  pi_antiwindup_enable(&foc->iq_pid, 0.99, true);
  pi_antiwindup_enable(&foc->id_pid, 0.99, true);
}

/****************************************************************************
 * Name: foc_vbase_update
 *
 * Description:
 *   Update base voltage for FOC controller
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   vbase - (in) base voltage for FOC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_vbase_update(FAR struct foc_data_f32_s *foc, float vbase)
{
  float scale   = 0.0f;
  float mag_max = 0.0f;

  /* Prevent division by zero */

  if (vbase < 1e-10f)
    {
      vbase = 1e-10f;
    }

  /* NOTE: this is base voltage for FOC, not bus voltage! */

  scale = (1.0f / vbase);
  mag_max = vbase;

  /* Update */

  foc_vab_mod_scale_set(foc, scale);
  foc_vdq_mag_max_set(foc, mag_max);
}

/****************************************************************************
 * Name: foc_angle_update
 *
 * Description:
 *   Update FOC data with new motor phase angle.
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   angle - (in) pointer to the phase angle data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_angle_update(FAR struct foc_data_f32_s *foc,
                      FAR phase_angle_f32_t *angle)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(angle != NULL);

  /* Copy angle to foc data */

  foc->angle.angle = angle->angle;
  foc->angle.sin   = angle->sin;
  foc->angle.cos   = angle->cos;
}

/****************************************************************************
 * Name: foc_iabc_update
 *
 * Description:
 *   Update FOC data with new iabc frame.
 *
 *   To work properly this function requires some additional steps:
 *     1. phase angle must be set with foc_angle_update()
 *
 * Input Parameters:
 *   foc   - (in/out) pointer to the FOC data
 *   i_abc - (in) pointer to the ABC current frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_iabc_update(FAR struct foc_data_f32_s *foc,
                     FAR abc_frame_f32_t *i_abc)
{
  dq_frame_f32_t i_dq;

  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(i_abc != NULL);

  /* Reset data */

  i_dq.d = 0;
  i_dq.q = 0;

  /* Copy ABC current to foc data */

  foc->i_abc.a = i_abc->a;
  foc->i_abc.b = i_abc->b;
  foc->i_abc.c = i_abc->c;

  /* Convert abc current to alpha-beta current */

  clarke_transform(&foc->i_abc, &foc->i_ab);

  /* Convert alpha-beta current to dq current */

  park_transform(&foc->angle, &foc->i_ab, &i_dq);

  /* Store dq current */

  foc->i_dq.d = i_dq.d;
  foc->i_dq.q = i_dq.q;
}

/****************************************************************************
 * Name: foc_voltage_control
 *
 * Description:
 *   Process FOC voltage control.
 *
 * Input Parameters:
 *   foc     - (in/out) pointer to the FOC data
 *   vdq_ref - (in) voltage dq reference frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_voltage_control(FAR struct foc_data_f32_s *foc,
                         FAR dq_frame_f32_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  /* Update VDQ request */

  foc_vdq_ref_set(foc, vdq_ref);

  /* Inverse Park transform (voltage dq -> voltage alpha-beta) */

  inv_park_transform(&foc->angle, &foc->v_dq, &foc->v_ab);

#ifdef CONFIG_LIBDSP_FOC_VABC
  /* Inverse Clarke transform (voltage alpha-beta -> voltage abc) */

  inv_clarke_transform(&foc->v_ab, &foc->v_abc);
#endif

  /* Normalize the alpha-beta voltage to get the alpha-beta modulation
   * voltage
   */

  foc->v_ab_mod.a = foc->v_ab.a * foc->vab_mod_scale;
  foc->v_ab_mod.b = foc->v_ab.b * foc->vab_mod_scale;
}

/****************************************************************************
 * Name: foc_current_control
 *
 * Description:
 *   Process FOC current control.
 *
 * Input Parameters:
 *   foc      - (in/out) pointer to the FOC data
 *   idq_ref  - (in) current dq reference frame
 *   vdq_comp - (in) voltage dq compensation frame
 *   vdq_ref  - (out) voltage dq reference frame
 *
 * Returned Value:
 *   None
 *
 * TODO: add some reference and a brief description of the FOC
 *
 ****************************************************************************/

void foc_current_control(FAR struct foc_data_f32_s *foc,
                         FAR dq_frame_f32_t *idq_ref,
                         FAR dq_frame_f32_t *vdq_comp,
                         FAR dq_frame_f32_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  /* Update IDQ reference */

  foc_idq_ref_set(foc, idq_ref);

  /* Run FOC current controller (current dq -> voltage dq) */

  foc_current_controller(foc, vdq_ref);

  /* DQ voltage compensation */

  vdq_ref->d = vdq_ref->d - vdq_comp->d;
  vdq_ref->q = vdq_ref->q - vdq_comp->q;
}

/****************************************************************************
 * Name: foc_vabmod_get
 *
 * Description:
 *   Get result from the FOC controller (foc_current_control or
 *   foc_voltage_control)
 *
 * Input Parameters:
 *   foc      - (in/out) pointer to the FOC data
 *   v_ab_mod - (out) pointer to the voltage alpha-beta modulation frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_vabmod_get(FAR struct foc_data_f32_s *foc,
                    FAR ab_frame_f32_t *v_ab_mod)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(v_ab_mod != NULL);

  v_ab_mod->a = foc->v_ab_mod.a;
  v_ab_mod->b = foc->v_ab_mod.b;
}

/****************************************************************************
 * Name: foc_vdq_mag_max_get
 *
 * Description:
 *   Get maximum dq voltage vector magnitude
 *
 * Input Parameters:
 *   foc - (in/out) pointer to the FOC data
 *   max - (out) maximum dq voltage magnitude
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_vdq_mag_max_get(FAR struct foc_data_f32_s *foc, FAR float *max)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  *max = foc->vdq_mag_max;
}
