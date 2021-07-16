/****************************************************************************
 * libs/libdsp/lib_foc_b16.c
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

#include <dspb16.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_current_controller_b16
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

static void foc_current_controller_b16(FAR struct foc_data_b16_s *foc,
                                      FAR dq_frame_b16_t *v_dq_req)
{
  FAR pid_controller_b16_t *id_pid = &foc->id_pid;
  FAR pid_controller_b16_t *iq_pid = &foc->iq_pid;

  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(v_dq_req != NULL);

  /* Get dq current error */

  foc->i_dq_err.d = foc->i_dq_ref.d - foc->i_dq.d;
  foc->i_dq_err.q = foc->i_dq_ref.q - foc->i_dq.q;

  /* NOTE: PI controllers saturation is updated in foc_vdq_mag_max_set() */

  /* PI controller for d-current (flux loop) */

  v_dq_req->d = pi_controller_b16(id_pid, foc->i_dq_err.d);

  /* PI controller for q-current (torque loop) */

  v_dq_req->q = pi_controller_b16(iq_pid, foc->i_dq_err.q);
}

/****************************************************************************
 * Name: foc_vab_mod_scale_set_b16
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

static void foc_vab_mod_scale_set_b16(FAR struct foc_data_b16_s *foc,
                                      b16_t scale)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  foc->vab_mod_scale = scale;
}

/****************************************************************************
 * Name: foc_vdq_mag_max_set_b16
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

static void foc_vdq_mag_max_set_b16(FAR struct foc_data_b16_s *foc,
                                    b16_t max)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  foc->vdq_mag_max = max;

  /* Update regulators saturation */

  pi_saturation_set_b16(&foc->id_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
  pi_saturation_set_b16(&foc->iq_pid, -foc->vdq_mag_max, foc->vdq_mag_max);
}

/****************************************************************************
 * Name: foc_vdq_ref_set_b16
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

static void foc_vdq_ref_set_b16(FAR struct foc_data_b16_s *foc,
                                FAR dq_frame_b16_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(vdq_ref != NULL);

  foc->v_dq.d = vdq_ref->d;
  foc->v_dq.q = vdq_ref->q;
}

/****************************************************************************
 * Name: foc_idq_ref_set_b16
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

static void foc_idq_ref_set_b16(FAR struct foc_data_b16_s *foc,
                                FAR dq_frame_b16_t *idq_ref)
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
 * Name: foc_init_b16
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

void foc_init_b16(FAR struct foc_data_b16_s *foc,
                  FAR struct foc_initdata_b16_s *init)
{
  /* Reset data */

  memset(foc, 0, sizeof(struct foc_data_b16_s));

  /* Initialize PI current d component */

  pi_controller_init_b16(&foc->id_pid, init->id_kp, init->id_ki);

  /* Initialize PI current q component */

  pi_controller_init_b16(&foc->iq_pid, init->iq_kp, init->iq_ki);

  /* Disable PI intergral part reset when saturated */

  pi_ireset_enable_b16(&foc->iq_pid, false);
  pi_ireset_enable_b16(&foc->id_pid, false);

  /* Enable PI anti-windup protection */

  pi_antiwindup_enable_b16(&foc->iq_pid, ftob16(0.99f), true);
  pi_antiwindup_enable_b16(&foc->id_pid, ftob16(0.99f), true);
}

/****************************************************************************
 * Name: foc_vbase_update_b16
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

void foc_vbase_update_b16(FAR struct foc_data_b16_s *foc, b16_t vbase)
{
  b16_t scale   = 0;
  b16_t mag_max = 0;

  /* Prevent division by zero */

  if (vbase < 1)
    {
      vbase = 1;
    }

  /* NOTE: this is base voltage for FOC, not bus voltage! */

  scale = b16divb16(b16ONE, vbase);
  mag_max = vbase;

  /* Update */

  foc_vab_mod_scale_set_b16(foc, scale);
  foc_vdq_mag_max_set_b16(foc, mag_max);
}

/****************************************************************************
 * Name: foc_angle_update_b16
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

void foc_angle_update_b16(FAR struct foc_data_b16_s *foc,
                          FAR phase_angle_b16_t *angle)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(angle != NULL);

  /* Copy angle to foc data */

  foc->angle.angle = angle->angle;
  foc->angle.sin   = angle->sin;
  foc->angle.cos   = angle->cos;
}

/****************************************************************************
 * Name: foc_iabc_update_b16
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

void foc_iabc_update_b16(FAR struct foc_data_b16_s *foc,
                         FAR abc_frame_b16_t *i_abc)
{
  dq_frame_b16_t i_dq;

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

  clarke_transform_b16(&foc->i_abc, &foc->i_ab);

  /* Convert alpha-beta current to dq current */

  park_transform_b16(&foc->angle, &foc->i_ab, &i_dq);

  /* Store dq current */

  foc->i_dq.d = i_dq.d;
  foc->i_dq.q = i_dq.q;
}

/****************************************************************************
 * Name: foc_voltage_control_b16
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

void foc_voltage_control_b16(FAR struct foc_data_b16_s *foc,
                             FAR dq_frame_b16_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  /* Update VDQ request */

  foc_vdq_ref_set_b16(foc, vdq_ref);

  /* Inverse Park transform (voltage dq -> voltage alpha-beta) */

  inv_park_transform_b16(&foc->angle, &foc->v_dq, &foc->v_ab);

#ifdef CONFIG_LIBDSP_FOC_VABC
  /* Inverse Clarke transform (voltage alpha-beta -> voltage abc) */

  inv_clarke_transform_b16(&foc->v_ab, &foc->v_abc);
#endif

  /* Normalize the alpha-beta voltage to get the alpha-beta modulation
   * voltage
   */

  foc->v_ab_mod.a = b16mulb16(foc->v_ab.a, foc->vab_mod_scale);
  foc->v_ab_mod.b = b16mulb16(foc->v_ab.b, foc->vab_mod_scale);
}

/****************************************************************************
 * Name: foc_current_control_b16
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

void foc_current_control_b16(FAR struct foc_data_b16_s *foc,
                             FAR dq_frame_b16_t *idq_ref,
                             FAR dq_frame_b16_t *vdq_comp,
                             FAR dq_frame_b16_t *vdq_ref)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  /* Update IDQ reference */

  foc_idq_ref_set_b16(foc, idq_ref);

  /* Run FOC current controller (current dq -> voltage dq) */

  foc_current_controller_b16(foc, vdq_ref);

  /* DQ voltage compensation */

  vdq_ref->d = vdq_ref->d - vdq_comp->d;
  vdq_ref->q = vdq_ref->q - vdq_comp->q;
}

/****************************************************************************
 * Name: foc_vabmod_get_b16
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

void foc_vabmod_get_b16(FAR struct foc_data_b16_s *foc,
                        FAR ab_frame_b16_t *v_ab_mod)
{
  LIBDSP_DEBUGASSERT(foc != NULL);
  LIBDSP_DEBUGASSERT(v_ab_mod != NULL);

  v_ab_mod->a = foc->v_ab_mod.a;
  v_ab_mod->b = foc->v_ab_mod.b;
}

/****************************************************************************
 * Name: foc_vdq_mag_max_get_b16
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

void foc_vdq_mag_max_get_b16(FAR struct foc_data_b16_s *foc, FAR b16_t *max)
{
  LIBDSP_DEBUGASSERT(foc != NULL);

  *max = foc->vdq_mag_max;
}
