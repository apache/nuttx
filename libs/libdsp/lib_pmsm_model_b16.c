/****************************************************************************
 * libs/libdsp/lib_pmsm_model_b16.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <dspb16.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmsm_model_initialize_b16
 *
 * Description:
 *   Initialzie FOC model
 *
 ****************************************************************************/

int pmsm_model_initialize_b16(FAR struct pmsm_model_b16_s *model,
                              FAR struct pmsm_phy_params_b16_s *phy,
                              b16_t per)
{
  DEBUGASSERT(model);
  DEBUGASSERT(phy);
  DEBUGASSERT(per > 0);

  /* Copy motor model parameters */

  memcpy(&model->phy, phy, sizeof(struct pmsm_phy_params_b16_s));

  /* Initialize controller period */

  model->per = per;

  return OK;
}

/****************************************************************************
 * Name: pmsm_model_elec_b16
 *
 * Description:
 *  Update motor model electrical state
 *
 ****************************************************************************/

int pmsm_model_elec_b16(FAR struct pmsm_model_b16_s *model,
                        FAR ab_frame_b16_t *vab)
{
  b16_t        tmp1   = 0;
  b16_t        tmp2   = 0;
  b16_t        tmp3   = 0;
  b16_t        tmp4   = 0;
  b16_t        tmp5   = 0;
  b16_t        tmp6   = 0;

  DEBUGASSERT(model);
  DEBUGASSERT(vab);

  /* Copy alpha-beta voltage */

  model->state.v_ab.a = vab->a;
  model->state.v_ab.b = vab->b;

  /* Inverse Clarke transform - get abc voltage */

  inv_clarke_transform_b16(&model->state.v_ab,
                           &model->state.v_abc);

  /* Park transform - get DQ voltage */

  park_transform_b16(&model->state.angle.angle_el,
                     &model->state.v_ab,
                     &model->state.v_dq);

  /* q current */

  tmp1 = b16mulb16(model->phy.motor.res, model->state.i_dq.q);
  tmp2 = b16mulb16(model->phy.ind_d, model->state.i_dq.d);
  tmp3 = tmp2 + model->phy.motor.flux_link;
  tmp4 = b16mulb16(model->state.omega_e, tmp3);
  tmp5 = model->state.v_dq.q - tmp1 - tmp4;
  tmp6 = b16mulb16(model->per, tmp5);

  model->iq_int += b16mulb16(tmp6, model->phy.one_by_indq);

  /* d current */

  tmp1 = b16mulb16(model->phy.motor.res, model->state.i_dq.d);
  tmp2 = b16mulb16(model->phy.ind_q, model->state.i_dq.q);
  tmp3 = b16mulb16(tmp2, model->state.omega_e);
  tmp4 = model->state.v_dq.d - tmp1 + tmp3;
  tmp5 = b16mulb16(model->per, tmp4);

  model->id_int += b16mulb16(tmp5, model->phy.one_by_indd);

  /* Store currents */

  model->state.i_dq.q = model->iq_int;
  model->state.i_dq.d = model->id_int;

  /* Inverse Park transform - get alpha-beta current */

  inv_park_transform_b16(&model->state.angle.angle_el,
                         &model->state.i_dq,
                         &model->state.i_ab);

  /* Inverse Clarke transform - get abc current */

  inv_clarke_transform_b16(&model->state.i_ab,
                           &model->state.i_abc);

  return OK;
}

/****************************************************************************
 * Name: pmsm_model_mech_b16
 *
 * Description:
 *  Update motor model mechanical state
 *
 ****************************************************************************/

int pmsm_model_mech_b16(FAR struct pmsm_model_b16_s *model, b16_t load)
{
  b16_t angle = 0;
  b16_t dir   = 0;
  b16_t te    = 0;
  b16_t tmp1  = 0;
  b16_t tmp2  = 0;
  b16_t tmp3  = 0;
  b16_t tmp4  = 0;
  b16_t tmp5  = 0;

  DEBUGASSERT(model);

  /* Get electrical torque developed by the motor */

  tmp1 = model->phy.ind_d - model->phy.ind_q;
  tmp2 = b16mulb16(tmp1, model->state.i_dq.d);
  tmp3 = model->phy.motor.flux_link - tmp2;
  tmp4 = b16mulb16((b16ONE + b16HALF), itob16(model->phy.motor.p));
  tmp5 = b16mulb16(tmp4, model->state.i_dq.q);

  te = b16mulb16(tmp5, tmp3);

  /* Get new mechanical velocity */

  tmp1 = te - load;
  tmp2 = b16mulb16(model->per, tmp1);
  tmp3 = b16mulb16(tmp2, model->phy.one_by_iner);

  model->state.omega_m = model->state.omega_m + tmp3;

  /* Get new electrical velocity */

  model->state.omega_e = b16mulb16(model->state.omega_m,
                                   itob16(model->phy.motor.p));

  /* Get rotation direction */

  dir = (model->state.omega_e > 0 ? DIR_CW_B16 : DIR_CCW_B16);

  /* Update electrical angle */

  tmp1 = b16mulb16(model->state.omega_e, model->per);

  angle = model->state.angle.angle_el.angle + tmp1;

  /* Update with mechanical angel */

  motor_angle_e_update_b16(&model->state.angle, angle, dir);

  return OK;
}
