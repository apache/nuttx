/****************************************************************************
 * libs/libdsp/lib_pmsm_model.c
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

/* Permanent magnet synchronous motor model (PMSM)
 *
 * 1) Flux equation:
 *
 *   lambda_q = L_q * i_q
 *   lambda_d = L_d * i_d + lampda_m
 *
 * 2) Voltage equation:
 *
 *   a) D-part:
 *
 *       id    Rs          Ld
 *    o-->----/\/\/\/\----mmmmmm----+
 *    ^                             |
 *    .                             |
 *    .                          .-----.
 *    .                          |  ^  |
 * vd .                          |  |  |
 *    .                          .-----.
 *    .                             | ed
 *    .                             |
 *    o-----------------------------+
 *
 *   ed = -we * lamda_q
 *   ed = -we * (Lq * iq)
 *
 *   vd = (Rs * id) + (d/dt * (Ld * id)) - (Lq * we * iq)
 *   Ld * (d/dt * id) = vd - (Rs * id) + (we * Lq * iq)
 *   (d/dt * id) = (vd - (Rs * id) + (we * Lq * iq)) / Ld
 *
 *   b) Q-part:
 *
 *       iq    Rs          Lq
 *    o-->----/\/\/\/\----mmmmmm----+
 *    ^                             |
 *    .                             |
 *    .                          .-----.
 *    .                          |  ^  |
 * vq .                          |  |  |
 *    .                          .-----.
 *    .                             | eq
 *    .                             |
 *    o-----------------------------+
 *
 *   eq = we * lamda_d
 *   eq = we * (Lq * iq + lamda_m)
 *
 *   vq = (Rs * iq) + (d/dt * (Lq * iq)) + (we * (Ld * id + lambda_m))
 *   Lq * (d/dt * iq) = vq - (Rs * iq) - (we * (Ld * id + lambda_m))
 *   (d/dt * iq) = (vq - (Rs * iq) - (we * (Ld * id + lambda_m))) / Lq
 *
 * 3) Torque equation:
 *
 *   Te = (3/2) * p * (lambda_d * i_q - lambda_q * i_d)
 *   Te = (3/2) * p * (lambda_m * i_q + (L_d - L_q) * i_q * i_d)
 *   Te = (3/2) * p * i_q * (lambda_m  + (L_d - L_q) * i_d)
 *
 * 4) Electromechanical power equation:
 *
 *   Pem = wm * Te
 *   Pem = (3/2) * wm * (lamda_d * i_q - lamda_q * i_d)
 *
 * 5) The general mechanical equation for the motor:
 *
 *   Te = Tl + Td + B * wm + J * (d/dt) * wm
 *   we = p * wm = (P/2) * wm
 *   p  = (P/2)
 *
 *   assume no friction:
 *
 *     Te = Tl + J * (d/dt) * wm
 *     (d/dt) * wm = (Te - Tl) / J
 *
 *  where:
 *   B        - viscous frictions coefficient
 *   J        - interia of the shaft and the load system
 *   Td       - dry friction
 *   Tl       - load torque
 *   Te       - electromagnetic torque
 *   Pe       - electromagnetical power
 *   we       - electrical velociti of the motor
 *   wm       - mechanical velocity of the rotor
 *   lambda_m - flux linkage
 *   P        - Number of poles
 *   p        - pole pairs
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <dsp.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmsm_model_initialize
 *
 * Description:
 *   Initialzie FOC model
 *
 ****************************************************************************/

int pmsm_model_initialize(FAR struct pmsm_model_f32_s *model,
                          FAR struct pmsm_phy_params_f32_s *phy,
                          float per)
{
  DEBUGASSERT(model);
  DEBUGASSERT(phy);
  DEBUGASSERT(per > 0.0f);

  /* Copy motor model parameters */

  memcpy(&model->phy, phy, sizeof(struct pmsm_phy_params_f32_s));

  /* Initialize controller period */

  model->per = per;

  return OK;
}

/****************************************************************************
 * Name: pmsm_model_elec
 *
 * Description:
 *  Update motor model electrical state
 *
 ****************************************************************************/

int pmsm_model_elec(FAR struct pmsm_model_f32_s *model,
                    FAR ab_frame_f32_t *vab)
{
  float tmp1 = 0.0f;
  float tmp2 = 0.0f;
  float tmp3 = 0.0f;
  float tmp4 = 0.0f;
  float tmp5 = 0.0f;
  float tmp6 = 0.0f;

  DEBUGASSERT(model);
  DEBUGASSERT(vab);

  /* Copy alpha-beta voltage */

  model->state.v_ab.a = vab->a;
  model->state.v_ab.b = vab->b;

  /* Inverse Clarke transform - get abc voltage */

  inv_clarke_transform(&model->state.v_ab,
                       &model->state.v_abc);

  /* Park transform - get DQ voltage */

  park_transform(&model->state.angle.angle_el,
                 &model->state.v_ab,
                 &model->state.v_dq);

  /* q current */

  tmp1 = model->phy.motor.res * model->state.i_dq.q;
  tmp2 = model->phy.ind_d * model->state.i_dq.d;
  tmp3 = tmp2 + model->phy.motor.flux_link;
  tmp4 = model->state.omega_e * tmp3;
  tmp5 = model->state.v_dq.q - tmp1 - tmp4;
  tmp6 = model->per * tmp5;

  model->iq_int += (tmp6 * model->phy.one_by_indq);

  /* d current */

  tmp1 = model->phy.motor.res * model->state.i_dq.d;
  tmp2 = model->phy.ind_q * model->state.i_dq.q;
  tmp3 = tmp2 * model->state.omega_e;
  tmp4 = model->state.v_dq.d - tmp1 + tmp3;
  tmp5 = model->per * tmp4;

  model->id_int += (tmp5 * model->phy.one_by_indd);

  /* Store currents */

  model->state.i_dq.q = model->iq_int;
  model->state.i_dq.d = model->id_int;

  /* Inverse Park transform - get alpha-beta current */

  inv_park_transform(&model->state.angle.angle_el,
                     &model->state.i_dq,
                     &model->state.i_ab);

  /* Inverse Clarke transform - get abc current */

  inv_clarke_transform(&model->state.i_ab,
                       &model->state.i_abc);

  return OK;
}

/****************************************************************************
 * Name: pmsm_model_mech
 *
 * Description:
 *  Update motor model mechanical state
 *
 ****************************************************************************/

int pmsm_model_mech(FAR struct pmsm_model_f32_s *model, float load)
{
  float angle = 0.0f;
  float dir   = 0.0f;
  float te    = 0.0f;
  float tmp1  = 0.0f;
  float tmp2  = 0.0f;
  float tmp3  = 0.0f;
  float tmp4  = 0.0f;
  float tmp5  = 0.0f;

  DEBUGASSERT(model);

  /* Get electrical torque developed by the motor */

  tmp1 = model->phy.ind_d - model->phy.ind_q;
  tmp2 = tmp1 * model->state.i_dq.d;
  tmp3 = model->phy.motor.flux_link - tmp2;
  tmp4 = 1.5f * model->phy.motor.p;
  tmp5 = tmp4 * model->state.i_dq.q;

  te =  tmp5 * tmp3;

  /* Get new mechanical velocity */

  tmp1 = te - load;
  tmp2 = model->per * tmp1 ;
  tmp3 = tmp2 * model->phy.one_by_iner;

  model->state.omega_m = (model->state.omega_m + tmp3);

  /* Get new electrical velocity */

  model->state.omega_e = model->state.omega_m * model->phy.motor.p;

  /* Get rotation direction */

  dir = (model->state.omega_e > 0 ? DIR_CW : DIR_CCW);

  /* Update electrical angle */

  tmp1 = model->state.omega_e * model->per;

  angle = model->state.angle.angle_el.angle + tmp1;

  /* Update with mechanical angel */

  motor_angle_e_update(&model->state.angle, angle, dir);

  return OK;
}
