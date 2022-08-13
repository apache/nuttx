/****************************************************************************
 * libs/libdsp/lib_observer.c
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
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* nan check for floats */

#define IS_NAN(x)   ((x) != (x))
#define NAN_ZERO(x) (x = IS_NAN(x) ? 0.0 : x)

/* Squared */

#define SQ(x)       ((x) * (x))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_sobserver_init
 *
 * Description:
 *   Initialize motor speed observer
 *
 * Input Parameters:
 *   observer - pointer to the speed observer data
 *   so       - pointer to the speed specific observer data
 *   per      - observer execution period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_sobserver_init(FAR struct motor_sobserver_f32_s *observer,
                          FAR void *so, float per)
{
  LIBDSP_DEBUGASSERT(observer != NULL);
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(per > 0.0f);

  /* Reset observer data */

  memset(observer, 0, sizeof(struct motor_sobserver_f32_s));

  /* Set observer period */

  observer->per = per;

  /* Connect speed estimation observer data */

  observer->so = so;
}

/****************************************************************************
 * Name: motor_aobserver_init
 *
 * Description:
 *   Initialize motor angle observer
 *
 * Input Parameters:
 *   observer - pointer to the angle observer data
 *   ao       - pointer to the angle specific observer data
 *   per      - observer execution period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_aobserver_init(FAR struct motor_aobserver_f32_s *observer,
                          FAR void *ao, float per)
{
  LIBDSP_DEBUGASSERT(observer != NULL);
  LIBDSP_DEBUGASSERT(ao != NULL);
  LIBDSP_DEBUGASSERT(per > 0.0f);

  /* Reset observer data */

  memset(observer, 0, sizeof(struct motor_aobserver_f32_s));

  /* Set observer period */

  observer->per = per;

  /* Connect angle estimation observer data */

  observer->ao = ao;
}

/****************************************************************************
 * Name: motor_aobserver_smo_init
 *
 * Description:
 *   Initialize motor sliding mode observer.
 *
 * Input Parameters:
 *   smo     - pointer to the sliding mode observer private data
 *   kslide  - SMO gain
 *   err_max - linear region upper limit
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_aobserver_smo_init(FAR struct motor_aobserver_smo_f32_s *smo,
                              float kslide, float err_max)
{
  LIBDSP_DEBUGASSERT(smo != NULL);
  LIBDSP_DEBUGASSERT(kslide > 0.0f);
  LIBDSP_DEBUGASSERT(err_max > 0.0f);

  /* Reset structure */

  memset(smo, 0, sizeof(struct motor_aobserver_smo_f32_s));

  /* Initialize structure */

  smo->k_slide = kslide;
  smo->err_max = err_max;

  /* Store inverted err_max to avoid division */

  smo->one_by_err_max = (1.0f / err_max);
}

/****************************************************************************
 * Name: motor_aobserver_smo
 *
 * Description:
 *  One step of the SMO observer.
 *  REFERENCE: http://ww1.microchip.com/downloads/en/AppNotes/01078B.pdf
 *
 *  Below some theoretical backgrounds about SMO.
 *
 *  The digitalized motor model can be represent as:
 *
 *    d(i_s.)/dt = (-R/L)*i_s. + (1/L)*(v_s - e_s. - z)
 *
 *  We compare estimated current (i_s.) with measured current (i_s):
 *
 *    err = i_s. - i_s
 *
 *  and get correction factor (z):
 *
 *    sign = sing(err)
 *    z = sign*K_SLIDE
 *
 *  Once the digitalized model is compensated, we estimate BEMF (e_s.) by
 *  filtering z:
 *
 *    e_s. = low_pass(z)
 *
 *  The estimated BEMF is filtered once again and used to approximate the
 *  motor angle:
 *
 *    e_filtered_s. = low_pass(e_s.)
 *    theta = arctan(-e_alpha/e_beta)
 *
 *  The estimated theta is phase-shifted due to low pass filtration, so we
 *  need some phase compensation. More details below.
 *
 *  where:
 *    v_s  - phase input voltage vector
 *    i_s. - estimated phase current vector
 *    i_s  - phase current vector
 *    e_s. - estimated phase BEMF vector
 *    R    - motor winding resistance
 *    L    - motor winding inductance
 *    z    - output correction factor voltage
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the angle observer data
 *   i_ab   - (in) inverter alpha-beta current
 *   v_ab   - (in) inverter alpha-beta voltage
 *   phy    - (in) pointer to the motor physical parameters
 *   dir    - (in) rotation direction (1.0 for CCW, -1.0 for CW)
 *            NOTE: (mechanical dir) = -(electrical dir)
 *   speed  - (in) electrical speed
 *            TODO: pass rotation direction with speed sign
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_aobserver_smo(FAR struct motor_aobserver_f32_s *o,
                         FAR ab_frame_f32_t *i_ab, FAR ab_frame_f32_t *v_ab,
                         FAR struct motor_phy_params_f32_s *phy, float dir,
                         float speed)
{
  LIBDSP_DEBUGASSERT(o != NULL);
  LIBDSP_DEBUGASSERT(i_ab != NULL);
  LIBDSP_DEBUGASSERT(v_ab != NULL);
  LIBDSP_DEBUGASSERT(phy != NULL);

  FAR struct motor_aobserver_smo_f32_s *smo =
    (FAR struct motor_aobserver_smo_f32_s *)o->ao;
  FAR ab_frame_f32_t *emf    = &smo->emf;
  FAR ab_frame_f32_t *emf_f  = &smo->emf_f;
  FAR ab_frame_f32_t *z      = &smo->z;
  FAR ab_frame_f32_t *i_est  = &smo->i_est;
  FAR ab_frame_f32_t *v_err  = &smo->v_err;
  FAR ab_frame_f32_t *i_err  = &smo->i_err;
  FAR ab_frame_f32_t *sign   = &smo->sign;
  float i_err_a_abs  = 0.0f;
  float i_err_b_abs  = 0.0f;
  float angle        = 0.0f;
  float filter       = 0.0f;

  LIBDSP_DEBUGASSERT(smo != NULL);

  /* REVISIT: observer works only when IQ current is high enough
   * Lower IQ current -> lower K_SLIDE
   */

  /* Calculate observer gains */

  smo->F = (1.0f - o->per * phy->res * phy->one_by_ind);
  smo->G = o->per * phy->one_by_ind;

  /* Saturate F gain */

  if (smo->F < 0.0f)
    {
      smo->F = 0.0f;
    }

  /* Saturate G gain */

  if (smo->G > 0.999f)
    {
      smo->G = 0.999f;
    }

  /* Configure low pass filters
   *
   * We tune low-pass filters to achieve cutoff frequency equal to
   * input signal frequency. This gives us constant phase shift between
   * input and output signals equals to:
   *
   *   phi = -arctan(f_in/f_c) = -arctan(1) = -45deg = -PI/4
   *
   * Input signal frequency is equal to the frequency of the motor currents,
   * which give us:
   *
   *   f_c = omega_e/(2*PI)
   *   omega_m = omega_e/pole_pairs
   *   f_c = omega_m*pole_pairs/(2*PI)
   *
   *   filter = T * (2*PI) * f_c
   *   filter = T * omega_m * pole_pairs
   *
   *   T          - [s] period at which the digital filter is being
   *                calculated
   *   f_in       - [Hz] input frequency of the filter
   *   f_c        - [Hz] cutoff frequency of the filter
   *   omega_m    - [rad/s] mechanical angular velocity
   *   omega_e    - [rad/s] electrical angular velocity
   *   pole_pairs - pole pairs
   *
   */

  filter = o->per * speed * phy->p;

  /* Limit SMO filters
   * REVISIT: lowest filter limit should depend on minimum speed:
   *          filter = T * (2*PI) * f_c = T * omega0
   *
   */

  if (filter >= 1.0f)
    {
      filter = 0.99f;
    }
  else if (filter < 0.005f)
    {
      filter = 0.005f;
    }

  smo->emf_lp_filter1 = filter;
  smo->emf_lp_filter2 = smo->emf_lp_filter1;

  /* Get voltage error: v_err = v_ab - emf */

  v_err->a = v_ab->a - emf->a;
  v_err->b = v_ab->b - emf->b;

  /* Estimate stator current */

  i_est->a = smo->F * i_est->a + smo->G * (v_err->a - z->a);
  i_est->b = smo->F * i_est->b + smo->G * (v_err->b - z->b);

  /* Get motor current error */

  i_err->a = i_ab->a - i_est->a;
  i_err->b = i_ab->b - i_est->b;

  /* Slide-mode controller */

  sign->a = (i_err->a > 0.0f ? 1.0f : -1.0f);
  sign->b = (i_err->b > 0.0f ? 1.0f : -1.0f);

  /* Get current error absolute value - just multiply value with its sign */

  i_err_a_abs = i_err->a * sign->a;
  i_err_b_abs = i_err->b * sign->b;

  /* Calculate new output correction factor voltage */

  if (i_err_a_abs < smo->err_max)
    {
      /* Enter linear region if error is small enough */

      z->a = i_err->a * smo->k_slide * smo->one_by_err_max;
    }
  else
    {
      /* Non-linear region */

      z->a = sign->a * smo->k_slide;
    }

  if (i_err_b_abs < smo->err_max)
    {
      /* Enter linear region if error is small enough */

      z->b = i_err->b * smo->k_slide * smo->one_by_err_max;
    }
  else
    {
      /* Non-linear region */

      z->b = sign->b * smo->k_slide;
    }

  /* Filter z to obtain estimated emf */

  LP_FILTER(emf->a, z->a, smo->emf_lp_filter1);
  LP_FILTER(emf->b, z->b, smo->emf_lp_filter1);

  /* Filter emf one more time before angle stimation */

  LP_FILTER(emf_f->a, emf->a, smo->emf_lp_filter2);
  LP_FILTER(emf_f->b, emf->b, smo->emf_lp_filter2);

  /* Estimate phase angle according to:
   *   emf_a = -|emf| * sin(th)
   *   emf_b =  |emf| * cos(th)
   *   th = atan2(-emf_a, emf->b)
   *
   * NOTE: bottleneck but we can't do much more to optimise this
   */

  angle = fast_atan2(-emf->a, emf->b);

  /* Angle compensation.
   * Due to low pass filtering we have some delay in estimated phase angle.
   *
   * Adaptive filters introduced above cause -PI/4 phase shift for each
   * filter. We use 2 times filtering which give us constant -PI/2 (-90deg)
   * phase shift.
   */

  angle = angle + dir * M_PI_2_F;

  /* Normalize angle to range <0, 2PI> */

  angle_norm_2pi(&angle, 0.0f, 2.0f*M_PI_F);

  /* Store estimated angle in observer data */

  o->angle = angle;
}

/****************************************************************************
 * Name: motor_sobserver_div_init
 *
 * Description:
 *   Initialize DIV speed observer
 *
 * Input Parameters:
 *   so     - (in/out) pointer to the DIV speed observer data
 *   sample - (in) number of angle samples
 *   filter - (in) low-pass filter for final omega
 *   per    - (in) speed observer execution period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_sobserver_div_init(FAR struct motor_sobserver_div_f32_s *so,
                              uint8_t samples, float filter, float per)
{
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(samples > 0);
  LIBDSP_DEBUGASSERT(filter > 0.0f);

  /* Reset observer data */

  memset(so, 0, sizeof(struct motor_sobserver_div_f32_s));

  /* Store number of samples for DIV observer */

  so->samples = samples;

  /* Store low-pass filter for DIV observer speed */

  so->filter  = filter;

  /* Store inverted sampling period */

  so->one_by_dt = 1.0f / (so->samples * per);
}

/****************************************************************************
 * Name: motor_sobserver_div
 *
 * Description:
 *   Estimate motor speed based on motor angle difference (electrical
 *   or mechanical)
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the speed observer data
 *   angle  - (in) angle normalized to <0.0, 2PI>
 *   dir    - (in) rotation direction. Valid values:
 *                 DIR_CW (1.0f) or DIR_CCW(-1.0f)
 *
 ****************************************************************************/

void motor_sobserver_div(FAR struct motor_sobserver_f32_s *o, float angle)
{
  LIBDSP_DEBUGASSERT(o != NULL);
  LIBDSP_DEBUGASSERT(angle >= 0.0f && angle <= 2*M_PI_F);

  FAR struct motor_sobserver_div_f32_s *so =
    (FAR struct motor_sobserver_div_f32_s *)o->so;
  volatile float omega = 0.0f;

  LIBDSP_DEBUGASSERT(so != NULL);

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi(&angle, -M_PI_F, M_PI_F);

  /* Get angle diff */

  so->angle_diff = angle - so->angle_prev;

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi(&so->angle_diff, -M_PI_F, M_PI_F);

  /* Accumulate angle only if sample is valid */

  so->angle_acc += so->angle_diff;

  /* Increase counter */

  so->cntr += 1;

  /* Accumulate angle until we get configured number of samples */

  if (so->cntr >= so->samples)
    {
      /* Estimate omega using accumulated angle samples.
       * In this case use simple estimation:
       *
       *   omega = delta_theta/delta_time
       *   speed_now = low_pass(omega)
       *
       */

      omega = so->angle_acc*so->one_by_dt;

      /* Store filtered omega.
       *
       * REVISIT: cut-off frequency for this filter should be
       *          (probably) set according to minimum supported omega:
       *
       *          filter = T * (2*PI) * f_c = T * omega0
       *
       *          where:
       *             omega0 - minimum angular speed
       *             T      - speed estimation period (samples*per)
       */

      LP_FILTER(o->speed, omega, so->filter);

      /* Reset samples counter and accumulated angle */

      so->cntr = 0;
      so->angle_acc = 0.0f;
    }

  /* Store current angle as previous angle */

  so->angle_prev = angle;
}

/****************************************************************************
 * Name: motor_aobserver_nfo_init
 *
 * Description:
 *   Initialize motor nolinear fluxlink observer.
 *
 * Input Parameters:
 *   nfo     - pointer to the nolinear fluxlink observer private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_aobserver_nfo_init(FAR struct motor_aobserver_nfo_f32_s *nfo)
{
  LIBDSP_DEBUGASSERT(nfo != NULL);

  /* Reset structure */

  memset(nfo, 0, sizeof(struct motor_aobserver_nfo_f32_s));
}

/****************************************************************************
 * Name: motor_aobserver_nfo
 *
 * Description:
 *  nolinear fluxlink observer.
 *  REFERENCE: http://cas.ensmp.fr/~praly/Telechargement/Journaux/
 *  2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the angle observer data
 *   i_ab   - (in) inverter alpha-beta current
 *   v_ab   - (in) inverter alpha-beta voltage
 *   phy    - (in) pointer to the motor physical parameters
 *   gain   - (in) dynamic observer gain
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_aobserver_nfo(FAR struct motor_aobserver_f32_s *o,
                         FAR ab_frame_f32_t *i_ab, FAR ab_frame_f32_t *v_ab,
                         FAR struct motor_phy_params_f32_s *phy, float gain)
{
  FAR struct motor_aobserver_nfo_f32_s *nfo =
                               (FAR struct motor_aobserver_nfo_f32_s *)o->ao;
  float angle;
  float err;
  float x1_dot;
  float x2_dot;

  float l_ia = (3.0f / 2.0f) * phy->ind * i_ab->a;
  float l_ib = (3.0f / 2.0f) * phy->ind * i_ab->b;
  float r_ia = (3.0f / 2.0f) * phy->res * i_ab->a;
  float r_ib = (3.0f / 2.0f) * phy->res * i_ab->b;

  LIBDSP_DEBUGASSERT(nfo != NULL);

  err = SQ(phy->flux_link) - (SQ(nfo->x1 - l_ia) + SQ(nfo->x2 - l_ib));

  /* Forcing this term to stay negative helps convergence according to
   * http://cas.ensmp.fr/Publications/Publications/Papers/
   * ObserverPermanentMagnet.pdf and
   * https://arxiv.org/pdf/1905.00833.pdf
   */

  if (err > 0.0f)
    {
      err = 0.0f;
    }

  x1_dot = -r_ia + v_ab->a + gain * (nfo->x1 - l_ia) * err;
  x2_dot = -r_ib + v_ab->b + gain * (nfo->x2 - l_ib) * err;
  nfo->x1 += x1_dot * o->per;
  nfo->x2 += x2_dot * o->per;

  NAN_ZERO(nfo->x1);
  NAN_ZERO(nfo->x2);

  /* Prevent the magnitude from getting too low
   * as that makes the angle very unstable.
   */

  if (vector2d_mag(nfo->x1, nfo->x2) < (phy->flux_link * 0.5))
    {
      nfo->x1 *= 1.1;
      nfo->x2 *= 1.1;
    }

  angle = fast_atan2(nfo->x2 - l_ib, nfo->x1 - l_ia);

  /* Normalize angle to range <0, 2PI> */

  angle_norm_2pi(&angle, 0.0f, 2.0f * M_PI_F);

  /* Store estimated angle in observer data */

  o->angle = angle;
}

/****************************************************************************
 * Name: motor_sobserver_pll_init
 *
 * Description:
 *   Initialize PLL speed observer
 *
 * Input Parameters:
 *   so     - (in/out) pointer to the PLL speed observer data
 *   pll_kp - (in) pll proportional gain
 *   pll_ki - (in) pll integral gain
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_sobserver_pll_init(FAR struct motor_sobserver_pll_f32_s *so,
                              float pll_kp, float pll_ki)
{
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(pll_kp > 0.0f);
  LIBDSP_DEBUGASSERT(pll_ki > 0.0f);

  /* Reset observer data */

  memset(so, 0, sizeof(struct motor_sobserver_pll_f32_s));

  /* Store kp for PLL observer */

  so->pll_kp = pll_kp;

  /* Store ki for PLL observer speed */

  so->pll_ki = pll_ki;
}

/****************************************************************************
 * Name: motor_sobserver_pll
 *
 * Description:
 *   Estimate motor electrical speed based on motor electrical angle
 *   difference.
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the speed observer data
 *   angle  - (in) electrical angle normalized to <0.0, 2PI>
 *
 ****************************************************************************/

void motor_sobserver_pll(FAR struct motor_sobserver_f32_s *o, float angle)
{
  FAR struct motor_sobserver_pll_f32_s *so =
      (FAR struct motor_sobserver_pll_f32_s *)o->so;
  float delta_theta = 0.0f;

  LIBDSP_DEBUGASSERT(so != NULL);

  NAN_ZERO(so->pll_phase);

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi(&angle, -M_PI_F, -M_PI_F);

  delta_theta = angle - so->pll_phase;

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi(&delta_theta, -M_PI_F, -M_PI_F);

  NAN_ZERO(o->speed);

  so->pll_phase += (o->speed + so->pll_kp * delta_theta) * o->per;

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi(&so->pll_phase, -M_PI_F, -M_PI_F);

  o->speed += so->pll_ki * delta_theta * o->per;
}

/****************************************************************************
 * Name: motor_sobserver_speed_get
 *
 * Description:
 *   Get the estmiated motor speed from the observer
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the speed observer data
 *
 * Returned Value:
 *   Return estimated motor speed from observer
 *
 ****************************************************************************/

float motor_sobserver_speed_get(FAR struct motor_sobserver_f32_s *o)
{
  LIBDSP_DEBUGASSERT(o != NULL);

  return o->speed;
}

/****************************************************************************
 * Name: motor_aobserver_angle_get
 *
 * Description:
 *   Get the estmiated motor electrical angle from the observer
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the angle observer data
 *
 * Returned Value:
 *   Return estimated motor electrical angle from observer
 *
 ****************************************************************************/

float motor_aobserver_angle_get(FAR struct motor_aobserver_f32_s *o)
{
  LIBDSP_DEBUGASSERT(o != NULL);

  return o->angle;
}
