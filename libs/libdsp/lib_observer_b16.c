/****************************************************************************
 * libs/libdsp/lib_observer_b16.c
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

/* Squared */

#define SQ_B16(x)       (b16mulb16((x), (x)))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_sobserver_init_b16
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

void motor_sobserver_init_b16(FAR struct motor_sobserver_b16_s *observer,
                          FAR void *so, b16_t per)
{
  LIBDSP_DEBUGASSERT(observer != NULL);
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(per > 0);

  /* Reset observer data */

  memset(observer, 0, sizeof(struct motor_sobserver_b16_s));

  /* Set observer period */

  observer->per = per;

  /* Connect speed estimation observer data */

  observer->so = so;
}

/****************************************************************************
 * Name: motor_aobserver_init_b16
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

void motor_aobserver_init_b16(FAR struct motor_aobserver_b16_s *observer,
                          FAR void *ao, b16_t per)
{
  LIBDSP_DEBUGASSERT(observer != NULL);
  LIBDSP_DEBUGASSERT(ao != NULL);
  LIBDSP_DEBUGASSERT(per > 0);

  /* Reset observer data */

  memset(observer, 0, sizeof(struct motor_aobserver_b16_s));

  /* Set observer period */

  observer->per = per;

  /* Connect angle estimation observer data */

  observer->ao = ao;
}

/****************************************************************************
 * Name: motor_aobserver_smo_init_b16
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

void motor_aobserver_smo_init_b16(FAR struct motor_aobserver_smo_b16_s *smo,
                              b16_t kslide, b16_t err_max)
{
  LIBDSP_DEBUGASSERT(smo != NULL);
  LIBDSP_DEBUGASSERT(kslide > 0);
  LIBDSP_DEBUGASSERT(err_max > 0);

  /* Reset structure */

  memset(smo, 0, sizeof(struct motor_aobserver_smo_b16_s));

  /* Initialize structure */

  smo->k_slide = kslide;
  smo->err_max = err_max;

  /* Store inverted err_max to avoid division */

  smo->one_by_err_max = b16divb16(b16ONE, err_max);
}

/****************************************************************************
 * Name: motor_aobserver_smo_b16
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

void motor_aobserver_smo_b16(FAR struct motor_aobserver_b16_s *o,
                         FAR ab_frame_b16_t *i_ab, FAR ab_frame_b16_t *v_ab,
                         FAR struct motor_phy_params_b16_s *phy, b16_t dir,
                         b16_t speed)
{
  LIBDSP_DEBUGASSERT(o != NULL);
  LIBDSP_DEBUGASSERT(i_ab != NULL);
  LIBDSP_DEBUGASSERT(v_ab != NULL);
  LIBDSP_DEBUGASSERT(phy != NULL);

  FAR struct motor_aobserver_smo_b16_s *smo =
    (FAR struct motor_aobserver_smo_b16_s *)o->ao;
  FAR ab_frame_b16_t *emf    = &smo->emf;
  FAR ab_frame_b16_t *emf_f  = &smo->emf_f;
  FAR ab_frame_b16_t *z      = &smo->z;
  FAR ab_frame_b16_t *i_est  = &smo->i_est;
  FAR ab_frame_b16_t *v_err  = &smo->v_err;
  FAR ab_frame_b16_t *i_err  = &smo->i_err;
  FAR ab_frame_b16_t *sign   = &smo->sign;
  b16_t i_err_a_abs  = 0;
  b16_t i_err_b_abs  = 0;
  b16_t angle        = 0;
  b16_t filter       = 0;

  LIBDSP_DEBUGASSERT(smo != NULL);

  /* REVISIT: observer works only when IQ current is high enough
   * Lower IQ current -> lower K_SLIDE
   */

  /* Calculate observer gains */

  smo->F = (b16ONE - b16mulb16(b16mulb16(o->per, phy->res),
                               phy->one_by_ind));
  smo->G = b16mulb16(o->per, phy->one_by_ind);

  /* Saturate F gain */

  if (smo->F < 0)
    {
      smo->F = 0;
    }

  /* Saturate G gain */

  if (smo->G > ftob16(0.999f))
    {
      smo->G = ftob16(0.999f);
    }

  /* Configure low pass filters
   *
   * We tune low-pass filters to achieve cutoff frequency equal to
   * input singal frequency. This gives us constant phase shift between
   * input and outpu signals equals to:
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

  filter = b16mulb16(b16mulb16(o->per, speed), itob16(phy->p));

  /* Limit SMO filters
   * REVISIT: lowest filter limit should depend on minimum speed:
   *          filter = T * (2*PI) * f_c = T * omega0
   *
   */

  if (filter >= b16ONE)
    {
      filter = ftob16(0.99f);
    }
  else if (filter < ftob16(0.005f))
    {
      filter = ftob16(0.005f);
    }

  smo->emf_lp_filter1 = filter;
  smo->emf_lp_filter2 = smo->emf_lp_filter1;

  /* Get voltage error: v_err = v_ab - emf */

  v_err->a = v_ab->a - emf->a;
  v_err->b = v_ab->b - emf->b;

  /* Estimate stator current */

  i_est->a = (b16mulb16(smo->F, i_est->a) +
              b16mulb16(smo->G, (v_err->a - z->a)));
  i_est->b = (b16mulb16(smo->F, i_est->b) +
              b16mulb16(smo->G, (v_err->b - z->b)));

  /* Get motor current error */

  i_err->a = i_ab->a - i_est->a;
  i_err->b = i_ab->b - i_est->b;

  /* Slide-mode controller */

  sign->a = (i_err->a > 0 ? b16ONE : -b16ONE);
  sign->b = (i_err->b > 0 ? b16ONE : -b16ONE);

  /* Get current error absolute value - just multiply value with its sign */

  i_err_a_abs = b16mulb16(i_err->a, sign->a);
  i_err_b_abs = b16mulb16(i_err->b, sign->b);

  /* Calculate new output correction factor voltage */

  if (i_err_a_abs < smo->err_max)
    {
      /* Enter linear region if error is small enough */

      z->a = b16mulb16(b16mulb16(i_err->a, smo->k_slide),
                       smo->one_by_err_max);
    }
  else
    {
      /* Non-linear region */

      z->a = b16mulb16(sign->a, smo->k_slide);
    }

  if (i_err_b_abs < smo->err_max)
    {
      /* Enter linear region if error is small enough */

      z->b = b16mulb16(b16mulb16(i_err->b, smo->k_slide),
                       smo->one_by_err_max);
    }
  else
    {
      /* Non-linear region */

      z->b = b16mulb16(sign->b, smo->k_slide);
    }

  /* Filter z to obtain estimated emf */

  LP_FILTER_B16(emf->a, z->a, smo->emf_lp_filter1);
  LP_FILTER_B16(emf->b, z->b, smo->emf_lp_filter1);

  /* Filter emf one more time before angle stimation */

  LP_FILTER_B16(emf_f->a, emf->a, smo->emf_lp_filter2);
  LP_FILTER_B16(emf_f->b, emf->b, smo->emf_lp_filter2);

  /* Estimate phase angle according to:
   *   emf_a = -|emf| * sin(th)
   *   emf_b =  |emf| * cos(th)
   *   th = atan2(-emf_a, emf->b)
   *
   * NOTE: bottleneck but we can't do much more to optimise this
   */

  angle = fast_atan2_b16(-emf->a, emf->b);

  /* Angle compensation.
   * Due to low pass filtering we have some delay in estimated phase angle.
   *
   * Adaptive filters introduced above cause -PI/4 phase shift for each
   * filter. We use 2 times filtering which give us constant -PI/2 (-90deg)
   * phase shift.
   */

  angle = angle + b16mulb16(dir, b16HALFPI);

  /* Normalize angle to range <0, 2PI> */

  angle_norm_2pi_b16(&angle, 0, b16TWOPI);

  /* Store estimated angle in observer data */

  o->angle = angle;
}

/****************************************************************************
 * Name: motor_sobserver_div_init_b16
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

void motor_sobserver_div_init_b16(FAR struct motor_sobserver_div_b16_s *so,
                              uint8_t samples, b16_t filter, b16_t per)
{
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(samples > 0);
  LIBDSP_DEBUGASSERT(filter > 0);

  /* Reset observer data */

  memset(so, 0, sizeof(struct motor_sobserver_div_b16_s));

  /* Store number of samples for DIV observer */

  so->samples = samples;

  /* Store low-pass filter for DIV observer speed */

  so->filter  = filter;

  /* Store inverted sampling period */

  so->one_by_dt = b16divb16(b16ONE, b16muli(per, so->samples));
}

/****************************************************************************
 * Name: motor_sobserver_div_b16
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

void motor_sobserver_div_b16(FAR struct motor_sobserver_b16_s *o,
                             b16_t angle)
{
  LIBDSP_DEBUGASSERT(o != NULL);
  LIBDSP_DEBUGASSERT(angle >= 0 && angle <= b16TWOPI);

  FAR struct motor_sobserver_div_b16_s *so =
    (FAR struct motor_sobserver_div_b16_s *)o->so;
  volatile b16_t omega = 0;

  LIBDSP_DEBUGASSERT(so != NULL);

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi_b16(&angle, -b16PI, b16PI);

  /* Get angle diff */

  so->angle_diff = angle - so->angle_prev;

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi_b16(&so->angle_diff, -b16PI, b16PI);

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

      omega = b16mulb16(so->angle_acc, so->one_by_dt);

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

      LP_FILTER_B16(o->speed, omega, so->filter);

      /* Reset samples counter and accumulated angle */

      so->cntr = 0;
      so->angle_acc = 0;
    }

  /* Store current angle as previous angle */

  so->angle_prev = angle;
}

/****************************************************************************
 * Name: motor_aobserver_nfo_init_b16
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

void motor_aobserver_nfo_init_b16(FAR struct motor_aobserver_nfo_b16_s *nfo)
{
  LIBDSP_DEBUGASSERT(nfo != NULL);

  /* Reset structure */

  memset(nfo, 0, sizeof(struct motor_aobserver_nfo_b16_s));
}

/****************************************************************************
 * Name: motor_aobserver_nfo_b16
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

void motor_aobserver_nfo_b16(FAR struct motor_aobserver_b16_s *o,
                         FAR ab_frame_b16_t *i_ab, FAR ab_frame_b16_t *v_ab,
                         FAR struct motor_phy_params_b16_s *phy, b16_t gain)
{
  FAR struct motor_aobserver_nfo_b16_s *nfo =
                               (FAR struct motor_aobserver_nfo_b16_s *)o->ao;
  b16_t angle;
  b16_t err;
  b16_t x1_dot;
  b16_t x2_dot;

  b16_t l_ia = 0;
  b16_t l_ib = 0;
  b16_t r_ia = 0;
  b16_t r_ib = 0;

  LIBDSP_DEBUGASSERT(nfo != NULL);

  l_ia = b16mulb16(b16mulb16((b16ONE + b16HALF), phy->ind), i_ab->a);
  l_ib = b16mulb16(b16mulb16((b16ONE + b16HALF), phy->ind), i_ab->b);
  r_ia = b16mulb16(b16mulb16((b16ONE + b16HALF), phy->res), i_ab->a);
  r_ib = b16mulb16(b16mulb16((b16ONE + b16HALF), phy->res), i_ab->b);

  err = SQ_B16(phy->flux_link) - (SQ_B16(nfo->x1 - l_ia) +
                                  SQ_B16(nfo->x2 - l_ib));

  /* Forcing this term to stay negative helps convergence according to
   * http://cas.ensmp.fr/Publications/Publications/Papers/
   * ObserverPermanentMagnet.pdf and
   * https://arxiv.org/pdf/1905.00833.pdf
   */

  if (err > 0)
    {
      err = 0;
    }

  x1_dot = -r_ia + v_ab->a + b16mulb16(b16mulb16(gain, (nfo->x1 - l_ia)),
                                       err);
  x2_dot = -r_ib + v_ab->b + b16mulb16(b16mulb16(gain, (nfo->x2 - l_ib)),
                                       err);
  nfo->x1 += b16mulb16(x1_dot, o->per);
  nfo->x2 += b16mulb16(x2_dot, o->per);

  /* Prevent the magnitude from getting too low
   * as that makes the angle very unstable.
   */

  if (vector2d_mag_b16(nfo->x1, nfo->x2) <
      (b16mulb16(phy->flux_link, b16HALF)))
    {
      nfo->x1 = b16mulb16((b16ONE + b16ONETENTH), nfo->x1);
      nfo->x2 = b16mulb16((b16ONE + b16ONETENTH), nfo->x2);
    }

  angle = fast_atan2_b16(nfo->x2 - l_ib, nfo->x1 - l_ia);

  /* Normalize angle to range <0, 2PI> */

  angle_norm_2pi_b16(&angle, 0, b16TWOPI);

  /* Store estimated angle in observer data */

  o->angle = angle;
}

/****************************************************************************
 * Name: motor_sobserver_pll_init_b16
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

void motor_sobserver_pll_init_b16(FAR struct motor_sobserver_pll_b16_s *so,
                              b16_t pll_kp, b16_t pll_ki)
{
  LIBDSP_DEBUGASSERT(so != NULL);
  LIBDSP_DEBUGASSERT(pll_kp > 0);
  LIBDSP_DEBUGASSERT(pll_ki > 0);

  /* Reset observer data */

  memset(so, 0, sizeof(struct motor_sobserver_pll_b16_s));

  /* Store kp for PLL observer */

  so->pll_kp = pll_kp;

  /* Store ki for PLL observer speed */

  so->pll_ki = pll_ki;
}

/****************************************************************************
 * Name: motor_sobserver_pll_b16
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

void motor_sobserver_pll_b16(FAR struct motor_sobserver_b16_s *o,
                             b16_t angle)
{
  FAR struct motor_sobserver_pll_b16_s *so =
      (FAR struct motor_sobserver_pll_b16_s *)o->so;
  b16_t delta_theta = 0;

  LIBDSP_DEBUGASSERT(so != NULL);

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi_b16(&angle, -b16PI, -b16PI);

  delta_theta = angle - so->pll_phase;

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi_b16(&delta_theta, -b16PI, -b16PI);

  so->pll_phase += b16mulb16((o->speed + b16mulb16(so->pll_kp, delta_theta)),
                             o->per);

  /* Normalize angle to range <-PI, PI> */

  angle_norm_2pi_b16(&so->pll_phase, -b16PI, -b16PI);

  o->speed += b16mulb16(b16mulb16(so->pll_ki, delta_theta), o->per);
}

/****************************************************************************
 * Name: motor_sobserver_speed_get_b16
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

b16_t motor_sobserver_speed_get_b16(FAR struct motor_sobserver_b16_s *o)
{
  LIBDSP_DEBUGASSERT(o != NULL);

  return o->speed;
}

/****************************************************************************
 * Name: motor_aobserver_angle_get_b16
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

b16_t motor_aobserver_angle_get_b16(FAR struct motor_aobserver_b16_s *o)
{
  LIBDSP_DEBUGASSERT(o != NULL);

  return o->angle;
}
