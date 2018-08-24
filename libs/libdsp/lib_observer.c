/****************************************************************************
 * control/lib_observer.c
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

#include <dsp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ANGLE_DIFF_THR M_PI_F

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_observer_init
 *
 * Description:
 *   Initialize motor observer
 *
 * Input Parameters:
 *   observer - pointer to the common observer data
 *   ao       - pointer to the angle specific observer data
 *   so       - pointer to the speed specific observer data
 *   per      - observer execution period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_observer_init(FAR struct motor_observer_s *observer,
                         FAR void *ao, FAR void *so, float per)
{
  DEBUGASSERT(observer != NULL);
  DEBUGASSERT(ao != NULL);
  DEBUGASSERT(so != NULL);
  DEBUGASSERT(per > 0.0f);

  /* Reset observer data */

  memset(observer, 0, sizeof(struct motor_observer_s));

  /* Set observer period */

  observer->per = per;

  /* Connect angle estimation observer data */

  observer->ao = ao;

  /* Connect speed estimation observer data */

  observer->so = so;
}

/****************************************************************************
 * Name: motor_observer_smo_init
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

void motor_observer_smo_init(FAR struct motor_observer_smo_s *smo,
                             float kslide,
                             float err_max)
{
  DEBUGASSERT(smo != NULL);
  DEBUGASSERT(kslide > 0.0f);
  DEBUGASSERT(err_max > 0.0f);

  /* Reset structure */

  memset(smo, 0, sizeof(struct motor_observer_smo_s));

  /* Initialize structure */

  smo->k_slide = kslide;
  smo->err_max = err_max;
}

/****************************************************************************
 * Name: motor_observer_smo
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
 *   o      - (in/out) pointer to the common observer data
 *   i_ab   - (in) inverter alpha-beta current
 *   v_ab   - (in) inverter alpha-beta voltage
 *   phy    - (in) pointer to the motor physical parameters
 *   dir    - (in) rotation direction (1.0 for CW, -1.0 for CCW)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_observer_smo(FAR struct motor_observer_s *o, FAR ab_frame_t *i_ab,
                        FAR ab_frame_t *v_ab, FAR struct motor_phy_params_s *phy,
                        float dir)
{
  DEBUGASSERT(o != NULL);
  DEBUGASSERT(i_ab != NULL);
  DEBUGASSERT(v_ab != NULL);
  DEBUGASSERT(phy != NULL);

  FAR struct motor_observer_smo_s *smo =
    (FAR struct motor_observer_smo_s *)o->ao;
  FAR ab_frame_t *emf    = &smo->emf;
  FAR ab_frame_t *emf_f  = &smo->emf_f;
  FAR ab_frame_t *z      = &smo->z;
  FAR ab_frame_t *i_est  = &smo->i_est;
  FAR ab_frame_t *v_err  = &smo->v_err;
  FAR ab_frame_t *i_err  = &smo->i_err;
  FAR ab_frame_t *sign   = &smo->sign;
  float i_err_a_abs  = 0.0f;
  float i_err_b_abs  = 0.0f;
  float angle        = 0.0f;
  float filter       = 0.0f;

  /* REVISIT: observer works only when IQ current is high enough */

  /* Calculate observer gains */

  smo->F_gain = (1.0f - o->per*phy->res*phy->one_by_ind);
  smo->G_gain = o->per*phy->one_by_ind;

  /* Saturate F gain */

  if (smo->F_gain < 0.0f)
    {
      smo->F_gain = 0.0f;
    }

  /* Saturate G gain */

  if (smo->G_gain > 0.999f)
    {
      smo->G_gain = 0.999f;
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
   *   T          - [s] period at which the digital filter is being calculated
   *   f_in       - [Hz] input frequency of the filter
   *   f_c        - [Hz] cutoff frequency of the filter
   *   omega_m    - [rad/s] mechanical angular velocity
   *   omega_e    - [rad/s] electrical angular velocity
   *   pole_pairs - pole pairs
   *
   */

  filter = o->per * o->speed * phy->p;

  /* Limit SMO filters
   * REVISIT: lowest filter limit should depend on minimum speed:
   *          filter = T * (2*PI) * f_c = T * omega0
   *
   */

  if (filter >= 1.0f)
    {
      filter = 0.99f;
    }
  else if (filter <= 0.0f)
    {
      filter = 0.005f;
    }

  smo->emf_lp_filter1 = filter;
  smo->emf_lp_filter2 = smo->emf_lp_filter1;

  /* Get voltage error: v_err = v_ab - emf */

  v_err->a = v_ab->a - emf->a;
  v_err->b = v_ab->b - emf->b;

  /* Estimate stator current */

  i_est->a = smo->F_gain * i_est->a + smo->G_gain * (v_err->a - z->a);
  i_est->b = smo->F_gain * i_est->b + smo->G_gain * (v_err->b - z->b);

  /* Get motor current errror */

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

      z->a = i_err->a * smo->k_slide / smo->err_max;
    }
  else
    {
      /* Non-linear region */

      z->a = sign->a * smo->k_slide;
    }

  if (i_err_b_abs < smo->err_max)
    {
      /* Enter linear region if error is small enough */

      z->b = i_err->b * smo->k_slide / smo->err_max;
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
   */

  angle = fast_atan2(-emf->a, emf->b);

#if 1
  /* Some assertions
   * TODO: simplify
   */

  if (angle != angle) angle = 0.0f;
  if (emf->a != emf->a) emf->a = 0.0f;
  if (emf->b != emf->b) emf->b = 0.0f;
  if (z->a != z->a) z->a = 0.0f;
  if (z->b != z->b) z->b = 0.0f;
  if (i_est->a != i_est->a) i_est->a = 0.0f;
  if (i_est->b != i_est->b) i_est->b = 0.0f;
#endif

  /* Angle compensation.
   * Due to low pass filtering we have some delay in estimated phase angle.
   *
   * Adaptive filters introduced above cause -PI/4 phase shift for each filter.
   * We use 2 times filtering which give us constant -PI/2 (-90deg) phase shift.
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
 *   sample - (in) number of mechanical angle samples
 *   filter - (in) low-pass filter for final omega
 *   per    - (in) speed observer execution period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_sobserver_div_init(FAR struct motor_sobserver_div_s *so,
                              uint8_t samples,
                              float filter,
                              float per)
{
  DEBUGASSERT(so != NULL);
  DEBUGASSERT(samples > 0);
  DEBUGASSERT(filter > 0.0f);

  /* Reset observer data */

  memset(so, 0, sizeof(struct motor_sobserver_div_s));

  /* Store number of samples for DIV observer */

  so->samples = samples;

  /* Store low-pass filter for DIV observer speed */

  so->filter  = filter;

  /*  */

  so->one_by_dt = 1.0f/(so->samples * per);
}

/****************************************************************************
 * Name: motor_sobserver_div
 *
 * Description:
 *   Estimate motor mechanical speed based on motor mechanical angle
 *   difference.
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the common observer data
 *   angle  - (in) mechanical angle normalized to <0.0, 2PI>
 *   dir    - (in) mechanical rotation direction. Valid values:
 *                 DIR_CW (1.0f) or DIR_CCW(-1.0f)
 *
 ****************************************************************************/

void motor_sobserver_div(FAR struct motor_observer_s *o,
                          float angle, float dir)
{
  DEBUGASSERT(o != NULL);
  DEBUGASSERT(angle >= 0.0f && angle <= 2*M_PI_F);
  DEBUGASSERT(dir == DIR_CW || dir == DIR_CCW);

  FAR struct motor_sobserver_div_s *so =
    (FAR struct motor_sobserver_div_s *)o->so;
  volatile float omega = 0.0f;

  /* Get angle diff */

  so->angle_diff = angle - so->angle_prev;

  /* Correct angle if we crossed angle boundary
   * REVISIT:
   */

  if ((dir == DIR_CW && so->angle_diff < -ANGLE_DIFF_THR) ||
      (dir == DIR_CCW && so->angle_diff > ANGLE_DIFF_THR))
    {
      /* Correction sign depends on rotation direction */

      so->angle_diff += dir*2*M_PI_F;
    }

  /* Get absoulte value */

  if (so->angle_diff < 0.0f)
    {
      so->angle_diff = -so->angle_diff;
    }

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
       *             T      - speed estimation period (samples*one_by_dt)
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
 * Name: motor_observer_speed_get
 *
 * Description:
 *   Get the estmiated motor mechanical speed from the observer
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the common observer data
 *
 * Returned Value:
 *   Return estimated motor mechanical speed from observer
 *
 ****************************************************************************/

float motor_observer_speed_get(FAR struct motor_observer_s *o)
{
  DEBUGASSERT(o != NULL);

  return o->speed;
}

/****************************************************************************
 * Name: motor_observer_angle_get
 *
 * Description:
 *   Get the estmiated motor electrical angle from the observer
 *
 * Input Parameters:
 *   o      - (in/out) pointer to the common observer data
 *
 * Returned Value:
 *   Return estimated motor mechanical angle from observer
 *
 ****************************************************************************/

float motor_observer_angle_get(FAR struct motor_observer_s *o)
{
  DEBUGASSERT(o != NULL);

  return o->angle;
}
