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

#include <stddef.h>
#include <assert.h>

#include <dsp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  DEBUGASSERT(per > 0.0);

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
  DEBUGASSERT(kslide > 0.0);
  DEBUGASSERT(err_max > 0.0);

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
 *   observer - (in/out) pointer to the common observer data
 *   i_ab     - (in) inverter alpha-beta current
 *   v_ab     - (in) inverter alpha-beta voltage
 *   phy      - (in) pointer to the motor physical parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void motor_observer_smo(FAR struct motor_observer_s *observer, FAR ab_frame_t *i_ab,
                  FAR ab_frame_t *v_ab, FAR struct motor_phy_params_s *phy)
{
  FAR struct motor_observer_smo_s *smo =
    (FAR struct motor_observer_smo_s *)observer->ao;
  FAR ab_frame_t *emf    = &smo->emf;
  FAR ab_frame_t *z      = &smo->z;
  FAR ab_frame_t *i_est  = &smo->i_est;
  FAR ab_frame_t *v_err  = &smo->v_err;
  FAR ab_frame_t *i_err  = &smo->i_err;
  FAR ab_frame_t *sign   = &smo->sign;
  float i_err_a_abs  = 0.0;
  float i_err_b_abs  = 0.0;
  float angle        = 0.0;

  /* REVISIT: observer works only when IQ current is high enough */

  /* Calculate observer gains */

  smo->F_gain = (1 - observer->per*phy->res/phy->ind);
  smo->G_gain = observer->per/phy->ind;

  /* Saturate F gain */

  if (smo->F_gain < 0)
    {
      smo->F_gain = 0.0;
    }

  /* Saturate G gain */

  if (smo->G_gain > 0.999)
    {
      smo->G_gain = 0.999;
    }

  /* Configure low pass filters
   *
   * We tune low-pass filters to achieve cutoff frequency equal to
   * input singal frequency. This gives us constant phase shift between
   * input and outpu signals equals to:
   *
   *   phi = -arctan(f_in/f_c) = -arctan(1) = -PI/2
   *
   * Input signal frequency is equal to the frequency of the motor currents,
   * which give us:
   *
   *   f_c = omega_e/(2*PI)
   *   omega_m = omega_e/poles
   *   f_c = omega_m*poles/(2*PI)
   *
   *   filter = T * (2*PI) * f_c
   *   filter = T * omega_m * poles
   *
   *   T       - [s] period at which the digital filter is being calculated
   *   f_in    - [Hz] input frequency of the filter
   *   f_c     - [Hz] cutoff frequency of the filter
   *   omega_m - [rad/s] mechanical angular velocity
   *   omega_e - [rad/s] electrical angular velocity
   *
   */

  smo->emf_lp_filter1 = observer->per * observer->speed * phy->poles;
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

  sign->a = (i_err->a > 0 ? 1.0 : -1.0);
  sign->b = (i_err->b > 0 ? 1.0 : -1.0);

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

  LP_FILTER(emf->a, emf->a, smo->emf_lp_filter2);
  LP_FILTER(emf->b, emf->b, smo->emf_lp_filter2);

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

  if (angle != angle) angle = 0.0;
  if (emf->a != emf->a) emf->a = 0.0;
  if (emf->b != emf->b) emf->b = 0.0;
  if (z->a != z->a) z->a = 0.0;
  if (z->b != z->b) z->b = 0.0;
  if (i_est->a != i_est->a) i_est->a = 0.0;
  if (i_est->b != i_est->b) i_est->b = 0.0;
#endif

  /* Angle compensation.
   * Due to low pass filtering we have some delay in estimated phase angle.
   *
   * Adaptive filters introduced above cause -PI/2 phase shift for each filter.
   * We use 2 times filtering which give us constant -PI phase shift.
   */

  angle = angle -M_PI_F;

  /* Normalize angle to range <0, 2PI> */

  angle_norm_2pi(&angle, 0.0, 2*M_PI_F);

  /* Store estimated angle in observer data*/

  observer->angle = angle;
}
