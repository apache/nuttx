/****************************************************************************
 * include/dsp.h
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

#ifndef __INCLUDE_DSP_H
#define __INCLUDE_DSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BUILD_FLAT
#  error "Only flat build supported for now"
#endif

/* Disable DEBUGASSERT macro if LIBDSP debug is not enabled */

#ifdef CONFIG_LIBDSP_DEBUG
#  ifndef CONFIG_DEBUG_ASSERTIONS
#    warning "Need CONFIG_DEBUG_ASSERTIONS to work properly"
#  endif
#else
#  undef DEBUGASSERT
#  define DEBUGASSERT(x)
#endif

#ifndef CONFIG_LIBDSP_PRECISION
#  define CONFIG_LIBDSP_PRECISION 0
#endif

#if !defined(CONFIG_LIBM) && !defined(CONFIG_ARCH_MATH_H)
#  error math.h not defined!
#endif

/* Phase rotation direction */

#define DIR_CW   (1.0f)
#define DIR_CCW  (-1.0f)

/* Some math constants *********************************************************/

#define SQRT3_BY_TWO_F     (0.866025f)
#define SQRT3_BY_THREE_F   (0.57735f)
#define ONE_BY_SQRT3_F     (0.57735f)
#define TWO_BY_SQRT3_F     (1.15470f)

/* Some lib constants **********************************************************/

/* Motor electrical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_E_MAX    (2.0f*M_PI_F)
#define MOTOR_ANGLE_E_MIN    (0.0f)
#define MOTOR_ANGLE_E_RANGE  (MOTOR_ANGLE_E_MAX - MOTOR_ANGLE_E_MIN)

/* Motor mechanical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_M_MAX    (2.0f*M_PI_F)
#define MOTOR_ANGLE_M_MIN    (0.0f)
#define MOTOR_ANGLE_M_RANGE  (MOTOR_ANGLE_M_MAX - MOTOR_ANGLE_M_MIN)

/* Some useful macros ***************************************************************/

/****************************************************************************
 * Name: LP_FILTER
 *
 * Description:
 *   Simple single-pole digital low pass filter:
 *     Y(n) = (1-beta)*Y(n-1) + beta*X(n) = (beta * (Y(n-1) - X(n)))
 *
 *     filter - (0.0 - 1.0) where 1.0 gives unfiltered values
 *     filter = T * (2*PI) * f_c
 *
 *     phase shift = -arctan(f_in/f_c)
 *
 *     T    - period at which the digital filter is being calculated
 *     f_in - input frequency of the filter
 *     f_c  - cutoff frequency of the filter
 *
 * REFERENCE: https://www.embeddedrelated.com/showarticle/779.php
 *
 ****************************************************************************/

#define LP_FILTER(val, sample, filter) val -= (filter * (val - sample))

/****************************************************************************
 * Name: SVM3_BASE_VOLTAGE_GET
 *
 * Description:
 *  Get maximum voltage for SVM3 without overmodulation
 *
 *  Notes:
 *   max possible phase voltage for 3-phase power inwerter:
 *     Vd = (2/3)*Vdc
 *   max phase reference voltage according to SVM modulation diagram:
 *     Vrefmax = Vd * cos(30*) = SQRT3_BY_2 * Vd
 *   which give us:
 *     Vrefmax = SQRT3_BY_3 * Vdc
 *
 *   Vdc - bus voltage
 *
 ****************************************************************************/

#define SVM3_BASE_VOLTAGE_GET(vbus) (vbus * SQRT3_BY_THREE_F)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents phase angle.
 * Besides angle value it also stores sine and cosine values for given angle.
 */

struct phase_angle_s
{
  float   angle;               /* Phase angle in radians <0, 2PI> */
  float   sin;                 /* Phase angle sine */
  float   cos;                 /* Phase angle cosine */
};

typedef struct phase_angle_s phase_angle_t;

/* This structure stores motor angles and corresponding sin and cos values
 *
 * th_el = th_m * pole_pairs
 * th_m = th_el/pole_pairs
 *
 * where:
 *   th_el      - motor electrical angle
 *   th_m       - motor mechanical angle
 *   pole_pairs - motor pole pairs
 *
 *  NOTE: pole_pairs = poles_total/2
 */

struct motor_angle_s
{
  phase_angle_t angle_el;      /* Electrical angle */
  float         anglem;        /* Mechanical angle in radians <0, 2PI> */
  float         one_by_p;      /* Aux variable */
  uint8_t       p;             /* Number of the motor pole pairs */
  int8_t        i;             /* Pole counter */
};

/* Float number saturaton */

struct float_sat_s
{
  float min;                    /* Lower limit */
  float max;                    /* Upper limit */
};

typedef struct float_sat_s float_sat_t;

/* PI/PID controler state structure */

struct pid_controller_s
{
  float       out;              /* Controller output */
  float_sat_t sat;              /* Output saturation */
  float       err;              /* Current error value */
  float       err_prev;         /* Previous error value */
  float       KP;               /* Proportional coefficient */
  float       KI;               /* Integral coefficient */
  float       KD;               /* Derivative coefficient */
  float       part[3];          /* 0 - proporitonal part
                                 * 1 - integral part
                                 * 2 - derivative part
                                 */
};

typedef struct pid_controller_s pid_controller_t;

/* This structure represents the ABC frame (3 phase vector) */

struct abc_frame_s
{
  float a;                     /* A component */
  float b;                     /* B component */
  float c;                     /* C component */
};

typedef struct abc_frame_s abc_frame_t;

/* This structure represents the alpha-beta frame (2 phase vector) */

struct ab_frame_s
{
  float a;                     /* Alpha component */
  float b;                     /* Beta component */
};

typedef struct ab_frame_s ab_frame_t;

/* This structure represent the direct-quadrature frame */

struct dq_frame_s
{
  float d;                     /* Driect component */
  float q;                     /* Quadrature component */
};

typedef struct dq_frame_s dq_frame_t;

/* Space Vector Modulation data for 3-phase system */

struct svm3_state_s
{
  uint8_t     sector;          /* Current space vector sector */
  float       d_u;             /* Duty cycle for phase U */
  float       d_v;             /* Duty cycle for phase V */
  float       d_w;             /* Duty cycle for phase W */
  float       d_max;           /* Duty cycle max */
  float       d_min;           /* Duty cycle min */
};

/* Motor open-loop control data */

struct openloop_data_s
{
  float max;           /* Open-loop max speed */
  float angle;         /* Open-loop current angle normalized to <0.0, 2PI> */
  float per;           /* Open-loop control execution period */
};

/* Common motor observer structure */

struct motor_observer_s
{
  float angle;               /* Estimated observer angle */
  float speed;               /* Estimated observer speed */
  float per;                 /* Observer execution period */

  float angle_err;           /* Observer angle error.
                              * This can be used to gradually eliminate
                              * error between openloop angle and observer angle
                              */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *so;                  /* Speed estimation observer data */
  void *ao;                  /* Angle estimation observer data */
};

/* Speed observer division method data */

struct motor_sobserver_div_s
{
  float angle_diff;             /* Mechanical angle difference */
  float angle_acc;              /* Accumulated mechanical angle */
  float angle_prev;             /* Previous mechanical angle */
  float one_by_dt;              /* Frequency of observer execution */
  float cntr;                   /* Sample counter */
  float samples;                /* Number of samples for observer */
  float filter;                 /* Low-pass filter for final omega */
};

/* Speed observer PLL method data */
#if 0
struct motor_sobserver_pll_s
{
  /* TODO */
};
#endif

/* Motor Sliding Mode Observer private data */

struct motor_observer_smo_s
{
  float k_slide;        /* Bang-bang controller gain */
  float err_max;        /* Linear mode threashold */
  float F_gain;         /* Current observer F gain (1-Ts*R/L) */
  float G_gain;         /* Current observer G gain (Ts/L) */
  float emf_lp_filter1; /* Adaptive first low pass EMF filter */
  float emf_lp_filter2; /* Adaptive second low pass EMF filter */
  ab_frame_t emf;       /* Estimated back-EMF */
  ab_frame_t emf_f;     /* Fitlered estimated back-EMF */
  ab_frame_t z;         /* Correction factor */
  ab_frame_t i_est;     /* Estimated idq current */
  ab_frame_t v_err;     /* v_err = v_ab - emf */
  ab_frame_t i_err;     /* i_err = i_est - i_dq */
  ab_frame_t sign;      /* Bang-bang controller sign */
};

/* Motor physical parameters.
 * This data structure was designed to work with BLDC/PMSM motors,
 * but probably can be used to describe different types of motors.
 */

struct motor_phy_params_s
{
  uint8_t p;                   /* Number of the motor pole pairs */

  float   res;                 /* Phase-to-neutral temperature compensated
                                * resistance
                                */
  float   res_base;            /* Phase-to-neutral base resistance */
  float   res_alpha;           /* Temperature coefficient of resistance */
  float   res_temp_ref;        /* Reference temperature of alpha */
  float   ind;                 /* Average phase-to-neutral inductance */
  float   one_by_ind;          /* Inverse phase-to-neutral inductance */
};

/* Field oriented control (FOC) data
 * REVISIT:
 */

struct foc_data_s
{
  abc_frame_t      v_abc;    /* Voltage in ABC frame */
  ab_frame_t       v_ab;     /* Voltage in alpha-beta frame */
  dq_frame_t       v_dq;     /* Voltage in dq frame */
  ab_frame_t       v_ab_mod; /* Modulation voltage normalized to
                              * magnitude (0.0, 1.0)
                              */

  abc_frame_t      i_abc;    /* Current in ABC frame */
  ab_frame_t       i_ab;     /* Current in apha-beta frame*/
  dq_frame_t       i_dq;     /* Current in dq frame */
  dq_frame_t       i_dq_err; /* DQ current error */

  dq_frame_t       i_dq_ref; /* Current dq reference frame */
  pid_controller_t id_pid;   /* Current d-axis component PI controller */
  pid_controller_t iq_pid;   /* Current q-axis component PI controller */

  float vdq_mag_max;         /* Maximum dq voltage magnitude */
  float vab_mod_scale;       /* Voltage alpha-beta modulation scale */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Math functions */

float fast_sin(float angle);
float fast_sin2(float angle);
float fast_cos(float angle);
float fast_cos2(float angle);
float fast_atan2(float y, float x);

void f_saturate(FAR float *val, float min, float max);

float vector2d_mag(float x, float y);
void vector2d_saturate(FAR float *x, FAR float *y, float max);

void dq_saturate(FAR dq_frame_t *dq, float max);
float dq_mag(FAR dq_frame_t *dq);

/* PID controller functions */

void pid_controller_init(FAR pid_controller_t *pid,
                         float KP, float KI, float KD);
void pi_controller_init(FAR pid_controller_t *pid,
                        float KP, float KI);
void pid_saturation_set(FAR pid_controller_t *pid, float min, float max);
void pi_saturation_set(FAR pid_controller_t *pid, float min, float max);
void pid_integral_reset(FAR pid_controller_t *pid);
void pi_integral_reset(FAR pid_controller_t *pid);
float pi_controller(FAR pid_controller_t *pid, float err);
float pid_controller(FAR pid_controller_t *pid, float err);

/* Transformation functions */

void clarke_transform(FAR abc_frame_t *abc, FAR ab_frame_t *ab);
void inv_clarke_transform(FAR ab_frame_t *ab, FAR abc_frame_t *abc);
void park_transform(FAR phase_angle_t *angle, FAR ab_frame_t *ab,
                    FAR dq_frame_t *dq);
void inv_park_transform(FAR phase_angle_t *angle, FAR dq_frame_t *dq,
                        FAR ab_frame_t *ab);

/* Phase angle related functions */

void angle_norm(FAR float *angle, float per, float bottom, float top);
void angle_norm_2pi(FAR float *angle, float bottom, float top);
void phase_angle_update(FAR struct phase_angle_s *angle, float val);

/* 3-phase system space vector modulation*/

void svm3_init(FAR struct svm3_state_s *s, float min, float max);
void svm3(FAR struct svm3_state_s *s, FAR ab_frame_t *ab);
void svm3_current_correct(FAR struct svm3_state_s *s,
                          int32_t *c0, int32_t *c1, int32_t *c2);

/* Field Oriented control */

void foc_vbase_update(FAR struct foc_data_s *foc, float vbase);
void foc_idq_ref_set(FAR struct foc_data_s *data, float d, float q);

void foc_init(FAR struct foc_data_s *data,
              float id_kp, float id_ki, float iq_kp, float iq_ki);
void foc_process(FAR struct foc_data_s *foc,
                 FAR abc_frame_t *i_abc,
                 FAR phase_angle_t *angle);

/* BLDC/PMSM motor observers */

void motor_observer_init(FAR struct motor_observer_s *observer,
                         FAR void *ao, FAR void *so, float per);
float motor_observer_speed_get(FAR struct motor_observer_s *o);
float motor_observer_angle_get(FAR struct motor_observer_s *o);

void motor_observer_smo_init(FAR struct motor_observer_smo_s *smo,
                             float kslide, float err_max);
void motor_observer_smo(FAR struct motor_observer_s *o,
                        FAR ab_frame_t *i_ab, FAR ab_frame_t *v_ab,
                        FAR struct motor_phy_params_s *phy, float dir);

void motor_sobserver_div_init(FAR struct motor_sobserver_div_s *so,
                              uint8_t samples, float filer, float per);
void motor_sobserver_div(FAR struct motor_observer_s *o,
                         float angle, float dir);

/* Motor openloop control */

void motor_openloop_init(FAR struct openloop_data_s *op,
                         float max, float per);
void motor_openloop(FAR struct openloop_data_s *op, float speed, float dir);
float motor_openloop_angle_get(FAR struct openloop_data_s *op);

/* Motor angle */

void motor_angle_init(FAR struct motor_angle_s *angle, uint8_t p);
void motor_angle_e_update(FAR struct motor_angle_s *angle,
                          float angle_new,float dir);
void motor_angle_m_update(FAR struct motor_angle_s *angle,
                          float angle_new,float dir);
float motor_angle_m_get(FAR struct motor_angle_s *angle);
float motor_angle_e_get(FAR struct motor_angle_s *angle);

/* Motor physical parameters functions */

void motor_phy_params_init(FAR struct motor_phy_params_s *phy, uint8_t poles,
                            float res, float ind);
void motor_phy_params_temp_set(FAR struct motor_phy_params_s *phy,
                               float res_alpha, float res_temp_ref);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DSP_H */
