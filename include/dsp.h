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
#include <stdint.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some math constants *********************************************************/

#define SQRT3_BY_TWO_F     ((float)0.866025)
#define SQRT3_BY_THREE_F   ((float)0.57735)
#define ONE_BY_SQRT3_F     ((float)0.57735)
#define TWO_BY_SQRT3_F     ((float)1.15470)

/* Some useful macros ***************************************************************/

/* Simple single-pole digital low pass filter:
 *   Y(n) = (1-beta)*Y(n-1) + beta*X(n) = (beta * (Y(n-1) - X(n)))
 *
 *   filter - (0.0 - 1.0) where 1.0 gives unfiltered values
 *   filter = T * (2*PI) * f_c
 *
 *   phase shift = -arctan(f_in/f_c)
 *
 *   T    - period at which the digital filter is being calculated
 *   f_in - input frequency of the filter
 *   f_c  - cutoff frequency of the filter
 *
 * REFERENCE: https://www.embeddedrelated.com/showarticle/779.php
 *
 */

#define LP_FILTER(val, sample, filter) val -= (filter * (val - sample))

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
  float       part[3]           /* 0 - proporitonal part
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
};

/* Common motor observer structure */

struct motor_observer_s
{
  float angle;               /* Estimated observer angle */
  float speed;               /* Estimated observer speed */
  float per;                 /* Observer execution period */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *so;                  /* Speed estimation observer data */
  void *ao;                  /* Angle estimation observer data */
};

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
  uint8_t poles;               /* Number of the motor poles */
  float   res;                 /* Phase-to-neutral temperature compensated resistance */
  float   ind;                 /* Average phase-to-neutral inductance */

  float   res_base;            /* Phase-to-neutral base resistance */
  float   res_alpha;           /* Temperature coefficient of resistance */
  float   res_temp_ref;        /* Reference temperature of alpha */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Math functions */

float fast_sin(float angle);
float fast_sin2(float angle);
float fast_cos(float angle);
float fast_cos2(float angle);
float fast_atan2(float y, float x);
void f_saturate(FAR float *val, float min, float max);
void vector2d_saturate(FAR float *x, FAR float *y, float max);
void dq_saturate(FAR dq_frame_t *dq, float max);

/* PID controller functions */

void pid_controller_init(FAR pid_controller_t *pid, float KP, float KI, float KD);
void pi_controller_init(FAR pid_controller_t *pid, float KP, float KI);
void pid_saturation_set(FAR pid_controller_t *pid, float min, float max);
void pi_saturation_set(FAR pid_controller_t *pid, float min, float max);
float pi_controller(FAR pid_controller_t *pid, float err);
float pid_controller(FAR pid_controller_t *pid, float err);

/* Transformation functions */

void clarke_transform(FAR abc_frame_t *abc, FAR ab_frame_t *ab);
void inv_clarke_transform(FAR ab_frame_t *ab, FAR abc_frame_t *abc);
void park_transform(FAR phase_angle_t *angle, FAR ab_frame_t *ab,
                    FAR dq_frame_t *dq);
void inv_park_transform(FAR phase_angle_t *angle, FAR dq_frame_t *dq,
                        FAR ab_frame_t *ab);
void angle_norm(FAR float *angle, float per, float bottom, float top);
void angle_norm_2pi(FAR float *angle, float bottom, float top);
void phase_angle_update(FAR struct phase_angle_s *angle, float val);

/* 3-phase system space vector modulation*/

void svm3(FAR struct svm3_state_s *s, FAR ab_frame_t *ab);

/* Field Oriented control */

void foc_current_control(FAR pid_controller_t *id_pid,
                         FAR pid_controller_t *iq_pid,
                         FAR dq_frame_t *idq_ref,
                         FAR dq_frame_t *idq,
                         FAR float_sat_t *sat,
                         FAR dq_frame_t *v_dq);

/* BLDC/PMSM motor observers */

void motor_observer_init(FAR struct motor_observer_s *observer,
                         FAR void *ao, FAR void *so, float per);

void motor_observer_smo_init(FAR struct motor_observer_smo_s *smo,
                             float kslide, float err_max);
void motor_observer_smo(FAR struct motor_observer_s *observer,
                        FAR ab_frame_t *i_ab, FAR ab_frame_t *v_ab,
                        FAR struct motor_phy_params_s *phy);


#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#endif /* __INCLUDE_DSP_H */
