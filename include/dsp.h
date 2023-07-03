/****************************************************************************
 * include/dsp.h
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

#ifndef __INCLUDE_DSP_H
#define __INCLUDE_DSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Disable DEBUGASSERT macro if LIBDSP debug is not enabled */

#ifdef CONFIG_LIBDSP_DEBUG
#  ifndef CONFIG_DEBUG_ASSERTIONS
#    warning "Need CONFIG_DEBUG_ASSERTIONS to work properly"
#  endif
#  define LIBDSP_DEBUGASSERT(x) DEBUGASSERT(x)
#else
#  undef LIBDSP_DEBUGASSERT
#  define LIBDSP_DEBUGASSERT(x)
#endif

#ifndef CONFIG_LIBDSP_PRECISION
#  define CONFIG_LIBDSP_PRECISION 0
#endif

/* Phase rotation direction */

#define DIR_NONE (0.0f)
#define DIR_CW   (1.0f)
#define DIR_CCW  (-1.0f)

/* Some math constants ******************************************************/

#define SQRT3_BY_TWO_F     (0.866025f)
#define SQRT3_BY_THREE_F   (0.57735f)
#define ONE_BY_SQRT3_F     (0.57735f)
#define TWO_BY_SQRT3_F     (1.15470f)

/* Some lib constants *******************************************************/

/* These are defined only in the NuttX math library */

#ifndef M_PI_F
#define M_PI_F    ((float)M_PI)
#endif

#ifndef M_PI_2_F
#define M_PI_2_F  ((float)M_PI_2)
#endif

/* Motor electrical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_E_MAX    (2.0f*M_PI_F)
#define MOTOR_ANGLE_E_MIN    (0.0f)
#define MOTOR_ANGLE_E_RANGE  (MOTOR_ANGLE_E_MAX - MOTOR_ANGLE_E_MIN)

/* Motor mechanical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_M_MAX    (2.0f*M_PI_F)
#define MOTOR_ANGLE_M_MIN    (0.0f)
#define MOTOR_ANGLE_M_RANGE  (MOTOR_ANGLE_M_MAX - MOTOR_ANGLE_M_MIN)

/* Some useful macros *******************************************************/

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
 *   max possible phase voltage for 3-phase power inverter:
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

struct phase_angle_f32_s
{
  float   angle;               /* Phase angle in radians <0, 2PI> */
  float   sin;                 /* Phase angle sine */
  float   cos;                 /* Phase angle cosine */
};

typedef struct phase_angle_f32_s phase_angle_f32_t;

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

struct motor_angle_f32_s
{
  phase_angle_f32_t angle_el;  /* Electrical angle */
  float             anglem;    /* Mechanical angle in radians <0, 2PI> */
  float             one_by_p;  /* Aux variable */
  uint8_t           p;         /* Number of the motor pole pairs */
  int8_t            i;         /* Pole counter */
};

/* Float number saturaton */

struct float_sat_f32_s
{
  float min;                    /* Lower limit */
  float max;                    /* Upper limit */
};

typedef struct float_sat_f32_s float_sat_f32_t;

/* PI/PID controller state structure */

struct pid_controller_f32_s
{
  bool            aw_en;       /* Integral part decay if saturated */
  bool            ireset_en;   /* Intergral part reset if saturated */
  bool            pisat_en;    /* PI saturation enabled */
  bool            pidsat_en;   /* PID saturation enabled */
  bool            _res;        /* Reserved */
  float           out;         /* Controller output */
  float_sat_f32_t sat;         /* Output saturation */
  float           err;         /* Current error value */
  float           err_prev;    /* Previous error value */
  float           KP;          /* Proportional coefficient */
  float           KI;          /* Integral coefficient */
  float           KD;          /* Derivative coefficient */
  float           part[3];     /* 0 - proporitonal part
                                * 1 - integral part
                                * 2 - derivative part
                                */
  float           KC;          /* Integral anti-windup decay coefficient */
  float           aw;          /* Integral anti-windup decay part */
};

typedef struct pid_controller_f32_s pid_controller_f32_t;

/* This structure represents the ABC frame (3 phase vector) */

struct abc_frame_f32_s
{
  float a;                     /* A component */
  float b;                     /* B component */
  float c;                     /* C component */
};

typedef struct abc_frame_f32_s abc_frame_f32_t;

/* This structure represents the alpha-beta frame (2 phase vector) */

struct ab_frame_f32_s
{
  float a;                     /* Alpha component */
  float b;                     /* Beta component */
};

typedef struct ab_frame_f32_s ab_frame_f32_t;

/* This structure represent the direct-quadrature frame */

struct dq_frame_f32_s
{
  float d;                     /* Driect component */
  float q;                     /* Quadrature component */
};

typedef struct dq_frame_f32_s dq_frame_f32_t;

/* Space Vector Modulation data for 3-phase system */

struct svm3_state_f32_s
{
  uint8_t     sector;          /* Current space vector sector */
  float       d_u;             /* Duty cycle for phase U */
  float       d_v;             /* Duty cycle for phase V */
  float       d_w;             /* Duty cycle for phase W */
};

/* Motor open-loop control data */

struct openloop_data_f32_s
{
  float angle;         /* Open-loop current angle normalized to <0.0, 2PI> */
  float per;           /* Open-loop control execution period */
};

/* Common motor speed observer structure */

struct motor_sobserver_f32_s
{
  float speed;             /* Estimated observer speed */
  float per;               /* Observer execution period */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *so;                  /* Speed estimation observer data */
};

/* Common motor angle observer structure */

struct motor_aobserver_f32_s
{
  float angle;             /* Estimated observer angle */
  float per;               /* Observer execution period */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *ao;                  /* Angle estimation observer data */
};

/* Speed observer division method data */

struct motor_sobserver_div_f32_s
{
  float angle_diff;           /* Angle difference */
  float angle_acc;            /* Accumulated angle */
  float angle_prev;           /* Previous angle */
  float one_by_dt;            /* Frequency of observer execution */
  float cntr;                 /* Sample counter */
  float samples;              /* Number of samples for observer */
  float filter;               /* Low-pass filter for final omega */
};

/* Speed observer PLL method data */

struct motor_sobserver_pll_f32_s
{
  float pll_phase;
  float pll_kp;
  float pll_ki;
};

/* Motor Sliding Mode Observer private data */

struct motor_aobserver_smo_f32_s
{
  float k_slide;        /* Bang-bang controller gain */
  float err_max;        /* Linear mode threshold */
  float one_by_err_max; /* One by err_max */
  float F;              /* Current observer F gain (1-Ts*R/L) */
  float G;              /* Current observer G gain (Ts/L) */
  float emf_lp_filter1; /* Adaptive first low pass EMF filter */
  float emf_lp_filter2; /* Adaptive second low pass EMF filter */
  ab_frame_f32_t emf;   /* Estimated back-EMF */
  ab_frame_f32_t emf_f; /* Fitlered estimated back-EMF */
  ab_frame_f32_t z;     /* Correction factor */
  ab_frame_f32_t i_est; /* Estimated idq current */
  ab_frame_f32_t v_err; /* v_err = v_ab - emf */
  ab_frame_f32_t i_err; /* i_err = i_est - i_dq */
  ab_frame_f32_t sign;  /* Bang-bang controller sign */
};

/* Motor Nonlinear FluxLink Observer private data */

struct motor_aobserver_nfo_f32_s
{
  float x1;
  float x2;
};

/* FOC initialize data */

struct foc_initdata_f32_s
{
  float id_kp;                  /* KP for d current */
  float id_ki;                  /* KI for d current */
  float iq_kp;                  /* KP for q current */
  float iq_ki;                  /* KI for q current */
};

/* Field Oriented Control (FOC) data */

struct foc_data_f32_s
{
  abc_frame_f32_t  v_abc;    /* Voltage in ABC frame */
  ab_frame_f32_t   v_ab;     /* Voltage in alpha-beta frame */
  dq_frame_f32_t   v_dq;     /* Requested voltage in dq frame */
  ab_frame_f32_t   v_ab_mod; /* Modulation voltage normalized to
                              * magnitude (0.0, 1.0)
                              */

  abc_frame_f32_t  i_abc;    /* Current in ABC frame */
  ab_frame_f32_t   i_ab;     /* Current in alpha-beta frame */
  dq_frame_f32_t   i_dq;     /* Current in dq frame */
  dq_frame_f32_t   i_dq_err; /* DQ current error */

  dq_frame_f32_t   i_dq_ref; /* Requested current for the FOC
                              * current controller
                              */

  pid_controller_f32_t id_pid; /* Current d-axis component PI controller */
  pid_controller_f32_t iq_pid; /* Current q-axis component PI controller */

  float vdq_mag_max;         /* Maximum dq voltage magnitude */
  float vab_mod_scale;       /* Voltage alpha-beta modulation scale */

  phase_angle_f32_t   angle; /* Phase angle */
};

/* Motor physical parameters.
 * This data structure was designed to work with BLDC/PMSM motors,
 * but probably can be used to describe different types of motors.
 */

struct motor_phy_params_f32_s
{
  uint8_t p;                   /* Number of the motor pole pairs */
  float   flux_link;           /* Flux linkage */
  float   res;                 /* Average phase-to-neutral resistance */
  float   ind;                 /* Average phase-to-neutral inductance */
  float   one_by_ind;          /* Inverse phase-to-neutral inductance */
  float   one_by_p;            /* Inverse number of motor pole pairs */
};

/* PMSM motor physical parameters */

struct pmsm_phy_params_f32_s
{
  struct motor_phy_params_f32_s motor;       /* Motor common PHY */
  float                         iner;        /* Rotor inertia */
  float                         ind_d;       /* d-inductance */
  float                         ind_q;       /* q-inductance */
  float                         one_by_iner; /* One by inertia */
  float                         one_by_indd; /* One by Ld */
  float                         one_by_indq; /* One by Lq */
};

/* PMSM motor model state */

struct pmsm_model_state_f32_s
{
  /* Motor model phase current */

  abc_frame_f32_t i_abc;
  ab_frame_f32_t  i_ab;
  dq_frame_f32_t  i_dq;

  /* Motor model phase voltage */

  abc_frame_f32_t v_abc;
  ab_frame_f32_t  v_ab;
  dq_frame_f32_t  v_dq;

  /* Motor model angle */

  struct motor_angle_f32_s angle;

  /* Angular speed */

  float omega_e;
  float omega_m;
};

/* PMSM motor model external conditions */

struct pmsm_model_ext_f32_s
{
  float load;                /* Motor model load torque */
};

/* PMSM motor model */

struct pmsm_model_f32_s
{
  struct pmsm_phy_params_f32_s  phy;    /* Motor model physical parameters */
  struct pmsm_model_state_f32_s state;  /* Motor model state */
  struct pmsm_model_ext_f32_s   ext;    /* Motor model external conditions */
  float                         per;    /* Control period */
  float                         id_int; /* Id integral part */
  float                         iq_int; /* Iq integral part */
};

/* Average filter */

struct avg_filter_data_s
{
  float prev_avg;      /* Previous average */
  float k;             /* k counter */
};

/****************************************************************************
 * Public Functions Prototypes
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

void dq_saturate(FAR dq_frame_f32_t *dq, float max);
float dq_mag(FAR dq_frame_f32_t *dq);

/* PID controller functions */

void pid_controller_init(FAR pid_controller_f32_t *pid,
                         float KP, float KI, float KD);
void pi_controller_init(FAR pid_controller_f32_t *pid,
                        float KP, float KI);
void pid_saturation_set(FAR pid_controller_f32_t *pid, float min, float max);
void pi_saturation_set(FAR pid_controller_f32_t *pid, float min, float max);
void pid_integral_reset(FAR pid_controller_f32_t *pid);
void pi_integral_reset(FAR pid_controller_f32_t *pid);
float pi_controller(FAR pid_controller_f32_t *pid, float err);
float pid_controller(FAR pid_controller_f32_t *pid, float err);
void pi_antiwindup_enable(FAR pid_controller_f32_t *pid, float KC,
                          bool enable);
void pi_ireset_enable(FAR pid_controller_f32_t *pid, bool enable);

/* Transformation functions */

void clarke_transform(FAR abc_frame_f32_t *abc, FAR ab_frame_f32_t *ab);
void inv_clarke_transform(FAR ab_frame_f32_t *ab, FAR abc_frame_f32_t *abc);
void park_transform(FAR phase_angle_f32_t *angle, FAR ab_frame_f32_t *ab,
                    FAR dq_frame_f32_t *dq);
void inv_park_transform(FAR phase_angle_f32_t *angle, FAR dq_frame_f32_t *dq,
                        FAR ab_frame_f32_t *ab);

/* Phase angle related functions */

void angle_norm(FAR float *angle, float per, float bottom, float top);
void angle_norm_2pi(FAR float *angle, float bottom, float top);
void phase_angle_update(FAR struct phase_angle_f32_s *angle, float val);

/* 3-phase system space vector modulation */

void svm3_init(FAR struct svm3_state_f32_s *s);
void svm3(FAR struct svm3_state_f32_s *s, FAR ab_frame_f32_t *ab);
void svm3_current_correct(FAR struct svm3_state_f32_s *s,
                          float *c0, float *c1, float *c2);

/* Field Oriented Control */

void foc_init(FAR struct foc_data_f32_s *foc,
              FAR struct foc_initdata_f32_s *init);
void foc_vbase_update(FAR struct foc_data_f32_s *foc, float vbase);
void foc_angle_update(FAR struct foc_data_f32_s *foc,
                      FAR phase_angle_f32_t *angle);
void foc_iabc_update(FAR struct foc_data_f32_s *foc,
                     FAR abc_frame_f32_t *i_abc);
void foc_voltage_control(FAR struct foc_data_f32_s *foc,
                         FAR dq_frame_f32_t *vdq_ref);
void foc_current_control(FAR struct foc_data_f32_s *foc,
                         FAR dq_frame_f32_t *idq_ref,
                         FAR dq_frame_f32_t *vdq_comp,
                         FAR dq_frame_f32_t *v_dq_ref);
void foc_vabmod_get(FAR struct foc_data_f32_s *foc,
                    FAR ab_frame_f32_t *v_ab_mod);
void foc_vdq_mag_max_get(FAR struct foc_data_f32_s *foc, FAR float *max);

/* BLDC/PMSM motor observers */

void motor_sobserver_init(FAR struct motor_sobserver_f32_s *observer,
                          FAR void *so, float per);
void motor_aobserver_init(FAR struct motor_aobserver_f32_s *observer,
                          FAR void *ao, float per);
float motor_sobserver_speed_get(FAR struct motor_sobserver_f32_s *o);
float motor_aobserver_angle_get(FAR struct motor_aobserver_f32_s *o);

void motor_aobserver_smo_init(FAR struct motor_aobserver_smo_f32_s *smo,
                              float kslide, float err_max);
void motor_aobserver_smo(FAR struct motor_aobserver_f32_s *o,
                         FAR ab_frame_f32_t *i_ab, FAR ab_frame_f32_t *v_ab,
                         FAR struct motor_phy_params_f32_s *phy, float dir,
                         float speed);

void motor_sobserver_div_init(FAR struct motor_sobserver_div_f32_s *so,
                              uint8_t samples, float filer, float per);
void motor_sobserver_div(FAR struct motor_sobserver_f32_s *o, float angle);

void motor_aobserver_nfo_init(FAR struct motor_aobserver_nfo_f32_s *nfo);
void motor_aobserver_nfo(FAR struct motor_aobserver_f32_s *o,
                         FAR ab_frame_f32_t *i_ab, FAR ab_frame_f32_t *v_ab,
                         FAR struct motor_phy_params_f32_s *phy, float gain);

void motor_sobserver_pll_init(FAR struct motor_sobserver_pll_f32_s *so,
                              float pll_kp, float pll_ki);
void motor_sobserver_pll(FAR struct motor_sobserver_f32_s *o, float angle);

/* Motor openloop control */

void motor_openloop_init(FAR struct openloop_data_f32_s *op, float per);
void motor_openloop(FAR struct openloop_data_f32_s *op, float speed,
                    float dir);
float motor_openloop_angle_get(FAR struct openloop_data_f32_s *op);

/* Motor angle */

void motor_angle_init(FAR struct motor_angle_f32_s *angle, uint8_t p);
void motor_angle_e_update(FAR struct motor_angle_f32_s *angle,
                          float angle_new, float dir);
void motor_angle_m_update(FAR struct motor_angle_f32_s *angle,
                          float angle_new, float dir);
float motor_angle_m_get(FAR struct motor_angle_f32_s *angle);
float motor_angle_e_get(FAR struct motor_angle_f32_s *angle);

/* Motor physical parameters */

void motor_phy_params_init(FAR struct motor_phy_params_f32_s *phy,
                           uint8_t poles, float res, float ind,
                           float fluxlink);

/* PMSM physical parameters functions */

void pmsm_phy_params_init(FAR struct pmsm_phy_params_f32_s *phy,
                          uint8_t poles, float res, float ind,
                          float iner, float flux,
                          float ind_d, float ind_q);

/* PMSM motor model */

int pmsm_model_initialize(FAR struct pmsm_model_f32_s *model,
                          FAR struct pmsm_phy_params_f32_s *phy,
                          float per);
int pmsm_model_elec(FAR struct pmsm_model_f32_s *model,
                    FAR ab_frame_f32_t *vab);
int pmsm_model_mech(FAR struct pmsm_model_f32_s *model, float load);

/* Average filter */

void avg_filter_data_init(FAR struct avg_filter_data_s *data,
                          float prev_avg, float k);
float avg_filter(FAR struct avg_filter_data_s *data, float x);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DSP_H */
