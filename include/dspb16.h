/****************************************************************************
 * include/dspb16.h
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

#ifndef __INCLUDE_DSPB16_H
#define __INCLUDE_DSPB16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <fixedmath.h>

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

#define DIR_NONE_B16 ftob16(0.0f)
#define DIR_CW_B16   ftob16(1.0f)
#define DIR_CCW_B16  ftob16(-1.0f)

/* Some math constants ******************************************************/

#define SQRT3_BY_TWO_B16     ftob16(0.866025f)
#define SQRT3_BY_THREE_B16   ftob16(0.57735f)
#define ONE_BY_SQRT3_B16     ftob16(0.57735f)
#define TWO_BY_SQRT3_B16     ftob16(1.15470f)

/* Some lib constants *******************************************************/

/* Motor electrical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_E_MAX_B16    (b16TWOPI)
#define MOTOR_ANGLE_E_MIN_B16    (0)
#define MOTOR_ANGLE_E_RANGE_B16  (MOTOR_ANGLE_E_MAX_B16 - MOTOR_ANGLE_E_MIN_B16)

/* Motor mechanical angle is in range 0.0 to 2*PI */

#define MOTOR_ANGLE_M_MAX_B16    (b16TWOPI)
#define MOTOR_ANGLE_M_MIN_B16    (0)
#define MOTOR_ANGLE_M_RANGE_B16  (MOTOR_ANGLE_M_MAX_B16 - MOTOR_ANGLE_M_MIN_B16)

/* Some useful macros *******************************************************/

/****************************************************************************
 * Name: LP_FILTER_B16
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

#define LP_FILTER_B16(val, sample, filter) val -= (b16mulb16(filter, (val - sample)))

/****************************************************************************
 * Name: SVM3_BASE_VOLTAGE_GET_B16
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

#define SVM3_BASE_VOLTAGE_GET_B16(vbus) (b16mulb16(vbus, SQRT3_BY_THREE_B16))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents phase angle.
 * Besides angle value it also stores sine and cosine values for given angle.
 */

struct phase_angle_b16_s
{
  b16_t   angle;               /* Phase angle in radians <0, 2PI> */
  b16_t   sin;                 /* Phase angle sine */
  b16_t   cos;                 /* Phase angle cosine */
};

typedef struct phase_angle_b16_s phase_angle_b16_t;

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

struct motor_angle_b16_s
{
  phase_angle_b16_t angle_el;  /* Electrical angle */
  b16_t             anglem;    /* Mechanical angle in radians <0, 2PI> */
  b16_t             one_by_p;  /* Aux variable */
  uint8_t           p;         /* Number of the motor pole pairs */
  int8_t            i;         /* Pole counter */
};

/* Float number saturaton */

struct float_sat_b16_s
{
  b16_t min;                    /* Lower limit */
  b16_t max;                    /* Upper limit */
};

typedef struct float_sat_b16_s float_sat_b16_t;

/* PI/PID controller state structure */

struct pid_controller_b16_s
{
  bool            aw_en;       /* Integral part decay if saturated */
  bool            ireset_en;   /* Intergral part reset if saturated */
  bool            pisat_en;    /* PI saturation enabled */
  bool            pidsat_en;   /* PID saturation enabled */
  bool            _res;        /* Reserved */
  b16_t           out;         /* Controller output */
  float_sat_b16_t sat;         /* Output saturation */
  b16_t           err;         /* Current error value */
  b16_t           err_prev;    /* Previous error value */
  b16_t           KP;          /* Proportional coefficient */
  b16_t           KI;          /* Integral coefficient */
  b16_t           KD;          /* Derivative coefficient */
  b16_t           part[3];     /* 0 - proporitonal part
                                * 1 - integral part
                                * 2 - derivative part
                                */
  b16_t           KC;          /* Integral anti-windup decay coefficient */
  b16_t           aw;          /* Integral anti-windup decay part */
};

typedef struct pid_controller_b16_s pid_controller_b16_t;

/* This structure represents the ABC frame (3 phase vector) */

struct abc_frame_b16_s
{
  b16_t  a;                       /* A component */
  b16_t  b;                       /* B component */
  b16_t  c;                       /* C component */
};

typedef struct abc_frame_b16_s abc_frame_b16_t;

/* This structure represents the alpha-beta frame (2 phase vector) */

struct ab_frame_b16_s
{
  b16_t  a;                       /* Alpha component */
  b16_t  b;                       /* Beta component */
};

typedef struct ab_frame_b16_s ab_frame_b16_t;

/* This structure represent the direct-quadrature frame */

struct dq_frame_b16_s
{
  b16_t  d;                       /* Driect component */
  b16_t  q;                       /* Quadrature component */
};

typedef struct dq_frame_b16_s dq_frame_b16_t;

/* Space Vector Modulation data for 3-phase system */

struct svm3_state_b16_s
{
  uint8_t   sector;          /* Current space vector sector */
  b16_t     d_u;             /* Duty cycle for phase U */
  b16_t     d_v;             /* Duty cycle for phase V */
  b16_t     d_w;             /* Duty cycle for phase W */
};

/* Motor open-loop control data */

struct openloop_data_b16_s
{
  b16_t angle;         /* Open-loop current angle normalized to <0.0, 2PI> */
  b16_t per;           /* Open-loop control execution period */
};

/* Common motor speed observer structure */

struct motor_sobserver_b16_s
{
  b16_t speed;             /* Estimated observer speed */
  b16_t per;               /* Observer execution period */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *so;                  /* Speed estimation observer data */
};

/* Common motor angle observer structure */

struct motor_aobserver_b16_s
{
  b16_t angle;             /* Estimated observer angle */
  b16_t per;               /* Observer execution period */

  /* There are different types of motor observers which different
   * sets of private data.
   */

  void *ao;                  /* Angle estimation observer data */
};

/* Speed observer division method data */

struct motor_sobserver_div_b16_s
{
  b16_t angle_diff;           /* Angle difference */
  b16_t angle_acc;            /* Accumulated angle */
  b16_t angle_prev;           /* Previous angle */
  b16_t one_by_dt;            /* Frequency of observer execution */
  b16_t cntr;                 /* Sample counter */
  b16_t samples;              /* Number of samples for observer */
  b16_t filter;               /* Low-pass filter for final omega */
};

/* Speed observer PLL method data */

struct motor_sobserver_pll_b16_s
{
  b16_t pll_phase;
  b16_t pll_kp;
  b16_t pll_ki;
};

/* Motor Sliding Mode Observer private data */

struct motor_aobserver_smo_b16_s
{
  b16_t k_slide;        /* Bang-bang controller gain */
  b16_t err_max;        /* Linear mode threshold */
  b16_t one_by_err_max; /* One by err_max */
  b16_t F;              /* Current observer F gain (1-Ts*R/L) */
  b16_t G;              /* Current observer G gain (Ts/L) */
  b16_t emf_lp_filter1; /* Adaptive first low pass EMF filter */
  b16_t emf_lp_filter2; /* Adaptive second low pass EMF filter */
  ab_frame_b16_t emf;   /* Estimated back-EMF */
  ab_frame_b16_t emf_f; /* Fitlered estimated back-EMF */
  ab_frame_b16_t z;     /* Correction factor */
  ab_frame_b16_t i_est; /* Estimated idq current */
  ab_frame_b16_t v_err; /* v_err = v_ab - emf */
  ab_frame_b16_t i_err; /* i_err = i_est - i_dq */
  ab_frame_b16_t sign;  /* Bang-bang controller sign */
};

/* Motor Nonlinear FluxLink Observer private data */

struct motor_aobserver_nfo_b16_s
{
  b16_t x1;
  b16_t x2;
};

/* FOC initialize data */

struct foc_initdata_b16_s
{
  b16_t id_kp;                  /* KP for d current */
  b16_t id_ki;                  /* KI for d current */
  b16_t iq_kp;                  /* KP for q current */
  b16_t iq_ki;                  /* KI for q current */
};

/* Field Oriented Control (FOC) data */

struct foc_data_b16_s
{
  abc_frame_b16_t  v_abc;    /* Voltage in ABC frame */
  ab_frame_b16_t   v_ab;     /* Voltage in alpha-beta frame */
  dq_frame_b16_t   v_dq;     /* Requested voltage in dq frame */
  ab_frame_b16_t   v_ab_mod; /* Modulation voltage normalized to
                              * magnitude (0.0, 1.0)
                              */

  abc_frame_b16_t  i_abc;    /* Current in ABC frame */
  ab_frame_b16_t   i_ab;     /* Current in alpha-beta frame */
  dq_frame_b16_t   i_dq;     /* Current in dq frame */
  dq_frame_b16_t   i_dq_err; /* DQ current error */

  dq_frame_b16_t   i_dq_ref; /* Requested current for the FOC
                              * current controller
                              */

  pid_controller_b16_t id_pid; /* Current d-axis component PI controller */
  pid_controller_b16_t iq_pid; /* Current q-axis component PI controller */

  b16_t vdq_mag_max;         /* Maximum dq voltage magnitude */
  b16_t vab_mod_scale;       /* Voltage alpha-beta modulation scale */

  phase_angle_b16_t   angle; /* Phase angle */
};

/* Motor physical parameters.
 * This data structure was designed to work with BLDC/PMSM motors,
 * but probably can be used to describe different types of motors.
 */

struct motor_phy_params_b16_s
{
  uint8_t p;                   /* Number of the motor pole pairs */
  b16_t   flux_link;           /* Flux linkage */
  b16_t   res;                 /* Average phase-to-neutral resistance */
  b16_t   ind;                 /* Average phase-to-neutral inductance */
  b16_t   one_by_ind;          /* Inverse phase-to-neutral inductance */
  b16_t   one_by_p;            /* Inverse number of motor pole pairs */
};

/* PMSM motor physcial parameters */

struct pmsm_phy_params_b16_s
{
  struct motor_phy_params_b16_s motor;       /* Motor common PHY */
  b16_t                         iner;        /* Rotor inertia */
  b16_t                         ind_d;       /* d-inductance */
  b16_t                         ind_q;       /* q-inductance */
  b16_t                         one_by_iner; /* One by J */
  b16_t                         one_by_indd; /* One by Ld */
  b16_t                         one_by_indq; /* One by Lq */
};

/* PMSM motor model state */

struct pmsm_model_state_b16_s
{
  /* Motor model phase current */

  abc_frame_b16_t i_abc;
  ab_frame_b16_t  i_ab;
  dq_frame_b16_t  i_dq;

  /* Motor model phase voltage */

  abc_frame_b16_t v_abc;
  ab_frame_b16_t  v_ab;
  dq_frame_b16_t  v_dq;

  /* Motor model angle */

  struct motor_angle_b16_s angle;

  /* Angular speed */

  b16_t omega_e;
  b16_t omega_m;
};

/* PMSM motor model external conditions */

struct pmsm_model_ext_b16_s
{
  b16_t load;                /* Motor model load torque */
};

/* PMSM motor model */

struct pmsm_model_b16_s
{
  struct pmsm_phy_params_b16_s  phy;    /* Motor model physical parameters */
  struct pmsm_model_state_b16_s state;  /* Motor model state */
  struct pmsm_model_ext_b16_s   ext;    /* Motor model external conditions */
  b16_t                         per;    /* Control period */
  b16_t                         id_int; /* Id integral part */
  b16_t                         iq_int; /* Iq integral part */
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

b16_t fast_sin_b16(b16_t angle);
b16_t fast_sin2_b16(b16_t angle);
b16_t fast_cos_b16(b16_t angle);
b16_t fast_cos2_b16(b16_t angle);
b16_t fast_atan2_b16(b16_t y, b16_t x);
void  f_saturate_b16(FAR b16_t *val, b16_t min, b16_t max);
b16_t vector2d_mag_b16(b16_t x, b16_t y);
void  vector2d_saturate_b16(FAR b16_t *x, FAR b16_t *y, b16_t max);
void  dq_saturate_b16(FAR dq_frame_b16_t *dq, b16_t max);
b16_t dq_mag_b16(FAR dq_frame_b16_t *dq);

/* PID controller functions */

void pid_controller_init_b16(FAR pid_controller_b16_t *pid,
                             b16_t KP, b16_t KI, b16_t KD);
void pi_controller_init_b16(FAR pid_controller_b16_t *pid,
                            b16_t KP, b16_t KI);
void pid_saturation_set_b16(FAR pid_controller_b16_t *pid, b16_t min,
                            b16_t max);
void pi_saturation_set_b16(FAR pid_controller_b16_t *pid, b16_t min,
                           b16_t max);
void pid_integral_reset_b16(FAR pid_controller_b16_t *pid);
void pi_integral_reset_b16(FAR pid_controller_b16_t *pid);
b16_t pi_controller_b16(FAR pid_controller_b16_t *pid, b16_t err);
b16_t pid_controller_b16(FAR pid_controller_b16_t *pid, b16_t err);
void pi_antiwindup_enable_b16(FAR pid_controller_b16_t *pid, b16_t KC,
                              bool enable);
void pi_ireset_enable_b16(FAR pid_controller_b16_t *pid, bool enable);

/* Transformation functions */

void clarke_transform_b16(FAR abc_frame_b16_t *abc, FAR ab_frame_b16_t *ab);
void inv_clarke_transform_b16(FAR ab_frame_b16_t *ab,
                              FAR abc_frame_b16_t *abc);
void park_transform_b16(FAR phase_angle_b16_t *angle, FAR ab_frame_b16_t *ab,
                        FAR dq_frame_b16_t *dq);
void inv_park_transform_b16(FAR phase_angle_b16_t *angle,
                            FAR dq_frame_b16_t *dq, FAR ab_frame_b16_t *ab);

/* Phase angle related functions */

void angle_norm_b16(FAR b16_t *angle, b16_t per, b16_t bottom, b16_t top);
void angle_norm_2pi_b16(FAR b16_t *angle, b16_t bottom, b16_t top);
void phase_angle_update_b16(FAR struct phase_angle_b16_s *angle, b16_t val);

/* 3-phase system space vector modulation */

void svm3_init_b16(FAR struct svm3_state_b16_s *s);
void svm3_b16(FAR struct svm3_state_b16_s *s, FAR ab_frame_b16_t *ab);
void svm3_current_correct_b16(FAR struct svm3_state_b16_s *s,
                              b16_t *c0, b16_t *c1, b16_t *c2);

/* Field Oriented Control */

void foc_init_b16(FAR struct foc_data_b16_s *foc,
                  FAR struct foc_initdata_b16_s *init);
void foc_vbase_update_b16(FAR struct foc_data_b16_s *foc, b16_t vbase);
void foc_angle_update_b16(FAR struct foc_data_b16_s *foc,
                          FAR phase_angle_b16_t *angle);
void foc_iabc_update_b16(FAR struct foc_data_b16_s *foc,
                         FAR abc_frame_b16_t *i_abc);
void foc_voltage_control_b16(FAR struct foc_data_b16_s *foc,
                             FAR dq_frame_b16_t *vdq_ref);
void foc_current_control_b16(FAR struct foc_data_b16_s *foc,
                             FAR dq_frame_b16_t *idq_ref,
                             FAR dq_frame_b16_t *vdq_comp,
                             FAR dq_frame_b16_t *v_dq_ref);
void foc_vabmod_get_b16(FAR struct foc_data_b16_s *foc,
                        FAR ab_frame_b16_t *v_ab_mod);
void foc_vdq_mag_max_get_b16(FAR struct foc_data_b16_s *foc, FAR b16_t *max);

/* BLDC/PMSM motor observers */

void motor_sobserver_init_b16(FAR struct motor_sobserver_b16_s *observer,
                              FAR void *so, b16_t per);
void motor_aobserver_init_b16(FAR struct motor_aobserver_b16_s *observer,
                              FAR void *ao, b16_t per);
b16_t motor_sobserver_speed_get_b16(FAR struct motor_sobserver_b16_s *o);
b16_t motor_aobserver_angle_get_b16(FAR struct motor_aobserver_b16_s *o);

void motor_aobserver_smo_init_b16(FAR struct motor_aobserver_smo_b16_s *smo,
                                  b16_t kslide, b16_t err_max);
void motor_aobserver_smo_b16(FAR struct motor_aobserver_b16_s *o,
                             FAR ab_frame_b16_t *i_ab,
                             FAR ab_frame_b16_t *v_ab,
                             FAR struct motor_phy_params_b16_s *phy,
                             b16_t dir, b16_t speed);

void motor_sobserver_div_init_b16(FAR struct motor_sobserver_div_b16_s *so,
                                  uint8_t samples, b16_t filer, b16_t per);
void motor_sobserver_div_b16(FAR struct motor_sobserver_b16_s *o,
                             b16_t angle);

void motor_aobserver_nfo_init_b16(FAR struct motor_aobserver_nfo_b16_s *nfo);
void motor_aobserver_nfo_b16(FAR struct motor_aobserver_b16_s *o,
                             FAR ab_frame_b16_t *i_ab,
                             FAR ab_frame_b16_t *v_ab,
                             FAR struct motor_phy_params_b16_s *phy,
                             b16_t gain);

void motor_sobserver_pll_init_b16(FAR struct motor_sobserver_pll_b16_s *so,
                                  b16_t pll_kp, b16_t pll_ki);
void motor_sobserver_pll_b16(FAR struct motor_sobserver_b16_s *o,
                             b16_t angle);

/* Motor openloop control */

void motor_openloop_init_b16(FAR struct openloop_data_b16_s *op, b16_t per);
void motor_openloop_b16(FAR struct openloop_data_b16_s *op, b16_t speed,
                        b16_t dir);
b16_t motor_openloop_angle_get_b16(FAR struct openloop_data_b16_s *op);

/* Motor angle */

void motor_angle_init_b16(FAR struct motor_angle_b16_s *angle, uint8_t p);
void motor_angle_e_update_b16(FAR struct motor_angle_b16_s *angle,
                              b16_t angle_new, b16_t dir);
void motor_angle_m_update_b16(FAR struct motor_angle_b16_s *angle,
                              b16_t angle_new, b16_t dir);
b16_t motor_angle_m_get_b16(FAR struct motor_angle_b16_s *angle);
b16_t motor_angle_e_get_b16(FAR struct motor_angle_b16_s *angle);

/* Motor physical parameters */

void motor_phy_params_init_b16(FAR struct motor_phy_params_b16_s *phy,
                               uint8_t poles, b16_t res, b16_t ind,
                               b16_t fluxlink);

/* PMSM physical parameters functions */

void pmsm_phy_params_init_b16(FAR struct pmsm_phy_params_b16_s *phy,
                              uint8_t poles, b16_t res, b16_t ind,
                              b16_t iner, b16_t flux,
                              b16_t ind_d, b16_t ind_q);

/* PMSM motor model */

int pmsm_model_initialize_b16(FAR struct pmsm_model_b16_s *model,
                              FAR struct pmsm_phy_params_b16_s *phy,
                              b16_t per);
int pmsm_model_elec_b16(FAR struct pmsm_model_b16_s *model,
                        FAR ab_frame_b16_t *vab);
int pmsm_model_mech_b16(FAR struct pmsm_model_b16_s *model, b16_t load);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DSPB16_H */
