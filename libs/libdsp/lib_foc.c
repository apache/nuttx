/****************************************************************************
 * control/lib_foc.c
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_current_control
 *
 * Description:
 *   This function implements FOC current control algorithm.
 *
 * Input Parameters:
 *   id_pid  - (in) pointer to the direct current PI controller data
 *   iq_pid  - (in) pointer to the quadrature current PI controller data
 *   idq_ref - (in) dq current reference
 *   i_dq    - (in) dq current
 *   vdq_min - (in) lower regulator limit
 *   vdq_max - (in) upper regulator limit
 *   v_dq    - (out) pointer to the dq voltage frame
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void foc_current_control(FAR pid_controller_t *id_pid,
                         FAR pid_controller_t *iq_pid,
                         FAR dq_frame_t *idq_ref,
                         FAR dq_frame_t *idq,
                         FAR float_sat_t *sat,
                         FAR dq_frame_t *v_dq)
{
  dq_frame_t idq_err;

  /* Get dq current error */

  idq_err.d = idq_ref->d - idq->d;
  idq_err.q = idq_ref->q - idq->q;

  /* Update regulators saturation */

  pi_saturation_set(id_pid, sat->min, sat->max);
  pi_saturation_set(iq_pid, sat->min, sat->max);

  /* PI controller for d-current (flux loop) */

  v_dq->d = pi_controller(id_pid, idq_err.d);

  /* PI controller for q-current (torque loop) */

  v_dq->q = pi_controller(iq_pid, idq_err.q);
}
