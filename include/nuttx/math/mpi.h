/****************************************************************************
 * include/nuttx/math/mpi.h
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

#ifndef __INCLUDE_NUTTX_MATH_MPI_H
#define __INCLUDE_NUTTX_MATH_MPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPI_MAXPARAM      4

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mpi_calc_func_e
{
  MPI_CALC_FUNC_INVAL   = 0,
  MPI_CALC_FUNC_ADD     = 1,  /* X = A + B
                               * first argument: A,
                               * second argument: B,
                               * third argument: X
                               */
  MPI_CALC_FUNC_SUB     = 2,  /* X = A - B
                               * first argument: A,
                               * second argument: B,
                               * third argument: X
                               */
  MPI_CALC_FUNC_MUL     = 3,  /* X = A * B,
                               * first argument: A,
                               * second argument: B,
                               * third argument: X
                               */
  MPI_CALC_FUNC_DIV     = 4,  /* A = Q * B + R
                               * first argument: A,
                               * second argument: B,
                               * third argument: Q,
                               * fourth argument: R
                               */
  MPI_CALC_FUNC_MOD     = 5,  /* R = A mod B
                               * first argument: A,
                               * second argument: B,
                               * third argument: R,
                               */
  MPI_CALC_FUNC_EXP_MOD = 6,  /* X = A ^ E mod N
                               * first argument: A,
                               * second argument: E,
                               * third argument: N,
                               * fourth argument: X,
                               */
  MPI_CALC_FUNC_INV_MOD = 7,  /* X = A ^ -1 mod N
                               * first argument: A,
                               * second argument: N,
                               * third argument: X,
                               */
  MPI_CALC_FUNC_GCD     = 8,  /* G = gcd(A, B)
                               * first argument: A,
                               * second argument: B,
                               * third argument: G,
                               */
  MPI_CALC_FUNC_LAST
};

/*  MPI structure
 *  s: -1 if the mpi is negative, 1 otherwise.
 *  n: total number of bits in p.
 *  p: pointer to mpi.
 */

struct mpiparam
{
  int s;
  size_t n;
  FAR uint8_t *p;
};

struct mpi_calc_s
{
  uint16_t op;
  struct mpiparam param[MPI_MAXPARAM];
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct mpi_lowerhalf_s;
struct mpi_ops_s
{
  /* MPI calculate request */

  CODE int (*calc)(FAR struct mpi_lowerhalf_s *lower,
                   FAR struct mpi_calc_s *calc);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct mpi_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct mpi_ops_s *ops;  /* Lower half operations */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpi_register
 *
 * Description:
 *   Register a MPI driver.
 *
 ****************************************************************************/

int mpi_register(FAR const char *path,
                 FAR struct mpi_lowerhalf_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MATH_MPI_H */
