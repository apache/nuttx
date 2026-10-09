/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_ba414e.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_BA414E_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_BA414E_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Largest operand: 512 bits */

#define BA414E_MAX_BYTES        64

/* Operation results (beyond negated errno values) */

#define BA414E_OK               0
#define BA414E_POINT_AT_INF     1   /* ECC result is the point at infinity */
#define BA414E_NOT_ON_CURVE     2   /* Point is not on the curve */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Short Weierstrass curve y^2 = x^3 + ax + b over GF(p).  All values are
 * little endian, 'len' bytes long.
 */

struct ba414e_curve_s
{
  uint8_t len;
  FAR const uint8_t *p;
  FAR const uint8_t *n;
  FAR const uint8_t *gx;
  FAR const uint8_t *gy;
  FAR const uint8_t *a;
  FAR const uint8_t *b;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NIST P-256 (FIPS 186-4 D.1.2.3), little endian */

extern const struct ba414e_curve_s g_ba414e_p256;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* All operands and results are little endian, 'len' bytes (at most
 * BA414E_MAX_BYTES).  The functions block until the engine is done and
 * return BA414E_OK (or one of the ECC results above) or a negated errno.
 */

int pic32mz_ba414e_initialize(void);

int pic32mz_ba414e_modadd(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len);
int pic32mz_ba414e_modsub(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len);
int pic32mz_ba414e_modmul(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len);
int pic32mz_ba414e_modexp(FAR uint8_t *c, FAR const uint8_t *m,
                          FAR const uint8_t *e, FAR const uint8_t *n,
                          int len);

int pic32mz_ba414e_ecc_add(FAR const struct ba414e_curve_s *curve,
                           FAR uint8_t *rx, FAR uint8_t *ry,
                           FAR const uint8_t *px, FAR const uint8_t *py,
                           FAR const uint8_t *qx, FAR const uint8_t *qy);
int pic32mz_ba414e_ecc_mul(FAR const struct ba414e_curve_s *curve,
                           FAR uint8_t *rx, FAR uint8_t *ry,
                           FAR const uint8_t *px, FAR const uint8_t *py,
                           FAR const uint8_t *k);
int pic32mz_ba414e_ecc_check(FAR const struct ba414e_curve_s *curve,
                             FAR const uint8_t *px, FAR const uint8_t *py);

#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_BA414E_H */
