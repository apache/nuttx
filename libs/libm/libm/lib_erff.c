/****************************************************************************
 * libs/libm/libm/lib_erff.c
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  A1     0.254829592F
#define  A2   (-0.284496736F)
#define  A3     1.421413741F
#define  A4   (-1.453152027F)
#define  A5     1.061405429F
#define  P      0.3275911F

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erff
 *
 * Description:
 *   This implementation comes from the Handbook of Mathematical Functions
 *   The implementations in this book are not protected by copyright.
 *   erf comes from formula 7.1.26
 *
 ****************************************************************************/

float erff(float x)
{
  float t;
  float z;

  z = fabsf(x);
  t = 1.0F / (1.0F + P * z);
  t = 1.0F -
      (((((A5 * t + A4) * t) + A3) * t + A2) * t + A1) * t * expf(-z * z);
  return copysignf(t, x);
}
