/****************************************************************************
 * include/nuttx/math/fft.h
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

#ifndef __INCLUDE_NUTTX_MATH_FFT_H
#define __INCLUDE_NUTTX_MATH_FFT_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* fft/dct request type definition
 * FFT, fft for fast fourier transform
 * DCT, dct for discrete cosine transform
 * DCT-II, the normally so called DCT
 * DCT-III, inverse of DCT-II, so called IDCT
 * DCT-IV, inverse of DCT-IV is itself
 * see all the formula and more mathmatics trueth in wikipedia:
 * https://en.wikipedia.org/wiki/Discrete_cosine_transform
 */

typedef enum
{
  MATH_FFT        = 0,  /* FFT */
  MATH_DCT_TYPE_2 = 1,  /* DCT-II */
  MATH_DCT_TYPE_3 = 2,  /* DCT-III */
  MATH_DCT_TYPE_4 = 3,  /* DCT-IV */
} fft_type;

/* fft calculate request */

struct fft_calc_s
{
  fft_type type;

  /* fft/dct bit width
   * power of 2, range from 8 to 32
   */

  size_t bitwidth;

  /* input data, address must be 4 bytes aligned */

  FAR const void *input_data;

  /* output data, address must be 4 bytes aligned */

  FAR void *output_data;

  /* FFT Length, power of 2
   * fft support 16 to 4096
   * dct support 8 to 256
   */

  size_t fft_length;

  /* Real flag. true: real fft/dct, false: complex fft/dct. */

  bool real;

  /* Inverse flag. true: ifft/idct, false: fft/dct */

  bool inverse;
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct fft_lowerhalf_s;
struct fft_ops_s
{
  /* FFT calculate request */

  CODE int (*calc)(FAR struct fft_lowerhalf_s *lower,
                   FAR struct fft_calc_s *calc);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct fft_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct fft_ops_s *ops;  /* Lower half operations */
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
 * Name: fft_register
 *
 * Description:
 *   Register a FFT driver.
 *
 ****************************************************************************/

int fft_register(FAR const char *path,
                 FAR struct fft_lowerhalf_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MATH_FFT_H */
