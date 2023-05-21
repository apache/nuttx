/****************************************************************************
 * libs/libm/newlib/include/complex.h
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

/* Written by Matthias Drochner.
 * Public domain.
 */

#ifndef _COMPLEX_H
#define _COMPLEX_H

#define complex _Complex
#define _Complex_I 1.0fi
#define I _Complex_I

#ifdef __cplusplus
extern "C"
{
#endif

/* 7.3.5 Trigonometric functions */

/* 7.3.5.1 The cacos functions */

double complex cacos(double complex);
float complex cacosf(float complex);
long double complex cacosl(long double complex);

/* 7.3.5.2 The casin functions */

double complex casin(double complex);
float complex casinf(float complex);
long double complex casinl(long double complex);

/* 7.3.5.1 The catan functions */

double complex catan(double complex);
float complex catanf(float complex);
long double complex catanl(long double complex);

/* 7.3.5.1 The ccos functions */

double complex ccos(double complex);
float complex ccosf(float complex);
long double complex ccosl(long double complex);

/* 7.3.5.1 The csin functions */

double complex csin(double complex);
float complex csinf(float complex);
long double complex csinl(long double complex);

/* 7.3.5.1 The ctan functions */

double complex ctan(double complex);
float complex ctanf(float complex);
long double complex ctanl(long double complex);

/* 7.3.6 Hyperbolic functions */

/* 7.3.6.1 The cacosh functions */

double complex cacosh(double complex);
float complex cacoshf(float complex);
long double complex cacoshl(long double complex);

/* 7.3.6.2 The casinh functions */

double complex casinh(double complex);
float complex casinhf(float complex);
long double complex casinhl(long double complex);

/* 7.3.6.3 The catanh functions */

double complex catanh(double complex);
float complex catanhf(float complex);
long double complex catanhl(long double complex);

/* 7.3.6.4 The ccosh functions */

double complex ccosh(double complex);
float complex ccoshf(float complex);
long double complex ccoshl(long double complex);

/* 7.3.6.5 The csinh functions */

double complex csinh(double complex);
float complex csinhf(float complex);
long double complex csinhl(long double complex);

/* 7.3.6.6 The ctanh functions */

double complex ctanh(double complex);
float complex ctanhf(float complex);
long double complex ctanhl(long double complex);

/* 7.3.7 Exponential and logarithmic functions */

/* 7.3.7.1 The cexp functions */

double complex cexp(double complex);
float complex cexpf(float complex);
long double complex cexpl(long double complex);

/* 7.3.7.2 The clog functions */

double complex clog(double complex);
float complex clogf(float complex);
long double complex clogl(long double complex);

/* 7.3.8 Power and absolute-value functions */

/* 7.3.8.1 The cabs functions */

long double cabsl(long double complex) ;
double cabs(double complex) ;
float cabsf(float complex) ;

/* 7.3.8.2 The cpow functions */

double complex cpow(double complex, double complex);
float complex cpowf(float complex, float complex);
long double complex cpowl(long double complex, long double complex);

/* 7.3.8.3 The csqrt functions */

double complex csqrt(double complex);
float complex csqrtf(float complex);
long double complex csqrtl(long double complex);

/* 7.3.9 Manipulation functions */

/* 7.3.9.1 The carg functions */

double carg(double complex);
float cargf(float complex);
long double cargl(long double complex);

/* 7.3.9.2 The cimag functions */

double cimag(double complex);
float cimagf(float complex);
long double cimagl(long double complex);

/* 7.3.9.3 The conj functions */

double complex conj(double complex);
float complex conjf(float complex);
long double complex conjl(long double complex);

/* 7.3.9.4 The cproj functions */

double complex cproj(double complex);
float complex cprojf(float complex);
long double complex cprojl(long double complex);

/* 7.3.9.5 The creal functions */

double creal(double complex);
float crealf(float complex);
long double creall(long double complex);

double complex clog10(double complex);
float complex clog10f(float complex);
#  if defined(__CYGWIN__)
long double complex clog10l(long double complex);
#  endif

#ifdef __cplusplus
}
#endif

#endif /* ! _COMPLEX_H */
