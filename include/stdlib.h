/****************************************************************************
 * include/stdlib.h
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

#ifndef __INCLUDE_STDLIB_H
#define __INCLUDE_STDLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The C standard specifies two constants, EXIT_SUCCESS and EXIT_FAILURE,
 * that may be passed to exit() to indicate successful or unsuccessful
 * termination, respectively.
 */

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/* The NULL pointer should be defined in this file but is currently defined
 * in sys/types.h.
 */

/* Maximum value returned by rand().  Must be a minimum of 32767. */

#define RAND_MAX INT_MAX

/* Integer expression whose value is the maximum number of bytes in a
 * character specified by the current locale.
 */

#define MB_CUR_MAX 1

/* The environ variable, normally 'char **environ;' is not implemented as a
 * function call.  However, get_environ_ptr() can be used in its place.
 */

#ifndef CONFIG_DISABLE_ENVIRON
#  define environ get_environ_ptr()
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structure type returned by the div() function. */

struct div_s
{
  int quot;     /* Quotient */
  int rem;      /* Remainder */
};

typedef struct div_s div_t;

/* Structure type returned by the ldiv() function. */

struct ldiv_s
{
  long quot;    /* Quotient */
  long rem;     /* Remainder */
};

typedef struct ldiv_s ldiv_t;

/* Structure type returned by the lldiv() function. */

struct lldiv_s
{
  long quot;    /* Quotient */
  long rem;     /* Remainder */
};

typedef struct lldiv_s lldiv_t;

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

/* Random number generation */

void      srand(unsigned int seed);
int       rand(void);

#define   srandom(s) srand(s)
long      random(void);

/* Environment variable support */

FAR char **get_environ_ptr(void);
FAR char *getenv(FAR const char *name);
int       putenv(FAR const char *string);
int       clearenv(void);
int       setenv(FAR const char *name, FAR const char *value, int overwrite);
int       unsetenv(FAR const char *name);

/* Process exit functions */

void      exit(int status) noreturn_function;
void      abort(void) noreturn_function;
int       atexit(CODE void (*func)(void));
int       on_exit(CODE void (*func)(int, FAR void *), FAR void *arg);

/* _Exit() is a stdlib.h equivalent to the unistd.h _exit() function */

void      _Exit(int status) noreturn_function;

/* System() command is not implemented in the NuttX libc because it is so
 * entangled with shell logic.  There is an experimental version at
 * apps/system/system.  system() is prototyped here, however, for
 * standards compatibility.
 */

#ifndef __KERNEL__
int       system(FAR const char *cmd);
#endif

FAR char *realpath(FAR const char *path, FAR char *resolved);

/* String to binary conversions */

long      strtol(FAR const char *nptr, FAR char **endptr, int base);
unsigned long strtoul(FAR const char *nptr, FAR char **endptr, int base);
#ifdef CONFIG_HAVE_LONG_LONG
long long strtoll(FAR const char *nptr, FAR char **endptr, int base);
unsigned long long strtoull(FAR const char *nptr, FAR char **endptr,
                            int base);
#endif
float     strtof(FAR const char *str, FAR char **endptr);
#ifdef CONFIG_HAVE_DOUBLE
double    strtod(FAR const char *str, FAR char **endptr);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double strtold(FAR const char *str, FAR char **endptr);
#endif

int       atoi(FAR const char *nptr);
long      atol(FAR const char *nptr);
#ifdef CONFIG_HAVE_LONG_LONG
long long atoll(FAR const char *nptr);
#endif
#ifdef CONFIG_HAVE_DOUBLE
double    atof(FAR const char *nptr);
#endif

/* Binary to string conversions */

FAR char *itoa(int val, FAR char *str, int base);

/* Wide character operations */

#ifdef CONFIG_LIBC_WCHAR
int       mblen(FAR const char *s, size_t n);
int       mbtowc(FAR wchar_t *pwc, FAR const char *s, size_t n);
size_t    mbstowcs(FAR wchar_t *dst, FAR const char *src, size_t len);
int       wctomb(FAR char *s, wchar_t wchar);
size_t    wcstombs(FAR char *dst, FAR const wchar_t *src, size_t len);
#endif

/* Memory Management */

FAR void *malloc(size_t);
void      free(FAR void *);
FAR void *realloc(FAR void *, size_t);
FAR void *memalign(size_t, size_t);
FAR void *zalloc(size_t);
FAR void *calloc(size_t, size_t);
FAR void *aligned_alloc(size_t, size_t);
int       posix_memalign(FAR void **, size_t, size_t);

/* Pseudo-Terminals */

#ifdef CONFIG_PSEUDOTERM_SUSV1
FAR char *ptsname(int fd);
int       ptsname_r(int fd, FAR char *buf, size_t buflen);
#endif

#ifdef CONFIG_PSEUDOTERM
int       unlockpt(int fd);

/* int grantpt(int fd); Not implemented */

#define grantpt(fd) (0)
#endif

/* Arithmetic */

int       abs(int j);
long int  labs(long int j);
#ifdef CONFIG_HAVE_LONG_LONG
long long int llabs(long long int j);
#endif

div_t     div(int number, int denom);
ldiv_t    ldiv(long number, long denom);
#ifdef CONFIG_HAVE_LONG_LONG
lldiv_t   lldiv(long long number, long long denom);
#endif

/* Temporary files */

FAR char *mktemp(FAR char *path_template);
int       mkstemp(FAR char *path_template);
FAR char *mkdtemp(FAR char *path_template);

/* Sorting */

void      qsort(FAR void *base, size_t nel, size_t width,
                CODE int (*compar)(FAR const void *, FAR const void *));

/* Binary search */

FAR void  *bsearch(FAR const void *key, FAR const void *base, size_t nel,
                   size_t width, CODE int (*compar)(FAR const void *,
                   FAR const void *));

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDLIB_H */
