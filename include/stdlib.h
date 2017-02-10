/****************************************************************************
 * include/stdlib.h
 *
 *   Copyright (C) 2007-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_STDLIB_H
#define __INCLUDE_STDLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>

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

/* Maximum value returned by rand() */

#define RAND_MAX 32767

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

struct mallinfo
{
  int arena;    /* This is the total size of memory allocated
                 * for use by malloc in bytes. */
  int ordblks;  /* This is the number of free (not in use) chunks */
  int mxordblk; /* Size of the largest free (not in use) chunk */
  int uordblks; /* This is the total size of memory occupied by
                 * chunks handed out by malloc. */
  int fordblks; /* This is the total size of memory occupied
                 * by free (not in use) chunks.*/
};

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

#ifndef CONFIG_DISABLE_ENVIRON
/* Environment variable support */

FAR char **get_environ_ptr(void);
FAR char *getenv(FAR const char *name);
int       putenv(FAR const char *string);
int       clearenv(void);
int       setenv(FAR const char *name, FAR const char *value, int overwrite);
int       unsetenv(FAR const char *name);
#endif

/* Process exit functions */

void      exit(int status) noreturn_function;
void      abort(void) noreturn_function;
#ifdef CONFIG_SCHED_ATEXIT
int       atexit(CODE void (*func)(void));
#endif
#ifdef CONFIG_SCHED_ONEXIT
int       on_exit(CODE void (*func)(int, FAR void *), FAR void *arg);
#endif

/* _Exit() is a stdlib.h equivalent to the unistd.h _exit() function */

void      _exit(int status); /* See unistd.h */
#define   _Exit(s) _exit(s)

/* System() command is not implemented in the NuttX libc because it is so
 * entangled with shell logic.  There is an experimental version at
 * apps/system/system.  system() is prototyped here, however, for
 * standards compatibility.
 */

#ifndef __KERNEL__
int       system(FAR char *cmd);
#endif

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

#define atoi(nptr)  ((int)strtol((nptr), NULL, 10))
#define atol(nptr)  strtol((nptr), NULL, 10)
#ifdef CONFIG_HAVE_LONG_LONG
#define atoll(nptr) strtoll((nptr), NULL, 10)
#endif
#ifdef CONFIG_HAVE_DOUBLE
#define atof(nptr)  strtod((nptr), NULL)
#endif

/* Binary to string conversions */

FAR char *itoa(int val, FAR char *str, int base);

/* Wide character operations */

#ifdef CONFIG_LIBC_WCHAR
int       mbtowc(FAR wchar_t *pwc, FAR const char *s, size_t n);
int       wctomb(FAR char *s, wchar_t wchar);
#endif

/* Memory Management */

FAR void *malloc(size_t);
void      free(FAR void *);
FAR void *realloc(FAR void *, size_t);
FAR void *memalign(size_t, size_t);
FAR void *zalloc(size_t);
FAR void *calloc(size_t, size_t);

#ifdef CONFIG_CAN_PASS_STRUCTS
struct mallinfo mallinfo(void);
#else
int      mallinfo(FAR struct mallinfo *info);
#endif

/* Pseudo-Terminals */

#ifdef CONFIG_PSEUDOTERM_SUSV1
FAR char *ptsname(int fd);
int ptsname_r(int fd, FAR char *buf, size_t buflen);
#endif

#ifdef CONFIG_PSEUDOTERM
int unlockpt(int fd);

/* int grantpt(int fd); Not implemented */

#define grantpt(fd) (0)
#endif

/* Arithmetic */

int      abs(int j);
long int labs(long int j);
#ifdef CONFIG_HAVE_LONG_LONG
long long int llabs(long long int j);
#endif

#ifdef CONFIG_CAN_PASS_STRUCTS
div_t    div(int numer, int denom);
ldiv_t   ldiv(long numer, long denom);
#ifdef CONFIG_HAVE_LONG_LONG
lldiv_t  lldiv(long long numer, long long denom);
#endif
#endif

/* Temporary files */

int      mktemp(FAR char *path_template);
int      mkstemp(FAR char *path_template);

/* Sorting */

void     qsort(FAR void *base, size_t nel, size_t width,
               CODE int (*compar)(FAR const void *, FAR const void *));

/* Binary search */

FAR void *bsearch(FAR const void *key, FAR const void *base, size_t nel,
                  size_t width, CODE int (*compar)(FAR const void *,
                  FAR const void *));

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDLIB_H */
