/****************************************************************************
 * include/stdlib.h
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
 * Definitions
 ****************************************************************************/

/* The C standard specifies two constants, EXIT_SUCCESS and
 * EXIT_FAILURE, that may be passed to exit() to indicate
 * successful or unsucessful termination, respectively.
 */

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/* The NULL pointer should be defined in this file but is currently defined
 * in sys/types.h.
 */

/* Maximum value returned by rand() */

#define MAX_RAND 32767

/* Integer expression whose value is the maximum number of bytes in a
 * character specified by the current locale.
 */

#define MB_CUR_MAX 1

/* The environ variable, normally 'extern char **environ;' is
 * not implemented as a function call.  However, get_environ_ptr()
 * can be used in its place.
 */

#ifndef CONFIG_DISABLE_ENIVRON
# define environ get_environ_ptr()
#endif

/****************************************************************************
 * Global Type Definitions
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

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
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

/* Environment variable support */

#ifndef CONFIG_DISABLE_ENIVRON
FAR char **get_environ_ptr( void );
FAR char *getenv(FAR const char *name);
int       putenv(FAR const char *string);
int       clearenv(void);
int       setenv(const char *name, const char *value, int overwrite);
int       unsetenv(const char *name);
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

/* String to binary conversions */

long      strtol(const char *, char **, int);
unsigned long strtoul(const char *, char **, int);
#ifdef CONFIG_HAVE_LONG_LONG
long long strtoll(const char *, char **, int);
unsigned long long strtoull(const char *, char **, int);
#endif
double_t  strtod(const char *, char **);

#define atoi(nptr)  strtol((nptr), NULL, 10)
#define atol(nptr)  strtol((nptr), NULL, 10)
#ifdef CONFIG_HAVE_LONG_LONG
#define atoll(nptr) strtoll((nptr), NULL, 10)
#endif
#define atof(nptr)  strtod((nptr), NULL)

/* Binary to string conversions */

char     *itoa(int value, char *str, int base);

/* Memory Management */

FAR void *malloc(size_t);
void      free(FAR void*);
FAR void *realloc(FAR void*, size_t);
FAR void *memalign(size_t, size_t);
FAR void *zalloc(size_t);
FAR void *calloc(size_t, size_t);

/* Misc */

int      abs(int j);
long int labs(long int j);
#ifdef CONFIG_HAVE_LONG_LONG
long long int llabs(long long int j);
#endif

/* Sorting */

void     qsort(void *base, size_t nmemb, size_t size,
               int(*compar)(const void *, const void *));

#ifdef CONFIG_CAN_PASS_STRUCTS
struct mallinfo mallinfo(void);
#else
int      mallinfo(struct mallinfo *info);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDLIB_H */
