/************************************************************
 * stdlib.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __STDLIB_H
#define __STDLIB_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>

/************************************************************
 * Definitions
 ************************************************************/

/* The C standard specifies two constants, EXIT_SUCCESS and
 * EXIT_FAILURE, that may be passed to exit() to indicate
 * successfuol or unsucessful termination, respectively.
 */

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/************************************************************
 * Global Type Definitions
 ************************************************************/

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

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Random number generation */

EXTERN void      srand(unsigned int seed);
EXTERN int       rand(void);

/* Environment variable support */

EXTERN char     *getenv(const char *name);

/* Process exit functions */

EXTERN void      exit(int status);
EXTERN void      abort(void);
EXTERN int       atexit(void (*func)(void));

/* String to binary conversions */

#define atoi(nptr) strtol((nptr), (FAR char**)NULL, 10)
EXTERN long      strtol(const char *, char **, int);
EXTERN double_t  strtod(const char *, char **);

/* Memory Management */

EXTERN FAR void  *malloc(size_t);
EXTERN void       free(FAR void*);
EXTERN FAR void  *realloc(FAR void*, size_t);
EXTERN FAR void  *memalign(size_t, size_t);
EXTERN FAR void  *zalloc(size_t);
EXTERN FAR void  *calloc(size_t, size_t);

#ifdef CONFIG_CAN_PASS_STRUCTS
EXTERN struct mallinfo mallinfo(void);
#else
EXTERN int        mallinfo(struct mallinfo *info);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __STDLIB_H */
