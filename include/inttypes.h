/****************************************************************************
 * include/inttypes.h
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

#ifndef __INCLUDE_INTTYPES_H
#define __INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h> /* for wchar_t */

/* Notes from www.opengroup.org:
 *
 * "The <inttypes.h> header shall include the <stdint.h> header."
 */

#include <stdint.h>

#include <arch/inttypes.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "The following macros shall be defined. Each expands to a character string
 *  literal containing a conversion specifier, possibly modified by a length
 *  modifier, suitable for use within the format argument of a formatted
 *  input/output function when converting the corresponding integer type.
 *  These macros have the general form of PRI (character string literals for
 *  the fprintf() and fwprintf() family of functions) or SCN (character
 *  string literals for the fscanf() and fwscanf() family of functions),
 *  followed by the conversion specifier, followed by a name corresponding
 *  to a similar type name in <stdint.h>. In these names, N represents the
 *  width of the type as described in <stdint.h>. For example, PRIdFAST32
 *  can be used in a format string to print the value of an integer of type
 *  int_fast32_t.
 *
 * "The fprintf() macros for signed integers are:
 *
 * PRIdN
 *  PRIdLEASTN
 *  PRIdFASTN
 *  PRIdMAX
 *  PRIdPTR
 *
 * PRIiN
 *  PRIiLEASTN
 *  PRIiFASTN
 *  PRIiMAX
 *  PRIiPTR
 *
 * "The fprintf() macros for unsigned integers are:
 *
 * PRIoN
 *  PRIoLEASTN
 *  PRIoFASTN
 *  PRIoMAX
 *  PRIoPTR
 *
 * PRIuN
 *  PRIuLEASTN
 *  PRIuFASTN
 *  PRIuMAX
 *  PRIuPTR
 *
 * PRIxN
 *  PRIxLEASTN
 *  PRIxFASTN
 *  PRIxMAX
 *  PRIxPTR
 *
 * PRIXN
 *  PRIXLEASTN
 *  PRIXFASTN
 *  PRIXMAX
 *  PRIXPTR
 *
 * "The fscanf() macros for signed integers are:
 *
 * SCNdN
 *  SCNdLEASTN
 *  SCNdFASTN
 *  SCNdMAX
 *  SCNdPTR
 *
 * SCNiN
 *  SCNiLEASTN
 *  SCNiFASTN
 *  SCNiMAX
 *  SCNiPTR
 *
 * "The fscanf() macros for unsigned integers are:
 *
 * SCNoN
 *  SCNoLEASTN
 *  SCNoFASTN
 *  SCNoMAX
 *  SCNoPTR
 *
 * SCNuN
 *  SCNuLEASTN
 *  SCNuFASTN
 *  SCNuMAX
 *  SCNuPTR
 *
 * SCNxN
 *  SCNxLEASTN
 *  SCNxFASTN
 *  SCNxMAX
 *  SCNxPTR
 *
 * "For each type that the implementation provides in <stdint.h>, the
 * corresponding fprintf() and fwprintf() macros shall be defined and the
 * corresponding fscanf() and fwscanf() macros shall be defined unless the
 * implementation does not have a suitable modifier for the type.
 */

/* On NuttX, least and fast types are aliases of the exact type.
 * (See stdint.h)
 */

#define PRIdLEAST8  PRId8
#define PRIdLEAST16 PRId16
#define PRIdLEAST32 PRId32
#if defined(PRId64)
#define PRIdLEAST64 PRId64
#endif

#define PRIdFAST8   PRId8
#define PRIdFAST16  PRId16
#define PRIdFAST32  PRId32
#if defined(PRId64)
#define PRIdFAST64  PRId64
#endif

#define PRIiLEAST8  PRIi8
#define PRIiLEAST16 PRIi16
#define PRIiLEAST32 PRIi32
#if defined(PRIi64)
#define PRIiLEAST64 PRIi64
#endif

#define PRIiFAST8   PRIi8
#define PRIiFAST16  PRIi16
#define PRIiFAST32  PRIi32
#if defined(PRIi64)
#define PRIiFAST64  PRIi64
#endif

#define PRIoLEAST8  PRIo8
#define PRIoLEAST16 PRIo16
#define PRIoLEAST32 PRIo32
#if defined(PRIo64)
#define PRIoLEAST64 PRIo64
#endif

#define PRIoFAST8   PRIo8
#define PRIoFAST16  PRIo16
#define PRIoFAST32  PRIo32
#if defined(PRIo64)
#define PRIoFAST64  PRIo64
#endif

#define PRIuLEAST8  PRIu8
#define PRIuLEAST16 PRIu16
#define PRIuLEAST32 PRIu32
#if defined(PRIu64)
#define PRIuLEAST64 PRIu64
#endif

#define PRIuFAST8   PRIu8
#define PRIuFAST16  PRIu16
#define PRIuFAST32  PRIu32
#if defined(PRIu64)
#define PRIuFAST64  PRIu64
#endif

#define PRIxLEAST8  PRIx8
#define PRIxLEAST16 PRIx16
#define PRIxLEAST32 PRIx32
#if defined(PRIx64)
#define PRIxLEAST64 PRIx64
#endif

#define PRIxFAST8   PRIx8
#define PRIxFAST16  PRIx16
#define PRIxFAST32  PRIx32
#if defined(PRIx64)
#define PRIxFAST64  PRIx64
#endif

#define PRIXLEAST8  PRIX8
#define PRIXLEAST16 PRIX16
#define PRIXLEAST32 PRIX32
#if defined(PRIX64)
#define PRIXLEAST64 PRIX64
#endif

#define PRIXFAST8   PRIX8
#define PRIXFAST16  PRIX16
#define PRIXFAST32  PRIX32
#if defined(PRIX64)
#define PRIXFAST64  PRIX64
#endif

#define SCNdLEAST8  SCNd8
#define SCNdLEAST16 SCNd16
#define SCNdLEAST32 SCNd32
#if defined(SCNd64)
#define SCNdLEAST64 SCNd64
#endif

#define SCNdFAST8   SCNd8
#define SCNdFAST16  SCNd16
#define SCNdFAST32  SCNd32
#if defined(SCNd64)
#define SCNdFAST64  SCNd64
#endif

#define SCNiLEAST8  SCNi8
#define SCNiLEAST16 SCNi16
#define SCNiLEAST32 SCNi32
#if defined(SCNi64)
#define SCNiLEAST64 SCNi64
#endif

#define SCNiFAST8   SCNi8
#define SCNiFAST16  SCNi16
#define SCNiFAST32  SCNi32
#if defined(SCNi64)
#define SCNiFAST64  SCNi64
#endif

#define SCNoLEAST8  SCNo8
#define SCNoLEAST16 SCNo16
#define SCNoLEAST32 SCNo32
#if defined(SCNo64)
#define SCNoLEAST64 SCNo64
#endif

#define SCNoFAST8   SCNo8
#define SCNoFAST16  SCNo16
#define SCNoFAST32  SCNo32
#if defined(SCNo64)
#define SCNoFAST64  SCNo64
#endif

#define SCNuLEAST8  SCNu8
#define SCNuLEAST16 SCNu16
#define SCNuLEAST32 SCNu32
#if defined(SCNu64)
#define SCNuLEAST64 SCNu64
#endif

#define SCNuFAST8   SCNu8
#define SCNuFAST16  SCNu16
#define SCNuFAST32  SCNu32
#if defined(SCNu64)
#define SCNuFAST64  SCNu64
#endif

#define SCNxLEAST8  SCNx8
#define SCNxLEAST16 SCNx16
#define SCNxLEAST32 SCNx32
#if defined(SCNx64)
#define SCNxLEAST64 SCNx64
#endif

#define SCNxFAST8   SCNx8
#define SCNxFAST16  SCNx16
#define SCNxFAST32  SCNx32
#if defined(SCNx64)
#define SCNxFAST64  SCNx64
#endif

/* intmax_t/uintmax_t */

#define PRIdMAX     "jd"
#define PRIiMAX     "ji"
#define PRIoMAX     "jo"
#define PRIuMAX     "ju"
#define PRIxMAX     "jx"
#define PRIXMAX     "jX"

#define SCNdMAX     "jd"
#define SCNiMAX     "ji"
#define SCNoMAX     "jo"
#define SCNuMAX     "ju"
#define SCNxMAX     "jx"

/* off_t */

#if defined(CONFIG_FS_LARGEFILE)
#define PRIdOFF     PRId64
#define PRIiOFF     PRIi64
#define PRIoOFF     PRIo64
#define PRIuOFF     PRIu64
#define PRIxOFF     PRIx64
#define PRIXOFF     PRIX64

#define SCNdOFF     SCNd64
#define SCNiOFF     SCNi64
#define SCNoOFF     SCNo64
#define SCNuOFF     SCNu64
#define SCNxOFF     SCNx64
#else
#define PRIdOFF     PRId32
#define PRIiOFF     PRIi32
#define PRIoOFF     PRIo32
#define PRIuOFF     PRIu32
#define PRIxOFF     PRIx32
#define PRIXOFF     PRIX32

#define SCNdOFF     SCNd32
#define SCNiOFF     SCNi32
#define SCNoOFF     SCNo32
#define SCNuOFF     SCNu32
#define SCNxOFF     SCNx32
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* "The <inttypes.h> header shall include a definition of at least the
 * following type:
 *
 * imaxdiv_t
 *   Structure type that is the type of the value returned by the imaxdiv()
 *   function.
 */

typedef void *imaxdiv_t; /* Dummy type since imaxdiv is not yet supported */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* "The following shall be declared as functions and may also be defined as
 *  macros. Function prototypes shall be provided."
 */

intmax_t  imaxabs(intmax_t j);
imaxdiv_t imaxdiv(intmax_t number, intmax_t denom);
intmax_t  strtoimax(FAR const char *nptr, FAR char **endptr, int base);
uintmax_t strtoumax(FAR const char *nptr, FAR char **endptr, int base);

intmax_t  wcstoimax(FAR const wchar_t *nptr, FAR wchar_t **endptr, int base);
uintmax_t wcstoumax(FAR const wchar_t *nptr, FAR wchar_t **endptr, int base);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_INTTYPES_H */
