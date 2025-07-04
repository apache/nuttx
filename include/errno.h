/****************************************************************************
 * include/errno.h
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

#ifndef __INCLUDE_ERRNO_H
#define __INCLUDE_ERRNO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convenience/compatibility definition.  If the errno is accessed from the
 * internal OS code, then the OS code should use the set_errno() and
 * get_errno().  Currently, those are just placeholders but would be needed
 * in the KERNEL mode build in order to instantiate the process address
 * environment as necessary to access the TLS-based errno variable.
 */

#define errno *__errno()
#define set_errno(e) \
  do \
    { \
      errno = (int)(e); \
    } \
  while (0)
#define get_errno() errno

/* Definitions of error numbers and the string that would be
 * returned by strerror().
 */

/* Sync with linux/include/asm-generic/errno-base.h */

#define EPERM               1
#define ENOENT              2
#define ESRCH               3
#define EINTR               4
#define EIO                 5
#define ENXIO               6
#define E2BIG               7
#define ENOEXEC             8
#define EBADF               9
#define ECHILD              10
#define EAGAIN              11
#define ENOMEM              12
#define EACCES              13
#define EFAULT              14                         /* Linux errno extension */
#define ENOTBLK             15
#define EBUSY               16
#define EEXIST              17
#define EXDEV               18
#define ENODEV              19
#define ENOTDIR             20
#define EISDIR              21
#define EINVAL              22
#define ENFILE              23
#define EMFILE              24
#define ENOTTY              25
#define ETXTBSY             26
#define EFBIG               27
#define ENOSPC              28
#define ESPIPE              29
#define EROFS               30
#define EMLINK              31
#define EPIPE               32
#define EDOM                33
#define ERANGE              34

/* Sync with linux/include/asm-generic/errno.h */

#define EDEADLK             35
#define ENAMETOOLONG        36
#define ENOLCK              37
#define ENOSYS              38
#define ENOTEMPTY           39
#define ELOOP               40
#define EWOULDBLOCK         EAGAIN
#define ENOMSG              42
#define EIDRM               43
#define ECHRNG              44                         /* Linux errno extension */
#define EL2NSYNC            45                         /* Linux errno extension */
#define EL3HLT              46                         /* Linux errno extension */
#define EL3RST              47                         /* Linux errno extension */
#define ELNRNG              48                         /* Linux errno extension */
#define EUNATCH             49                         /* Linux errno extension */
#define ENOCSI              50                         /* Linux errno extension */
#define EL2HLT              51                         /* Linux errno extension */
#define EBADE               52                         /* Linux errno extension */
#define EBADR               53                         /* Linux errno extension */
#define EXFULL              54                         /* Linux errno extension */
#define ENOANO              55                         /* Linux errno extension */
#define EBADRQC             56                         /* Linux errno extension */
#define EBADSLT             57                         /* Linux errno extension */
#define EDEADLOCK           EDEADLK                    /* Linux errno extension */
#define EBFONT              59                         /* Linux errno extension */
#define ENOSTR              60
#define ENODATA             61
#define ETIME               62
#define ENOSR               63
#define ENONET              64                         /* Linux errno extension */
#define ENOPKG              65                         /* Linux errno extension */
#define EREMOTE             66                         /* Linux errno extension */
#define ENOLINK             67
#define EADV                68                         /* Linux errno extension */
#define ESRMNT              69                         /* Linux errno extension */
#define ECOMM               70                         /* Linux errno extension */
#define EPROTO              71
#define EMULTIHOP           72
#define EDOTDOT             73                         /* Linux errno extension */
#define EBADMSG             74
#define EOVERFLOW           75
#define ENOTUNIQ            76                         /* Linux errno extension */
#define EBADFD              77                         /* Linux errno extension */
#define EREMCHG             78                         /* Linux errno extension */
#define ELIBACC             79                         /* Linux errno extension */
#define ELIBBAD             80                         /* Linux errno extension */
#define ELIBSCN             81                         /* Linux errno extension */
#define ELIBMAX             82                         /* Linux errno extension */
#define ELIBEXEC            83                         /* Linux errno extension */
#define EILSEQ              84
#define ERESTART            85
#define ESTRPIPE            86                         /* Linux errno extension */
#define EUSERS              87
#define ENOTSOCK            88
#define EDESTADDRREQ        89
#define EMSGSIZE            90
#define EPROTOTYPE          91
#define ENOPROTOOPT         92
#define EPROTONOSUPPORT     93
#define ESOCKTNOSUPPORT     94                         /* Linux errno extension */
#define EOPNOTSUPP          95
#define EPFNOSUPPORT        96
#define EAFNOSUPPORT        97
#define EADDRINUSE          98
#define EADDRNOTAVAIL       99
#define ENETDOWN            100
#define ENETUNREACH         101
#define ENETRESET           102
#define ECONNABORTED        103
#define ECONNRESET          104
#define ENOBUFS             105
#define EISCONN             106
#define ENOTCONN            107
#define ESHUTDOWN           108                         /* Linux errno extension */
#define ETOOMANYREFS        109
#define ETIMEDOUT           110
#define ECONNREFUSED        111
#define EHOSTDOWN           112
#define EHOSTUNREACH        113
#define EALREADY            114
#define EINPROGRESS         115
#define ESTALE              116
#define EUCLEAN             117
#define ENOTNAM             118
#define ENAVAIL             119
#define EISNAM              120
#define EREMOTEIO           121
#define EDQUOT              122
#define ENOMEDIUM           123                         /* Linux errno extension */
#define EMEDIUMTYPE         124
#define ECANCELED           125
#define ENOKEY              126
#define EKEYEXPIRED         127
#define EKEYREVOKED         128
#define EKEYREJECTED        129
#define EOWNERDEAD          130
#define ENOTRECOVERABLE     131
#define ERFKILL             132
#define EHWPOISON           133

/* NuttX additional error codes */

#define ELBIN               134                         /* Linux errno extension */
#define EFTYPE              135
#define ENMFILE             136                         /* Cygwin */
#define EPROCLIM            137
#define ENOTSUP             138
#define ENOSHARE            139                         /* Cygwin */
#define ECASECLASH          140                         /* Cygwin */

#define __ELASTERROR        2000                        /* Users can add values starting here */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

/* Return a pointer to the thread specific errno. */

FAR int *__errno(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_ERRNO_H */
