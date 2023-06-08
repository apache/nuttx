/****************************************************************************
 * drivers/modem/alt1250/altcom_errno.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_ERRNO_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_ERRNO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_EPERM              1
#define ALTCOM_ENOENT             2
#define ALTCOM_ESRCH              3
#define ALTCOM_EINTR              4
#define ALTCOM_EIO                5
#define ALTCOM_ENXIO              6
#define ALTCOM_E2BIG              7
#define ALTCOM_ENOEXEC            8
#define ALTCOM_EBADF              9
#define ALTCOM_ECHILD             10
#define ALTCOM_EAGAIN             11
#define ALTCOM_ENOMEM             12
#define ALTCOM_EACCES             13
#define ALTCOM_EFAULT             14
#define ALTCOM_ENOTBLK            15
#define ALTCOM_EBUSY              16
#define ALTCOM_EEXIST             17
#define ALTCOM_EXDEV              18
#define ALTCOM_ENODEV             19
#define ALTCOM_ENOTDIR            20
#define ALTCOM_EISDIR             21
#define ALTCOM_EINVAL             22
#define ALTCOM_ENFILE             23
#define ALTCOM_EMFILE             24
#define ALTCOM_ENOTTY             25
#define ALTCOM_ETXTBSY            26
#define ALTCOM_EFBIG              27
#define ALTCOM_ENOSPC             28
#define ALTCOM_ESPIPE             29
#define ALTCOM_EROFS              30
#define ALTCOM_EMLINK             31
#define ALTCOM_EPIPE              32
#define ALTCOM_EDOM               33
#define ALTCOM_ERANGE             34
#define ALTCOM_ENOMSG             35
#define ALTCOM_EIDRM              36
#define ALTCOM_ECHRNG             37
#define ALTCOM_EL2NSYNC           38
#define ALTCOM_EL3HLT             39
#define ALTCOM_EL3RST             40
#define ALTCOM_ELNRNG             41
#define ALTCOM_EUNATCH            42
#define ALTCOM_ENOCSI             43
#define ALTCOM_EL2HLT             44
#define ALTCOM_EDEADLK            45
#define ALTCOM_ENOLCK             46
#define ALTCOM_EBADE              50
#define ALTCOM_EBADR              51
#define ALTCOM_EXFULL             52
#define ALTCOM_ENOANO             53
#define ALTCOM_EBADRQC            54
#define ALTCOM_EBADSLT            55
#define ALTCOM_EDEADLOCK          56
#define ALTCOM_EBFONT             57
#define ALTCOM_ENOSTR             60
#define ALTCOM_ENODATA            61
#define ALTCOM_ETIME              62
#define ALTCOM_ENOSR              63
#define ALTCOM_ENONET             64
#define ALTCOM_ENOPKG             65
#define ALTCOM_EREMOTE            66
#define ALTCOM_ENOLINK            67
#define ALTCOM_EADV               68
#define ALTCOM_ESRMNT             69
#define ALTCOM_ECOMM              70
#define ALTCOM_EPROTO             71
#define ALTCOM_EMULTIHOP          74
#define ALTCOM_ELBIN              75
#define ALTCOM_EDOTDOT            76
#define ALTCOM_EBADMSG            77
#define ALTCOM_EFTYPE             79
#define ALTCOM_ENOTUNIQ           80
#define ALTCOM_EBADFD             81
#define ALTCOM_EREMCHG            82
#define ALTCOM_ELIBACC            83
#define ALTCOM_ELIBBAD            84
#define ALTCOM_ELIBSCN            85
#define ALTCOM_ELIBMAX            86
#define ALTCOM_ELIBEXEC           87
#define ALTCOM_ENOSYS             88
#define ALTCOM_ENMFILE            89
#define ALTCOM_ENOTEMPTY          90
#define ALTCOM_ENAMETOOLONG       91
#define ALTCOM_ELOOP              92
#define ALTCOM_EOPNOTSUPP         95
#define ALTCOM_EPFNOSUPPORT       96
#define ALTCOM_ECONNRESET         104
#define ALTCOM_ENOBUFS            105
#define ALTCOM_EAFNOSUPPORT       106
#define ALTCOM_EPROTOTYPE         107
#define ALTCOM_ENOTSOCK           108
#define ALTCOM_ENOPROTOOPT        109
#define ALTCOM_ESHUTDOWN          110
#define ALTCOM_ECONNREFUSED       111
#define ALTCOM_EADDRINUSE         112
#define ALTCOM_ECONNABORTED       113
#define ALTCOM_ENETUNREACH        114
#define ALTCOM_ENETDOWN           115
#define ALTCOM_ETIMEDOUT          116
#define ALTCOM_EHOSTDOWN          117
#define ALTCOM_EHOSTUNREACH       118
#define ALTCOM_EINPROGRESS        119
#define ALTCOM_EALREADY           120
#define ALTCOM_EDESTADDRREQ       121
#define ALTCOM_EMSGSIZE           122
#define ALTCOM_EPROTONOSUPPORT    123
#define ALTCOM_ESOCKTNOSUPPORT    124
#define ALTCOM_EADDRNOTAVAIL      125
#define ALTCOM_ENETRESET          126
#define ALTCOM_EISCONN            127
#define ALTCOM_ENOTCONN           128
#define ALTCOM_ETOOMANYREFS       129
#define ALTCOM_EPROCLIM           130
#define ALTCOM_EUSERS             131
#define ALTCOM_EDQUOT             132
#define ALTCOM_ESTALE             133
#define ALTCOM_ENOTSUP            134
#define ALTCOM_ENOMEDIUM          135
#define ALTCOM_ENOSHARE           136
#define ALTCOM_ECASECLASH         137
#define ALTCOM_EILSEQ             138
#define ALTCOM_EOVERFLOW          139
#define ALTCOM_ECANCELED          140
#define ALTCOM_ENOTRECOVERABLE    141
#define ALTCOM_EOWNERDEAD         142
#define ALTCOM_ESTRPIPE           143
#define ALTCOM_EWOULDBLOCK        ALTCOM_EAGAIN

#define ALTCOM_ERRNO_MIN          ALTCOM_EPERM
#define ALTCOM_ERRNO_MAX          ALTCOM_ESTRPIPE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_errno2nuttx
 *
 * Description:
 *   Get Nuttx errno from ALTCOM errno.
 *
 * Input Parameters:
 *   altcom_errno   -  ALTCOM errno.
 *
 * Returned Value:
 *   Returns NuttX errno.
 *
 ****************************************************************************/

int altcom_errno2nuttx(int altcom_errno);

/****************************************************************************
 * Name: altcom_geterrcode
 *
 * Description:
 *   Get Nuttx errno from ALTCOM paccket.
 *
 * Input Parameters:
 *   err_code   -  The err_code of the ALTCOM packet.
 *
 * Returned Value:
 *   Returns NuttX errno.
 *
 ****************************************************************************/

int altcom_geterrcode(int32_t err_code);

#endif /* __DRIVERS_MODEM_ALT1250_ALTCOM_ERRNO_H */
