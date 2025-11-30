/****************************************************************************
 * include/nuttx/errno_lookup.h
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

/* Sync with linux/include/asm-generic/errno-base.h */

ERRNO_ITEM(EPERM,            1,  "Operation not permitted")
ERRNO_ITEM(ENOENT,           2,  "No such file or directory")
ERRNO_ITEM(ESRCH,            3,  "No such process")
ERRNO_ITEM(EINTR,            4,  "Interrupted system call")
ERRNO_ITEM(EIO,              5,  "I/O error")
ERRNO_ITEM(ENXIO,            6,  "No such device or address")
ERRNO_ITEM(E2BIG,            7,  "Arg list too long")
ERRNO_ITEM(ENOEXEC,          8,  "Exec format error")
ERRNO_ITEM(EBADF,            9,  "Bad file number")
ERRNO_ITEM(ECHILD,          10,  "No child processes")
ERRNO_ITEM(EAGAIN,          11,  "Try again")
ERRNO_ITEM(ENOMEM,          12,  "Out of memory")
ERRNO_ITEM(EACCES,          13,  "Permission denied")
ERRNO_ITEM(EFAULT,          14,  "Bad address")
ERRNO_ITEM(ENOTBLK,         15,  "Block device required")
ERRNO_ITEM(EBUSY,           16,  "Device or resource busy")
ERRNO_ITEM(EEXIST,          17,  "File exists")
ERRNO_ITEM(EXDEV,           18,  "Cross-device link")
ERRNO_ITEM(ENODEV,          19,  "No such device")
ERRNO_ITEM(ENOTDIR,         20,  "Not a directory")
ERRNO_ITEM(EISDIR,          21,  "Is a directory")
ERRNO_ITEM(EINVAL,          22,  "Invalid argument")
ERRNO_ITEM(ENFILE,          23,  "File table overflow")
ERRNO_ITEM(EMFILE,          24,  "Too many open files")
ERRNO_ITEM(ENOTTY,          25,  "Not a typewriter")
ERRNO_ITEM(ETXTBSY,         26,  "Text file busy")
ERRNO_ITEM(EFBIG,           27,  "File too large")
ERRNO_ITEM(ENOSPC,          28,  "No space left on device")
ERRNO_ITEM(ESPIPE,          29,  "Illegal seek")
ERRNO_ITEM(EROFS,           30,  "Read-only file system")
ERRNO_ITEM(EMLINK,          31,  "Too many links")
ERRNO_ITEM(EPIPE,           32,  "Broken pipe")
ERRNO_ITEM(EDOM,            33,  "Math argument out of domain of func")
ERRNO_ITEM(ERANGE,          34,  "Math result not representable")
ERRNO_ITEM(EDEADLK,         35,  "Resource deadlock would occur")

/* Sync with linux/include/asm-generic/errno.h */

ERRNO_ITEM(ENAMETOOLONG,    36,  "File name too long")
ERRNO_ITEM(ENOLCK,          37,  "No record locks available")
ERRNO_ITEM(ENOSYS,          38,  "Invalid system call number")
ERRNO_ITEM(ENOTEMPTY,       39,  "Directory not empty")
ERRNO_ITEM(ELOOP,           40,  "Too many symbolic links encountered")

/* ERRNO_ITEM(EWOULDBLOCK,  EAGAIN,  "Operation would block") */

ERRNO_ITEM(ENOMSG,          42,  "No message of desired type")
ERRNO_ITEM(EIDRM,           43,  "Identifier removed")
ERRNO_ITEM(ECHRNG,          44,  "Channel number out of range")
ERRNO_ITEM(EL2NSYNC,        45,  "Level 2 not synchronized")
ERRNO_ITEM(EL3HLT,          46,  "Level 3 halted")
ERRNO_ITEM(EL3RST,          47,  "Level 3 reset")
ERRNO_ITEM(ELNRNG,          48,  "Link number out of range")
ERRNO_ITEM(EUNATCH,         49,  "Protocol driver not attached")
ERRNO_ITEM(ENOCSI,          50,  "No CSI structure available")
ERRNO_ITEM(EL2HLT,          51,  "Level 2 halted")
ERRNO_ITEM(EBADE,           52,  "Invalid exchange")
ERRNO_ITEM(EBADR,           53,  "Invalid request descriptor")
ERRNO_ITEM(EXFULL,          54,  "Exchange full")
ERRNO_ITEM(ENOANO,          55,  "No anode")
ERRNO_ITEM(EBADRQC,         56,  "Invalid request code")
ERRNO_ITEM(EBADSLT,         57,  "Invalid slot")

/* ERRNO_ITEM(EDEADLOCK,    EDEADLK,  "File locking deadlock error") */

ERRNO_ITEM(EBFONT,          59,  "Bad font file format")
ERRNO_ITEM(ENOSTR,          60,  "Device not a stream")
ERRNO_ITEM(ENODATA,         61,  "No data available")
ERRNO_ITEM(ETIME,           62,  "Timer expired")
ERRNO_ITEM(ENOSR,           63,  "Out of streams resources")
ERRNO_ITEM(ENONET,          64,  "Machine is not on the network")
ERRNO_ITEM(ENOPKG,          65,  "Package not installed")
ERRNO_ITEM(EREMOTE,         66,  "Object is remote")
ERRNO_ITEM(ENOLINK,         67,  "Link has been severed")
ERRNO_ITEM(EADV,            68,  "Advertise error")
ERRNO_ITEM(ESRMNT,          69,  "Srmount error")
ERRNO_ITEM(ECOMM,           70,  "Communication error on send")
ERRNO_ITEM(EPROTO,          71,  "Protocol error")
ERRNO_ITEM(EMULTIHOP,       72,  "Multihop attempted")
ERRNO_ITEM(EDOTDOT,         73,  "RFS specific error")
ERRNO_ITEM(EBADMSG,         74,  "Not a data message")
ERRNO_ITEM(EOVERFLOW,       75,  "Value too large for defined data type")
ERRNO_ITEM(ENOTUNIQ,        76,  "Name not unique on network")
ERRNO_ITEM(EBADFD,          77,  "File descriptor in bad state")
ERRNO_ITEM(EREMCHG,         78,  "Remote address changed")
ERRNO_ITEM(ELIBACC,         79,  "Can not access a needed shared library")
ERRNO_ITEM(ELIBBAD,         80,  "Accessing a corrupted shared library")
ERRNO_ITEM(ELIBSCN,         81,  ".lib section in a.out corrupted")
ERRNO_ITEM(ELIBMAX,         82,
           "Attempting to link in too many shared libraries")
ERRNO_ITEM(ELIBEXEC,        83,  "Cannot exec a shared library directly")
ERRNO_ITEM(EILSEQ,          84,  "Illegal byte sequence")
ERRNO_ITEM(ERESTART,        85,
           "Interrupted system call should be restarted")
ERRNO_ITEM(ESTRPIPE,        86,  "Streams pipe error")
ERRNO_ITEM(EUSERS,          87,  "Too many users")
ERRNO_ITEM(ENOTSOCK,        88,  "Socket operation on non-socket")
ERRNO_ITEM(EDESTADDRREQ,    89,  "Destination address required")
ERRNO_ITEM(EMSGSIZE,        90,  "Message too long")
ERRNO_ITEM(EPROTOTYPE,      91,  "Protocol wrong type for socket")
ERRNO_ITEM(ENOPROTOOPT,     92,  "Protocol not available")
ERRNO_ITEM(EPROTONOSUPPORT, 93,  "Protocol not supported")
ERRNO_ITEM(ESOCKTNOSUPPORT, 94,  "Socket type not supported")
ERRNO_ITEM(EOPNOTSUPP,      95,
           "Operation not supported on transport endpoint")
ERRNO_ITEM(EPFNOSUPPORT,    96,  "Protocol family not supported")
ERRNO_ITEM(EAFNOSUPPORT,    97,  "Address family not supported by protocol")
ERRNO_ITEM(EADDRINUSE,      98,  "Address already in use")
ERRNO_ITEM(EADDRNOTAVAIL,   99,  "Cannot assign requested address")
ERRNO_ITEM(ENETDOWN,        100, "Network is down")
ERRNO_ITEM(ENETUNREACH,     101, "Network is unreachable")
ERRNO_ITEM(ENETRESET,       102,
           "Network dropped connection because of reset")
ERRNO_ITEM(ECONNABORTED,    103, "Software caused connection abort")
ERRNO_ITEM(ECONNRESET,      104, "Connection reset by peer")
ERRNO_ITEM(ENOBUFS,         105, "No buffer space available")
ERRNO_ITEM(EISCONN,         106, "Transport endpoint is already connected")
ERRNO_ITEM(ENOTCONN,        107, "Transport endpoint is not connected")
ERRNO_ITEM(ESHUTDOWN,       108,
           "Cannot send after transport endpoint shutdown")
ERRNO_ITEM(ETOOMANYREFS,    109, "Too many references: cannot splice")
ERRNO_ITEM(ETIMEDOUT,       110, "Connection timed out")
ERRNO_ITEM(ECONNREFUSED,    111, "Connection refused")
ERRNO_ITEM(EHOSTDOWN,       112, "Host is down")
ERRNO_ITEM(EHOSTUNREACH,    113, "No route to host")
ERRNO_ITEM(EALREADY,        114, "Operation already in progress")
ERRNO_ITEM(EINPROGRESS,     115, "Operation now in progress")
ERRNO_ITEM(ESTALE,          116, "Stale file handle")
ERRNO_ITEM(EUCLEAN,         117, "Structure needs cleaning")
ERRNO_ITEM(ENOTNAM,         118, "Not a XENIX named type file")
ERRNO_ITEM(ENAVAIL,         119, "No XENIX semaphores available")
ERRNO_ITEM(EISNAM,          120, "Is a named type file")
ERRNO_ITEM(EREMOTEIO,       121, "Remote I/O error")
ERRNO_ITEM(EDQUOT,          122, "Quota exceeded")
ERRNO_ITEM(ENOMEDIUM,       123, "No medium found")
ERRNO_ITEM(EMEDIUMTYPE,     124, "Wrong medium type")
ERRNO_ITEM(ECANCELED,       125, "Operation cancelled")
ERRNO_ITEM(ENOKEY,          126, "Required key not available")
ERRNO_ITEM(EKEYEXPIRED,     127, "Key has expired")
ERRNO_ITEM(EKEYREVOKED,     128, "Key has been revoked")
ERRNO_ITEM(EKEYREJECTED,    129, "Key was rejected by service")
ERRNO_ITEM(EOWNERDEAD,      130, "Previous owner died")
ERRNO_ITEM(ENOTRECOVERABLE, 131, "State not recoverable")
ERRNO_ITEM(ERFKILL,         132, "Operation not possible due to RF-kill")
ERRNO_ITEM(EHWPOISON,       133, "Memory page has hardware error")

/* NuttX additional error codes */

ERRNO_ITEM(ELBIN,           134, "Inode is remote")
ERRNO_ITEM(EFTYPE,          135, "Inappropriate file type or format")
ERRNO_ITEM(ENMFILE,         136, "No more files")
ERRNO_ITEM(EPROCLIM,        137, "Limit would be exceeded by attempted fork")
ERRNO_ITEM(ENOTSUP,         138, "Not supported")
ERRNO_ITEM(ENOSHARE,        139, "No such host or network path")
ERRNO_ITEM(ECASECLASH,      140, "Filename exists with different case")
