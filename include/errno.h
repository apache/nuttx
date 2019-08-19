/****************************************************************************
 * include/errno.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_ERRNO_H
#define __INCLUDE_ERRNO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How can we access the errno variable? */

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
   /* Flat build */

#  if defined(CONFIG_LIB_SYSCALL) && !defined(__KERNEL__)
   /* We still might be using system calls in user code.  If so, then
    * user code will have no direct access to the errno variable.
    */

#    undef __DIRECT_ERRNO_ACCESS

#   else
   /* Flat build with no system calls OR internal kernel logic... There
    * is direct access.
    */

#    define __DIRECT_ERRNO_ACCESS 1
#  endif

#elif defined(CONFIG_BUILD_PROTECTED)
#  if defined(__KERNEL__)
   /* Kernel portion of protected build.  Kernel code has direct access */

#    define __DIRECT_ERRNO_ACCESS 1

#  else
   /* User portion of protected build.  Application code has only indirect
    * access
    */

#    undef __DIRECT_ERRNO_ACCESS
#  endif

#elif defined(CONFIG_BUILD_KERNEL) && !defined(__KERNEL__)
#  if defined(__KERNEL__)
   /* Kernel build.  Kernel code has direct access */

#    define __DIRECT_ERRNO_ACCESS 1

#  else
   /* User libraries for the kernel.  Only indirect access from user
    * libraries
   */

#    undef __DIRECT_ERRNO_ACCESS
#  endif
#endif

/* Convenience/compatibility definition.
 *
 * For a flat, all kernel-mode build, the error can be read and written
 * from all code using a simple pointer.
 */

#ifdef __DIRECT_ERRNO_ACCESS

#  define errno *get_errno_ptr()
#  define set_errno(e) do { errno = (int)(e); } while (0)
#  define get_errno(e) errno

#else

/* We doing separate user-/kernel-mode builds, then the errno has to be
 * a little differently. In kernel-mode, the TCB errno value can still be
 * read and written using a pointer from code executing within the
 * kernel.
 *
 * But in user-mode, the errno can only be read using the name 'errno'.
 * The non-standard API set_errno() must explicitly be used from user-
 * mode code in order to set the errno value.
 *
 * The same is true of the case where we have syscalls enabled but this
 * is not a kernel build, then we really have no option but to use the
 * set_errno() accessor function explicitly, even from OS logic!
 */

#  define errno get_errno()

#endif /* __DIRECT_ERRNO_ACCESS */

/* Definitions of error numbers and the string that would be
 * returned by strerror().
 */

#define EPERM               1
#define EPERM_STR           "Operation not permitted"
#define ENOENT              2
#define ENOENT_STR          "No such file or directory"
#define ESRCH               3
#define ESRCH_STR           "No such process"
#define EINTR               4
#define EINTR_STR           "Interrupted system call"
#define EIO                 5
#define EIO_STR             "I/O error"
#define ENXIO               6
#define ENXIO_STR           "No such device or address"
#define E2BIG               7
#define E2BIG_STR           "Arg list too long"
#define ENOEXEC             8
#define ENOEXEC_STR         "Exec format error"
#define EBADF               9
#define EBADF_STR           "Bad file number"
#define ECHILD              10
#define ECHILD_STR          "No child processes"
#define EAGAIN              11
#define EWOULDBLOCK         EAGAIN
#define EAGAIN_STR          "Try again"
#define ENOMEM              12
#define ENOMEM_STR          "Out of memory"
#define EACCES              13
#define EACCES_STR          "Permission denied"
#define EFAULT              14                         /* Linux errno extension */
#define EFAULT_STR          "Bad address"
#define ENOTBLK             15
#define ENOTBLK_STR         "Block device required"
#define EBUSY               16
#define EBUSY_STR           "Device or resource busy"
#define EEXIST              17
#define EEXIST_STR          "File exists"
#define EXDEV               18
#define EXDEV_STR           "Cross-device link"
#define ENODEV              19
#define ENODEV_STR          "No such device"
#define ENOTDIR             20
#define ENOTDIR_STR         "Not a directory"
#define EISDIR              21
#define EISDIR_STR          "Is a directory"
#define EINVAL              22
#define EINVAL_STR          "Invalid argument"
#define ENFILE              23
#define ENFILE_STR          "File table overflow"
#define EMFILE              24
#define EMFILE_STR          "Too many open files"
#define ENOTTY              25
#define ENOTTY_STR          "Not a typewriter"
#define ETXTBSY             26
#define ETXTBSY_STR         "Text file busy"
#define EFBIG               27
#define EFBIG_STR           "File too large"
#define ENOSPC              28
#define ENOSPC_STR          "No space left on device"
#define ESPIPE              29
#define ESPIPE_STR          "Illegal seek"
#define EROFS               30
#define EROFS_STR           "Read-only file system"
#define EMLINK              31
#define EMLINK_STR          "Too many links"
#define EPIPE               32
#define EPIPE_STR           "Broken pipe"
#define EDOM                33
#define EDOM_STR            "Math argument out of domain of func"
#define ERANGE              34
#define ERANGE_STR          "Math result not representable"
#define ENOMSG              35
#define ENOMSG_STR          "No message of desired type"
#define EIDRM               36
#define EIDRM_STR           "Identifier removed"
#define ECHRNG              37                         /* Linux errno extension */
#define ECHRNG_STR          "Channel number out of range"
#define EL2NSYNC            38                         /* Linux errno extension */
#define EL2NSYNC_STR        "Level 2 not synchronized"
#define EL3HLT              39                         /* Linux errno extension */
#define EL3HLT_STR          "Level 3 halted"
#define EL3RST              40                         /* Linux errno extension */
#define EL3RST_STR          "Level 3 reset"
#define ELNRNG              41                         /* Linux errno extension */
#define ELNRNG_STR          "Link number out of range"
#define EUNATCH             42                         /* Linux errno extension */
#define EUNATCH_STR         "Protocol driver not attached"
#define ENOCSI              43                         /* Linux errno extension */
#define ENOCSI_STR          "No CSI structure available"
#define EL2HLT              44                         /* Linux errno extension */
#define EL2HLT_STR          "Level 2 halted"
#define EDEADLK             45
#define EDEADLK_STR         "Resource deadlock would occur"
#define ENOLCK              46
#define ENOLCK_STR          "No record locks available"

#define EBADE               50                         /* Linux errno extension */
#define EBADE_STR           "Invalid exchange"
#define EBADR               51                         /* Linux errno extension */
#define EBADR_STR           "Invalid request descriptor"
#define EXFULL              52                         /* Linux errno extension */
#define EXFULL_STR          "Exchange full"
#define ENOANO              53                         /* Linux errno extension */
#define ENOANO_STR          "No anode"
#define EBADRQC             54                         /* Linux errno extension */
#define EBADRQC_STR         "Invalid request code"
#define EBADSLT             55                         /* Linux errno extension */
#define EBADSLT_STR         "Invalid slot"
#define EDEADLOCK           56                         /* Linux errno extension */
#define EDEADLOCK_STR       "File locking deadlock error"
#define EBFONT              57                         /* Linux errno extension */
#define EBFONT_STR          "Bad font file format"

#define ENOSTR              60
#define ENOSTR_STR          "Device not a stream"
#define ENODATA             61
#define ENODATA_STR         "No data available"
#define ETIME               62
#define ETIME_STR           "Timer expired"
#define ENOSR               63
#define ENOSR_STR           "Out of streams resources"
#define ENONET              64                         /* Linux errno extension */
#define ENONET_STR          "Machine is not on the network"
#define ENOPKG              65                         /* Linux errno extension */
#define ENOPKG_STR          "Package not installed"
#define EREMOTE             66                         /* Linux errno extension */
#define EREMOTE_STR         "Object is remote"
#define ENOLINK             67
#define ENOLINK_STR         "Link has been severed"
#define EADV                68                         /* Linux errno extension */
#define EADV_STR            "Advertise error"
#define ESRMNT              69                         /* Linux errno extension */
#define ESRMNT_STR          "Srmount error"
#define ECOMM               70                         /* Linux errno extension */
#define ECOMM_STR           "Communication error on send"
#define EPROTO              71
#define EPROTO_STR          "Protocol error"

#define EMULTIHOP           74
#define EMULTIHOP_STR       "Multihop attempted"
#define ELBIN               75                         /* Linux errno extension */
#define ELBIN_STR           "Inode is remote"
#define EDOTDOT             76                         /* Linux errno extension */
#define EDOTDOT_STR         "RFS specific error"
#define EBADMSG             77
#define EBADMSG_STR         "Not a data message"

#define EFTYPE              79
#define EFTYPE_STR          "Inappropriate file type or format"
#define ENOTUNIQ            80                         /* Linux errno extension */
#define ENOTUNIQ_STR        "Name not unique on network"
#define EBADFD              81                         /* Linux errno extension */
#define EBADFD_STR          "File descriptor in bad state"
#define EREMCHG             82                         /* Linux errno extension */
#define EREMCHG_STR         "Remote address changed"
#define ELIBACC             83                         /* Linux errno extension */
#define ELIBACC_STR         "Can not access a needed shared library"
#define ELIBBAD             84                         /* Linux errno extension */
#define ELIBBAD_STR         "Accessing a corrupted shared library"
#define ELIBSCN             85                         /* Linux errno extension */
#define ELIBSCN_STR         ".lib section in a.out corrupted"
#define ELIBMAX             86                         /* Linux errno extension */
#define ELIBMAX_STR         "Attempting to link in too many shared libraries"
#define ELIBEXEC            87                         /* Linux errno extension */
#define ELIBEXEC_STR        "Cannot exec a shared library directly"
#define ENOSYS              88
#define ENOSYS_STR          "Function not implemented"
#define ENMFILE             89                         /* Cygwin */
#define ENMFILE_STR         "No more files"
#define ENOTEMPTY           90
#define ENOTEMPTY_STR       "Directory not empty"
#define ENAMETOOLONG        91
#define ENAMETOOLONG_STR    "File name too long"
#define ELOOP               92
#define ELOOP_STR           "Too many symbolic links encountered"

#define EOPNOTSUPP          95
#define EOPNOTSUPP_STR      "Operation not supported on transport endpoint"
#define EPFNOSUPPORT        96
#define EPFNOSUPPORT_STR    "Protocol family not supported"

#define ECONNRESET          104
#define ECONNRESET_STR      "Connection reset by peer"
#define ENOBUFS             105
#define ENOBUFS_STR         "No buffer space available"
#define EAFNOSUPPORT        106
#define EAFNOSUPPORT_STR    "Address family not supported by protocol"
#define EPROTOTYPE          107
#define EPROTOTYPE_STR      "Protocol wrong type for socket"
#define ENOTSOCK            108
#define ENOTSOCK_STR        "Socket operation on non-socket"
#define ENOPROTOOPT         109
#define ENOPROTOOPT_STR     "Protocol not available"
#define ESHUTDOWN           110                         /* Linux errno extension */
#define ESHUTDOWN_STR       "Cannot send after transport endpoint shutdown"
#define ECONNREFUSED        111
#define ECONNREFUSED_STR    "Connection refused"
#define EADDRINUSE          112
#define EADDRINUSE_STR      "Address already in use"
#define ECONNABORTED        113
#define ECONNABORTED_STR    "Software caused connection abort"
#define ENETUNREACH         114
#define ENETUNREACH_STR     "Network is unreachable"
#define ENETDOWN            115
#define ENETDOWN_STR        "Network is down"
#define ETIMEDOUT           116
#define ETIMEDOUT_STR       "Connection timed out"
#define EHOSTDOWN           117
#define EHOSTDOWN_STR       "Host is down"
#define EHOSTUNREACH        118
#define EHOSTUNREACH_STR    "No route to host"
#define EINPROGRESS         119
#define EINPROGRESS_STR     "Operation now in progress"
#define EALREADY            120
#define EALREADY_STR        "Socket already connected"
#define EDESTADDRREQ        121
#define EDESTADDRREQ_STR    "Destination address required"
#define EMSGSIZE            122
#define EMSGSIZE_STR        "Message too long"
#define EPROTONOSUPPORT     123
#define EPROTONOSUPPORT_STR "Protocol not supported"
#define ESOCKTNOSUPPORT     124                         /* Linux errno extension */
#define ESOCKTNOSUPPORT_STR "Socket type not supported"
#define EADDRNOTAVAIL       125
#define EADDRNOTAVAIL_STR   "Cannot assign requested address"
#define ENETRESET           126
#define ENETRESET_STR       "Network dropped connection because of reset"
#define EISCONN             127
#define EISCONN_STR         "Transport endpoint is already connected"
#define ENOTCONN            128
#define ENOTCONN_STR        "Transport endpoint is not connected"
#define ETOOMANYREFS        129
#define ETOOMANYREFS_STR    "Too many references: cannot splice"
#define EPROCLIM            130
#define EPROCLIM_STR        "Limit would be exceeded by attempted fork"
#define EUSERS              131
#define EUSERS_STR          "Too many users"
#define EDQUOT              132
#define EDQUOT_STR          "Quota exceeded"
#define ESTALE              133
#define ESTALE_STR          "Stale NFS file handle"
#define ENOTSUP             134
#define ENOTSUP_STR         "Not supported"
#define ENOMEDIUM           135                         /* Linux errno extension */
#define ENOMEDIUM_STR       "No medium found"
#define ENOSHARE            136                         /* Cygwin */
#define ENOSHARE_STR        "No such host or network path"
#define ECASECLASH          137                         /* Cygwin */
#define ECASECLASH_STR      "Filename exists with different case"
#define EILSEQ              138
#define EILSEQ_STR          "Illegal byte sequence"
#define EOVERFLOW           139
#define EOVERFLOW_STR       "Value too large for defined data type"
#define ECANCELED           140
#define ECANCELED_STR       "Operation cancelled"
#define ENOTRECOVERABLE     141
#define ENOTRECOVERABLE_STR "State not recoverable"
#define EOWNERDEAD          142
#define EOWNERDEAD_STR      "Previous owner died"
#define ESTRPIPE            143                         /* Linux errno extension */
#define ESTRPIPE_STR        "Streams pipe error"

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

/* Return a pointer to the thread specific errno.  NOTE:  When doing a
 * kernel-/user-mode build, this function can only be used within the
 * kernel-mode space.
 *
 * In the user-mode space, set_errno() and get_errno() are always available,
 * either as macros or via syscalls.
 */

FAR int *get_errno_ptr(void);

#ifndef __DIRECT_ERRNO_ACCESS
void set_errno(int errcode);
int  get_errno(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_ERRNO_H */
