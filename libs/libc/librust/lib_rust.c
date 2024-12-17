/****************************************************************************
 * libs/libc/librust/lib_rust.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <dlfcn.h>
#include <pthread.h>
#include <semaphore.h>
#include <locale.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/poll.h>
#include <sys/resource.h>
#include <pwd.h>
#include <time.h>
#include <netdb.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>
#include <sys/un.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * This file contains definitions and assertions to ensure compatibility
 * between C and Rust data structures. It defines the sizes of various
 * POSIX structures and asserts that their sizes are compatible with Rust.
 *
 * Most of these sizes are defined in terms of the size of a pointer, which
 * can be either 4 or 8 bytes depending on the architecture. The sizes are
 * defined in terms of the number of pointer-sized elements they contain.
 *
 * Most of these structures have a reserved space for future use, and
 * the current actual size of them are calculated on riscv32 platform.
 */

#undef NP
#define NP(x) (sizeof(void *) * (x))

#define __PTHREAD_ATTR_SIZE__     (NP(5))  /* Actual size is 3 */
#define __PTHREAD_MUTEX_SIZE__    (NP(9))  /* Actual size is 7 */
#define __PTHREAD_COND_SIZE__     (NP(7))  /* Actual size is 5 */
#define __PTHREAD_CONDATTR_SIZE__ (NP(5))  /* Actual size is 3 */
#define __PTHREAD_RWLOCK_SIZE__   (NP(17)) /* Actual size is 15 */
#define __SEM_SIZE__              (NP(6))  /* Actual size is 4 */
#define __STAT_SIZE__             (NP(28)) /* Actual size is 26 */
#define __DIRENT_SIZE__           (NP(20)) /* Actual size is 17.5 */
#define __STATVFS_SIZE__          (NP(20)) /* Actual size is 18 */
#define __PASSWD_SIZE__           (NP(16)) /* Actual size is 14 */
#define __DL_INFO_SIZE__          (NP(4))  /* Actual size is 4 */
#define __LCONV_SIZE__            (NP(40)) /* Actual size is 38 */
#define __TM_SIZE__               (NP(13)) /* Actual size is 11 */
#define __ADDRINFO_SIZE__         (NP(10)) /* Actual size is 8 */
#define __FDSET_SIZE__            (NP(10)) /* Actual size is 8 */
#define __SIGSET_SIZE__           (NP(4))  /* Actual size is 2 */
#define __SIGACTION_SIZE__        (NP(7))  /* Actual size is 5 */
#define __TERMIOS_SIZE__          (NP(10)) /* Actual size is 8 */
#define __SOCKADDR_STORAGE_SIZE__ (NP(36)) /* Actual size is 34*/
#define __NAME_MAX_SIZE__         (64)

#define ALIGNUP_TO(x, align) (((x) + (align) - 1) & ~((align) - 1))

/**
 * This section contains static assertions to ensure that the sizes of
 * various POSIX data types and structures are compatible with Rust.
 * These assertions are crucial for maintaining interoperability between
 * C and Rust code.
 *
 * The assertions check the sizes of common POSIX types such as pid_t,
 * pthread_t, and various structures like pthread_attr_t, pthread_mutex_t,
 * etc. They also verify the sizes of structures used in file system
 * operations, networking, and threading.
 *
 * If any of these assertions fail, it indicates a mismatch between
 * the expected sizes in Rust and the actual sizes in C, which could lead
 * to runtime errors or undefined behavior when passing data between
 * the Rust side and NuttX side.
 */

static_assert(sizeof(pid_t) == sizeof(int), "pid_t size mismatch with Rust");
static_assert(sizeof(pthread_t) == sizeof(int),
              "pthread_t size mismatch with Rust");
static_assert(sizeof(pthread_attr_t) <= __PTHREAD_ATTR_SIZE__,
              "pthread_attr_t too large for Rust");
static_assert(sizeof(pthread_mutex_t) <= __PTHREAD_MUTEX_SIZE__,
              "pthread_mutex_t too large for Rust");
static_assert(sizeof(pthread_cond_t) <= __PTHREAD_COND_SIZE__,
              "pthread_cond_t too large for Rust");
static_assert(sizeof(pthread_condattr_t) <= __PTHREAD_CONDATTR_SIZE__,
              "pthread_condattr_t too large for Rust");
static_assert(sizeof(pthread_rwlock_t) <= __PTHREAD_RWLOCK_SIZE__,
              "pthread_rwlock_t too large for Rust");
static_assert(sizeof(sem_t) <= __SEM_SIZE__, "sem_t too large for Rust");
static_assert(sizeof(struct stat) <= __STAT_SIZE__,
              "struct stat size mismatch with Rust");
static_assert(sizeof(struct statvfs) <= __STATVFS_SIZE__,
              "struct statvfs too large for Rust");
static_assert(sizeof(struct passwd) <= __PASSWD_SIZE__,
              "struct passwd size mismatch with Rust");
static_assert(sizeof(Dl_info) == __DL_INFO_SIZE__,
              "Dl_info size mismatch with Rust");
static_assert(sizeof(struct lconv) <= __LCONV_SIZE__,
              "__Lconv size mismatch with Rust");
static_assert(sizeof(struct tm) <= __TM_SIZE__,
              "struct tm too large for Rust");
static_assert(sizeof(struct addrinfo) <= __ADDRINFO_SIZE__,
              "struct addrinfo too large for Rust");
static_assert(sizeof(fd_set) <= __FDSET_SIZE__,
              "struct fd_set too large for Rust");
static_assert(sizeof(sigset_t) <= __SIGSET_SIZE__,
              "sigset_t too large for Rust");
static_assert(sizeof(struct sigaction) <= __SIGACTION_SIZE__,
              "struct sigaction too large for Rust");
static_assert(sizeof(struct termios) <= __TERMIOS_SIZE__,
              "struct termios too large for Rust");

static_assert(NAME_MAX <= __NAME_MAX_SIZE__,
              "NAME_MAX size mismatch with Rust");

static_assert(sizeof(blkcnt_t) == 8, "blkcnt_t size mismatch with Rust");
static_assert(sizeof(blksize_t) == 2, "blksize_t size mismatch with Rust");
static_assert(sizeof(cc_t) == 1, "cc_t size mismatch with Rust");
static_assert(sizeof(clock_t) == 8, "clock_t size mismatch with Rust");
static_assert(sizeof(dev_t) == 4, "dev_t size mismatch with Rust");
static_assert(sizeof(fsblkcnt_t) == 8, "fsblkcnt_t size mismatch with Rust");
static_assert(sizeof(locale_t) == sizeof(void *),
              "locale_t size mismatch with Rust");
static_assert(sizeof(mode_t) == 4, "mode_t size mismatch with Rust");
static_assert(sizeof(nfds_t) == 4, "nfds_t size mismatch with Rust");
static_assert(sizeof(off_t) == 8, "off_t size mismatch with Rust");
static_assert(sizeof(pthread_key_t) == 4,
              "pthread_key_t size mismatch with Rust");
static_assert(sizeof(pthread_mutexattr_t) == 1,
              "pthread_mutexattr_t size mismatch with Rust");
static_assert(sizeof(pthread_rwlockattr_t) == 4,
              "pthread_rwlockattr_t size mismatch with Rust");
static_assert(sizeof(pthread_t) == 4, "pthread_t size mismatch with Rust");
static_assert(sizeof(rlim_t) == 8, "rlim_t size mismatch with Rust");
static_assert(sizeof(sa_family_t) == 2,
              "sa_family_t size mismatch with Rust");
static_assert(sizeof(socklen_t) == 4, "socklen_t size mismatch with Rust");
static_assert(sizeof(speed_t) == sizeof(unsigned long),
              "speed_t size mismatch with Rust");
static_assert(sizeof(suseconds_t) == 4,
              "suseconds_t size mismatch with Rust");
static_assert(sizeof(tcflag_t) == 4, "tcflag_t size mismatch with Rust");
static_assert(sizeof(time_t) == 8, "time_t size mismatch with Rust");
static_assert(sizeof(wchar_t) == 4, "wchar_t size mismatch with Rust");

/* Field ordering checks */

/**
 * This section contains static assertions to ensure that the field offsets
 * for compatibility with Rust, ensuring that the memory layout is consistent
 * across both languages.
 */

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct stat` match the expected values.
 */

static_assert(offsetof(struct stat, st_dev) == 0, "st_dev offset mismatch");
static_assert(offsetof(struct stat, st_ino) ==
                  offsetof(struct stat, st_dev) + sizeof(dev_t),
              "st_ino offset mismatch");
static_assert(offsetof(struct stat, st_mode) ==
                  offsetof(struct stat, st_ino) + sizeof(mode_t),
              "st_mode offset mismatch");
static_assert(offsetof(struct stat, st_nlink) ==
                  offsetof(struct stat, st_mode) + sizeof(mode_t),
              "st_nlink offset mismatch");
static_assert(offsetof(struct stat, st_uid) ==
                  offsetof(struct stat, st_nlink) + sizeof(uid_t),
              "st_uid offset mismatch");
static_assert(offsetof(struct stat, st_gid) ==
                  offsetof(struct stat, st_uid) + sizeof(gid_t),
              "st_gid offset mismatch");
static_assert(offsetof(struct stat, st_rdev) ==
                  offsetof(struct stat, st_gid) + sizeof(dev_t),
              "st_rdev offset mismatch");
static_assert(offsetof(struct stat, st_size) ==
                  offsetof(struct stat, st_rdev) + sizeof(off_t),
              "st_size offset mismatch");
static_assert(offsetof(struct stat, st_atim) ==
                  offsetof(struct stat, st_size) + sizeof(off_t),
              "st_atim offset mismatch");
static_assert(offsetof(struct stat, st_mtim) ==
                  offsetof(struct stat, st_atim) + sizeof(struct timespec),
              "st_mtim offset mismatch");
static_assert(offsetof(struct stat, st_ctim) ==
                  offsetof(struct stat, st_mtim) + sizeof(struct timespec),
              "st_ctim offset mismatch");
static_assert(offsetof(struct stat, st_blksize) ==
                  offsetof(struct stat, st_ctim) + sizeof(struct timespec),
              "st_blksize offset mismatch");
static_assert(offsetof(struct stat, st_blocks) ==
                  offsetof(struct stat, st_blksize) + sizeof(blkcnt_t),
              "st_blocks offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct in_addr` match the expected values.
 */

static_assert(sizeof(struct in_addr) == 4,
              "struct in_addr size mismatch with Rust");
static_assert(offsetof(struct in_addr, s_addr) == 0,
              "s_addr offset mismatch in struct in_addr");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct dirent` match the expected values.
 */

static_assert(sizeof(struct dirent) <= __DIRENT_SIZE__,
              "struct dirent size mismatch with Rust");
static_assert(offsetof(struct dirent, d_type) == 0,
              "d_type offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct sockaddr` match the expected values.
 */

static_assert(offsetof(struct sockaddr, sa_family) == 0,
              "sa_family offset mismatch");
static_assert(offsetof(struct sockaddr, sa_data) ==
                  offsetof(struct sockaddr, sa_family) + sizeof(sa_family_t),
              "sa_data offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct sockaddr_in` match the expected values.
 */

static_assert(sizeof(struct sockaddr_in) == 16,
              "struct sockaddr_in size mismatch with Rust");
static_assert(offsetof(struct sockaddr_in, sin_family) == 0,
              "sin_family offset mismatch in struct sockaddr_in");
static_assert(offsetof(struct sockaddr_in, sin_port) == sizeof(sa_family_t),
              "sin_port offset mismatch in struct sockaddr_in");
static_assert(offsetof(struct sockaddr_in, sin_addr) ==
                offsetof(struct sockaddr_in, sin_port) + sizeof(in_port_t),
              "sin_addr offset mismatch in struct sockaddr_in");
static_assert(offsetof(struct sockaddr_in, sin_zero) ==
                offsetof(struct sockaddr_in, sin_addr) + sizeof(in_addr_t),
              "sin_zero offset mismatch in struct sockaddr_in");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct sockaddr_in6` match the expected values.
 */

static_assert(sizeof(struct sockaddr_in6) == 28,
              "struct sockaddr_in6 size mismatch with Rust");
static_assert(offsetof(struct sockaddr_in6, sin6_family) == 0,
              "sin6_family offset mismatch in struct sockaddr_in6");
static_assert(offsetof(struct sockaddr_in6, sin6_port) ==
                sizeof(sa_family_t),
              "sin6_port offset mismatch in struct sockaddr_in6");
static_assert(offsetof(struct sockaddr_in6, sin6_flowinfo) ==
                offsetof(struct sockaddr_in6, sin6_port) + sizeof(in_port_t),
              "sin6_flowinfo offset mismatch in struct sockaddr_in6");
static_assert(offsetof(struct sockaddr_in6, sin6_addr) ==
                offsetof(struct sockaddr_in6, sin6_flowinfo) +
                  sizeof(uint32_t),
              "sin6_addr offset mismatch in struct sockaddr_in6");
static_assert(offsetof(struct sockaddr_in6, sin6_scope_id) ==
                offsetof(struct sockaddr_in6, sin6_addr) +
                  sizeof(struct in6_addr),
              "sin6_scope_id offset mismatch in struct sockaddr_in6");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct sockaddr_un` match the expected values.
 */

static_assert(sizeof(struct sockaddr_un) == 110,
              "struct sockaddr_un size mismatch with Rust");
static_assert(offsetof(struct sockaddr_un, sun_family) == 0,
              "sun_family offset mismatch in struct sockaddr_un");
static_assert(offsetof(struct sockaddr_un, sun_path) == sizeof(sa_family_t),
              "sun_path offset mismatch in struct sockaddr_un");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct ip_mreq` match the expected values.
 */

static_assert(sizeof(struct ip_mreq) == 8,
              "struct ip_mreq size mismatch with Rust");
static_assert(offsetof(struct ip_mreq, imr_multiaddr) == 0,
              "imr_multiaddr offset mismatch in struct ip_mreq");
static_assert(offsetof(struct ip_mreq, imr_interface) ==
                offsetof(struct ip_mreq, imr_multiaddr) +
                  sizeof(struct in_addr),
              "imr_interface offset mismatch in struct ip_mreq");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct ipv6_mreq` match the expected values.
 */

static_assert(sizeof(struct ipv6_mreq) == 20,
              "struct ipv6_mreq size mismatch with Rust");
static_assert(offsetof(struct ipv6_mreq, ipv6mr_multiaddr) == 0,
              "ipv6mr_multiaddr offset mismatch in struct ipv6_mreq");
static_assert(offsetof(struct ipv6_mreq, ipv6mr_interface) ==
                offsetof(struct ipv6_mreq, ipv6mr_multiaddr) +
                  sizeof(struct in6_addr),
              "ipv6mr_interface offset mismatch in struct ipv6_mreq");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct sockaddr_storage` match the expected values.
 */

static_assert(sizeof(struct sockaddr_storage) <= __SOCKADDR_STORAGE_SIZE__,
              "struct sockaddr_storage size mismatch with Rust");
static_assert(offsetof(struct sockaddr_storage, ss_family) == 0,
              "ss_family offset mismatch in struct sockaddr_storage");
static_assert(offsetof(struct sockaddr_storage, ss_data) == 8,
              "ss_data offset mismatch in struct sockaddr_storage");

/**
 * This section contains static assertions to ensure that the field offsets
 * and size of the `struct timeval` match the expected values.
 */

static_assert(sizeof(struct timeval) == 16,
              "struct timeval size mismatch with Rust");
static_assert(offsetof(struct timeval, tv_sec) == 0,
              "tv_sec offset mismatch in struct timeval");
static_assert(offsetof(struct timeval, tv_usec) ==
              offsetof(struct timeval, tv_sec) + sizeof(time_t),
              "tv_usec offset mismatch in struct timeval");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct passwd` match the expected values.
 */

static_assert(offsetof(struct passwd, pw_name) == 0,
              "pw_name offset mismatch");
static_assert(offsetof(struct passwd, pw_uid) ==
                  offsetof(struct passwd, pw_name) + sizeof(FAR char *),
              "pw_uid offset mismatch");
static_assert(offsetof(struct passwd, pw_gid) ==
                  offsetof(struct passwd, pw_uid) + sizeof(uid_t),
              "pw_gid offset mismatch");
static_assert(offsetof(struct passwd, pw_gecos) ==
                  offsetof(struct passwd, pw_gid) + sizeof(gid_t),
              "pw_gecos offset mismatch");
static_assert(offsetof(struct passwd, pw_dir) ==
                  offsetof(struct passwd, pw_gecos) + sizeof(FAR char *),
              "pw_dir offset mismatch");
static_assert(offsetof(struct passwd, pw_shell) ==
                  offsetof(struct passwd, pw_dir) + sizeof(FAR char *),
              "pw_shell offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `Dl_info` struct match the expected values.
 */

static_assert(offsetof(Dl_info, dli_fname) == 0,
              "dli_fname offset mismatch");
static_assert(offsetof(Dl_info, dli_fbase) ==
                  offsetof(Dl_info, dli_fname) + sizeof(FAR char *),
              "dli_fbase offset mismatch");
static_assert(offsetof(Dl_info, dli_sname) ==
                  offsetof(Dl_info, dli_fbase) + sizeof(FAR void *),
              "dli_sname offset mismatch");
static_assert(offsetof(Dl_info, dli_saddr) ==
                  offsetof(Dl_info, dli_sname) + sizeof(FAR char *),
              "dli_saddr offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct lconv` match the expected values.
 */

static_assert(offsetof(struct lconv, decimal_point) == 0,
              "decimal_point offset mismatch");
static_assert(offsetof(struct lconv, thousands_sep) ==
                  offsetof(struct lconv, decimal_point) + sizeof(FAR char *),
              "thousands_sep offset mismatch");
static_assert(offsetof(struct lconv, grouping) ==
                  offsetof(struct lconv, thousands_sep) + sizeof(FAR char *),
              "grouping offset mismatch");
static_assert(offsetof(struct lconv, int_curr_symbol) ==
                  offsetof(struct lconv, grouping) + sizeof(FAR char *),
              "int_curr_symbol offset mismatch");
static_assert(offsetof(struct lconv, currency_symbol) ==
                  offsetof(struct lconv, int_curr_symbol) +
                      sizeof(FAR char *),
              "currency_symbol offset mismatch");
static_assert(offsetof(struct lconv, mon_decimal_point) ==
                  offsetof(struct lconv, currency_symbol) +
                      sizeof(FAR char *),
              "mon_decimal_point offset mismatch");
static_assert(offsetof(struct lconv, mon_thousands_sep) ==
                  offsetof(struct lconv, mon_decimal_point) +
                      sizeof(FAR char *),
              "mon_thousands_sep offset mismatch");
static_assert(offsetof(struct lconv, mon_grouping) ==
                  offsetof(struct lconv, mon_thousands_sep) +
                      sizeof(FAR char *),
              "mon_grouping offset mismatch");
static_assert(offsetof(struct lconv, positive_sign) ==
                  offsetof(struct lconv, mon_grouping) + sizeof(FAR char *),
              "positive_sign offset mismatch");
static_assert(offsetof(struct lconv, negative_sign) ==
                  offsetof(struct lconv, positive_sign) + sizeof(FAR char *),
              "negative_sign offset mismatch");
static_assert(offsetof(struct lconv, int_frac_digits) ==
                  offsetof(struct lconv, negative_sign) + sizeof(FAR char *),
              "int_frac_digits offset mismatch");
static_assert(offsetof(struct lconv, frac_digits) ==
                  offsetof(struct lconv, int_frac_digits) + sizeof(FAR char),
              "frac_digits offset mismatch");
static_assert(offsetof(struct lconv, p_cs_precedes) ==
                  offsetof(struct lconv, frac_digits) + sizeof(FAR char),
              "p_cs_precedes offset mismatch");
static_assert(offsetof(struct lconv, p_sep_by_space) ==
                  offsetof(struct lconv, p_cs_precedes) + sizeof(FAR char),
              "p_sep_by_space offset mismatch");
static_assert(offsetof(struct lconv, n_cs_precedes) ==
                  offsetof(struct lconv, p_sep_by_space) + sizeof(FAR char),
              "n_cs_precedes offset mismatch");
static_assert(offsetof(struct lconv, n_sep_by_space) ==
                  offsetof(struct lconv, n_cs_precedes) + sizeof(FAR char),
              "n_sep_by_space offset mismatch");
static_assert(offsetof(struct lconv, p_sign_posn) ==
                  offsetof(struct lconv, n_sep_by_space) + sizeof(FAR char),
              "p_sign_posn offset mismatch");
static_assert(offsetof(struct lconv, n_sign_posn) ==
                  offsetof(struct lconv, p_sign_posn) + sizeof(FAR char),
              "n_sign_posn offset mismatch");
static_assert(offsetof(struct lconv, int_n_cs_precedes) ==
                  offsetof(struct lconv, n_sign_posn) + sizeof(FAR char),
              "int_n_cs_precedes offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct tm` match the expected values.
 */

static_assert(offsetof(struct tm, tm_sec) == 0, "tm_sec offset mismatch");
static_assert(offsetof(struct tm, tm_min) ==
                  offsetof(struct tm, tm_sec) + sizeof(int),
              "tm_min offset mismatch");
static_assert(offsetof(struct tm, tm_hour) ==
                  offsetof(struct tm, tm_min) + sizeof(int),
              "tm_hour offset mismatch");
static_assert(offsetof(struct tm, tm_mday) ==
                  offsetof(struct tm, tm_hour) + sizeof(int),
              "tm_mday offset mismatch");
static_assert(offsetof(struct tm, tm_mon) ==
                  offsetof(struct tm, tm_mday) + sizeof(int),
              "tm_mon offset mismatch");
static_assert(offsetof(struct tm, tm_year) ==
                  offsetof(struct tm, tm_mon) + sizeof(int),
              "tm_year offset mismatch");
static_assert(offsetof(struct tm, tm_wday) ==
                  offsetof(struct tm, tm_year) + sizeof(int),
              "tm_wday offset mismatch");
static_assert(offsetof(struct tm, tm_yday) ==
                  offsetof(struct tm, tm_wday) + sizeof(int),
              "tm_yday offset mismatch");
static_assert(offsetof(struct tm, tm_isdst) ==
                  offsetof(struct tm, tm_yday) + sizeof(int),
              "tm_isdst offset mismatch");
static_assert(offsetof(struct tm, tm_gmtoff) ==
                  offsetof(struct tm, tm_isdst) + sizeof(long),
              "tm_gmtoff offset mismatch");
static_assert(offsetof(struct tm, tm_zone) ==
                  offsetof(struct tm, tm_gmtoff) + sizeof(long),
              "tm_zone offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct addrinfo` match the expected values.
 */

static_assert(offsetof(struct addrinfo, ai_flags) == 0,
              "ai_flags offset mismatch");
static_assert(offsetof(struct addrinfo, ai_family) ==
                  offsetof(struct addrinfo, ai_flags) + sizeof(int),
              "ai_family offset mismatch");
static_assert(offsetof(struct addrinfo, ai_socktype) ==
                  offsetof(struct addrinfo, ai_family) + sizeof(int),
              "ai_socktype offset mismatch");
static_assert(offsetof(struct addrinfo, ai_protocol) ==
                  offsetof(struct addrinfo, ai_socktype) + sizeof(int),
              "ai_protocol offset mismatch");
static_assert(offsetof(struct addrinfo, ai_addrlen) ==
                  offsetof(struct addrinfo, ai_protocol) + sizeof(int),
              "ai_addrlen offset mismatch");
static_assert(offsetof(struct addrinfo, ai_addr) ==
                  offsetof(struct addrinfo, ai_addrlen) + sizeof(void *),
              "ai_addr offset mismatch");
static_assert(offsetof(struct addrinfo, ai_canonname) ==
                  offsetof(struct addrinfo, ai_addr) +
                      sizeof(struct sockaddr *),
              "ai_canonname offset mismatch");
static_assert(offsetof(struct addrinfo, ai_next) ==
                  offsetof(struct addrinfo, ai_canonname) + sizeof(char *),
              "ai_next offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct statvfs` match the expected values.
 */

static_assert(offsetof(struct statvfs, f_bsize) == 0,
              "f_bsize offset mismatch");
static_assert(offsetof(struct statvfs, f_frsize) ==
                  offsetof(struct statvfs, f_bsize) + sizeof(unsigned long),
              "f_frsize offset mismatch");
static_assert(offsetof(struct statvfs, f_blocks) ==
                  offsetof(struct statvfs, f_frsize) + sizeof(unsigned long),
              "f_blocks offset mismatch");
static_assert(offsetof(struct statvfs, f_bfree) ==
                  offsetof(struct statvfs, f_blocks) + sizeof(fsblkcnt_t),
              "f_bfree offset mismatch");
static_assert(offsetof(struct statvfs, f_bavail) ==
                  offsetof(struct statvfs, f_bfree) + sizeof(fsblkcnt_t),
              "f_bavail offset mismatch");
static_assert(offsetof(struct statvfs, f_files) ==
                  offsetof(struct statvfs, f_bavail) + sizeof(fsblkcnt_t),
              "f_files offset mismatch");
static_assert(offsetof(struct statvfs, f_ffree) ==
                  offsetof(struct statvfs, f_files) + sizeof(fsfilcnt_t),
              "f_ffree offset mismatch");
static_assert(offsetof(struct statvfs, f_favail) ==
                  offsetof(struct statvfs, f_ffree) + sizeof(fsfilcnt_t),
              "f_favail offset mismatch");
static_assert(offsetof(struct statvfs, f_fsid) ==
                  offsetof(struct statvfs, f_favail) + sizeof(fsfilcnt_t),
              "f_fsid offset mismatch");
static_assert(offsetof(struct statvfs, f_flag) ==
                  offsetof(struct statvfs, f_fsid) + sizeof(unsigned long),
              "f_flag offset mismatch");
static_assert(offsetof(struct statvfs, f_namemax) ==
                  offsetof(struct statvfs, f_flag) + sizeof(unsigned long),
              "f_namemax offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct sigaction` match the expected values.
 */

static_assert(offsetof(struct sigaction, sa_u) == 0, "sa_u offset mismatch");
static_assert(offsetof(struct sigaction, sa_mask) ==
                  offsetof(struct sigaction, sa_u) + sizeof(void *),
              "sa_mask offset mismatch");
static_assert(offsetof(struct sigaction, sa_flags) ==
                  offsetof(struct sigaction, sa_mask) + sizeof(sigset_t),
              "sa_flags offset mismatch");
static_assert(offsetof(struct sigaction, sa_user) ==
                  offsetof(struct sigaction, sa_flags) + sizeof(void *),
              "sa_user offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct termios` match the expected values.
 */

static_assert(offsetof(struct termios, c_iflag) == 0,
              "c_iflag offset mismatch");
static_assert(offsetof(struct termios, c_oflag) ==
                  offsetof(struct termios, c_iflag) + sizeof(tcflag_t),
              "c_oflag offset mismatch");
static_assert(offsetof(struct termios, c_cflag) ==
                  offsetof(struct termios, c_oflag) + sizeof(tcflag_t),
              "c_cflag offset mismatch");
static_assert(offsetof(struct termios, c_lflag) ==
                  offsetof(struct termios, c_cflag) + sizeof(tcflag_t),
              "c_lflag offset mismatch");
static_assert(offsetof(struct termios, c_cc) ==
                  offsetof(struct termios, c_lflag) + sizeof(tcflag_t),
              "c_cc offset mismatch");
static_assert(offsetof(struct termios, c_speed) ==
                  offsetof(struct termios, c_cc) +
                      ALIGNUP_TO(sizeof(cc_t[NCCS]), sizeof(long)),
              "c_speed offset mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from stdlib.h match the expected values.
 */

static_assert(EXIT_SUCCESS == 0, "EXIT_SUCCESS mismatch");
static_assert(EXIT_FAILURE == 1, "EXIT_FAILURE mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from signal.h match the expected values.
 */

static_assert(SIGPIPE == 13, "SIGPIPE mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from pthread.h match the expected values.
 */

static_assert(PTHREAD_MUTEX_NORMAL == 0, "PTHREAD_MUTEX_NORMAL mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from time.h match the expected values.
 */

static_assert(CLOCK_REALTIME == 0, "CLOCK_REALTIME mismatch");
static_assert(CLOCK_MONOTONIC == 1, "CLOCK_MONOTONIC mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from errno.h match the expected values.
 */

static_assert(EPERM == 1, "EPERM mismatch");
static_assert(ENOENT == 2, "ENOENT mismatch");
static_assert(ESRCH == 3, "ESRCH mismatch");
static_assert(EINTR == 4, "EINTR mismatch");
static_assert(EIO == 5, "EIO mismatch");
static_assert(ENXIO == 6, "ENXIO mismatch");
static_assert(E2BIG == 7, "E2BIG mismatch");
static_assert(ENOEXEC == 8, "ENOEXEC mismatch");
static_assert(EBADF == 9, "EBADF mismatch");
static_assert(ECHILD == 10, "ECHILD mismatch");
static_assert(EAGAIN == 11, "EAGAIN mismatch");
static_assert(ENOMEM == 12, "ENOMEM mismatch");
static_assert(EACCES == 13, "EACCES mismatch");
static_assert(EFAULT == 14, "EFAULT mismatch");
static_assert(ENOTBLK == 15, "ENOTBLK mismatch");
static_assert(EBUSY == 16, "EBUSY mismatch");
static_assert(EEXIST == 17, "EEXIST mismatch");
static_assert(EXDEV == 18, "EXDEV mismatch");
static_assert(ENODEV == 19, "ENODEV mismatch");
static_assert(ENOTDIR == 20, "ENOTDIR mismatch");
static_assert(EISDIR == 21, "EISDIR mismatch");
static_assert(EINVAL == 22, "EINVAL mismatch");
static_assert(ENFILE == 23, "ENFILE mismatch");
static_assert(EMFILE == 24, "EMFILE mismatch");
static_assert(ENOTTY == 25, "ENOTTY mismatch");
static_assert(ETXTBSY == 26, "ETXTBSY mismatch");
static_assert(EFBIG == 27, "EFBIG mismatch");
static_assert(ENOSPC == 28, "ENOSPC mismatch");
static_assert(ESPIPE == 29, "ESPIPE mismatch");
static_assert(EROFS == 30, "EROFS mismatch");
static_assert(EMLINK == 31, "EMLINK mismatch");
static_assert(EPIPE == 32, "EPIPE mismatch");
static_assert(EDOM == 33, "EDOM mismatch");
static_assert(ERANGE == 34, "ERANGE mismatch");
static_assert(EDEADLK == 35, "EDEADLK mismatch");
static_assert(ENAMETOOLONG == 36, "ENAMETOOLONG mismatch");
static_assert(ENOLCK == 37, "ENOLCK mismatch");
static_assert(ENOSYS == 38, "ENOSYS mismatch");
static_assert(ENOTEMPTY == 39, "ENOTEMPTY mismatch");
static_assert(ELOOP == 40, "ELOOP mismatch");
static_assert(EWOULDBLOCK == EAGAIN, "EWOULDBLOCK mismatch");
static_assert(ENOMSG == 42, "ENOMSG mismatch");
static_assert(EIDRM == 43, "EIDRM mismatch");
static_assert(ECHRNG == 44, "ECHRNG mismatch");
static_assert(EL2NSYNC == 45, "EL2NSYNC mismatch");
static_assert(EL3HLT == 46, "EL3HLT mismatch");
static_assert(EL3RST == 47, "EL3RST mismatch");
static_assert(ELNRNG == 48, "ELNRNG mismatch");
static_assert(EUNATCH == 49, "EUNATCH mismatch");
static_assert(ENOCSI == 50, "ENOCSI mismatch");
static_assert(EL2HLT == 51, "EL2HLT mismatch");
static_assert(EBADE == 52, "EBADE mismatch");
static_assert(EBADR == 53, "EBADR mismatch");
static_assert(EXFULL == 54, "EXFULL mismatch");
static_assert(ENOANO == 55, "ENOANO mismatch");
static_assert(EBADRQC == 56, "EBADRQC mismatch");
static_assert(EBADSLT == 57, "EBADSLT mismatch");
static_assert(EDEADLOCK == EDEADLK, "EDEADLOCK mismatch");
static_assert(EBFONT == 59, "EBFONT mismatch");
static_assert(ENOSTR == 60, "ENOSTR mismatch");
static_assert(ENODATA == 61, "ENODATA mismatch");
static_assert(ETIME == 62, "ETIME mismatch");
static_assert(ENOSR == 63, "ENOSR mismatch");
static_assert(ENONET == 64, "ENONET mismatch");
static_assert(ENOPKG == 65, "ENOPKG mismatch");
static_assert(EREMOTE == 66, "EREMOTE mismatch");
static_assert(ENOLINK == 67, "ENOLINK mismatch");
static_assert(EADV == 68, "EADV mismatch");
static_assert(ESRMNT == 69, "ESRMNT mismatch");
static_assert(ECOMM == 70, "ECOMM mismatch");
static_assert(EPROTO == 71, "EPROTO mismatch");
static_assert(EMULTIHOP == 72, "EMULTIHOP mismatch");
static_assert(EDOTDOT == 73, "EDOTDOT mismatch");
static_assert(EBADMSG == 74, "EBADMSG mismatch");
static_assert(EOVERFLOW == 75, "EOVERFLOW mismatch");
static_assert(ENOTUNIQ == 76, "ENOTUNIQ mismatch");
static_assert(EBADFD == 77, "EBADFD mismatch");
static_assert(EREMCHG == 78, "EREMCHG mismatch");
static_assert(ELIBACC == 79, "ELIBACC mismatch");
static_assert(ELIBBAD == 80, "ELIBBAD mismatch");
static_assert(ELIBSCN == 81, "ELIBSCN mismatch");
static_assert(ELIBMAX == 82, "ELIBMAX mismatch");
static_assert(ELIBEXEC == 83, "ELIBEXEC mismatch");
static_assert(EILSEQ == 84, "EILSEQ mismatch");
static_assert(ERESTART == 85, "ERESTART mismatch");
static_assert(ESTRPIPE == 86, "ESTRPIPE mismatch");
static_assert(EUSERS == 87, "EUSERS mismatch");
static_assert(ENOTSOCK == 88, "ENOTSOCK mismatch");
static_assert(EDESTADDRREQ == 89, "EDESTADDRREQ mismatch");
static_assert(EMSGSIZE == 90, "EMSGSIZE mismatch");
static_assert(EPROTOTYPE == 91, "EPROTOTYPE mismatch");
static_assert(ENOPROTOOPT == 92, "ENOPROTOOPT mismatch");
static_assert(EPROTONOSUPPORT == 93, "EPROTONOSUPPORT mismatch");
static_assert(ESOCKTNOSUPPORT == 94, "ESOCKTNOSUPPORT mismatch");
static_assert(EOPNOTSUPP == 95, "EOPNOTSUPP mismatch");
static_assert(EPFNOSUPPORT == 96, "EPFNOSUPPORT mismatch");
static_assert(EAFNOSUPPORT == 97, "EAFNOSUPPORT mismatch");
static_assert(EADDRINUSE == 98, "EADDRINUSE mismatch");
static_assert(EADDRNOTAVAIL == 99, "EADDRNOTAVAIL mismatch");
static_assert(ENETDOWN == 100, "ENETDOWN mismatch");
static_assert(ENETUNREACH == 101, "ENETUNREACH mismatch");
static_assert(ENETRESET == 102, "ENETRESET mismatch");
static_assert(ECONNABORTED == 103, "ECONNABORTED mismatch");
static_assert(ECONNRESET == 104, "ECONNRESET mismatch");
static_assert(ENOBUFS == 105, "ENOBUFS mismatch");
static_assert(EISCONN == 106, "EISCONN mismatch");
static_assert(ENOTCONN == 107, "ENOTCONN mismatch");
static_assert(ESHUTDOWN == 108, "ESHUTDOWN mismatch");
static_assert(ETOOMANYREFS == 109, "ETOOMANYREFS mismatch");
static_assert(ETIMEDOUT == 110, "ETIMEDOUT mismatch");
static_assert(ECONNREFUSED == 111, "ECONNREFUSED mismatch");
static_assert(EHOSTDOWN == 112, "EHOSTDOWN mismatch");
static_assert(EHOSTUNREACH == 113, "EHOSTUNREACH mismatch");
static_assert(EALREADY == 114, "EALREADY mismatch");
static_assert(EINPROGRESS == 115, "EINPROGRESS mismatch");
static_assert(ESTALE == 116, "ESTALE mismatch");
static_assert(EUCLEAN == 117, "EUCLEAN mismatch");
static_assert(ENOTNAM == 118, "ENOTNAM mismatch");
static_assert(ENAVAIL == 119, "ENAVAIL mismatch");
static_assert(EISNAM == 120, "EISNAM mismatch");
static_assert(EREMOTEIO == 121, "EREMOTEIO mismatch");
static_assert(EDQUOT == 122, "EDQUOT mismatch");
static_assert(ENOMEDIUM == 123, "ENOMEDIUM mismatch");
static_assert(EMEDIUMTYPE == 124, "EMEDIUMTYPE mismatch");
static_assert(ECANCELED == 125, "ECANCELED mismatch");
static_assert(ENOKEY == 126, "ENOKEY mismatch");
static_assert(EKEYEXPIRED == 127, "EKEYEXPIRED mismatch");
static_assert(EKEYREVOKED == 128, "EKEYREVOKED mismatch");
static_assert(EKEYREJECTED == 129, "EKEYREJECTED mismatch");
static_assert(EOWNERDEAD == 130, "EOWNERDEAD mismatch");
static_assert(ENOTRECOVERABLE == 131, "ENOTRECOVERABLE mismatch");
static_assert(ERFKILL == 132, "ERFKILL mismatch");
static_assert(EHWPOISON == 133, "EHWPOISON mismatch");
static_assert(ELBIN == 134, "ELBIN mismatch");
static_assert(EFTYPE == 135, "EFTYPE mismatch");
static_assert(ENMFILE == 136, "ENMFILE mismatch");
static_assert(EPROCLIM == 137, "EPROCLIM mismatch");
static_assert(ENOTSUP == 138, "ENOTSUP mismatch");
static_assert(ENOSHARE == 139, "ENOSHARE mismatch");
static_assert(ECASECLASH == 140, "ECASECLASH mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from unistd.h match the expected values.
 */

static_assert(STDIN_FILENO == 0, "STDIN_FILENO mismatch");
static_assert(STDOUT_FILENO == 1, "STDOUT_FILENO mismatch");
static_assert(STDERR_FILENO == 2, "STDERR_FILENO mismatch");
static_assert(_SC_PAGESIZE == 0x36, "_SC_PAGESIZE mismatch");
static_assert(_SC_THREAD_STACK_MIN == 0x58, "_SC_THREAD_STACK_MIN mismatch");
static_assert(_SC_GETPW_R_SIZE_MAX == 0x25, "_SC_GETPW_R_SIZE_MAX mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from dlfcn.h match the expected values.
 */

/* static_assert(RTLD_DEFAULT == 0, "RTLD_DEFAULT mismatch"); */

/**
 * This section contains static assertions to ensure that the macro
 * definitions from fcntl.h match the expected values.
 */

static_assert(FIOCLEX == 0x30b, "FIOCLEX mismatch");
static_assert(F_SETFL == 0x9, "F_SETFL mismatch");
static_assert(F_DUPFD_CLOEXEC == 0x12, "F_DUPFD_CLOEXEC mismatch");
static_assert(F_GETFD == 0x1, "F_GETFD mismatch");
static_assert(F_GETFL == 0x2, "F_GETFL mismatch");
static_assert(O_RDONLY == 0x1, "O_RDONLY mismatch");
static_assert(O_WRONLY == 0x2, "O_WRONLY mismatch");
static_assert(O_RDWR == 0x3, "O_RDWR mismatch");
static_assert(O_CREAT == 0x4, "O_CREAT mismatch");
static_assert(O_EXCL == 0x8, "O_EXCL mismatch");
static_assert(O_NOCTTY == 0x0, "O_NOCTTY mismatch");
static_assert(O_TRUNC == 0x20, "O_TRUNC mismatch");
static_assert(O_APPEND == 0x10, "O_APPEND mismatch");
static_assert(O_NONBLOCK == 0x40, "O_NONBLOCK mismatch");
static_assert(O_DSYNC == 0x80, "O_DSYNC mismatch");
static_assert(O_DIRECT == 0x200, "O_DIRECT mismatch");
static_assert(O_LARGEFILE == 0x2000, "O_LARGEFILE mismatch");
static_assert(O_DIRECTORY == 0x800, "O_DIRECTORY mismatch");
static_assert(O_NOFOLLOW == 0x1000, "O_NOFOLLOW mismatch");
static_assert(O_NOATIME == 0x40000, "O_NOATIME mismatch");
static_assert(O_CLOEXEC == 0x400, "O_CLOEXEC mismatch");
static_assert(O_ACCMODE == 0x0003, "O_ACCMODE mismatch");
static_assert(AT_FDCWD == -100, "AT_FDCWD mismatch");
static_assert(AT_REMOVEDIR == 0x200, "AT_REMOVEDIR mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from sys/types.h match the expected values.
 */

static_assert(SEEK_SET == 0, "SEEK_SET mismatch");
static_assert(SEEK_CUR == 1, "SEEK_CUR mismatch");
static_assert(SEEK_END == 2, "SEEK_END mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from sys/stat.h match the expected values.
 */

static_assert(S_IFDIR == 0x4000, "S_IFDIR mismatch");
static_assert(S_IFLNK == 0xa000, "S_IFLNK mismatch");
static_assert(S_IFREG == 0x8000, "S_IFREG mismatch");
static_assert(S_IFMT == 0xf000, "S_IFMT mismatch");
static_assert(S_IFIFO == 0x1000, "S_IFIFO mismatch");
static_assert(S_IFSOCK == 0xc000, "S_IFSOCK mismatch");
static_assert(S_IFBLK == 0x6000, "S_IFBLK mismatch");
static_assert(S_IFCHR == 0x2000, "S_IFCHR mismatch");
static_assert(S_IRUSR == 0x100, "S_IRUSR mismatch");
static_assert(S_IWUSR == 0x80, "S_IWUSR mismatch");
static_assert(S_IXUSR == 0x40, "S_IXUSR mismatch");
static_assert(S_IRGRP == 0x20, "S_IRGRP mismatch");
static_assert(S_IWGRP == 0x10, "S_IWGRP mismatch");
static_assert(S_IXGRP == 0x8, "S_IXGRP mismatch");
static_assert(S_IROTH == 0x004, "S_IROTH mismatch");
static_assert(S_IWOTH == 0x002, "S_IWOTH mismatch");
static_assert(S_IXOTH == 0x001, "S_IXOTH mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from sys/poll.h match the expected values.
 */

static_assert(POLLIN == 0x01, "POLLIN mismatch");
static_assert(POLLOUT == 0x04, "POLLOUT mismatch");
static_assert(POLLHUP == 0x10, "POLLHUP mismatch");
static_assert(POLLERR == 0x08, "POLLERR mismatch");
static_assert(POLLNVAL == 0x20, "POLLNVAL mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from sys/socket.h match the expected values.
 */

static_assert(AF_UNIX == 1, "AF_UNIX mismatch");
static_assert(SOCK_DGRAM == 2, "SOCK_DGRAM mismatch");
static_assert(SOCK_STREAM == 1, "SOCK_STREAM mismatch");
static_assert(AF_INET == 2, "AF_INET mismatch");
static_assert(AF_INET6 == 10, "AF_INET6 mismatch");
static_assert(MSG_PEEK == 0x02, "MSG_PEEK mismatch");
static_assert(SOL_SOCKET == 1, "SOL_SOCKET mismatch");
static_assert(SHUT_WR == 2, "SHUT_WR mismatch");
static_assert(SHUT_RD == 1, "SHUT_RD mismatch");
static_assert(SHUT_RDWR == 3, "SHUT_RDWR mismatch");
static_assert(SO_ERROR == 4, "SO_ERROR mismatch");
static_assert(SO_REUSEADDR == 11, "SO_REUSEADDR mismatch");
static_assert(SOMAXCONN == 8, "SOMAXCONN mismatch");
static_assert(SO_LINGER == 6, "SO_LINGER mismatch");
static_assert(SO_RCVTIMEO == 0xa, "SO_RCVTIMEO mismatch");
static_assert(SO_SNDTIMEO == 0xe, "SO_SNDTIMEO mismatch");
static_assert(SO_BROADCAST == 1, "SO_BROADCAST mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from netinet/tcp.h match the expected values.
 */

static_assert(TCP_NODELAY == 0x10, "TCP_NODELAY mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from netinet/in.h match the expected values.
 */

static_assert(IP_TTL == 0x1e, "IP_TTL mismatch");
static_assert(IPV6_V6ONLY == 0x17, "IPV6_V6ONLY mismatch");
static_assert(IPV6_JOIN_GROUP == 0x11, "IPV6_JOIN_GROUP mismatch");
static_assert(IPV6_LEAVE_GROUP == 0x12, "IPV6_LEAVE_GROUP mismatch");
static_assert(IP_MULTICAST_LOOP == 0x13, "IP_MULTICAST_LOOP mismatch");
static_assert(IPV6_MULTICAST_LOOP == 0x15, "IPV6_MULTICAST_LOOP mismatch");
static_assert(IP_MULTICAST_TTL == 0x12, "IP_MULTICAST_TTL mismatch");
static_assert(IP_ADD_MEMBERSHIP == 0x14, "IP_ADD_MEMBERSHIP mismatch");
static_assert(IP_DROP_MEMBERSHIP == 0x15, "IP_DROP_MEMBERSHIP mismatch");

/**
 * This section contains static assertions to ensure that the macro
 * definitions from nuttx/fs/ioctl.h match the expected values.
 */

static_assert(FIONBIO == 0x30a, "FIONBIO mismatch");

/****************************************************************************
 * Public Functions
 ****************************************************************************/
