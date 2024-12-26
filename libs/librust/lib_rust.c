/****************************************************************************
 * libs/librust/lib_rust.c
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
 * of the `struct sockaddr` match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
 */

static_assert(offsetof(struct sockaddr, sa_family) == 0,
              "sa_family offset mismatch");
static_assert(offsetof(struct sockaddr, sa_data) ==
                  offsetof(struct sockaddr, sa_family) + sizeof(sa_family_t),
              "sa_data offset mismatch");

/**
 * This section contains static assertions to ensure that the field offsets
 * of the `struct passwd` match the expected values. These checks are crucial
 * for compatibility with Rust, ensuring that the memory layout is consistent
 * across both languages.
 */

static_assert(offsetof(struct passwd, pw_name) == 0,
              "pw_name offset mismatch");
static_assert(offsetof(struct passwd, pw_passwd) ==
                  offsetof(struct passwd, pw_name) + sizeof(FAR char *),
              "pw_passwd offset mismatch");
static_assert(offsetof(struct passwd, pw_uid) ==
                  offsetof(struct passwd, pw_passwd) + sizeof(FAR char *),
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
 * of the `Dl_info` struct match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
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
 * of the `struct lconv` match the expected values. These checks are crucial
 * for compatibility with Rust, ensuring that the memory layout is consistent
 * across both languages.
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
 * of the `struct tm` match the expected values. These checks are crucial
 * for compatibility with Rust, ensuring that the memory layout is consistent
 * across both languages.
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
 * of the `struct addrinfo` match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
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
 * of the `struct statvfs` match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
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
 * of the `struct sigaction` match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
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
 * of the `struct termios` match the expected values. These checks are
 * crucial for compatibility with Rust, ensuring that the memory layout is
 * consistent across both languages.
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/
