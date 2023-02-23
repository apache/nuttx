/****************************************************************************
 * arch/xtensa/src/esp32s3/rom/esp32s3_libc_stubs.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ROM_ESP32S3_LIBC_STUBS_H
#define __ARCH_XTENSA_SRC_ESP32S3_ROM_ESP32S3_LIBC_STUBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>

#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <nuttx/mutex.h>

#define _lock_t int

/* Forward declaration */

struct _reent;

/**
 * @brief ESP32-S3 ROM code contains implementations of some of C
 * library functions.
 * Whenever a function in ROM needs to use a syscall, it calls a
 * pointer to the corresponding syscall implementation defined in
 * the following struct.
 *
 * The table itself, by default, is not allocated in RAM. There are
 * two pointers, `syscall_table_ptr_pro` and `syscall_table_ptr_app`,
 * which can be set to point to the locations of syscall tables of
 * CPU 0 (aka PRO CPU) and CPU 1 (aka APP CPU). Location of these
 * pointers in .bss segment of ROM code is defined in linker script.
 *
 * So, before using any of the C library functions (except for pure
 * functions and memcpy/memset functions), application must allocate
 * syscall table structure for each CPU being used, and populate it
 * with pointers to actual implementations of corresponding syscalls.
 */

struct syscall_stub_table
{
    struct _reent *(* __getreent)(void);
    void *(* _malloc_r)(struct _reent *r, size_t);
    void (* _free_r)(struct _reent *r, void *);
    void *(* _realloc_r)(struct _reent *r, void *, size_t);
    void *(* _calloc_r)(struct _reent *r, size_t, size_t);
    void (* _abort)(void);
    int (* _system_r)(struct _reent *r, const char *);
    int (* _rename_r)(struct _reent *r, const char *, const char *);
    clock_t (* _times_r)(struct _reent *r, struct tms *);
    int (* _gettimeofday_r) (struct _reent *r, struct timeval *, void *);
    void (* _raise_r)(struct _reent *r);
    int (* _unlink_r)(struct _reent *r, const char *);
    int (* _link_r)(struct _reent *r, const char *, const char *);
    int (* _stat_r)(struct _reent *r, const char *, struct stat *);
    int (* _fstat_r)(struct _reent *r, int, struct stat *);
    void *(* _sbrk_r)(struct _reent *r, ptrdiff_t);
    int (* _getpid_r)(struct _reent *r);
    int (* _kill_r)(struct _reent *r, int, int);
    void (* _exit_r)(struct _reent *r, int);
    int (* _close_r)(struct _reent *r, int);
    int (* _open_r)(struct _reent *r, const char *, int, int);
    int (* _write_r)(struct _reent *r, int, const void *, int);
    int (* _lseek_r)(struct _reent *r, int, int, int);
    int (* _read_r)(struct _reent *r, int, void *, int);
    void (* _retarget_lock_init)(_lock_t *lock);
    void (* _retarget_lock_init_recursive)(_lock_t *lock);
    void (* _retarget_lock_close)(_lock_t lock);
    void (* _retarget_lock_close_recursive)(_lock_t lock);
    void (* _retarget_lock_acquire)(_lock_t lock);
    void (* _retarget_lock_acquire_recursive)(_lock_t lock);
    int (* _retarget_lock_try_acquire)(_lock_t lock);
    int (* _retarget_lock_try_acquire_recursive)(_lock_t lock);
    void (* _retarget_lock_release)(_lock_t lock);
    void (* _retarget_lock_release_recursive)(_lock_t lock);
    int (*_printf_float)(struct _reent *data, void *pdata, FILE *fp,
                         int (*pfunc) (struct _reent *, FILE *,
                         const char *, size_t len), va_list * ap);
    int (*_scanf_float) (struct _reent *rptr, void *pdata, FILE *fp,
                         va_list *ap);
    void (* __assert_func) (const char *file, int line,
                            const char *func, const char *failedexpr)
                            __attribute__((noreturn));
    void (* __sinit) (struct _reent *r);
    void (* _cleanup_r) (struct _reent *r);
};

extern const struct syscall_stub_table *syscall_table_ptr;
#define syscall_table_ptr_pro syscall_table_ptr
#define syscall_table_ptr_app syscall_table_ptr

/****************************************************************************
 * Name: esp_setup_syscall_table
 *
 * Description:
 *   Configure the syscall table used by the ROM code for calling C library
 *   functions.
 *   ROM code from Espressif's chips contains implementations of some of C
 *   library functions. Whenever a function in ROM needs to use a syscall,
 *   it calls a pointer to the corresponding syscall implementation defined
 *   in the syscall_stub_table struct.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_setup_syscall_table(void);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ROM_ESP32S3_LIBC_STUBS_H */
