/****************************************************************************
 * libs/libbuiltin/compiler-rt/coverage.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INSTR_PROF_RAW_VERSION 8
#define INSTR_PROF_PROFILE_RUNTIME_VAR __llvm_profile_runtime

/* Magic number to detect file format and endianness.
 * Use 255 at one end, since no UTF-8 file can use that character.  Avoid 0,
 * so that utilities, like strings, don't grab it as a string.  129 is also
 * invalid UTF-8, and high enough to be interesting.
 * Use "lprofr" in the centre to stand for "LLVM Profile Raw", or "lprofR"
 * for 32-bit platforms.
 */

#define INSTR_PROF_RAW_MAGIC_64 \
  (uint64_t)255 << 56 | (uint64_t)'l' << 48 | (uint64_t)'p' << 40 | \
      (uint64_t)'r' << 32 | (uint64_t)'o' << 24 | (uint64_t)'f' << 16 | \
      (uint64_t)'r' << 8 | (uint64_t)129

#define INSTR_PROF_RAW_MAGIC_32 \
  (uint64_t)255 << 56 | (uint64_t)'l' << 48 | (uint64_t)'p' << 40 | \
      (uint64_t)'r' << 32 | (uint64_t)'o' << 24 | (uint64_t)'f' << 16 | \
      (uint64_t)'R' << 8 | (uint64_t)129

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum value_kind
{
  IPVK_INDIRECT_CALL_TARGET = 0,
  IPVK_MEM_OP_SIZE = 1,
  IPVK_FIRST = IPVK_INDIRECT_CALL_TARGET,
  IPVK_LAST = IPVK_MEM_OP_SIZE,
};

typedef struct aligned_data(8) __llvm_profile_data
{
  const uint64_t name_ref;
  const uint64_t func_hash;
  const intptr_t counter_ptr;
  const intptr_t func_ptr;
  intptr_t       values;
  const uint32_t num_counters;
  const uint16_t num_value_sites[IPVK_LAST + 1];
}__llvm_profile_data;

typedef struct __llvm_profile_header
{
  uint64_t magic;
  uint64_t version;
  uint64_t binary_ids_size;
  uint64_t data_size;
  uint64_t padding_bytes_before_counters;
  uint64_t counters_size;
  uint64_t padding_bytes_after_counters;
  uint64_t names_size;
  uint64_t counters_delta;
  uint64_t names_delta;
  enum value_kind value_kind_last;
}__llvm_profile_header;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Record where the data is in memory. Within each of the types of data,
 * it's stored consecutively.
 */

extern char __start__llvm_prf_names[];
extern char __end__llvm_prf_names[];
extern char __start__llvm_prf_data[];
extern char __end__llvm_prf_data[];
extern char __start__llvm_prf_cnts[];
extern char __end__llvm_prf_cnts[];

int INSTR_PROF_PROFILE_RUNTIME_VAR;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static size_t __llvm_profile_counter_entry_size(void)
{
  return sizeof(uint64_t);
}

static uint64_t __llvm_profile_get_magic(void)
{
  return sizeof(void *) == sizeof(uint64_t) ? (INSTR_PROF_RAW_MAGIC_64)
                                            : (INSTR_PROF_RAW_MAGIC_32);
}

static uint64_t __llvm_profile_get_version(void)
{
  return INSTR_PROF_RAW_VERSION;
}

static uint64_t __llvm_profile_get_num_counters(const char *begin,
                                                const char *end)
{
  return (((intptr_t)end + __llvm_profile_counter_entry_size() - 1) -
          (intptr_t)begin) / __llvm_profile_counter_entry_size();
}

static int __llvm_write_binary_ids(void)
{
  return 0;
}

static const __llvm_profile_data *__llvm_profile_begin_data(void)
{
  return (const __llvm_profile_data *)__start__llvm_prf_data;
}

static const __llvm_profile_data *__llvm_profile_end_data(void)
{
  return (const __llvm_profile_data *)__end__llvm_prf_data;
}

static const char *__llvm_profile_begin_names(void)
{
  return (const char *)__start__llvm_prf_names;
}

static const char *__llvm_profile_end_names(void)
{
  return (const char *)__end__llvm_prf_names;
}

static char *__llvm_profile_begin_counters(void)
{
  return (char *)__start__llvm_prf_cnts;
}

static char *__llvm_profile_end_counters(void)
{
  return (char *)__end__llvm_prf_cnts;
}

static uint64_t __llvm_profile_get_num_padding_bytes(uint64_t size)
{
  return 7 & (sizeof(uint64_t) - size % sizeof(uint64_t));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint64_t __llvm_profile_get_num_data(const __llvm_profile_data *begin,
                                     const __llvm_profile_data *end)
{
  return (((intptr_t)end + sizeof(__llvm_profile_data) - 1) -
          (intptr_t)begin) / sizeof(__llvm_profile_data);
}

/* Given a pointer to the __llvm_profile_data for the function, record the
 * bounds of the profile data and profile count sections.
 * This function is called several time by the __llvm_profile_init function
 * at program start.
 *
 * If this function is called we register __llvm_profile_dump() with
 * atexit to write out the profile information to file.
 */

void __llvm_profile_register_function(void *data_)
{
  (void)data_;
}

void __llvm_profile_register_names_function(void *names_start,
                                       uint64_t names_size)
{
  (void)names_start;
  (void)names_size;
}

/* Called by an atexit handler. Writes a file called default.profraw
 * containing the profile data. This needs to be merged by
 * llvm-prof. See the clang profiling documentation for details.
 */

size_t __llvm_profile_dump(FAR struct lib_outstream_s *stream)
{
  const char c = '\0';
  size_t size = 0;

  /* Calculate size of sections. */

  const __llvm_profile_data *data_begin = __llvm_profile_begin_data();
  const __llvm_profile_data *data_end = __llvm_profile_end_data();
  const uint64_t num_data =
    __llvm_profile_get_num_data(data_begin, data_end);

  const char *counters_begin = __llvm_profile_begin_counters();
  const char *counters_end = __llvm_profile_end_counters();
  const uint64_t num_counters =
    __llvm_profile_get_num_counters(counters_begin, counters_end);

  const char *names_begin = __llvm_profile_begin_names();
  const char *names_end = __llvm_profile_end_names();
  const uint64_t names_size = (names_end - names_begin) * sizeof(char);

  uint64_t padding_bytes_after_names =
    __llvm_profile_get_num_padding_bytes(names_size);

  __llvm_profile_header hdr;
  hdr.magic = __llvm_profile_get_magic();
  hdr.version = __llvm_profile_get_version();
  hdr.binary_ids_size = __llvm_write_binary_ids();
  hdr.data_size = num_data;
  hdr.padding_bytes_before_counters = 0;
  hdr.counters_size = num_counters;
  hdr.padding_bytes_after_counters = 0;
  hdr.names_size = names_size;
  hdr.counters_delta = (uintptr_t)counters_begin - (uintptr_t)data_begin;
  hdr.names_delta = (uintptr_t)names_begin;
  hdr.value_kind_last = IPVK_LAST;

  size += sizeof(hdr);
  if (sizeof(hdr) != stream->puts(stream, &hdr, sizeof(hdr)))
    {
      goto exit;
    }

  size += sizeof(__llvm_profile_data) * num_data;
  if (sizeof(__llvm_profile_data) * num_data !=
      stream->puts(stream, data_begin,
                   sizeof(__llvm_profile_data) * num_data))
    {
      goto exit;
    }

  size += sizeof(uint64_t) * num_counters;
  if (sizeof(uint64_t) * num_counters !=
      stream->puts(stream, counters_begin,
                   sizeof(uint64_t) * num_counters))
    {
      goto exit;
    }

  size += names_size;
  if (names_size != stream->puts(stream, names_begin, names_size))
    {
      goto exit;
    }

  for (; padding_bytes_after_names != 0; --padding_bytes_after_names)
    {
      size += 1;
      if (1 != stream->puts(stream, &c, 1))
        {
          break;
        }
    }

exit:

  return size;
}

void __gcov_dump(void)
{
  struct lib_rawoutstream_s stream;
  FAR char *path;
  int fd;

  path = getenv("GCOV_PREFIX");
  if (!path)
    {
      return;
    }

  fd = _NX_OPEN(path, O_WRONLY | O_CREAT);
  if (fd < 0)
    {
      _NX_SETERRNO(fd);
      return;
    }

  lib_rawoutstream(&stream, fd);

  __llvm_profile_dump(&stream.common);

  _NX_CLOSE(fd);
}

size_t __gcov_dump_to_memory(FAR void *ptr, size_t size)
{
  struct lib_memoutstream_s stream;

  lib_memoutstream(&stream, ptr, size);

  return __llvm_profile_dump(&stream.common);
}

void __gcov_reset(void)
{
}
