/****************************************************************************
 * libs/libbuiltin/libgcc/gcov.c
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

#include <gcov.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/streams.h>
#include <nuttx/reboot_notifier.h>

#include "gcov_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dump_args
{
  FAR CODE int  (*mkdir)(FAR const char *);
  FAR struct lib_outstream_s *stream;
  FAR char *prefix;
  FAR char *tokens[GCOV_PATH_MAX_TOKENS];
  size_t    token_cnt;
  FAR char *path_buffer;
  int       strip;
};

typedef CODE int (*dump_fn_t)(FAR struct dump_args *,
                              FAR char *, FAR uint8_t *, size_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sentinel node: the last real node's next points here.
 * This distinguishes "already in list" (next != NULL) from
 * "never inserted" (next == NULL, zero-initialized in BSS).
 */

static struct gcov_info g_gcov_list_end;

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct gcov_info *__gcov_info_start;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_path_tokens(FAR char *path,
                           FAR char **tokens, size_t max_tokens)
{
  size_t token_count = 0;
  FAR char *token;

  token = strtok(path, "/");
  while (token != NULL)
    {
      if (token_count >= max_tokens)
        {
          return -ENAMETOOLONG;
        }

      tokens[token_count++] = token;
      token = strtok(NULL, "/");
    }

  tokens[token_count] = NULL;
  return token_count;
}

static size_t gcov_convert(FAR uint8_t *buffer,
                           FAR const struct gcov_info *info)
{
  FAR struct gcov_fn_info *gfi;
  FAR struct gcov_ctr_info *gci;
  uint32_t functions;
  uint32_t counts;
  uint32_t value;
  size_t pos = 0;

  /* File header. */

  dump_unsigned(buffer, &pos, GCOV_DATA_MAGIC);
  dump_unsigned(buffer, &pos, info->version);
  dump_unsigned(buffer, &pos, info->stamp);

#ifdef GCOV_12_FORMAT
  dump_unsigned(buffer, &pos, info->checksum);
#endif

  /* Function headers. */

  for (functions = 0; functions < info->n_functions; functions++)
    {
      unsigned int length = 0;
      gfi = info->functions[functions];

      if (gfi && gfi->key == info)
        {
          length = GCOV_TAG_FUNCTION_LENGTH;
        }

      /* Function record. */

      dump_unsigned(buffer, &pos, GCOV_TAG_FUNCTION);
      dump_unsigned(buffer, &pos, length);

      if (length == 0)
        {
          continue;
        }

      dump_unsigned(buffer, &pos, gfi->ident);
      dump_unsigned(buffer, &pos, gfi->lineno_checksum);
      dump_unsigned(buffer, &pos, gfi->cfg_checksum);

      gci = gfi->ctrs;
      for (counts = 0; counts < GCOV_COUNTERS; counts++)
        {
          if (!info->merge[counts])
            {
              continue;
            }

          /* Counter record. */

          if (counts == GCOV_COUNTER_V_TOPN ||
              counts == GCOV_COUNTER_V_INDIR)
            {
              write_topn_counters(buffer, &pos, gci, counts, gci->num);
            }
          else
            {
              dump_unsigned(buffer, &pos, GCOV_TAG_FOR_COUNTER(counts));
              dump_unsigned(buffer, &pos, GCOV_TAG_COUNTER_LENGTH(gci->num));

              for (value = 0; value < gci->num; value++)
                {
                  dump_counter(buffer, &pos, gci->values[value]);
                }
            }

          gci++;
        }
    }

  return pos;
}

static int gcov_mkdir(FAR const char *path)
{
  if (access(path, F_OK) != 0)
    {
      return mkdir(path, 0777);
    }

  return OK;
}

#ifdef CONFIG_COVERAGE_GCOV_DUMP_REBOOT
static int gcov_reboot_notify(FAR struct notifier_block *nb,
                              unsigned long action, FAR void *data)
{
  /* __gcov_dump() performs filesystem operations (stat/open/write/mkdir)
   * that are unsafe in IRQ/panic context where IRQs are disabled and the
   * kernel state may be corrupted.  The assert path invokes this notifier
   * with action == SYS_HALT; running heavy filesystem code from there can
   * easily fault and trigger a recursive assert, masking the original
   * crash cause.  Only dump coverage data on clean shutdowns (restart or
   * power-off) from task context, matching the guard used by the VFS
   * sync reboot handler.
   */

  if (up_interrupt_context() ||
      (action != SYS_RESTART && action != SYS_POWER_OFF))
    {
      return OK;
    }

  __gcov_dump();
  return OK;
}
#endif

static int gcov_standard_dump(FAR struct dump_args *args,
                              FAR char *path, FAR uint8_t *data, size_t size)
{
  int written;
  int fd;

  fd = _NX_OPEN(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0)
    {
      return -errno;
    }

  written = _NX_WRITE(fd, data, size);
  if (written != size)
    {
      return -errno;
    }

  _NX_CLOSE(fd);
  return OK;
}

static int gcov_onefile_dump(FAR struct dump_args *args,
                             FAR char *path,
                             FAR uint8_t *data, size_t size)
{
  FAR struct lib_outstream_s *stream = args->stream;
  struct lib_hexdumpstream_s hexstream;
  uint16_t checksum = 0;
  int i;

  lib_hexdumpstream(&hexstream, stream);
  for (i = 0; i < size; i++)
    {
      checksum += ((FAR const uint8_t *)data)[i];
    }

  lib_sprintf(stream,
              "gcov start filename:%s size: %zuByte\n",
              path, size);

  lib_stream_puts(&hexstream, data, size);
  lib_stream_flush(&hexstream);

  lib_sprintf(stream,
              "gcov end filename:%s checksum: %#0x\n",
              path, checksum);
  lib_stream_flush(stream);

  return OK;
}

static int gcov_binfile_dump(FAR struct dump_args *args,
                             FAR char *path,
                             FAR uint8_t *data, size_t size)
{
  /* Binary pack: [path_len(4B LE)][path+NUL][size(4B LE)][data]
   * Use gcov_split.py on host to unpack.
   * Write header and data separately to avoid double malloc.
   */

  uint32_t path_len = strlen(path) + 1;
  uint32_t data_size = size;
  uint8_t hdr[PATH_MAX + 8];
  FAR struct lib_rawoutstream_s *rstream =
      (FAR struct lib_rawoutstream_s *)args->stream;

  memcpy(hdr, &path_len, sizeof(path_len));
  memcpy(hdr + sizeof(path_len), path, path_len);
  memcpy(hdr + sizeof(path_len) + path_len,
         &data_size, sizeof(data_size));

  _NX_WRITE(rstream->fd, hdr,
            sizeof(path_len) + path_len + sizeof(data_size));
  _NX_WRITE(rstream->fd, data, size);

  return OK;
}

static int gcov_dump_foreach(dump_fn_t handler, FAR struct dump_args *args)
{
  FAR struct gcov_info *info;
  FAR char *filename;
  FAR uint8_t *data;
  int token_count;
  int size;
  int i;

  for (info = __gcov_info_start;
       info && info != &g_gcov_list_end;
       info = info->next)
    {
      memset(args->path_buffer, 0x00, PATH_MAX);

      size = gcov_convert(NULL, info);
      data = lib_malloc(size);
      if (data == NULL)
        {
          syslog(LOG_ERR, "gcov alloc failed! skip %s",
                          info->filename);
          continue;
        }

      gcov_convert(data, info);

      filename = strdup(info->filename);
      if (filename == NULL)
        {
          syslog(LOG_ERR, "gcov alloc failed! skip %s",
                          info->filename);
          lib_free(data);
          continue;
        }

      token_count = args->token_cnt +
                    get_path_tokens(filename,
                                    &args->tokens[args->token_cnt],
                                    GCOV_PATH_MAX_TOKENS);
      if (token_count < args->token_cnt)
        {
          syslog(LOG_ERR, "gcov get path tokens failed! skip %s",
                          info->filename);
          lib_free(filename);
          lib_free(data);
          continue;
        }

      /* Build prefix part (tokens[0 .. token_cnt-1]) */

      for (i = 0; i < args->token_cnt; i++)
        {
          strcat(args->path_buffer, "/");
          strcat(args->path_buffer, args->tokens[i]);
        }

      /* Build source path part, skipping first 'strip' tokens */

      for (i = args->token_cnt + args->strip;
           i < token_count - 1; i++)
        {
          strcat(args->path_buffer, "/");
          strcat(args->path_buffer, args->tokens[i]);
          if (args->mkdir)
            {
              args->mkdir(args->path_buffer);
            }
        }

      strcat(args->path_buffer, "/");
      strcat(args->path_buffer,
             args->tokens[token_count - 1]);

      if (handler(args, args->path_buffer, data, size) < 0)
        {
          syslog(LOG_ERR, "gcov dump %s failed!\n", args->path_buffer);
        }

      lib_free(filename);
      lib_free(data);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __gcov_init(FAR struct gcov_info *info)
{
#ifdef CONFIG_COVERAGE_GCOV_DUMP_REBOOT
  static struct notifier_block nb;
#endif
  char path[PATH_MAX] = CONFIG_COVERAGE_DEFAULT_PREFIX;
  static int inited = 0;
  struct tm *tm_info;
  time_t cur;

  if (!inited)
    {
      cur = time(NULL);
      tm_info = localtime(&cur);

      strftime(path + strlen(path),
               PATH_MAX,
               "/gcov_%Y%m%d_%H%M%S",
               tm_info);

      setenv("GCOV_PREFIX_STRIP", CONFIG_COVERAGE_DEFAULT_PREFIX_STRIP, 1);
      setenv("GCOV_PREFIX", path, 1);

#ifdef CONFIG_COVERAGE_GCOV_DUMP_REBOOT
      nb.notifier_call = gcov_reboot_notify;
      register_reboot_notifier(&nb);
#endif

      inited++;
    }

  /* Guard against duplicate insertion which would create a cycle.
   * GCC zero-initializes the static gcov_info structs, so next == NULL
   * means "never inserted".  Once inserted, next points to either
   * the previous head or the sentinel g_gcov_list_end (for the tail).
   */

  if (info->next != NULL || info == __gcov_info_start)
    {
      return;
    }

  info->next = __gcov_info_start ? __gcov_info_start : &g_gcov_list_end;
  __gcov_info_start = info;
}

void __gcov_merge_add(FAR gcov_type *counters, unsigned int n_counters)
{
}

void __gcov_exit(void)
{
}

void __gcov_execve(void)
{
}

void __gcov_execl(void)
{
}

void __gcov_execv(void)
{
}

void __gcov_execle(void)
{
}

pid_t __gcov_fork(void)
{
  return fork();
}

void __gcov_dump(void)
{
  int ret = -1;
  FAR char *prefix;
  struct stat state;
  struct dump_args args =
  {
    0
  };

  prefix = getenv("GCOV_PREFIX");
  if (prefix == NULL)
    {
      syslog(LOG_ERR, "No path prefix specified");
    }

  args.path_buffer = lib_get_tempbuffer(PATH_MAX);
  if (args.path_buffer == NULL)
    {
      syslog(LOG_ERR, "gcov alloc failed!");
      return;
    }

  if (stat(prefix, &state) ||
      S_ISREG(state.st_mode) || S_ISCHR(state.st_mode))
    {
      FAR char *strip_env = getenv("GCOV_PREFIX_STRIP");
      struct lib_rawoutstream_s stream;
      dump_fn_t handler;
      int fd;

      fd = _NX_OPEN(prefix, O_WRONLY | O_CREAT | O_APPEND, 0666);
      if (fd < 0)
        {
          syslog(LOG_ERR, "open %s failed! ret: %d\n", prefix, ret);
          goto exit;
        }

      lib_rawoutstream(&stream, fd);
      args.stream = &stream.common;

      if (strip_env && atoi(strip_env) > 0)
        {
          args.strip = atoi(strip_env);
          handler = gcov_binfile_dump;
        }
      else
        {
          handler = gcov_onefile_dump;
        }

      ret = gcov_dump_foreach(handler, &args);

      _NX_CLOSE(fd);
    }
  else if(S_ISDIR(state.st_mode))
    {
      args.mkdir = gcov_mkdir;
      args.strip = atoi(getenv("GCOV_PREFIX_STRIP"));
      args.token_cnt = get_path_tokens(prefix,
                                       args.tokens,
                                       GCOV_PATH_MAX_TOKENS);

      ret = gcov_dump_foreach(gcov_standard_dump, &args);
    }
  else
    {
      ret = -ENOTDIR;
    }

exit:

  if (ret < 0)
    {
      syslog(LOG_INFO, "Gcov dump failed\n");
    }
  else
    {
      syslog(LOG_INFO, "Gcov dump complete\n");
    }

  lib_put_tempbuffer(args.path_buffer);
}

void __gcov_reset(void)
{
  FAR struct gcov_info *info;
  FAR struct gcov_ctr_info *gci;
  uint32_t functions;
  uint32_t counts;

  for (info = __gcov_info_start;
       info && info != &g_gcov_list_end;
       info = info->next)
    {
      for (functions = 0; functions < info->n_functions; functions++)
        {
          gci = info->functions[functions]->ctrs;

          for (counts = 0; counts < GCOV_COUNTERS; counts++)
            {
              if (!info->merge[counts])
                {
                  continue;
                }

              memset(gci->values, 0, gci->num * sizeof(gcov_type));
              gci++;
            }
        }
    }
}

#if defined(CONFIG_LIB_SYSCALL) && defined(__KERNEL__)
int gcov(int op)
{
  if (op < GCOV_OP_MIN || op > GCOV_OP_MAX)
    {
      return -EINVAL;
    }

  switch (op)
    {
      case GCOV_OP_RESET:
        __gcov_reset();
        break;
      case GCOV_OP_DUMP:
        __gcov_dump();
        break;
    }

  return 0;
}
#endif
