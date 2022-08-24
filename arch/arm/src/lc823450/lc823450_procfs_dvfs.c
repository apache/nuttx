/****************************************************************************
 * arch/arm/src/lc823450/lc823450_procfs_dvfs.c
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

#include <sys/types.h>
#include <sys/stat.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>

#include <arch/irq.h>

#include "lc823450_dvfs2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DVFS_LINELEN  128

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dvfs_file_s
{
  struct procfs_file_s  base;    /* Base open file structure */
  unsigned int linesize;         /* Number of valid characters in line[] */
  char line[DVFS_LINELEN];       /* Pre-allocated buffer for formatted lines */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     dvfs_open(struct file *filep, const char *relpath,
                         int oflags, mode_t mode);
static int     dvfs_close(struct file *filep);
static ssize_t dvfs_read(struct file *filep, char *buffer,
                         size_t buflen);
static ssize_t dvfs_write(struct file *filep, const char *buffer,
                          size_t buflen);
static int     dvfs_dup(const struct file *oldp,
                        struct file *newp);
static int     dvfs_stat(const char *relpath, struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct procfs_operations dvfs_procfsoperations =
{
  dvfs_open,      /* open */
  dvfs_close,     /* close */
  dvfs_read,      /* read */
  dvfs_write,     /* write */
  dvfs_dup,       /* dup */
  NULL,           /* opendir */
  NULL,           /* closedir */
  NULL,           /* readdir */
  NULL,           /* rewinddir */
  dvfs_stat       /* stat */
};

static const struct procfs_entry_s g_procfs_dvfs =
{
  "dvfs",
  &dvfs_procfsoperations
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern int8_t   g_dvfs_enabled;
extern int8_t   g_dvfs_auto;
extern uint16_t g_dvfs_cur_freq;
extern uint32_t g_dvfs_freq_stat[3];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dvfs_open
 ****************************************************************************/

static int dvfs_open(struct file *filep, const char *relpath,
                    int oflags, mode_t mode)
{
  struct dvfs_file_s *priv;

  finfo("Open '%s'\n", relpath);

  /* Allocate a container to hold the task and attribute selection */

  priv = (struct dvfs_file_s *)kmm_zalloc(sizeof(struct dvfs_file_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (void *)priv;
  return OK;
}

/****************************************************************************
 * Name: dvfs_close
 ****************************************************************************/

static int dvfs_close(struct file *filep)
{
  struct dvfs_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (struct dvfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: dvfs_read
 ****************************************************************************/

static ssize_t dvfs_read(struct file *filep, char *buffer,
                         size_t buflen)
{
  struct dvfs_file_s *priv;
  size_t linesize;
  size_t copysize;
  size_t remaining;
  size_t totalsize;
  off_t  offset = filep->f_pos;
  int    i;
  uint64_t idletime[CONFIG_SMP_NCPUS];

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  priv = (struct dvfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  remaining = buflen;
  totalsize = 0;

  linesize = snprintf(priv->line,
                      DVFS_LINELEN,
                      "cur_freq %d\n", g_dvfs_cur_freq);
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  linesize = snprintf(priv->line,
                      DVFS_LINELEN,
                      "enable %d\n", g_dvfs_enabled);
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  linesize = snprintf(priv->line,
                      DVFS_LINELEN,
                      "auto %d\n", g_dvfs_auto);
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  linesize = snprintf(priv->line,
                      DVFS_LINELEN,
                      "fstat %" PRId32 " %" PRId32 " %" PRId32 "\n",
                      g_dvfs_freq_stat[0],
                      g_dvfs_freq_stat[1],
                      g_dvfs_freq_stat[2]);
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  lc823450_dvfs_get_idletime(idletime);

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      linesize = snprintf(priv->line,
                          DVFS_LINELEN,
                          "idle%d %lld\n",
                          i, idletime[i]);

      copysize = procfs_memcpy(priv->line, linesize, buffer,
                               remaining, &offset);
      totalsize += copysize;
      buffer    += copysize;
      remaining -= copysize;
    }

  /* Update the file offset */

  if (totalsize > 0)
    {
      filep->f_pos += totalsize;
    }

  return totalsize;
}

/****************************************************************************
 * Name: procfs_write
 ****************************************************************************/

static ssize_t dvfs_write(struct file *filep, const char *buffer,
                          size_t buflen)
{
  char line[DVFS_LINELEN];
  char cmd[16];
  int  n;
  int  tmp;

  n = MIN(buflen, DVFS_LINELEN);
  strlcpy(line, buffer, n);

  n = strcspn(line, " ");
  n = MIN(n, sizeof(cmd));
  strlcpy(cmd, line, n);

  if (0 == strcmp(cmd, "cur_freq"))
    {
      tmp = atoi(line + (n + 1));
      lc823450_dvfs_set_freq(tmp);
    }
  else if (0 == strcmp(cmd, "enable"))
    {
      tmp = atoi(line + (n + 1));
      g_dvfs_enabled = tmp;
    }
  else if (0 == strcmp(cmd, "auto"))
    {
      tmp = atoi(line + (n + 1));
      g_dvfs_auto = tmp;
    }
  else
    {
      serr("ERROR: %s not supported.\n", cmd);
    }

  return buflen;
}

/****************************************************************************
 * Name: dvfs_dup
 ****************************************************************************/

static int dvfs_dup(const struct file *oldp, struct file *newp)
{
  struct dvfs_file_s *oldpriv;
  struct dvfs_file_s *newpriv;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (struct dvfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (struct dvfs_file_s *)kmm_zalloc(sizeof(struct dvfs_file_s));
  if (!newpriv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct dvfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: dvfs_stat
 ****************************************************************************/

static int dvfs_stat(const char *relpath, struct stat *buf)
{
  buf->st_mode    =
    S_IFREG |
    S_IROTH | S_IWOTH |
    S_IRGRP | S_IWGRP |
    S_IRUSR | S_IWUSR;

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dvfs_procfs_register
 ****************************************************************************/

int dvfs_procfs_register(void)
{
  return procfs_register(&g_procfs_dvfs);
}
