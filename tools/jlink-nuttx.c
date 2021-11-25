/****************************************************************************
 * tools/jlink-nuttx.c
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

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Marcos for J-Link plugin API */

#define PLUGIN_VER                100
#define DISPLAY_LENGTH            256
#define THREADID_BASE             1

/* Marco for TCB struct size */

#define TCB_NAMESIZE              256

/* Marcos for J-Link API ops */

#define REALLOC(ptr, size)        g_plugin_priv.jops->realloc(ptr, size)
#define ALLOC(size)               g_plugin_priv.jops->alloc(size)
#define FREE(ptr)                 g_plugin_priv.jops->free(ptr)

#define READMEM(addr, ptr, size)  g_plugin_priv.jops->readmem(addr, ptr, size)
#define READU32(addr, data)       g_plugin_priv.jops->readu32(addr, data)
#define READU16(addr, data)       g_plugin_priv.jops->readu16(addr, data)
#define READU8(addr, data)        g_plugin_priv.jops->readu8(addr, data)

#define WRITEU32(addr, data)      g_plugin_priv.jops->writeu32(addr, data)

#define PERROR                    g_plugin_priv.jops->erroroutf
#define PLOG                      g_plugin_priv.jops->logoutf

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum symbol_e
{
  PIDHASH = 0,
  NPIDHASH,
  TCBINFO,
  READYTORUN,
  NSYMBOLS
};

struct tcbinfo_s
{
  uint16_t pid_off;
  uint16_t state_off;
  uint16_t pri_off;
  uint16_t name_off;
  uint16_t reg_num;
  uint16_t reg_offs[0];
};

struct symbols_s
{
  const char *name;
  int optional;
  uint32_t address;
};

/* J-Link server functions that can be called by the plugin */

struct jlink_ops_s
{
  /* API version v1.0 and higher */

  void  (*free)          (void *p);
  void *(*alloc)         (unsigned size);
  void *(*realloc)       (void *p, unsigned size);
  void  (*logoutf)       (const char *sformat, ...);
  void  (*debugoutf)     (const char *sformat, ...);
  void  (*warnoutf)      (const char *sformat, ...);
  void  (*erroroutf)     (const char *sformat, ...);
  int   (*readmem)       (uint32_t addr, char *pdata,
                          unsigned int numbytes);
  char  (*readu8)        (uint32_t addr, uint8_t *pdata);
  char  (*readu16)       (uint32_t addr, uint16_t *pdata);
  char  (*readu32)       (uint32_t addr, uint32_t *pdata);
  int   (*writemem)      (uint32_t addr, const char *pdata,
                          unsigned numbytes);
  void  (*writeu8)       (uint32_t addr, uint8_t data);
  void  (*writeu16)      (uint32_t addr, uint16_t data);
  void  (*writeu32)      (uint32_t addr, uint32_t data);
  uint32_t (*load16te)   (const uint8_t *p);
  uint32_t (*load24te)   (const uint8_t *p);
  uint32_t (*load32te)   (const uint8_t *p);

  /* API version v1.1 and higher */

  uint32_t   (*readreg)  (uint32_t regindex);
  void       (*writereg) (uint32_t regindex, uint32_t value);

  /* End marker */

  void   *dummy;
};

struct plugin_priv_s
{
  uint32_t                 *pidhash;
  uint32_t                 npidhash;
  struct tcbinfo_s         *tcbinfo;
  uint16_t                 running;
  uint32_t                 ntcb;
  const struct jlink_ops_s *jops;
};

/****************************************************************************
 * Private data
 ****************************************************************************/

static struct symbols_s g_symbols[] =
{
  {"g_pidhash",            0, 0},
  {"g_npidhash",           0, 0},
  {"g_tcbinfo",            0, 0},
  {"g_readytorun",         0, 0},
  { NULL,                  0, 0}
};

static struct plugin_priv_s g_plugin_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int encode_hex(char *line, uint32_t value)
{
  /* output line in intel hex format */

  return snprintf(line, 9, "%02x%02x%02x%02x", value & 0xff,
                 (value & 0xff00) >> 8, (value & 0xff0000) >> 16,
                 (value & 0xff000000) >> 24);
}

static inline uint32_t decode_hex(const char *line)
{
  /* Get value from hex format line */

  uint32_t i;
  uint32_t value = 0;

  for (i = 7; i >= 0; )
    {
      value += (value << 8) + (line[i--] - '0');
      value += (line[i--] - '0') << 4;
    }

  return value;
}

static int get_pid(struct plugin_priv_s *priv, uint32_t idx,
                   uint16_t *pid)
{
  int ret;

  ret = READU16(priv->pidhash[idx] + priv->tcbinfo->pid_off, pid);
  if (ret != 0)
    {
      PERROR("read %d pid error return %d\n", idx, ret);
      return ret;
    }

  return 0;
}

static int get_idx_from_pid(struct plugin_priv_s *priv,
                            uint32_t pid)
{
  int idx;

  for (idx = 0; idx < priv->ntcb; idx++)
    {
      uint16_t tmppid;

      if (get_pid(priv, idx, &tmppid))
        {
          return -1;
        }

      if (tmppid == pid)
        {
          return idx;
        }
    }

  return -1;
}

static int setget_reg(struct plugin_priv_s *priv, uint32_t idx,
                      uint32_t regidx, uint32_t *regval, bool write)
{
  uint32_t regaddr;
  int ret = 0;

  regaddr = priv->pidhash[idx] + priv->tcbinfo->reg_offs[regidx];

  if (write)
    {
      WRITEU32(regaddr, *regval);
    }
  else
    {
      ret = READU32(regaddr, regval);
      if (ret != 0)
        {
          PERROR("regread %d regidx %d error %d\n", idx, regidx, ret);
          return ret;
        }
    }

  return ret;
}

static int update_tcbinfo(struct plugin_priv_s *priv)
{
  if (!priv->tcbinfo)
    {
      uint16_t reg_num;
      int ret;

      ret = READU16(g_symbols[TCBINFO].address +
                    offsetof(struct tcbinfo_s, reg_num), &reg_num);
      if (ret != 0 || !reg_num)
        {
          PERROR("error reading regs ret %d\n", ret);
          return ret;
        }

      priv->tcbinfo = ALLOC(sizeof(struct tcbinfo_s) +
                            reg_num * sizeof(uint16_t));

      if (!priv->tcbinfo)
        {
          PERROR("error in malloc tcbinfo_s\n");
          return -ENOMEM;
        }

      ret = READMEM(g_symbols[TCBINFO].address, (char *)priv->tcbinfo,
                    sizeof(struct tcbinfo_s) + reg_num * sizeof(uint16_t));
      if (ret != sizeof(struct tcbinfo_s) + reg_num * sizeof(uint16_t))
        {
          PERROR("error in read tcbinfo_s ret %d\n", ret);
          return ret;
        }

      PLOG("setup success! regs %d\n", priv->tcbinfo->reg_num);
    }

  return 0;
}

static int update_pidhash(struct plugin_priv_s *priv)
{
  uint32_t npidhash;
  uint32_t pidhashaddr;
  int ret;

  ret = READU32(g_symbols[NPIDHASH].address, &npidhash);
  if (ret != 0 || npidhash == 0)
    {
      PERROR("error reading npidhash ret %d\n", ret);
      return ret;
    }

  if (npidhash != priv->npidhash)
    {
      priv->pidhash = REALLOC(priv->pidhash,
                              npidhash * sizeof(uint32_t *));
      if (!priv->pidhash)
        {
          PERROR("error in malloc pidhash\n");
          return -ENOMEM;
        }

      PLOG("npidhash change from %d to %d!\n", priv->npidhash, npidhash);
      priv->npidhash = npidhash;
    }

  ret = READU32(g_symbols[PIDHASH].address, &pidhashaddr);
  if (ret != 0 || pidhashaddr == 0)
    {
      PERROR("error in read pidhashaddr ret %d\n", ret);
      return ret;
    }

  ret = READMEM(pidhashaddr, (char *)priv->pidhash,
                priv->npidhash * sizeof(uint32_t *));
  if (ret != priv->npidhash * sizeof(uint32_t *))
    {
      PERROR("error in read pidhash ret %d\n", ret);
      return ret;
    }

  return 0;
}

static int normalize_tcb(struct plugin_priv_s *priv)
{
  uint32_t i;
  uint32_t tcbaddr;
  int ret;

  priv->ntcb = 0;

  for (i = 0; i < priv->npidhash; i++)
    {
      if (priv->pidhash[i])
        {
          priv->pidhash[priv->ntcb++] = priv->pidhash[i];
        }
    }

  ret = READU32(g_symbols[READYTORUN].address, &tcbaddr);
  if (ret != 0)
    {
      PERROR("read readytorun error return %d\n", ret);
    }

  ret = READU16(tcbaddr + priv->tcbinfo->pid_off, &priv->running);
  if (ret != 0)
    {
      PERROR("read readytorun pid error return %d\n", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int RTOS_Init(const struct jlink_ops_s *api, uint32_t core)
{
  g_plugin_priv.jops = api;

  return 1;
}

uint32_t RTOS_GetVersion(void)
{
  return PLUGIN_VER;
}

struct symbols_s *RTOS_GetSymbols(void)
{
  return g_symbols;
}

uint32_t RTOS_GetNumThreads(void)
{
  return g_plugin_priv.ntcb;
}

uint32_t RTOS_GetCurrentThreadId(void)
{
  return THREADID_BASE + g_plugin_priv.running;
}

uint32_t RTOS_GetThreadId(uint32_t n)
{
  if (n < g_plugin_priv.ntcb)
    {
      uint16_t pid;

      if (get_pid(&g_plugin_priv, n, &pid) == 0)
        {
          return THREADID_BASE + pid;
        }
    }

  return 0;
}

int RTOS_GetThreadDisplay(char *display, uint32_t threadid)
{
  int idx;
  uint32_t len = 0;
  char name[TCB_NAMESIZE];
  uint8_t readval;
  int ret;

  threadid -= THREADID_BASE;

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return -1;
    }

  len += snprintf(display + len,
                  DISPLAY_LENGTH - len, "[PID:%03d]", threadid);

  ret = READMEM(g_plugin_priv.pidhash[idx] +
                g_plugin_priv.tcbinfo->name_off, name, TCB_NAMESIZE);
  if (ret != TCB_NAMESIZE)
    {
      PERROR("error in read tcb name return %d\n", ret);
    }

  /* check tcb name is valid or not */

  if ((name[0] > 'a' && name[0] < 'z')
      || (name[0] > 'A' && name[0] < 'Z'))
    {
      len += snprintf(display + len, DISPLAY_LENGTH - len,
                      "%s", name);
    }
  else
    {
      len += snprintf(display + len, DISPLAY_LENGTH - len,
                      "thread-%d", threadid);
    }

  ret = READU8(g_plugin_priv.pidhash[idx] +
               g_plugin_priv.tcbinfo->state_off, &readval);
  if (ret != 0)
    {
      PERROR("error in read tcb state return %d\n", ret);
    }

  len += snprintf(display + len, DISPLAY_LENGTH - len, ":%04d",
       readval);

  ret = READU8(g_plugin_priv.pidhash[idx] +
               g_plugin_priv.tcbinfo->pri_off, &readval);
  if (ret != 0)
    {
      PERROR("error in read tcb pri return %d\n", ret);
    }

  len += snprintf(display + len,
                  DISPLAY_LENGTH - len, "[PRI:%03d]", readval);

  return len;
}

int RTOS_GetThreadReg(char *hexregval, uint32_t regindex, uint32_t threadid)
{
  int idx;
  uint32_t regval = 0;

  threadid -= THREADID_BASE;

  /* current task read by J-Link self */

  if (threadid == g_plugin_priv.running)
    {
      return -1;
    }

  if (regindex > g_plugin_priv.tcbinfo->reg_num)
    {
      return -1;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return -1;
    }

  if (g_plugin_priv.tcbinfo->reg_offs[regindex])
    {
      setget_reg(&g_plugin_priv, idx, regindex, &regval, false);

      encode_hex(hexregval, regval);
      return 0;
    }

  return -1;
}

int RTOS_GetThreadRegList(char *hexreglist, uint32_t threadid)
{
  int idx;
  uint32_t j;
  uint32_t regval;

  threadid -= THREADID_BASE;

  /* current task read by J-Link self */

  if (threadid == g_plugin_priv.running)
    {
      return -1;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return -1;
    }

  for (j = 0; j < 17; j++)
    {
      regval = 0;

      if (g_plugin_priv.tcbinfo->reg_offs[j])
        {
          setget_reg(&g_plugin_priv, idx, j, &regval, false);
        }

      hexreglist += encode_hex(hexreglist, regval);
    }

  return 0;
}

int RTOS_SetThreadReg(char *hexregval,
                      uint32_t regindex, uint32_t threadid)
{
  int idx;
  uint32_t regval;

  threadid -= THREADID_BASE;

  if (threadid == g_plugin_priv.running)
    {
      return -1;
    }

  if (regindex > g_plugin_priv.tcbinfo->reg_num)
    {
      return -1;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return -1;
    }

  if (g_plugin_priv.tcbinfo->reg_offs[regindex])
    {
      regval = decode_hex(hexregval);

      setget_reg(&g_plugin_priv, idx, regindex, &regval, true);
      return 0;
    }

  return -1;
}

int RTOS_SetThreadRegList(char *hexreglist, uint32_t threadid)
{
  int idx;
  uint32_t j;
  uint32_t regval;

  threadid -= THREADID_BASE;

  if (threadid == g_plugin_priv.running)
    {
      return -1;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return -1;
    }

  for (j = 0; j < 17; j++)
    {
      if (g_plugin_priv.tcbinfo->reg_offs[j])
        {
          regval = decode_hex(hexreglist);

          setget_reg(&g_plugin_priv, idx, j, &regval, true);

          hexreglist += 4;
        }
      else
        {
          return -1;
        }
    }

  return 0;
}

int RTOS_UpdateThreads(void)
{
  int ret;

  ret = update_tcbinfo(&g_plugin_priv);
  if (ret)
    {
      return ret;
    }

  ret = update_pidhash(&g_plugin_priv);
  if (ret)
    {
      return ret;
    }

  ret = normalize_tcb(&g_plugin_priv);
  if (ret)
    {
      return ret;
    }

  return 0;
}
