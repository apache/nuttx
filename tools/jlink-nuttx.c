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

#define API_VER                   101
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

/* GCC specific definitions */

#ifdef __GNUC__

/* The packed attribute informs GCC that the structure elements are packed,
 * ignoring other alignment rules.
 */

#  define begin_packed_struct
#  define end_packed_struct __attribute__ ((packed))

#else

#  warning "Unsupported compiler"
#  define begin_packed_struct
#  define end_packed_struct

#endif

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

begin_packed_struct struct tcbinfo_s
{
  uint16_t pid_off;
  uint16_t state_off;
  uint16_t pri_off;
  uint16_t name_off;
  uint16_t regs_off;
  uint16_t basic_num;
  uint16_t total_num;
  begin_packed_struct
  union
  {
    uint8_t  u[8];
    uint16_t *p;
  }
  end_packed_struct reg_off;
  uint16_t reg_offs[0];
} end_packed_struct;

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
  uint32_t                 *regsaddr;
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
                   uint32_t *pid)
{
  int ret;

  ret = READU32(priv->pidhash[idx] + priv->tcbinfo->pid_off, pid);
  if (ret != 0)
    {
      PERROR("read %d pid error return %d\n", idx, ret);
    }

  return ret;
}

static int get_idx_from_pid(struct plugin_priv_s *priv,
                            uint32_t pid)
{
  int idx;

  for (idx = 0; idx < priv->ntcb; idx++)
    {
      uint32_t tmppid;

      int ret = get_pid(priv, idx, &tmppid);
      if (ret != 0)
        {
          return ret;
        }

      if (tmppid == pid)
        {
          return idx;
        }
    }

  return -ENOENT;
}

static int setget_reg(struct plugin_priv_s *priv, uint32_t idx,
                      uint32_t regidx, uint32_t *regval, bool write)
{
  uint32_t regaddr;

  if (regidx >= priv->tcbinfo->total_num)
    {
      return -EINVAL;
    }

  if (priv->tcbinfo->reg_offs[regidx] == UINT16_MAX)
    {
      if (write == 0)
        {
          *regval = 0;
        }

      return 0;
    }

  regaddr = priv->regsaddr[idx] + priv->tcbinfo->reg_offs[regidx];

  if (write)
    {
      WRITEU32(regaddr, *regval);
    }
  else
    {
      int ret = READU32(regaddr, regval);
      if (ret != 0)
        {
          PERROR("regread %d regidx %d error %d\n", idx, regidx, ret);
          return ret;
        }
    }

  return 0;
}

static int update_tcbinfo(struct plugin_priv_s *priv)
{
  if (!priv->tcbinfo)
    {
      uint16_t total_num;
      uint32_t reg_off;
      int ret;

      ret = READU16(g_symbols[TCBINFO].address +
                    offsetof(struct tcbinfo_s, total_num), &total_num);
      if (ret != 0)
        {
          PERROR("error reading regs ret %d\n", ret);
          return ret;
        }

      if (!total_num)
        {
          return -EIO;
        }

      ret = READU32(g_symbols[TCBINFO].address +
                    offsetof(struct tcbinfo_s, reg_off), &reg_off);
      if (ret != 0)
        {
          PERROR("error in read regoffs address ret %d\n", ret);
          return ret;
        }

      priv->tcbinfo = ALLOC(sizeof(struct tcbinfo_s) +
                            total_num * sizeof(uint16_t));

      if (!priv->tcbinfo)
        {
          PERROR("error in malloc tcbinfo_s\n");
          return -ENOMEM;
        }

      ret = READMEM(g_symbols[TCBINFO].address, (char *)priv->tcbinfo,
                    sizeof(struct tcbinfo_s));
      if (ret != sizeof(struct tcbinfo_s))
        {
          PERROR("error in read tcbinfo_s ret %d\n", ret);
          return ret;
        }

      ret = READMEM(reg_off, (char *)&priv->tcbinfo->reg_offs[0],
                    total_num * sizeof(uint16_t));
      if (ret != total_num * sizeof(uint16_t))
        {
          PERROR("error in read tcbinfo_s reg_offs ret %d\n", ret);
          return ret;
        }
    }

  return 0;
}

static int update_pidhash(struct plugin_priv_s *priv)
{
  uint32_t npidhash;
  uint32_t pidhashaddr;
  int ret;
  int i;

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

      priv->regsaddr =
        REALLOC(priv->regsaddr, npidhash * sizeof(uint32_t *));
      if (!priv->regsaddr)
        {
          PERROR("error in malloc regsaddr\n");
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

  for (i = 0; i < priv->npidhash; i++)
    {
      if (priv->pidhash[i])
        {
          ret = READU32(priv->pidhash[i] + priv->tcbinfo->regs_off,
                        &priv->regsaddr[i]);
          if (ret != 0)
            {
              PERROR("error in task %d read regs pointer %d\n", i, ret);
              return ret;
            }
        }
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
          priv->pidhash[priv->ntcb]  = priv->pidhash[i];
          priv->regsaddr[priv->ntcb] = priv->regsaddr[i];
          priv->ntcb++;
        }
    }

  ret = READU32(g_symbols[READYTORUN].address, &tcbaddr);
  if (ret != 0)
    {
      PERROR("read readytorun error return %d\n", ret);
      return ret;
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
  return API_VER;
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

struct symbols_s *RTOS_GetSymbols(void)
{
  return g_symbols;
}

uint32_t RTOS_GetNumThreads(void)
{
  if (g_plugin_priv.ntcb == 0)
    {
      RTOS_UpdateThreads();
    }

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
      uint32_t pid;

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
      return idx;
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
  int ret;
  uint32_t regval = 0;

  threadid -= THREADID_BASE;

  /* current task read by J-Link self */

  if (threadid == g_plugin_priv.running)
    {
      return -ENOTSUP;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return idx;
    }

  ret = setget_reg(&g_plugin_priv, idx, regindex, &regval, false);
  if (ret != 0)
    {
      return ret;
    }

  encode_hex(hexregval, regval);
  return 0;
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
      return -ENOTSUP;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return idx;
    }

  for (j = 0; j < g_plugin_priv.tcbinfo->basic_num; j++)
    {
      regval = 0;

      int ret = setget_reg(&g_plugin_priv, idx, j, &regval, false);
      if (ret != 0)
        {
          return ret;
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
      return -ENOTSUP;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return idx;
    }

  regval = decode_hex(hexregval);

  return setget_reg(&g_plugin_priv, idx, regindex, &regval, true);
}

int RTOS_SetThreadRegList(char *hexreglist, uint32_t threadid)
{
  int idx;
  uint32_t j;
  uint32_t regval;

  threadid -= THREADID_BASE;

  if (threadid == g_plugin_priv.running)
    {
      return -ENOTSUP;
    }

  idx = get_idx_from_pid(&g_plugin_priv, threadid);
  if (idx < 0)
    {
      PERROR("error in get_idx_from_pid return %d\n", idx);
      return idx;
    }

  for (j = 0; j < g_plugin_priv.tcbinfo->basic_num; j++)
    {
      regval = decode_hex(hexreglist);

      int ret = setget_reg(&g_plugin_priv, idx, j, &regval, true);
      if (ret != 0)
        {
          return ret;
        }

      hexreglist += 4;
    }

  return 0;
}
