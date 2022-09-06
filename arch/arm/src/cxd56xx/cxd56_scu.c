/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_scu.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/chip/scu.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_scufifo.h"
#include "cxd56_clock.h"
#include "cxd56_adc.h"

#ifdef CONFIG_CXD56_UDMAC
#include "cxd56_udmac.h"
#include <arch/chip/pm.h>
#endif

#include "hardware/cxd56_scu.h"
#include "hardware/cxd56_scuseq.h"
#include "hardware/cxd56_scufifo.h"

/* SCU firmware (iSoP) binary */

#ifdef CONFIG_CXD56_HPADC0_HIGHSPEED
#include "hardware/cxd5602_isop_hadc0_highspeed.h"
#else
#include "hardware/cxd5602_isop.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CXD56_SCU_DEBUG_ERR
#define scuerr(fmt, ...) syslog(LOG_ERR, fmt, ## __VA_ARGS__)
#else
#define scuerr(x, ...)
#endif
#ifdef CONFIG_CXD56_SCU_DEBUG_WARN
#define scuwarn(fmt, ...) syslog(LOG_WARN, fmt, ## __VA_ARGS__)
#else
#define scuwarn(x, ...)
#endif
#ifdef CONFIG_CXD56_SCU_DEBUG_INFO
#define scuinfo(fmt, ...) syslog(LOG_INFO, fmt, ## __VA_ARGS__)
#else
#define scuinfo(x, ...)
#endif

/* Sequencer has 128 instruction area (16bits/inst)
 * We allocate statically 12 instructions for each sequencers.
 */

#define INSTRUCTION_PER_SEQ 12

/* Sequencer request code */

#define REQ_DONE 3
#define REQ_SLEEP 4
#define REQ_STOP 5

/* Disable decimation by set 15 to ratio (N bit field) */

#define DECIMATION_OFF 15

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ev_notify_s
{
  int signo;               /* Signal number */
  pid_t pid;               /* Target PID */
  struct scuev_arg_s *arg; /* Event argument */
  struct scufifo_s *fifo;  /* Reverse reference to FIFO */
};

struct wm_notify_s
{
  int signo;                 /* Signal number */
  pid_t pid;                 /* Target PID */
  struct scutimestamp_s *ts; /* Event argument */
  struct scufifo_s *fifo;    /* Reverse reference to FIFO */
};

/* SCU FIFO management structure */

struct scufifo_s
{
  int8_t wid;   /* FIFO ID (Write side) */
  int8_t rid;   /* FIFO ID (Read side) */
  uint8_t type; /* Normal or Decimator */
  int8_t mid;   /* Combined MATHFUNC */

  uint16_t start; /* Start of FIFO memory */
  uint16_t size;  /* Size of FIFO memory */

  uint32_t interval; /* Sample interval used for timestamp calculation */
  uint16_t adjust;   /* Adjustment value used for timestamp calculation */

#ifdef CONFIG_CXD56_UDMAC
  DMA_HANDLE dma; /* DMA for reading sensing data */
  sem_t dmawait;  /* Wait semaphore for DMA complete */
  int dmaresult;  /* DMA result */
#endif
};

/* Sequencer */

struct seq_s
{
  int8_t id;       /* Sequencer ID */
  uint8_t type;    /* Normal or Decimator */
  uint8_t bustype; /* I2C[01] or SPI or ADC */
  int8_t active;   /* Indicate active status.
                    * When active, any APIs will be returned state
                    * error.
                    */
  uint8_t sample;  /* Bytes per sample */
  uint8_t rate;    /* Sampling rate must be 2^rate (rate = 0 - 9) */

  int8_t nr_fifos;        /* For decimation, max 3 decimators can be used. */
  struct scufifo_s *fifo; /* Owned FIFO (normal only) */
};

/* Decimation FIFO */

struct decimation_fifo_s
{
  struct scufifo_s *fifo; /* Pointer to decimation FIFO */
  uint8_t ratio;          /* Decimation ratio */
  uint8_t leveladj;       /* Output data level adjustment */
  uint8_t forcethrough;   /* Force through */
};

/* Decimator, sub class of sequencer */

struct decimator_s
{
  struct seq_s seq;                  /* Inherit, must be top of the structure */
  struct decimation_fifo_s dfifo[3]; /* Decimation FIFO */
};

struct cxd56_scudev_s
{
  sem_t syncwait;   /* Semaphore for synchronize with SCU firmware */
  mutex_t synclock; /* Mutex for exclusive access to sync */

  /* SCU hardware resource management bitmaps (1 = allocated) */

  uint8_t decimators; /* Bitmap for decimators */
  uint8_t sequencers; /* Bitmap for normal sequencers */
  uint8_t mathfuncs;  /* Bitmap for MATHFUNC */
  uint8_t oneshot;    /* Bitmap for Oneshots */

  sem_t oneshotwait[3]; /* Semaphore for wait oneshot sequence is done */
  int oneshoterr[3];    /* error code for oneshot sequencer */

  struct ev_notify_s event[3]; /* MATHFUNC event notify */
  struct wm_notify_s wm[14];   /* Watermark notify */
  int currentreq;
};

struct coeff_addr_s
{
  uint32_t s;
  uint32_t c0;
  uint32_t c1;
  uint32_t c2;
  uint32_t c3;
  uint32_t c4;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SCU hardware resource management *****************************************/

static int findzeroandset(uint8_t *bitmap, int nbits);
static void bitmapclear(uint8_t *bitmap, int bit);
static inline int8_t deci_alloc(void);
static inline void deci_free(int8_t sid);
static inline int8_t mathf_alloc(void);
static inline void mathf_free(int8_t mid);
static int8_t seq_alloc(void);
static inline void seq_free(int8_t sid);
static inline int8_t oneshot_alloc(void);
static inline void oneshot_free(int8_t tid);
static void seq_fifosetactive(struct seq_s *seq, int fifoid);
static void seq_fifosetinactive(struct seq_s *seq, int fifoid);
static int seq_fifoisactive(struct seq_s *seq, int fifoid);
static int seq_isactive(struct seq_s *seq);

/* Sequencer control ********************************************************/

static struct seq_s *seq_new(void);
static struct seq_s *deci_new(void);

static void seq_inhibitrequest(int req, bool set);
static void seq_sync(struct seq_s *seq, int req);
static void seq_setproperty(int sid, int slave, int dest,
                            int offset, int len);
static void seq_setinst(int sid, int offset, int len);
static void seq_setbus(int sid, int bustype);
static void seq_setdataformat(int sid, int start, int size, int swap,
                              int sample);
static void seq_setmathwritevecelenum(int sid, uint8_t elem);
static void seq_setsignconversion(int sid, int sign);
static void seq_setstartmode(int sid, int mode);
static void seq_setstartinterval(int sid, int interval);
static void seq_setstartphase(int sid, int phase);
static void seq_startseq(int sid);
static void seq_stopseq(int sid);
static int seq_setadjustment(struct seq_s *seq,
                             struct adjust_xyz_s *adj);
static int seq_setfilter(struct scufifo_s *fifo, int pos,
                         struct iir_filter_s iir[2]);
static int seq_seteventnotifier(struct scufifo_s *fifo,
                                struct scuev_notify_s *ev);
static void seq_offsetgainenable(int sid, bool enable);
static int seq_start(struct seq_s *seq, int fifoid);
static int seq_stop(struct seq_s *seq, int fifoid);
static int seq_setsamplingrate(struct seq_s *seq, uint8_t samplingrate);
static int seq_fifoinit(struct seq_s *seq, int fifoid, uint16_t fsize);
static void seq_fifofree(struct scufifo_s *fifo);
static inline struct scufifo_s *seq_getfifo(struct seq_s *seq,
                                            int fifoid);
static void seq_setdecimation(int wid, uint8_t ratio, uint8_t leveladj,
                              uint8_t forcethrough);
static int seq_setwatermark(struct seq_s *seq, int fifoid,
                            struct scufifo_wm_s *wm);
static void convert_firsttimestamp(struct scutimestamp_s *tm,
                                   uint16_t interval, uint16_t sample,
                                   uint16_t adjust);
static void latest_timestamp(struct scufifo_s *fifo, uint32_t interval,
                             struct scutimestamp_s *tm, uint16_t *samples);
static void seq_gettimestamp(struct scufifo_s *fifo,
                             struct scutimestamp_s *tm);

static int seq_oneshot(int bustype, int slave, uint16_t *inst,
                       uint32_t nr_insts, uint8_t *buffer, int len);
static void seq_setfifomode(struct seq_s *seq, int fifoid, int enable);
#ifdef CONFIG_CXD56_UDMAC
static void seq_fifodmadone(DMA_HANDLE handle, uint8_t status, void *arg);
#endif
static uint16_t seq_remakeinstruction(int bustype, uint16_t inst);

/* Mathfunction *************************************************************/

static void mathf_enable(int8_t mid, uint8_t wid);
static void mathf_disable(int8_t mid);
static inline void mathf_set_coeff(uint32_t caddr,
                                   struct iir_coeff_s *c);
static void mathf_setiirfilter(int mid, int n,
                               struct iir_filter_s *filter);

/* Interrupt handlers *******************************************************/

static int seq_scuirqhandler(int irq, void *context, void *arg);
static void seq_handlefifointr(struct cxd56_scudev_s *priv,
                               uint32_t intr);
static void seq_handlemathfintr(struct cxd56_scudev_s *priv,
                                uint32_t intr);
static void seq_handleoneshot(struct cxd56_scudev_s *priv,
                              uint32_t intr);
static void seq_handleisopdoneintr(struct cxd56_scudev_s *priv,
                                   uint32_t intr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cxd56_scudev_s g_scudev =
{
  .syncwait = SEM_INITIALIZER(0),
  .synclock = NXMUTEX_INITIALIZER,
  .oneshotwait =
  {
    SEM_INITIALIZER(0),
    SEM_INITIALIZER(0),
    SEM_INITIALIZER(0),
  },
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* SCU firmware (iSoP) */

extern const unsigned long scu_isopprog_array[];
extern const unsigned long sizeof_scu_isopprog_array;

/* XXX: Convert coefficiencies register address. */

#define __CADDR(m, n) \
  { \
    .s = SCU_MATHFUNC_PARAM_##m##_##n, \
    .c0 = SCU_MATHFUNC_PARAM_C0_##m##_##n##_MSB, \
    .c1 = SCU_MATHFUNC_PARAM_C1_##m##_##n##_MSB, \
    .c2 = SCU_MATHFUNC_PARAM_C2_##m##_##n##_MSB, \
    .c3 = SCU_MATHFUNC_PARAM_C3_##m##_##n##_MSB, \
    .c4 = SCU_MATHFUNC_PARAM_C4_##m##_##n##_MSB, \
  }

static const struct coeff_addr_s g_caddrs[3][2] =
{
  {
    __CADDR(0, 0),
    __CADDR(0, 1)
  },
  {
    __CADDR(1, 0),
    __CADDR(1, 1)
  },
  {
    __CADDR(2, 0),
    __CADDR(2, 1)
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seq_fifosetactive
 ****************************************************************************/

static void seq_fifosetactive(struct seq_s *seq, int fifoid)
{
  irqstate_t flags = enter_critical_section();
  seq->active |= 1 << fifoid;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: seq_fifosetinactive
 ****************************************************************************/

static void seq_fifosetinactive(struct seq_s *seq, int fifoid)
{
  irqstate_t flags = enter_critical_section();
  seq->active &= ~(1 << fifoid);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: seq_fifoisactive
 ****************************************************************************/

static int seq_fifoisactive(struct seq_s *seq, int fifoid)
{
  irqstate_t flags = enter_critical_section();
  int8_t active = seq->active;
  leave_critical_section(flags);
  return active & (1 << fifoid);
}

/****************************************************************************
 * Name: seq_isactive
 ****************************************************************************/

static int seq_isactive(struct seq_s *seq)
{
  irqstate_t flags = enter_critical_section();
  int8_t active = seq->active;
  leave_critical_section(flags);
  return active;
}

/****************************************************************************
 * Name: findzeroandset
 *
 * Description:
 *   Find zero bit from LSB and set when found.
 *
 * Input Parameters:
 *   bitmap - Bitmap for searching
 *   nbits  - Size of bitmap in bits
 *
 * Returned value:
 *   Bit number of set 1. -1 is not found.
 *
 ****************************************************************************/

static int findzeroandset(uint8_t *bitmap, int nbits)
{
  int i;
  irqstate_t flags;

  flags = enter_critical_section();
  for (i = 0; i < nbits; i++)
    {
      if (!(*bitmap & (1 << i)))
        {
          *bitmap |= 1 << i;
          leave_critical_section(flags);
          return i;
        }
    }

  leave_critical_section(flags);
  return -1;
}

/****************************************************************************
 * Name: bitmapclear
 *
 * Description:
 *   Common atomic bit clear
 *
 ****************************************************************************/

static void bitmapclear(uint8_t *bitmap, int bit)
{
  irqstate_t flags = enter_critical_section();
  *bitmap &= ~(1 << bit);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: deci_alloc
 *
 * Description:
 *   Allocate sequencer for decimation.
 *
 * Returned Value:
 *   Sequencer ID is returned on success. -1 is returned on failure.
 *
 ****************************************************************************/

static inline int8_t deci_alloc(void)
{
  int8_t ret;
  ret = findzeroandset(&g_scudev.decimators, 2);
  if (ret >= 0)
    {
      /* Wakeup SCU anyway if sequencer allocated successfully */

      seq_inhibitrequest(REQ_SLEEP, true);
    }

  return ret;
}

/****************************************************************************
 * Name: deci_free
 *
 * Description:
 *   Free sequencer for decimation.
 *
 ****************************************************************************/

static void deci_free(int8_t sid)
{
  DEBUGASSERT(sid == 0 || sid == 1);
  bitmapclear(&g_scudev.decimators, sid);
  if (!g_scudev.decimators && !g_scudev.sequencers)
    {
      /* SCU goes sleep when no sequencers running */

      seq_inhibitrequest(REQ_SLEEP, false);
    }
}

/****************************************************************************
 * Name: mathf_alloc
 *
 * Description:
 *   Allocate MATHFUNC filter. MATHFUNC is max 3.
 *
 * Returned Value:
 *   Allocated MATHFUNC ID is returned on success. -1 is returned on failure.
 *
 ****************************************************************************/

static inline int8_t mathf_alloc(void)
{
  return findzeroandset(&g_scudev.mathfuncs, 3);
}

/****************************************************************************
 * Name: mathf_free
 *
 * Description:
 *  Free MATHFUNC
 *
 ****************************************************************************/

static inline void mathf_free(int8_t mid)
{
  DEBUGASSERT(mid >= 0 && mid < 3);
  bitmapclear(&g_scudev.mathfuncs, mid);
}

/****************************************************************************
 * Name: seq_alloc
 *
 * Description:
 *   Allocate sequencer
 *
 * Returned Value:
 *   Allocated sequencer ID is returned on success.
 *   -1 is returned on failure.
 *
 ****************************************************************************/

static int8_t seq_alloc(void)
{
  int8_t sid = findzeroandset(&g_scudev.sequencers, 8);
  if (sid < 0)
    {
      return -1;
    }

  /* Wakeup SCU anyway if sequencer allocated successfully */

  seq_inhibitrequest(REQ_SLEEP, true);

  return sid + 2;
}

/****************************************************************************
 * Name: seq_free
 *
 * Description:
 *   Free sequencer
 *
 ****************************************************************************/

static void seq_free(int8_t sid)
{
  DEBUGASSERT(sid >= 2 && sid < 10);
  bitmapclear(&g_scudev.sequencers, sid - 2);
  if (!g_scudev.decimators && !g_scudev.sequencers)
    {
      /* SCU goes sleep when no sequencers running */

      seq_inhibitrequest(REQ_SLEEP, false);
    }
}

/****************************************************************************
 * Name: oneshot_alloc
 *
 * Description:
 *   Allocate oneshot sequencer
 *
 * Returned Value:
 *   Oneshot ID
 *
 ****************************************************************************/

static inline int8_t oneshot_alloc(void)
{
  return findzeroandset(&g_scudev.oneshot, 3);
}

/****************************************************************************
 * Name: oneshot_free
 *
 * Description:
 *   Free oneshot sequencer
 *
 * Input Parameters:
 *   tid - Oneshot ID (0-2)
 *
 ****************************************************************************/

static inline void oneshot_free(int8_t tid)
{
  DEBUGASSERT(tid >= 0 && tid < 3);
  bitmapclear(&g_scudev.oneshot, tid);
}

/****************************************************************************
 * Name: seq_inhibitrequest
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *   Holy terrible SEQ_ACCESS_INHIBIT register's bit field name is
 *   different from actual function.
 *
 * Input Parameters:
 *   req     - REQ_DONE, REQ_SLEEP, REQ_STOP
 *   inhibit - false (release) or true (set)
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static void seq_inhibitrequest(int req, bool set)
{
  uint32_t val;
  val = getreg32(SCU_SEQ_ACCESS_INHIBIT) & ~(1 << req);
  if (set)
    {
      val |= 1 << req;
    }

  putreg32(val, SCU_SEQ_ACCESS_INHIBIT);
}

/****************************************************************************
 * Name: seq_setdecimation
 *
 * Description:
 *   Set decimation configurations
 *
 * Input Parameters:
 *   wid          - FIFO ID (write side)
 *   ratio        - Decimation ratio
 *   leveladj     - Output data level adjustment (2^n)
 *   forcethrough - Bypassing decimation calculation (not same as ratio x1)
 *
 ****************************************************************************/

static void seq_setdecimation(int wid, uint8_t ratio, uint8_t leveladj,
                              uint8_t forcethrough)
{
  uint32_t val;
  uint32_t addr;
  int shift;

  shift = 8 * (wid & 3);
  addr = wid < 4 ? SCU_DECIMATION_PARAM0 : SCU_DECIMATION_PARAM1;

  val = getreg32(addr) & ~(0xff << shift);
  val |= (ratio & 0xf) << shift;          /* N */
  val |= (leveladj & 0x3) << (shift + 4); /* LEVEL_ADJ */
  val |= (forcethrough & 1) << (shift + 7);
  putreg32(val, addr);
}

/****************************************************************************
 * Name: seq_setproperty
 *
 * Description:
 *   Set sequencer property parameters
 *
 * Input Parameters:
 *   sid    - Sequender ID
 *   slave  - Slave select (SPI) or address (I2C)
 *   dest   - Output buffer (1 - 3, oneshot only)
 *   offset - Instruction start offset
 *   len    - Instruction length
 *
 ****************************************************************************/

static void seq_setproperty(int sid, int slave, int out, int offset, int len)
{
  uint32_t val;
  val = slave & 0x3ff;
  val |= (out & 0x3) << 12;
  val |= (offset & 0x7f) << 16;
  val |= (len & 0xf) << 24;
  putreg32(val, SCUSEQ_PROPERTY(sid));
}

/****************************************************************************
 * Name: seq_setinst
 *
 * Description:
 *   Set instruction and its length to sequencer property
 *
 * Input Parameters:
 *   sid    - Sequender ID
 *   offset - Instruction start offset
 *   len    - Instruction length
 *
 ****************************************************************************/

static void seq_setinst(int sid, int offset, int len)
{
  uint32_t val;
  val = getreg32(SCUSEQ_PROPERTY(sid)) & ~((0xf << 24) | (0x7f << 16));
  val |= (offset & 0x7f) << 16;
  val |= (len & 0xf) << 24;
  putreg32(val, SCUSEQ_PROPERTY(sid));
}

/****************************************************************************
 * Name: seq_setbus
 *
 * Description:
 *   Set source bus type for sequencer
 *
 * Input Parameters:
 *   sid     - Sequencer ID
 *   bustype - Bus type (SCU_BUS_*)
 *
 ****************************************************************************/

static void seq_setbus(int sid, int bustype)
{
  uint32_t val;
  uint32_t mask;
  uint32_t bit;

  bit = 1 << sid;
  mask = bit | bit << 10 | bit << 20;
  switch (bustype)
    {
      case SCU_BUS_I2C0:
        {
          bit <<= 10;
        }
        break;
      case SCU_BUS_I2C1:
        {
          bit <<= 20;
        }
        break;
      case SCU_BUS_SPI:
        break;
      case SCU_BUS_LPADC0:
      case SCU_BUS_LPADC1:
      case SCU_BUS_LPADC2:
      case SCU_BUS_LPADC3:
      case SCU_BUS_HPADC0:
      case SCU_BUS_HPADC1:
      default:
        return;
    }

  val = getreg32(SCUSEQ_SRC_SEL) & ~mask;
  putreg32(val | bit, SCUSEQ_SRC_SEL);
}

/****************************************************************************
 * Name: seq_setdataformat
 *
 * Description:
 *   Reading sensor data format
 *
 * Input Parameters:
 *   sid    - Sequencer ID
 *   start  - Sensor data start offset in sequencer picked
 *   bps    - Bytes per sample
 *   swap   - Wwap bytes
 *   elem   - Number of elements in sample
 *
 ****************************************************************************/

static void seq_setdataformat(int sid, int start,
                              int bps, int swap, int elem)
{
  uint32_t val;
  val = start & 0xf;
  val |= (bps & 0xf) << 4;
  val |= (swap & 0x1) << 16;
  val |= (elem & 0x3) << 28;
  putreg32(val, SCUSEQ_OUT_FORMAT(sid));
}

/****************************************************************************
 * Name: seq_setmathwritevecelenum
 *
 * Description:
 *   Set number of elements in sample
 *   (MATH_WRITE_VEC_ELE_NUM field in SEQ_OUT_FORMAT_n)
 *
 * Input Parameters:
 *   sid    - Sequencer ID
 *   elem   - Number of elements in sample
 *
 ****************************************************************************/

static void seq_setmathwritevecelenum(int sid, uint8_t elem)
{
  uint32_t val;

  DEBUGASSERT(elem > 0 && elem <= 3);

  val = getreg32(SCUSEQ_OUT_FORMAT(sid));
  val = (val & ~(0x3 << 28)) | ((elem - 1) & 0x3) << 28;
  putreg32(val, SCUSEQ_OUT_FORMAT(sid));
}

/****************************************************************************
 * Name: seq_setsignconversion
 *
 * Description:
 *   Enable or disable sensor data sign conversion feature for preprocessing.
 *
 * Input Parameters:
 *   sid  - Sequencer ID
 *   sign - 0 to no conversion, 1 to convert unsigned to signed
 *
 ****************************************************************************/

static void seq_setsignconversion(int sid, int sign)
{
  uint32_t val;
  int shift = sid < 2 ? sid * 4 : sid + 6;

  val = getreg32(SCU_UNSIGNED_TO_SIGNED);
  val &= ~(1 << shift);
  putreg32(val | (sign << shift), SCU_UNSIGNED_TO_SIGNED);
}

/****************************************************************************
 * Name: seq_setstartmode
 *
 * Description:
 *   Set sequencer start mode
 *
 * Input Parameters:
 *   sid  - Sequencer ID
 *   mode - Start mode
 *
 ****************************************************************************/

static void seq_setstartmode(int sid, int mode)
{
  uint32_t val;
  uint32_t shift;

  shift = sid * 2;
  val = getreg32(SCU_START_MODE1) & ~(3 << shift);
  putreg32(val | (mode << shift), SCU_START_MODE1);
}

/****************************************************************************
 * Name: seq_setstartinterval
 *
 * Description:
 *   Set sequencer interval
 *
 * Input Parameters:
 *   sid      - Sequencer ID
 *   interval - Interval
 *
 ****************************************************************************/

static void seq_setstartinterval(int sid, int interval)
{
  uint32_t addr = SCU_START_INTERVAL3_0 + (sid & 0xc);
  uint32_t val;
  uint32_t shift;

  shift = (sid & 3) * 8;
  val = getreg32(addr) & ~(0xf << shift);
  putreg32(val | (interval << shift), addr);
}

/****************************************************************************
 * Name: seq_setstartphase
 *
 * Description:
 *   Set sequencer start phase.
 *   This function always set to zero right now. I don't know what this
 *   parameter affects for.
 *
 * Input Parameters:
 *   sid   - Sequencer ID
 *   phase - Phase value
 *
 ****************************************************************************/

static void seq_setstartphase(int sid, int phase)
{
  uint32_t addr = SCU_START_PHASE1_0 + ((sid & 0xe) << 1);
  uint32_t val;
  uint32_t shift;

  shift = (sid & 1) * 16;
  val = getreg32(addr) & ~(0x1ff << shift);
  putreg32(val | (phase << shift), addr);
}

/****************************************************************************
 * Name: seq_startseq
 *
 * Description:
 *   Start sequencer
 *
 * Input Parameters:
 *   sid - Sequencer ID
 *
 ****************************************************************************/

static void seq_startseq(int sid)
{
  uint32_t val;
  irqstate_t flags;
  flags = enter_critical_section();
  val = getreg32(SCU_START_MODE0);
  putreg32(val | (1 << sid), SCU_START_MODE0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: seq_stopseq
 *
 * Description:
 *   Stop sequencer
 *
 * Input Parameters:
 *   sid - Sequencer ID
 *
 ****************************************************************************/

static void seq_stopseq(int sid)
{
  uint32_t val;
  irqstate_t flags;
  flags = enter_critical_section();
  val = getreg32(SCU_START_MODE0);
  putreg32(val & ~(1 << sid), SCU_START_MODE0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: seq_oneshot
 *
 * Description:
 *   Perform one shot feature, this function can be use for any register
 *   access to connected sensors. (e.g. configure power mode)
 *   This function use empty sequencer temporarily, other than allocated
 *   sequencer to the sensor device driver. Thus, this function fails when
 *   all of normal sequencers are allocated.
 *
 * Input Parameters:
 *   bustype  - One of SCU_BUS_SPI, SCU_BUS_I2C0 and SCU_BUS_I2C1
 *   slave    - Slave select (SPI) or address (I2C)
 *   inst     - Pointer to series of read/write instruction array
 *   nr_insts - Number of instructions
 *   buffer   - Receive data buffer
 *   len      - Length of receive data buffer
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int seq_oneshot(int bustype, int slave, uint16_t *inst,
                       uint32_t nr_insts, uint8_t *buffer, int len)
{
  struct cxd56_scudev_s *priv = &g_scudev;
  irqstate_t flags;
  int sid;
  int tid;
  int istart;
  int i;
  int ret = OK;
  uint16_t lastinst;

  ASSERT(bustype == SCU_BUS_SPI || bustype == SCU_BUS_I2C0 ||
         bustype == SCU_BUS_I2C1);

  if (nr_insts > INSTRUCTION_PER_SEQ)
    {
      return -EINVAL;
    }

  /* If buffer is non NULL, check receive length is valid */

  if (buffer && (len <= 0 || len > 16))
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  sid = seq_alloc();
  if (sid < 0)
    {
      leave_critical_section(flags);
      return -ENOENT;
    }

  tid = oneshot_alloc();
  if (tid < 0)
    {
      seq_free(sid);
      leave_critical_section(flags);
      return -ENOENT;
    }

  leave_critical_section(flags);

  /* Remake last instruction, if needed. */

  lastinst = seq_remakeinstruction(bustype, inst[nr_insts - 1]);

  /* Copy sequencer instruction */

  istart = sid * INSTRUCTION_PER_SEQ;
  for (i = 0; i < nr_insts - 1; i++)
    {
      putreg16(inst[i], SCUSEQ_INSTRUCTION(istart + i));
    }

  putreg16(lastinst, SCUSEQ_INSTRUCTION(istart + nr_insts - 1));

  /* Setup sequencer as oneshot mode
   * Oneshot mode is special function, so can not be use to MATHFUNC,
   * sign conversion and offset/gain adjustment.
   */

  seq_setbus(sid, bustype);
  seq_setproperty(sid, slave, tid + 1, istart, nr_insts - 1);
  seq_setsignconversion(sid, 0);
  seq_offsetgainenable(sid, false);
  seq_setdataformat(sid, 0, buffer ? len - 1 : 0, 0, 0);

  /* Mode set "Stop" */

  seq_setstartmode(sid, 0);

  /* Set fixed value interval (0xa) for one shot sequencer */

  seq_setstartinterval(sid, 0xa);

  /* Enable interrupt for one shot sequencer */

  putreg32(1 << (tid + 24), SCU_INT_ENABLE_MAIN);

  scuinfo("Sequencer start.\n");
  priv->oneshoterr[tid] = 0;

  /* Start sequencer as one shot mode */

  seq_startseq(sid);
  putreg32(1 << sid, SCU_SINGLE_EXE);

  /* Wait for one shot is done */

  nxsem_wait_uninterruptible(&priv->oneshotwait[tid]);

  /* Disable interrupt for one shot sequencer */

  putreg32(1 << (tid + 24), SCU_INT_DISABLE_MAIN);

  scuinfo("Sequencer done.\n");

  if (priv->oneshoterr[tid] < 0)
    {
      ret = ERROR;
    }
  else
    {
      /* Copy sequencer output results to user buffer.
       * XXX: Sequencer output RAM offset is differ from document.
       */

      memcpy(buffer, (void *)(SCUSEQ_RAM_OUT_DATA0 + (0x10 * tid)), len);
    }

  /* Stop sequencer */

  seq_stopseq(sid);

  /* Destroy oneshot resources */

  flags = enter_critical_section();
  oneshot_free(tid);
  seq_free(sid);
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: seq_enableoffsetgain
 *
 * Description:
 *   Enable offset/gain preprocessing
 *
 ****************************************************************************/

static void seq_offsetgainenable(int sid, bool enable)
{
  uint32_t val;
  int n = sid < 2 ? sid * 4 : sid + 6;

  val = getreg32(SCU_OFST_GAIN_EN);
  if (enable)
    {
      val |= 1 << n;
    }
  else
    {
      val &= ~(1 << n);
    }

  putreg32(val, SCU_OFST_GAIN_EN);
}

/****************************************************************************
 * Name: seq_start
 *
 * Description:
 *   Start sequencer. This function is for SCUIOC_START.
 *
 * Input Parameters:
 *   seq    - An instance of sequencer
 *   fifoid - FIFO ID (0-2, decimator only)
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int seq_start(struct seq_s *seq, int fifoid)
{
  struct scufifo_s *fifo;
  uint32_t interval;

  DEBUGASSERT(seq);
  DEBUGASSERT(fifoid >= 0 && fifoid < 3);

  if (seq->type & SEQ_TYPE_DECI)
    {
      struct decimator_s *deci = (struct decimator_s *)seq;
      struct decimation_fifo_s *dfifo = &deci->dfifo[fifoid];

      /* Avoid confilict for DECIMATION_PARAM register access with SCU */

      if (seq_isactive(seq))
        {
          seq_sync(&deci->seq, REQ_STOP);
        }

      seq_setdecimation(dfifo->fifo->wid, dfifo->ratio, dfifo->leveladj,
                        dfifo->forcethrough);
      fifo = dfifo->fifo;

      interval = dfifo->ratio + seq->rate;

      /* Prevent SCU freeze by frequently REQ_STOP */

      if (seq_isactive(seq))
        {
          up_udelay(100);
        }
    }
  else
    {
      fifo = seq->fifo;
      interval = seq->rate;
    }

  if (!fifo)
    {
      return -EPERM;
    }

  if (!seq_isactive(seq))
    {
      if (!(seq->bustype & 0x10))
        {
          /* Calculate timestamp interval prediv * (2 ^ interval) */

          fifo->interval = CONFIG_CXD56_SCU_PREDIV * (1 << interval);
          fifo->adjust = 0;

          /* Go sequencer */

          seq_setstartmode(seq->id, 1);
          seq_setstartinterval(seq->id, seq->rate);
          seq_setstartphase(seq->id, 1);
          seq_startseq(seq->id);
        }
      else
        {
          uint32_t mask;
          uint32_t val;

          /* Calculate timestamp interval for ADC */

          cxd56_adc_getinterval(seq->bustype,
                                &fifo->interval,
                                &fifo->adjust);

          /* Enable ADC */

          mask = 0x1 << (24 + (seq->bustype & 0x7));
          val = getreg32(SCUSEQ_ADC_PROPERTY) & ~mask;
          val |= (0x1 << (24 + (seq->bustype & 0x7)));
          putreg32(val, SCUSEQ_ADC_PROPERTY);
        }
    }

  seq_fifosetactive(seq, fifoid);

  return 0;
}

/****************************************************************************
 * Name: seq_stop
 *
 * Description:
 *   Stop sequencer (external API)
 *
 ****************************************************************************/

static int seq_stop(struct seq_s *seq, int fifoid)
{
  struct scufifo_s *fifo;
  uint32_t val;

  DEBUGASSERT(seq);
  DEBUGASSERT(fifoid >= 0 && fifoid < 3);

  fifo = seq_getfifo(seq, fifoid);

  if (fifo == NULL)
    {
      scuerr("missing FIFO\n");
      return -EPERM;
    }

  if (!seq_isactive(seq))
    {
      return OK;
    }

  val = getreg32(SCUFIFO_R_CTRL1(fifo->rid));
  putreg32(val & ~(1 << 24), SCUFIFO_R_CTRL1(fifo->rid));

  if (seq->type & SEQ_TYPE_DECI)
    {
      seq_sync(seq, REQ_STOP);

      /* Set ratio to 15 to disable decimator */

      seq_setdecimation(fifo->wid, DECIMATION_OFF, 0, 0);

      /* Prevent SCU freeze by frequently REQ_STOP */

      up_udelay(100);
    }

  seq_fifosetinactive(seq, fifoid);

  /* If all of FIFOs are stopped, then stop sequencer. */

  if (!seq_isactive(seq))
    {
      if (!(seq->bustype & 0x10))
        {
          seq_setstartmode(seq->id, 0);
          seq_stopseq(seq->id);

          /* Wait for stop done */

          seq_sync(seq, REQ_DONE);
        }
      else
        {
          /* Disable ADC */

          uint32_t mask = 0x1 << (24 + (seq->bustype & 0x7));
          val = getreg32(SCUSEQ_ADC_PROPERTY) & ~mask;
          putreg32(val, SCUSEQ_ADC_PROPERTY);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mathf_enable
 *
 * Description:
 *   Enable MATHFUNC to specified FIFO
 *
 * Input Parameters:
 *   mid - MATHFUNC ID
 *   wid - FIFO ID (Write side)
 *
 ****************************************************************************/

static void mathf_enable(int8_t mid, uint8_t wid)
{
  uint32_t val;

  val = getreg32(SCU_MATHFUNC_SEL);
  val &= ~(0xf << (mid * 8));
  val |= 1 << ((mid * 8) + 4);
  putreg32(val | (wid << (mid * 8)), SCU_MATHFUNC_SEL);
  putreg32(1 << mid, SCU_MATHFUNC_CLR);
}

/****************************************************************************
 * Name: mathf_disable
 *
 * Description:
 *   Disable MATHFUNC
 *
 * Input Parameters:
 *   mid - MATHFUNC ID
 *
 ****************************************************************************/

static void mathf_disable(int8_t mid)
{
  uint32_t val;
  val = getreg32(SCU_MATHFUNC_SEL);
  val &= ~(1 << ((mid * 8) + 4));
  putreg32(val, SCU_MATHFUNC_SEL);
}

/****************************************************************************
 * Name: mathf_set_coeff
 *
 * Description:
 *   Set 1 coefficiency value
 *
 ****************************************************************************/

static inline void mathf_set_coeff(uint32_t caddr, struct iir_coeff_s *c)
{
  putreg32(c->h, caddr);
  putreg32((c->l & 0x3) << 30, caddr + 4);
}

/****************************************************************************
 * Name: mathf_setiirfilter
 *
 * Description:
 *   Set series of coefficiencies for 2 IIR filters
 *
 * Input Parameters:
 *   mid    - MATHFUNC ID
 *   n      - Filter ID
 *   filter - IIR filter setting
 *
 ****************************************************************************/

static void mathf_setiirfilter(int mid, int n,
                               struct iir_filter_s *filter)
{
  const struct coeff_addr_s *caddr;

  caddr = g_caddrs[mid];

  putreg32((filter->ishift & 0x7) | ((filter->oshift & 0x7) << 8),
            caddr[0].s);

  mathf_set_coeff(caddr[n].c0, &filter->coeff[0]);
  mathf_set_coeff(caddr[n].c1, &filter->coeff[1]);
  mathf_set_coeff(caddr[n].c2, &filter->coeff[2]);
  mathf_set_coeff(caddr[n].c3, &filter->coeff[3]);
  mathf_set_coeff(caddr[n].c4, &filter->coeff[4]);
}

/****************************************************************************
 * Name: seq_setsamplingrate
 *
 * Description:
 *   Set sequencer sampling rate
 *
 * Input Parameters:
 *   seq          - An instance of sequencer
 *   samplingrate - Sampling rate. Sampling rate is based on
 *                  32768 Hz / predivider.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int seq_setsamplingrate(struct seq_s *seq, uint8_t samplingrate)
{
  DEBUGASSERT(seq);

  if (samplingrate > 9)
    {
      return -EINVAL;
    }

  seq->rate = samplingrate;

  return OK;
}

/****************************************************************************
 * Name: seq_sync
 *
 * Description:
 *   Send specific request to SCU firmware
 *
 * Input Parameters:
 *   seq - An instance of sequencer
 *   req - Request code for SCU firmware (REQ_*)
 *
 ****************************************************************************/

static void seq_sync(struct seq_s *seq, int req)
{
  struct cxd56_scudev_s *priv = &g_scudev;

  nxmutex_lock(&priv->synclock);

  /* Save current request */

  priv->currentreq = req;

  putreg32(1 << 27, SCU_INT_CLEAR_MAIN);
  putreg32(1 << 27, SCU_INT_ENABLE_MAIN);

  putreg32(seq->id & 0xf, SCUSEQ_SYNCRO_CPU2ISOP);

  /* Send request to SCU */

  seq_inhibitrequest(req, true);

  /* Wait for interrupt from SCU firmware */

  nxsem_wait_uninterruptible(&priv->syncwait);

  priv->currentreq = 0;

  nxmutex_unlock(&priv->synclock);
}

/****************************************************************************
 * Name: seq_handlefifintr
 *
 * Description:
 *   Handle FIFO overrun error
 *
 ****************************************************************************/

static void seq_handlefifointr(struct cxd56_scudev_s *priv,
                               uint32_t intr)
{
  uint32_t bit;
  int i;
  struct wm_notify_s *notify;
  union sigval value;

  if ((intr & 0x007ffe00) == 0)
    {
      return;
    }

  /* Each FIFO almost full interrupt cause are starts with 9 */

  for (i = 0, bit = 1 << 9; i < 14; i++, bit <<= 1)
    {
      if (intr & bit)
        {
          /* Clear interrupt */

          putreg32(bit, SCU_INT_CLEAR_MAIN);

          notify = &priv->wm[i];

          if (notify->ts)
            {
              seq_gettimestamp(notify->fifo, notify->ts);
            }

          DEBUGASSERT(notify->pid != 0);

          value.sival_ptr = notify->ts;
          nxsig_queue(notify->pid, notify->signo, value);
        }
    }
}

/****************************************************************************
 * Name: seq_handlemathfintr
 *
 * Description:
 *   Handle MATHFUNC rise and fall event interrupts
 *
 ****************************************************************************/

static void seq_handlemathfintr(struct cxd56_scudev_s *priv,
                                uint32_t intr)
{
  int i;
  uint32_t bit;
  uint32_t rise;
  uint32_t fall;
  struct ev_notify_s *notify;
  int detected = 0;

  rise = (intr >> 6) & 0x7;
  fall = (intr >> 28) & 0x7;

  if (rise == 0 && fall == 0)
    {
      return;
    }

  for (i = 0, bit = 1; i < 3; i++, bit <<= 1)
    {
      notify = &priv->event[i];

      /* Detect rise event */

      if (rise & bit)
        {
          putreg32(bit << 6, SCU_INT_CLEAR_MAIN);

          /* Get rise event occurred timestamp */

          if (notify->arg)
            {
              notify->arg->ts.sec =
                getreg32(SCU_EVENT_TIMESTAMP0_R_MSB + (i * 8));
              notify->arg->ts.tick =
                getreg32(SCU_EVENT_TIMESTAMP0_R_LSB + (i * 8));
              notify->arg->type = SCU_EV_RISE;
            }

          detected = 1;
        }

      /* Detect fall event */

      if (fall & bit)
        {
          putreg32(bit << 28, SCU_INT_CLEAR_MAIN);

          /* Get fall event occurred timestamp */

          if (notify->arg)
            {
              notify->arg->ts.sec =
                getreg32(SCU_EVENT_TIMESTAMP0_F_MSB + (i * 8));
              notify->arg->ts.tick =
                getreg32(SCU_EVENT_TIMESTAMP0_F_LSB + (i * 8));
              notify->arg->type = SCU_EV_FALL;
            }

          detected = 1;
        }

      if (detected)
        {
          union sigval value;

          DEBUGASSERT(notify->pid != 0);

          value.sival_ptr = notify->arg;
          nxsig_queue(notify->pid, notify->signo, value);
          detected = 0;
        }
    }
}

/****************************************************************************
 * Name: seq_handleoneshot
 *
 * Description:
 *   Handle one shot sequencer done interrupt
 *
 ****************************************************************************/

static void seq_handleoneshot(struct cxd56_scudev_s *priv, uint32_t intr)
{
  uint32_t bit;
  int i;

  if ((intr & 0x07000000) == 0)
    {
      return;
    }

  for (i = 0, bit = 1 << 24; i < 3; i++, bit <<= 1)
    {
      if (intr & bit)
        {
          putreg32(bit, SCU_INT_CLEAR_MAIN);

          nxsem_post(&priv->oneshotwait[i]);
        }
    }
}

/****************************************************************************
 * Name: seq_handleisopdoneintr
 *
 * Description:
 *   Handle interrupts response against request for SCU firmware
 *
 ****************************************************************************/

static void seq_handleisopdoneintr(struct cxd56_scudev_s *priv,
                                   uint32_t intr)
{
  /* Detect ISOP3 as done or stop. */

  if (intr & (1 << 27))
    {
      DEBUGASSERT(priv->currentreq != 0);

      seq_inhibitrequest(priv->currentreq, false);

      putreg32(1 << 27, SCU_INT_DISABLE_MAIN);
      putreg32(1 << 27, SCU_INT_CLEAR_MAIN);

      nxsem_post(&priv->syncwait);
    }
}

/****************************************************************************
 * Name: seq_scuirqhandler
 *
 * Description:
 *   Root interrupt handler for many SCU interrupts
 *
 ****************************************************************************/

static int seq_scuirqhandler(int irq, void *context, void *arg)
{
  struct cxd56_scudev_s *priv = arg;
  uint32_t intr;
  uint32_t ierr0;
  uint32_t ierr1;
  uint32_t ierr2;
  uint32_t out;
  int tid;
  int i;

  intr = getreg32(SCU_INT_MASKED_STT_MAIN);
  ierr0 = getreg32(SCU_INT_MASKED_STT_ERR_0);
  ierr1 = getreg32(SCU_INT_MASKED_STT_ERR_1);
  ierr2 = getreg32(SCU_INT_MASKED_STT_ERR_2);

  seq_handlefifointr(priv, intr);

  seq_handlemathfintr(priv, intr);

  seq_handleoneshot(priv, intr);

  seq_handleisopdoneintr(priv, intr);

  /* Detect all FIFO overrun errors */

  if (ierr0 != 0)
    {
      scuerr("err0: %08" PRIx32 "\n", ierr0);
      ierr0 = (ierr0 >> 9) & 0x3fff;
      for (i = 0; i < 14; i++)
        {
          if ((ierr0 & (0x1 << i)) != 0)
            {
              /* Clear FIFO overrun error */

              putreg32(getreg32(SCUFIFO_R_CTRL1(i)) | SCUFIFO_OVERRUNCLR,
                       SCUFIFO_R_CTRL1(i));
            }
        }
    }

  /**
   * Detect all FIFO underrun errors
   * This error may not happened because check reading bytes at seq_read().
   * Thus, it is a program error when ERR1 detected.
   */

  if (ierr1 != 0)
    {
      scuerr("err1: %08" PRIx32 "\n", ierr1);
      ierr1 = (ierr1 >> 9) & 0x3fff;
      for (i = 0; i < 14; i++)
        {
          if ((ierr1 & (0x1 << i)) != 0)
            {
              /* Clear FIFO underrun error */

              putreg32(getreg32(SCUFIFO_R_CTRL1(i)) | SCUFIFO_UNDERRUNCLR,
                       SCUFIFO_R_CTRL1(i));
            }
        }
    }

  /* Detect sequencer error */

  if (ierr2 != 0)
    {
      scuerr("err2: %08" PRIx32 "\n", ierr2);

      for (i = 0; i < 10; i++)
        {
          if (ierr2 & (0x00010001 << i))
            {
              seq_stopseq(i);

              /* Get sequencer output selector */

              out = (getreg32(SCUSEQ_PROPERTY(i)) >> 12) & 0x3;

              if (0 < out)
                {
                  /* Set error code to oneshot sequencer id */

                  tid = out - 1;

                  priv->oneshoterr[tid] = -EIO;
                  nxsem_post(&priv->oneshotwait[tid]);
                }
            }
        }

      putreg32(ierr2, SCU_INT_CLEAR_ERR_2);
    }

  return 0;
}

/****************************************************************************
 * Name: seq_new
 *
 * Description:
 *   Create new sequencer instance
 *
 * Returned Value:
 *   Created sequencer instance or NULL
 *
 ****************************************************************************/

static struct seq_s *seq_new(void)
{
  struct seq_s *seq;
  int sid;
  irqstate_t flags;

  flags = enter_critical_section();
  sid = seq_alloc();
  if (sid < 0)
    {
      /* Sequencer is full */

      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  seq = (struct seq_s *)kmm_malloc(sizeof(struct seq_s));
  if (!seq)
    {
      seq_free(sid);
      return NULL;
    }

  memset(seq, 0, sizeof(struct seq_s));

  seq->id = sid;

  return seq;
}

/****************************************************************************
 * Name: deci_new
 *
 * Description:
 *   Create new decimator instance
 *
 * Returned Value:
 *   Created decimator instance or NULL
 *
 ****************************************************************************/

static struct seq_s *deci_new(void)
{
  struct decimator_s *deci;
  int sid;
  irqstate_t flags;

  flags = enter_critical_section();
  sid = deci_alloc();
  if (sid < 0)
    {
      /* Sequencer is full */

      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  deci = (struct decimator_s *)kmm_malloc(sizeof(struct decimator_s));
  if (!deci)
    {
      deci_free(sid);
      return NULL;
    }

  memset(deci, 0, sizeof(struct decimator_s));

  deci->seq.id = sid;

  return &deci->seq;
}

/****************************************************************************
 * Name: seq_fifoinit
 *
 * Description:
 *   Initialize sequencer output FIFO
 *
 * Input Parameters:
 *   seq    - Sequencer instance
 *   fifoid - FIFO ID (decimator only)
 *   fsize  - Allocate FIFO memory size
 *
 ****************************************************************************/

static int seq_fifoinit(struct seq_s *seq, int fifoid, uint16_t fsize)
{
  struct scufifo_s *fifo;
  struct decimator_s *deci = (struct decimator_s *)seq;
  int wid;
  int rid;
  uint32_t val;
  uint32_t mask;
  uint16_t start;
  uint16_t samples;

  DEBUGASSERT(seq);
  DEBUGASSERT(fifoid >= 0 && fifoid < 3);

  if (seq->type & SEQ_TYPE_DECI)
    {
      if (deci->dfifo[fifoid].fifo)
        {
          return OK;
        }
    }
  else
    {
      if (seq->fifo)
        {
          /* FIFO already initialized */

          return OK;
        }
    }

  fifo = (struct scufifo_s *)kmm_malloc(sizeof(struct scufifo_s));
  if (!fifo)
    {
      return -ENOMEM;
    }

  memset(fifo, 0, sizeof(struct scufifo_s));

  /* Setup FIFO, normal FIFO wid and rid are based on 8 and 4 respectively. */

  start = scufifo_memalloc(fsize);
  if (start == FIFOMEM_INVALID)
    {
      kmm_free(fifo);
      return -ENOMEM;
    }

  /**
   * FIFO IDs (* is unavailable)
   *  D = Decimation FIFO
   *  N = Normal FIFO
   *
   * sid   wid name   rid name
   *   0     0 D0_W0*
   *         1 D0_W1    0 D0_R1
   *         2 D0_W2    1 D0_R2
   *         3 D0_W3   12 D0_R3
   *   1     4 D1_W0*
   *         5 D1_W1    2 D1_R1
   *         6 D1_W2    3 D1_R2
   *         7 D1_W3   13 D1_R3
   *
   *   2     8 N0_W     4 N0_R1
   *   3     9 N1_W     5 N0_R1
   *   4    10 N2_W     6 N0_R1
   *   5    11 N3_W     7 N0_R1
   *   6    12 N4_W     8 N0_R1
   *   7    13 N5_W     9 N0_R1
   *   8    14 N6_W    10 N0_R1
   *   9    15 N7_W    11 N0_R1
   */

  if (seq->type & SEQ_TYPE_DECI)
    {
      wid = fifoid + 1 + (seq->id * 4);
      if (fifoid < 2)
        {
          rid = fifoid + (seq->id * 2);
        }
      else
        {
          rid = seq->id + 12;
        }

      if (!seq_isactive(seq))
        {
          /* Additionally, reset first stage of decimation logic
           * when other decimator not used.
           */

          putreg32(1 << (seq->id * 4), SCU_DEC_CLR);
        }

      /* Reset decimator */

      putreg32(1 << wid, SCU_DEC_CLR);
    }
  else
    {
      wid = seq->id + 6;
      rid = seq->id + 2;
    }

  /* Set FIFO start offset and its size */

  DEBUGASSERT(seq->sample);
  samples = fsize / seq->sample;
  putreg32((uint32_t)start << 16 | (samples - 1), SCUFIFO_W_CTRL0(wid));

  /* Set 1 sample size and FIFO reset */

  val = seq->sample - 1;
  val |= (1 << 16);
  putreg32(val, SCUFIFO_W_CTRL1(wid));

  /* Set watermark */

  putreg32(0xffff, SCUFIFO_R_CTRL0(rid));

  /* Perform enable FIFO outlet, reset and clear error */

  val = (1 << 24) | (1 << 16) | (1 << 8) | 3;
  putreg32(val, SCUFIFO_R_CTRL1(rid));

  /* Clear write event control */

  putreg32(0, SCUSEQ_FIFOWREVNTCTRL(wid));

  /* Save instance data */

  fifo->start = start;
  fifo->size = fsize;
  fifo->wid = wid;
  fifo->rid = rid;
  fifo->mid = -1;

#ifdef CONFIG_CXD56_UDMAC
  /* Allocate DMA for read sensing data */

  fifo->dma = cxd56_udmachannel();

  /* Initialize DMA done wait semaphore */

  nxsem_init(&fifo->dmawait, 0, 0);

  fifo->dmaresult = -1;
#endif

  if (seq->type & SEQ_TYPE_DECI)
    {
      struct decimation_fifo_s *dec = &deci->dfifo[fifoid];
      dec->fifo = fifo;
      dec->ratio = 0;
      dec->leveladj = 0;
      dec->forcethrough = 1;
    }
  else
    {
      seq->fifo = fifo;
    }

  if (seq->bustype & 0x10)
    {
      /* Set wid to ADC_PROPERTY */

      mask = 0xf << (4 * (seq->bustype & 0x7));
      val = getreg32(SCUSEQ_ADC_PROPERTY) & ~mask;
      val |= (wid << (4 * (seq->bustype & 0x7)));
      putreg32(val, SCUSEQ_ADC_PROPERTY);
    }

  return OK;
}

/****************************************************************************
 * Name: seq_fifofree
 *
 * Description:
 *   Free sequencer output FIFO
 *
 * Input Parameters:
 *   fifo - An instance of FIFO
 *
 ****************************************************************************/

static void seq_fifofree(struct scufifo_s *fifo)
{
  int wid;
  int rid;
  int mid;

  if (!fifo)
    {
      return;
    }

#ifdef CONFIG_CXD56_UDMAC
  /* Free DMA */

  cxd56_udmafree(fifo->dma);
#endif

  wid = fifo->wid;
  rid = fifo->rid;
  mid = fifo->mid;

  if (mid >= 0 && mid < 3)
    {
      putreg32(3, SCUSEQ_FIFOWREVNTCTRL(wid));
      putreg32(0, SCU_EVENT_INTR_ENABLE(mid));
      putreg32(0x9 << mid, SCU_FIFO_WRITE_CTRL);
      putreg32(1 << mid, SCU_EVENT_STT);

      putreg32((1 << (wid + 28)) | (1 << (wid + 6)), SCU_INT_DISABLE_MAIN);

      mathf_disable(mid);
      mathf_free(mid);
    }

  /* Disable watermark interrupt anyway */

  putreg32(1 << (rid + 9), SCU_INT_DISABLE_MAIN);

  /* Make sure want to be FIFO disabled */

  putreg32(0, SCUFIFO_R_CTRL1(rid));

  scufifo_memfree(fifo->start);

#ifdef CONFIG_CXD56_UDMAC
  nxsem_destroy(&fifo->dmawait);
#endif

  kmm_free(fifo);
}

/****************************************************************************
 * Name: seq_getfifo
 *
 * Description:
 *   Utility for ease of obtaining FIFO instans from sequencer.
 *
 * Input Parameters:
 *   seq    - An instance of sequencer
 *   fifoid - FIFO ID (decimator only)
 *
 ****************************************************************************/

static inline struct scufifo_s *seq_getfifo(struct seq_s *seq,
                                            int fifoid)
{
  DEBUGASSERT(fifoid >= 0 && fifoid < 3);

  if (seq->type & SEQ_TYPE_DECI)
    {
      struct decimator_s *deci = (struct decimator_s *)seq;
      return deci->dfifo[fifoid].fifo;
    }
  else
    {
      return seq->fifo;
    }
}

/****************************************************************************
 * Name: seq_setadjustment
 *
 * Description:
 *   Set parameters for offset/gain adjustment, preprocessing of sequencer.
 *
 * Input Parameters:
 *   seq - An instance of sequencer
 *   adj - offset/gain adjustment parameter by 3 axis
 *
 ****************************************************************************/

static int seq_setadjustment(struct seq_s *seq,
                             struct adjust_xyz_s *adj)
{
  int sid;
  uint32_t val;

  DEBUGASSERT(seq);

  /* offset/gain parameters can be set when sequencer not running */

  if (seq_isactive(seq))
    {
      return -EBUSY;
    }

  sid = seq->id;

  /* Set offset/gain adjustment parameters for each 3 axis data */

  val = adj->x.offset | ((uint32_t)adj->x.gain << 16);
  putreg32(val, SCUSEQ_MATH_PROC_OFST_GAIN_X(sid));

  val = adj->y.offset | ((uint32_t)adj->y.gain << 16);
  putreg32(val, SCUSEQ_MATH_PROC_OFST_GAIN_Y(sid));

  val = adj->z.offset | ((uint32_t)adj->z.gain << 16);
  putreg32(val, SCUSEQ_MATH_PROC_OFST_GAIN_Z(sid));

  return OK;
}

/****************************************************************************
 * Name: seq_setfilter
 *
 * Description:
 *   Set MATHFUNC IIR filter feature
 *
 * Input Parameters:
 *   fifo - An instance of FIFO
 *   pos  - Where to IIR filter inserted
 *   iir  - IIR filter coefficiencies
 *
 ****************************************************************************/

static int seq_setfilter(struct scufifo_s *fifo, int pos,
                         struct iir_filter_s iir[2])
{
  int mid;

  DEBUGASSERT(fifo);

  if (fifo->mid < 0)
    {
      mid = mathf_alloc();
      if (mid < 0)
        {
          return -ENOENT;
        }

      fifo->mid = mid;
      mathf_enable(mid, fifo->wid);
    }
  else
    {
      mid = fifo->mid;
    }

  putreg32(pos, SCU_MATHFUNC_POS(mid));

  mathf_setiirfilter(mid, 0, &iir[0]);
  mathf_setiirfilter(mid, 1, &iir[1]);

  return OK;
}

/****************************************************************************
 * Name: seq_seteventnotifier
 *
 * Description:
 *   Set event notifier
 *
 * Input Parameters:
 *   fifo - An instance of FIFO
 *   ev   - Event notify setting
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int seq_seteventnotifier(struct scufifo_s *fifo,
                                struct scuev_notify_s *ev)
{
  struct cxd56_scudev_s *priv = &g_scudev;
  uint32_t val;
  uint32_t thresh;
  uint32_t count0;
  uint32_t count1;
  uint32_t delaysample;
  int riseint;
  int fallint;
  int mid;
  irqstate_t flags;

  DEBUGASSERT(fifo && ev);

  /* Math Function must be assigned first. */

  if (fifo->mid < 0)
    {
      return -ENOENT;
    }

  mid = fifo->mid;

  /* Save signal number and target PID */

  flags = enter_critical_section();
  priv->event[mid].signo = ev->signo;
  priv->event[mid].pid = getpid();
  priv->event[mid].arg = ev->arg;
  priv->event[mid].fifo = fifo;
  leave_critical_section(flags);

  thresh = count0 = count1 = delaysample = 0;
  riseint = fallint = 0;

  thresh = ev->rise.threshold;
  count0 = ev->rise.count0;
  count1 = ev->rise.count1;
  delaysample = ev->rise.delaysamples;

  thresh |= ev->fall.threshold << 16;
  count0 |= ev->fall.count0 << 16;
  count1 |= ev->fall.count1 << 16;
  delaysample |= ev->fall.delaysamples << 16;

  if (ev->ctrl & SCU_EV_RISE_EN)
    {
      riseint = 1;
    }

  if (ev->ctrl & SCU_EV_FALL_EN)
    {
      fallint = 1;
    }

  putreg32(thresh, SCU_EVENT_PARAM_THRESH(mid));
  putreg32(count0, SCU_EVENT_PARAM_COUNT0(mid));
  putreg32(count1, SCU_EVENT_PARAM_COUNT1(mid));
  putreg32(delaysample, SCU_EVENT_PARAM_DELAY_SAMPLE(mid));

  /* Enable interrupt */

  putreg32(fallint << 8 | riseint, SCU_EVENT_INTR_ENABLE(mid));

  val = (ev->ctrl & SCU_EV_OUTMASK) >> SCU_EV_OUTSHIFT;
  val |= (mid & 0x3) << 4;
  val |= (ev->ctrl & SCU_EV_WRITESAMPLEMASK) << 8;
  val |= ev->ctrl & SCU_EV_OUT8BITS ? 1 << 24 : 0;

  putreg32(val, SCUSEQ_FIFOWREVNTCTRL(fifo->wid));

  val = riseint << (mid + 6);
  val |= fallint << (mid + 28);
  putreg32(val, SCU_INT_ENABLE_MAIN);

  putreg32(1 << mid, SCU_EVENT_STT);

  return OK;
}

/****************************************************************************
 * Name: seq_setwatermark
 *
 * Description:
 *   Set watermark value for specified FIFO
 *
 ****************************************************************************/

static int seq_setwatermark(struct seq_s *seq, int fifoid,
                            struct scufifo_wm_s *wm)
{
  struct cxd56_scudev_s *priv = &g_scudev;
  struct scufifo_s *fifo = seq_getfifo(seq, fifoid);
  struct wm_notify_s *notify;
  int rid;
  irqstate_t flags;

  DEBUGASSERT(fifo && wm);

  if (wm->watermark == 0)
    {
      return -EINVAL;
    }

  rid = fifo->rid;

  notify = &priv->wm[rid];

  flags = enter_critical_section();
  notify->signo = wm->signo;
  notify->pid = getpid();
  notify->ts = wm->ts;
  notify->fifo = fifo;

  /* Set watermark */

  putreg32(wm->watermark, SCUFIFO_R_CTRL0(rid));

  /* Enable FIFO almost full interrupt */

  putreg32(1 << (rid + 9), SCU_INT_ENABLE_MAIN);

  leave_critical_section(flags);

  scuinfo("watermark = %04x, PID = %d, signo = %d\n", wm->watermark,
          notify->pid, notify->signo);

  return OK;
}

/****************************************************************************
 * Name: convert_firsttimestamp
 *
 * Description:
 *
 ****************************************************************************/

static void convert_firsttimestamp(struct scutimestamp_s *tm,
                                   uint16_t interval,
                                   uint16_t sample, uint16_t adjust)
{
  uint32_t delta;
  uint32_t tick;
  uint16_t mod;

  if (sample == 0 || interval == 0)
    return;

  mod = sample & 0x7;

  if (adjust && mod)
    {
      delta = interval * (sample + (8 - mod));
      if (adjust == 1)
        {
          if (mod <= 2)
            {
              delta -= 2;
            }
          else if (mod <= 5)
            {
              delta -= 1;
            }
        }
    }
  else
    {
      delta = interval * sample;
    }

  tick = tm->tick;
  if (tick < delta)
    {
      tick += 0x80000000u - delta;
      tm->tick = (tick & 0x00007fff);
      tm->sec = tm->sec + (tick >> 15) - (0x80000000u >> 15);
    }
  else
    {
      tm->tick = tm->tick - delta;
    }
}

/****************************************************************************
 * Name: latest_timestamp
 *
 * Description:
 *   Get latest timestamp when data stored into FIFO.
 *
 ****************************************************************************/

static void latest_timestamp(struct scufifo_s *fifo, uint32_t interval,
                             struct scutimestamp_s *tm, uint16_t *samples)
{
  struct scutimestamp_s pre;
  struct scutimestamp_s post;
  uint16_t pres;
  uint16_t posts;
  int rid = fifo->rid;
  int wid = fifo->wid;

  /* Get number of samples and timestamp twice for adapt roll over */

  pres = getreg32(SCUFIFO_R_STATUS0(rid)) & 0xffff;
  pre.sec = getreg32(SCUFIFO_W_TIMESTAMP0(wid));
  pre.tick = getreg16(SCUFIFO_W_TIMESTAMP1(wid));

  posts = getreg32(SCUFIFO_R_STATUS0(rid)) & 0xffff;
  post.sec = getreg32(SCUFIFO_W_TIMESTAMP0(wid));
  post.tick = getreg16(SCUFIFO_W_TIMESTAMP1(wid));

  if (pres == posts)
    {
      *samples = pres;
      tm->sec = pre.sec;
      tm->tick = pre.tick;
    }
  else
    {
      *samples = posts;
      tm->sec = post.sec;
      tm->tick = post.tick;
      if (tm->tick >= interval)
        {
          tm->tick -= interval;
        }
      else
        {
          tm->sec -= 1;
          tm->tick += 0x8000 - interval;
        }
    }
}

/****************************************************************************
 * Name: gettimestamp
 *
 * Description:
 *   Get timestamp of head of FIFO sensing data.
 *
 ****************************************************************************/

static void seq_gettimestamp(struct scufifo_s *fifo,
                             struct scutimestamp_s *tm)
{
  uint16_t sample;
  uint16_t adjust = fifo->adjust;
  uint32_t interval = fifo->interval;

  latest_timestamp(fifo, interval, tm, &sample);

  convert_firsttimestamp(tm, interval, sample, adjust);
}

/****************************************************************************
 * Name: seq_setfifomode
 *
 * Description:
 *   Enable or disable FIFO overwrite mode.
 *
 ****************************************************************************/

static void seq_setfifomode(struct seq_s *seq, int fifoid, int enable)
{
  struct scufifo_s *fifo = seq_getfifo(seq, fifoid);
  uint32_t val;
  irqstate_t flags;
  struct cxd56_scudev_s *priv = &g_scudev;
  struct wm_notify_s *notify = &priv->wm[fifo->rid];
  bool iswtmk = false;

  scuinfo("FIFO mode %d wid %d\n", enable, fifo->wid);

  if (notify->ts)
    {
      iswtmk = true;
    }

  flags = enter_critical_section();

  /* control SCU FIFO overwrite */

  val = getreg32(SCUFIFO_W_CTRL1(fifo->wid));
  val &= ~(0x1 << 4);
  if (enable)
    {
      val |= (0x1 << 4);
    }

  putreg32(val, SCUFIFO_W_CTRL1(fifo->wid));

  if (enable)
    {
      /* disable overrun err interrupt */

      val = 0x1 << (fifo->rid + 9);
      putreg32(val, SCU_INT_DISABLE_ERR_0);

      /* disable almostfull interrupt */

      if (iswtmk)
        {
          putreg32(val, SCU_INT_DISABLE_MAIN);
        }
    }
  else
    {
      /* clear overrun err */

      putreg32(getreg32(SCUFIFO_R_CTRL1(fifo->rid)) | SCUFIFO_OVERRUNCLR,
               SCUFIFO_R_CTRL1(fifo->rid));

      /* enable overrun err interrupt */

      val = 0x1 << (fifo->rid + 9);
      putreg32(val, SCU_INT_ENABLE_ERR_0);

      /* enable almostfull interrupt */

      if (iswtmk)
        {
          val = 0x1 << (fifo->rid + 9);
          putreg32(val, SCU_INT_ENABLE_MAIN);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: seq_remakeinstruction
 *
 * Description:
 *   Workaround for the sequencer last instruction issue.
 *   If the last instruction is i2c write, clear TERMINATE bit.
 *
 ****************************************************************************/

static uint16_t seq_remakeinstruction(int bustype, uint16_t inst)
{
  uint16_t ret = inst;

  if (bustype == SCU_BUS_I2C0 || bustype == SCU_BUS_I2C1)
    {
      /* When the instruction is write, clear TERMINATE bit */

      if (!(ret & (1 << 8)))
        {
          ret &= ~SCU_INST_TERM;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: scu_hwinit
 ****************************************************************************/

static void scu_hwinit(void)
{
  int i;

  /* Request don't sleep */

  seq_inhibitrequest(REQ_SLEEP, true);

  /* Disable all interrupts */

  putreg32(0xffffffff, SCU_INT_DISABLE_MAIN);
  putreg32(0xffffffff, SCU_INT_CLEAR_MAIN);
  putreg32(0x0, SCU_LEVEL_SEL_MAIN);

  for (i = 0; i < 10; i++)
    {
      putreg32(0x08000000, SCUSEQ_MATH_PROC_OFST_GAIN_X(i));
      putreg32(0x08000000, SCUSEQ_MATH_PROC_OFST_GAIN_Y(i));
      putreg32(0x08000000, SCUSEQ_MATH_PROC_OFST_GAIN_Z(i));
    }

  putreg32(0, SCU_OFST_GAIN_EN);

  putreg32(0x0f0f0f00, SCU_DECIMATION_PARAM0);
  putreg32(0x0f0f0f00, SCU_DECIMATION_PARAM1);
  putreg32(0xff, SCU_DEC_CLR);

  for (i = 0; i < 3; i++)
    {
      putreg32(0, SCU_EVENT_INTR_ENABLE(i));
    }

  /* Reset MATHFUNC */

  putreg32(0, SCU_MATHFUNC_SEL);
  putreg32(0x7, SCU_MATHFUNC_CLR);

  /* Clear MATHFUNC EVENT... */

  putreg32(0x3f, SCU_FIFO_WRITE_CTRL);
  putreg32(0x7, SCU_EVENT_STT);

  /* Reset all of FIFOs I/O */

  for (i = 0; i < 16; i++)
    {
      putreg32(3, SCUSEQ_FIFOWREVNTCTRL(i));
      putreg32(0, SCUFIFO_W_CTRL0(i));
      putreg32(0x00010000, SCUFIFO_W_CTRL1(i));
      putreg32(0, SCUFIFO_W_CTRL1(i));
    }

  for (i = 0; i < 14; i++)
    {
      putreg32(0x00010003, SCUFIFO_R_CTRL1(i));
      putreg32(0, SCUFIFO_R_CTRL1(i));
    }

  /* Decimation FIFO selection.
   * All of decimation FIFO I/O linked to each IDs.
   */

  putreg32(0x00e400e4, SCUFIFO_DECI_PARTITION_SEL);

  /* Enable SPI and I2C[01] interrupts */

  putreg32(7, SCU_INT_ENABLE_MAIN);

  /* Wait for sequencer ready */

  while (!(getreg32(SCU_SEQ_ENABLE_ALL) & 2));

  putreg32(((CONFIG_CXD56_SCU_PREDIV - 1) << 8), SCU_START_CTRL_COMMON);

  putreg32(1, SCU_SEQ_ENABLE_ALL);

  /* Enable SCU sleep */

  seq_inhibitrequest(REQ_SLEEP, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scu_spitransfer
 ****************************************************************************/

int scu_spitransfer(int slavesel, uint16_t *inst, uint32_t nr_insts,
                    uint8_t *buffer, int len)
{
  return seq_oneshot(SCU_BUS_SPI, slavesel, inst, nr_insts, buffer, len);
}

/****************************************************************************
 * Name: scu_i2ctransfer
 ****************************************************************************/

int scu_i2ctransfer(int port, int slave, uint16_t *inst,
                    uint32_t nr_insts, uint8_t *buffer, int len)
{
  int bustype;

  if (port == 0)
    {
      bustype = SCU_BUS_I2C0;
    }
  else if (port == 1)
    {
      bustype = SCU_BUS_I2C1;
    }
  else
    {
      return -ENODEV;
    }

  return seq_oneshot(bustype, slave, inst, nr_insts, buffer, len);
}

/****************************************************************************
 * Name: seq_open
 *
 * Description:
 *   Open sequencer. This API should be call from board initialize or from
 *   drivers.
 *
 * Input Parameters:
 *   type       - A type of sequencer (SEQ_TYPE_NORMAL or SEQ_TYPE_DECI)
 *   bustype    - A type of bus under SCU (SCU_BUS_I2C0, SCU_BUS_I2C1 or
 *                SCU_BUS_SPI)
 *
 ****************************************************************************/

struct seq_s *seq_open(int type, int bustype)
{
  struct seq_s *seq;

  /* Check bustype is valid */

  switch (bustype)
    {
      case SCU_BUS_SPI:
      case SCU_BUS_I2C0:
      case SCU_BUS_I2C1:
      case SCU_BUS_LPADC0:
      case SCU_BUS_LPADC1:
      case SCU_BUS_LPADC2:
      case SCU_BUS_LPADC3:
      case SCU_BUS_HPADC0:
      case SCU_BUS_HPADC1:
        break;
      default:
        return NULL;
    }

  if (type == SEQ_TYPE_DECI)
    {
      seq = deci_new();
    }
  else if (type == SEQ_TYPE_NORMAL)
    {
      seq = seq_new();
    }
  else
    {
      return NULL;
    }

  if (!seq)
    {
      return NULL;
    }

  seq->bustype = bustype;
  seq->type = type;

  seq_setbus(seq->id, bustype);

  /* Disable all of data preprocessing */

  seq_offsetgainenable(seq->id, false);
  seq_setsignconversion(seq->id, 0);

  return seq;
}

/****************************************************************************
 * Name: seq_setinstruction
 *
 * Description:
 *   Set instruction set array for sequencer execution.
 *
 * Input Parameters:
 *   seq      - An instance of sequencer
 *   inst     - Pointer of instruction array
 *   nr_insts - Number of instructions
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int seq_setinstruction(struct seq_s *seq, const uint16_t *inst,
                       uint16_t nr_insts)
{
  int istart;
  int i;
  uint16_t lastinst;

  if (nr_insts > INSTRUCTION_PER_SEQ)
    {
      return -EINVAL;
    }

  /* Remake last instruction, if needed. */

  lastinst = seq_remakeinstruction(seq->bustype, *(inst + nr_insts - 1));

  /* Copy cyclic running instruction */

  istart = seq->id * INSTRUCTION_PER_SEQ;
  for (i = 0; i < nr_insts - 1; i++)
    {
      putreg16(inst[i], SCUSEQ_INSTRUCTION(istart + i));
    }

  putreg16(lastinst, SCUSEQ_INSTRUCTION(istart + nr_insts - 1));

  /* Set instruction parameters */

  seq_setinst(seq->id, istart, nr_insts - 1);

  return OK;
}

/****************************************************************************
 * Name: seq_setbytespersample
 *
 * Description:
 *   Set sample data format
 *
 * Input Parameters:
 *   seq      - An instance of sequencer
 *   sample   - Size of sample (e.g. 16 bit 3 axis data = 6)
 *   offset   - Start offset of sampling data
 *   elemsize - Size of 1 element (e.g. 16 bit 3 axis data = 2)
 *   swapbyte - Enable/Disable byte swapping if available
 *
 ****************************************************************************/

void seq_setsample(struct seq_s *seq, uint8_t sample, uint8_t offset,
                   uint8_t elemsize, bool swapbyte)
{
  DEBUGASSERT(seq);

  seq->sample = sample;

  seq_setdataformat(seq->id, offset, sample - 1, swapbyte ? 1 : 0, elemsize);
}

/****************************************************************************
 * Name: seq_setaddress
 *
 * Description:
 *   Set slave ID or address
 *
 * Input Parameters:
 *   seq      - An instance of sequencer
 *   slave_addr - In SPI, slave select ID. In I2C, bus address.
 *
 ****************************************************************************/

void seq_setaddress(struct seq_s *seq, uint32_t slave_addr)
{
  seq_setproperty(seq->id, slave_addr, 0, 0, 0);
}

#ifdef CONFIG_CXD56_UDMAC
/****************************************************************************
 * Name: seq_fifodmadone
 *
 * Description:
 *   Callback function for receive DMA done
 *
 ****************************************************************************/

static void seq_fifodmadone(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct scufifo_s *fifo = (struct scufifo_s *)arg;
  fifo->dmaresult = status;
  nxsem_post(&fifo->dmawait);
}
#else
/****************************************************************************
 * Name: seq_read8
 ****************************************************************************/

static inline void seq_read8(uint32_t addr, uint8_t *buffer, int length)
{
  int i;

  for (i = 0; i < length; i++)
    {
      *buffer++ = getreg8(addr);
    }
}

/****************************************************************************
 * Name: seq_read16
 ****************************************************************************/

static inline void seq_read16(uint32_t addr,
                              uint16_t *buffer,
                              int length)
{
  int i;

  for (i = 0; i < length / 2; i++)
    {
      *buffer++ = getreg16(addr);
    }
}

/****************************************************************************
 * Name: seq_read32
 ****************************************************************************/

static inline void seq_read32(uint32_t addr,
                              uint32_t *buffer,
                              int length)
{
  int i;

  for (i = 0; i < length / 4; i++)
    {
      *buffer++ = getreg32(addr);
    }
}
#endif

/****************************************************************************
 * Name: seq_read
 *
 * Description:
 *   Read sensor data from sequencer FIFO
 *
 * Input Parameters:
 *   seq    - An instance of sequencer
 *   fifoid - FIFO ID (decimator only)
 *   buffer - Pointer to data receive buffer
 *   length - Length of buffer
 *
 * Returned Value:
 *   Number of bytes of read data
 *
 ****************************************************************************/

int seq_read(struct seq_s *seq, int fifoid, char *buffer, int length)
{
  struct scufifo_s *fifo;
  uint32_t outlet;
  int avail;
#ifdef CONFIG_CXD56_UDMAC
  dma_config_t config;
  uint32_t dstbuf;
  char *dst;
  int maxlen = 1024;
  int dmalen;
  int rest;
  int need_wakelock = 0;
  struct pm_cpu_wakelock_s wlock;
  wlock.info = PM_CPUWAKELOCK_TAG('S', 'C', 0);
  wlock.count = 0;
#else
  int i;
#endif
#ifdef CONFIG_CXD56_SCU_DEBUG
  uint32_t status;
#endif
  DEBUGASSERT(seq);
  DEBUGASSERT(fifoid >= 0 && fifoid < 3);

  fifo = seq_getfifo(seq, fifoid);

  DEBUGASSERT(fifo);

  outlet = SCUFIFO_FIFO_DATA(fifo->rid);

  avail = getreg32(SCUFIFO_R_STATUS0(fifo->rid));
  scuinfo("Available %d samples\n", avail);
#ifdef CONFIG_CXD56_SCU_DEBUG
  status = getreg32(SCUFIFO_R_STATUS1(fifo->rid));
  scuinfo("Status: %08" PRIx32 "\n", status);
#endif
  avail *= seq->sample;
  length = MIN(avail, length);
  if (length == 0)
    {
      return 0;
    }

#ifdef CONFIG_CXD56_UDMAC
  /* Get sensor data from FIFO by uDMAC (PL230) */

  /* TODO: Check DMA transfer limit or restart DMA to get all data. */

  config.channel_cfg = CXD56_UDMA_SINGLE;
  if (buffer != NULL)
    {
      config.channel_cfg |= CXD56_UDMA_MEMINCR;
      dst = buffer;
    }
  else
    {
      config.channel_cfg |= CXD56_UDMA_NOINCR;
      dst = (char *)&dstbuf;
    }

  if (length & 1)
    {
      config.channel_cfg |= CXD56_UDMA_XFERSIZE_BYTE;
    }
  else if (length & 2)
    {
      config.channel_cfg |= CXD56_UDMA_XFERSIZE_HWORD;
      maxlen = 2048;
    }
  else
    {
      config.channel_cfg |= CXD56_UDMA_XFERSIZE_WORD;
      maxlen = 4096;
    }

  if (((uint32_t)dst >= CXD56_RAM_BASE) &&
      ((uint32_t)dst <= (CXD56_RAM_BASE + CXD56_RAM_SIZE)))
    {
      need_wakelock = 1;
      up_pm_acquire_wakelock(&wlock);
    }

  rest = length;
  while (rest > 0)
    {
      dmalen = MIN(rest, maxlen);
      cxd56_rxudmasetup(fifo->dma, outlet, (uintptr_t)dst, dmalen, config);
      cxd56_udmastart(fifo->dma, seq_fifodmadone, fifo);

      /* Wait for DMA is done */

      nxsem_wait_uninterruptible(&fifo->dmawait);
      if (fifo->dmaresult)
        {
          /* ERROR */

          length = length - rest;
          break;
        }

      dst += dmalen;
      rest -= dmalen;
    }

  if (need_wakelock)
    {
      up_pm_release_wakelock(&wlock);
    }
#else
  /* Get sensor data from FIFO by PIO */

  if (length & 1)
    {
      if (buffer != NULL)
        {
          seq_read8(outlet, (uint8_t *)buffer, length);
        }
      else
        {
          for (i = 0; i < length; i++)
            {
              getreg8(outlet);
            }
        }
    }
  else if (length & 2)
    {
      if (buffer != NULL)
        {
          seq_read16(outlet, (uint16_t *)buffer, length);
        }
      else
        {
          for (i = 0; i < length / 2; i++)
            {
              getreg16(outlet);
            }
        }
    }
  else
    {
      if (buffer != NULL)
        {
          seq_read32(outlet, (uint32_t *)buffer, length);
        }
      else
        {
          for (i = 0; i < length / 4; i++)
            {
              getreg32(outlet);
            }
        }
    }
#endif

  return length;
}

/****************************************************************************
 * Name: seq_ioctl
 *
 * Description:
 *   Lower ioctl handler. SCU supported driver must call this API from it's
 *   ioctl().
 *
 * Input Parameters:
 *   seq    - An instance of sequencer
 *   fifoid - FIFO ID (0 - 3, decimator only)
 *   cmd    - SCU ioctl command (SCUIOC_*)
 *   arg    - Arguments for cmd
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int seq_ioctl(struct seq_s *seq, int fifoid, int cmd, unsigned long arg)
{
  int ret = OK;

  if (fifoid < 0 || fifoid > 2)
    {
      return -EINVAL;
    }

  scuinfo("cmd = %04x, arg = %08" PRIx32 "\n", cmd, arg);

  switch (cmd)
    {
      /* Set FIFO memory and initialize it. Arg: uint16_t */

      case SCUIOC_SETFIFO:
        {
          ret = seq_fifoinit(seq, fifoid, (uint16_t)arg);
        }
        break;

      /* Free FIFO memory. Arg: none */

      case SCUIOC_FREEFIFO:
        {
          struct decimator_s *deci = (struct decimator_s *)seq;
          struct scufifo_s *fifo;

          /* Check sequencer already stopped. */

          if (seq_fifoisactive(seq, fifoid))
            {
              ret = -EBUSY;
              break;
            }

          if (seq->type & SEQ_TYPE_DECI)
            {
              fifo = deci->dfifo[fifoid].fifo;
              deci->dfifo[fifoid].fifo = NULL;
            }
          else
            {
              fifo = seq->fifo;
              seq->fifo = NULL;
            }

          seq_fifofree(fifo);
        }
        break;

      /* Set sampling rate. Arg: uint8_t */

      case SCUIOC_SETSAMPLE:
        {
          ret = seq_setsamplingrate(seq, arg);
        }
        break;

      /**
       * Enable/disable sign conversion feature
       * Arg: unsigned long, 0 = off, other = on
       */

      case SCUIOC_SETSIGNCONV:
        {
          seq_setsignconversion(seq->id, arg);
        }
        break;

      /**
       * Enable offset/gain adjustment preprocessing.
       * Arg: Pointer of adjust_xyz_t
       * If arg is null, just enable offset/gain (use current setting value).
       */

      case SCUIOC_SETOGADJUST:
        {
          if (arg)
            {
              struct adjust_xyz_s *p =
                (struct adjust_xyz_s *)(uintptr_t)arg;
              ret = seq_setadjustment(seq, p);
              if (!ret)
                {
                  seq_offsetgainenable(seq->id, true);
                }
            }
          else
            {
              seq_offsetgainenable(seq->id, true);
            }
        }
        break;

      /**
       * Disable offset/gain adjustment preprocessing.
       * Arg: None
       */

      case SCUIOC_CLROGADJUST:
        {
          seq_offsetgainenable(seq->id, false);
        }
        break;

      /**
       * Set IIR filter position and coefficiencies.
       * Arg: Pointer of struct math_filter_s
       */

      case SCUIOC_SETFILTER:
        {
          struct math_filter_s *f =
            (struct math_filter_s *)(uintptr_t)arg;
          struct scufifo_s *fifo = seq_getfifo(seq, fifoid);

          if (seq_fifoisactive(seq, fifoid))
            {
              ret = -EBUSY;
              break;
            }

          if (seq->type & SEQ_TYPE_DECI)
            {
              struct decimator_s *dec = (struct decimator_s *)seq;
              fifo = dec->dfifo[fifoid].fifo;
            }

          ret = seq_setfilter(fifo, f->pos, f->filter);
        }
        break;

      /**
       * Set event notifier
       * Arg: Pointer of struct scuev_notify_s
       */

      case SCUIOC_SETNOTIFY:
        {
          struct scuev_notify_s *en =
            (struct scuev_notify_s *)(uintptr_t)arg;
          struct scufifo_s *fifo = seq_getfifo(seq, fifoid);

          if (seq_fifoisactive(seq, fifoid))
            {
              ret = -EBUSY;
              break;
            }

          if (seq->type & SEQ_TYPE_DECI)
            {
              struct decimator_s *dec = (struct decimator_s *)seq;
              fifo = dec->dfifo[fifoid].fifo;
            }

          ret = seq_seteventnotifier(fifo, en);
        }
        break;

      /**
       * Set number of elements per sample for mathfunc
       * Arg: uint8_t
       */

      case SCUIOC_SETELEMENTS:
        {
          seq_setmathwritevecelenum(seq->id, (uint8_t)arg);
        }
        break;

      /**
       * Set decimation parameters
       * Arg: Pointer of struct decimation_s
       */

      case SCUIOC_SETDECIMATION:
        {
          struct decimation_s *d =
            (struct decimation_s *)(uintptr_t)arg;
          struct decimator_s *deci = (struct decimator_s *)seq;
          struct decimation_fifo_s *dfifo = &deci->dfifo[fifoid];

          if (!(seq->type & SEQ_TYPE_DECI))
            {
              ret = -EINVAL;
              break;
            }

          /**
           * Now only save decimation parameters because decimation parameter
           * cannot be set while sequencer running.
           */

          dfifo->ratio = d->ratio;
          dfifo->leveladj = d->leveladj;
          dfifo->forcethrough = d->forcethrough;
        }
        break;

      /**
       * Set FIFO watermark
       * Arg: Pointer of struct scufifo_wm_s
       */

      case SCUIOC_SETWATERMARK:
        {
          struct scufifo_wm_s *wm =
            (struct scufifo_wm_s *)(uintptr_t)arg;

          ret = seq_setwatermark(seq, fifoid, wm);
        }
        break;

      /* Sequencer start */

      case SCUIOC_START:
        {
          ret = seq_start(seq, fifoid);
        }
        break;

      /* Sequencer stop */

      case SCUIOC_STOP:
        {
          seq_stop(seq, fifoid);
        }
        break;

      /* Set FIFO overwrite mode */

      case SCUIOC_SETFIFOMODE:
        {
          seq_setfifomode(seq, fifoid, arg);
        }
        break;

      /* Delete FIFO data */

      case SCUIOC_DELFIFODATA:
        {
          ret = seq_read(seq, fifoid, NULL, (uint16_t)arg);
        }
        break;

      default:
        scuerr("Unrecognized cmd: %d\n", cmd);
        ret = -EIO;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: seq_close
 *
 * Description:
 *   Close sequencer
 *
 * Input Parameters:
 *   seq - An instance of sequencer
 *
 ****************************************************************************/

void seq_close(struct seq_s *seq)
{
  irqstate_t flags;

  DEBUGASSERT(seq);

  if (seq->type & SEQ_TYPE_DECI)
    {
      struct decimator_s *deci = (struct decimator_s *)seq;
      int i;

      flags = enter_critical_section();
      deci_free(seq->id);
      leave_critical_section(flags);

      for (i = 0; i < 3; i++)
        {
          seq_fifofree(deci->dfifo[i].fifo);
          deci->dfifo[i].fifo = NULL;
        }
    }
  else
    {
      flags = enter_critical_section();
      seq_free(seq->id);
      leave_critical_section(flags);

      seq_fifofree(seq->fifo);
      seq->fifo = NULL;
    }

  kmm_free(seq);
}

/****************************************************************************
 * Name: scu_initialize
 ****************************************************************************/

void scu_initialize(void)
{
#ifdef CONFIG_CXD56_UDMAC
  cxd56_udmainitialize();
#endif

  scufifo_initialize();

  /**
   * If SCU clock has been already enabled, keep SCU running without loading
   * and reset of SCU firmware.
   */

  if (false == cxd56_scuseq_clock_is_enabled())
    {
      /**
       * Enable SCU clock. This process must do before loading firmware
       * because SCU instruction RAM is not accessible.
       */

      cxd56_scuseq_clock_enable();

      /* Load firmware & clear data RAM */

      memcpy((void *)CXD56_SCU_SEQ_IRAM_BASE, scu_isopprog_array,
             sizeof_scu_isopprog_array);
      memset((void *)CXD56_SCU_SEQ_DRAM_BASE, 0, 0x324);

      /* Release SCU reset to bring up SCU firmware */

      cxd56_scuseq_release_reset();

      /* Initialize SCU registers */

      scu_hwinit();
    }

  /* Enable error interrupt  */

  putreg32(0x007ffe00, SCU_INT_ENABLE_ERR_0);
  putreg32(0x03ff03ff, SCU_INT_ENABLE_ERR_2);

  /* Set the number of TxAbort repeat times */

  putreg32(5, SCUSEQ_REPEAT_TXABORT);

  /* Enable SCU IRQ */

  irq_attach(CXD56_IRQ_SCU_3, seq_scuirqhandler, &g_scudev);
  up_enable_irq(CXD56_IRQ_SCU_3);
}

/****************************************************************************
 * Name: scu_uninitialize
 ****************************************************************************/

void scu_uninitialize(void)
{
  /* Request don't sleep */

  seq_inhibitrequest(REQ_SLEEP, true);

  up_disable_irq(CXD56_IRQ_SCU_3);

  cxd56_scuseq_clock_disable();
}
