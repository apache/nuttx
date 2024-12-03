/****************************************************************************
 * libs/libbuiltin/libgcc/profile.c
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

#include <debug.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/gmon.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GMONVERSION     0x00051879

/* Histogram counters are unsigned shorts (according to the kernel). */

#define HISTCOUNTER     unsigned short

/* Fraction of text space to allocate for histogram counters here, 1/2 */

#define HISTFRACTION    2

/* Fraction of text space to allocate for from hash buckets.
 * The value of HASHFRACTION is based on the minimum number of bytes
 * of separation between two subroutine call points in the object code.
 * Given MIN_SUBR_SEPARATION bytes of separation the value of
 * HASHFRACTION is calculated as:
 *
 * HASHFRACTION = MIN_SUBR_SEPARATION / (2 * sizeof(short) - 1);
 *
 * For example, on the VAX, the shortest two call sequence is:
 *
 *      calls     $0,(r0)
 *      calls     $0,(r0)
 *
 * Which is separated by only three bytes, thus HASHFRACTION is
 * calculated as:
 *
 * HASHFRACTION = 3 / (2 * 2 - 1) = 1
 *
 * Note that the division above rounds down, thus if MIN_SUBR_FRACTION
 * is less than three, this algorithm will not work!
 *
 * In practice, however, call instructions are rarely at a minimal
 * distance.  Hence, we will define HASHFRACTION to be 2 across all
 * architectures.  This saves a reasonable amount of space for
 * profiling data structures without (in practice) sacrificing
 * any granularity.
 */

#define HASHFRACTION    2

/* Percent of text space to allocate for tostructs with a minimum.
 * This is a heuristic; we will fail with a warning when profiling
 * programs with a very large number of very small functions, but
 * that's normally OK.
 * 2 is probably still a good value for normal programs.
 * Profiling a test case with 64000 small functions will work if
 * you raise this value to 3 and link statically (which bloats the
 * text size, thus raising the number of arcs expected by the heuristic).
 */

#define ARCDENSITY      3

/* Always allocate at least this many tostructs.  This
 * hides the inadequacy of the ARCDENSITY heuristic, at least
 * for small programs.
 */

#define MINARCS         50

/* The type used to represent indices into gmonparam.tos[]. */

#define ARCINDEX        unsigned long

/* Maximum number of arcs we want to allow.
 * Used to be max representable value of ARCINDEX minus 2, but now
 * that ARCINDEX is a long, that's too large; we don't really want
 * to allow a 48 gigabyte table.
 */

#define MAXARCS         (1 << 20)

/* General rounding functions. */

#define ROUNDDOWN(x, y) (((x) / (y)) * (y))
#define ROUNDUP(x, y)   ((((x) + (y) - 1) / (y)) * (y))

/* See profil(2) where this is described (incorrectly) */

#define SCALE_1_TO_1    0x10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tostruct
{
  uintptr_t selfpc; /* Callee address/program counter. The caller address
                     * is in froms[] array which points to tos[] array
                     */
  long      count;  /* How many times it has been called */
  ARCINDEX  link;   /* Link to next entry in hash table. For tos[0] this
                     * points to the last used entry
                     */
};

/* Structure prepended to gmon.out profiling data file. */

struct gmonhdr
{
  uintptr_t lpc;      /* Base pc address of sample buffer */
  uintptr_t hpc;      /* Max pc address of sampled buffer */
  uint32_t  ncnt;     /* Size of sample buffer (plus this header) */
  uint32_t  version;  /* Version number */
  uint32_t  profrate; /* Profiling clock rate */
  uint32_t  spare[3]; /* Reserved */
};

/* A raw arc, with pointers to the calling site and
 * the called site and a count.
 */

struct rawarc
{
  uintptr_t raw_frompc;
  uintptr_t raw_selfpc;
  long      raw_count;
};

/* The profiling data structures are housed in this structure. */

struct gmonparam
{
  bool                 running;
  FAR unsigned short  *kcount;     /* Histogram PC sample array */
  size_t               kcountsize; /* Size of kcount[] array in bytes */
  FAR ARCINDEX        *froms;      /* Array of hashed 'from' addresses. The 16bit
                                    * value is an index into the tos[] array
                                    */
  size_t               fromssize;  /* Size of froms[] array in bytes */
  FAR struct tostruct *tos;        /* To struct, contains histogram counter */
  size_t               tossize;    /* Size of tos[] array in bytes */
  size_t               tolimit;
  uintptr_t            lowpc;      /* Low program counter of area */
  uintptr_t            highpc;     /* High program counter */
  size_t               textsize;   /* Code size */
  spinlock_t           lock;       /* Lock for this structure */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gmonparam g_monparam;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _stext[];
extern uint8_t _etext[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

noinstrument_function
static int write_gmon(FAR struct gmonparam *p, FAR const char *output)
{
  struct gmonhdr gmonhdr;
  struct rawarc rawarc;
  struct file file;
  uintptr_t frompc;
  ARCINDEX toindex;
  size_t fromindex;
  size_t endfrom;
  int ret;

  ret = file_open(&file, output, O_CREAT | O_TRUNC | O_WRONLY, 0666);
  if (ret < 0)
    {
      serr("cannot open %s\n", output);
      return ret;
    }

  gmonhdr.lpc = p->lowpc;
  gmonhdr.hpc = p->highpc;
  gmonhdr.ncnt = sizeof(gmonhdr) + p->kcountsize;
  gmonhdr.version = GMONVERSION;
  gmonhdr.profrate = CONFIG_SCHED_PROFILE_TICKSPERSEC;

  ret = file_write(&file, &gmonhdr, sizeof(gmonhdr));
  if (ret != sizeof(gmonhdr))
    {
      serr("write gmonhdr failed\n");
      goto out;
    }

  ret = file_write(&file, p->kcount, p->kcountsize);
  if (ret != p->kcountsize)
    {
      serr("write kcount failed\n");
      goto out;
    }

  endfrom = p->fromssize / sizeof(*p->froms);
  for (fromindex = 0; fromindex < endfrom; fromindex++)
    {
      if (p->froms[fromindex] == 0)
        {
          continue;
        }

      frompc = p->lowpc;
      frompc += fromindex * HASHFRACTION * sizeof(*p->froms);

      for (toindex = p->froms[fromindex]; toindex != 0;
           toindex = p->tos[toindex].link)
        {
          rawarc.raw_frompc = frompc;
          rawarc.raw_selfpc = p->tos[toindex].selfpc;
          rawarc.raw_count = p->tos[toindex].count;
          ret = file_write(&file, &rawarc, sizeof(rawarc));
          if (ret != sizeof(rawarc))
            {
              serr("write rawarc failed\n");
              goto out;
            }
        }
    }

out:
  file_close(&file);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Control profiling
 *  profiling is what mcount checks to see if
 *  all the data structures are ready.
 */

noinstrument_function
void moncontrol(int mode)
{
  FAR struct gmonparam *p = &g_monparam;
  irqstate_t flags;

  if (p->running == !!mode)
    {
      return;
    }

  if (mode)
    {
      uintptr_t lowpc = ROUNDDOWN((uintptr_t)&_stext,
                                   HISTFRACTION * sizeof(HISTCOUNTER));
      uintptr_t highpc = ROUNDUP((uintptr_t)&_etext,
                                 HISTFRACTION * sizeof(HISTCOUNTER));
      size_t textsize = highpc - lowpc;
      size_t kcountsize = ROUNDUP(textsize / HISTFRACTION,
                                  sizeof(*p->kcount));
      int scale = kcountsize >= textsize ? SCALE_1_TO_1 :
                  (float)kcountsize / textsize * SCALE_1_TO_1;
      FAR unsigned short *kcount = kmm_zalloc(kcountsize);
      if (kcount == NULL)
        {
          serr("out of memory\n");
          return;
        }

      flags = spin_lock_irqsave(&p->lock);
      if (p->kcount)
        {
          spin_unlock_irqrestore(&p->lock, flags);
          kmm_free(kcount);
          return;
        }

      p->running = true;
      p->lowpc = lowpc;
      p->highpc = highpc;
      p->textsize = textsize;
      p->kcount = kcount;
      p->kcountsize = kcountsize;
      spin_unlock_irqrestore(&p->lock, flags);

      profil(kcount, kcountsize, lowpc, scale);
    }
  else
    {
      bool running;

      flags = spin_lock_irqsave(&p->lock);
      running = p->running;
      p->running = false;
      spin_unlock_irqrestore(&p->lock, flags);

      if (running)
        {
          profil(NULL, 0, 0, 0);
        }
    }
}

noinstrument_function
void monstartup(unsigned long lowpc, unsigned long highpc)
{
  FAR struct gmonparam *p = &g_monparam;
  irqstate_t flags;
  FAR char *buffer;
  size_t textsize;
  size_t fromssize;
  size_t tolimit;
  size_t tossize;

  /* If we are incorrectly called twice in a row (without an
   * intervening call to _mcleanup), ignore the second call to
   * prevent leaking memory.
   */

  if (p->tos != NULL)
    {
      return;
    }

  /* Return if the allocation doesn't allow in the current context */

  if (!OSINIT_OS_READY() || up_interrupt_context())
    {
      return;
    }

  /* Round lowpc and highpc to multiples of the density we're using
   * so the rest of the scaling (here and in gprof) stays in ints.
   */

  lowpc = ROUNDDOWN(lowpc, HISTFRACTION * sizeof(HISTCOUNTER));
  highpc = ROUNDUP(highpc, HISTFRACTION * sizeof(HISTCOUNTER));
  textsize = highpc - lowpc;
  fromssize = ROUNDUP(textsize / HASHFRACTION, sizeof(*p->froms));
  tolimit = textsize * ARCDENSITY / 100;

  if (tolimit < MINARCS)
    {
      tolimit = MINARCS;
    }
  else if (tolimit > MAXARCS)
    {
      tolimit = MAXARCS;
    }

  tossize = tolimit * sizeof(struct tostruct);

  buffer = kmm_zalloc(fromssize + tossize);
  if (buffer == NULL)
    {
      serr("out of memory\n");
      return;
    }

  flags = spin_lock_irqsave(&p->lock);
  if (p->tos != NULL)
    {
      spin_unlock_irqrestore(&p->lock, flags);
      kmm_free(buffer);
      return;
    }

  p->lowpc = lowpc;
  p->highpc = highpc;
  p->textsize = textsize;
  p->fromssize = fromssize;
  p->tolimit = tolimit;
  p->tossize = tossize;

  p->tos = (FAR struct tostruct *)buffer;
  buffer += p->tossize;
  p->froms = (FAR ARCINDEX *)buffer;
  spin_unlock_irqrestore(&p->lock, flags);

  moncontrol(1);
}

noinstrument_function
void _mcleanup(void)
{
  FAR struct gmonparam *p = &g_monparam;
  FAR const char *prefix = NULL;

#ifndef CONFIG_DISABLE_ENVIRON
  prefix = getenv("GMON_OUT_PREFIX");
#endif
  if (prefix == NULL)
    {
      prefix = "gmon.out";
    }

  moncontrol(0);
  if (p->kcount)
    {
      write_gmon(p, prefix);
    }

  kmm_free(p->tos);
  kmm_free(p->kcount);

  /* Reset buffer to initial state for safety */

  memset(p, 0, sizeof(*p));
}

/* mcount_internal is called on entry to each function compiled with
 * the profiling switch set by an assembly stub in:
 * libs/libc/machine/xxx/mcount.S
 * which updates data structures that represent traversals of the
 * program's call graph edges.  frompc and selfpc are the return
 * address and function address that represents the given call graph edge.
 */

noinstrument_function
void mcount_internal(uintptr_t frompc, uintptr_t selfpc)
{
  FAR struct gmonparam *p = &g_monparam;
  FAR struct tostruct *prevtop;
  FAR struct tostruct *top;
  FAR ARCINDEX *frompcindex;
  ARCINDEX toindex;
  irqstate_t flags;

  /* Check that we are profiling */

  if (!p->running)
    {
      return;
    }

  /* Initialize the internal structure if not yet */

  monstartup((uintptr_t)&_stext, (uintptr_t)&_etext);

  flags = spin_lock_irqsave(&p->lock);

  /* Try next time if fail to initialize for some reason */

  if (p->tos == NULL)
    {
      goto done;
    }

  /* Check that frompc is a reasonable pc value.
   * For example: signal catchers get called from the stack,
   * not from text space.  Too bad.
   */

  frompc -= p->lowpc;
  if (frompc > p->textsize)
    {
      goto done;
    }

  frompcindex = &p->froms[frompc / (HASHFRACTION * sizeof(*p->froms))];
  toindex = *frompcindex; /* Get froms[] value */
  if (toindex == 0)
    {
      /* First time traversing this arc */

      toindex = ++p->tos[0].link; /* The link of tos[0] points to the last
                                   * used record in the array
                                   */
      if (toindex >= p->tolimit)
        {
          /* More tos[] entries than we can handle! */

          goto done;
        }

      /* Store new 'to' value into froms[] */

      *frompcindex = toindex;
      top = &p->tos[toindex];
      top->selfpc = selfpc;
      top->count = 1;
      top->link = 0;
      goto done;
    }

  top = &p->tos[toindex];
  if (top->selfpc == selfpc)
    {
      /* Arc at front of chain; usual case. */

      top->count++;
      goto done;
    }

  /* Have to go looking down chain for it.
   * Top points to what we are looking at,
   * prevtop points to previous top.
   * We know it is not at the head of the chain.
   */

  for (; ; )
    {
      if (top->link == 0)
        {
          /* Top is end of the chain and none of the chain
           * had top->selfpc == selfpc.
           * So we allocate a new tostruct
           * and link it to the head of the chain.
           */

          toindex = ++p->tos[0].link;
          if (toindex >= p->tolimit)
            {
              goto done;
            }

          top = &p->tos[toindex];
          top->selfpc = selfpc;
          top->count = 1;
          top->link = *frompcindex;
          *frompcindex = toindex;
          goto done;
        }

      /* Otherwise, check the next arc on the chain. */

      prevtop = top;
      top = &p->tos[top->link];
      if (top->selfpc == selfpc)
        {
          /* There it is.
           * Increment its count
           * move it to the head of the chain.
           */

          top->count++;
          toindex = prevtop->link;
          prevtop->link = top->link;
          top->link = *frompcindex;
          *frompcindex = toindex;
          goto done;
        }
    }

done:
  spin_unlock_irqrestore(&p->lock, flags);
}
