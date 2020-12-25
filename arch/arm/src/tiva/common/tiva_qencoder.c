/****************************************************************************
 * arch/arm/src/tiva/common/tiva_qencoder.c
 *
 *   Copyright (C) 2016 Young Mu. All rights reserved.
 *   Author: Young Mu <young.mu@aliyun.com>
 *
 * The basic structure of this driver derives in spirit (if nothing more)
 * from the NuttX STM32 QEI driver which has:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>

#include "arm_arch.h"
#include "tiva_gpio.h"
#include "tiva_qencoder.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"

#include "hardware/tiva_qencoder.h"
#include "hardware/tiva_pinmap.h"
#include "hardware/tiva_memorymap.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tiva_qe_s
{
  const struct qe_ops_s *ops;
  uint8_t id;
  uintptr_t base;
  uint32_t idx;
  uint32_t pha;
  uint32_t phb;
  uint32_t maxpos;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void tiva_qe_putreg(struct tiva_qe_s *qe, unsigned int offset,
                                  uint32_t regval);
static inline uint32_t tiva_qe_getreg(struct tiva_qe_s *qe,
                                      unsigned int offset);

static int tiva_qe_setup(FAR struct qe_lowerhalf_s *lower);
static int tiva_qe_shutdown(FAR struct qe_lowerhalf_s *lower);
static int tiva_qe_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos);
static int tiva_qe_reset(FAR struct qe_lowerhalf_s *lower);
static int tiva_qe_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg);

static int tiva_qe_direction(struct tiva_qe_s *qe, unsigned long *dir);
static int tiva_qe_velocity(struct tiva_qe_s *qe, unsigned long *vel);
static int tiva_qe_resetatppr(struct tiva_qe_s *qe, unsigned long ppr);
static int tiva_qe_resetatmaxpos(FAR struct tiva_qe_s *qe,
                                 unsigned long offs);
static int tiva_qe_resetatindex(FAR struct tiva_qe_s *qe);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qe_ops =
{
  .setup    = tiva_qe_setup,
  .shutdown = tiva_qe_shutdown,
  .position = tiva_qe_position,
  .reset    = tiva_qe_reset,
  .ioctl    = tiva_qe_ioctl,
};

#ifdef CONFIG_TIVA_QEI0
static struct tiva_qe_s g_qe0 =
{
  .ops      = &g_qe_ops,
  .id       = 0,
  .base     = TIVA_QEI0_BASE,
  .idx      = GPIO_QEI0_IDX,
  .pha      = GPIO_QEI0_PHA,
  .phb      = GPIO_QEI0_PHB,
  .maxpos   = 0,
};
#endif

#ifdef CONFIG_TIVA_QEI1
static struct tiva_qe_s g_qe1 =
{
  .ops      = &g_qe_ops,
  .id       = 1,
  .base     = TIVA_QEI1_BASE,
  .idx      = GPIO_QEI1_IDX,
  .pha      = GPIO_QEI1_PHA,
  .phb      = GPIO_QEI1_PHB,
  .maxpos   = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_qe_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t tiva_qe_getreg(struct tiva_qe_s *qe,
                                      unsigned int offset)
{
  uintptr_t regaddr = qe->base + offset;
  return getreg32(regaddr);
}

/****************************************************************************
 * Name: tiva_qe_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void tiva_qe_putreg(struct tiva_qe_s *qe, unsigned int offset,
                                  uint32_t regval)
{
  uintptr_t regaddr = qe->base + offset;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: qe_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   will be configured and initialized the device so that it is ready for
 *   use.  It will not, however, output pulses until the start method is
 *   called.
 *
 * Input Parameters:
 *   lower - A reference to the lower half QEI driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_setup(FAR struct qe_lowerhalf_s *lower)
{
  uint32_t ctlreg = 0;
  FAR struct tiva_qe_s *qe = (FAR struct tiva_qe_s *)lower;
  int ret;

  sninfo("setup QEI %d\n", qe->id);

  /* Enable GPIO port, GPIO pin type and GPIO alternate function */

  ret = tiva_configgpio(qe->idx);
  if (ret < 0)
    {
      snerr("ERROR: tiva_configgpio failed (%x)\n", qe->idx);
      return -1;
    }

  ret = tiva_configgpio(qe->pha);
  if (ret < 0)
    {
      snerr("ERROR: tiva_configgpio failed (%x)\n", qe->pha);
      return -1;
    }

  ret = tiva_configgpio(qe->phb);
  if (ret < 0)
    {
      snerr("ERROR: tiva_configgpio failed (%x)\n", qe->phb);
      return -1;
    }

  /* Get initial value of register */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);

  /* Turn off the bits we're about to control */

  ctlreg &= ~(uint32_t)((1 << TIVA_QEI_CTL_RESMODE) |
                        (1 << TIVA_QEI_CTL_CAPMODE) |
                        (1 << TIVA_QEI_CTL_VELEN) |
                        (1 << TIVA_QEI_CTL_SIGMODE) |
                        (1 << TIVA_QEI_CTL_SWAP));

  /* Set reset mode (default as MAXPOS) */

  ctlreg |= RESMODE_BY_MAXPOS << TIVA_QEI_CTL_RESMODE;

  /* Set capture mode (default as PHA_AND_PHB) */

  ctlreg |= CAPMODE_PHA_AND_PHB << TIVA_QEI_CTL_CAPMODE;

  /* Enable velocity capture */

  ctlreg |= VELEN_ENABLE << TIVA_QEI_CTL_VELEN;

  /* Set signal mode (default as quadrature) */

  ctlreg |= SIGMODE_QUADRATURE << TIVA_QEI_CTL_SIGMODE;

  /* Set swap mode (default as no swap) */

  ctlreg |= SWAP_NO_SWAP << TIVA_QEI_CTL_SWAP;

  /* Program the register */

  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  /* Set default maxpos value to entire uint32_t range */

  qe->maxpos = UINT32_MAX;
  tiva_qe_putreg(qe, TIVA_QEI_MAXPOS_OFFSET, qe->maxpos);

  /* Reset the position */

  tiva_qe_putreg(qe, TIVA_QEI_POS_OFFSET, 0);

  /* Set prediv (1) */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg |= VELDIV_1 << TIVA_QEI_CTL_VELDIV;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  /* Set period load (10ms for TM4C1294NCPDT) */

  tiva_qe_putreg(qe, TIVA_QEI_LOAD_OFFSET, 1200000);

  /* Enable the QEI */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg |= QEI_ENABLE << TIVA_QEI_CTL_ENABLE;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  return OK;
}

/****************************************************************************
 * Name: qe_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop data collection, free any resources, disable the timer hardware,
 *   and put the system into the lowest possible power usage state.
 *
 * Input Parameters:
 *   lower - A reference to the lower half QEI driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct tiva_qe_s *qe = (FAR struct tiva_qe_s *)lower;

  sninfo("shutdown QEI %d\n", qe->id);

  /* Disable the QEI */

  tiva_qe_putreg(qe, TIVA_SYSCON_SRQEI_OFFSET, SYSCON_SRQEI(qe->id));

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 * Input Parameters:
 *   lower - A reference to the lower half QEI driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct tiva_qe_s *qe = (FAR struct tiva_qe_s *)lower;

  sninfo("reset QEI %d\n", qe->id);

  tiva_qe_putreg(qe, TIVA_QEI_POS_OFFSET, 0);

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_position
 *
 * Description:
 *   Return the position mesaured by QEI.
 *
 * Input Parameters:
 *   lower - A reference to the lower half QEI driver state structure
 *   pos - pointer to the position returned
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos)
{
  FAR struct tiva_qe_s *qe = (FAR struct tiva_qe_s *)lower;

  sninfo("get position of QEI %d\n", qe->id);

  *pos = (int32_t) tiva_qe_getreg(qe, TIVA_QEI_POS_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   lower - A reference to the lower half QEI driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg)
{
  FAR struct tiva_qe_s *qe = (FAR struct tiva_qe_s *)lower;

  sninfo("ioctl QEI %d\n", qe->id);

  switch (cmd)
    {
    case QEIOC_DIRECTION:
      tiva_qe_direction(qe, (unsigned long *)arg);
      break;

    case QEIOC_VELOCITY:
      tiva_qe_velocity(qe, (unsigned long *)arg);
      break;

    case QEIOC_RESETATPPR:
      tiva_qe_resetatppr(qe, arg);
      break;

    case QEIOC_RESETATMAXPOS:
      tiva_qe_resetatmaxpos(qe, arg);
      break;

    case QEIOC_RESETATINDEX:
      tiva_qe_resetatindex(qe);
      break;

    default:
      snerr("ERROR: invalid cmd %x\n", cmd);
      break;
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_direction
 *
 * Description:
 *   Return the direction mesaured by QEI.
 *
 * Input Parameters:
 *   qe - A reference to the TIVA QEI structure
 *   dir - pointer to the direction returned
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_direction(FAR struct tiva_qe_s *qe, unsigned long *dir)
{
  sninfo("get direction of QEI %d\n", qe->id);

  uint32_t statreg;
  statreg = tiva_qe_getreg(qe, TIVA_QEI_STAT_OFFSET);

  int32_t dirbit;
  dirbit =
    (statreg & (1 << TIVA_QEI_STAT_DIRECTION)) == DIRECTION_FORWARD ? 1 : -1;

  *dir = dirbit;

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_direction
 *
 * Description:
 *   Return the velocity (A/B pulses per second) mesaured by QEI.
 *
 * Input Parameters:
 *   qe - A reference to the TIVA QEI structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_velocity(FAR struct tiva_qe_s *qe, unsigned long *vel)
{
  sninfo("get direction of QEI %d\n", qe->id);

  *vel = (int32_t) tiva_qe_getreg(qe, TIVA_QEI_SPEED_OFFSET) * 100 / 4;

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_resetatppr
 *
 * Description:
 *   Set reset mode as MAXPOS and set maxpos value to number of pulses
 *   per round of encoder. Note that this function multiplies the given
 *   ppr by 4 because capture mode is PHA_AND_PHB, meaning we count 4
 *   edges per "pulse."
 *
 * Input Parameters:
 *   qe - A reference to the TIVA QEI structure
 *   ppr - pulses per round of encoder
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_resetatppr(FAR struct tiva_qe_s *qe, unsigned long ppr)
{
  /* maxpos is 4 times of ppr since we set capture mode as PHA_AND_PHB */

  return tiva_qe_resetatmaxpos(qe, ppr * 4);
}

/****************************************************************************
 * Name: tiva_qe_resetatmaxpos
 *
 * Description:
 *   Set reset mode as MAXPOS and set maxpos value as given.
 *
 * Input Parameters:
 *   qe - A reference to the TIVA QEI structure
 *   maxpos - Maximum position count at which QEI resets. To get the full
 *            range, give UINT32_MAX.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_resetatmaxpos(FAR struct tiva_qe_s *qe,
                                 unsigned long maxpos)
{
  sninfo("set maxpos reset mode and maxpos value of QEI %d\n", qe->id);

  FAR struct qe_lowerhalf_s *lower;
  uint32_t ctlreg = 0;

  /* Disable the QEI */

  lower = (FAR struct qe_lowerhalf_s *)qe;
  tiva_qe_shutdown(lower);

  qe->maxpos = maxpos;

  /* Set reset mode as MAXPOS */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg &= ~(uint32_t)(1 << TIVA_QEI_CTL_RESMODE);
  ctlreg |= RESMODE_BY_MAXPOS << TIVA_QEI_CTL_RESMODE;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  /* Set maxpos value */

  tiva_qe_putreg(qe, TIVA_QEI_MAXPOS_OFFSET, qe->maxpos);

  /* Enable the QEI */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg |= QEI_ENABLE << TIVA_QEI_CTL_ENABLE;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  return OK;
}

/****************************************************************************
 * Name: tiva_qe_resetatindex
 *
 * Description:
 *   Set reset mode as INDEX
 *
 * Input Parameters:
 *   qe - A reference to the TIVA QEI structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_qe_resetatindex(FAR struct tiva_qe_s *qe)
{
  sninfo("set index reset mode of QEI %d\n", qe->id);

  FAR struct qe_lowerhalf_s *lower;
  uint32_t ctlreg = 0;

  /* Disable the QEI */

  lower = (FAR struct qe_lowerhalf_s *)qe;
  tiva_qe_shutdown(lower);

  /* Set reset mode as INDEX */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg &= ~(uint32_t)(1 << TIVA_QEI_CTL_RESMODE);
  ctlreg |= RESMODE_BY_INDEX_PULSE << TIVA_QEI_CTL_RESMODE;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  /* Enable the QEI */

  ctlreg = tiva_qe_getreg(qe, TIVA_QEI_CTL_OFFSET);
  ctlreg |= QEI_ENABLE << TIVA_QEI_CTL_ENABLE;
  tiva_qe_putreg(qe, TIVA_QEI_CTL_OFFSET, ctlreg);

  return OK;
}

/****************************************************************************
 * Name: tiva_qei_initialize
 *
 * Description:
 *   Enable power and clock for quadrature encoder interface. This function
 *   must be called from board-specific logic.
 *
 * Input Parameters:
 *   id - A number identifying certain QEI.
 *
 * Returned Value:
 *   On success, a pointer to the lower half QEI driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *tiva_qei_initialize(int id)
{
  FAR struct tiva_qe_s *qe;
  FAR struct qe_lowerhalf_s *lower;

  switch (id)
    {
#ifdef CONFIG_TIVA_QEI0
    case 0:
      qe = &g_qe0;
      break;
#endif

#ifdef CONFIG_TIVA_QEI1
    case 1:
      qe = &g_qe1;
      break;
#endif

    default:
      snerr("ERROR: invalid QEI %d\n", id);
      return NULL;
    }

  /* Enable QEI clock */

  tiva_qei_enablepwr(qe->id);
  tiva_qei_enableclk(qe->id);

  /* Make sure that the QEI enable bit has been cleared */

  lower = (FAR struct qe_lowerhalf_s *)qe;
  tiva_qe_shutdown(lower);

  return lower;
}
