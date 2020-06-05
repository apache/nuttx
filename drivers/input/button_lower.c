/****************************************************************************
 * drivers/input/button_lower.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/input/buttons.h>

#include <nuttx/irq.h>

#if CONFIG_BUTTONS_LOWER

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static btn_buttonset_t
btn_supported(FAR const struct btn_lowerhalf_s *lower);
static btn_buttonset_t btn_buttons(FAR const struct btn_lowerhalf_s *lower);
static void btn_enable(FAR const struct btn_lowerhalf_s *lower,
                       btn_buttonset_t press, btn_buttonset_t release,
                       btn_handler_t handler, FAR void *arg);

static void btn_disable(void);
static int btn_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_btnnum;

/* This is the button button lower half driver interface */

static const struct btn_lowerhalf_s g_btnlower =
{
  .bl_supported  = btn_supported,
  .bl_buttons    = btn_buttons,
  .bl_enable     = btn_enable,
};

/* Current interrupt handler and argument */

static btn_handler_t g_btnhandler;
static FAR void *g_btnarg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_supported
 *
 * Description:
 *   Return the set of buttons supported
 *
 ****************************************************************************/

static btn_buttonset_t btn_supported(FAR const struct btn_lowerhalf_s *lower)
{
  iinfo("NUM_BUTTONS: %02x\n", g_btnnum);
  return (btn_buttonset_t)((1 << g_btnnum) - 1);
}

/****************************************************************************
 * Name: btn_buttons
 *
 * Description:
 *   Return the current state of button data
 *
 ****************************************************************************/

static btn_buttonset_t btn_buttons(FAR const struct btn_lowerhalf_s *lower)
{
  return board_buttons();
}

/****************************************************************************
 * Name: btn_enable
 *
 * Description:
 *   Enable interrupts on the selected set of buttons.  And empty set or
 *   a NULL handler will disable all interrupts.
 *
 ****************************************************************************/

static void btn_enable(FAR const struct btn_lowerhalf_s *lower,
                       btn_buttonset_t press, btn_buttonset_t release,
                       btn_handler_t handler, FAR void *arg)
{
  btn_buttonset_t mask;
  btn_buttonset_t either = press | release;
  irqstate_t flags;
  uint32_t id;

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  btn_disable();

  iinfo("press: %02x release: %02x handler: %p arg: %p\n",
        press, release, handler, arg);

  /* If no events are indicated or if no handler is provided, then this
   * must really be a request to disable interrupts.
   */

  if (either && handler)
    {
      /* Save the new the handler and argument */

      g_btnhandler = handler;
      g_btnarg     = arg;

      /* Attach and enable each button interrupt */

      for (id = 0; id < g_btnnum; id++)
        {
          mask = (1 << id);
          if ((either & mask) != 0)
            {
              board_button_irq(id, btn_interrupt, NULL);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: btn_disable
 *
 * Description:
 *   Disable all button interrupts
 *
 ****************************************************************************/

static void btn_disable(void)
{
  irqstate_t flags;
  uint32_t id;

  /* Disable each button interrupt */

  flags = enter_critical_section();
  for (id = 0; id < g_btnnum; id++)
    {
      board_button_irq(id, NULL, NULL);
    }

  /* Nullify the handler and argument */

  g_btnhandler = NULL;
  g_btnarg     = NULL;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: btn_interrupt
 *
 * Description:
 *   Discrete button interrupt handler (all buttons)
 *
 ****************************************************************************/

static int btn_interrupt(int irq, FAR void *context, FAR void *arg)
{
  DEBUGASSERT(g_btnhandler);

  if (g_btnhandler)
    {
      g_btnhandler(&g_btnlower, g_btnarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_lower_initialize
 *
 * Description:
 *   Initialize the generic button lower half driver, bind it and register
 *   it with the upper half button driver as devname.
 *
 ****************************************************************************/

int btn_lower_initialize(FAR const char *devname)
{
  g_btnnum = board_button_initialize();
  return btn_register(devname, &g_btnlower);
}

#endif /* CONFIG_BUTTONS_LOWER */
