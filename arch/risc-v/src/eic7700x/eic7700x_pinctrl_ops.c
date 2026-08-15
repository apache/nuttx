/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_pinctrl_ops.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/pinctrl/pinctrl.h>

#include "riscv_internal.h"
#include "eic7700x_pinctrl.h"
#include "hardware/eic7700x_pinctrl.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int eic7700x_pinctrl_setfunction(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t function);
static int eic7700x_pinctrl_setstrength(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t strength);
static int eic7700x_pinctrl_setdriver(FAR struct pinctrl_dev_s *dev,
                                      uint32_t pin,
                                      enum pinctrl_drivertype_e type);
static int eic7700x_pinctrl_setslewrate(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t slewrate);
static int eic7700x_pinctrl_selectgpio(FAR struct pinctrl_dev_s *dev,
                                       uint32_t pin);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* All five operations are supplied even though this hardware has nothing
 * behind one of them.  drivers/pinctrl/pinctrl.c reaches an operation as
 * dev->ops->method(...) with no check that the pointer is there, so
 * leaving a member NULL turns the matching ioctl into a jump through NULL
 * that any process holding /dev/pinctrl0 can make.  set_slewrate returns
 * -ENOTSUP instead.
 */

const struct pinctrl_ops_s g_eic7700x_pinctrl_ops =
{
  .set_function = eic7700x_pinctrl_setfunction,
  .set_strength = eic7700x_pinctrl_setstrength,
  .set_driver   = eic7700x_pinctrl_setdriver,
  .set_slewrate = eic7700x_pinctrl_setslewrate,
  .select_gpio  = eic7700x_pinctrl_selectgpio,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_pad_setfield
 *
 * Description:
 *   Change one field of a pad and leave the others as they are.
 *
 *   The framework's operations each carry a single field, so a read has to
 *   happen somewhere.  It happens here, and the result goes back through
 *   eic7700x_pad_config() so that one set of checks covers both entry
 *   points.  Note the consequence: a pad the boot loader left holding an
 *   undocumented function select cannot have its drive strength or pull
 *   changed through this path, because the value read back fails the
 *   function check on the way out.  Refusing is the right answer there,
 *   since the alternative is quietly rewriting a function select the
 *   caller said nothing about.
 *
 * Input Parameters:
 *   pin  - The pad id
 *   mask - The field's bits within the pad register
 *   val  - The field's new value, already positioned
 *
 * Returned Value:
 *   -EINVAL if the id names no pad, otherwise whatever
 *   eic7700x_pad_config() makes of the result.
 *
 ****************************************************************************/

static int eic7700x_pad_setfield(uint32_t pin, uint32_t mask, uint32_t val)
{
  FAR const struct eic7700x_pad_s *desc;
  uint32_t cur;

  desc = eic7700x_pad_lookup(pin);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  cur = getreg32(EIC7700X_PINCTRL_PAD(pin)) & desc->wmask;
  return eic7700x_pad_config(pin, (cur & ~mask) | (val & mask));
}

/****************************************************************************
 * Name: eic7700x_pinctrl_setfunction
 *
 * Description:
 *   Select what a pad is wired to.  Only the general layout has a function
 *   select at all; the oscillator pads and the two RGMII mode registers do
 *   not, and say so rather than silently writing nothing.
 *
 * Input Parameters:
 *   dev      - The pinctrl device
 *   pin      - The pad id
 *   function - The function select to install
 *
 * Returned Value:
 *   -EINVAL if the id names no pad, -ENOTSUP if the pad has no function
 *   select, otherwise the result of writing the field.
 *
 ****************************************************************************/

static int eic7700x_pinctrl_setfunction(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t function)
{
  FAR const struct eic7700x_pad_s *desc;

  desc = eic7700x_pad_lookup(pin);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if (desc->shape != EIC7700X_PADSHAPE_GENERAL)
    {
      pinctrlerr("pad %" PRIu32 ": this pad has no function select\n", pin);
      return -ENOTSUP;
    }

  return eic7700x_pad_setfield(pin, PINCTRL_GEN_FUNC_MASK,
                               PINCTRL_GEN_FUNC(function));
}

/****************************************************************************
 * Name: eic7700x_pinctrl_setstrength
 *
 * Description:
 *   Set the pad drive strength.  The field is four bits wide on every
 *   layout that has one, though it sits in different places, so the shape
 *   decides which mask is used.
 *
 *   Section 12.1.2.1 recommends leaving this alone.
 *
 * Input Parameters:
 *   dev      - The pinctrl device
 *   pin      - The pad id
 *   strength - The drive strength field value
 *
 * Returned Value:
 *   -EINVAL if the id names no pad, -ENOTSUP if the pad has no such
 *   field, otherwise the result of writing it.
 *
 ****************************************************************************/

static int eic7700x_pinctrl_setstrength(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t strength)
{
  FAR const struct eic7700x_pad_s *desc;

  desc = eic7700x_pad_lookup(pin);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if (strength > 15)
    {
      pinctrlerr("pad %" PRIu32 ": drive strength %" PRIu32 " is over 15\n",
                 pin, strength);
      return -EINVAL;
    }

  switch (desc->shape)
    {
      case EIC7700X_PADSHAPE_GENERAL:
        return eic7700x_pad_setfield(pin, PINCTRL_GEN_DS_MASK,
                                     PINCTRL_GEN_DS(strength));

      case EIC7700X_PADSHAPE_RGMII:
        return eic7700x_pad_setfield(pin, PINCTRL_RGMII_DS_MASK,
                     strength << PINCTRL_RGMII_DS_SHIFT);

      case EIC7700X_PADSHAPE_OSC:
        return eic7700x_pad_setfield(pin, PINCTRL_OSC_DS_MASK,
                     strength << PINCTRL_OSC_DS_SHIFT);

      default:
        pinctrlerr("pad %" PRIu32 ": this pad has no drive strength\n", pin);
        return -ENOTSUP;
    }
}

/****************************************************************************
 * Name: eic7700x_pinctrl_setdriver
 *
 * Description:
 *   Set the pad bias.  The hardware has a pull up bit and a pull down bit
 *   and nothing else, so a request for a strong pull down is refused
 *   rather than served as an ordinary one: quietly giving a caller a
 *   weaker pull than it asked for is how a bus ends up marginal.
 *
 * Input Parameters:
 *   dev  - The pinctrl device
 *   pin  - The pad id
 *   type - The bias to apply
 *
 * Returned Value:
 *   -EINVAL if the id names no pad, -ENOTSUP for a bias this hardware
 *   cannot make, otherwise the result of writing the field.
 *
 ****************************************************************************/

static int eic7700x_pinctrl_setdriver(FAR struct pinctrl_dev_s *dev,
                                      uint32_t pin,
                                      enum pinctrl_drivertype_e type)
{
  FAR const struct eic7700x_pad_s *desc;
  uint32_t val;

  desc = eic7700x_pad_lookup(pin);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if (desc->shape == EIC7700X_PADSHAPE_OSC ||
      desc->shape == EIC7700X_PADSHAPE_MODESEL)
    {
      pinctrlerr("pad %" PRIu32 ": this pad has no pull up or down\n", pin);
      return -ENOTSUP;
    }

  switch (type)
    {
      case BIAS_DISABLE:
        val = 0;
        break;

      case BIAS_PULLUP:
        val = PINCTRL_GEN_PU;
        break;

      case BIAS_PULLDOWN:
        val = PINCTRL_GEN_PD;
        break;

      default:
        pinctrlerr("pad %" PRIu32 ": bias %d has no equivalent here\n",
                   pin, type);
        return -ENOTSUP;
    }

  return eic7700x_pad_setfield(pin, PINCTRL_GEN_PU | PINCTRL_GEN_PD, val);
}

/****************************************************************************
 * Name: eic7700x_pinctrl_setslewrate
 *
 * Description:
 *   Refuse.  These pads have a Schmitt trigger on the input and no slew
 *   rate control on the output, so there is nothing this could set.
 *
 *   It exists only because the framework calls through the operations
 *   table without checking, so a NULL here would be reachable from user
 *   space.
 *
 * Input Parameters:
 *   dev      - The pinctrl device
 *   pin      - The pad id
 *   slewrate - Ignored
 *
 * Returned Value:
 *   -ENOTSUP, always.
 *
 ****************************************************************************/

static int eic7700x_pinctrl_setslewrate(FAR struct pinctrl_dev_s *dev,
                                        uint32_t pin, uint32_t slewrate)
{
  pinctrlerr("pad %" PRIu32 ": these pads have no slew rate control\n", pin);
  return -ENOTSUP;
}

/****************************************************************************
 * Name: eic7700x_pinctrl_selectgpio
 *
 * Description:
 *   Point a pad at the GPIO block.  Which function select value does that
 *   differs per pad, so it comes from the table rather than being a
 *   constant: it is 2 on most pads and 0 on the few whose only function is
 *   GPIO.
 *
 *   The input enable is set at the same time.  Section 12.6.3.1 says a
 *   GPIO used as an input needs it, this is the only place that knows
 *   where the bit lives, and enabling the input buffer does not stop the
 *   pad driving, so setting it costs an output nothing.
 *
 * Input Parameters:
 *   dev - The pinctrl device
 *   pin - The pad id
 *
 * Returned Value:
 *   -EINVAL if the id names no pad, -ENOTSUP if it cannot be pointed at
 *   the GPIO block, otherwise the result of writing the fields.
 *
 ****************************************************************************/

static int eic7700x_pinctrl_selectgpio(FAR struct pinctrl_dev_s *dev,
                                       uint32_t pin)
{
  FAR const struct eic7700x_pad_s *desc;

  desc = eic7700x_pad_lookup(pin);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if (desc->gpiofunc == EIC7700X_PAD_NOGPIO)
    {
      pinctrlerr("pad %" PRIu32 ": no GPIO function is documented\n", pin);
      return -ENOTSUP;
    }

  return eic7700x_pad_setfield(pin,
                               PINCTRL_GEN_FUNC_MASK | PINCTRL_GEN_IE,
                               PINCTRL_GEN_FUNC(desc->gpiofunc) |
                               PINCTRL_GEN_IE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_pad_lookup
 *
 * Description:
 *   Bound a pad id and return the row describing it.  Nothing else in this
 *   driver may index the pad table directly: this is the single place an
 *   out of range id is caught.
 *
 *   The pinctrl framework bounds nothing.  pinctrl_register() records the
 *   device and every ioctl hands the pin number straight through, so a pin
 *   number arriving from user space is whatever the caller typed.  The
 *   provider is the only bound there is.
 *
 * Input Parameters:
 *   pad - The pad id, an enum eic7700x_pad_e value
 *
 * Returned Value:
 *   The pad description, or NULL if the id names no pad.
 *
 ****************************************************************************/

FAR const struct eic7700x_pad_s *eic7700x_pad_lookup(unsigned int pad)
{
  if (pad >= EIC7700X_PAD_NPADS)
    {
      pinctrlerr("pad %u is past the end of the block\n", pad);
      return NULL;
    }

  return &g_eic7700x_pads[pad];
}

/****************************************************************************
 * Name: eic7700x_pad_config
 *
 * Description:
 *   Configure one pad.  See the prototype for the contract.
 *
 *   Three things are checked before anything is written, in order of how
 *   much they cost to get wrong.
 *
 *   First the policy: three registers in this range are refused outright.
 *   LPDDR_REF_CLK carries the DDR reference clock, and the two RGMII mode
 *   registers select the IO voltage for a whole group, where the manual
 *   says values other than the two it lists are forbidden.  Note that the
 *   hardware would allow all three to be written; only this refuses.
 *
 *   Then the write mask, which is the manual's access column.  Asking to
 *   drive a bit that reports state only is a mistake worth reporting
 *   rather than a write to drop silently, so it fails instead of being
 *   masked away.
 *
 *   Last the function select, and only for pads that have one.  A pad's
 *   documented functions are a subset of the eight the field can hold, and
 *   the two are not the same thing: seventeen pads have a writable
 *   function select but only one function named for them anywhere in the
 *   manual.  Selecting an undocumented value would be asking the pad for
 *   something nobody has written down, so it is refused.
 *
 ****************************************************************************/

int eic7700x_pad_config(unsigned int pad, uint32_t cfg)
{
  FAR const struct eic7700x_pad_s *desc;

  desc = eic7700x_pad_lookup(pad);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if ((desc->flags & EIC7700X_PAD_LOCKED) != 0)
    {
      pinctrlerr("pad %u: refusing to move a pad the system runs on\n",
                 pad);
      return -EPERM;
    }

  if ((cfg & ~desc->wmask) != 0)
    {
      pinctrlerr("pad %u: %08" PRIx32 " drives bits outside %08" PRIx32
                 "\n", pad, cfg, desc->wmask);
      return -EROFS;
    }

  if (desc->shape == EIC7700X_PADSHAPE_GENERAL)
    {
      unsigned int func;

      func = (cfg & PINCTRL_GEN_FUNC_MASK) >> PINCTRL_GEN_FUNC_SHIFT;
      if ((desc->funcmask & (1 << func)) == 0)
        {
          pinctrlerr("pad %u: function %u is not documented, have %02x\n",
                     pad, func, desc->funcmask);
          return -EINVAL;
        }
    }

  /* Write the writable bits and leave the rest as they were.  modifyreg32
   * takes a spinlock with interrupts disabled, so the read modify write is
   * safe against another hart configuring a different pad in the same
   * register.  Nothing here serialises two callers configuring the same
   * pad.  Two callers configuring one pad is not something a board does,
   * and guarding against it would mean a lock the framework does not
   * expect this provider to hold.
   */

  modifyreg32(EIC7700X_PINCTRL_PAD(pad), desc->wmask, cfg);

  pinctrlinfo("pad %u: %08" PRIx32 "\n", pad, cfg);
  return OK;
}
