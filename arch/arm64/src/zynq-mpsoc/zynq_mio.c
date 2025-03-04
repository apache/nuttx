/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_mio.c
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
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "chip.h"
#include "zynq_mio.h"

/* Register offsets for the GPIO. Each register is 32 bits. */

#define ZYNQ_MIO_DATA_LSW_OFFSET 0x00000000 /* Mask and Data Register LSW */
#define ZYNQ_MIO_DATA_MSW_OFFSET 0x00000004 /* Mask and Data Register MSW */
#define ZYNQ_MIO_DATA_OFFSET     0x00000040 /* Data Register */
#define ZYNQ_MIO_DATA_RO_OFFSET  0x00000060 /* Data Register - Input */
#define ZYNQ_MIO_DIRM_OFFSET     0x00000204 /* Direction Mode Register */
#define ZYNQ_MIO_OUTEN_OFFSET    0x00000208 /* Output Enable Register */
#define ZYNQ_MIO_INTMASK_OFFSET  0x0000020C /* Interrupt Mask Register */
#define ZYNQ_MIO_INTEN_OFFSET    0x00000210 /* Interrupt Enable Register */
#define ZYNQ_MIO_INTDIS_OFFSET   0x00000214 /* Interrupt Disable Register */
#define ZYNQ_MIO_INTSTS_OFFSET   0x00000218 /* Interrupt Status Register */
#define ZYNQ_MIO_INTTYPE_OFFSET  0x0000021C /* Interrupt Type Register */
#define ZYNQ_MIO_INTPOL_OFFSET   0x00000220 /* Interrupt Polarity Register */
#define ZYNQ_MIO_INTANY_OFFSET   0x00000224 /* Interrupt On Any Register */

/* Register offsets for each Bank. */

#define ZYNQ_MIO_DATA_MASK_OFFSET 0x00000008 /* Data/Mask Registers offset */
#define ZYNQ_MIO_DATA_BANK_OFFSET 0x00000004 /* Data Registers offset */
#define ZYNQ_MIO_REG_MASK_OFFSET  0x00000040 /* Registers offset */

#define ZYNQ_MIO_MID_PIN_NUM           (16)
#define ZYNQ_MIO_UPPER_MASK            0xFFFF0000 /* GPIO upper 16 bit mask */
#define ZYNQ_MIO_INTTYPE_BANK012_RESET 0x03ffffff /* Resets value */
#define ZYNQ_MIO_INTTYPE_BANK345_RESET 0xffffffff /* Resets value */

/****************************************************************************
 *
 * This macro reads the given register.
 *
 * BaseAddr is the base address of the device.
 * reg_offset is the register offset to be read.
 *
 * return The 32-bit value of the register
 *
 ****************************************************************************/

#define mio_read_reg(reg_offset)    \
getreg32(ZYNQ_MPSOC_GPIO_ADDR + (uint32_t)(reg_offset))

/****************************************************************************
 *
 * This macro writes to the given register.
 *
 * BaseAddr is the base address of the device.
 * reg_offset is the offset of the register to be written.
 * data is the 32-bit value to write to the register.
 *
 ****************************************************************************/

#define mio_write_reg(reg_offset, data)  \
putreg32(data, ZYNQ_MPSOC_GPIO_ADDR + (uint32_t)(reg_offset))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 *
 * Get the bank number and the pin number in the bank, for the given pinnum
 * in the GPIO device.
 *
 * pinnum is the pin number in the GPIO device.
 * bank returns the bank in which this GPIO pin is present.
 * Valid values are 0 to ZYNQ_MIO_MAX_BANKS - 1.
 * pin_in_bank returns the pin Number within the bank.
 *
 ****************************************************************************/

static void mio_get_bank_pin(uint32_t pinnum,
                             uint32_t *bank, uint32_t *pin_in_bank)
{
  uint32_t pin_table[6];

  /* This structure defines the mapping of the pin numbers to the banks when
   * the driver APIs are used for working on the individual pins.
   */

  pin_table[0] = 25;  /* 0 - 25, bank 0 */
  pin_table[1] = 51;  /* 26 - 51, bank 1 */
  pin_table[2] = 77;  /* 52 - 77, bank 2 */
  pin_table[3] = 109; /* 78 - 109, bank 3 */
  pin_table[4] = 141; /* 110 - 141, bank 4 */
  pin_table[5] = 173; /* 142 - 173 bank 5 */

  *bank = 0;
  while (*bank < ZYNQ_MIO_MAX_BANK)
    {
      if (pinnum <= pin_table[*bank])
        {
          break;
        }

      (*bank)++;
    }

  if (*bank == 0)
    {
      *pin_in_bank = pinnum;
    }
  else
    {
      *pin_in_bank = pinnum % (pin_table[*bank - 1] + 1);
    }
}

/****************************************************************************
 *
 * This function resets the GPIO module by writing reset values to all
 * registers
 *
 ****************************************************************************/

void zynq_mio_initialize(void)
{
  uint32_t  bank;

  /* Write reset values to all mask data registers */

  for (bank = 2; bank < ZYNQ_MIO_MAX_BANK; bank++)
    {
      mio_write_reg(((bank * ZYNQ_MIO_DATA_MASK_OFFSET) +
                    ZYNQ_MIO_DATA_LSW_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_DATA_MASK_OFFSET) +
                    ZYNQ_MIO_DATA_MSW_OFFSET), 0x0);
    }

  /* Write reset values to all output data registers */

  for (bank = 2; bank < ZYNQ_MIO_MAX_BANK; bank++)
    {
      mio_write_reg(((bank * ZYNQ_MIO_DATA_BANK_OFFSET) +
                    ZYNQ_MIO_DATA_OFFSET), 0x0);
    }

  /* Reset all registers of all GPIO banks */

  for (bank = 0; bank < ZYNQ_MIO_MAX_BANK; bank++)
    {
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_DIRM_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_OUTEN_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTMASK_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTEN_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTDIS_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTSTS_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTPOL_OFFSET), 0x0);
      mio_write_reg(((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTANY_OFFSET), 0x0);
    }

  /* By default, interrupts are not masked in GPIO. Disable
   * interrupts for all pins in all the 4 banks.
   */

  for (bank = 0; bank < 3; bank++)
    {
      mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTDIS_OFFSET, ZYNQ_MIO_INTTYPE_BANK012_RESET);
    }

  for (bank = 3; bank < ZYNQ_MIO_MAX_BANK; bank++)
    {
      mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                    ZYNQ_MIO_INTDIS_OFFSET, ZYNQ_MIO_INTTYPE_BANK345_RESET);
    }
}

/****************************************************************************
 *
 * read the data register of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return  Current value of the data register.
 *
 * note: This function is used for reading the state of all the GPIO pins
 *    of specified bank.
 *
 ****************************************************************************/

uint32_t zynq_mio_read(uint32_t bank)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  return mio_read_reg((bank * ZYNQ_MIO_DATA_BANK_OFFSET) +
                      ZYNQ_MIO_DATA_RO_OFFSET);
}

/****************************************************************************
 *
 * read data from the specified pin.
 *
 * pin is the pin number for which the data has to be read.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return  Current value of the pin (0 or 1).
 *
 * note:  This function is used for reading the state of the specified
 *    GPIO pin.
 *
 ****************************************************************************/

bool zynq_mio_readpin(uint32_t pin)
{
  uint32_t bank;
  uint32_t pinnum;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  return (bool)(zynq_mio_read(bank) >> pinnum);
}

/****************************************************************************
 *
 * write to the data register of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * data is the value to be written to the data register.
 *
 * note: This function is used for writing to all the GPIO pins of
 *    the bank. The previous state of the pins is not maintained.
 *
 ****************************************************************************/

void zynq_mio_write(uint32_t bank, uint32_t data)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  mio_write_reg((bank * ZYNQ_MIO_DATA_BANK_OFFSET) +
                ZYNQ_MIO_DATA_OFFSET, data);
}

/****************************************************************************
 *
 * write data to the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * data is the data to be written to the specified pin (0 or 1).
 *
 * note: This function does a masked write to the specified pin of
 *    the specified GPIO bank. The previous state of other pins
 *    is maintained.
 *
 ****************************************************************************/

void zynq_mio_writepin(uint32_t pin, bool data)
{
  uint32_t reg_offset;
  uint32_t value;
  uint32_t bank;
  uint32_t pinnum;
  uint32_t data_var = (uint32_t)data;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  if (pinnum >= ZYNQ_MIO_MID_PIN_NUM)
    {
      /* There are only 16 data bits in bit maskable register. */

      pinnum -= ZYNQ_MIO_MID_PIN_NUM;
      reg_offset = ZYNQ_MIO_DATA_MSW_OFFSET;
    }
  else
    {
      reg_offset = ZYNQ_MIO_DATA_LSW_OFFSET;
    }

  /* Get the 32 bit value to be written to the Mask/data register where
   * the upper 16 bits is the mask and lower 16 bits is the data.
   */

  value = ~(0x1 << (pinnum + ZYNQ_MIO_MID_PIN_NUM)) &
           ((data_var << pinnum) | ZYNQ_MIO_UPPER_MASK);
  mio_write_reg((bank * ZYNQ_MIO_DATA_MASK_OFFSET) + reg_offset, value);
}

/****************************************************************************
 *
 * Set the dir of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * dir is the 32 bit mask of the pin direction to be set for
 *    all the pins in the bank. Bits with 0 are set to Input mode,
 *    bits with 1 are  set to Output Mode.
 *
 * note: This function is used for setting the direction of all the pins
 *    in the specified bank. The previous state of the pins is
 *    not maintained.
 *
 ****************************************************************************/

void zynq_mio_setdir(uint32_t bank, uint32_t dir)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                ZYNQ_MIO_DIRM_OFFSET, dir);
}

/****************************************************************************
 *
 * Set the dir of the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * dir is the direction to be set for the specified pin.
 *    Valid values are 0 for Input dir, 1 for Output dir.
 *
 ****************************************************************************/

void zynq_mio_setdirpin(uint32_t pin, bool dir)
{
  uint32_t bank;
  uint32_t pinnum;
  uint32_t dir_mode;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  dir_mode = mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                          ZYNQ_MIO_DIRM_OFFSET);

  if (dir)
    {
      dir_mode |= (1 << pinnum); /*  Output dir */
    }
  else
    {
      dir_mode &= ~ (1 << pinnum); /* Input dir */
    }

  mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                ZYNQ_MIO_DIRM_OFFSET, dir_mode);
}

/****************************************************************************
 *
 * Get the dir of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return  Returns a 32 bit mask of the dir register. Bits with 0 are
 *     in Input mode, bits with 1 are in Output Mode.
 *
 ****************************************************************************/

uint32_t zynq_mio_getdir(uint32_t bank)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  return mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                      ZYNQ_MIO_DIRM_OFFSET);
}

/****************************************************************************
 *
 * Get the dir of the specified pin.
 *
 * pin is the pin number for which the dir is to be
 *    retrieved.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return  dir of the specified pin.
 *    - 0 for Input dir
 *    - 1 for Output dir
 *
 ****************************************************************************/

uint32_t zynq_mio_getdirpin(uint32_t pin)
{
  uint32_t bank;
  uint32_t pinnum;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  return (mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
          ZYNQ_MIO_DIRM_OFFSET) >> pinnum) & 1;
}

/****************************************************************************
 *
 * Set the Output Enable of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 * outen is the 32 bit mask of the Output Enables to be set for
 *    all the pins in the bank. The Output Enable of bits with 0 are
 *    disabled, the Output Enable of bits with 1 are enabled.
 *
 * return  None.
 *
 * note: This function is used for setting the Output Enables of all the
 *    pins in the specified bank. The previous state of the Output
 *    Enables is not maintained.
 *
 ****************************************************************************/

void zynq_mio_setouten(uint32_t bank, uint32_t outen)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                ZYNQ_MIO_OUTEN_OFFSET, outen);
}

/****************************************************************************
 *
 * Set the Output Enable of the specified pin.
 *
 * pin is the pin number to which the data is to be written.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 * outen specifies whether the Output Enable for the specified
 *    pin should be enabled.
 *    Valid values are 0 for Disabling Output Enable,
 *    1 for Enabling Output Enable.
 *
 ****************************************************************************/

void zynq_mio_setoutenpin(uint32_t pin, bool outen)
{
  uint32_t bank;
  uint32_t pinnum;
  uint32_t outen_reg;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  outen_reg = mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                           ZYNQ_MIO_OUTEN_OFFSET);

  if (outen)
    {
      outen_reg |= (1 << pinnum); /*  Enable Output Enable */
    }
  else
    {
      outen_reg &= ~ (1 << pinnum); /* Disable Output Enable */
    }

  mio_write_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                ZYNQ_MIO_OUTEN_OFFSET, outen_reg);
}

/****************************************************************************
 *
 * Get the Output Enable status of the pins of the specified GPIO bank.
 *
 * bank is the bank number of the GPIO to operate on.
 *    Valid values are 0-5 in Zynq Ultrascale+ MP.
 *
 * return  Returns a a 32 bit mask of the Output Enable register.
 *    Bits with 0 are in Disabled state, bits with 1 are in
 *    Enabled State.
 *
 ****************************************************************************/

uint32_t zynq_mio_getouten(uint32_t bank)
{
  DEBUGASSERT(bank < ZYNQ_MIO_MAX_BANK);

  return mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
                      ZYNQ_MIO_OUTEN_OFFSET);
}

/****************************************************************************
 *
 * Get the Output Enable status of the specified pin.
 *
 * pin is the pin number for which the Output Enable status is to
 *    be retrieved.
 *    Valid values are 0-173 in Zynq Ultrascale+ MP.
 *
 * return  Output Enable of the specified pin.
 *    - 0 if Output Enable is disabled for this pin
 *    - 1 if Output Enable is enabled for this pin
 *
 ****************************************************************************/

uint32_t zynq_mio_getoutenpin(uint32_t pin)
{
  uint32_t bank;
  uint32_t pinnum;

  DEBUGASSERT(pin < ZYNQ_MIO_PIN_MAX);

  /* Get the bank number and pin number within the bank. */

  mio_get_bank_pin(pin, &bank, &pinnum);

  return (mio_read_reg((bank * ZYNQ_MIO_REG_MASK_OFFSET) +
          ZYNQ_MIO_OUTEN_OFFSET) >> pinnum) & 1;
}
