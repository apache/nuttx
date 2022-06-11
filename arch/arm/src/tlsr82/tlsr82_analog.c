/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_analog.c
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

#include <nuttx/irq.h>
#include <assert.h>
#include <debug.h>

#include "hardware/tlsr82_register.h"
#include "hardware/tlsr82_analog.h"
#include "tlsr82_analog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ANALOG_BUSY                 BIT(0)
#define ANALOG_RSV                  BIT(4)
#define ANALOG_RW                   BIT(5)
#define ANALOG_START                BIT(6)
#define ANALOG_CYC                  BIT(7)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_analog_wait
 *
 * Description:
 *   Wait until the analog quit from busy state
 *
 * Input Parameters:
 *   void
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static inline void tlsr82_analog_wait(void)
{
  while (ANALOG_CTRL_REG & ANALOG_BUSY);
}

/****************************************************************************
 * Name: tlsr82_analog_read
 *
 * Description:
 *   Read the analog register value at address <addr>
 *
 * Input Parameters:
 *   addr - the address of analog register that read
 *
 * Returned Value:
 *   read value
 *
 ****************************************************************************/

uint8_t locate_code(".ram_code") tlsr82_analog_read(uint8_t addr)
{
  irqstate_t flags;
  uint8_t data;

  flags = enter_critical_section();

  /* Read analog address addr */

  ANALOG_ADDR_REG = addr;
  ANALOG_CTRL_REG = ANALOG_START;

  /* Wait until the read finish */

  tlsr82_analog_wait();

  /* Get the data and clear the analog contrl register */

  data = ANALOG_DATA_REG;
  ANALOG_CTRL_REG = 0;

  leave_critical_section(flags);
  return data;
}

/****************************************************************************
 * Name: tlsr82_analog_read
 *
 * Description:
 *   Write the analog register value at address <addr>
 *
 * Input Parameters:
 *   addr - the address of analog register that write
 *   val  - the write value
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_analog_write(uint8_t addr, uint8_t val)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* Set the write address and value */

  ANALOG_ADDR_REG = addr;
  ANALOG_DATA_REG = val;
  ANALOG_CTRL_REG = (ANALOG_START | ANALOG_RW);

  /* Wait until the write finish */

  tlsr82_analog_wait();

  /* Clear the analog contrl register */

  ANALOG_CTRL_REG = 0;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tlsr82_analog_modify
 *
 * Description:
 *   Modity the value at <addr>
 *
 * Input Parameters:
 *   addr  - the address of analog register that write
 *   val   - the write value
 *   mask  - mask of the modified value
 *   shift - modified value shift
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void locate_code(".ram_code") tlsr82_analog_modify(uint8_t addr,
                                                   uint8_t mask,
                                                   uint8_t val)
{
  tlsr82_analog_write(addr, (tlsr82_analog_read(addr) & (~mask)) | val);
}
