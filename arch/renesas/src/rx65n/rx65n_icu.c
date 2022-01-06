/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_icu.c
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

#include "rx65n_macrodriver.h"
#include "rx65n_icu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r_icu_create
 *
 * Description:
 * ICU Initialization
 ****************************************************************************/

void r_icu_create(void)
{
  /* Disable IRQ interrupts */

  ICU.IER[0x08].BYTE = _00_ICU_IRQ0_DISABLE  | _00_ICU_IRQ1_DISABLE  |
                       _00_ICU_IRQ2_DISABLE  | _00_ICU_IRQ3_DISABLE  |
                       _00_ICU_IRQ4_DISABLE  | _00_ICU_IRQ5_DISABLE  |
                       _00_ICU_IRQ6_DISABLE  | _00_ICU_IRQ7_DISABLE  ;
  ICU.IER[0x09].BYTE = _00_ICU_IRQ8_DISABLE  | _00_ICU_IRQ9_DISABLE  |
                       _00_ICU_IRQ10_DISABLE | _00_ICU_IRQ11_DISABLE |
                       _00_ICU_IRQ12_DISABLE | _00_ICU_IRQ13_DISABLE |
                       _00_ICU_IRQ14_DISABLE | _00_ICU_IRQ15_DISABLE ;

  /* Disable group interrupts */

  IEN(ICU, GROUPBL0) = 0u;

  /* Set IRQ settings */

  ICU.IRQCR[8].BYTE = _04_ICU_IRQ_EDGE_FALLING;
  ICU.IRQCR[9].BYTE = _04_ICU_IRQ_EDGE_FALLING;

  /* Set IRQ8 priority level */

  IPR(ICU, IRQ8) = _0F_ICU_PRIORITY_LEVEL15;

  /* Set IRQ9 priority level */

  IPR(ICU, IRQ9) = _0F_ICU_PRIORITY_LEVEL15;

  /* Set Group BL0 priority level */

  IPR(ICU, GROUPBL0) = _0F_ICU_PRIORITY_LEVEL15;

  /* Enable group BL0 interrupt */

  IEN(ICU, GROUPBL0) = 1U;

  /* Disable software interrupt */

  IEN(ICU, SWINT)  = 0U;
  IEN(ICU, SWINT2) = 0U;

  /* Set SWINT priority level */

  IPR(ICU, SWINT) = _0F_ICU_PRIORITY_LEVEL15;

  /* Set SWINT2 priority level */

  IPR(ICU, SWINT2) = _0F_ICU_PRIORITY_LEVEL15;

  /* Set IRQ8 pin */

  MPC.P00PFS.BYTE = 0x40u;
  PORT0.PDR.BYTE &= 0xfeu;
  PORT0.PMR.BYTE &= 0xfeu;

  /* Set IRQ9 pin */

  MPC.P01PFS.BYTE = 0x40u;
  PORT0.PDR.BYTE &= 0xfdu;
  PORT0.PMR.BYTE &= 0xfdu;
}

/****************************************************************************
 * Name: r_icu_irq8_start
 *
 * Description:
 * Enable IRQ8 Interrupt
 ****************************************************************************/

void r_icu_irq8_start(void)
{
  /* Enable IRQ8 interrupt */

  IEN(ICU, IRQ8) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq8_stop
 *
 * Description:
 *Initialize IRQ9 Interrupt
 ****************************************************************************/

void r_icu_irq8_stop(void)
{
  /* Disable IRQ8 interrupt */

  IEN(ICU, IRQ8) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq9_start
 *
 * Description:
 * Enable IRQ9 Interrupt
 ****************************************************************************/

void r_icu_irq9_start(void)
{
  /* Enable IRQ9 interrupt */

  IEN(ICU, IRQ9) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq9_stop
 *
 * Description:
 * Disable IRQ9 Interrupt
 ****************************************************************************/

void r_icu_irq9_stop(void)
{
  /* Disable IRQ9 interrupt */

  IEN(ICU, IRQ9) = 0u;
}

/****************************************************************************
 * Name: r_config_icu_software_start
 *
 * Description:
 * Enable S/W Interrupt
 ****************************************************************************/

void r_config_icu_software_start(void)
{
  /* Enable software interrupt */

  IEN(ICU, SWINT) = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt_generate
 *
 * Description:
 * Generate S/W Interrupt
 ****************************************************************************/

void r_config_icu_softwareinterrupt_generate(void)
{
  /* Generate software interrupt */

  ICU.SWINTR.BIT.SWINT = 1u;
}

/****************************************************************************
 * Name: r_config_icu_software_stop
 *
 * Description:
 * Disable S/W Interrupt
 ****************************************************************************/

void r_config_icu_software_stop(void)
{
  /* Disable software interrupt */

  IEN(ICU, SWINT) = 0u;
}

/****************************************************************************
 * Name: r_config_icu_software2_start
 *
 * Description:
 * Enable S/W Interrupt 2
 ****************************************************************************/

void r_config_icu_software2_start(void)
{
  /* Enable software interrupt 2 */

  IEN(ICU, SWINT2) = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt2_generate
 *
 * Description:
 * Generate software interrupt 2
 ****************************************************************************/

void r_config_icu_softwareinterrupt2_generate(void)
{
  /* Generate software interrupt 2 */

  ICU.SWINT2R.BIT.SWINT2 = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt2_stop
 *
 * Description:
 * Disable software interrupt 2
 ****************************************************************************/

void r_config_icu_software2_stop(void)
{
  /* Disable software interrupt 2 */

  IEN(ICU, SWINT2) = 0u;
}

/****************************************************************************
 * Name: r_icu_irqisfallingedge
 *
 * Description:
 * Detect if falling edge interrupt is triggered
 ****************************************************************************/

uint8_t r_icu_irqisfallingedge (const uint8_t irq_no)
{
  uint8_t falling_edge_trig = 0x0;
  if (ICU.IRQCR[irq_no].BYTE & _04_ICU_IRQ_EDGE_FALLING)
    {
      falling_edge_trig = 1;
    }

  return (falling_edge_trig);
}

/****************************************************************************
 * Name: r_icu_irqsetfallingedge
 *
 * Description:
 * Sets or unsets falling edge triggered
 ****************************************************************************/

void r_icu_irqsetfallingedge (const uint8_t irq_no, const uint8_t set_f_edge)
{
  if (1 == set_f_edge)
    {
      ICU.IRQCR[irq_no].BYTE |= _04_ICU_IRQ_EDGE_FALLING;
    }
  else
    {
      ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_04_ICU_IRQ_EDGE_FALLING;
    }
}

/****************************************************************************
 * Name: r_icu_irqsetrisingedge
 *
 * Description:
 * Sets or unsets rising edge triggered
 ****************************************************************************/

void r_icu_irqsetrisingedge (const uint8_t irq_no, const uint8_t set_r_edge)
{
  if (1 == set_r_edge)
    {
      ICU.IRQCR[irq_no].BYTE |= _08_ICU_IRQ_EDGE_RISING;
    }
  else
    {
      ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_08_ICU_IRQ_EDGE_RISING;
    }
}
