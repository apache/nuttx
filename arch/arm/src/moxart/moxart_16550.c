/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <nuttx/arch.h>

#include <nuttx/serial/uart_16550.h>

#include "arm.h"
#include "up_arch.h"

uart_datawidth_t uart_getreg(uart_addrwidth_t base, unsigned int offset)
{
  return *((volatile uart_addrwidth_t *)base + offset);
}

void uart_putreg(uart_addrwidth_t base, unsigned int offset, uart_datawidth_t value)
{
  *((volatile uart_addrwidth_t *)base + offset) = value;
}
