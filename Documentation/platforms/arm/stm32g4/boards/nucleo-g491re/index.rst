================
ST Nucleo G491RE
================

.. tags:: arch:arm, chip:stm32g4, part:stm32g491, vendor:st

Nucleo-64 board with the STM32G491RE MCU:

  https://www.st.com/en/evaluation-tools/nucleo-g491re.html

The STM32G491 is the category 4 member of the family: compared to the
STM32G474 it has no HRTIM, ADC4/5, DAC2/4, COMP5-7, OPAMP4/5, I2C4, SPI4,
FDCAN3, TIM5, FMC or QUADSPI, 112 KB of SRAM (80 + 16 KB plus 16 KB CCM)
and a single bank 512 KB flash with 2 KB pages. The board is otherwise
pin compatible with the Nucleo-G474RE.

Clocking
========

SYSCLK is 170 MHz from the PLL fed by the 16 MHz HSI. HSE and LSE are not
installed.

Serial Console
==============

LPUART1 on PA2/PA3 is connected to the ST-LINK virtual COM port and is the
default console. USART3 on PC10/PC11 is available on CN7 pins 1 and 2.

Configurations
==============

nsh
---

NSH on the ST-LINK virtual COM port.
