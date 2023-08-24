===================
ST STM32VLDiscovery
===================

This page discusses issues unique to NuttX configurations for the STMicro
STM32VLDiscovery (Value Line Discovery) board.

LEDs
====

It is assumed that STMicro STM32F100RB generic board board has one LED on PA0.
You should configure the port and pin number in
boards/arm/stm32/stm32vldiscovery/src/stm32vldiscovery.h. This LED is not used by
the board port unless CONFIG_ARCH_LEDS is defined.  In that case, the usage by
the board port is defined in include/board.h and src/up_leds.c. The LED is used
to encode OS-related events as follows::

  SYMBOL                Meaning                 LED1*
                                                green
  -------------------  -----------------------  -------
  LED_STARTED          NuttX has been started   ON
  LED_HEAPALLOCATE     Heap has been allocated  ON
  LED_IRQSENABLED      Interrupts enabled       ON
  LED_STACKCREATED     Idle stack created       ON
  LED_INIRQ            In an interrupt          ON
  LED_SIGNAL           In a signal handler      ON
  LED_ASSERTION        An assertion failed      OFF
  LED_PANIC            The system has crashed   OFF

So basically if the LED is off it means that there is a problem.

UART
====

Default USART/UART Configuration
--------------------------------

USART1 is enabled in all configurations (see \*/defconfig).  RX and TX are
configured on pins PA10 and PA9, respectively. Then connect the RX pin of
your USB/Serial adapter to TX pin (PA9) and the TX pin of your adapter to
RX pin (PA10) of your board besides, of course, the GND pin.

"STMicro STM32F100RB generic" specific Configuration Options
============================================================

::

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F100RB=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm32vldiscovery

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32VL_DISCOVERY=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=8192 (8kB)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LED to show state. Unique to boards that have LED(s)

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  Individual subsystems can be enabled::

       AHB
       ----
       CONFIG_STM32_CRC
       CONFIG_STM32_DMA1
       CONFIG_STM32_DMA2

       APB1
       ----
       CONFIG_STM32_TIM2
       CONFIG_STM32_TIM3
       CONFIG_STM32_TIM4
       CONFIG_STM32_TIM5
       CONFIG_STM32_TIM6
       CONFIG_STM32_TIM7
       CONFIG_STM32_TIM12
       CONFIG_STM32_TIM13
       CONFIG_STM32_TIM14
       CONFIG_STM32_RTC
       CONFIG_STM32_WWDG
       CONFIG_STM32_IWDG
       CONFIG_STM32_SPI2
       CONFIG_STM32_SPI3
       CONFIG_STM32_USART2
       CONFIG_STM32_USART3
       CONFIG_STM32_UART4
       CONFIG_STM32_UART5
       CONFIG_STM32_I2C1
       CONFIG_STM32_I2C2
       CONFIG_STM32_PWR -- Required for RTC
       CONFIG_STM32_BKP -- Required for RTC
       CONFIG_STM32_DAC1
       CONFIG_STM32_DAC2
       CONFIG_STM32_CEC

       APB2
       ----
       CONFIG_STM32_ADC1
       CONFIG_STM32_TIM1
       CONFIG_STM32_SPI1
       CONFIG_STM32_USART1
       CONFIG_STM32_TIM15
       CONFIG_STM32_TIM16
       CONFIG_STM32_TIM17

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,17
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,17
    CONFIG_STM32_TIMn_ADC1  Reserve timer n to trigger ADCm, n=1,..,17
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,17
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,17, m=1,..,2

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  JTAG Enable settings (by default full SWJ is enabled):

    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STMicro STM32F100RB generic specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

Configurations
==============

Each STMicro STM32F100RB generic configuration is maintained in a sub-directory
and can be selected as follow::

    tools/configure.sh stm32vldiscovery:<subdir>

Where <subdir> is one of the following:

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh.  The
Configuration enables only the serial NSH interfaces.

Default toolchain::

    CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Linux
