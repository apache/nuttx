README
======

The Olimex STM32-H407 configuration is based on
stm32Fdiscovery and Olimex STM32-H405.

The H407 was programmed with ST-LINK/V2 from both Win8.1 and Ubuntu 14.04
This release provides baseline for H407 12MHZ clock in include/board.h

nsh - Only basic shell response tested on USART2
nsh_uext - Basic shell response tested on USART6 (UEXT)

Development Environment
=======================

Either Linux or Cygwin on Windows can be used for the development environment.
The source has been built only using the GNU toolchain (see below).  Other
toolchains will likely cause problems.

LEDs
====

The H407 board has 1 Status LED;

SDCard
======

Expects to be plugged in else hangs. TODO: Shouldn't hang.
SDIO 4bit with pulls/downs on pins. Doesn't have a SD card detect pin.

UARTs
=====

On the H407 board, ?? all uarts are available for pin mappings
USART2 can be used for nsh console like on Discovery board,
or you can use USART6 exposed via UEXT connector.

Olimex offers MOD-RS232 voltage level converter for the UEXT so it can be
attached to computer serial port.

STM32-H407-specific Configuration Options
===============================================
