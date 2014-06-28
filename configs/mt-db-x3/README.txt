README
======

This README file discusses the port of NuttX to the MT-DB-X3 development
board from MattairTech.  This board may host a variety of 64-pin AVR XMega
part.  This port was specifically performed using the ATXmega384c3.

See the MattairTech website for further information about the MT-DB-X3:
http://www.mattairtech.com/.

Contents
========

  - PIO Usage
  - LEDs and Buttons
  - Serial Console
  - Configurations

PIO Usage
=========

PA0  CONN_P_10 pin 1, Aref via J34
PA1  CONN_P_10 pin 2
PA2  CONN_P_10 pin 3
PA3  CONN_P_10 pin 4
PA4  CONN_P_10 pin 5
PA5  CONN_P_10 pin 6
PA6  CONN_P_10 pin 7
PA7  CONN_P_10 pin 8

PB0  CONN_P_10 pin 9, Aref via J25
PB1  CONN_P_10 pin 10
PB2  C4_LS CONN_P14 pin 5
PB3  C4_LS CONN_P14 pin 5
PB4  C4_LS CONN_P14 pin 5, Alt JTAG TMS
PB5  C4_LS CONN_P14 pin 5, Alt JTAG TDI
PB6  C4_LS CONN_P14 pin 5, Alt JTAG TCK
PB7  C4_LS CONN_P14 pin 5, Alt JTAG TDO, Voltage measurement via J15

PC0  C0_LS CONN_P14 pin 7
PC1  C1_LS CONN_P14 pin 8
PC2  C2_LS CONN_P14 pin 9
PC3  C3_LS CONN_P14 pin 10
PC4  C4_LS CONN_P14 pin 5, PDI/SPI Header pin 5 via J8
PC5  C5_LS CONN_P14 pin 6, PDI/SPI Header pin 4 via J8
PC6  C6_LS CONN_P14 pin 7, PDI/SPI Header pin 1 via J13
PC7  C7_LS CONN_P14 pin 8, PDI/SPI Header pin 3 via J12

PD0  CONN_P14 pin 9
PD1  CONN_P14 pin 10
PD2  CONN_P14 pin 11
PD3  CONN_P14 pin 12
PD4  CONN_P14 pin 13
PD5  CONN_P14 pin 14, TXB0103 Output Enable
PD6  CONN_P14 pin 12, USB D-
PD7  CONN_P14 pin 13, USB D+

PE0  CONN_P10 pin 1
PE1  CONN_P10 pin 2
PE2  CONN_P10 pin 3
PE3  CONN_P10 pin 4
PE4  CONN_P10 pin 5
PE5  CONN_P10 pin 6
PE6  CONN_P10 pin 7, TOSC1 via J28
PE7  CONN_P10 pin 8, TOSC1 via J29

PF0  PORT F pin 1
PF1  PORT F pin 2
PF2  PORT F pin 3
PF3  PORT F pin 4
PF4  PORT F pin 5, Bootloader jumper
PF5  PORT F pin 6
PF6  PORT F pin 7, Green LED via J31
PF7  PORT F pin 8, User button via J33 (otherwise reset)

LEDs and Buttons
================

Serial Console
==============

Configurations
==============