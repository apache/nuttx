/**************************************************************************
 *  ArduinoCC3000SPI.h - SPI functions to connect an Arduidno to the TI
 *                       CC3000
 *
 *  This code uses the Arduino hardware SPI library (or a bit-banged
 *  SPI for the Teensy 3.0) to send & receive data between the library
 *  API calls and the CC3000 hardware. Every
 *
 *  Version 1.0.1b
 *
 *  Copyright (C) 2013 Chris Magagna - cmagagna@yahoo.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  Don't sue me if my code blows up your board and burns down your house
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_CC3000_SPI_H
#define __DRIVERS_WIRELESS_CC3000_SPI_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdint.h>

/*****************************************************************************
 * Public Types
 *****************************************************************************/

typedef void (*gcSpiHandleRx)(void *p);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

extern uint16_t SPIInterruptsEnabled;
extern uint8_t wlan_tx_buffer[];

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

void SpiOpen(gcSpiHandleRx pfRxHandler);
void SpiClose(void);
long SpiWrite(uint8_t *pUserBuffer, uint16_t usLength);
long SpiRead(uint8_t *pUserBuffer, uint16_t usLength);
void SpiResumeSpi(void);
int CC3000InterruptHandler(int irq, void *context);

#endif /* __DRIVERS_WIRELESS_CC3000_SPI_H */
