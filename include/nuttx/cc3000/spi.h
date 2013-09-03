/**************************************************************************
*
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




typedef void (*gcSpiHandleRx)(void *p);

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

void SpiOpen(gcSpiHandleRx pfRxHandler);

void SpiClose(void);

long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);

void SpiResumeSpi(void);

int CC3000InterruptHandler(int irq, void *context);

short SPIInterruptsEnabled;

unsigned char wlan_tx_buffer[];
