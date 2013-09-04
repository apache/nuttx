/**************************************************************************
*
*  spi. - SPI functions to connect an Arduidno to the TI CC3000
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

#include <stdio.h>
#include <debug.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <arch/board/kl_wifi.h>
#include <nuttx/cc3000/hci.h>
#include <nuttx/cc3000/spi.h>
//#include <nuttx/cc3000/ArduinoCC3000Core.h>

// This flag lets the interrupt handler know if it should respond to
// the WL_SPI_IRQ pin going low or not
int16_t SPIInterruptsEnabled=0;


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

#define SPI_HEADER_SIZE			(5)

#define 	eSPI_STATE_POWERUP 		(0)
#define 	eSPI_STATE_INITIALIZED  	(1)
#define 	eSPI_STATE_IDLE			(2)
#define 	eSPI_STATE_WRITE_IRQ	   	(3)
#define 	eSPI_STATE_WRITE_FIRST_PORTION  (4)
#define 	eSPI_STATE_WRITE_EOT		(5)
#define 	eSPI_STATE_READ_IRQ		(6)
#define 	eSPI_STATE_READ_FIRST_PORTION	(7)
#define 	eSPI_STATE_READ_EOT		(8)

/* !!!HACK!!!*/
#define KL_PORTA_ISFR 0x400490a0
#define PIN16 16
#define getreg32(a)          (*(volatile uint32_t *)(a))
#define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))


#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif


#ifdef CONFIG_KL_SPI0
void kl_spi0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool selected)
{
  spivdbg("devid: %d CS: %s\n",
           (int)devid, selected ? "assert" : "de-assert");
}
#endif

#ifdef CONFIG_KL_SPI1
void kl_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool selected)
{
  spivdbg("devid: %d CS: %s\n",
           (int)devid, selected ? "assert" : "de-assert");
}
#endif


#ifdef CONFIG_KL_SPI0
uint8_t kl_spi0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#ifdef CONFIG_KL_SPI1
uint8_t kl_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif



typedef struct
{
	gcSpiHandleRx  SPIRxHandler;

	uint16_t usTxPacketLength;
	uint16_t usRxPacketLength;
	unsigned long  ulSpiState;
	uint8_t *pTxPacket;
	uint8_t *pRxPacket;

}tSpiInformation;


tSpiInformation sSpiInformation;

//
// Static buffer for 5 bytes of SPI HEADER
//
uint8_t tSpiReadHeader[] = {READ, 0, 0, 0, 0};


// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
// for the purpose of detection of the overrun. The location of the memory where the magic number 
// resides shall never be written. In case it is written - the overrun occured and either recevie function
// or send function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

char spi_buffer[CC3000_RX_BUFFER_SIZE];
uint8_t wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

struct spi_dev_s *spi = NULL;

unsigned int SPIPump(uint8_t data)
{
	uint8_t rx;

	printf("SPIPump tx = 0x%X ", data);

	if(!spi){
		spi = up_spiinitialize(1);
		SPI_SETBITS(spi, 8);
		SPI_SETMODE(spi, SPIDEV_MODE1);
	}

	SPI_EXCHANGE(spi, &data, &rx, 1);

	printf(" rx = 0x%X\n", rx);
	
	return rx;
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiPauseSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void SpiPauseSpi(void)
{
	SPIInterruptsEnabled = 0;
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiResumeSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void SpiResumeSpi(void)
{
	SPIInterruptsEnabled = 1;
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiTriggerRxProcessing
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************
void SpiTriggerRxProcessing(void)
{
	//
	// Trigger Rx processing
	//
	SpiPauseSpi();
	DeassertWlanCS();
        
        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
        // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun 
        // occurred - and we will stuck here forever!
	if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void SpiReadDataSynchronous(uint8_t *data, uint16_t size)
{
	long i = 0;
    uint8_t *data_to_send = tSpiReadHeader;
    	
	for (i = 0; i < size; i ++) {
		data[i] = SPIPump(data_to_send[0]);
	}
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void SpiWriteDataSynchronous(uint8_t *data, uint16_t size)
{
	
	while (size)
    	{   	
		SPIPump(*data);
		size --;
        data++;
	}
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long SpiFirstWrite(uint8_t *ucBuf, uint16_t usLength)
{
    //
    // workaround for first transaction
    //
    int i = 0, j = 1;
    AssertWlanCS();
	
    usleep(70);
    
    // SPI writes first 4 bytes of data
    SpiWriteDataSynchronous(ucBuf, 4);
    
    usleep(70);
	
    SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    
    DeassertWlanCS();

    //printf("Executed SpiFirstWrite!\n");

    return(0);
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long SpiWrite(uint8_t *pUserBuffer, uint16_t usLength)
{
    uint8_t ucPad = 0;
    
	//
	// Figure out the total length of the packet in order to figure out if there is padding or not
	//
    if(!(usLength & 0x0001))
    {
        ucPad++;
    }


    pUserBuffer[0] = WRITE;
    pUserBuffer[1] = HI(usLength + ucPad);
    pUserBuffer[2] = LO(usLength + ucPad);
    pUserBuffer[3] = 0;
    pUserBuffer[4] = 0;

    usLength += (SPI_HEADER_SIZE + ucPad);
        
        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
        // for the purpose of overrun detection. If the magic number is overwritten - buffer overrun 
        // occurred - and we will be stuck here forever!
	if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED) {
			}
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		//
		// This is time for first TX/RX transactions over SPI:
		// the IRQ is down - so need to send read buffer size command
		//
		SpiFirstWrite(pUserBuffer, usLength);
	}
	else 
	{
		//
		// We need to prevent here race that can occur in case two back to back packets are sent to the 
		// device, so the state will move to IDLE and once again to not IDLE due to IRQ
		//
		tSLInformation.WlanInterruptDisable();

		while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			;
		}

		
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		sSpiInformation.pTxPacket = pUserBuffer;
		sSpiInformation.usTxPacketLength = usLength;
		
		//
		// Assert the CS line and wait till SSI IRQ line is active and then initialize write operation
		//
		AssertWlanCS();

		//
		// Re-enable IRQ - if it was not disabled - this is not a problem...
		//
		tSLInformation.WlanInterruptEnable();

		//
		// check for a missing interrupt between the CS assertion and enabling back the interrupts
		//
		if (tSLInformation.ReadWlanInterruptPin() == 0)
		{
                	SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			DeassertWlanCS();
		}
	}


	//
	// Due to the fact that we are currently implementing a blocking situation
	// here we will wait till end of transaction
	//

	while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
		;
	
    return(0);
}


//*****************************************************************************
//
//! This function processes received SPI Header and in accordance with it - continues reading 
//!	the packet
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  ...
//
//*****************************************************************************
long SpiReadDataCont(void)
{
	long data_to_recv;
	uint8_t *evnt_buff, type;

	
    //
    //determine what type of packet we have
    //
    evnt_buff =  sSpiInformation.pRxPacket;
    data_to_recv = 0;
	STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);
	
    switch(type)
    {
        case HCI_TYPE_DATA:
        {
			//
			// We need to read the rest of data..
			//
			STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
			if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
			{	
    	        data_to_recv++;
			}

			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}
            break;
        }
        case HCI_TYPE_EVNT:
        {
			// 
			// Calculate the rest length of the data
			//
            STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
			data_to_recv -= 1;
			
			// 
			// Add padding byte if needed
			//
			if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
			{
				
	            data_to_recv++;
			}
			
			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
            break;
        }
    }
	
    return (0);
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SSIContReadOperation
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void SSIContReadOperation(void)
{
	//
	// The header was read - continue with  the payload read
	//
	if (!SpiReadDataCont())
	{
		
		
		//
		// All the data was read - finalize handling by switching to teh task
		//	and calling from task Event Handler
		//
		SpiTriggerRxProcessing();
	}
}


//*****************************************************************************
//
//! This function enter point for read flow: first we read minimal 5 SPI header bytes and 5 Event
//!	Data bytes
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void SpiReadHeader(void)
{
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}


//*****************************************************************************
// 
//!  The IntSpiGPIOHandler interrupt handler
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registrated this interrupt request
//!          it set the corresponding /CS in active state.
// 
//*****************************************************************************
//#pragma vector=PORT2_VECTOR
//__interrupt void IntSpiGPIOHandler(void)
int CC3000InterruptHandler(int irq, void *context)
{

        uint32_t regval = 0;

        regval = getreg32(KL_PORTA_ISFR);
        if (regval & (1 << PIN16))
        {
                //printf("\nAn interrupt was issued!\n");

		if (!SPIInterruptsEnabled) {
			goto out;
		}

                //printf("\nSPIInterrupt was enabled!\n");

		if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
		{
			/* This means IRQ line was low call a callback of HCI Layer to inform on event */
	 		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
		{			
			sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
			/* IRQ line goes down - start reception */
			AssertWlanCS();

			//
			// Wait for TX/RX Complete which will come as DMA interrupt
			// 
	       		SpiReadHeader();

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			
			SSIContReadOperation();
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
		{
			
			SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			DeassertWlanCS();
		}
		else {
		}
		
out:
                regval = (1 << PIN16);
                putreg32(regval, KL_PORTA_ISFR);
        }
	return 0;
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;
        
        memset(spi_buffer, 0, sizeof(spi_buffer));
        memset(wlan_tx_buffer, 0, sizeof(spi_buffer));

	sSpiInformation.SPIRxHandler = pfRxHandler;
	sSpiInformation.usTxPacketLength = 0;
	sSpiInformation.pTxPacket = NULL;
	sSpiInformation.pRxPacket = (uint8_t *)spi_buffer;
	sSpiInformation.usRxPacketLength = 0;
	spi_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
	wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

	//
	// Enable interrupt on the GPIO pin of WLAN IRQ
	//
	tSLInformation.WlanInterruptEnable();
	
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void SpiClose(void)
{
	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}

	//
	//	Disable Interrupt in GPIOA module...
	//
    tSLInformation.WlanInterruptDisable();
}

