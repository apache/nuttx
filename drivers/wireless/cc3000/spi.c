/**************************************************************************
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <debug.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "spi.h"

#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/cc3000_common.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#ifndef CONFIG_CC3000_UNSOLICED_STACKSIZE
#  define CONFIG_CC3000_UNSOLICED_STACKSIZE 264
#endif

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

/*****************************************************************************
 * Private Types
 *****************************************************************************/

static struct
{
  int cc3000fd;
  gcSpiHandleRx pfRxHandler;
  pthread_t unsoliced_thread;
  bool run;
  uint8_t rx_buffer[CC3000_RX_BUFFER_SIZE];
  mqd_t  queue;
  sem_t *done;
} spiconf;

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: SpiResumeSpi
 *
 * Description:
 *   Will re enable the SPI_IRQ'a ability to create interrupts. It is used to
 *   resume processing after the code passed to SpiOpen is Called
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void SpiResumeSpi(void)
{
  DEBUGASSERT(spiconf.cc3000fd && spiconf.done);
  sem_post(spiconf.done);
  nllvdbg("Done\n");
}

/*****************************************************************************
 * Name: SpiWrite
 *
 * Description:
 *   This function enter point for write flow
 *
 * Input Parameters:
 *   pUserBuffer
 *   usLength
 *
 * Returned Value:
 *
 *****************************************************************************/

long SpiWrite(uint8_t *pUserBuffer, uint16_t usLength)
{
  DEBUGASSERT(spiconf.cc3000fd);
  return write(spiconf.cc3000fd,pUserBuffer,usLength) == usLength ? 0 : -errno;
}

/*****************************************************************************
 * Name: SpiRead
 *
 * Description:
 *   This function enter point for read flow. This function will block the
 *   caller untinlthere is data Available
 *
 * Input Parameters:
 *   pUserBuffer
 *   usLength
 *
 * Returned Value:
 *
 *****************************************************************************/

long SpiRead(uint8_t *pUserBuffer, uint16_t usLength)
{
  DEBUGASSERT(spiconf.cc3000fd);
  return read(spiconf.cc3000fd,pUserBuffer,usLength);
}

/*****************************************************************************
 * Name: SpiRead
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *
 *****************************************************************************/

uint8_t *SpiWait(void)
{
  DEBUGASSERT(spiconf.cc3000fd);

  mq_receive(spiconf.queue, spiconf.rx_buffer, CC3000_RX_BUFFER_SIZE, 0);
  return spiconf.rx_buffer;
}

/*****************************************************************************
 * Name: unsoliced_thread_func
 *
 * Description:
 *   This is the thread for unsolicited events. This function will block the
 *   caller untinlthere is data Available
 *
 * Input Parameters:
 *   parameter
 *
 * Returned Value:
 *
 *****************************************************************************/

static void *unsoliced_thread_func(void *parameter)
{
  char buff[QUEUE_NAMELEN];
  int status = 0;
  int nbytes = 0;
  int minor = 0;

  ioctl(spiconf.cc3000fd, CC3000IOC_GETQUESEMID, (unsigned long)&minor);
  snprintf(buff, QUEUE_NAMELEN, QUEUE_FORMAT, minor);
  spiconf.queue = mq_open(buff,O_RDONLY);
  DEBUGASSERT(spiconf.queue != (mqd_t) -1);
  DEBUGASSERT(SEM_NAMELEN == QUEUE_NAMELEN);
  snprintf(buff, SEM_NAMELEN, SEM_FORMAT, minor);
  spiconf.done = sem_open(buff,O_RDONLY);
  DEBUGASSERT(spiconf.done != (sem_t *)-1);

  while(spiconf.run)
    {
      memset(spiconf.rx_buffer,0,sizeof(spiconf.rx_buffer));
      nbytes = mq_receive(spiconf.queue, spiconf.rx_buffer,
                          CC3000_RX_BUFFER_SIZE, 0);
      if (nbytes > 0)
        {
          nlldbg("%d Processed\n",nbytes);
          spiconf.pfRxHandler(spiconf.rx_buffer);
        }
    }

  mq_close(spiconf.queue);
  sem_close(spiconf.done);
  pthread_exit((pthread_addr_t)status);
  return (pthread_addr_t)status;
}

/*****************************************************************************
 * Name: SpiOpen
 *
 * Description:
 *   Configure the SPI
 *
 * Input Parameters:
 *   pfRxHandler the Rx handler for SPI
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void SpiOpen(gcSpiHandleRx pfRxHandler)
{
  int status;
  int fd;

  DEBUGASSERT(spiconf.cc3000fd == 0);

  fd = open("/dev/wireless0",O_RDWR|O_BINARY);
  if (fd > 0)
    {
      spiconf.pfRxHandler = pfRxHandler;
      spiconf.cc3000fd = fd;
      spiconf.run = true;

      pthread_attr_t attr;
      struct sched_param param;
      pthread_attr_init(&attr);
      attr.stacksize = CONFIG_CC3000_UNSOLICED_STACKSIZE;
      param.sched_priority = SCHED_PRIORITY_DEFAULT-10;
      pthread_attr_setschedparam(&attr, &param);
      status = pthread_create(&spiconf.unsoliced_thread, &attr,
                              unsoliced_thread_func, NULL);
      DEBUGASSERT(status == 0)
   }

  DEBUGASSERT(spiconf.cc3000fd);
}

/*****************************************************************************
 * Name: SpiClose
 *
 * Description:
 *   Configure the SPI
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void SpiClose(void)
{
  if (spiconf.cc3000fd)
    {
      int status;
      spiconf.run = false;

      pthread_cancel(spiconf.unsoliced_thread);
      pthread_join(spiconf.unsoliced_thread, (pthread_addr_t*)&status);

      close(spiconf.cc3000fd);
      spiconf.cc3000fd = 0;
    }
}
