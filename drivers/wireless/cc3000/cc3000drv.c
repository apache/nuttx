/**************************************************************************
 *  drivers/wireless/cc3000/cc3000drv.c - Driver wrapper functions to
 *  conntect nuttx to the TI CC3000
 *
 *  Port to nuttx:
 *      David Sidrane <david_s5@nscdg.com>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

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
#include "cc3000drv.h"

#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/cc3000_common.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

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
  cc3000_buffer_desc rx_buffer;
  mqd_t  queue;
  sem_t *done;
  sem_t unsoliced_thread_wakesem;
} spiconf =
{
  -1,
};

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: cc3000_resume
 *
 * Description:
 *   Will re enable the to deliver messages
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void cc3000_resume(void)
{
  DEBUGASSERT(spiconf.cc3000fd >= 0 && spiconf.done);
  sem_post(spiconf.done);
  nllvdbg("Done\n");
}

/*****************************************************************************
 * Name: cc3000_write
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

long cc3000_write(uint8_t *pUserBuffer, uint16_t usLength)
{
  DEBUGASSERT(spiconf.cc3000fd >= 0);
  return write(spiconf.cc3000fd,pUserBuffer,usLength) == usLength ? 0 : -errno;
}

/*****************************************************************************
 * Name: cc3000_read
 *
 * Description:
 *   This function enter point for read flow. This function will block the
 *   caller until there is data available
 *
 * Input Parameters:
 *   pUserBuffer
 *   usLength
 *
 * Returned Value:
 *
 *****************************************************************************/

long cc3000_read(uint8_t *pUserBuffer, uint16_t usLength)
{
  DEBUGASSERT(spiconf.cc3000fd >= 0);
  return read(spiconf.cc3000fd,pUserBuffer,usLength);
}

/*****************************************************************************
 * Name: cc3000_wait
 *
 * Description:
 *      Waits on a message from the driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *
 *****************************************************************************/

uint8_t *cc3000_wait(void)
{
  DEBUGASSERT(spiconf.cc3000fd >= 0);

  mq_receive(spiconf.queue, &spiconf.rx_buffer, sizeof(spiconf.rx_buffer), 0);
  return spiconf.rx_buffer.pbuffer;
}

/*****************************************************************************
 * Name: unsoliced_thread_func
 *
 * Description:
 *   This is the thread for unsolicited events. This function will block the
 *   caller until there is data available
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

  sem_post(&spiconf.unsoliced_thread_wakesem);

  while (spiconf.run)
    {
      memset(&spiconf.rx_buffer,0,sizeof(spiconf.rx_buffer));
      nbytes = mq_receive(spiconf.queue, &spiconf.rx_buffer,
                          sizeof(spiconf.rx_buffer), 0);
      if (nbytes > 0)
        {
          nlldbg("%d Processed\n",nbytes);
          spiconf.pfRxHandler(spiconf.rx_buffer.pbuffer);
        }
    }

  mq_close(spiconf.queue);
  sem_close(spiconf.done);
  pthread_exit((pthread_addr_t)status);
  return (pthread_addr_t)status;
}

/*****************************************************************************
 * Name: cc3000_open
 *
 * Description:
 *   Open the cc3000 driver
 *
 * Input Parameters:
 *   pfRxHandler the Rx handler for messages
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void cc3000_open(gcSpiHandleRx pfRxHandler)
{
  int status;
  int fd;

  DEBUGASSERT(spiconf.cc3000fd == -1);

  fd = open("/dev/wireless0",O_RDWR|O_BINARY);
  if (fd >= 0)
    {
      spiconf.pfRxHandler = pfRxHandler;
      spiconf.cc3000fd = fd;
      spiconf.run = true;

      sem_init(&spiconf.unsoliced_thread_wakesem, 0, 0);

      pthread_attr_t attr;
      struct sched_param param;
      pthread_attr_init(&attr);
      attr.stacksize = CONFIG_CC3000_UNSOLICED_STACKSIZE;
      param.sched_priority = SCHED_PRIORITY_DEFAULT-10;
      pthread_attr_setschedparam(&attr, &param);
      status = pthread_create(&spiconf.unsoliced_thread, &attr,
                              unsoliced_thread_func, NULL);
      DEBUGASSERT(status == 0);
      UNUSED(status);

      /* Wait unsoliced_thread to wake-up. */

      while (sem_wait(&spiconf.unsoliced_thread_wakesem) != 0)
        {
          ASSERT(errno == EINTR);
        }
   }

  DEBUGASSERT(spiconf.cc3000fd >= 0);
}

/*****************************************************************************
 * Name: cc3000_close
 *
 * Description:
 *   Close the cc3000 driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

void cc3000_close(void)
{
  if (spiconf.cc3000fd >= 0)
    {
      int status;
      spiconf.run = false;

      pthread_cancel(spiconf.unsoliced_thread);
      pthread_join(spiconf.unsoliced_thread, (pthread_addr_t*)&status);

      close(spiconf.cc3000fd);

      memset(&spiconf, 0, sizeof(spiconf));
      spiconf.cc3000fd = -1;
    }
}

/****************************************************************************
 * Name: cc3000_wait_data
 *
 * Description:
 *   Adds this socket for monitoring for the data operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle or -1 tp remove it
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

int cc3000_wait_data(int sockfd)
{
  int rv = sockfd;

  DEBUGASSERT(spiconf.cc3000fd >= 0);
  ioctl(spiconf.cc3000fd, CC3000IOC_SELECTDATA, (unsigned long)&rv);
  return rv;
}

/****************************************************************************
 * Name: cc3000_accept_socket
 *
 * Description:
 *   Adds this socket for monitoring for the accept operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle or -1 tp remove it
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

int to_cc3000_accept_socket(int sockfd, struct sockaddr *addr, socklen_t *addrlen);

int cc3000_accept_socket(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
  //return to_cc3000_accept_socket(sockfd, addr,addrlen);
  DEBUGASSERT(spiconf.cc3000fd >= 0);

  cc3000_acceptcfg cfg;
  cfg.sockfd = sockfd;
  cfg.addr = addr;
  cfg.addrlen = addrlen;
  ioctl(spiconf.cc3000fd, CC3000IOC_SELECTACCEPT, (unsigned long)&cfg);
  return cfg.sockfd;
}

/****************************************************************************
 * Name: cc3000_add_socket
 *
 * Description:
 *   Adds a socket to the list for monitoring for long operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

int cc3000_add_socket(int sockfd)
{
  int rv = sockfd;

  DEBUGASSERT(spiconf.cc3000fd >= 0);
  ioctl(spiconf.cc3000fd, CC3000IOC_ADDSOCKET, (unsigned long)&rv);
  return rv;
}

/****************************************************************************
 * Name: cc3000_remove_socket
 *
 * Description:
 *   Removes a socket from the list of monitoring for long operation
 *
 * Input Parameters:
 *   sd      cc3000 socket handle
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

int cc3000_remove_socket(int sockfd)
{
  int rv = sockfd;

  DEBUGASSERT(spiconf.cc3000fd >= 0);
  ioctl(spiconf.cc3000fd, CC3000IOC_REMOVESOCKET, (unsigned long)&rv);
  return rv;
}

/****************************************************************************
 * Name: cc3000_remote_closed_socket
 *
 * Description:
 *   Mark socket as closed by remote host
 *
 * Input Parameters:
 *   sd        cc3000 socket handle
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a -1 value is
 *   returned to indicate socket not found.
 *
 ****************************************************************************/

int cc3000_remote_closed_socket(int sockfd)
{
  int rv = sockfd;

  DEBUGASSERT(spiconf.cc3000fd >= 0);
  ioctl(spiconf.cc3000fd, CC3000IOC_REMOTECLOSEDSOCKET, (unsigned long)&rv);
  return rv;
}
