/**************************************************************************
 *  drivers/wireless/cc3000/cc3000drv.h - Driver wrapper functions to
 *  connect nuttx to the TI CC3000
 *
 *   Port to nuttx:
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

#ifndef __DRIVERS_WIRELESS_CC3000_CC3000DRV_H
#define __DRIVERS_WIRELESS_CC3000_CC3000DRV_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdint.h>
#include <sys/socket.h>

/*****************************************************************************
 * Public Types
 *****************************************************************************/

typedef void (*gcSpiHandleRx)(void *p);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void cc3000_open(gcSpiHandleRx pfRxHandler);
void cc3000_close(void);
long cc3000_write(uint8_t *pUserBuffer, uint16_t usLength);
uint8_t *cc3000_wait(void);
long cc3000_read(uint8_t *pUserBuffer, uint16_t usLength);
void cc3000_resume(void);

int cc3000_wait_data(int sockfd);
int cc3000_accept_socket(int sockfd, struct sockaddr *addr,
                         socklen_t *addrlen);
int cc3000_add_socket(int sockfd);
int cc3000_remove_socket(int sockfd);
int cc3000_remote_closed_socket(int sockfd);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_CC3000_CC3000DRV_H */
