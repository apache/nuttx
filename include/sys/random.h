/****************************************************************************
 * include/sys/random.h
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SYS_RANDOM_H
#define __INCLUDE_SYS_RANDOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flags for getrandom(2)
 *
 * GRND_NONBLOCK  Don't block and return EAGAIN instead
 * GRND_RANDOM    No effect
 * GRND_INSECURE  Return non-cryptographic random bytes
 */

#define GRND_NONBLOCK   0x0001
#define GRND_RANDOM     0x0002
#define GRND_INSECURE   0x0004

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RANDOM_POOL

/****************************************************************************
 * Name: getrandom
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This is the
 *   preferred interface for getting random numbers. The traditional
 *   /dev/random approach is susceptible for things like the attacker
 *   exhausting file descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Input Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *   flags  - Bit mask that can contain zero or more of the ORed values
 *            together.
 *
 * Returned Value:
 *   On success, getrandom() returns the number of bytes that were copied
 *   to the buffer buf.  This may be less than the number of bytes
 *   requested via buflen if either GRND_RANDOM was specified in flags and
 *   insufficient entropy was present in the random source or the system
 *   call was interrupted by a signal.
 *
 *   On error, -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

ssize_t getrandom(FAR void *bytes, size_t nbytes, unsigned int flags);

#endif /* CONFIG_CRYPTO_RANDOM_POOL */

#endif /* __INCLUDE_SYS_RANDOM_H */
