/****************************************************************************
 * examples/usbstorage/usbstrg.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_EXAMPLES_USBSTRG_NLUNS
#  define CONFIG_EXAMPLES_USBSTRG_NLUNS 1
#endif

#ifndef CONFIG_EXAMPLES_USBSTRG_DEVMINOR1
#  define CONFIG_EXAMPLES_USBSTRG_DEVMINOR1 0
#endif

#ifndef CONFIG_EXAMPLES_USBSTRG_DEVPATH1
#  define CONFIG_EXAMPLES_USBSTRG_DEVPATH1 "/dev/mmcsd0"
#endif

#if CONFIG_EXAMPLES_USBSTRG_NLUNS > 1
#  ifndef CONFIG_EXAMPLES_USBSTRG_DEVMINOR2
#    error "CONFIG_EXAMPLES_USBSTRG_DEVMINOR2 for LUN=2"
#  endif
#  ifndef CONFIG_EXAMPLES_USBSTRG_DEVPATH2
#    error "CONFIG_EXAMPLES_USBSTRG_DEVPATH2 for LUN=2"
#  endif
#  if CONFIG_EXAMPLES_USBSTRG_NLUNS > 2
#    ifndef CONFIG_EXAMPLES_USBSTRG_DEVMINOR3
#      error "CONFIG_EXAMPLES_USBSTRG_DEVMINOR2 for LUN=3"
#    endif
#    ifndef CONFIG_EXAMPLES_USBSTRG_DEVPATH2
#      error "CONFIG_EXAMPLES_USBSTRG_DEVPATH2 for LUN=3"
#    endif
#    if CONFIG_EXAMPLES_USBSTRG_NLUNS > 3
#      error "CONFIG_EXAMPLES_USBSTRG_NLUNS must be {1,2,3}"
#    endif
#  endif
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbstrg_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

extern int usbstrg_archinitialize(void);
