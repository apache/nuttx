The case of ARM CM4 & cxd32xx @NuttX12.4.0
==========================================

Step1 : Add the SoC
-------------------

Create the SoC directory.

.. code-block::

  $ touch arch/arm/include/cxd32xx
  $ touch arch/arm/src/cxd32xx

Modify "arch/arm/Kconfig" to register the SoC directory.

Add "Kconfig" and "Make.defs" for the SoC directory.

.. code-block::

  $ touch arch/arm/src/cxd32xx/Kconfig
  $ touch arch/arm/src/cxd32xx/Make.defs

For details about these files, see `apache/nuttx/pull/12441`_ 

Step2 : Add the Board
---------------------

There are two ways about adding the Board directory.
The one is in-of-tree, another one is out-of-tree.
For details see :doc:`/guides/customboards`.
And if you wanted to add own apps, see :doc:`/guides/customapps`.

This time, the way of out-of-tree is selected.
The directory structure is following :

.. code-block::

  apps/
  nuttx/
  out-of-tree/cxd32xx_boards

Create the board : cb2sdb2 as the board of cxd32xx.

.. code-block::

  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/configs/nsh
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/include
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/scripts
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/src

Add "Kconfig", "defconfig", "Make.defs" and "Makefile".

.. code-block::

  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/Kconfig
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/configs/nsh/defconfig
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/scripts/Make.defs
  $ touch out-of-tree/cxd32xx_boards/cb2sdb2/src/Makefile

For details about these files, see :ref:`out-of-tree`.

Step3 : Configure
-----------------

Now you can do configure by following commands.
It is important to confirm these commands are working.

.. code-block::

  $ cd nuttx
  $ ./tools/configure.sh -l ../out-of-tree/cxd32xx_boards/cb2sdb2/configs/nsh

And you can do menuconfig by following command.

.. code-block::

  $ make menuconfig

And you can do distclean by following command.

.. code-block::

  $ make distclean

Step4 : Compile
---------------

Try to build with the implementation of Step1-2, you would face the error as following
if you implemented only directory / Kconfig / Make.defs / Makefile / defconfig about Step1-2.

.. code-block::

  $ make
  Create version.h
  LN: platform/board to /home/lassdk/workdir/nuttx1/apps/platform/dummy
  In file included from /home/lassdk/workdir/nuttx1/nuttx/include/nuttx/irq.h:37:0,
                   from /home/lassdk/workdir/nuttx1/nuttx/include/nuttx/sched.h:40,
                   from /home/lassdk/workdir/nuttx1/nuttx/include/nuttx/arch.h:87,
                   from ./clock/clock_initialize.c:37:
  /home/lassdk/workdir/nuttx1/nuttx/include/arch/irq.h:43:27: 
  fatal error: arch/chip/irq.h: No such file or directory
  compilation terminated.
  ERROR: arm-none-eabi-gcc failed: 1
         command: arm-none-eabi-gcc -MT ./clock_initialize.o  -M '-Wstrict-prototypes'
          '-Wno-attributes' '-Wno-unknown-pragmas' '-Wno-psabi' '-Os'
           '-fno-strict-aliasing' '-fomit-frame-pointer' '-fno-common' '-Wall'
           '-Wshadow' '-Wundef' 
           '-ffunction-sections' '-fdata-sections' '-mlittle-endian' 
           '-march=armv7e-m' '-mtune=cortex-m4' 
            '-mfpu=fpv4-sp-d16' '-mfloat-abi=hard' '-mthumb' '-Wa,-mthumb' 
            '-Wa,-mimplicit-it=always' '-isystem' 
            '/home/lassdk/workdir/nuttx1/nuttx/include'
             '-D__NuttX__' '-DNDEBUG' '-D__KERNEL__' '-pipe' '-I' 
             '/home/lassdk/workdir/nuttx1/nuttx/sched' ./clock/clock_initialize.c
  make[2]: *** [/home/lassdk/workdir/nuttx1/nuttx/tools/Config.mk:230: 
  clock_initialize.ddc] Error 1
  make[1]: *** [Makefile:71: .depend] Error 2
  make: *** [tools/Unix.mk:619: pass2dep] Error 2

Try to add "irq.h" with referring to cxd56xx and retry to build.
This time, the build system will teach that you have to implement "chip.h" as following.

.. code-block::

  $ make
  CC:  clock/clock_initialize.c In file included from
   /home/lassdk/workdir/nuttx2/nuttx/include/arch/armv7-m/irq.h:37:0,
                   from /home/lassdk/workdir/nuttx2/nuttx/include/arch/irq.h:56,
                   from /home/lassdk/workdir/nuttx2/nuttx/include/nuttx/irq.h:37,
                   from /home/lassdk/workdir/nuttx2/nuttx/include/nuttx/sched.h:40,
                   from /home/lassdk/workdir/nuttx2/nuttx/include/nuttx/arch.h:87,
                   from clock/clock_initialize.c:37:
  /home/lassdk/workdir/nuttx2/nuttx/include/arch/armv7-m/nvicpri.h:28:28
   fatal error: arch/chip/chip.h: No such file or directory
  compilation terminated.
  make[1]: *** [Makefile:61: clock_initialize.o] Error 1
  make: *** [tools/LibTargets.mk:65: sched/libsched.a] Error 2

Continue this try and error until pass the Compiling.

Step5 : Link
------------

After the pass of Compiling, you will get following error.
To resolve this error, you have to implement the linker script.

.. code-block::

  $ make
  make[1]: *** No rule to make target 
  '/home/lassdk/workdir/nuttx1/nuttx/../cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld', 
  needed by '/home/lassdk/workdir/nuttx1/nuttx/../cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld.tmp'.
    Stop.
  make: *** [tools/Unix.mk:545: nuttx] Error 2

Try to add the linker script with referring to some board(cxd56xx, lpc43xx, etc) or :ref:`out-of-tree`
and retry to build, the build system will teach that you have to implement some symbols as following.

.. code-block::

  $ make
  CPP:  /home/lassdk/workdir/nuttx1/nuttx/../cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld->
   /home/lassdk/workdir/nuttx1/nuttx/../boarLD: nuttx
  arm-none-eabi-ld: warning: cannot find entry symbol __start; defaulting to 24011000
  /home/lassdk/workdir/nuttx1/nuttx/staging/libarch.a(arm_vectors.o):(.vectors+0x4): 
  undefined reference to `__start'
  /home/lassdk/workdir/nuttx1/nuttx/staging/libarch.a(arm_doirq.o): In function `arm_doirq':
  arm_doirq.c:(.text.arm_doirq+0x12): undefined reference to `arm_ack_irq'
  /home/lassdk/workdir/nuttx1/nuttx/staging/libdrivers.a(syslog_channel.o): 
  In function `syslog_default_putc':
  syslog_channel.c:(.text.syslog_default_putc+0x2): undefined reference to `up_putc'
  /home/lassdk/workdir/nuttx1/nuttx/staging/libarch.a(arm_nputs.o): In function `up_nputs':
  arm_nputs.c:(.text.up_nputs+0x10): undefined reference to `up_putc'
  make[1]: *** [Makefile:197: nuttx] Error 1
  make: *** [tools/Unix.mk:545: nuttx] Error 2

Step6 : Implement
-----------------

There are some implementation items, major items are bellow listed. To pass the Linking,
try to implement these symbols.

+-----------------+------------------------+--------------------------------------------------------------------+
| Category        | Item                   | Comment                                                            |
+-----------------+------------------------+--------------------------------------------------------------------+
| SoC directory   | entory point : __start | `apache/nuttx/pull/12441`_                                         |
+                 +------------------------+                                                                    +
|                 | Serial Driver          |                                                                    |
+                 +------------------------+                                                                    +
|                 | IDLE task stack        |                                                                    |
+                 +------------------------+                                                                    +
|                 | INTC                   |                                                                    |
+                 +------------------------+                                                                    +
|                 | Timer                  |                                                                    |
+-----------------+------------------------+------------------------------------------------+-------------------+
| Board directory | board_app_initialize   | :doc:`/applications/nsh/customizing`           |:ref:`out-of-tree` |
+                 +------------------------+------------------------------------------------+                   |
|                 |<arch>_board_initialize | `sched/Kconfig`_ :BOARD_<EARLY/LATE>_INITIALIZE|                   |
+-----------------+------------------------+------------------------------------------------+-------------------+

.. _apache/nuttx/pull/12441: https://github.com/apache/nuttx/pull/12441
.. _sched/Kconfig: https://github.com/apache/nuttx/blob/master/sched/Kconfig

If got the pass of Linking, you would see following result.

.. code-block::

  $ make
  Create version.h
  LN: platform/board to /home/lassdk/workdir/nuttx2/apps/platform/dummy
  Register: nsh
  Register: sh
  CPP:  /home/lassdk/workdir/nuttx2/nuttx/../boards/cb2sdb2/scripts/cb2sdb2.ld->
   /home/lassdk/workdir/nuttx2/nuttx/../boarLD: nuttx

Step7 : Verify
--------------

To execute and debug on the evaluation board, add following configurations.

.. code-block::

  CONFIG_RAW_BINARY=y
  CONFIG_DEBUG_SYMBOLS=y

If your implementations were good, you would see following logs.

.. code-block::

  NuttShell (NSH) NuttX-12.4.0
  nsh>
  nsh>
  nsh>

After confirming to boot NuttShell(NSH), try to execute "apps/testing/ostest".
The "apps/testing/ostest" needs following configurations.

.. code-block::

  CONFIG_BUILTIN=y
  CONFIG_NSH_BUILTIN_APPS=y
  CONFIG_SCHED_WAITPID=y
  CONFIG_TESTING_OSTEST=y

And one more test is recommended to check the timer implementation whether
the kernel could count the time accurately or not.
I checked it by following code. About the function : cxd32_timerget(),
see `apache/nuttx/pull/12441`_ 

.. code-block::

  extern uint32_t cxd32_timerget(void);

    {
      uint32_t pre;
      uint32_t post;

      pre = cxd32_timerget();
      sleep(1);
      post = cxd32_timerget();
      printf("pre:%lu post:%lu exptect:1sec\n", pre, post);

      pre = cxd32_timerget();
      sleep(10);
      post = cxd32_timerget();
      printf("pre:%lu post:%lu exptect:10sec\n", pre, post);
    }

.. _out-of-tree:

Appendix : out-of-tree code
---------------------------

.. code-block::

  diff --git a/cxd32xx_apps/Make.defs b/cxd32xx_apps/Make.defs
  new file mode 100644
  index 0000000..2d6b86e
  --- /dev/null
  +++ b/cxd32xx_apps/Make.defs
  @@ -0,0 +1 @@
  +include $(wildcard $(APPDIR)/cxd32xx_apps/*/Make.defs)
  diff --git a/cxd32xx_apps/Makefile b/cxd32xx_apps/Makefile
  new file mode 100644
  index 0000000..e451b29
  --- /dev/null
  +++ b/cxd32xx_apps/Makefile
  @@ -0,0 +1,3 @@
  +MENUDESC = "cxd32xx_apps"
  +
  +include $(APPDIR)/Directory.mk
  diff --git a/cxd32xx_apps/hello/Kconfig b/cxd32xx_apps/hello/Kconfig
  new file mode 100644
  index 0000000..7f0b41e
  --- /dev/null
  +++ b/cxd32xx_apps/hello/Kconfig
  @@ -0,0 +1,29 @@
  +#
  +# For a description of the syntax of this configuration file,
  +# see the file kconfig-language.txt in the NuttX tools repository.
  +#
  +
  +config CXD32XX_APPS_HELLO
  +    tristate "CXD32XX Hello App"
  +    default n
  +    ---help---
  +        Enable the CXD32XX Hello App
  +
  +if CXD32XX_APPS_HELLO
  +
  +config CXD32XX_APPS_HELLO_PROGNAME
  +    string "Program name"
  +    default "hello"
  +    ---help---
  +        This is the name of the program that will be used when the NSH ELF
  +        program is installed.
  +
  +config CXD32XX_APPS_HELLO_PRIORITY
  +    int "CXD32XX Hello task priority"
  +    default 100
  +
  +config CXD32XX_APPS_HELLO_STACKSIZE
  +    int "CXD32XX Hello stack size"
  +    default DEFAULT_TASK_STACKSIZE
  +
  +endif
  diff --git a/cxd32xx_apps/hello/Make.defs b/cxd32xx_apps/hello/Make.defs
  new file mode 100644
  index 0000000..730329a
  --- /dev/null
  +++ b/cxd32xx_apps/hello/Make.defs
  @@ -0,0 +1,3 @@
  +ifneq ($(CONFIG_CXD32XX_APPS_HELLO),)
  +CONFIGURED_APPS += $(APPDIR)/cxd32xx_apps/hello
  +endif
  diff --git a/cxd32xx_apps/hello/Makefile b/cxd32xx_apps/hello/Makefile
  new file mode 100644
  index 0000000..b3438e1
  --- /dev/null
  +++ b/cxd32xx_apps/hello/Makefile
  @@ -0,0 +1,14 @@
  +include $(APPDIR)/Make.defs
  +
  +# Custom Hello built-in application info
  +
  +PROGNAME = $(CONFIG_CXD32XX_APPS_HELLO_PROGNAME)
  +PRIORITY = $(CONFIG_CXD32XX_APPS_HELLO_PRIORITY)
  +STACKSIZE = $(CONFIG_CXD32XX_APPS_HELLO_STACKSIZE)
  +MODULE = $(CONFIG_CXD32XX_APPS_HELLO)
  +
  +# Custom Hello
  +
  +MAINSRC = hello.c
  +
  +include $(APPDIR)/Application.mk
  diff --git a/cxd32xx_apps/hello/hello.c b/cxd32xx_apps/hello/hello.c
  new file mode 100644
  index 0000000..b14a762
  --- /dev/null
  +++ b/cxd32xx_apps/hello/hello.c
  @@ -0,0 +1,24 @@
  +/****************************************************************************
  + * cxd32xx_apps/hello/hello.c
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Included Files
  + ****************************************************************************/
  +
  +#include <nuttx/config.h>
  +#include <stdio.h>
  +
  +/****************************************************************************
  + * Public Functions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * hello_main
  + ****************************************************************************/
  +
  +int main(int argc, FAR char *argv[])
  +{
  +  printf("Hello, World!!\n");
  +  return 0;
  +}
  diff --git a/cxd32xx_apps/time/Kconfig b/cxd32xx_apps/time/Kconfig
  new file mode 100644
  index 0000000..64e7591
  --- /dev/null
  +++ b/cxd32xx_apps/time/Kconfig
  @@ -0,0 +1,28 @@
  +#
  +# For a description of the syntax of this configuration file,
  +# see the file kconfig-language.txt in the NuttX tools repository.
  +#
  +
  +config CXD32XX_APPS_TIME
  +    tristate "CXD32XX Time App"
  +    default n
  +    ---help---
  +        Enable the CXD32XX Time App
  +
  +if CXD32XX_APPS_HELLO
  +
  +config CXD32XX_APPS_TIME_PROGNAME
  +    string "Program name"
  +    default "time"
  +    ---help---
  +        This apps measures the elapsed time of sleep system call to confirm the timer implementation.
  +
  +config CXD32XX_APPS_TIME_PRIORITY
  +    int "CXD32XX Time task priority"
  +    default 100
  +
  +config CXD32XX_APPS_TIME_STACKSIZE
  +    int "CXD32XX Time stack size"
  +    default DEFAULT_TASK_STACKSIZE
  +
  +endif
  diff --git a/cxd32xx_apps/time/Make.defs b/cxd32xx_apps/time/Make.defs
  new file mode 100644
  index 0000000..cdfb4cc
  --- /dev/null
  +++ b/cxd32xx_apps/time/Make.defs
  @@ -0,0 +1,3 @@
  +ifneq ($(CONFIG_CXD32XX_APPS_TIME),)
  +CONFIGURED_APPS += $(APPDIR)/cxd32xx_apps/time
  +endif
  diff --git a/cxd32xx_apps/time/Makefile b/cxd32xx_apps/time/Makefile
  new file mode 100644
  index 0000000..94c9007
  --- /dev/null
  +++ b/cxd32xx_apps/time/Makefile
  @@ -0,0 +1,14 @@
  +include $(APPDIR)/Make.defs
  +
  +# Custom Hello built-in application info
  +
  +PROGNAME = $(CONFIG_CXD32XX_APPS_TIME_PROGNAME)
  +PRIORITY = $(CONFIG_CXD32XX_APPS_TIME_PRIORITY)
  +STACKSIZE = $(CONFIG_CXD32XX_APPS_TIME_STACKSIZE)
  +MODULE = $(CONFIG_CXD32XX_APPS_TIME)
  +
  +# Custom Hello
  +
  +MAINSRC = time.c
  +
  +include $(APPDIR)/Application.mk
  diff --git a/cxd32xx_apps/time/time.c b/cxd32xx_apps/time/time.c
  new file mode 100644
  index 0000000..9775ef2
  --- /dev/null
  +++ b/cxd32xx_apps/time/time.c
  @@ -0,0 +1,41 @@
  +/****************************************************************************
  + * cxd32xx_apps/time/time.c
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Included Files
  + ****************************************************************************/
  +
  +#include <nuttx/config.h>
  +#include <stdio.h>
  +
  +/****************************************************************************
  + * Public Functions
  + ****************************************************************************/
  +
  +extern uint32_t cxd32_timerget(void);
  +
  +/****************************************************************************
  + * hello_main
  + ****************************************************************************/
  +
  +int main(int argc, FAR char *argv[])
  +{
  +  uint32_t pre;
  +  uint32_t post;
  +
  +  printf("measures the elapsed time of sleep system call to confirm \
  +  the timer implementation.\n");
  +
  +  pre = cxd32_timerget();
  +  sleep(1);
  +  post = cxd32_timerget();
  +  printf("pre:%lu post:%lu exptect:1sec\n", pre, post);
  +
  +  pre = cxd32_timerget();
  +  sleep(10);
  +  post = cxd32_timerget();
  +  printf("pre:%lu post:%lu exptect:10sec\n", pre, post);
  +
  +  return 0;
  +}
  diff --git a/cxd32xx_boards/cb2sdb2/Kconfig b/cxd32xx_boards/cb2sdb2/Kconfig
  new file mode 100644
  index 0000000..f72f3c0
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/Kconfig
  @@ -0,0 +1,4 @@
  +#
  +# For a description of the syntax of this configuration file,
  +# see the file kconfig-language.txt in the NuttX tools repository.
  +#
  diff --git a/cxd32xx_boards/cb2sdb2/configs/nsh/defconfig b/cxd32xx_boards/cb2sdb2/configs/nsh/defconfig
  new file mode 100644
  index 0000000..1d4c3e2
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/configs/nsh/defconfig
  @@ -0,0 +1,38 @@
  +#
  +# This file is autogenerated: PLEASE DO NOT EDIT IT.
  +#
  +# You can use "make menuconfig" to make any modifications to the installed .config file.
  +# You can then do "make savedefconfig" to generate a new defconfig file that includes your
  +# modifications.
  +#
  +CONFIG_ARCH="arm"
  +CONFIG_ARCH_CORTEXM4=y
  +CONFIG_ARCH_FPU=y
  +CONFIG_ARCH_CHIP="cxd32xx"
  +CONFIG_ARCH_CHIP_CXD32XX=y
  +CONFIG_ARCH_BOARD_CUSTOM=y
  +CONFIG_ARCH_BOARD_CUSTOM_DIR="../out-of-tree/cxd32xx_boards/cb2sdb2"
  +CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
  +CONFIG_ARCH_BOARD_CUSTOM_NAME="cb2sdb2"
  +CONFIG_INIT_ENTRYPOINT="nsh_main"
  +CONFIG_UART0_SERIAL_CONSOLE=y
  +CONFIG_SYSTEM_NSH=y
  +CONFIG_RAM_START=0x00800000
  +CONFIG_RAM_SIZE=244736
  +CONFIG_ARCH_RAMVECTORS=y
  +CONFIG_RAW_BINARY=y
  +CONFIG_DEBUG_SYMBOLS=y
  +CONFIG_UART_PL011=y
  +CONFIG_UART0_PL011=y
  +CONFIG_UART0_BASE=0x42104000
  +CONFIG_UART0_IRQ=89
  +CONFIG_UART0_CLK_FREQ=32000000
  +CONFIG_UART_PL011_PLATFORMIF=y
  +CONFIG_BUILTIN=y
  +CONFIG_NSH_BUILTIN_APPS=y
  +CONFIG_SCHED_WAITPID=y
  +CONFIG_TESTING_OSTEST=y
  +CONFIG_FS_PROCFS=y
  +CONFIG_STACK_COLORATION=y
  +CONFIG_CXD32XX_APPS_HELLO=y
  +CONFIG_CXD32XX_APPS_TIME=y
  diff --git a/cxd32xx_boards/cb2sdb2/include/board.h b/cxd32xx_boards/cb2sdb2/include/board.h
  new file mode 100644
  index 0000000..eae687e
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/include/board.h
  @@ -0,0 +1,54 @@
  +/****************************************************************************
  + * cxd32xx_boards/cb2sdb2/include/board.h
  + *
  + * Licensed to the Apache Software Foundation (ASF) under one or more
  + * contributor license agreements.  See the NOTICE file distributed with
  + * this work for additional information regarding copyright ownership.  The
  + * ASF licenses this file to you under the Apache License, Version 2.0 (the
  + * "License"); you may not use this file except in compliance with the
  + * License.  You may obtain a copy of the License at
  + *
  + *   http://www.apache.org/licenses/LICENSE-2.0
  + *
  + * Unless required by applicable law or agreed to in writing, software
  + * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  + * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  + * License for the specific language governing permissions and limitations
  + * under the License.
  + *
  + ****************************************************************************/
  +
  +#ifndef __CXD32XX_BOARDS_CB2SDB2_INCLUDE_BOARD_H
  +#define __CXD32XX_BOARDS_CB2SDB2_INCLUDE_BOARD_H
  +
  +/****************************************************************************
  + * Included Files
  + ****************************************************************************/
  +
  +#include <nuttx/config.h>
  +#include <nuttx/irq.h>
  +#include <sys/boardctl.h>
  +#include <stdbool.h>
  +
  +/****************************************************************************
  + * Pre-processor Definitions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Public Types
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Public Function Prototypes
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Name: cxd32_boardinitialize
  + *
  + * Description:
  + *
  + ****************************************************************************/
  +
  +void cxd32_boardinitialize(void);
  +
  +#endif /* __CXD32XX_BOARDS_CB2SDB2_INCLUDE_BOARD_H */
  diff --git a/cxd32xx_boards/cb2sdb2/scripts/Make.defs b/cxd32xx_boards/cb2sdb2/scripts/Make.defs
  new file mode 100644
  index 0000000..4986d69
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/scripts/Make.defs
  @@ -0,0 +1,40 @@
  +############################################################################
  +# cxd32xx_boards/cb2sdb2/scripts/Make.defs
  +#
  +# Licensed to the Apache Software Foundation (ASF) under one or more
  +# contributor license agreements.  See the NOTICE file distributed with
  +# this work for additional information regarding copyright ownership.  The
  +# ASF licenses this file to you under the Apache License, Version 2.0 (the
  +# "License"); you may not use this file except in compliance with the
  +# License.  You may obtain a copy of the License at
  +#
  +#   http://www.apache.org/licenses/LICENSE-2.0
  +#
  +# Unless required by applicable law or agreed to in writing, software
  +# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  +# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  +# License for the specific language governing permissions and limitations
  +# under the License.
  +#
  +############################################################################
  +
  +include $(TOPDIR)/.config
  +include $(TOPDIR)/tools/Config.mk
  +include $(TOPDIR)/arch/arm/src/armv7-m/Toolchain.defs
  +
  +LDSCRIPT = cb2sdb2.ld
  +
  +ARCHSCRIPT += $(BOARD_DIR)$(DELIM)scripts$(DELIM)$(LDSCRIPT)
  +
  +ARCHPICFLAGS = -fpic -msingle-pic-base -mpic-register=r10
  +
  +CFLAGS := $(ARCHCFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRAFLAGS) -pipe
  +CPICFLAGS = $(ARCHPICFLAGS) $(CFLAGS)
  +CXXFLAGS := $(ARCHCXXFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(ARCHXXINCLUDES) $(ARCHDEFINES) $(EXTRAFLAGS) -pipe
  +CXXPICFLAGS = $(ARCHPICFLAGS) $(CXXFLAGS)
  +CPPFLAGS := $(ARCHINCLUDES) $(ARCHDEFINES) $(EXTRAFLAGS)
  +AFLAGS := $(CFLAGS) -D__ASSEMBLY__
  +
  +NXFLATLDFLAGS1 = -r -d -warn-common
  +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections
  +LDNXFLATFLAGS = -e main -s 2048
  diff --git a/cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld b/cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld
  new file mode 100644
  index 0000000..34ac8ac
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld
  @@ -0,0 +1,99 @@
  +/****************************************************************************
  + * cxd32xx_boards/cb2sdb2/scripts/cb2sdb2.ld
  + *
  + * Licensed to the Apache Software Foundation (ASF) under one or more
  + * contributor license agreements.  See the NOTICE file distributed with
  + * this work for additional information regarding copyright ownership.  The
  + * ASF licenses this file to you under the Apache License, Version 2.0 (the
  + * "License"); you may not use this file except in compliance with the
  + * License.  You may obtain a copy of the License at
  + *
  + *   http://www.apache.org/licenses/LICENSE-2.0
  + *
  + * Unless required by applicable law or agreed to in writing, software
  + * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  + * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  + * License for the specific language governing permissions and limitations
  + * under the License.
  + *
  + ****************************************************************************/
  +
  +INCLUDE ../../../../out-of-tree/cxd32xx_boards/cb2sdb2/scripts/memmap.ld
  +
  +OUTPUT_ARCH(arm)
  +ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
  +EXTERN(_vectors)       /* Force the vectors to be included in the output */
  +SECTIONS
  +{
  +    .text : {
  +        _stext = ABSOLUTE(.);
  +        *(.entry)
  +        *(.vectors)
  +        *(.text .text.*)
  +        *(.fixup)
  +        *(.gnu.warning)
  +        *(.rodata .rodata.*)
  +        *(.gnu.linkonce.t.*)
  +        *(.glue_7)
  +        *(.glue_7t)
  +        *(.got)
  +        *(.gcc_except_table)
  +        *(.gnu.linkonce.r.*)
  +        _etext = ABSOLUTE(.);
  +    } > flash_Cached
  +
  +    .init_section : {
  +        _sinit = ABSOLUTE(.);
  +        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
  +        KEEP(*(.init_array .ctors))
  +        _einit = ABSOLUTE(.);
  +    } > flash_Cached
  +
  +    .ARM.extab : {
  +        *(.ARM.extab*)
  +    } > flash_Cached
  +
  +    __exidx_start = ABSOLUTE(.);
  +    .ARM.exidx : {
  +        *(.ARM.exidx*)
  +    } > flash_Cached
  +    __exidx_end = ABSOLUTE(.);
  +
  +    _eronly = ABSOLUTE(.);
  +
  +    .data : {
  +        _sdata = ABSOLUTE(.);
  +        *(.data .data.*)
  +        *(.gnu.linkonce.d.*)
  +        CONSTRUCTORS
  +        . = ALIGN(4);
  +        _edata = ABSOLUTE(.);
  +    } > sram AT > flash_Cached
  +
  +    .ram_vectors : {
  +        *(.ram_vectors)
  +    } > sram
  +
  +    .bss : {            /* BSS              */
  +        _sbss = ABSOLUTE(.);
  +        *(.bss .bss.*)
  +        *(.gnu.linkonce.b.*)
  +        *(COMMON)
  +        . = ALIGN(4);
  +        _ebss = ABSOLUTE(.);
  +    } > sram
  +
  +    /* Stabs debugging sections.    */
  +    .stab 0 : { *(.stab) }
  +    .stabstr 0 : { *(.stabstr) }
  +    .stab.excl 0 : { *(.stab.excl) }
  +    .stab.exclstr 0 : { *(.stab.exclstr) }
  +    .stab.index 0 : { *(.stab.index) }
  +    .stab.indexstr 0 : { *(.stab.indexstr) }
  +    .comment 0 : { *(.comment) }
  +    .debug_abbrev 0 : { *(.debug_abbrev) }
  +    .debug_info 0 : { *(.debug_info) }
  +    .debug_line 0 : { *(.debug_line) }
  +    .debug_pubnames 0 : { *(.debug_pubnames) }
  +    .debug_aranges 0 : { *(.debug_aranges) }
  +}
  diff --git a/cxd32xx_boards/cb2sdb2/scripts/memmap.ld b/cxd32xx_boards/cb2sdb2/scripts/memmap.ld
  new file mode 100644
  index 0000000..848f336
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/scripts/memmap.ld
  @@ -0,0 +1,13 @@
  +/****************************************************************************
  + * Automatically generated file; DO NOT EDIT.
  + ****************************************************************************/
  +
  +MEMORY
  +{
  +  sram              (rwx) : ORIGIN = 0x00800000, LENGTH = 0x0003BC00
  +  sram_erw          (rw)  : ORIGIN = 0x0083BC00, LENGTH = 0x00001000
  +  sram_frw          (rw)  : ORIGIN = 0x0083CC00, LENGTH = 0x00002000
  +  sram_prt          (rw)  : ORIGIN = 0x0083EC00, LENGTH = 0x00001000
  +  sram_lib          (rwx) : ORIGIN = 0x0083FC00, LENGTH = 0x00000400
  +  flash_Cached      (rx)  : ORIGIN = 0x24010000, LENGTH = 0x0013F000
  +}
  diff --git a/cxd32xx_boards/cb2sdb2/src/Makefile b/cxd32xx_boards/cb2sdb2/src/Makefile
  new file mode 100644
  index 0000000..f8b1cd8
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/src/Makefile
  @@ -0,0 +1,25 @@
  +############################################################################
  +# cxd32xx_boards/cb2sdb2/src/Makefile
  +#
  +# Licensed to the Apache Software Foundation (ASF) under one or more
  +# contributor license agreements.  See the NOTICE file distributed with
  +# this work for additional information regarding copyright ownership.  The
  +# ASF licenses this file to you under the Apache License, Version 2.0 (the
  +# "License"); you may not use this file except in compliance with the
  +# License.  You may obtain a copy of the License at
  +#
  +#   http://www.apache.org/licenses/LICENSE-2.0
  +#
  +# Unless required by applicable law or agreed to in writing, software
  +# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  +# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  +# License for the specific language governing permissions and limitations
  +# under the License.
  +#
  +############################################################################
  +
  +include $(TOPDIR)/Make.defs
  +
  +CSRCS += cxd32_appinit.c cxd32_boot.c
  +
  +include $(TOPDIR)/boards/Board.mk
  diff --git a/cxd32xx_boards/cb2sdb2/src/cxd32_appinit.c b/cxd32xx_boards/cb2sdb2/src/cxd32_appinit.c
  new file mode 100644
  index 0000000..c1bef5f
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/src/cxd32_appinit.c
  @@ -0,0 +1,90 @@
  +/****************************************************************************
  + * cxd32xx_boards/cb2sdb2/src/cxd32_appinit.c
  + *
  + * Licensed to the Apache Software Foundation (ASF) under one or more
  + * contributor license agreements.  See the NOTICE file distributed with
  + * this work for additional information regarding copyright ownership.  The
  + * ASF licenses this file to you under the Apache License, Version 2.0 (the
  + * "License"); you may not use this file except in compliance with the
  + * License.  You may obtain a copy of the License at
  + *
  + *   http://www.apache.org/licenses/LICENSE-2.0
  + *
  + * Unless required by applicable law or agreed to in writing, software
  + * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  + * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  + * License for the specific language governing permissions and limitations
  + * under the License.
  + *
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Included Files
  + ****************************************************************************/
  +
  +#include <nuttx/config.h>
  +#include <nuttx/compiler.h>
  +
  +#include <sys/types.h>
  +#include <nuttx/board.h>
  +
  +/****************************************************************************
  + * Pre-processor Definitions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Public Functions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Name: board_app_initialize
  + *
  + * Description:
  + *   Perform application specific initialization.  This function is never
  + *   called directly from application code, but only indirectly via the
  + *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
  + *
  + * Input Parameters:
  + *   arg - The boardctl() argument is passed to the board_app_initialize()
  + *         implementation without modification.  The argument has no
  + *         meaning to NuttX; the meaning of the argument is a contract
  + *         between the board-specific initialization logic and the
  + *         matching application logic.  The value could be such things as a
  + *         mode enumeration value, a set of DIP switch switch settings, a
  + *         pointer to configuration data read from a file or serial FLASH,
  + *         or whatever you would like to do with it.  Every implementation
  + *         should accept zero/NULL as a default configuration.
  + *
  + * Returned Value:
  + *   Zero (OK) is returned on success; a negated errno value is returned on
  + *   any failure to indicate the nature of the failure.
  + *
  + ****************************************************************************/
  +
  +int board_app_initialize(uintptr_t arg)
  +{
  +  /* XXX : it will be implemented later */
  +
  +  return OK;
  +}
  +
  +/****************************************************************************
  + * Name: board_late_initialize
  + *
  + * Description:
  + *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
  + *   initialization call will be performed in the boot-up sequence to a
  + *   function called board_late_initialize().  board_late_initialize()
  + *   will be called immediately after up_initialize() is called and just
  + *   before the initial application is started.  This additional
  + *   initialization phase may be used, for example, to initialize board-
  + *   specific device drivers.
  + *
  + ****************************************************************************/
  +
  +#ifdef CONFIG_BOARD_LATE_INITIALIZE
  +void weak_function board_late_initialize(void)
  +{
  +  /* XXX : it will be implemented later */
  +}
  +#endif
  diff --git a/cxd32xx_boards/cb2sdb2/src/cxd32_boot.c b/cxd32xx_boards/cb2sdb2/src/cxd32_boot.c
  new file mode 100644
  index 0000000..edf4328
  --- /dev/null
  +++ b/cxd32xx_boards/cb2sdb2/src/cxd32_boot.c
  @@ -0,0 +1,56 @@
  +/****************************************************************************
  + * cxd32xx_boards/cb2sdb2/src/cxd32_boot.c
  + *
  + * Licensed to the Apache Software Foundation (ASF) under one or more
  + * contributor license agreements.  See the NOTICE file distributed with
  + * this work for additional information regarding copyright ownership.  The
  + * ASF licenses this file to you under the Apache License, Version 2.0 (the
  + * "License"); you may not use this file except in compliance with the
  + * License.  You may obtain a copy of the License at
  + *
  + *   http://www.apache.org/licenses/LICENSE-2.0
  + *
  + * Unless required by applicable law or agreed to in writing, software
  + * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  + * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
  + * License for the specific language governing permissions and limitations
  + * under the License.
  + *
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Included Files
  + ****************************************************************************/
  +
  +#include <nuttx/config.h>
  +
  +#include <debug.h>
  +
  +#include <nuttx/board.h>
  +#include <arch/board/board.h>
  +
  +#include "arm_internal.h"
  +
  +/****************************************************************************
  + * Pre-processor Definitions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Private Functions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Public Functions
  + ****************************************************************************/
  +
  +/****************************************************************************
  + * Name: cxd32_boardinitialize
  + *
  + * Description:
  + *
  + ****************************************************************************/
  +
  +void cxd32_boardinitialize(void)
  +{
  +  /* XXX : it will be implemented later */
  +}

