=========================
Run time stack statistics
=========================

Introduce
=========
When debugging code, it is often necessary to focus on how to track
the maximum stack usage of the calling functions in order to optimize
the code structure and reduce stack usage. This article will introduce
a method based on the running state to track the deepest call stack of all tasks.

Configuration
=============
   .. code-block:: c

      CONFIG_SCHED_STACKRECORD=32
      CONFIG_ARCH_INSTRUMENT_ALL=y

```CONFIG_SCHED_STACKRECORD``` is used to record the maximum stack usage of all tasks
```CONFIG_ARCH_INSTRUMENT_ALL``` is used to instrument all code.

Please note that CONFIG_ARCH_INSTRUMENT_ALL is not necessary.
This configuration option will instrument all code,
but if you only want to instrument specific functions,
you can add '-finstrument-functions' to the corresponding makefile.

Example
=======
1. ```./tools/configure.sh esp32c3-devkit:stack```
2. ```make -j20```
3. flash the image to your board
   .. code-block :: bash

      nsh> cat /proc/1/stack
      StackAlloc: 0x3fc8b5b0
      StackBase:  0x3fc8b5e0
      StackSize:  2000
      MaxStackUsed:1344
      Backtrace         Size
      0x42009198          32
      0x42009200          48
      0x420081a0         128
      0x42008d18          64
      0x4201da60          80
      0x420199e0          80
      0x42018c6c          48
      0x420194f4          48
      0x42017d30          32
      0x4201634c          32
      0x420163ac          48
      0x42016408          32
      0x420132c0          48
      0x42010598          32
      0x4200fd98          48
      0x4200f5dc          80
      0x4200f8e0         160

Implementation details
======================
The specific principle is based on the instrumentation of gcc.
In the TCB (Thread Control Block) of the corresponding task,
the maximum value of the stack pointer (sp) is recorded at the entry of each function.
If it is the maximum value, the backtrace is recorded.

Notice
======
Be cautious when using the CONFIG_ARCH_INSTRUMENT_ALL option:
1. It will instrument every function, which may have a risk of recursion.
2. It will also instrument entry functions, such as _start(). At this point,
the bss segment and data segment have not been initialized,
which may cause errors. The current implementation uses a magic number to avoid this,
but it performs poorly during hot start. The solution is to mark
the entry function with the noinstrument_function flag to prevent instrumentation.
