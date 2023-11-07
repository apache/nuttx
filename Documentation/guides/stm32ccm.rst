===================
STM32 CCM Allocator
===================

CCM Memory
==========

The STM32 F2, F3, and F4 families have a special block of SRAM available called
CCM (Core Coupled Memory). This memory has the drawback that it cannot be used
for STM32 DMA operations.

By default, the CCM memory is lumped in with the rest of memory when the NuttX
heaps are created. But this can be a problem because it will be a toss of the
coin if non-DMA-able CCM memory or other DMA-able memory gets returned when
``malloc()`` is called. That usually does not matter but it certainly does make
a difference if you are allocating memory that will be used for DMA! In that
case, getting CCM memory for your DMA buffer will cause a failure.

CONFIG_STM32_CCMEXCLUDE
=======================

There is a configuration option called ``CONFIG_STM32_CCMEXCLUDE`` that can be
used to exclude CCM memory from the heap. That solves the problem of getting
CCM memory when you want to allocate a DMA buffer. But then what do you do
with the CCM memory? Do you let it go unused?

CCM Allocator
=============

In order to make use of the CCM memory, a CCM memory allocator is available.
This memory allocator is automatically enabled when the following options are set:

* ``CONFIG_STM32_CCMEXCLUDE`` CCM memory is excluded from the normal heap, and
* ``CONFIG_MM_MULTIHEAP`` Support for multiple heaps is enabled.

Under those conditions, the CCM memory allocator is enabled and the allocator
interfaces prototyped in the ``arch/arm/src/stm32/stm32_ccm.h`` are available.

NOTE: These interfaces are, technically, not prototyped since they are really
provided via C pre-processor macros.

NOTE: In order to use the CCM memory allocator functions, you must first call
``ccm_initialize()`` somwhere in your early boot-up logic.

With these interfaces you have a (nearly) standard way to manage memory from a
heap that consists of the the CCM SRAM. And, since the CCM memory is no longer
a part of the normal heap, all allocated I/O buffers will be DMA-able (unless you
have included other non-DMA-able memory regions in the stack).

CCM Stacks
==========

One particular problem that has been reported by Petteri Aimonen requires some
additional work-arounds. The STM32 SPI driver supports DMA and with SPI it is
sometimes necessary to do some very small transfers for which there is no real
gain from using DMA. In this case, Petteri has devised a clever way to both 1) make
use of the CMM memory and 2) to force fallback to non-DMA transfers for these small
stack transfers.

Here is what Petteri has done:

#. First, he has modified ``arch/arm/src/common/up_createstack.c`` and
   ``up_releasestack.c`` so that stacks are allocated from CCM memory. That
   allocation is something like the following:

   .. code-block:: C

      void *result = ccm_zalloc(size);
      if (!result)
        {
         /* Fall back to main heap */
          result = zalloc(size);
        }

   With the matching:

   .. code-block:: C

      if (((uint32_t)p & 0xF0000000) == 0x10000000)
        {
          ccm_free(p);
        }
      else
        {
          free(p);
        }

#. Then Petteri added special DMA support enabled with ``CONFIG_STM32_DMACAPABLE``.
   That option enables an option in all of the DMA logic called:

   .. code-block:: C

      bool stm32_dmacapable(uint32_t maddr);

   That will return true if it is possible to do DMA from the address and false
   if not.

#. Finally, Petteri added logic to the STM32 SPI driver that use ``stm32_dmacapable()``:
   If the address is not DMA capable, then the SPI driver will fall back to
   non-DMA operation.

   With Petteri's changes all of the large I/O buffers will be allocated from
   DMA-able memory. All stacks will be allocated from non-DMA-able CCM memory
   (provided that there is space). Small SPI DMA buffers on the non-DMA-able stack
   will be detected by ``stm32_dmacapable()`` and in that case, the STM32 SPI driver
   will fall back and use non-DMA-transfers.

   From all reports this works quite well.
