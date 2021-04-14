mm/README.txt
=============

This directory contains the NuttX memory management logic.  This include:

1) Standard Memory Management Functions:

   Standard Functions:

     The standard memory management functions as prototyped in stdlib.h as
     specified in the  Base definitions volume of IEEE Std 1003.1-2001.  This
     include the files:

     o Standard Interfaces: mm_malloc.c, mm_calloc.c, mm_realloc.c,
       mm_memalign.c, mm_free.c
     o Less-Standard Interfaces: mm_zalloc.c, mm_mallinfo.c
     o Internal Implementation: mm_initialize.c mm_sem.c  mm_addfreechunk.c
       mm_size2ndx.c mm_shrinkchunk.c
     o Build and Configuration files: Kconfig, Makefile

   Memory Models:

     o Small Memory Model.  If the MCU supports only 16-bit data addressing
       then the small memory model is automatically used.  The maximum size
       of the heap is then 64K.  The small memory model can also be forced
       MCUs with wider addressing by defining CONFIG_SMALL_MEMORY in the
       NuttX configuration file.
     o Large Memory Model.  Otherwise, the allocator uses a model that
       supports a heap of up to 4G.

     This implementation uses a variable length allocator with the following
     properties:

     o Overhead:  Either 8- or 4-bytes per allocation for large and small
       models, respectively.
     o Alignment:  All allocations are aligned to 8- or 4-bytes for large
       and small models, respectively.

   Multiple Heaps:

     This allocator can be used to manage multiple heaps (albeit with some
     non-standard interfaces).  A heap is represented by struct mm_heap_s
     as defined in the file include/nuttx/mm/mm.h.  To create another heap
     instance, you would allocate a heap structure, most likely statically
     in memory:

       include <nuttx/mm/mm.h>
       static struct mm_heap_s *g_myheap;

     Then initialize the heap using:

       g_myheap = mm_initialize(myheap_start, myheap_size);

     Where mm_initialize() and all related interfaces are prototyped in the
     header file include/nuttx/mm/mm.h.

     After the new heap instance has been initialized, it can then be used
     with these almost familiar interfaces: mm_malloc(), mm_realloc(), mm_free(),
     etc.  These are 'almost familiar' because they are analogous of the
     standard malloc(), realloc(), free(), etc. except that they expect a
     reference to the initialized heap structure as the first parameter.

     In fact, the standard malloc(), realloc(), free() use this same mechanism,
     but with a global heap structure called g_mmheap.

   User/Kernel Heaps

     This multiple heap capability is exploited in some of the more complex NuttX
     build configurations to provide separate kernel-mode and user-mode heaps.

   Sub-Directories:

     mm/mm_heap  - Holds the common base logic for all heap allocators
     mm/umm_heap - Holds the user-mode memory allocation interfaces
     mm/kmm_heap - Holds the kernel-mode memory allocation interfaces

   Debugging:

    Please follow these steps to hook all memory related routines:

    1.Add a new header file(e.g. xxx_malloc.h):

      ...
      #include <malloc.h>
      #include <stdlib.h>
      #include <string.h>
      #include <strings.h>

      #ifndef __ASSEMBLY__
      FAR void *xxx_malloc(FAR const char *file, int line, size_t size);
      void xxx_free(FAR const char *file, int line, FAR const void *ptr);
      FAR void *xxx_memcpy(FAR const char *file, int line,
                           FAR void *dst, FAR const void *src, size_t len);
      ...
      #define malloc(s) xxx_malloc(__FILE__, __LINE__, s)
      #define free(p) xxx_free(__FILE__, __LINE__, p)
      #define memcpy(d, s, l) xxx_memcpy(__FILE__, __LINE__, d, s, l)
      ...
      #endif
      ...

    2.Implement xxx_malloc, xxx_free, xxx_memcpy... in source code, you can:
      a.Modify some arguments(e.g. extend the allocation size for redzone)
      d.Check the critical arguments(e.g. pointer and length) in the range 
      b.Forward to the original implementation(call malloc/free/memcpy)
      c.Attach the context info(e.g. file and line) before return

    3.Enable the hook by either:
      a.Include xxx_malloc.h in your source code to hook one file
      b.Add -include xxx_malloc.h to CFLAGS to hook all source code

2) Granule Allocator.

     A non-standard granule allocator is also available in this directory  The
     granule allocator allocates memory in units of a fixed sized block ("granule").
     Allocations may be aligned to a user-provided address boundary.

     The granule allocator interfaces are defined in nuttx/include/nuttx/mm/gran.h.
     The granule allocator consists of these files in this directory:

       mm_gran.h, mm_granalloc.c, mm_grancritical.c, mm_granfree.c
       mm_graninit.c

     The granule allocator is not used anywhere within the base NuttX code
     as of this writing.  The intent of the granule allocator is to provide
     a tool to support platform-specific management of aligned DMA memory.

     NOTE: Because each granule may be aligned and each allocation is in
     units of the granule size, selection of the granule size is important:
     Larger granules will give better performance and less overhead but more
     losses of memory due to quantization waste.  Additional memory waste
     can occur from alignment;  Of course, heap alignment should no be
     used unless (a) you are using the granule allocator to manage DMA memory
     and (b) your hardware has specific memory alignment requirements.

     The current implementation also restricts the maximum allocation size
     to 32 granules.  That restriction could be eliminated with some
     additional coding effort, but currently requires larger granule
     sizes for larger allocations.

   General Usage Example.

     This is an example using the GCC section attribute to position a DMA
     heap in memory (logic in the linker script would assign the section
     .dmaheap to the DMA memory.

        FAR uint32_t g_dmaheap[DMAHEAP_SIZE] __attribute__((section(.dmaheap)));

     The heap is created by calling gran_initialize.  Here the granule size
     is set to 64 bytes and the alignment to 16 bytes:

       GRAN_HANDLE handle = gran_initialize(g_dmaheap, DMAHEAP_SIZE, 6, 4);

     Then the GRAN_HANDLE can be used to allocate memory:

       FAR uint8_t *dma_memory = (FAR uint8_t *)gran_alloc(handle, 47);

     The actual memory allocates will be 64 byte (wasting 17 bytes) and
     will be aligned at least to (1 << log2align).

   Sub-Directories:

     mm/mm_gran - Holds the granule allocation logic

3) Page Allocator

   The page allocator is an application of the granule allocator.  It is a
   special purpose memory allocator intended to allocate physical memory
   pages for use with systems that have a memory management unit (MMU).

   Sub-Directories:

     mm/mm_gran - The page allocator cohabits the same directory as the
       granule allocator.

4) Shared Memory Management

   When NuttX is build in kernel mode with a separate, privileged, kernel-
   mode address space and multiple, unprivileged, user-mode address spaces,
   then shared memory regions must also be managed.  Shared memory regions
   are user-accessible memory regions that can be attached into the user
   process address space for sharing between user process.

   Sub-Directories:

     mm/shm - The shared memory logic

   The shared memory management logic has its own README file that can be
   found at nuttx/mm/shm/README.txt.

5) I/O Buffers

   The iob subdirectory contains a simple allocator of I/O buffers.  These
   I/O buffers, IOBs, are used extensively for networking but are generally
   available for usage by drivers.  The I/O buffers have these properties:

   1. Uses a pool of a fixed number of fixed fixed size buffers.
   2. Free buffers are retained in free list:  When a buffer is allocated
      it is removed from the free list; when a buffer is freed it is
      returned to the free list.
   3. The calling application will wait if there are not free buffers.
