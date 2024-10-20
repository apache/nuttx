===============================
Porting Drivers to the STM32 F7
===============================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Porting+Drivers+to+the+STM32+F7

Problem Statement
=================

I recently completed a port to the STMicro STM32F746G Discovery board. 
That MCU is clearly a derivative of the STM32 F3/F4 and many peripherals 
are, in fact, essentially identical to the STM32F429. The biggest 
difference is that the STM32F746 sports a Cortex-M7 which includes 
several improvements over the Cortex-M4 and including, most relevant 
to this discussion, a fully integrated data cache (`D-Cache`).

Because of this one difference, I chose to provide the STM32 F7 code its 
own directories separate from the STM32 F1, F2, F3, and F4.

Porting Simple Drivers
======================

Some of the STM32 F4 drivers can be used with the STM32 F7 can be ported 
very simply; many ports would just be a matter of copying files and some 
search-and-replacement. Like:

* Compare the two register definitions files; make sure that the STM32 
  F4 peripheral is identical (or nearly identical) to the F7 peripheral. 
  If so then,
* Copy the register definition file from the ``stm32/hardware`` to the 
  ``stm32f7/hardware`` directory, making name changes as appropriate and 
  updating any minor register differences.
* Copy the corresponding C file (and possibly a ``.h`` file) from the 
  ``stm32/`` directory to the ``stm32f7/`` directory, again making any naming 
  changes and modifications for any register differences.
* Update the ``Make.defs`` file to include the new C file in the build.

Porting Complex Drivers
=======================

The Cortex-M7 D-Cache, however, does raise issues with the compatibility 
of most complex STM32 F4 and F7 drivers. Even though the peripheral 
registers may be essentially the same between the STM32F429 and the 
the STM32F746, many drivers for the STM32F429 will not be directly 
compatible with the STM32F746, particularly drivers that use DMA. 
And that includes most complex STM32 drivers!

Cache Coherency
===============

With DMA, physical RAM memory contents is accessed directly by peripheral 
hardware without intervention from the CPU. The CPU itself deals only the 
indirectly with RAM through the D-Cache: When you read data from RAM, it 
is first loaded in the D-Cache then accessed by the CPU. If the RAM 
contents is already in the D-Cache, then physical RAM is not accessed 
at all! Similarly, when you write data into RAM (with write buffering 
enabled), it may actually not be written to physical RAM but may just 
remain in the D-Cache in a `dirty` cache line until that cache line is 
flushed to memory. Thus, there may be inconsistencies in the contents 
of the D-Cache and in the contents of contents of physical RAM due 
related to DMA. Such issues are referred to as `Cache Coherency` problems.

DMA
===

DMA Read Accesses
-----------------

A DMA read access occurs when we program DMA hardware to read data 
from a peripheral and store that data into RAM. This happens, for 
example, when we read a packet from the network, when we read a 
serial byte of data from a UART, when we read a block from an 
MMC/SD card, and so on.

In this case, the DMA hardware will change the contents of physical 
RAM without knowledge of the CPU. So if that same memory that was 
modified by the DMA read operation is also in the D-Cache, then 
the contents of the D-Cache will no longer be valid; it will no 
longer match the physical contents of the memory. In order to fix 
this, the Cortex-M7 supports a special `cache operation` that can be 
used to `invalidate` the D-Cache contents associate with the read DMA 
buffer address range. Invalidation simply means discarding the 
currently cached D-Cache lines so that they will be refetched 
from physical RAM. **Rule 1a**: Always invalidate RX DMA buffers 
sometime before or after starting the read DMA but certainly `before` 
accessing the read buffer data. **Rule 1b**: Never read from the read 
DMA buffer before the read DMA buffer completes, or otherwise you 
will re-cache the DMA buffer content.

`What if the D-Cache line is also dirty? What if we have writes to 
the DMA buffer that were never flushed to physical RAM?` Those writes 
will then never make it to physical memory if the D-Cache is 
invalidated. **Rule 2**: Never write to read DMA buffer memory! 
**Rule 3**: Make sure that all DMA read buffers are aligned to the 
D-Cache line size so that there are no spill-over cache effects 
at the boarders of the invalidated cache line.

DMA Write Accesses
------------------

A DMA write access occurs when we program DMA hardware to write data from 
RAM into a peripheral. This happen for example, when we send a packet on 
a network or when we write a block of data to an MMC/SD card. In this, 
the hardware expects the correct data to be in physical RAM when write 
DMA is performed. If not then, the wrong data will be sent.

We assure that we do not have pending writes in a `dirty` cache line by 
`cleaning` (or `flushing`) the `dirty` cache lines; i.e., for forcing any 
pending writes in the D-Cache lines to be written to physical RAM. 
**Rule 4**: Always `clean` (or `flush`) the D-Cache to force all data to 
be written from the D-Cache into physical RAM.

`What if you had two adjacent DMA buffers side-by-side? Couldn't the 
cleaning of the write buffer force writing into the adjacent read 
buffer?`` Yes! **Rule 5**: Make sure that all DMA write buffers are 
aligned to the D-Cache line size so that there are no spill-over 
cache effects at the borders of the cleaned cache line.

Write-back vs. Write-through D-Cache
------------------------------------

The Cortex-M7 supports both `write-back` and `write-through` data cache 
configurations. The write-back D-Cache works just as described above: 
`dirty` cache lines are not written to physical memory until the cache 
line is flushed. But write-through D-Cache works just as without the 
D-Cache. Writes always go directly to physical RAM.

`If I am using a write-through D-Cache, can't I just forget about 
cleaning the D-Cache?` No, because you don't know how a user is going 
to configuration the D-Cache. **Rule 6**: Always assume that `write-back` 
caching is being performed; otherwise, your driver will not be portable.

You may notice in ``/arch/arm/src/armv7-m/cache.h``:

.. code-block:: c

    #if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
    void arch_clean_dcache(uintptr_t start, uintptr_t end);
    #else
    #  define arch_clean_dcache(s,e)
    #endif

NOTE: I have experienced other cases (on the SAMV7) where write buffering 
`must` be disabled: In one case, a certain peripheral used 16-byte DMA 
descriptors in an array. Clearly it is impossible to manage the 
caching of the 16-byte DMA descriptors with a 32-byte cache line in 
this case: I think that the only option is to disabled the write buffer.

And what if the driver receives arbitrarily aligned buffers from the 
application? Then what? Should write buffering be disabled in that 
case too? And what is the performance cost for disabling the write 
buffer?


DMA Module
----------

Some STM32 F7 peripherals have built in DMA. The STM32 F7 Ethernet 
driver discussed below is a good example of such a peripheral with 
built in DMA capability. Most STM32 F7 peripherals, however, have 
no built-in DMA capability and, instead, must use a common STM32 
F7 DMA module to perform DMA data transfers. The interfaces to that 
common DMA module are described in ``arch/arm/src/stm32f7/stm32_dma.h``.

The DMA modules `does not do any cache operations`. Rather, the client 
of the DMA module must perform the cache operations. Here are the 
basic rules:

* TX DMA Transfers. Before calling ``stm32_dmastart()`` to start an TX 
  transfer, the DMA client must clean the DMA buffer so that the 
  content to be DMA'ed is present in physical memory.
* RX DMA transfers. At the completion of all DMAs, the DMA client 
  will receive a callback providing the final status of the DMA 
  transfer. For the case of RX DMA completion callbacks, logic in 
  the callback handler should invalidate the RX buffer before any 
  attempt is made to access new RX buffer content.

Converting an STM32F429 Driver for the STM32F746
================================================

Since the STM32 F7 is so similar to the STM32 F4, we have a wealth 
of working drivers to port from. Only a little effort is required. 
Below is a summary of the kinds of things that you would have to do 
to convert an STM32F429 driver to the STM32F746.

An Example
----------

There is a good example in the STM32 Ethernet driver. The STM32 F7 
Ethernet driver (``arch/arm/src/stm32f7/stm32_ethernet.c``) derives 
directly from the STM32 F4 Ethernet driver 
(``arch/arm/src/stm32/stm32_eth.c``). These two Ethernet MAC peripherals 
are nearly identical. Only changes that are a direct consequence of the 
STM32 F7 D-Cache were required to make the driver work on the STM32 F7. 
Those changes are summarized below.

Reorganize DMA Data Structure
-----------------------------

The STM32 Ethernet driver has four different kinds DMA buffers:

* RX DMA descriptor,
* TX DMA descriptors,
* RX packet buffers, and
* TX packet buffers,

In the STM32F429 driver, these are simply implemented as part of the 
driver data structure:

.. code-block:: c

    struct stm32_ethmac_s
    {
        ...
        /* Descriptor allocations */
        
        struct eth_rxdesc_s rxtable[CONFIG_STM32_ETH_NRXDESC];
        struct eth_txdesc_s txtable[CONFIG_STM32_ETH_NTXDESC];
        
        /* Buffer allocations */
        
        uint8_t rxbuffer[CONFIG_STM32_ETH_NRXDESC*CONFIG_STM32_ETH_BUFSIZE];
        uint8_t alloc[STM32_ETH_NFREEBUFFERS*CONFIG_STM32_ETH_BUFSIZE];
    };

There are potentially three problems with this: (1) We don't know what 
kind of memory the data structure will be defined in. What if it is 
DTCM memory? Then the DMAs will fail. (2) We don't know the alignment 
of the DMA buffers. They must be aligned on D-Cache line boundaries. 
(3a) The size of RX or TX descriptor is either 16- or 32-bytes. In 
order to individually clean or invalidate the cache line, they must 
be sized in multiples of the cache line size and (3b) the same applies 
to the DMA buffers.

To fix this, several things were done:

* The buffer allocations were moved from the device structure into 
  separate declarations that can have attributes.
* One attribute that could be added would be a section name to assure 
  that the structures are linked into DMA-able memory (via definitions 
  in the linker script).
* Another attribute is that we can force the alignment of the structure 
  to the D-Cache line size.

The following definitions were added to support aligning the sizes of 
the buffers to the Cortex-M7 D-Cache line size:

.. code-block:: c

    /* Buffers use fro DMA access must begin on an address aligned with the
   * D-Cache line and must be an even multiple of the D-Cache line size.
   * These size/alignment requirements are necessary so that D-Cache flush
   * and invalidate operations will not have any additional effects.
   *
   * The TX and RX descriptors are normally 16 bytes in size but could be
   * 32 bytes in size if the enhanced descriptor format is used (it is not).
   */
    
    #define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
    #define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
    #define DMA_ALIGN_DOWN(n)  ((n) & ~DMA_BUFFER_MASK)
    
    #ifndef CONFIG_STM32F7_ETH_ENHANCEDDESC
    #  define RXDESC_SIZE       16
    #  define TXDESC_SIZE       16
    #else
    #  define RXDESC_SIZE       32
    #  define TXDESC_SIZE       32
    #endif
    
    #define RXDESC_PADSIZE      DMA_ALIGN_UP(RXDESC_SIZE)
    #define TXDESC_PADSIZE      DMA_ALIGN_UP(TXDESC_SIZE)
    #define ALIGNED_BUFSIZE     DMA_ALIGN_UP(ETH_BUFSIZE)
    
    #define RXTABLE_SIZE        (STM32F7_NETHERNET * CONFIG_STM32F7_ETH_NRXDESC)
    #define TXTABLE_SIZE        (STM32F7_NETHERNET * CONFIG_STM32F7_ETH_NTXDESC)
    
    #define RXBUFFER_SIZE       (CONFIG_STM32F7_ETH_NRXDESC * ALIGNED_BUFSIZE)
    #define RXBUFFER_ALLOC      (STM32F7_NETHERNET * RXBUFFER_SIZE)
    
    #define TXBUFFER_SIZE       (STM32_ETH_NFREEBUFFERS * ALIGNED_BUFSIZE)
    #define TXBUFFER_ALLOC      (STM32F7_NETHERNET * TXBUFFER_SIZE)

The RX and TX descriptor types are replace with a union type 
that assures that the allocations will be aligned in size:

.. code-block:: c

    /* This union type forces the allocated size of RX descriptors to be the
    * padded to a exact multiple of the Cortex-M7 D-Cache line size.
    */
     
    union stm32_txdesc_u
    {
      uint8_t             pad[TXDESC_PADSIZE];
      struct eth_txdesc_s txdesc;
    };
     
    union stm32_rxdesc_u
    {
      uint8_t             pad[RXDESC_PADSIZE];
      struct eth_rxdesc_s rxdesc;
    };

Then, finally, the new buffers are defined by the following globals:

.. code-block:: c

    /* DMA buffers.  DMA buffers must:
    *
    * 1. Be a multiple of the D-Cache line size.  This requirement is assured
    *    by the definition of RXDMA buffer size above.
    * 2. Be aligned a D-Cache line boundaries, and
    * 3. Be positioned in DMA-able memory (*NOT* DTCM memory).  This must
    *    be managed by logic in the linker script file.
    *
    * These DMA buffers are defined sequentially here to best assure optimal
    * packing of the buffers.
    */
    
    /* Descriptor allocations */
    
    static union stm32_rxdesc_u g_rxtable[RXTABLE_SIZE]
    __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
    static union stm32_txdesc_u g_txtable[TXTABLE_SIZE]
    __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
    
    /* Buffer allocations */
    
    static uint8_t g_rxbuffer[RXBUFFER_ALLOC]
    __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
    static uint8_t g_txbuffer[TXBUFFER_ALLOC]
    __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));

This does, of course, force additional changes to the functions 
that initialize the buffer chains, but I will leave that to the 
interested reader to discover.

Add Cache Operations
--------------------

The Cortex-M7 cache operations are available the following file is included:


.. code-block:: c

    #include "cache.h"

Here is an example where the RX descriptors are invalidated:

.. code-block:: c

    static int stm32_recvframe(struct stm32_ethmac_s *priv)
    {
    ...
    /* Scan descriptors owned by the CPU.  */
    
    rxdesc = priv->rxhead;
    
    /* Forces the first RX descriptor to be re-read from physical memory */
    
    arch_invalidate_dcache((uintptr_t)rxdesc,
                            (uintptr_t)rxdesc + sizeof(struct eth_rxdesc_s));
    
    for (i = 0;
        (rxdesc->rdes0 & ETH_RDES0_OWN) == 0 &&
            i < CONFIG_STM32F7_ETH_NRXDESC &&
            priv->inflight < CONFIG_STM32F7_ETH_NTXDESC;
        i++)
        {
        ...
        /* Try the next descriptor */
    
        rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    
        /* Force the next RX descriptor to be re-read from physical memory */
    
        arch_invalidate_dcache((uintptr_t)rxdesc,
                                (uintptr_t)rxdesc + sizeof(struct eth_rxdesc_s));
        }
    ...
    }

Here is an example where a TX descriptor is cleaned:

.. code-block:: c

    static int stm32_transmit(struct stm32_ethmac_s *priv)
    {
    ...
            /* Give the descriptor to DMA */
    
            txdesc->tdes0 |= ETH_TDES0_OWN;
    
            /* Flush the contents of the modified TX descriptor into physical
            * memory.
            */
    
            arch_clean_dcache((uintptr_t)txdesc,
                                (uintptr_t)txdesc + sizeof(struct eth_txdesc_s));
    ...
    }

Here is where the read buffer is invalidated just after 
completed a read DMA:

.. code-block:: c

    static int stm32_recvframe(struct stm32_ethmac_s *priv)
    {
    ...
        /* Force the completed RX DMA buffer to be re-read from
        * physical memory.
        */
    
        arch_invalidate_dcache((uintptr_t)dev->d_buf,
                            (uintptr_t)dev->d_buf + dev->d_len);
    
        nllvdbg("rxhead: %p d_buf: %p d_len: %d\n",
                priv->rxhead, dev->d_buf, dev->d_len);
    
        /* Return success*/
    
        return OK;
    ...
    }

Here is where the write buffer in clean prior to starting a write DMA:

.. code-block:: c

    static int stm32_transmit(struct stm32_ethmac_s *priv)
    {
    ...
    /* Flush the contents of the TX buffer into physical memory */
    
    arch_clean_dcache((uintptr_t)priv->dev.d_buf,
                        (uintptr_t)priv->dev.d_buf + priv->dev.d_len);
    ...
    }