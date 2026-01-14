=====================
Smaller Vector Tables
=====================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Smaller+Vector+Tables 


One of the largest OS data structures is the vector table, 
``g_irqvector[]``. This is the table that holds the vector 
information when ``irq_attach()`` is called and used to 
dispatch interrupts by ``irq_dispatch()``. Recent changes 
have made that table even larger, for 32-bit arm the 
size of that table is given by:

.. code-block:: c

    nbytes = number_of_interrupts * (2 * sizeof(void *))

We will focus on the STM32 for this discussion to keep 
things simple. However, this discussion applies to all 
architectures.

The number of (physical) interrupt vectors supported by 
the MCU hardwared given by the definition ``NR_IRQ`` which 
is provided in a header file in ``arch/arm/include/stm32``. 
This is, by default, the value of ``number_of_interrupts`` 
in the above equation.

For a 32-bit ARM like the STM32 with, say, 100 interrupt 
vectors, this size would be 800 bytes of memory. That is 
not a lot for high-end MCUs with a lot of RAM memory, 
but could be a show stopper for MCUs with minimal RAM.

Two approaches for reducing the size of the vector tables 
are described below. Both depend on the fact that not all 
interrupts are used on a given MCU. Most of the time, 
the majority of entries in ``g_irqvector[]`` are zero because 
only a small number of interrupts are actually attached 
and enabled by the application. If you know that certain 
IRQ numbers are not going to be used, then it is possible 
to filter those out and reduce the size to the number of 
supported interrupts.

For example, if the actual number of interrupts used were 
20, the the above requirement would go from 800 bytes to 
160 bytes.

Software IRQ Remapping
======================

`[On March 3, 2017, support for this "Software IRQ Remapping" 
as included in the NuttX repository.]`

One of the simplest way of reducing the size of 
``g_irqvector[]`` would be to remap the large set of physical 
interrupt vectors into a much small set of interrupts that 
are actually used. For the sake of discussion, let's 
imagine two new configuration settings:

* ``CONFIG_ARCH_MINIMAL_VECTORTABLE``: Enables IRQ mapping
* ``CONFIG_ARCH_NUSER_INTERRUPTS``: The number of IRQs after mapping.

Then it could allocate the interrupt vector table to be 
size ``CONFIG_IRQ_NMAPPED_IRQ`` instead of the much bigger 
``NR_IRQS``:

.. code-block:: c 

    #ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
    struct irq_info_s g_irqvector[CONFIG_ARCH_NUSER_INTERRUPTS];
    #else
    struct irq_info_s g_irqvector[NR_IRQS];
    #endif

The ``g_irqvector[]`` table is accessed in only three places:

``irq_attach()``
----------------

``irq_attach()`` receives the physical vector number along 
with the information needed later to dispatch interrupts:

.. code-block:: c

    int irq_attach(int irq, xcpt_t isr, FAR void *arg);

Logic in ``irq_attach()`` would map the incoming physical 
vector number to a table index like:

.. code-block:: c 

    #ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
    int ndx = g_irqmap[irq];
    #else
    int ndx = irq;
    #endif

where ``up_mapirq[]`` is an array indexed by the physical 
interrupt vector number and contains the new, mapped 
interrupt vector table index. This array must be 
provided by platform-specific code.

``irq_attach()`` would this use this index to set the ``g_irqvector[]``.

.. code-block:: c 

    g_irqvector[ndx].handler = isr;
    g_irqvector[ndx].arg     = arg;

``irq_dispatch()``
------------------

``irq_dispatch()`` is called by MCU logic when an interrupt is received:

.. code-block:: c 

    void irq_dispatch(int irq, FAR void *context);

Where, again irq is the physical interrupt vector number.

``irq_dispatch()`` would do essentially the same thing as 
``irq_attach()``. First it would map the irq number to 
a table index:

.. code-block:: c 

    #ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
    int ndx = g_irqmap[irq];
    #else
    int ndx = irq;
    #endif

Then dispatch the interrupt handling to the attached 
interrupt handler. NOTE that the physical vector 
number is passed to the handler so it is completely 
unaware of the underlying `shell` game:

.. code-block:: c 

    vector = g_irqvector[ndx].handler;
    arg    = g_irqvector[ndx].arg;
    
    vector(irq, context, arg);

``irq_initialize()``
--------------------

``irq_initialize()``: simply set the ``g_irqvector[]`` table 
a known state on power-up. It would only have to distinguish 
the difference in sizes.

.. code-block:: c 

    #ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
    #  define TAB_SIZE CONFIG_ARCH_NUSER_INTERRUPTS
    #else
    #  define TAB_SIZE NR_IRQS
    #endif
    
    for (i = 0; i < TAB_SIZE; i++)

``g_mapirq[]``
--------------

An implementation of ``up_mapirq()`` might be something like:

.. code-block:: c 

    #include <nuttx/irq.h>

    const irq_mapped_t g_irqmap[NR_IRQS] =
    {
    ... IRQ to index mapping values ...
    };

``g_irqmap[]`` is a array of mapped irq table indices. It 
contains the mapped index value and is itself indexed 
by the physical interrupt vector number. It provides 
an ``irq_mapped_t`` value in the range of 0 to 
``CONFIG_ARCH_NUSER_INTERRUPTS`` that is the new, mapped 
index into the vector table. Unsupported IRQs would 
simply map to an out of range value like ``IRQMAPPED_MAX``. 
So, for example, if ``g_irqmap[37] == 24``, then the hardware 
interrupt vector 37 will be mapped to the interrupt vector 
table at index 24. if ``g_irqmap[42] == IRQMAPPED_MAX``, then 
hardware interrupt vector 42 is not used and if it occurs 
will result in an unexpected interrupt crash.

Hardware Vector Remapping
=========================

`[This technical approach is discussed here but is 
discouraged because of technical "Complications" and 
"Dubious Performance Improvements" discussed at the 
end of this section.]`

Most ARMv7-M architectures support two mechanism for handling interrupts:

* The so-called `common` vector handler logic enabled with 
  ``CONFIG_ARMV7M_CMNVECTOR=y`` that can be found in 
  ``arch/arm/src/armv7-m/``, and
* MCU-specific interrupt handling logic. For the 
  STM32, this logic can be found at ``arch/arm/src/stm32/gnu/stm32_vectors.S``.

The `common` vector logic is slightly more efficient, 
the MCU-specific logic is slightly more flexible.

If we don't use the `common` vector logic enabled with 
``CONFIG_ARMV7M_CMNVECTOR=y``, but instead the more 
flexible MCU-specific implementation, then we can 
also use this to map the large set of hardware 
interrupt vector numbers to a smaller set of software 
interrupt numbers. This involves minimal changes to 
the OS and does not require any magic software lookup 
table. But is considerably more complex to implement.

This technical approach requires changes to three files:

* A new header file at ``arch/arm/include/stm32``, say 
  ``xyz_irq.h`` for the purposes of this discussion. 
  This new header file is like the other IRQ definition 
  header files in that directory except that it 
  defines only the IRQ number of the interrupts after 
  remapping. So, instead of having the 100 IRQ number 
  definitions of the original IRQ header file based on 
  the physical vector numbers, this header file would 
  define ``only`` the small set of 20 ``mapped`` IRQ numbers in 
  the range from 0 through 19. It would also set ``NR_IRQS`` 
  to the value 20.
* A new header file at ``arch/arm/src/stm32/hardware``, say 
  ``xyz_vector.h``. It would be similar to the other vector 
  definitions files in that directory: It will consist 
  of a sequence of 100 ``VECTOR`` and ``UNUSED`` macros. It will 
  define ``VECTOR`` entries for the 20 valid interrupts and 
  80 ``UNUSED`` entries for the unused interrupt vector numbers. 
  More about this below.
* Modification of the ``stm32_vectors.S`` file. These changes 
  are trivial and involve only the conditional inclusion 
  of the new, special ``xyz_vectors.h`` header file.

**REVISIT**: This needs to be updated. Neither the ``xyz_vector.h`` 
files nor the ``stm32_vectors.S`` exist in the current realization. 
This has all been replaced with the common vector handling at 
``arch/arm/src/armv7-m``.

Vector Definitions
==================

In ``arch/arm/src/stm32/gnu/stm32_vector.S``, notice that the 
``xyz_vector.h`` file will be included twice. Before each 
inclusion, the macros ``VECTOR`` and ``UNUSED`` are defined.

The first time that ``xyz_vector.h`` included, it defines the 
hardware vector table. The hardware vector table consists 
of ``NR_IRQS`` 32-bit addresses in an array. This is 
accomplished by setting:

.. code-block:: c 

    #undef VECTOR
    #define VECTOR(l,i) .word l
    
    #undef UNUSED
    #define UNUSED(i)   .word stm32_reserved

Then including ``xyz_vector.h``. So consider the following 
definitions in the original file:

.. code-block:: c

    ...
    VECTOR(stm32_usart1, STM32_IRQ_USART1) /* Vector 16+37: USART1 global interrupt */
    VECTOR(stm32_usart2, STM32_IRQ_USART2) /* Vector 16+38: USART2 global interrupt */
    VECTOR(stm32_usart3, STM32_IRQ_USART3) /* Vector 16+39: USART3 global interrupt */
    ...

Suppose that we wanted to support only USART1 and that 
we wanted to have the IRQ number for USART1 to be 12. 
That would be accomplished in the ``xyz_vector.h`` header 
file like this:

.. code-block:: c

    ...
    VECTOR(stm32_usart1, STM32_IRQ_USART1) /* Vector 16+37: USART1 global interrupt */
    UNUSED(0)                              /* Vector 16+38: USART2 global interrupt */
    UNUSED(0)                              /* Vector 16+39: USART3 global interrupt */
    ...

Where the value of ``STM32_IRQ_USART1`` was defined to 
be 12 in the ``arch/arm/include/stm32/xyz_irq.h`` header 
file. When ``xyz_vector.h`` is included by ``stm32_vectors.S`` 
with the above definitions for ``VECTOR`` and ``UNUSED``, the 
following would result:

.. code-block:: c 

    ...
    .word stm32_usart1
    .word stm32_reserved
    .word stm32_reserved
    ...

These are the settings for vector 53, 54, and 55, 
respectively. The entire vector table would be populated 
in this way. ``stm32_reserved``, if called would result in 
an "unexpected ISR" crash. ``stm32_usart1``, if called will 
process the USART1 interrupt normally as we will see below.

Interrupt Handler Definitions
-----------------------------

in the vector table, all of the valid vectors are set to 
the address of a `handler` function. All unused vectors 
are force to vector to ``stm32_reserved``. Currently, only 
vectors that are not supported by the hardware are 
marked ``UNUSED``, but you can mark any vector ``UNUSED`` in 
order to eliminate it.

The second time that ``xyz_vector.h`` is included by 
``stm32_vector.S``, the `handler` functions are generated. 
Each of the valid vectors point to the matching handler 
function. In this case, you do NOT have to provide 
handlers for the ``UNUSED`` vectors, only for the used 
``VECTOR`` vectors. All of the unused vectors will go 
to the common ``stm32_reserved`` handler. The remaining 
set of handlers is very sparse.

These are the values of ``UNUSED`` and ``VECTOR`` macros on the 
second time the ``xzy_vector.h`` is included by ``stm32_vectors.S``:

.. code-block:: asm

    .macro HANDLER, label, irqno
        .thumb_func
    label:
        mov r0, #\irqno
        b       exception_common
    .endm
    
    #undef VECTOR
    #define VECTOR(l,i) HANDLER l, i
    
    #undef UNUSED
    #define UNUSED(i)

In the above USART1 example, a single handler would be 
generated that will provide the IRQ number 12. Remember 
that 12 is the expansion of the macro ``STM32_IRQ_USART1`` 
that is provided in the ``arch/arm/include/stm32/xyz_irq.h`` 
header file:

.. code-block:: asm 

        .thumb_func
    stm32_usart1:
        mov r0, #12
        b       exception_common

Now, when vector 16+37 occurs it is mapped to IRQ 12 
with no significant software overhead.

A Complication
--------------

A complication in the above logic has been noted by David Sidrane: 
When we access the NVIC in ``stm32_irq.c`` in order to enable 
and disable interrupts, the logic requires the physical 
vector number in order to select the NVIC register and 
the bit(s) the modify in the NVIC register.

This could be handled with another small IRQ lookup table 
(20 ``uint8_t`` entries in our example situation above). But 
then this approach is not so much better than the `Software 
Vector Mapping` described about which does not suffer from 
this problem. Certainly enabling/disabling interrupts in a 
much lower rate operation and at least does not put the 
lookup in the critical interrupt path.

Another option suggested by David Sidrane is equally ugly:

* Don't change the ``arch/arm/include/stm32`` IRQ definition file.
* Instead, encode the IRQ number so that it has both 
  the index and physical vector number:

.. code-block:: c 

    ...
    VECTOR(stm32_usart1, STM32_IRQ_USART1 << 8 | STM32_INDEX_USART1)
    UNUSED(0)
    UNUSED(0)
    ...

The STM32_INDEX_USART1 would have the value 12 and 
STM32_IRQ_USART1 would be as before (53). This encoded 
value would be received by ``irq_dispatch()`` and it would 
decode both the index and the physical vector number. 
It would use the index to look up in the ``g_irqvector[]`` 
table but would pass the physical vector number to the 
interrupt handler as the IRQ number.

A lookup would still be required in ``irq_attach()`` in 
order to convert the physical vector number back to 
an index (100 ``uint8_t`` entries in our example). So 
some lookup is unavoidable.

Based upon these analysis, my recommendation is that 
we do not consider the second option any further. The 
first option is cleaner, more portable, and generally 
preferable.is well worth that.

Dubious Performance Improvements
--------------------------------

The intent of this second option was to provide a higher 
performance mapping of physical interrupt vectors to IRQ 
numbers compared to the pure software mapping of option 1. However, 
in order to implement this approach, we had 
to use the less efficient, non-common vector handling 
logic. That logic is not terribly less efficient, the 
cost is probably only a 16 bit load immediate instruction 
and branch to another location in FLASH (which will cause 
the CPU pipeline to be flushed).

The variant of option 2 where both the physical vector number 
and vector table index are encoded would require even more 
processing in ``irq_dispatch()`` in order to decode the 
physical vector number and vector table index. 
Possible just AND and SHIFT instructions.

However, the minimal cost of the first pure software 
mapping approach was possibly as small as a single 
indexed byte fetch from FLASH in ``irq_attach()``. 
Indexing is, of course, essentially `free` in the ARM 
ISA, the primary cost would be the FLASH memory access. 
So my first assessment is that the performance of both 
approaches is the essentially the same. If anything, the 
first approach is possibly the more performant if 
implemented efficiently.

Both options would require some minor range checking in 
``irq_attach()`` as well.

Because of this and because of the simplicity of the 
first option, I see no reason to support or consider 
this second option any further.

Complexity and Generalizability
-------------------------------

Option 2 is overly complex; it depends on a deep understanding 
on how the MCU interrupt logic works and on a high level of 
Thumb assembly language skills.

Another problem with option 2 is that really only applies to 
the Cortex-M family of processors and perhaps others that 
support interrupt vectored interrupts in a similar fashion. 
It is not a general solution that can be used with any CPU 
architectures.

And even worse, the MCU-specific interrupt handling logic 
that this support depends upon is is very limited. As soon 
as the common interrupt handler logic was added, I stopped 
implementing the MCU specific logic in all newer ARMv7-M 
ports. So that MCU specific interrupt handler logic is 
only present for EFM32, Kinetis, LPC17, SAM3/4, STM32, 
Tiva, and nothing else. Very limited!

These are further reasons why option 2 is no recommended and 
will not be supported explicitly.
