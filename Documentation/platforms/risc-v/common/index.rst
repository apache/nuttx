===========================================================================
RISC-V Specific Features
===========================================================================

RISC-V CLIC Interrupt Threshold Configuration
==============================================

Overview
--------

The RISC-V Core-Level Interrupt Controller (CLIC) provides more flexible interrupt
control compared to the legacy CLINT. One key feature is the interrupt threshold
mechanism (INTTHRESH), which allows fine-grained control over which interrupts
are processed based on their priority levels.

CLIC Interrupt Threshold Basics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CLIC interrupt threshold works by:

1. **Threshold Register**: Each privilege mode has its own interrupt threshold register:
   - Machine mode: ``CSR_MINTTHRESH`` (0x347)
   - Supervisor mode: ``CSR_SINTTHRESH`` (0x147)

2. **Interrupt Filtering**: Only interrupts with priority levels above the current
   threshold are delivered to the processor.

3. **Context Preservation**: The threshold value must be saved and restored during
   context switches to maintain proper interrupt priority handling.

Configuration in Custom Chip Layer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Kconfig Configuration
"""""""""""""""""""""

To enable CLIC interrupt threshold support in your custom chip, add the following
to your chip's Kconfig file:

.. code-block:: kconfig

   config ARCH_CHIP_MYCUSTOM_CHIP
       bool "My Custom RISC-V Chip"
       select ARCH_RV32  # or ARCH_RV64
       select ARCH_RV_HAVE_CLIC
       ---help---
         My custom RISC-V chip with CLIC support

Required Definitions
""""""""""""""""""""

In your chip-specific header file (e.g., ``arch/risc-v/include/mycustom/irq.h``):

.. code-block:: c

   /* Define maximum interrupt threshold value for your CLIC implementation */
   #define RISCV_MAX_INTTHRESH    255  /* Adjust based on your CLIC design */

   /* Optional: Define interrupt priority levels */
   #define CLIC_PRIO_CRITICAL          200
   #define CLIC_PRIO_HIGH              150
   #define CLIC_PRIO_NORMAL            100
   #define CLIC_PRIO_LOW               50

.. note::
   If ``RISCV_MAX_INTTHRESH`` is not defined by the chip, NuttX will use a default
   value of ``0xff`` (255). This default should work for most CLIC implementations
   that support 8-bit interrupt threshold values. Chips with different threshold
   widths should define their own maximum value accordingly.

Implementation Details
^^^^^^^^^^^^^^^^^^^^^^

Interrupt Save/Restore Mechanism
"""""""""""""""""""""""""""""""""

When ``ARCH_RV_HAVE_CLIC`` is enabled, NuttX automatically uses the
interrupt threshold register instead of the standard status register for
interrupt control. The CLIC implementation uses the ``SWAP_CSR()`` macro
for atomic read-modify-write operations on the threshold register:

.. code-block:: c

   /* Standard RISC-V interrupt disable (without CLIC) */
   noinstrument_function static inline_function irqstate_t up_irq_save(void)
   {
     irqstate_t flags;

     /* Read mstatus & clear machine interrupt enable (MIE) in mstatus */

     __asm__ __volatile__
       (
         "csrrc %0, " __XSTR(CSR_STATUS) ", %1\n"
         : "=r" (flags)
         : "r"(STATUS_IE)
         : "memory"
       );

     return flags;
   }

   /* CLIC interrupt threshold disable (with ARCH_RV_HAVE_CLIC) */
   noinstrument_function static inline_function irqstate_t up_irq_save(void)
   {
     /* Read current interrupt threshold and set to maximum to mask all */

     return SWAP_CSR(CSR_INTTHRESH, RISCV_MAX_INTTHRESH);
   }

Context Switch Handling
"""""""""""""""""""""""

The interrupt context register (``REG_INT_CTX``) stores different values
depending on CLIC configuration:

.. code-block:: c

   /*
    * Without CLIC (standard RISC-V): REG_INT_CTX stores CSR_MSTATUS or CSR_SSTATUS
    * With CLIC (ARCH_RV_HAVE_CLIC): REG_INT_CTX stores CSR_MINTTHRESH or CSR_SINTTHRESH
    *
    * The interrupt context is automatically determined by the CONFIG_ARCH_RV_HAVE_CLIC
    * configuration, eliminating the need for a separate threshold enable option.
    */

CLIC Driver Implementation
""""""""""""""""""""""""""

To implement the CLIC driver with interrupt threshold support, you may need to configure
CLIC properly to handle priority management and threshold levels in your chip's
driver code.

Integration Checklist
^^^^^^^^^^^^^^^^^^^^^^

When integrating CLIC interrupt threshold support:

☐ Enable ``ARCH_RV_HAVE_CLIC`` in Kconfig
☐ Define ``RISCV_MAX_INTTHRESH`` for your chip (optional - defaults to 0xff/255)
☐ Implement CLIC driver with priority management
☐ Verify interrupt threshold save/restore in context switches
☐ Test interrupt priority levels and threshold functionality
☐ Add debug support for troubleshooting

References
^^^^^^^^^^

* RISC-V CLIC Specification
* NuttX RISC-V Architecture Documentation
* ``arch/risc-v/include/irq.h`` - Core interrupt handling definitions
* ``arch/risc-v/src/common/riscv_exception_common.S`` - Assembly interrupt handling
