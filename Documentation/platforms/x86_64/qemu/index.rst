===========
QEMU x86_64
===========

**QEMU/Intel64** An x86_64 flat address port was ported in NuttX-9.0. It
consists of the following feautres:

- Runs in x86_64 long mode.
- Configurable SSE/AVX support.
- IRQs are managed by LAPIC(X2APIC) and IOAPIC.
- Used TSC_DEADLINE or APIC timer for systick.
- Pages are now maps the kernel at 4GB~, but changeable.

This kernel with ostest have been tested with

-  Qemu/KVM on a Xeon 2630v4 machine.
-  Bochs with broadwell_ult emulation.
