README
^^^^^^

README for NuttX port to the SAM4CMP-DB board.

  http://www.atmel.com/tools/SAM4CMP-DB.aspx

The board is intended to test NuttX SMP features for dual Cortex-M4.


Settings
^^^^^^^^
1. Both CPUs are running at 92.160MHz with PLLB.
2. Serial console can be used via on-board USB-UART (115200/8/N/1)
3. Interrupt handlers such as timer and UART are handled on CPU0
4. Both CPUs share internal SRAM0 (128KB)
5. SRAM1 is used to boot CPU1.
6. Cache controllers are disabled because of no snooping features.

Status
^^^^^^
Currently SMP freature works on the board but is not stable.

1. "nsh> sleep 1 &" works without crash.
2. "nsh> smp " sometimes works but some assertions might happen.
3. "nsh> ostest " causes deadlocks during the test.
