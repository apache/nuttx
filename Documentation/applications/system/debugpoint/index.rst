=====================================
``debugpoint`` Debug Utility
=====================================

``CONFIG_SYSTEM_DEBUGPOINT=y``

The ``debugpoint`` utility is a tool for testing and managing debug points (breakpoints and watchpoints) in the system. It allows users to set, remove, and test various types of debug points.

Usage::

    debugpoint [options]

Options::

    -r addr  Set a read watchpoint at address
    -w addr  Set a write watchpoint at address
    -b addr  Set a breakpoint at address
    -x addr  Set a read/write watchpoint at address
    -c       Cancel the watchpoint or breakpoint (must be used with -r, -w, -b, or -x)
    -l len   Set the watch length (must be used with -r, -w, -b, or -x)

Examples::

    # Set a read watchpoint at address 0x1000
    debugpoint -r 0x1000

    # Set a write watchpoint at address 0x2000
    debugpoint -w 0x2000

    # Set a breakpoint at address 0x3000
    debugpoint -b 0x3000

    # Set a read/write watchpoint at address 0x4000
    debugpoint -x 0x4000

    # Cancel the read watchpoint at address 0x1000
    debugpoint -r 0x1000 -c

    # Cancel the write watchpoint at address 0x2000
    debugpoint -w 0x2000 -c

    # Cancel the breakpoint at address 0x3000
    debugpoint -b 0x3000 -c

    # Cancel the read/write watchpoint at address 0x4000
    debugpoint -x 0x4000 -c

    # Set the watch length to 8 bytes for a read watchpoint at address 0x1000
    debugpoint -r 0x1000 -l 8

    # Set the watch length to 8 bytes for a write watchpoint at address 0x2000
    debugpoint -w 0x2000 -l 8

    # Set the watch length to 8 bytes for a breakpoint at address 0x3000
    debugpoint -b 0x3000 -l 8

The ``debug`` utility also includes automated tests for breakpoints and watchpoints. When run without any options, it will execute these tests to verify the functionality of the debug points.
