==============
DEVMEM Drivers
==============

The `devmem` driver provides an interface for accessing memory-mapped
I/O in an embedded system. This driver allows for reading, writing, and
memory mapping of specific memory regions or device registers.

``read()``: This function reads data from the memory-mapped I/O address
  space into the buffer provided by the caller. The first byte read
  corresponds to the address specified by the device's "current memory
  address". The addresses for subsequent bytes depend on the auto-increment
  behavior of the specific device.

``write()``: This function transfers data from the provided data buffer by
  the caller to the memory-mapped I/O address space. The first byte written
  corresponds to the address specified by the device's "current memory address".

``mmap()``: The `mmap()` function provides a mechanism to map a device's
  memory region into the user space, allowing direct access to device
  registers or memory regions. The mapped region can be accessed using
  normal memory operations.

  The function requires a base address and size for the memory region
  to be mapped. If successful, it returns a pointer to the mapped region.
  If mapping fails, it returns `EINVAL` and `errno` is set appropriately.
