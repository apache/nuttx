==================================
`baremetal` Hello World in Rust
==================================

This example demonstrates how to create a simple "Hello World" program in Rust for a bare-metal environment. The program is compiled using the `rustc` compiler directly, without relying on any operating system or standard library.

The key aspects of this example include:

- **No Standard Library**: The program uses the `#![no_std]` attribute, which means it does not link against the standard library. This is essential for bare-metal programming where the standard library is not available.
- **No Main Function**: The program uses the `#![no_main]` attribute, which indicates that the program does not have a standard `main` function. Instead, it defines a custom entry point.
- **Panic Handler**: A custom panic handler is defined using the `#[panic_handler]` attribute. This handler is called when a panic occurs, and in this case, it enters an infinite loop to halt the program.
- **C Interoperability**: The program uses the `extern "C"` block to declare the `printf` function from the C standard library. This allows the Rust program to call C functions directly.
- **Entry Point**: The `hello_rust_main` function is the entry point of the program. It is marked with `#[no_mangle]` to prevent the Rust compiler from mangling its name, making it callable from C.
- **Printing**: The program uses the `printf` function to print "Hello, Rust!!" to the console. The `printf` function is called using the `unsafe` block because it involves calling a C function.

This example is a great starting point for understanding how to write and compile Rust programs for bare-metal environments, where you have full control over the hardware and no operating system overhead.
