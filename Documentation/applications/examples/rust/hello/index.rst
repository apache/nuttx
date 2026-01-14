==================================
`hello` Example in Rust
==================================

This example demonstrates how to use Rust's powerful features in a NuttX environment, including:

- **JSON Serialization/Deserialization**: Using the popular `serde` and `serde_json` crates to work with JSON data
- **Async Runtime**: Demonstrates basic usage of the `tokio` async runtime
- **C Interoperability**: Shows how to expose Rust functions to be called from C code

Key Features
------------

1. JSON Handling
   - Defines a `Person` struct with `Serialize` and `Deserialize` traits
   - Serializes Rust structs to JSON strings
   - Deserializes JSON strings into Rust structs
   - Demonstrates pretty-printing JSON

2. Async Runtime
   - Initializes a single-threaded `tokio` runtime
   - Runs a simple async task that prints a message

3. C Interop
   - Exports `hello_rust_cargo_main` function with `#[no_mangle]` for C calling
   - Uses `extern "C"` to define the C ABI

The example shows how Rust's modern features can be used in embedded systems while maintaining compatibility with C-based systems.

This example serves as a foundation for building more complex Rust applications in NuttX that need to handle JSON data and async operations.
