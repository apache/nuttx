// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// Force emission of the std::__1::basic_string destructors for the
// character sets that NuttX enables.  The upstream libc++ headers declare
// the relevant template instantiations as "extern", so without providing
// explicit definitions the linker cannot resolve references coming from
// locale-heavy translation units such as locale.cpp.

#include <string>

_LIBCPP_BEGIN_NAMESPACE_STD

// Emit the full char specialization so all key functions (including dtors)
// are visible from libxx.a instead of remaining TU-local.
template class _LIBCPP_EXPORTED_FROM_ABI basic_string<char>;

#ifndef _LIBCPP_HAS_NO_WIDE_CHARACTERS
// Emit the wchar_t specialization to satisfy localization-widechar users.
template class _LIBCPP_EXPORTED_FROM_ABI basic_string<wchar_t>;
#endif

_LIBCPP_END_NAMESPACE_STD
