
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(SYSTEM ${NUTTX_DIR}/include/cxx)

set(CXXSRCS)

#if(NOT "${CONFIG_UCLIBCXX_EXCEPTION}" STREQUAL "y")
#	list(APPEND CXXSRCS libxx__gnu_unwind_find_exidx.cxx)
#endif()

# Some of the libs/libxx/ files are not need if uClibc++ or libcxx is installed
# because uClibx++ or libcxx will replace them
if("${CONFIG_LIBCXX}" STREQUAL "y")
	list(APPEND CXXSRCS
		libxx_delete.cxx libxx_delete_sized.cxx libxx_deletea.cxx
		libxx_deletea_sized.cxx libxx_new.cxx libxx_newa.cxx
		libxx_stdthrow.cxx
	)
endif()

# Include the uClibc++ Make.defs file if selected.  If it is included,
# the uClibc++/Make.defs file will add its files to the source file list,
# add its DEPPATH info, and will add the appropriate paths to the VPATH
# variable
#
# Note that an error will occur if you select CONFIG_LIBXX_UCLIBCXX
# without installing the uClibc++ package.  This is intentional to let
# you know about the configuration problem.  Refer to the README.txt file
# in the NuttX uClibc++ GIT repository for more information

if("${CONFIG_UCLIBCXX}" STREQUAL "y")
	message(FATAL_ERROR "REVIEW")
	#include uClibc++/Make.defs
endif()


nuttx_add_library(cxx
	libxx_cxapurevirtual.cxx libxx_eabi_atexit.cxx libxx_cxa_atexit.cxx
	libxx_cxa_guard.cxx

	${CXXSRCS}
)

