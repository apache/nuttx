# Parse nuttx config options for cmake

file(STRINGS ${CMAKE_BINARY_DIR}/.config ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if (Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		# remove extra quotes
		string(REPLACE "\"" "" Value ${Value})

		# Set the variable
		#message(STATUS "${Name} ${Value}")
		#set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
    set(${Name} ${Value})
	endif()
endforeach()

# Compatibility for symbols usually user-settable (now, set via env vars)
# TODO: change usage of these symbols into the corresponding cmake variables
if (LINUX)
  set(CONFIG_HOST_LINUX true)
elseif(APPLE)
  set(CONFIG_HOST_MACOS true)
elseif(WIN32)
  set(CONFIG_HOST_WINDOWS true)
else()
  set(CONFIG_HOST_OTHER true)
endif()
