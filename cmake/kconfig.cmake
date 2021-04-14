# Parse nuttx config options for cmake

file(STRINGS ${NUTTX_DIR}/.config ConfigContents)
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
		set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
	endif()
endforeach()

