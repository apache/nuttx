list(APPEND SRCS
	lib_meminstream.c lib_memoutstream.c lib_memsistream.c
	lib_memsostream.c lib_lowoutstream.c lib_rawinstream.c
	lib_rawoutstream.c lib_rawsistream.c lib_rawsostream.c
	lib_zeroinstream.c lib_nullinstream.c lib_nulloutstream.c
	lib_libnoflush.c lib_libsnoflush.c lib_stdoutstream.c
)

target_sources(c PRIVATE ${SRCS})
