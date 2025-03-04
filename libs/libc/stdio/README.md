# lib_bsprintf
  This function is mainly used to output the contents of the input structure. Supports standard formats for printf and scanf.
  For detailed parameters, see:
  1. https://en.cppreference.com/w/c/io/fprintf
  2. https://en.cppreference.com/w/c/io/fscanf

- **special**:
  1. Float use %hf, "%f" or "%lf" is double, "%Lf" is long double.
  2. The char array is specified with %.xs. for example: "char t[30]" is specified with "%.30s", char a [20] - " %.20s "
  3. "%u" is unsigned int.
  4. "%d" is int.
  5. When using %f to format a double data type, the double is truncated to 6 decimal places by default.
  6. It is recommended that the "char[]" array be placed at the end of the structure to prevent parameter configuration errors such as "%.20s" from causing problems in parsing the entire buffer.
- **demo**
  1. **struct**:
  ~~~
  begin_packed_struct
  struct test
  {
    uint8_t a;
    uint16_t b;
    uint32_t c;
    int8_t d;
    int16_t e;
    int32_t f;
    float g;
    double h;
    char i[32];
    uint64_t j;
    int64_t k;
    char l;
    unsigned char m;
    short int n;
    unsigned short int o;
    int p;
    unsigned int q;
    long r;
    unsigned long s;
    long long t;
    unsigned long long u;
    size_t v;
    long double w;
  }end_packed_struct;
  ~~~
  1. **format string**:
  ~~~
  const char* sg = "           uint8_t:[%hhu]\n" \
                   "          uint16_t:[%hu]\n" \
                   "          uint32_t:[%u]\n" \
                   "            int8_t:[%hhd]\n" \
                   "           int16_t:[%hd]\n" \
                   "           int32_t:[%d]\n" \
                   "             float:[%hf]\n" \
                   "            double:[%f]\n" \
                   "            char[]:[%.32s]\n" \
                   "          uint64_t:[%lu]\n" \
                   "           int64_t:[%ld]\n" \
                   "              char:[%hhd]\n" \
                   "     unsigned char:[%hhu]\n" \
                   "         short int:[%hd]\n" \
                   "unsigned short int:[%hu]\n" \
                   "               int:[%d]\n" \
                   "      unsigned int:[%u]\n" \
                   "              long:[%ld]\n" \
                   "     unsigned long:[%lu]\n" \
                   "         long long:[%lld]\n" \
                   "unsigned long long:[%llu]\n" \
                   "            size_t:[%uz]\n" \
                   "       long double:[%Lf]\n";
  ~~~
  1. **use**:
    -  output to terminal:
   ~~~
  #ifdef CONFIG_FILE_STREAM
    struct lib_stdoutstream_s stdoutstream;

    lib_stdoutstream(&stdoutstream, stdout);

    flockfile(stdout);
    lib_bsprintf(&stdoutstream.common, sv, &test_v);
    lib_bsprintf(&stdoutstream.common, sg, &test_g);
    funlockfile(stdout);
  #else
    struct lib_rawoutstream_s rawoutstream;
    struct lib_bufferedoutstream_s outstream;

    lib_rawoutstream(&rawoutstream, STDOUT_FILENO);
    lib_bufferedoutstream(&outstream, &rawoutstream.common);

    lib_bsprintf(&outstream.common, sv, &test_v);
    lib_bsprintf(&outstream.common, sg, &test_g);

    lib_stream_flush(&outstream.common);
  #endif
   ~~~

# lib_bscanf
  This function adds a formatted standard scanf string to the structure(lib_bscanf).
  1. https://zh.cppreference.com/w/c/io/fscanf

- **special**:
  1. Please use %lf for double precision, "%hf" or "%f" for float, long double ("%Lf") is not supported.
  2. Please use %hhd or %hhu for a single char or unsigned char.
  3. Use %hd or %hu for short int or unsigned short int.
  4. When using %s or %c, please specify the length of the char array, such as %32s, %32c.
  5. %s will check the string for spaces. When there are spaces in the string, it will be truncated. If you want to use string with spaces, please use %{length}c, but make sure that the length of the string can fill the array, otherwise an error will occur.
  6. %[] collection and %n are not supported.

- **demo**
  1. **struct**:
  Same as above
  1. **format string**:
  ~~~
  #define TOSTR(str)   #str
  #define TONNAME(name) TOSTR(name)

  #define v_uint8_t    97
  #define v_uint16_t   19299
  #define v_uint32_t   22155

  ......

  #define v_l_double   -9299.9299929912122464755474

  char bflag[] = "%hhu%hu%u%hhd%hd%d%f%lf%32s%llu%lld%hhd%hhu%hd%hu%d%u%ld%lu%lld%llu%zu%ld";

  char binput[] = TONNAME(v_uint8_t) \
                 " " TONNAME(v_uint16_t) \
                 " " TONNAME(v_uint32_t) \
                 " " TONNAME(v_int8_t) \
                 " " TONNAME(v_int16_t) \
                 " " TONNAME(v_int32_t) \
                 " " TONNAME(v_float) \
                 " " TONNAME(v_double) \
                 " " TONNAME(v_char_arr) \
                 " " TONNAME(v_uint64_t) \
                 " " TONNAME(v_int64_t) \
                 " " TONNAME(v_char) \
                 " " TONNAME(v_u_char) \
                 " " TONNAME(v_s_int) \
                 " " TONNAME(v_u_s_int) \
                 " " TONNAME(v_int) \
                 " " TONNAME(v_u_int) \
                 " " TONNAME(v_long) \
                 " " TONNAME(v_u_long) \
                 " " TONNAME(v_l_l) \
                 " " TONNAME(v_u_l_l) \
                 " " TONNAME(v_size_t) \
                 " " TONNAME(v_l_double);
  ~~~
  3. **use**:
  ~~~
  struct test vg;
  ret = lib_bscanf(binput, bflag, &vg);
  ~~~