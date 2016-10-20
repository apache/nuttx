/****************************************************************************
 * arch/xtensa/include/esp32/core-isa.h
 * Xtensa processor CORE configuration
 *
 * Customer ID=11657; Build=0x5fe96; Copyright (c) 1999-2016 Tensilica Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_INCLUDE_ESP32_CORE_ISA_H
#define __ARCH_XTENSA_INCLUDE_ESP32_CORE_ISA_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Note:  Macros of the form XTENSA_HAVE_*** have a value of 1 if the option
 * is configured, and a value of 0 otherwise.  These macros are always
 * defined.
 */

/* ISA **********************************************************************/

#define XTENSA_HAVE_BE                    0    /* big-endian byte ordering */
#define XTENSA_HAVE_WINDOWED              1    /* windowed registers option */
#define XTENSA_NUM_AREGS                  64   /* num of physical addr regs */
#define XTENSA_NUM_AREGS_LOG2             6    /* log2(XTENSA_NUM_AREGS) */
#define XTENSA_MAX_INSTRUCTION_SIZE       3    /* max instr bytes (3..8) */
#define XTENSA_HAVE_DEBUG                 1    /* debug option */
#define XTENSA_HAVE_DENSITY               1    /* 16-bit instructions */
#define XTENSA_HAVE_LOOPS                 1    /* zero-overhead loops */
#define XTENSA_LOOP_BUFFER_SIZE           256  /* zero-ov. loop instr buffer size */
#define XTENSA_HAVE_NSA                   1    /* NSA/NSAU instructions */
#define XTENSA_HAVE_MINMAX                1    /* MIN/MAX instructions */
#define XTENSA_HAVE_SEXT                  1    /* SEXT instruction */
#define XTENSA_HAVE_DEPBITS               0    /* DEPBITS instruction */
#define XTENSA_HAVE_CLAMPS                1    /* CLAMPS instruction */
#define XTENSA_HAVE_MUL16                 1    /* MUL16S/MUL16U instructions */
#define XTENSA_HAVE_MUL32                 1    /* MULL instruction */
#define XTENSA_HAVE_MUL32_HIGH            1    /* MULUH/MULSH instructions */
#define XTENSA_HAVE_DIV32                 1    /* QUOS/QUOU/REMS/REMU instructions */
#define XTENSA_HAVE_L32R                  1    /* L32R instruction */
#define XTENSA_HAVE_ABSOLUTE_LITERALS     0    /* non-PC-rel (extended) L32R */
#define XTENSA_HAVE_CONST16               0    /* CONST16 instruction */
#define XTENSA_HAVE_ADDX                  1    /* ADDX#/SUBX# instructions */
#define XTENSA_HAVE_WIDE_BRANCHES         0    /* B*.W18 or B*.W15 instr's */
#define XTENSA_HAVE_PREDICTED_BRANCHES    0    /* B[EQ/EQZ/NE/NEZ]T instr's */
#define XTENSA_HAVE_CALL4AND12            1    /* (obsolete option) */
#define XTENSA_HAVE_ABS                   1    /* ABS instruction */
/*#define XTENSA_HAVE_POPC                0*/  /* POPC instruction */
/*#define XTENSA_HAVE_CRC                 0*/  /* CRC instruction */
#define XTENSA_HAVE_RELEASE_SYNC          1    /* L32AI/S32RI instructions */
#define XTENSA_HAVE_S32C1I                1    /* S32C1I instruction */
#define XTENSA_HAVE_SPECULATION           0    /* speculation */
#define XTENSA_HAVE_FULL_RESET            1    /* all regs/state reset */
#define XTENSA_NUM_CONTEXTS               1    /* */
#define XTENSA_NUM_MISC_REGS              4    /* num of scratch regs (0..4) */
#define XTENSA_HAVE_TAP_MASTER            0    /* JTAG TAP control instr's */
#define XTENSA_HAVE_PRID                  1    /* processor ID register */
#define XTENSA_HAVE_EXTERN_REGS           1    /* WER/RER instructions */
#define XTENSA_HAVE_MX                    0    /* MX core (Tensilica internal) */
#define XTENSA_HAVE_MP_INTERRUPTS         0    /* interrupt distributor port */
#define XTENSA_HAVE_MP_RUNSTALL           0    /* core RunStall control port */
#define XTENSA_HAVE_PSO                   0    /* Power Shut-Off */
#define XTENSA_HAVE_PSO_CDM               0    /* core/debug/mem pwr domains */
#define XTENSA_HAVE_PSO_FULL_RETENTION    0    /* all regs preserved on PSO */
#define XTENSA_HAVE_THREADPTR             1    /* THREADPTR register */
#define XTENSA_HAVE_BOOLEANS              1    /* boolean registers */
#define XTENSA_HAVE_CP                    1    /* CPENABLE reg (coprocessor) */
#define XTENSA_CP_MAXCFG                  8    /* max allowed cp id plus one */
#define XTENSA_HAVE_MAC16                 1    /* MAC16 package */

#define XTENSA_HAVE_FUSION                0    /* Fusion*/
#define XTENSA_HAVE_FUSION_FP             0    /* Fusion FP option */
#define XTENSA_HAVE_FUSION_LOW_POWER      0    /* Fusion Low Power option */
#define XTENSA_HAVE_FUSION_AES            0    /* Fusion BLE/Wifi AES-128 CCM option */
#define XTENSA_HAVE_FUSION_CONVENC        0    /* Fusion Conv Encode option */
#define XTENSA_HAVE_FUSION_LFSR_CRC       0    /* Fusion LFSR-CRC option */
#define XTENSA_HAVE_FUSION_BITOPS         0    /* Fusion Bit Operations Support option */
#define XTENSA_HAVE_FUSION_AVS            0    /* Fusion AVS option */
#define XTENSA_HAVE_FUSION_16BIT_BASEBAND 0    /* Fusion 16-bit Baseband option */
#define XTENSA_HAVE_FUSION_VITERBI        0    /* Fusion Viterbi option */
#define XTENSA_HAVE_FUSION_SOFTDEMAP      0    /* Fusion Soft Bit Demap option */
#define XTENSA_HAVE_HIFIPRO               0    /* HiFiPro Audio Engine pkg */
#define XTENSA_HAVE_HIFI4                 0    /* HiFi4 Audio Engine pkg */
#define XTENSA_HAVE_HIFI4_VFPU            0    /* HiFi4 Audio Engine VFPU option */
#define XTENSA_HAVE_HIFI3                 0    /* HiFi3 Audio Engine pkg */
#define XTENSA_HAVE_HIFI3_VFPU            0    /* HiFi3 Audio Engine VFPU option */
#define XTENSA_HAVE_HIFI2                 0    /* HiFi2 Audio Engine pkg */
#define XTENSA_HAVE_HIFI2EP               0    /* HiFi2EP */
#define XTENSA_HAVE_HIFI_MINI             0    

#define XTENSA_HAVE_VECTORFPU2005         0    /* vector or user floating-point pkg */
#define XTENSA_HAVE_USER_DPFPU            0    /* user DP floating-point pkg */
#define XTENSA_HAVE_USER_SPFPU            0    /* user DP floating-point pkg */
#define XTENSA_HAVE_FP                    1    /* single prec floating point */
#define XTENSA_HAVE_FP_DIV                1    /* FP with DIV instructions */
#define XTENSA_HAVE_FP_RECIP              1    /* FP with RECIP instructions */
#define XTENSA_HAVE_FP_SQRT               1    /* FP with SQRT instructions */
#define XTENSA_HAVE_FP_RSQRT              1    /* FP with RSQRT instructions */
#define XTENSA_HAVE_DFP                   0    /* double precision FP pkg */
#define XTENSA_HAVE_DFP_DIV               0    /* DFP with DIV instructions */
#define XTENSA_HAVE_DFP_RECIP             0    /* DFP with RECIP instructions*/
#define XTENSA_HAVE_DFP_SQRT              0    /* DFP with SQRT instructions */
#define XTENSA_HAVE_DFP_RSQRT             0    /* DFP with RSQRT instructions*/
#define XTENSA_HAVE_DFP_ACCEL             1    /* double precision FP acceleration pkg */
#define XTENSA_HAVE_DFP_accel             XTENSA_HAVE_DFP_ACCEL /* for backward compatibility */

#define XTENSA_HAVE_DFPU_SINGLE_ONLY      1   /* DFPU Coprocessor, single precision only */
#define XTENSA_HAVE_DFPU_SINGLE_DOUBLE    0   /* DFPU Coprocessor, single and double precision */
#define XTENSA_HAVE_VECTRA1               0   /* Vectra I  pkg */
#define XTENSA_HAVE_VECTRALX              0   /* Vectra LX pkg */
#define XTENSA_HAVE_PDX4                  0   /* PDX4 */
#define XTENSA_HAVE_CONNXD2               0   /* ConnX D2 pkg */
#define XTENSA_HAVE_CONNXD2_DUALLSFLIX    0   /* ConnX D2 & Dual LoadStore Flix */
#define XTENSA_HAVE_BBE16                 0   /* ConnX BBE16 pkg */
#define XTENSA_HAVE_BBE16_RSQRT           0   /* BBE16 & vector recip sqrt */
#define XTENSA_HAVE_BBE16_VECDIV          0   /* BBE16 & vector divide */
#define XTENSA_HAVE_BBE16_DESPREAD        0   /* BBE16 & despread */
#define XTENSA_HAVE_BBENEP                0   /* ConnX BBENEP pkgs */
#define XTENSA_HAVE_BSP3                  0   /* ConnX BSP3 pkg */
#define XTENSA_HAVE_BSP3_TRANSPOSE        0   /* BSP3 & transpose32x32 */
#define XTENSA_HAVE_SSP16                 0   /* ConnX SSP16 pkg */
#define XTENSA_HAVE_SSP16_VITERBI         0   /* SSP16 & viterbi */
#define XTENSA_HAVE_TURBO16               0   /* ConnX Turbo16 pkg */
#define XTENSA_HAVE_BBP16                 0   /* ConnX BBP16 pkg */
#define XTENSA_HAVE_FLIX3                 0   /* basic 3-way FLIX option */
#define XTENSA_HAVE_GRIVPEP               0   /* GRIVPEP is General Release of IVPEP */
#define XTENSA_HAVE_GRIVPEP_HISTOGRAM     0   /* Histogram option on GRIVPEP */

/* Misc. ********************************************************************/

#define XTENSA_NUM_LOADSTORE_UNITS        1    /* load/store units */
#define XTENSA_NUM_WRITEBUFFER_ENTRIES    4    /* size of write buffer */
#define XTENSA_INST_FETCH_WIDTH           4    /* instr-fetch width in bytes */
#define XTENSA_DATA_WIDTH                 4    /* data width in bytes */
#define XTENSA_DATA_PIPE_DELAY            2    /* d-side pipeline delay
                                                * (1 = 5-stage, 2 = 7-stage) */
#define XTENSA_CLOCK_GATING_GLOBAL        1    /* global clock gating */
#define XTENSA_CLOCK_GATING_FUNCUNIT      1    /* funct. unit clock gating */
/* In T1050, applies to selected core load and store instructions (see ISA): */
#define XTENSA_UNALIGNED_LOAD_EXCEPTION   0    /* unaligned loads cause exc. */
#define XTENSA_UNALIGNED_STORE_EXCEPTION  0    /* unaligned stores cause exc.*/
#define XTENSA_UNALIGNED_LOAD_HW          1    /* unaligned loads work in hw */
#define XTENSA_UNALIGNED_STORE_HW         1    /* unaligned stores work in hw*/

#define XTENSA_SW_VERSION                 1100003 /* sw version of this header */

#define XTENSA_CORE_ID                    "esp32_v3_49_prod" /* alphanum core name
                                                  * (CoreID) set in the Xtensa
                                                  * Processor Generator */

#define XTENSA_BUILD_UNIQUE_ID            0x0005fe96 /* 22-bit sw build ID */

/*
 *  These definitions describe the hardware targeted by this software.
 */
#define XTENSA_HW_CONFIGID0                0xc2bcfffe  /* ConfigID hi 32 bits*/
#define XTENSA_HW_CONFIGID1                0x1cc5fe96  /* ConfigID lo 32 bits*/
#define XTENSA_HW_VERSION_NAME            "LX6.0.3"    /* full version name */
#define XTENSA_HW_VERSION_MAJOR            2600    /* major ver# of targeted hw */
#define XTENSA_HW_VERSION_MINOR            3       /* minor ver# of targeted hw */
#define XTENSA_HW_VERSION                  260003  /* major*100+minor */
#define XTENSA_HW_REL_LX6                  1
#define XTENSA_HW_REL_LX6_0                1
#define XTENSA_HW_REL_LX6_0_3              1
#define XTENSA_HW_CONFIGID_RELIABLE        1

/* If software targets a *range* of hardware versions, these are the bounds: */

#define XTENSA_HW_MIN_VERSION_MAJOR        2600    /* major v of earliest tgt hw */
#define XTENSA_HW_MIN_VERSION_MINOR        3       /* minor v of earliest tgt hw */
#define XTENSA_HW_MIN_VERSION              260003  /* earliest targeted hw */
#define XTENSA_HW_MAX_VERSION_MAJOR        2600    /* major v of latest tgt hw */
#define XTENSA_HW_MAX_VERSION_MINOR        3       /* minor v of latest tgt hw */
#define XTENSA_HW_MAX_VERSION              260003  /* latest targeted hw */

/* Cache ********************************************************************/

#define XTENSA_ICACHE_LINESIZE            4    /* I-cache line size in bytes */
#define XTENSA_DCACHE_LINESIZE            4    /* D-cache line size in bytes */
#define XTENSA_ICACHE_LINEWIDTH           2    /* log2(I line size in bytes) */
#define XTENSA_DCACHE_LINEWIDTH           2    /* log2(D line size in bytes) */

#define XTENSA_ICACHE_SIZE                0    /* I-cache size in bytes or 0 */
#define XTENSA_DCACHE_SIZE                0    /* D-cache size in bytes or 0 */

#define XTENSA_DCACHE_IS_WRITEBACK        0    /* writeback feature */
#define XTENSA_DCACHE_IS_COHERENT         0    /* MP coherence feature */

#define XTENSA_HAVE_PREFETCH              0    /* PREFCTL register */
#define XTENSA_HAVE_PREFETCH_L1           0    /* prefetch to L1 dcache */
#define XTENSA_PREFETCH_CASTOUT_LINES     0    /* dcache pref. castout bufsz */
#define XTENSA_PREFETCH_ENTRIES           0    /* cache prefetch entries */
#define XTENSA_PREFETCH_BLOCK_ENTRIES     0    /* prefetch block streams */
#define XTENSA_HAVE_CACHE_BLOCKOPS        0    /* block prefetch for caches */
#define XTENSA_HAVE_ICACHE_TEST           0    /* Icache test instructions */
#define XTENSA_HAVE_DCACHE_TEST           0    /* Dcache test instructions */
#define XTENSA_HAVE_ICACHE_DYN_WAYS       0    /* Icache dynamic way support */
#define XTENSA_HAVE_DCACHE_DYN_WAYS       0    /* Dcache dynamic way support */

/* Parameters Useful for PRIVILEGED (Supervisory or Non-Virtualized) Code */

#ifndef XTENSA_HAL_NON_PRIVILEGED_ONLY

/* Cache ********************************************************************/

#define XTENSA_HAVE_PIF                   1    /* any outbound PIF present */
#define XTENSA_HAVE_AXI                   0    /* AXI bus */

#define XTENSA_HAVE_PIF_WR_RESP           0    /* pif write response */
#define XTENSA_HAVE_PIF_REQ_ATTR          0    /* pif attribute */

/* If present, cache size in bytes == (ways * 2^(linewidth + setwidth)). */

/* Number of cache sets in log2(lines per way): */

#define XTENSA_ICACHE_SETWIDTH            0
#define XTENSA_DCACHE_SETWIDTH            0

/* Cache set associativity (number of ways): */

#define XTENSA_ICACHE_WAYS                1
#define XTENSA_DCACHE_WAYS                1

/* Cache features: */

#define XTENSA_ICACHE_LINE_LOCKABLE       0
#define XTENSA_DCACHE_LINE_LOCKABLE       0
#define XTENSA_ICACHE_ECC_PARITY          0
#define XTENSA_DCACHE_ECC_PARITY          0

/* Cache access size in bytes (affects operation of SICW instruction): */

#define XTENSA_ICACHE_ACCESS_SIZE         1
#define XTENSA_DCACHE_ACCESS_SIZE         1

#define XTENSA_DCACHE_BANKS               0    /* number of banks */

/* Number of encoded cache attr bits for decoded bits): */

#define XTENSA_CA_BITS                    4

/* Internal I/D RAM/ROMs and XLMI *******************************************/

#define XTENSA_NUM_INSTROM                1    /* number of core instr. ROMs */
#define XTENSA_NUM_INSTRAM                2    /* number of core instr. RAMs */
#define XTENSA_NUM_DATAROM                1    /* number of core data ROMs */
#define XTENSA_NUM_DATARAM                2    /* number of core data RAMs */
#define XTENSA_NUM_URAM                   0    /* number of core unified RAMs*/
#define XTENSA_NUM_XLMI                   1    /* number of core XLMI ports */

/* Instruction ROM 0: */

#define XTENSA_INSTROM0_VADDR             0x40800000 /* virtual address */
#define XTENSA_INSTROM0_PADDR             0x40800000 /* physical address */
#define XTENSA_INSTROM0_SIZE              4194304    /* size in bytes */
#define XTENSA_INSTROM0_ECC_PARITY        0    /* ECC/parity type, 0=none */

/* Instruction RAM 0: */

#define XTENSA_INSTRAM0_VADDR             0x40000000 /* virtual address */
#define XTENSA_INSTRAM0_PADDR             0x40000000 /* physical address */
#define XTENSA_INSTRAM0_SIZE              4194304    /* size in bytes */
#define XTENSA_INSTRAM0_ECC_PARITY        0    /* ECC/parity type, 0=none */

/* Instruction RAM 1: */

#define XTENSA_INSTRAM1_VADDR             0x40400000 /* virtual address */
#define XTENSA_INSTRAM1_PADDR             0x40400000 /* physical address */
#define XTENSA_INSTRAM1_SIZE              4194304    /* size in bytes */
#define XTENSA_INSTRAM1_ECC_PARITY        0    /* ECC/parity type, 0=none */

/* Data ROM 0: */

#define XTENSA_DATAROM0_VADDR             0x3F400000 /* virtual address */
#define XTENSA_DATAROM0_PADDR             0x3F400000 /* physical address */
#define XTENSA_DATAROM0_SIZE              4194304    /* size in bytes */
#define XTENSA_DATAROM0_ECC_PARITY        0    /* ECC/parity type, 0=none */
#define XTENSA_DATAROM0_BANKS             1    /* number of banks */

/* Data RAM 0: */
#define XTENSA_DATARAM0_VADDR             0x3FF80000 /* virtual address */
#define XTENSA_DATARAM0_PADDR             0x3FF80000 /* physical address */
#define XTENSA_DATARAM0_SIZE              524288     /* size in bytes */
#define XTENSA_DATARAM0_ECC_PARITY        0    /* ECC/parity type, 0=none */
#define XTENSA_DATARAM0_BANKS             1    /* number of banks */

/* Data RAM 1: */

#define XTENSA_DATARAM1_VADDR             0x3F800000 /* virtual address */
#define XTENSA_DATARAM1_PADDR             0x3F800000 /* physical address */
#define XTENSA_DATARAM1_SIZE              4194304    /* size in bytes */
#define XTENSA_DATARAM1_ECC_PARITY        0    /* ECC/parity type, 0=none */
#define XTENSA_DATARAM1_BANKS             1    /* number of banks */

/* XLMI Port 0: */

#define XTENSA_XLMI0_VADDR                0x3FF00000 /* virtual address */
#define XTENSA_XLMI0_PADDR                0x3FF00000 /* physical address */
#define XTENSA_XLMI0_SIZE                 524288    /* size in bytes */
#define XTENSA_XLMI0_ECC_PARITY           0    /* ECC/parity type, 0=none */

#define XTENSA_HAVE_IMEM_LOADSTORE        1    /* can load/store to IROM/IRAM*/

/* Interrupts and Timers ****************************************************/

#define XTENSA_HAVE_INTERRUPTS            1    /* interrupt option */
#define XTENSA_HAVE_HIGHPRI_INTERRUPTS    1    /* med/high-pri. interrupts */
#define XTENSA_HAVE_NMI                   1    /* non-maskable interrupt */
#define XTENSA_HAVE_CCOUNT                1    /* CCOUNT reg. (timer option) */
#define XTENSA_NUM_TIMERS                 3    /* number of CCOMPAREn regs */
#define XTENSA_NUM_INTERRUPTS             32   /* number of interrupts */
#define XTENSA_NUM_INTERRUPTS_LOG2        5    /* ceil(log2(NUM_INTERRUPTS)) */
#define XTENSA_NUM_EXTINTERRUPTS          26   /* num of external interrupts */
#define XTENSA_INT_NLEVELS                6    /* number of interrupt levels
                           (not including level zero) */
#define XTENSA_EXCM_LEVEL                 3    /* level masked by PS.EXCM */
    /* (always 1 in XEA1; levels 2 .. EXCM_LEVEL are "medium priority") */

/* Masks of interrupts at each interrupt level: */

#define XTENSA_INTLEVEL1_MASK             0x000637FF
#define XTENSA_INTLEVEL2_MASK             0x00380000
#define XTENSA_INTLEVEL3_MASK             0x28C08800
#define XTENSA_INTLEVEL4_MASK             0x53000000
#define XTENSA_INTLEVEL5_MASK             0x84010000
#define XTENSA_INTLEVEL6_MASK             0x00000000
#define XTENSA_INTLEVEL7_MASK             0x00004000

/* Masks of interrupts at each range 1..n of interrupt levels: */

#define XTENSA_INTLEVEL1_ANDBELOW_MASK    0x000637FF
#define XTENSA_INTLEVEL2_ANDBELOW_MASK    0x003E37FF
#define XTENSA_INTLEVEL3_ANDBELOW_MASK    0x28FEBFFF
#define XTENSA_INTLEVEL4_ANDBELOW_MASK    0x7BFEBFFF
#define XTENSA_INTLEVEL5_ANDBELOW_MASK    0xFFFFBFFF
#define XTENSA_INTLEVEL6_ANDBELOW_MASK    0xFFFFBFFF
#define XTENSA_INTLEVEL7_ANDBELOW_MASK    0xFFFFFFFF

/* Level of each interrupt: */

#define XTENSA_INT0_LEVEL                 1
#define XTENSA_INT1_LEVEL                 1
#define XTENSA_INT2_LEVEL                 1
#define XTENSA_INT3_LEVEL                 1
#define XTENSA_INT4_LEVEL                 1
#define XTENSA_INT5_LEVEL                 1
#define XTENSA_INT6_LEVEL                 1
#define XTENSA_INT7_LEVEL                 1
#define XTENSA_INT8_LEVEL                 1
#define XTENSA_INT9_LEVEL                 1
#define XTENSA_INT10_LEVEL                1
#define XTENSA_INT11_LEVEL                3
#define XTENSA_INT12_LEVEL                1
#define XTENSA_INT13_LEVEL                1
#define XTENSA_INT14_LEVEL                7
#define XTENSA_INT15_LEVEL                3
#define XTENSA_INT16_LEVEL                5
#define XTENSA_INT17_LEVEL                1
#define XTENSA_INT18_LEVEL                1
#define XTENSA_INT19_LEVEL                2
#define XTENSA_INT20_LEVEL                2
#define XTENSA_INT21_LEVEL                2
#define XTENSA_INT22_LEVEL                3
#define XTENSA_INT23_LEVEL                3
#define XTENSA_INT24_LEVEL                4
#define XTENSA_INT25_LEVEL                4
#define XTENSA_INT26_LEVEL                5
#define XTENSA_INT27_LEVEL                3
#define XTENSA_INT28_LEVEL                4
#define XTENSA_INT29_LEVEL                3
#define XTENSA_INT30_LEVEL                4
#define XTENSA_INT31_LEVEL                5
#define XTENSA_DEBUGLEVEL                 6    /* debug interrupt level */
#define XTENSA_HAVE_DEBUG_EXTERN_INT      1    /* OCD external db interrupt */
#define XTENSA_NMILEVEL                   7    /* NMI "level" (for use with
                                                * EXCSAVE/EPS/EPC_n, RFI n) */

/* Type of each interrupt: */

#define XTENSA_INT0_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT1_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT2_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT3_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT4_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT5_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT6_TYPE                  XTHAL_INTTYPE_TIMER
#define XTENSA_INT7_TYPE                  XTHAL_INTTYPE_SOFTWARE
#define XTENSA_INT8_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT9_TYPE                  XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT10_TYPE                 XTHAL_INTTYPE_EXTERN_EDGE
#define XTENSA_INT11_TYPE                 XTHAL_INTTYPE_PROFILING
#define XTENSA_INT12_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT13_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT14_TYPE                 XTHAL_INTTYPE_NMI
#define XTENSA_INT15_TYPE                 XTHAL_INTTYPE_TIMER
#define XTENSA_INT16_TYPE                 XTHAL_INTTYPE_TIMER
#define XTENSA_INT17_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT18_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT19_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT20_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT21_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT22_TYPE                 XTHAL_INTTYPE_EXTERN_EDGE
#define XTENSA_INT23_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT24_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT25_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT26_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT27_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL
#define XTENSA_INT28_TYPE                 XTHAL_INTTYPE_EXTERN_EDGE
#define XTENSA_INT29_TYPE                 XTHAL_INTTYPE_SOFTWARE
#define XTENSA_INT30_TYPE                 XTHAL_INTTYPE_EXTERN_EDGE
#define XTENSA_INT31_TYPE                 XTHAL_INTTYPE_EXTERN_LEVEL

/* Masks of interrupts for each type of interrupt: */

#define XTENSA_INTTYPE_MASK_UNCONFIGURED  0x00000000
#define XTENSA_INTTYPE_MASK_SOFTWARE      0x20000080
#define XTENSA_INTTYPE_MASK_EXTERN_EDGE   0x50400400
#define XTENSA_INTTYPE_MASK_EXTERN_LEVEL  0x8FBE333F
#define XTENSA_INTTYPE_MASK_TIMER         0x00018040
#define XTENSA_INTTYPE_MASK_NMI           0x00004000
#define XTENSA_INTTYPE_MASK_WRITE_ERROR   0x00000000
#define XTENSA_INTTYPE_MASK_PROFILING     0x00000800

/* Interrupt numbers assigned to specific interrupt sources: */

#define XTENSA_TIMER0_INTERRUPT           6     /* CCOMPARE0 */
#define XTENSA_TIMER1_INTERRUPT           15    /* CCOMPARE1 */
#define XTENSA_TIMER2_INTERRUPT           16    /* CCOMPARE2 */
#define XTENSA_TIMER3_INTERRUPT           XTHAL_TIMER_UNCONFIGURED
#define XTENSA_NMI_INTERRUPT              14    /* non-maskable interrupt */
#define XTENSA_PROFILING_INTERRUPT        11    /* profiling interrupt */

/* Interrupt numbers for levels at which only one interrupt is configured: */

#define XTENSA_INTLEVEL7_NUM              14

/* (There are many interrupts each at level(s) 1, 2, 3, 4, 5.) */

/* External interrupt mapping.
 *
 * These macros describe how Xtensa processor interrupt numbers
 * (as numbered internally, eg. in INTERRUPT and INTENABLE registers)
 * map to external BInterrupt<n> pins, for those interrupts
 * configured as external (level-triggered, edge-triggered, or NMI).
 * See the Xtensa processor databook for more details.
 */

/* Core interrupt numbers mapped to each EXTERNAL BInterrupt pin number: */

#define XTENSA_EXTINT0_NUM              0     /* (intlevel 1) */
#define XTENSA_EXTINT1_NUM              1     /* (intlevel 1) */
#define XTENSA_EXTINT2_NUM              2     /* (intlevel 1) */
#define XTENSA_EXTINT3_NUM              3     /* (intlevel 1) */
#define XTENSA_EXTINT4_NUM              4     /* (intlevel 1) */
#define XTENSA_EXTINT5_NUM              5     /* (intlevel 1) */
#define XTENSA_EXTINT6_NUM              8     /* (intlevel 1) */
#define XTENSA_EXTINT7_NUM              9     /* (intlevel 1) */
#define XTENSA_EXTINT8_NUM              10    /* (intlevel 1) */
#define XTENSA_EXTINT9_NUM              12    /* (intlevel 1) */
#define XTENSA_EXTINT10_NUM             13    /* (intlevel 1) */
#define XTENSA_EXTINT11_NUM             14    /* (intlevel 7) */
#define XTENSA_EXTINT12_NUM             17    /* (intlevel 1) */
#define XTENSA_EXTINT13_NUM             18    /* (intlevel 1) */
#define XTENSA_EXTINT14_NUM             19    /* (intlevel 2) */
#define XTENSA_EXTINT15_NUM             20    /* (intlevel 2) */
#define XTENSA_EXTINT16_NUM             21    /* (intlevel 2) */
#define XTENSA_EXTINT17_NUM             22    /* (intlevel 3) */
#define XTENSA_EXTINT18_NUM             23    /* (intlevel 3) */
#define XTENSA_EXTINT19_NUM             24    /* (intlevel 4) */
#define XTENSA_EXTINT20_NUM             25    /* (intlevel 4) */
#define XTENSA_EXTINT21_NUM             26    /* (intlevel 5) */
#define XTENSA_EXTINT22_NUM             27    /* (intlevel 3) */
#define XTENSA_EXTINT23_NUM             28    /* (intlevel 4) */
#define XTENSA_EXTINT24_NUM             30    /* (intlevel 4) */
#define XTENSA_EXTINT25_NUM             31    /* (intlevel 5) */
/* EXTERNAL BInterrupt pin numbers mapped to each core interrupt number: */
#define XTENSA_INT0_EXTNUM              0     /* (intlevel 1) */
#define XTENSA_INT1_EXTNUM              1     /* (intlevel 1) */
#define XTENSA_INT2_EXTNUM              2     /* (intlevel 1) */
#define XTENSA_INT3_EXTNUM              3     /* (intlevel 1) */
#define XTENSA_INT4_EXTNUM              4     /* (intlevel 1) */
#define XTENSA_INT5_EXTNUM              5     /* (intlevel 1) */
#define XTENSA_INT8_EXTNUM              6     /* (intlevel 1) */
#define XTENSA_INT9_EXTNUM              7     /* (intlevel 1) */
#define XTENSA_INT10_EXTNUM             8     /* (intlevel 1) */
#define XTENSA_INT12_EXTNUM             9     /* (intlevel 1) */
#define XTENSA_INT13_EXTNUM             10    /* (intlevel 1) */
#define XTENSA_INT14_EXTNUM             11    /* (intlevel 7) */
#define XTENSA_INT17_EXTNUM             12    /* (intlevel 1) */
#define XTENSA_INT18_EXTNUM             13    /* (intlevel 1) */
#define XTENSA_INT19_EXTNUM             14    /* (intlevel 2) */
#define XTENSA_INT20_EXTNUM             15    /* (intlevel 2) */
#define XTENSA_INT21_EXTNUM             16    /* (intlevel 2) */
#define XTENSA_INT22_EXTNUM             17    /* (intlevel 3) */
#define XTENSA_INT23_EXTNUM             18    /* (intlevel 3) */
#define XTENSA_INT24_EXTNUM             19    /* (intlevel 4) */
#define XTENSA_INT25_EXTNUM             20    /* (intlevel 4) */
#define XTENSA_INT26_EXTNUM             21    /* (intlevel 5) */
#define XTENSA_INT27_EXTNUM             22    /* (intlevel 3) */
#define XTENSA_INT28_EXTNUM             23    /* (intlevel 4) */
#define XTENSA_INT30_EXTNUM             24    /* (intlevel 4) */
#define XTENSA_INT31_EXTNUM             25    /* (intlevel 5) */

/* Exceptions and Vectors ***************************************************/

#define XTENSA_XEA_VERSION              2    /* Xtensa Exception Architecture
                                              * number: 1 == XEA1 (old)
                                              * 2 == XEA2 (new)
                                              * 0 == XEAX (extern) or TX */
#define XTENSA_HAVE_XEA1                0    /* Exception Architecture 1 */
#define XTENSA_HAVE_XEA2                1    /* Exception Architecture 2 */
#define XTENSA_HAVE_XEAX                0    /* External Exception Arch. */
#define XTENSA_HAVE_EXCEPTIONS          1    /* exception option */
#define XTENSA_HAVE_HALT                0    /* halt architecture option */
#define XTENSA_HAVE_BOOTLOADER          0    /* boot loader (for TX) */
#define XTENSA_HAVE_MEM_ECC_PARITY      0    /* local memory ECC/parity */
#define XTENSA_HAVE_VECTOR_SELECT       1    /* relocatable vectors */
#define XTENSA_HAVE_VECBASE             1    /* relocatable vectors */
#define XTENSA_VECBASE_RESET_VADDR      0x40000000  /* VECBASE reset value */
#define XTENSA_VECBASE_RESET_PADDR      0x40000000
#define XTENSA_RESET_VECBASE_OVERLAP    0

#define XTENSA_RESET_VECTOR0_VADDR      0x50000000
#define XTENSA_RESET_VECTOR0_PADDR      0x50000000
#define XTENSA_RESET_VECTOR1_VADDR      0x40000400
#define XTENSA_RESET_VECTOR1_PADDR      0x40000400
#define XTENSA_RESET_VECTOR_VADDR       0x40000400
#define XTENSA_RESET_VECTOR_PADDR       0x40000400
#define XTENSA_USER_VECOFS              0x00000340
#define XTENSA_USER_VECTOR_VADDR        0x40000340
#define XTENSA_USER_VECTOR_PADDR        0x40000340
#define XTENSA_KERNEL_VECOFS            0x00000300
#define XTENSA_KERNEL_VECTOR_VADDR      0x40000300
#define XTENSA_KERNEL_VECTOR_PADDR      0x40000300
#define XTENSA_DOUBLEEXC_VECOFS         0x000003C0
#define XTENSA_DOUBLEEXC_VECTOR_VADDR   0x400003C0
#define XTENSA_DOUBLEEXC_VECTOR_PADDR   0x400003C0
#define XTENSA_WINDOW_OF4_VECOFS        0x00000000
#define XTENSA_WINDOW_UF4_VECOFS        0x00000040
#define XTENSA_WINDOW_OF8_VECOFS        0x00000080
#define XTENSA_WINDOW_UF8_VECOFS        0x000000C0
#define XTENSA_WINDOW_OF12_VECOFS       0x00000100
#define XTENSA_WINDOW_UF12_VECOFS       0x00000140
#define XTENSA_WINDOW_VECTORS_VADDR     0x40000000
#define XTENSA_WINDOW_VECTORS_PADDR     0x40000000
#define XTENSA_INTLEVEL2_VECOFS         0x00000180
#define XTENSA_INTLEVEL2_VECTOR_VADDR   0x40000180
#define XTENSA_INTLEVEL2_VECTOR_PADDR   0x40000180
#define XTENSA_INTLEVEL3_VECOFS         0x000001C0
#define XTENSA_INTLEVEL3_VECTOR_VADDR   0x400001C0
#define XTENSA_INTLEVEL3_VECTOR_PADDR   0x400001C0
#define XTENSA_INTLEVEL4_VECOFS         0x00000200
#define XTENSA_INTLEVEL4_VECTOR_VADDR   0x40000200
#define XTENSA_INTLEVEL4_VECTOR_PADDR   0x40000200
#define XTENSA_INTLEVEL5_VECOFS         0x00000240
#define XTENSA_INTLEVEL5_VECTOR_VADDR   0x40000240
#define XTENSA_INTLEVEL5_VECTOR_PADDR   0x40000240
#define XTENSA_INTLEVEL6_VECOFS         0x00000280
#define XTENSA_INTLEVEL6_VECTOR_VADDR   0x40000280
#define XTENSA_INTLEVEL6_VECTOR_PADDR   0x40000280
#define XTENSA_DEBUG_VECOFS             XTENSA_INTLEVEL6_VECOFS
#define XTENSA_DEBUG_VECTOR_VADDR       XTENSA_INTLEVEL6_VECTOR_VADDR
#define XTENSA_DEBUG_VECTOR_PADDR       XTENSA_INTLEVEL6_VECTOR_PADDR
#define XTENSA_NMI_VECOFS               0x000002C0
#define XTENSA_NMI_VECTOR_VADDR         0x400002C0
#define XTENSA_NMI_VECTOR_PADDR         0x400002C0
#define XTENSA_INTLEVEL7_VECOFS         XTENSA_NMI_VECOFS
#define XTENSA_INTLEVEL7_VECTOR_VADDR   XTENSA_NMI_VECTOR_VADDR
#define XTENSA_INTLEVEL7_VECTOR_PADDR   XTENSA_NMI_VECTOR_PADDR

/* Debug Module *************************************************************/

/* Misc */

#define XTENSA_HAVE_DEBUG_ERI              1    /* ERI to debug module */
#define XTENSA_HAVE_DEBUG_APB              1    /* APB to debug module */
#define XTENSA_HAVE_DEBUG_JTAG             1    /* JTAG to debug module */

/* On-Chip Debug (OCD) */

#define XTENSA_HAVE_OCD                    1    /* OnChipDebug option */
#define XTENSA_NUM_IBREAK                  2    /* number of IBREAKn regs */
#define XTENSA_NUM_DBREAK                  2    /* number of DBREAKn regs */
#define XTENSA_HAVE_OCD_DIR_ARRAY          0    /* faster OCD option (to LX4) */
#define XTENSA_HAVE_OCD_LS32DDR            1    /* L32DDR/S32DDR (faster OCD) */

/* TRAX (in core) */

#define XTENSA_HAVE_TRAX                   1    /* TRAX in debug module */
#define XTENSA_TRAX_MEM_SIZE               16384 /* TRAX memory size in bytes */
#define XTENSA_TRAX_MEM_SHAREABLE          1    /* start/end regs; ready sig. */
#define XTENSA_TRAX_ATB_WIDTH              32    /* ATB width (bits), 0=no ATB */
#define XTENSA_TRAX_TIME_WIDTH             0    /* timestamp bitwidth, 0=none */

/* Perf counters */

#define XTENSA_NUM_PERF_COUNTERS           2    /* performance counters */

/* MMU **********************************************************************/

#define XTENSA_HAVE_TLBS                   1    /* inverse of HAVE_CACHEATTR */
#define XTENSA_HAVE_SPANNING_WAY           1    /* one way maps I+D 4GB vaddr */
#define XTENSA_SPANNING_WAY                0    /* TLB spanning way number */
#define XTENSA_HAVE_IDENTITY_MAP           1    /* vaddr == paddr always */
#define XTENSA_HAVE_CACHEATTR              0    /* CACHEATTR register present */
#define XTENSA_HAVE_MIMIC_CACHEATTR        1    /* region protection */
#define XTENSA_HAVE_XLT_CACHEATTR          0    /* region prot. w/translation */
#define XTENSA_HAVE_PTP_MMU                0    /* full MMU (with page table
                                                 * [autorefill] and protection)
                                                 * usable for an MMU-based OS */
/* If none of the above last 4 are set, it's a custom TLB configuration. */

#define XTENSA_MMU_ASID_BITS              0    /* number of bits in ASIDs */
#define XTENSA_MMU_RINGS                  1    /* number of rings (1..4) */
#define XTENSA_MMU_RING_BITS              0    /* num of bits in RING field */

#endif /* !XTENSA_HAL_NON_PRIVILEGED_ONLY */

#endif /* __ARCH_XTENSA_INCLUDE_ESP32_CORE_ISA_H */
