#ifndef CXIS_H
#define CXIS_H

#include <stdint.h>
#include <stddef.h>

/* ── version ─────────────────────────────────────────────────── */
#define CXIS_VERSION_MAJOR 1
#define CXIS_VERSION_MINOR 1

/* ── opcodes ─────────────────────────────────────────────────── */
typedef enum {
    /* data movement 0x01–0x0d */
    OP_MOV    = 0x01, OP_MOVSX  = 0x02, OP_MOVZX  = 0x03,
    OP_MOVSXD = 0x04, OP_PUSH   = 0x05, OP_POP    = 0x06,
    OP_PUSHA  = 0x07, OP_POPA   = 0x08, OP_LEA    = 0x09,
    OP_FMOV   = 0x0A, OP_DMOV   = 0x0B, OP_LMOV   = 0x0C,
    OP_CMOV   = 0x0D,

    OP_MOVB   = 0x0E, /* move byte (8-bit)  */
    OP_MOVW   = 0x0F, /* move word (16-bit) */
    OP_ADD    = 0x10, OP_ADDC   = 0x11, OP_SUB    = 0x12,
    OP_SUBB   = 0x13, OP_MUL    = 0x14, OP_IMUL   = 0x15,
    OP_DIV    = 0x16, OP_IDIV   = 0x17, OP_INC    = 0x18,
    OP_DEC    = 0x19, OP_NEG    = 0x1A, OP_LNEG   = 0x1B,
    OP_LADD   = 0x1C, OP_LSUB   = 0x1D, OP_LMUL   = 0x1E,
    OP_LDIV   = 0x1F, OP_FADD   = 0x20, OP_FSUB   = 0x21,
    OP_FMUL   = 0x22, OP_FDIV   = 0x23, OP_FNEG   = 0x24,
    OP_DADD   = 0x25, OP_DSUB   = 0x26, OP_DMUL   = 0x27,
    OP_DDIV   = 0x28, OP_DNEG   = 0x29,

    /* bitwise / shift 0x30–0x47 */
    OP_AND    = 0x30, OP_OR     = 0x31, OP_XOR    = 0x32,
    OP_NOT    = 0x33, OP_SHL    = 0x34, OP_SHR    = 0x35,
    OP_SAR    = 0x36, OP_ROL    = 0x37, OP_ROR    = 0x38,
    OP_RCL    = 0x39, OP_RCR    = 0x3A, OP_SHLD   = 0x3B,
    OP_SHRD   = 0x3C, OP_BSF    = 0x3D, OP_BSR    = 0x3E,
    OP_BT     = 0x3F, OP_BTS    = 0x40, OP_BTR    = 0x41,
    OP_BTC    = 0x42, OP_POPCNT = 0x43, OP_LZCNT  = 0x44,
    OP_TZCNT  = 0x45, OP_TEST   = 0x46, OP_XCHG   = 0x47,

    /* compare & set 0x50–0x59 */
    OP_CMP    = 0x50, OP_LCMP   = 0x51, OP_FCMP   = 0x52,
    OP_DCMP   = 0x53, OP_EQ     = 0x54, OP_NE     = 0x55,
    OP_GT     = 0x56, OP_LT     = 0x57, OP_GTE    = 0x58,
    OP_LTE    = 0x59,

    /* control flow 0x60–0x70 */
    OP_JMP    = 0x60, OP_GOTO   = 0x61, OP_JCC    = 0x62,
    OP_JE     = 0x63, OP_JNE    = 0x64, OP_JG     = 0x65,
    OP_JGE    = 0x66, OP_JL     = 0x67, OP_JLE    = 0x68,
    OP_JA     = 0x69, OP_JB     = 0x6A, OP_LOOP   = 0x6B,
    OP_CALL   = 0x6C, OP_RET    = 0x6D, OP_RETN   = 0x6E,
    OP_EXIT   = 0x6F, OP_HALT   = 0x70,

    /* string ops 0x78–0x7f */
    OP_MOVS   = 0x78, OP_STOS   = 0x79, OP_LODS   = 0x7A,
    OP_CMPS   = 0x7B, OP_SCAS   = 0x7C, OP_REP    = 0x7D,
    OP_REPE   = 0x7E, OP_REPNE  = 0x7F,

    /* misc / system 0x80–0x8f */
    OP_NOP    = 0x80, OP_WAIT   = 0x81, OP_IN     = 0x82,
    OP_OUT    = 0x83, OP_INT    = 0x84, OP_IRET   = 0x85,
    OP_CPUID  = 0x86, OP_RDTSC  = 0x87, OP_PAUSE  = 0x88,
    OP_UD     = 0x89, OP_CLI    = 0x8A, OP_STI    = 0x8B,

    /* type conversions 0x90–0x9b */
    OP_ITOF   = 0x90, OP_ITOD   = 0x91, OP_ITOL   = 0x92,
    OP_LTOF   = 0x93, OP_LTOD   = 0x94, OP_FTOI   = 0x95,
    OP_FTOD   = 0x96, OP_FTOL   = 0x97, OP_DTOI   = 0x98,
    OP_DTOF   = 0x99, OP_DTOL   = 0x9A, OP_LTOI   = 0x9B,
} Opcode;

/* ── register classes & ids ──────────────────────────────────── */
typedef enum {
    RC_I = 0,  /* int32   i0–i31  */
    RC_L = 1,  /* int64   l0–l31  */
    RC_F = 2,  /* float32 f0–f31  */
    RC_D = 3,  /* float64 d0–d31  */
    RC_C = 4,  /* cond    c0–c31  */
    RC_S = 5,  /* scratch s0–s31  */
    RC_A = 6,  /* arg     a0–a9   */
    RC_SP= 7,  /* special: sp sf bp bf */
} RegClass;

/* register id encoding: 0–31=i, 32–63=l, 64–95=f, 96–127=d,
   128–159=c, 160–191=s, 192–201=a, 202=sp, 203=sf, 204=bp, 205=bf */
#define REG_I(n)  (n)
#define REG_L(n)  (32 + (n))
#define REG_F(n)  (64 + (n))
#define REG_D(n)  (96 + (n))
#define REG_C(n)  (128 + (n))
#define REG_S(n)  (160 + (n))
#define REG_A(n)  (192 + (n))
#define REG_SP    202
#define REG_SF    203
#define REG_BP    204
#define REG_BF    205

#define REG_INVALID 0xFF

/* ── mod byte ────────────────────────────────────────────────── */
/* byte 0: opcode
   byte 1: mod byte  [7:6]=operand count [5]=imm [4]=mem [3]=ext [2:0]=rsvd
   byte 2: type byte (when ext=1) [8:6]=dst [5:3]=src1 [2:0]=src2 class
   byte 3+: register ids (1 byte each) then immediates */

#define MOD_NOPERANDS(n) (((n) & 0x3) << 6)
#define MOD_IMM          (1 << 5)
#define MOD_MEM          (1 << 4)
#define MOD_EXT          (1 << 3)

#define TYPE_BYTE(dst, s1, s2) (((dst)&7)<<6 | ((s1)&7)<<3 | ((s2)&7))

/* ── bios interrupt numbers ──────────────────────────────────── */
#define BIOS_PRINT_STR  0x01  /* a0=ptr a1=len              */
#define BIOS_PRINT_CHAR 0x02  /* a0=char                    */
#define BIOS_READ_CHAR  0x03  /* → a0=char                  */
#define BIOS_EXIT       0x04  /* a0=exit code               */
#define BIOS_MEM_ALLOC  0x05  /* a0=size → a0=ptr           */
#define BIOS_MEM_FREE   0x06  /* a0=ptr                     */
#define BIOS_TIME       0x07  /* → l0=nanoseconds           */

/* ── vm memory layout (64 MB) ────────────────────────────────── */
#define VM_RAM_SIZE     (64 * 1024 * 1024)
#define VM_TEXT_BASE    0x00001000   /* code starts here       */
#define VM_STACK_TOP    0x03F00000   /* stack top              */
#define VM_HEAP_BASE    0x02000000   /* heap base              */

#endif /* CXIS_H */
