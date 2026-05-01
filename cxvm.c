#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>

#include "include/cxis.h"
#include "include/cxe.h"

/* ════════════════════════════════════════════════════════════════
   CPU STATE
════════════════════════════════════════════════════════════════ */

typedef struct {
    /* register file — all 206 registers */
    int32_t  i[32];    /* i0–i31  integer 32-bit   */
    int64_t  l[32];    /* l0–l31  integer 64-bit   */
    float    f[32];    /* f0–f31  float 32-bit     */
    double   d[32];    /* d0–d31  double 64-bit    */
    int32_t  c[32];    /* c0–c31  condition        */
    int32_t  s[32];    /* s0–s31  scratch           */
    /* argument registers — 32-bit default, 64-bit capable */
    uint64_t a[10];    /* a0–a9   argument/return  */
    /* special */
    uint32_t sp;       /* stack pointer            */
    uint32_t sf;       /* stack frame              */
    uint32_t bp;       /* base pointer             */
    uint32_t bf;       /* base frame               */

    uint32_t pc;       /* program counter          */
    uint8_t  carry;    /* carry flag               */
    uint8_t  intf;     /* interrupt enable flag    */
    uint8_t  halted;
    uint8_t  running;

    uint8_t *ram;      /* 64MB RAM                 */
    uint32_t ram_size;

    uint32_t heap_ptr; /* bump allocator for BIOS malloc */
} CPU;

/* ════════════════════════════════════════════════════════════════
   MEMORY HELPERS
════════════════════════════════════════════════════════════════ */

static void mem_check(CPU *cpu, uint32_t addr, uint32_t size, const char *op) {
    if (addr + size > cpu->ram_size) {
        fprintf(stderr,"vm: %s fault at 0x%08X (size %u)\n", op, addr, size);
        cpu->running = 0;
    }
}

static uint8_t  mem_r8 (CPU *cpu, uint32_t a) { mem_check(cpu,a,1,"read");  return cpu->ram[a]; }
static uint16_t mem_r16(CPU *cpu, uint32_t a) { uint16_t v; mem_check(cpu,a,2,"read");  memcpy(&v,cpu->ram+a,2); return v; }
static uint32_t mem_r32(CPU *cpu, uint32_t a) { uint32_t v; mem_check(cpu,a,4,"read");  memcpy(&v,cpu->ram+a,4); return v; }
static uint64_t mem_r64(CPU *cpu, uint32_t a) { uint64_t v; mem_check(cpu,a,8,"read");  memcpy(&v,cpu->ram+a,8); return v; }
static float    mem_rf (CPU *cpu, uint32_t a) { float    v; mem_check(cpu,a,4,"read");  memcpy(&v,cpu->ram+a,4); return v; }
static double   mem_rd (CPU *cpu, uint32_t a) { double   v; mem_check(cpu,a,8,"read");  memcpy(&v,cpu->ram+a,8); return v; }

static void mem_w8 (CPU *cpu, uint32_t a, uint8_t  v) { mem_check(cpu,a,1,"write"); cpu->ram[a]=v; }
static void mem_w32(CPU *cpu, uint32_t a, uint32_t v) { mem_check(cpu,a,4,"write"); memcpy(cpu->ram+a,&v,4); }
static void mem_w64(CPU *cpu, uint32_t a, uint64_t v) { mem_check(cpu,a,8,"write"); memcpy(cpu->ram+a,&v,8); }
static void mem_wf (CPU *cpu, uint32_t a, float    v) { mem_check(cpu,a,4,"write"); memcpy(cpu->ram+a,&v,4); }
static void mem_wd (CPU *cpu, uint32_t a, double   v) { mem_check(cpu,a,8,"write"); memcpy(cpu->ram+a,&v,8); }

/* fetch helpers */
static uint8_t  fetch8 (CPU *cpu) { uint8_t  v=mem_r8 (cpu,cpu->pc); cpu->pc+=1; return v; }
static uint16_t fetch16(CPU *cpu) { uint16_t v=mem_r16(cpu,cpu->pc); cpu->pc+=2; return v; }
static uint32_t fetch32(CPU *cpu) { uint32_t v=mem_r32(cpu,cpu->pc); cpu->pc+=4; return v; }
static uint64_t fetch64(CPU *cpu) { uint64_t v=mem_r64(cpu,cpu->pc); cpu->pc+=8; return v; }
static float    fetchf (CPU *cpu) { float    v=mem_rf (cpu,cpu->pc); cpu->pc+=4; return v; }
static double   fetchd (CPU *cpu) { double   v=mem_rd (cpu,cpu->pc); cpu->pc+=8; return v; }

/* stack */
static void push32(CPU *cpu, uint32_t v) { cpu->sp-=4; mem_w32(cpu,cpu->sp,v); }
static uint32_t pop32(CPU *cpu) { uint32_t v=mem_r32(cpu,cpu->sp); cpu->sp+=4; return v; }

/* ════════════════════════════════════════════════════════════════
   REGISTER ACCESS
════════════════════════════════════════════════════════════════ */

static int32_t  get_i(CPU *cpu, uint8_t id) { return cpu->i[id]; }
static int64_t  get_l(CPU *cpu, uint8_t id) { return cpu->l[id-32]; }
static float    get_f(CPU *cpu, uint8_t id) { return cpu->f[id-64]; }
static double   get_d(CPU *cpu, uint8_t id) { return cpu->d[id-96]; }
static int32_t  get_c(CPU *cpu, uint8_t id) { return cpu->c[id-128]; }
static int32_t  get_s(CPU *cpu, uint8_t id) { return cpu->s[id-160]; }
static uint64_t get_a(CPU *cpu, uint8_t id) { return cpu->a[id-192]; }

static void set_i(CPU *cpu, uint8_t id, int32_t  v) { cpu->i[id]=v; }
static void set_l(CPU *cpu, uint8_t id, int64_t  v) { cpu->l[id-32]=v; }
static void set_f(CPU *cpu, uint8_t id, float    v) { cpu->f[id-64]=v; }
static void set_d(CPU *cpu, uint8_t id, double   v) { cpu->d[id-96]=v; }
static void set_c(CPU *cpu, uint8_t id, int32_t  v) { cpu->c[id-128]=v; }
static void set_s(CPU *cpu, uint8_t id, int32_t  v) { cpu->s[id-160]=v; }
static void set_a(CPU *cpu, uint8_t id, uint64_t v) { cpu->a[id-192]=v; }

/* generic 32-bit int access for common ops */
static int32_t getreg32(CPU *cpu, uint8_t id) {
    if (id < 32)  return cpu->i[id];
    if (id < 64)  return (int32_t)cpu->l[id-32];
    if (id < 96)  return (int32_t)cpu->f[id-64];
    if (id < 128) return (int32_t)cpu->d[id-96];
    if (id < 160) return cpu->c[id-128];
    if (id < 192) return cpu->s[id-160];
    if (id < 202) return (int32_t)cpu->a[id-192];
    if (id==202)  return (int32_t)cpu->sp;
    if (id==203)  return (int32_t)cpu->sf;
    if (id==204)  return (int32_t)cpu->bp;
    if (id==205)  return (int32_t)cpu->bf;
    return 0;
}

static void setreg32(CPU *cpu, uint8_t id, int32_t v) {
    if (id < 32)  { cpu->i[id]=v; return; }
    if (id < 64)  { cpu->l[id-32]=(int64_t)v; return; }
    if (id < 96)  { cpu->f[id-64]=(float)v; return; }
    if (id < 128) { cpu->d[id-96]=(double)v; return; }
    if (id < 160) { cpu->c[id-128]=v; return; }
    if (id < 192) { cpu->s[id-160]=v; return; }
    if (id < 202) { cpu->a[id-192]=(uint64_t)(int64_t)v; return; }
    if (id==202)  { cpu->sp=(uint32_t)v; return; }
    if (id==203)  { cpu->sf=(uint32_t)v; return; }
    if (id==204)  { cpu->bp=(uint32_t)v; return; }
    if (id==205)  { cpu->bf=(uint32_t)v; return; }
}

/* ════════════════════════════════════════════════════════════════
   MEMORY OPERAND DECODE
════════════════════════════════════════════════════════════════ */

static uint32_t decode_mem_addr(CPU *cpu) {
    uint8_t mflags = fetch8(cpu);
    uint32_t addr  = 0;
    if (mflags & 0x01) { uint8_t base = fetch8(cpu); addr += (uint32_t)getreg32(cpu,base); }
    if (mflags & 0x02) {
        uint8_t idx = fetch8(cpu);
        uint8_t sc  = fetch8(cpu);
        addr += (uint32_t)getreg32(cpu,idx) * sc;
    }
    uint32_t disp = fetch32(cpu);
    addr += disp;
    return addr;
}

/* ════════════════════════════════════════════════════════════════
   BIOS INTERRUPT HANDLER
════════════════════════════════════════════════════════════════ */

static void bios_handle(CPU *cpu, uint8_t vector) {
    switch (vector) {
    case BIOS_PRINT_STR: {
        uint32_t ptr = (uint32_t)cpu->a[0];
        uint32_t len = (uint32_t)cpu->a[1];
        for (uint32_t i = 0; i < len && ptr+i < cpu->ram_size; i++)
            putchar(cpu->ram[ptr+i]);
        fflush(stdout);
        break;
    }
    case BIOS_PRINT_CHAR:
        putchar((int)(cpu->a[0] & 0xFF));
        fflush(stdout);
        break;
    case BIOS_READ_CHAR: {
        int ch = getchar();
        cpu->a[0] = (ch == EOF) ? 0xFFFFFFFF : (uint64_t)(unsigned char)ch;
        break;
    }
    case BIOS_EXIT:
        cpu->running = 0;
        cpu->halted  = 1;
        exit((int)cpu->a[0]);
        break;
    case BIOS_MEM_ALLOC: {
        uint32_t sz = (uint32_t)cpu->a[0];
        sz = (sz + 7) & ~7;  /* align 8 */
        if (cpu->heap_ptr + sz < VM_STACK_TOP - 0x10000) {
            cpu->a[0] = cpu->heap_ptr;
            cpu->heap_ptr += sz;
        } else {
            cpu->a[0] = 0; /* null — out of memory */
        }
        break;
    }
    case BIOS_MEM_FREE:
        /* bump allocator — free is a no-op */
        break;
    case BIOS_TIME: {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        cpu->l[0] = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
        break;
    }
    default:
        fprintf(stderr,"vm: unknown bios interrupt 0x%02X at pc=0x%08X\n", vector, cpu->pc);
        break;
    }
}

/* ════════════════════════════════════════════════════════════════
   INSTRUCTION DECODE & EXECUTE
════════════════════════════════════════════════════════════════ */

static void step(CPU *cpu) {
    uint8_t opcode = fetch8(cpu);
    uint8_t mod    = fetch8(cpu);

    int  nops    = (mod >> 6) & 0x3;
    int  has_imm = (mod >> 5) & 0x1;
    int  has_mem = (mod >> 4) & 0x1;
    int  has_ext = (mod >> 3) & 0x1;

    uint8_t dst_class=0, s1_class=0, s2_class=0;
    if (has_ext) {
        dst_class = fetch8(cpu);
        s1_class  = fetch8(cpu);
        s2_class  = fetch8(cpu);
    }

    /* pre-read all register ids and immediates upfront
       assembler emits: [ext bytes] [reg0] [reg1] ... [regN] [imm/label if any]
       so we read all regs first, then the imm */
    uint8_t  rv[4] = {0,0,0,0};   /* register operand ids  */
    int      nr    = 0;            /* number of reg operands */
    int64_t  imm   = 0;            /* immediate value        */
    uint32_t label_addr = 0;       /* resolved label address */

    /* count register operands = nops minus 1 if has_imm (imm replaces one slot) */
    int nregs = nops - (has_imm ? 1 : 0) - (has_mem ? 1 : 0);
    if (nregs < 0) nregs = 0;
    for (int _i = 0; _i < nregs && _i < 4; _i++) rv[nr++] = fetch8(cpu);
    if (has_imm) { imm = (int64_t)(int32_t)fetch32(cpu); label_addr = (uint32_t)(int32_t)imm; }

    /* convenience: src1, src2, dst register values */
    /* for 3-reg ops: rv[0]=src1 rv[1]=src2 rv[2]=dst
       for 2-reg+imm: rv[0]=src1  rv[1]=dst  imm=src2
       for 1-reg+imm: rv[0]=dst   imm=src    (mov imm, dst)
       for shifts:    rv[0]=src   rv[1]=dst  imm=n  */
#define RV0 rv[0]
#define RV1 rv[1]
#define RV2 rv[2]
#define GR(id) getreg32(cpu,id)
#define SR(id,v) setreg32(cpu,id,v)

    switch (opcode) {

    /* ── DATA MOVEMENT ── */
    case OP_NOP: break;
    case OP_HALT: cpu->halted=1; cpu->running=0; break;

    case OP_MOV: {
        if (has_imm && !has_mem) {
            /* mov imm, dst  — rv[0] = dst reg */
            SR(RV0, (int32_t)imm);
        } else if (has_mem) {
            uint32_t addr = decode_mem_addr(cpu);
            if (mod & 0x04) mem_w32(cpu, addr, (uint32_t)GR(RV0)); /* store */
            else            SR(RV0, (int32_t)mem_r32(cpu, addr));   /* load  */
        } else {
            /* mov src, dst */
            SR(RV1, GR(RV0));
        }
        break;
    }
    case OP_FMOV: {
        if (has_imm) set_f(cpu, RV0, (float)*(float*)&imm);
        else         set_f(cpu, RV1, get_f(cpu, RV0));
        break;
    }
    case OP_DMOV: {
        if (has_imm) set_d(cpu, RV0, (double)imm);
        else         set_d(cpu, RV1, get_d(cpu, RV0));
        break;
    }
    case OP_LMOV: {
        if (has_imm) set_l(cpu, RV0, imm);
        else         set_l(cpu, RV1, get_l(cpu, RV0));
        break;
    }
    case OP_MOVSX:  { SR(RV1, (int32_t)(int16_t)(GR(RV0)&0xFFFF)); break; }
    case OP_MOVZX:  { SR(RV1, (int32_t)(uint16_t)(uint32_t)GR(RV0)); break; }
    case OP_MOVSXD: { set_l(cpu,RV1,(int64_t)(int32_t)GR(RV0)); break; }
    case OP_MOVB: {
        /* byte move: movb [mem], dst  or  movb src, [mem] */
        if (has_mem) {
            uint32_t addr = decode_mem_addr(cpu);
            if (mod & 0x04) mem_w8(cpu, addr, (uint8_t)GR(RV0));  /* store */
            else            SR(RV0, (int32_t)(uint32_t)mem_r8(cpu, addr)); /* load */
        } else {
            SR(RV1, GR(RV0) & 0xFF);
        }
        break;
    }
    case OP_MOVW: {
        /* word (16-bit) move */
        if (has_mem) {
            uint32_t addr = decode_mem_addr(cpu);
            if (mod & 0x04) { uint16_t v=(uint16_t)GR(RV0); memcpy(cpu->ram+addr,&v,2); }
            else            { uint16_t v; memcpy(&v,cpu->ram+addr,2); SR(RV0,(int32_t)(uint32_t)v); }
        } else {
            SR(RV1, GR(RV0) & 0xFFFF);
        }
        break;
    }
    case OP_LEA: {
        uint32_t addr = decode_mem_addr(cpu);
        uint8_t  dst  = fetch8(cpu);
        SR(dst, (int32_t)addr); break;
    }
    case OP_CMOV:  { if(get_c(cpu,RV0)) SR(RV2, GR(RV1)); break; }
    case OP_PUSH:  { push32(cpu,(uint32_t)(has_imm?(int32_t)imm:GR(RV0))); break; }
    case OP_POP:   { SR(RV0,(int32_t)pop32(cpu)); break; }
    case OP_PUSHA: { for(int _i=31;_i>=0;_i--) push32(cpu,(uint32_t)cpu->i[_i]); break; }
    case OP_POPA:  { for(int _i=0;_i<32;_i++) cpu->i[_i]=(int32_t)pop32(cpu); break; }

    /* ── ARITHMETIC i32 ── */
    case OP_ADD: {
        int32_t av = GR(RV0);
        int32_t bv = has_imm ? (int32_t)imm : GR(RV1);
        uint8_t  d = has_imm ? RV1 : RV2;
        int64_t  r = (int64_t)av + (int64_t)bv;
        cpu->carry = (r>(int64_t)0x7FFFFFFF||r<(int64_t)(int32_t)0x80000000)?1:0;
        SR(d,(int32_t)r); break; }
    case OP_ADDC: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        int64_t r=(int64_t)av+(int64_t)bv+cpu->carry;
        cpu->carry=(uint32_t)(r>>32)?1:0; SR(d,(int32_t)r); break; }
    case OP_SUB: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        int64_t r=(int64_t)av-(int64_t)bv;
        cpu->carry=(r<0)?1:0; SR(d,(int32_t)r); break; }
    case OP_SUBB: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        int64_t r=(int64_t)av-(int64_t)bv-cpu->carry;
        cpu->carry=(r<0)?1:0; SR(d,(int32_t)r); break; }
    case OP_MUL: {
        uint32_t av=(uint32_t)GR(RV0); uint32_t bv=has_imm?(uint32_t)imm:(uint32_t)GR(RV1);
        uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)(av*bv)); break; }
    case OP_IMUL: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2; SR(d,av*bv); break; }
    case OP_DIV: {
        uint32_t av=(uint32_t)GR(RV0); uint32_t bv=has_imm?(uint32_t)imm:(uint32_t)GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        if(!bv){fprintf(stderr,"vm: div/0\n");cpu->running=0;break;}
        SR(d,(int32_t)(av/bv)); break; }
    case OP_IDIV: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        if(!bv){fprintf(stderr,"vm: div/0\n");cpu->running=0;break;}
        SR(d,av/bv); break; }
    case OP_INC: { SR(RV0,GR(RV0)+1); break; }
    case OP_DEC: { SR(RV0,GR(RV0)-1); break; }
    case OP_NEG: { SR(RV1,-GR(RV0)); break; }

    /* ── ARITHMETIC i64 ── */
    case OP_LNEG: { set_l(cpu,RV1,-get_l(cpu,RV0)); break; }
    case OP_LADD: { set_l(cpu,RV2,get_l(cpu,RV0)+get_l(cpu,RV1)); break; }
    case OP_LSUB: { set_l(cpu,RV2,get_l(cpu,RV0)-get_l(cpu,RV1)); break; }
    case OP_LMUL: { set_l(cpu,RV2,get_l(cpu,RV0)*get_l(cpu,RV1)); break; }
    case OP_LDIV: {
        if(!get_l(cpu,RV1)){fprintf(stderr,"vm: div/0\n");cpu->running=0;break;}
        set_l(cpu,RV2,get_l(cpu,RV0)/get_l(cpu,RV1)); break; }

    /* ── ARITHMETIC float ── */
    case OP_FADD: { set_f(cpu,RV2,get_f(cpu,RV0)+get_f(cpu,RV1)); break; }
    case OP_FSUB: { set_f(cpu,RV2,get_f(cpu,RV0)-get_f(cpu,RV1)); break; }
    case OP_FMUL: { set_f(cpu,RV2,get_f(cpu,RV0)*get_f(cpu,RV1)); break; }
    case OP_FDIV: { set_f(cpu,RV2,get_f(cpu,RV0)/get_f(cpu,RV1)); break; }
    case OP_FNEG: { set_f(cpu,RV1,-get_f(cpu,RV0)); break; }

    /* ── ARITHMETIC double ── */
    case OP_DADD: { set_d(cpu,RV2,get_d(cpu,RV0)+get_d(cpu,RV1)); break; }
    case OP_DSUB: { set_d(cpu,RV2,get_d(cpu,RV0)-get_d(cpu,RV1)); break; }
    case OP_DMUL: { set_d(cpu,RV2,get_d(cpu,RV0)*get_d(cpu,RV1)); break; }
    case OP_DDIV: { set_d(cpu,RV2,get_d(cpu,RV0)/get_d(cpu,RV1)); break; }
    case OP_DNEG: { set_d(cpu,RV1,-get_d(cpu,RV0)); break; }

    /* ── BITWISE ── */
    case OP_AND: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a&b); break; }
    case OP_OR:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a|b); break; }
    case OP_XOR: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a^b); break; }
    case OP_NOT:    { SR(RV1,~GR(RV0)); break; }
    case OP_SHL: {
        int32_t sv=GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31);
        uint8_t d=has_imm?RV1:RV2; SR(d,sv<<n); break; }
    case OP_SHR: {
        uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31);
        uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)(sv>>n)); break; }
    case OP_SAR: {
        int32_t sv=GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31);
        uint8_t d=has_imm?RV1:RV2; SR(d,sv>>n); break; }
    case OP_ROL: {
        uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31);
        uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)((sv<<n)|(sv>>(32-n)))); break; }
    case OP_ROR: {
        uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31);
        uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)((sv>>n)|(sv<<(32-n)))); break; }
    case OP_BSF:    { uint32_t v=(uint32_t)GR(RV0); int idx=0; while(idx<32&&!((v>>idx)&1))idx++; SR(RV1,idx); break; }
    case OP_BSR:    { uint32_t v=(uint32_t)GR(RV0); int idx=31; while(idx>=0&&!((v>>idx)&1))idx--; SR(RV1,idx); break; }
    case OP_POPCNT: { uint32_t v=(uint32_t)GR(RV0); int c=0; while(v){c+=v&1;v>>=1;} SR(RV1,c); break; }
    case OP_LZCNT:  { uint32_t v=(uint32_t)GR(RV0); int c=0; for(int _i=31;_i>=0;_i--){if(!((v>>_i)&1))c++;else break;} SR(RV1,c); break; }
    case OP_TZCNT:  { uint32_t v=(uint32_t)GR(RV0); int c=0; while(c<32&&!((v>>c)&1))c++; SR(RV1,c); break; }
    case OP_TEST:   { set_c(cpu,RV2,(GR(RV0)&GR(RV1))?1:0); break; }
    case OP_XCHG:   { int32_t t=GR(RV0); SR(RV0,GR(RV1)); SR(RV1,t); break; }
    case OP_BT:     { set_c(cpu,RV2,((uint32_t)GR(RV0)>>(imm&31))&1); break; }
    case OP_BTS:    { SR(RV2,GR(RV0)|(1<<(imm&31))); break; }
    case OP_BTR:    { SR(RV2,GR(RV0)&~(1<<(imm&31))); break; }
    case OP_BTC:    { SR(RV2,GR(RV0)^(1<<(imm&31))); break; }

    /* ── COMPARE ── */
    case OP_CMP: {
        int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1);
        uint8_t d=has_imm?RV1:RV2;
        set_c(cpu,d,(av<bv)?-1:(av>bv)?1:0); break; }
    case OP_LCMP: { int64_t av=get_l(cpu,RV0),bv=get_l(cpu,RV1); set_c(cpu,RV2,(av<bv)?-1:(av>bv)?1:0); break; }
    case OP_FCMP: { float  av=get_f(cpu,RV0),bv=get_f(cpu,RV1); set_c(cpu,RV2,(av<bv)?-1:(av>bv)?1:0); break; }
    case OP_DCMP: { double av=get_d(cpu,RV0),bv=get_d(cpu,RV1); set_c(cpu,RV2,(av<bv)?-1:(av>bv)?1:0); break; }
    case OP_EQ:  { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av==bv?1:0); break; }
    case OP_NE:  { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av!=bv?1:0); break; }
    case OP_GT:  { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av>bv?1:0);  break; }
    case OP_LT:  { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av<bv?1:0);  break; }
    case OP_GTE: { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av>=bv?1:0); break; }
    case OP_LTE: { int32_t av=GR(RV0); int32_t bv=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(cpu,d,av<=bv?1:0); break; }

    /* ── CONTROL FLOW ── */
    case OP_JMP:
    case OP_GOTO: { cpu->pc = label_addr; break; }
    case OP_JCC:
    case OP_JE:   { if ( get_c(cpu,RV0))      cpu->pc=label_addr; break; }
    case OP_JNE:  { if (!get_c(cpu,RV0))      cpu->pc=label_addr; break; }
    case OP_JG:   { if ( get_c(cpu,RV0)>0)    cpu->pc=label_addr; break; }
    case OP_JGE:  { if ( get_c(cpu,RV0)>=0)   cpu->pc=label_addr; break; }
    case OP_JL:   { if ( get_c(cpu,RV0)<0)    cpu->pc=label_addr; break; }
    case OP_JLE:  { if ( get_c(cpu,RV0)<=0)   cpu->pc=label_addr; break; }
    case OP_JA:   { if ((uint32_t)get_c(cpu,RV0)>0)  cpu->pc=label_addr; break; }
    case OP_JB:   { if ((uint32_t)get_c(cpu,RV0)==0)  cpu->pc=label_addr; break; }
    case OP_LOOP: { cpu->i[0]--; if (cpu->i[0]!=0) cpu->pc=label_addr; break; }
    case OP_CALL: { push32(cpu,cpu->pc); cpu->pc=label_addr; break; }
    case OP_RET:  { cpu->pc=pop32(cpu); break; }
    case OP_RETN: { cpu->pc=pop32(cpu); cpu->sp+=(uint32_t)imm; break; }
    case OP_EXIT: { exit((int)imm); break; }

    /* ── SYSTEM ── */
    case OP_INT:  { bios_handle(cpu,(uint8_t)imm); break; }
    case OP_IRET: { cpu->pc=pop32(cpu); cpu->intf=1; break; }
    case OP_CLI:  { cpu->intf=0; break; }
    case OP_STI:  { cpu->intf=1; break; }
    case OP_CPUID:{ cpu->a[0]=0x43584953; cpu->a[1]=0x00010001; cpu->a[2]=0; cpu->a[3]=0; break; }
    case OP_RDTSC:{ struct timespec _ts; clock_gettime(CLOCK_MONOTONIC,&_ts);
        cpu->l[0]=(int64_t)_ts.tv_sec*1000000000LL+_ts.tv_nsec; break; }
    case OP_WAIT: break;
    case OP_PAUSE:break;
    case OP_UD:
        fprintf(stderr,"vm: undefined instruction at pc=0x%08X\n", cpu->pc);
        cpu->running=0; break;

    /* ── TYPE CONVERSIONS ── */
    case OP_ITOF: { set_f(cpu,RV1,(float)get_i(cpu,RV0)); break; }
    case OP_ITOD: { set_d(cpu,RV1,(double)get_i(cpu,RV0)); break; }
    case OP_ITOL: { set_l(cpu,RV1,(int64_t)get_i(cpu,RV0)); break; }
    case OP_LTOF: { set_f(cpu,RV1,(float)get_l(cpu,RV0)); break; }
    case OP_LTOD: { set_d(cpu,RV1,(double)get_l(cpu,RV0)); break; }
    case OP_FTOI: { set_i(cpu,RV1,(int32_t)get_f(cpu,RV0)); break; }
    case OP_FTOD: { set_d(cpu,RV1,(double)get_f(cpu,RV0)); break; }
    case OP_FTOL: { set_l(cpu,RV1,(int64_t)get_f(cpu,RV0)); break; }
    case OP_DTOI: { set_i(cpu,RV1,(int32_t)get_d(cpu,RV0)); break; }
    case OP_DTOF: { set_f(cpu,RV1,(float)get_d(cpu,RV0)); break; }
    case OP_DTOL: { set_l(cpu,RV1,(int64_t)get_d(cpu,RV0)); break; }
    case OP_LTOI: { set_i(cpu,RV1,(int32_t)get_l(cpu,RV0)); break; }

    /* ── I/O ── */
    case OP_IN:  { SR(RV1,0); break; }
    case OP_OUT: { break; }

    default:
        fprintf(stderr,"vm: unknown opcode 0x%02X at pc=0x%08X\n", opcode, cpu->pc-2);
        cpu->running=0; break;
    }
}

/* ════════════════════════════════════════════════════════════════
   LOADER
════════════════════════════════════════════════════════════════ */

static int load_cxe(CPU *cpu, const char *path) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"vm: cannot open '%s': %s\n",path,strerror(errno)); return 0; }

    CxeHeader hdr;
    fread(&hdr,sizeof(hdr),1,f);
    if (hdr.magic != CXE_MAGIC) {
        fprintf(stderr,"vm: '%s' is not a valid .cxe file\n",path);
        fclose(f); return 0;
    }

    for (int i = 0; i < hdr.section_count; i++) {
        CxeSection sec;
        fread(&sec,sizeof(sec),1,f);
        long saved = ftell(f);
        if (sec.vaddr + sec.mem_size > cpu->ram_size) {
            fprintf(stderr,"vm: section %d exceeds RAM\n",i);
            fclose(f); return 0;
        }
        if (sec.flags & CXE_SEC_ZERO) {
            memset(cpu->ram + sec.vaddr, 0, sec.mem_size);
        } else {
            fseek(f, sec.offset, SEEK_SET);
            fread(cpu->ram + sec.vaddr, 1, sec.file_size, f);
            if (sec.mem_size > sec.file_size)
                memset(cpu->ram + sec.vaddr + sec.file_size, 0, sec.mem_size - sec.file_size);
        }
        fseek(f, saved, SEEK_SET);
    }

    cpu->pc      = hdr.entry_point;
    fclose(f); return 1;
}

/* ════════════════════════════════════════════════════════════════
   MAIN
════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,"usage: cxvm <file.cxe> [--trace]\n");
        return 1;
    }

    int trace = 0;
    for (int i = 2; i < argc; i++)
        if (strcmp(argv[i],"--trace")==0) trace=1;

    CPU *cpu = calloc(1,sizeof(CPU));
    cpu->ram      = calloc(1, VM_RAM_SIZE);
    cpu->ram_size = VM_RAM_SIZE;
    cpu->sp       = VM_STACK_TOP;
    cpu->sf       = VM_STACK_TOP;
    cpu->bp       = VM_TEXT_BASE;
    cpu->bf       = 0;
    cpu->intf     = 1;
    cpu->heap_ptr = VM_HEAP_BASE;

    if (!load_cxe(cpu, argv[1])) { free(cpu->ram); free(cpu); return 1; }

    cpu->running = 1;
    fprintf(stderr,"cxvm: starting at 0x%08X  ram=%dMB\n", cpu->pc, VM_RAM_SIZE/(1024*1024));

    while (cpu->running && !cpu->halted) {
        if (trace) fprintf(stderr,"  pc=0x%08X sp=0x%08X i0=%d i1=%d\n",
                           cpu->pc, cpu->sp, cpu->i[0], cpu->i[1]);
        step(cpu);
    }

    int exit_code = (int)cpu->a[0];
    free(cpu->ram); free(cpu);
    return exit_code;
}
