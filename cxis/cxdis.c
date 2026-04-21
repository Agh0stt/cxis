#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "include/cxis.h"
#include "include/cxe.h"
#include "include/cxo.h"

/* ════════════════════════════════════════════════════════════════
   REGISTER NAMES
════════════════════════════════════════════════════════════════ */

static const char *reg_name(uint8_t id) {
    static char bufs[4][16];
    static int  bi = 0;
    char *buf = bufs[bi++ & 3];
    if (id < 32)  { snprintf(buf,16,"i%d",id);     return buf; }
    if (id < 64)  { snprintf(buf,16,"l%d",id-32);  return buf; }
    if (id < 96)  { snprintf(buf,16,"f%d",id-64);  return buf; }
    if (id < 128) { snprintf(buf,16,"d%d",id-96);  return buf; }
    if (id < 160) { snprintf(buf,16,"c%d",id-128); return buf; }
    if (id < 192) { snprintf(buf,16,"s%d",id-160); return buf; }
    if (id < 202) { snprintf(buf,16,"a%d",id-192); return buf; }
    if (id==202)  return "sp";
    if (id==203)  return "sf";
    if (id==204)  return "bp";
    if (id==205)  return "bf";
    snprintf(buf,16,"r%d",id); return buf;
}

/* ════════════════════════════════════════════════════════════════
   OPCODE NAME TABLE
════════════════════════════════════════════════════════════════ */

static const char *opname(uint8_t op) {
    switch(op) {
    case 0x01: return "mov";    case 0x02: return "movsx";  case 0x03: return "movzx";
    case 0x04: return "movsxd"; case 0x05: return "push";   case 0x06: return "pop";
    case 0x07: return "pusha";  case 0x08: return "popa";   case 0x09: return "lea";
    case 0x0A: return "fmov";   case 0x0B: return "dmov";   case 0x0C: return "lmov";
    case 0x0D: return "cmov";   case 0x0E: return "movb";   case 0x0F: return "movw";
    case 0x10: return "add";    case 0x11: return "addc";   case 0x12: return "sub";
    case 0x13: return "subb";   case 0x14: return "mul";    case 0x15: return "imul";
    case 0x16: return "div";    case 0x17: return "idiv";   case 0x18: return "inc";
    case 0x19: return "dec";    case 0x1A: return "neg";    case 0x1B: return "lneg";
    case 0x1C: return "ladd";   case 0x1D: return "lsub";   case 0x1E: return "lmul";
    case 0x1F: return "ldiv";   case 0x20: return "fadd";   case 0x21: return "fsub";
    case 0x22: return "fmul";   case 0x23: return "fdiv";   case 0x24: return "fneg";
    case 0x25: return "dadd";   case 0x26: return "dsub";   case 0x27: return "dmul";
    case 0x28: return "ddiv";   case 0x29: return "dneg";
    case 0x30: return "and";    case 0x31: return "or";     case 0x32: return "xor";
    case 0x33: return "not";    case 0x34: return "shl";    case 0x35: return "shr";
    case 0x36: return "sar";    case 0x37: return "rol";    case 0x38: return "ror";
    case 0x39: return "rcl";    case 0x3A: return "rcr";    case 0x3B: return "shld";
    case 0x3C: return "shrd";   case 0x3D: return "bsf";    case 0x3E: return "bsr";
    case 0x3F: return "bt";     case 0x40: return "bts";    case 0x41: return "btr";
    case 0x42: return "btc";    case 0x43: return "popcnt"; case 0x44: return "lzcnt";
    case 0x45: return "tzcnt";  case 0x46: return "test";   case 0x47: return "xchg";
    case 0x50: return "cmp";    case 0x51: return "lcmp";   case 0x52: return "fcmp";
    case 0x53: return "dcmp";   case 0x54: return "eq";     case 0x55: return "ne";
    case 0x56: return "gt";     case 0x57: return "lt";     case 0x58: return "gte";
    case 0x59: return "lte";
    case 0x60: return "jmp";    case 0x61: return "goto";   case 0x62: return "jcc";
    case 0x63: return "je";     case 0x64: return "jne";    case 0x65: return "jg";
    case 0x66: return "jge";    case 0x67: return "jl";     case 0x68: return "jle";
    case 0x69: return "ja";     case 0x6A: return "jb";     case 0x6B: return "loop";
    case 0x6C: return "call";   case 0x6D: return "ret";    case 0x6E: return "retn";
    case 0x6F: return "exit";   case 0x70: return "halt";
    case 0x78: return "movs";   case 0x79: return "stos";   case 0x7A: return "lods";
    case 0x7B: return "cmps";   case 0x7C: return "scas";   case 0x7D: return "rep";
    case 0x7E: return "repe";   case 0x7F: return "repne";
    case 0x80: return "nop";    case 0x81: return "wait";   case 0x82: return "in";
    case 0x83: return "out";    case 0x84: return "int";    case 0x85: return "iret";
    case 0x86: return "cpuid";  case 0x87: return "rdtsc";  case 0x88: return "pause";
    case 0x89: return "ud";     case 0x8A: return "cli";    case 0x8B: return "sti";
    case 0x90: return "itof";   case 0x91: return "itod";   case 0x92: return "itol";
    case 0x93: return "ltof";   case 0x94: return "ltod";   case 0x95: return "ftoi";
    case 0x96: return "ftod";   case 0x97: return "ftol";   case 0x98: return "dtoi";
    case 0x99: return "dtof";   case 0x9A: return "dtol";   case 0x9B: return "ltoi";
    default:   return "???";
    }
}

/* ════════════════════════════════════════════════════════════════
   DISASSEMBLE ONE INSTRUCTION
════════════════════════════════════════════════════════════════ */

/* returns number of bytes consumed, 0 on error */
static int disasm_one(const uint8_t *buf, int len, uint32_t pc, char *out, int outsz) {
    if (len < 2) { snprintf(out,outsz,"<truncated>"); return 0; }

    int pos = 0;
    uint8_t opcode = buf[pos++];
    uint8_t mod    = buf[pos++];

    int nops    = (mod >> 6) & 0x3;
    int has_imm = (mod >> 5) & 0x1;
    int has_mem = (mod >> 4) & 0x1;
    int has_ext = (mod >> 3) & 0x1;

    uint8_t dst_cls=0, s1_cls=0, s2_cls=0;
    if (has_ext) {
        if (pos+3 > len) { snprintf(out,outsz,"<truncated>"); return 0; }
        dst_cls = buf[pos++];
        s1_cls  = buf[pos++];
        s2_cls  = buf[pos++];
    }
    (void)s1_cls; (void)s2_cls; (void)dst_cls;

    /* read register ids */
    int nregs = nops - (has_imm?1:0) - (has_mem?1:0);
    if (nregs < 0) nregs = 0;
    uint8_t rv[4] = {0,0,0,0};
    for (int i = 0; i < nregs && i < 4; i++) {
        if (pos >= len) { snprintf(out,outsz,"<truncated>"); return 0; }
        rv[i] = buf[pos++];
    }

    /* read immediate */
    int32_t imm = 0;
    if (has_imm) {
        if (pos+4 > len) { snprintf(out,outsz,"<truncated>"); return 0; }
        memcpy(&imm, buf+pos, 4); pos += 4;
    }

    /* build disassembly string */
    const char *mn = opname(opcode);
    const char *r0 = reg_name(rv[0]);
    const char *r1 = reg_name(rv[1]);
    const char *r2 = reg_name(rv[2]);

    if (nops == 0) {
        snprintf(out,outsz,"%s", mn);
    } else if (opcode == 0x18 || opcode == 0x19) {         /* inc / dec */
        snprintf(out,outsz,"%-8s %s", mn, r0);
    } else if (opcode == 0x84) {                            /* int */
        snprintf(out,outsz,"%-8s 0x%02x", mn, (uint8_t)imm);
    } else if (opcode == 0x6F) {                            /* exit */
        if (has_imm) snprintf(out,outsz,"%-8s %d", mn, imm);
        else         snprintf(out,outsz,"%-8s %s", mn, r0);
    } else if (opcode == 0x6E) {                            /* retn */
        snprintf(out,outsz,"%-8s %d", mn, imm);
    } else if (opcode == 0x6D) {                            /* ret */
        snprintf(out,outsz,"ret");
    } else if (opcode == 0x6C) {                            /* call */
        snprintf(out,outsz,"%-8s 0x%08x", mn, (uint32_t)imm);
    } else if (opcode >= 0x60 && opcode <= 0x70) {         /* branches */
        if (nregs == 0)
            snprintf(out,outsz,"%-8s 0x%08x", mn, (uint32_t)imm);
        else
            snprintf(out,outsz,"%-8s %s, 0x%08x", mn, r0, (uint32_t)imm);
    } else if (nregs == 3 && !has_imm) {                   /* src1, src2, dst */
        snprintf(out,outsz,"%-8s %s, %s, %s", mn, r0, r1, r2);
    } else if (nregs == 2 && has_imm) {                    /* src, imm, dst */
        snprintf(out,outsz,"%-8s %s, %d, %s", mn, r0, imm, r1);
    } else if (nregs == 2 && !has_imm) {                   /* src, dst */
        snprintf(out,outsz,"%-8s %s, %s", mn, r0, r1);
    } else if (nregs == 1 && has_imm) {                    /* imm, dst  or  dst */
        snprintf(out,outsz,"%-8s 0x%x, %s", mn, (uint32_t)imm, r0);
    } else if (nregs == 1 && !has_imm) {
        snprintf(out,outsz,"%-8s %s", mn, r0);
    } else if (nregs == 0 && has_imm) {
        snprintf(out,outsz,"%-8s 0x%x", mn, (uint32_t)imm);
    } else {
        snprintf(out,outsz,"%-8s [?]", mn);
    }

    return pos;
}

/* ════════════════════════════════════════════════════════════════
   DISASSEMBLE A BUFFER
════════════════════════════════════════════════════════════════ */

static void disasm_buf(const uint8_t *buf, uint32_t size, uint32_t base_addr) {
    uint32_t off = 0;
    while (off < size) {
        char line[256];
        int n = disasm_one(buf+off, (int)(size-off), base_addr+off, line, sizeof(line));
        if (n <= 0) {
            printf("  %08x:  %02x                      ???\n", base_addr+off, buf[off]);
            off++;
            continue;
        }
        /* print address + raw bytes + mnemonic */
        printf("  %08x:  ", base_addr+off);
        for (int i = 0; i < n && i < 8; i++) printf("%02x ", buf[off+i]);
        for (int i = n; i < 8; i++) printf("   ");
        printf("  %s\n", line);
        off += (uint32_t)n;
    }
}

/* ════════════════════════════════════════════════════════════════
   LOAD AND DISASSEMBLE .cxe
════════════════════════════════════════════════════════════════ */

static void disasm_cxe(const char *path) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"cxdis: cannot open '%s'\n",path); return; }

    CxeHeader hdr;
    if (fread(&hdr,sizeof(hdr),1,f) != 1) { fclose(f); return; }
    if (hdr.magic != CXE_MAGIC) { fprintf(stderr,"cxdis: not a .cxe file\n"); fclose(f); return; }

    printf("; cxis disassembly of %s\n", path);
    printf("; entry point: 0x%08x\n\n", hdr.entry_point);

    for (int i = 0; i < hdr.section_count; i++) {
        CxeSection sec;
        if (fread(&sec,sizeof(sec),1,f) != 1) break;
        long saved = ftell(f);

        if (sec.flags & CXE_SEC_EXEC) {
            printf("section text:\n");
            uint8_t *buf = malloc(sec.file_size);
            fseek(f, sec.offset, SEEK_SET);
            if (fread(buf,1,sec.file_size,f) == sec.file_size) {
                disasm_buf(buf, sec.file_size, sec.vaddr);
            }
            free(buf);
            printf("\n");
        } else if (sec.flags & CXE_SEC_READ && !(sec.flags & CXE_SEC_WRITE)) {
            printf("section rodata:  ; vaddr=0x%08x size=%u\n", sec.vaddr, sec.mem_size);
            uint8_t *buf = malloc(sec.file_size);
            fseek(f, sec.offset, SEEK_SET);
            if (fread(buf,1,sec.file_size,f) == sec.file_size) {
                for (uint32_t j = 0; j < sec.file_size; j += 16) {
                    printf("  %08x:  ", sec.vaddr+j);
                    for (uint32_t k = j; k < j+16 && k < sec.file_size; k++)
                        printf("%02x ", buf[k]);
                    printf(" |");
                    for (uint32_t k = j; k < j+16 && k < sec.file_size; k++)
                        printf("%c", (buf[k]>=32&&buf[k]<127)?buf[k]:'.');
                    printf("|\n");
                }
            }
            free(buf);
            printf("\n");
        } else if (sec.flags & CXE_SEC_ZERO) {
            printf("section bss:     ; vaddr=0x%08x size=%u bytes\n\n", sec.vaddr, sec.mem_size);
        } else {
            printf("section data:    ; vaddr=0x%08x size=%u\n\n", sec.vaddr, sec.mem_size);
        }

        fseek(f, saved, SEEK_SET);
    }

    fclose(f);
}

/* ════════════════════════════════════════════════════════════════
   MAIN
════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,"usage: cxdis <file.cxe>\n");
        return 1;
    }
    disasm_cxe(argv[1]);
    return 0;
}
