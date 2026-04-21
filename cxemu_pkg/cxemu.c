/* cxemu.c — CXIS Machine Emulator + ABIOS
 * Build:  gcc -O2 -std=c11 -Wall -o cxemu cxemu.c
 * Usage:  cxemu <kernel.cxe> [disk0.img [disk1.img ...]]
 *         cxemu --help
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>

#include "include/cxis.h"
#include "include/cxe.h"
#include "include/abios.h"

/* ════════════════════════════════════════════════════════════════
   DISK STATE
════════════════════════════════════════════════════════════════ */

typedef struct {
    FILE    *fp;
    uint32_t sector_count;
    char     path[256];
} Disk;

/* ════════════════════════════════════════════════════════════════
   TIMER STATE
════════════════════════════════════════════════════════════════ */

typedef struct {
    uint32_t period_ms;
    uint32_t isr_addr;
    uint64_t next_fire_ns;
    int      active;
} TimerEntry;

#define MAX_TIMERS 8

/* ════════════════════════════════════════════════════════════════
   CPU STATE
════════════════════════════════════════════════════════════════ */

typedef struct {
    /* register file */
    int32_t  i[32];
    int64_t  l[32];
    float    f[32];
    double   d[32];
    int32_t  c[32];
    int32_t  s[32];
    uint64_t a[10];
    uint32_t sp, sf, bp, bf;
    uint32_t pc;
    uint8_t  carry;
    uint8_t  intf;
    uint8_t  halted;
    uint8_t  running;

    /* machine RAM */
    uint8_t *ram;
    uint32_t ram_size;

    /* ABIOS state */
    uint64_t boot_ns;          /* boot timestamp */
    Disk     disks[DISK_MAX_DRIVES];
    int      disk_count;
    TimerEntry timers[MAX_TIMERS];
    int      trace;
    long long steps;
} CPU;

/* ════════════════════════════════════════════════════════════════
   MEMORY HELPERS
════════════════════════════════════════════════════════════════ */

static void mem_fault(CPU *cpu, uint32_t addr, const char *op) {
    fprintf(stderr, "cxemu: %s fault at 0x%08X (pc=0x%08X)\n", op, addr, cpu->pc);
    cpu->running = 0;
}

static uint8_t  mr8 (CPU *c, uint32_t a)          { if (a>=c->ram_size){mem_fault(c,a,"read8");return 0;}  return c->ram[a]; }
static uint16_t mr16(CPU *c, uint32_t a)          { uint16_t v; if(a+1>=c->ram_size){mem_fault(c,a,"read16");return 0;} memcpy(&v,c->ram+a,2); return v; }
static uint32_t mr32(CPU *c, uint32_t a)          { uint32_t v; if(a+3>=c->ram_size){mem_fault(c,a,"read32");return 0;} memcpy(&v,c->ram+a,4); return v; }
static uint64_t mr64(CPU *c, uint32_t a)          { uint64_t v; if(a+7>=c->ram_size){mem_fault(c,a,"read64");return 0;} memcpy(&v,c->ram+a,8); return v; }
static float    mrf (CPU *c, uint32_t a)          { float    v; if(a+3>=c->ram_size){mem_fault(c,a,"readf");return 0;}  memcpy(&v,c->ram+a,4); return v; }
static double   mrd (CPU *c, uint32_t a)          { double   v; if(a+7>=c->ram_size){mem_fault(c,a,"readd");return 0;}  memcpy(&v,c->ram+a,8); return v; }
static void     mw8 (CPU *c, uint32_t a, uint8_t  v) { if(a>=c->ram_size){mem_fault(c,a,"write8");return;}  c->ram[a]=v; }
static void     mw32(CPU *c, uint32_t a, uint32_t v) { if(a+3>=c->ram_size){mem_fault(c,a,"write32");return;} memcpy(c->ram+a,&v,4); }
static void     mw64(CPU *c, uint32_t a, uint64_t v) { if(a+7>=c->ram_size){mem_fault(c,a,"write64");return;} memcpy(c->ram+a,&v,8); }
static void     mwf (CPU *c, uint32_t a, float    v) { if(a+3>=c->ram_size){mem_fault(c,a,"writef");return;} memcpy(c->ram+a,&v,4); }
static void     mwd (CPU *c, uint32_t a, double   v) { if(a+7>=c->ram_size){mem_fault(c,a,"writed");return;} memcpy(c->ram+a,&v,8); }

static uint8_t  f8 (CPU *c) { uint8_t  v=mr8 (c,c->pc); c->pc+=1; return v; }
static uint16_t f16(CPU *c) { uint16_t v=mr16(c,c->pc); c->pc+=2; return v; }
static uint32_t f32(CPU *c) { uint32_t v=mr32(c,c->pc); c->pc+=4; return v; }
static uint64_t f64(CPU *c) { uint64_t v=mr64(c,c->pc); c->pc+=8; return v; }
static float    ff (CPU *c) { float    v=mrf (c,c->pc); c->pc+=4; return v; }
static double   fd (CPU *c) { double   v=mrd (c,c->pc); c->pc+=8; return v; }

static void     push32(CPU *c, uint32_t v) { c->sp-=4; mw32(c,c->sp,v); }
static uint32_t pop32 (CPU *c)             { uint32_t v=mr32(c,c->sp); c->sp+=4; return v; }

/* ════════════════════════════════════════════════════════════════
   REGISTER ACCESS
════════════════════════════════════════════════════════════════ */

static int32_t  get_i(CPU *c,uint8_t id){return c->i[id];}
static int64_t  get_l(CPU *c,uint8_t id){return c->l[id-32];}
static float    get_f(CPU *c,uint8_t id){return c->f[id-64];}
static double   get_d(CPU *c,uint8_t id){return c->d[id-96];}
static int32_t  get_c(CPU *c,uint8_t id){return c->c[id-128];}
static void set_i(CPU *c,uint8_t id,int32_t  v){c->i[id]=v;}
static void set_l(CPU *c,uint8_t id,int64_t  v){c->l[id-32]=v;}
static void set_f(CPU *c,uint8_t id,float    v){c->f[id-64]=v;}
static void set_d(CPU *c,uint8_t id,double   v){c->d[id-96]=v;}
static void set_c(CPU *c,uint8_t id,int32_t  v){c->c[id-128]=v;}

static int32_t getreg32(CPU *c, uint8_t id) {
    if (id<32)  return c->i[id];
    if (id<64)  return (int32_t)c->l[id-32];
    if (id<96)  return (int32_t)c->f[id-64];
    if (id<128) return (int32_t)c->d[id-96];
    if (id<160) return c->c[id-128];
    if (id<192) return c->s[id-160];
    if (id<202) return (int32_t)c->a[id-192];
    if (id==202) return (int32_t)c->sp;
    if (id==203) return (int32_t)c->sf;
    if (id==204) return (int32_t)c->bp;
    if (id==205) return (int32_t)c->bf;
    return 0;
}
static void setreg32(CPU *c, uint8_t id, int32_t v) {
    if (id<32)  { c->i[id]=v; return; }
    if (id<64)  { c->l[id-32]=(int64_t)v; return; }
    if (id<96)  { c->f[id-64]=(float)v; return; }
    if (id<128) { c->d[id-96]=(double)v; return; }
    if (id<160) { c->c[id-128]=v; return; }
    if (id<192) { c->s[id-160]=v; return; }
    if (id<202) { c->a[id-192]=(uint64_t)(int64_t)v; return; }
    if (id==202) { c->sp=(uint32_t)v; return; }
    if (id==203) { c->sf=(uint32_t)v; return; }
    if (id==204) { c->bp=(uint32_t)v; return; }
    if (id==205) { c->bf=(uint32_t)v; return; }
}

/* ════════════════════════════════════════════════════════════════
   MEMORY OPERAND DECODE
════════════════════════════════════════════════════════════════ */

static uint32_t decode_mem_addr(CPU *c) {
    uint8_t  mf   = f8(c);
    uint32_t addr = 0;
    if (mf & 0x01) { uint8_t base=f8(c); addr+=(uint32_t)getreg32(c,base); }
    if (mf & 0x02) { uint8_t idx=f8(c); uint8_t sc=f8(c); addr+=(uint32_t)getreg32(c,idx)*sc; }
    addr += f32(c);
    return addr;
}

/* ════════════════════════════════════════════════════════════════
   TIMER HELPERS
════════════════════════════════════════════════════════════════ */

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static uint64_t elapsed_ns(CPU *c) { return now_ns() - c->boot_ns; }
static uint64_t elapsed_ms(CPU *c) { return elapsed_ns(c) / 1000000ULL; }

static void check_timers(CPU *c) {
    if (!c->intf) return;
    uint64_t t = elapsed_ns(c);
    for (int i = 0; i < MAX_TIMERS; i++) {
        TimerEntry *te = &c->timers[i];
        if (!te->active) continue;
        if (t >= te->next_fire_ns) {
            te->next_fire_ns += (uint64_t)te->period_ms * 1000000ULL;
            /* call ISR: push return address, jump */
            push32(c, c->pc);
            c->pc = te->isr_addr;
        }
    }
}

/* ════════════════════════════════════════════════════════════════
   ABIOS VIDEO — writes to VRAM in RAM, mirrors to terminal
════════════════════════════════════════════════════════════════ */

/* ANSI color map: 8 standard CGA colors → ANSI */
static const char *ansi_fg[16] = {
    "30","34","32","36","31","35","33","37",
    "90","94","92","96","91","95","93","97"
};
static const char *ansi_bg[8] = {
    "40","44","42","46","41","45","43","47"
};

static void vid_update_cell(CPU *c, int col, int row) {
    uint32_t off = MEM_VRAM_BASE + (uint32_t)(row * VID_COLS + col) * 2;
    uint8_t  ch  = c->ram[off];
    uint8_t  at  = c->ram[off+1];
    int fg = at & 0x0F;
    int bg = (at >> 4) & 0x07;
    /* move cursor to position, set colors, print char */
    printf("\033[%d;%dH\033[%s;%sm%c",
           row+1, col+1,
           ansi_fg[fg], ansi_bg[bg],
           ch ? ch : ' ');
    fflush(stdout);
}

static void vid_clear(CPU *c, uint8_t attr) {
    for (int row = 0; row < VID_ROWS; row++)
        for (int col = 0; col < VID_COLS; col++) {
            uint32_t off = MEM_VRAM_BASE + (uint32_t)(row*VID_COLS+col)*2;
            c->ram[off]   = ' ';
            c->ram[off+1] = attr;
        }
    printf("\033[2J\033[H\033[0m");
    fflush(stdout);
}

static void vid_putchar_at(CPU *c, uint8_t ch, int col, int row, uint8_t attr) {
    if (col < 0 || col >= VID_COLS || row < 0 || row >= VID_ROWS) return;
    uint32_t off = MEM_VRAM_BASE + (uint32_t)(row*VID_COLS+col)*2;
    c->ram[off]   = ch;
    c->ram[off+1] = attr;
    vid_update_cell(c, col, row);
}

static void vid_scroll_up(CPU *c, int lines, uint8_t attr) {
    if (lines <= 0) return;
    if (lines >= VID_ROWS) { vid_clear(c, attr); return; }
    /* shift VRAM up */
    memmove(c->ram + MEM_VRAM_BASE,
            c->ram + MEM_VRAM_BASE + (uint32_t)lines*VID_COLS*2,
            (uint32_t)(VID_ROWS-lines)*VID_COLS*2);
    /* blank bottom lines */
    for (int r = VID_ROWS-lines; r < VID_ROWS; r++)
        for (int col = 0; col < VID_COLS; col++) {
            uint32_t off = MEM_VRAM_BASE + (uint32_t)(r*VID_COLS+col)*2;
            c->ram[off]   = ' ';
            c->ram[off+1] = attr;
        }
    printf("\033[%dS", lines);   /* ANSI scroll up */
    fflush(stdout);
}

/* ── ABIOS data area helpers ── */
static uint8_t  ada_r8 (CPU *c, int off) { return c->ram[MEM_ABIOS_DATA + off]; }
static uint32_t ada_r32(CPU *c, int off) { return mr32(c, MEM_ABIOS_DATA + off); }
static void     ada_w8 (CPU *c, int off, uint8_t  v) { c->ram[MEM_ABIOS_DATA+off]=v; }
static void     ada_w32(CPU *c, int off, uint32_t v) { mw32(c, MEM_ABIOS_DATA+off, v); }

static int  cur_x(CPU *c) { return ada_r8(c, ADA_CUR_X); }
static int  cur_y(CPU *c) { return ada_r8(c, ADA_CUR_Y); }
static uint8_t def_attr(CPU *c) { return ada_r8(c, ADA_ATTR); }

/* ── console print through video ── */
static void con_emit(CPU *c, uint8_t ch) {
    uint8_t attr = def_attr(c);
    int x = cur_x(c), y = cur_y(c);

    if (ch == '\n') {
        x = 0; y++;
    } else if (ch == '\r') {
        x = 0;
    } else if (ch == '\t') {
        int nx = (x + 8) & ~7;
        for (; x < nx && x < VID_COLS; x++)
            vid_putchar_at(c, ' ', x, y, attr);
    } else if (ch == '\b') {
        if (x > 0) { x--; vid_putchar_at(c, ' ', x, y, attr); }
    } else {
        vid_putchar_at(c, ch, x, y, attr);
        x++;
        if (x >= VID_COLS) { x = 0; y++; }
    }

    if (y >= VID_ROWS) {
        vid_scroll_up(c, y - VID_ROWS + 1, attr);
        y = VID_ROWS - 1;
    }

    ada_w8(c, ADA_CUR_X, (uint8_t)x);
    ada_w8(c, ADA_CUR_Y, (uint8_t)y);
    /* reposition terminal cursor */
    printf("\033[%d;%dH", y+1, x+1);
    fflush(stdout);
}

static void con_puts_raw(CPU *c, uint32_t ptr, uint32_t len) {
    for (uint32_t i = 0; i < len && ptr+i < c->ram_size; i++)
        con_emit(c, c->ram[ptr+i]);
}

static void con_puts_cstr(CPU *c, uint32_t ptr) {
    while (ptr < c->ram_size && c->ram[ptr])
        con_emit(c, c->ram[ptr++]);
}

static void con_print_int(CPU *c, int32_t val) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%d", val);
    for (int i = 0; buf[i]; i++) con_emit(c, (uint8_t)buf[i]);
}

static void con_print_hex(CPU *c, uint32_t val) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%08X", val);
    for (int i = 0; buf[i]; i++) con_emit(c, (uint8_t)buf[i]);
}

/* ════════════════════════════════════════════════════════════════
   ABIOS DISK
════════════════════════════════════════════════════════════════ */

static int disk_read_sectors(CPU *c, int id, uint32_t lba, uint32_t buf, uint32_t count) {
    if (id < 0 || id >= c->disk_count || !c->disks[id].fp) return 1;
    if (buf + count*DISK_SECTOR_SIZE > c->ram_size) return 1;
    fseek(c->disks[id].fp, (long)lba * DISK_SECTOR_SIZE, SEEK_SET);
    size_t got = fread(c->ram + buf, DISK_SECTOR_SIZE, count, c->disks[id].fp);
    return (got == count) ? 0 : 1;
}

static int disk_write_sectors(CPU *c, int id, uint32_t lba, uint32_t buf, uint32_t count) {
    if (id < 0 || id >= c->disk_count || !c->disks[id].fp) return 1;
    if (buf + count*DISK_SECTOR_SIZE > c->ram_size) return 1;
    fseek(c->disks[id].fp, (long)lba * DISK_SECTOR_SIZE, SEEK_SET);
    size_t wrote = fwrite(c->ram + buf, DISK_SECTOR_SIZE, count, c->disks[id].fp);
    fflush(c->disks[id].fp);
    return (wrote == count) ? 0 : 1;
}

/* ════════════════════════════════════════════════════════════════
   ABIOS MEMORY MAP (written into RAM at MEM_ABIOS_STR area)
════════════════════════════════════════════════════════════════ */

#define MEM_MAP_ADDR  (MEM_ABIOS_STR + 0x100)
#define MEM_MAP_ENTRIES 7

static void abios_write_memmap(CPU *c) {
    /* write AbiosMemEntry array into RAM */
    typedef struct { uint32_t base; uint32_t size; uint8_t type; uint8_t pad[3]; } E;
    E map[MEM_MAP_ENTRIES] = {
        {MEM_IVT_BASE,        MEM_IVT_SIZE,         ABIOS_MEM_RESERVED, {0,0,0}},
        {MEM_ABIOS_DATA,      0x00BFF000,            ABIOS_MEM_FIRMWARE, {0,0,0}},
        {MEM_VRAM_BASE,       MEM_VRAM_SIZE,         ABIOS_MEM_VRAM,     {0,0,0}},
        {MEM_KBUF_BASE,       MEM_KBUF_SIZE,         ABIOS_MEM_FIRMWARE, {0,0,0}},
        {MEM_DISK_CACHE_BASE, MEM_DISK_CACHE_SIZE,   ABIOS_MEM_FIRMWARE, {0,0,0}},
        {MEM_KERNEL_BASE,     0x01000000,            ABIOS_MEM_KERNEL,   {0,0,0}},
        {MEM_HEAP_BASE,       MEM_STACK_TOP - MEM_HEAP_BASE, ABIOS_MEM_FREE, {0,0,0}},
    };
    memcpy(c->ram + MEM_MAP_ADDR, map, sizeof(map));
}

/* ════════════════════════════════════════════════════════════════
   ABIOS INTERRUPT HANDLER
════════════════════════════════════════════════════════════════ */

/* ── legacy cxvm BIOS vectors (for old .cxe binaries) ──────────
   int 0x01 = PRINT_STR  a0=ptr a1=len
   int 0x02 = PRINT_CHAR a0=char
   int 0x03 = READ_CHAR  → a0
   int 0x04 = EXIT       a0=code
   int 0x05 = MEM_ALLOC  a0=size → a0=ptr
   int 0x06 = MEM_FREE
   int 0x07 = TIME       → l0=ns
   These shadow the lower ABIOS vectors.  OS code that wants the
   full ABIOS must call int 0x10–0x18 (remapped) OR set a0 to a
   sub-function before calling int 0x01–0x09 with the high bit of
   a0 set (0x80000000 flag).  For simplicity we detect legacy by
   checking if a0 looks like a small sub-fn id vs a pointer/code. */

#define LEGACY_THRESHOLD 0x10   /* if a0 < 16 assume sub-fn (ABIOS), else legacy? */
/* Actually simpler: keep legacy on 0x01-0x07, add new ABIOS on 0x10-0x18 */

static void abios_handle(CPU *c, uint8_t vector) {
    uint32_t fn   = (uint32_t)c->a[0];
    uint32_t arg1 = (uint32_t)c->a[1];
    uint32_t arg2 = (uint32_t)c->a[2];
    uint32_t arg3 = (uint32_t)c->a[3];
    uint32_t arg4 = (uint32_t)c->a[4];

    /* ── legacy cxvm shim (vectors 0x01–0x07) ── */
    switch (vector) {
    case 0x01: /* PRINT_STR: a0=ptr a1=len */
        con_puts_raw(c, fn, arg1); return;
    case 0x02: /* PRINT_CHAR: a0=char */
        con_emit(c, (uint8_t)fn); return;
    case 0x03: /* READ_CHAR */
        { int ch = getchar(); c->a[0]=(ch==EOF)?0xFFFFFFFFu:(uint64_t)(unsigned char)ch; return; }
    case 0x04: /* EXIT: a0=code */
        fprintf(stderr,"\ncxemu: exit(%u)  steps=%lld\n", fn, c->steps);
        exit((int)fn);
    case 0x05: /* MEM_ALLOC: a0=size → a0=ptr */
        { uint32_t sz=(fn+7)&~7u; uint32_t ptr=ada_r32(c,ADA_HEAP_PTR);
          if(ptr+sz < MEM_STACK_TOP-0x10000){c->a[0]=ptr;ada_w32(c,ADA_HEAP_PTR,ptr+sz);}
          else c->a[0]=0; return; }
    case 0x06: /* MEM_FREE */ return;
    case 0x07: /* TIME → l0=ns */
        c->l[0]=(int64_t)elapsed_ns(c); return;
    }

    /* ── full ABIOS vectors (0x10–0x18 or any unhandled) ── */
    /* remap 0x10-0x18 → 0x01-0x09 */
    if (vector >= 0x10 && vector <= 0x18) vector = (uint8_t)(vector - 0x0F);

    switch (vector) {

    /* ── CONSOLE ── */
    case VEC_CONSOLE:
        switch (fn) {
        case CON_PUTCHAR:  con_emit(c, (uint8_t)arg1); break;
        case CON_WRITE:    con_puts_raw(c, arg1, arg2); break;
        case CON_PUTS:     con_puts_cstr(c, arg1); break;
        case CON_GETCHAR: {
            int ch = getchar();
            c->a[0] = (ch == EOF) ? 0xFFFFFFFFu : (uint64_t)(unsigned char)ch;
            break;
        }
        case CON_READLINE: {
            uint32_t buf = arg1, maxlen = arg2, got = 0;
            int ch;
            while (got < maxlen-1) {
                ch = getchar();
                if (ch == EOF || ch == '\n') break;
                c->ram[buf + got++] = (uint8_t)ch;
                con_emit(c, (uint8_t)ch);
            }
            c->ram[buf + got] = 0;
            c->a[0] = got;
            con_emit(c, '\n');
            break;
        }
        case CON_PRINT_INT: con_print_int(c, (int32_t)arg1); break;
        case CON_PRINT_HEX: con_print_hex(c, arg1); break;
        case CON_EXIT:      exit((int)arg1); break;
        default:
            fprintf(stderr, "abios: unknown console fn 0x%02X\n", fn);
        }
        break;

    /* ── KEYBOARD ── */
    case VEC_KEYBOARD:
        switch (fn) {
        case KB_READ: {
            int ch = getchar();
            c->a[0] = (ch == EOF) ? 0u : (uint64_t)(unsigned char)ch;
            break;
        }
        case KB_POLL:
            /* non-blocking not easily portable without termios; return 0 */
            c->a[0] = 0;
            break;
        case KB_FLUSH:
            /* flush stdin */
            while ((getchar()) != '\n' && !feof(stdin));
            break;
        case KB_AVAILABLE:
            c->a[0] = 0;
            break;
        }
        break;

    /* ── VIDEO ── */
    case VEC_VIDEO: {
        uint8_t attr = (uint8_t)arg4;
        switch (fn) {
        case VID_CLEAR:
            vid_clear(c, (uint8_t)arg1);
            ada_w8(c, ADA_ATTR, (uint8_t)arg1);
            break;
        case VID_PUTCHAR:
            vid_putchar_at(c, (uint8_t)arg1, (int)arg2, (int)arg3, attr);
            break;
        case VID_PUTS: {
            uint32_t ptr = arg1;
            int col = (int)arg2, row = (int)arg3;
            while (ptr < c->ram_size && c->ram[ptr] && col < VID_COLS) {
                vid_putchar_at(c, c->ram[ptr++], col++, row, attr);
            }
            break;
        }
        case VID_SCROLL_UP:
            vid_scroll_up(c, (int)arg1, (uint8_t)arg2);
            break;
        case VID_SET_CURSOR:
            ada_w8(c, ADA_CUR_X, (uint8_t)arg1);
            ada_w8(c, ADA_CUR_Y, (uint8_t)arg2);
            printf("\033[%d;%dH", (int)arg2+1, (int)arg1+1);
            fflush(stdout);
            break;
        case VID_GET_CURSOR:
            c->a[0] = cur_x(c);
            c->a[1] = cur_y(c);
            break;
        case VID_SET_ATTR:
            ada_w8(c, ADA_ATTR, (uint8_t)arg1);
            break;
        case VID_GET_CHAR: {
            int col=(int)arg1, row=(int)arg2;
            if (col>=0&&col<VID_COLS&&row>=0&&row<VID_ROWS) {
                uint32_t off = MEM_VRAM_BASE + (uint32_t)(row*VID_COLS+col)*2;
                c->a[0] = c->ram[off];
                c->a[1] = c->ram[off+1];
            } else { c->a[0]=0; c->a[1]=0; }
            break;
        }
        case VID_VRAM_ADDR:
            c->a[0] = MEM_VRAM_BASE;
            break;
        case VID_DIMENSIONS:
            c->a[0] = VID_COLS;
            c->a[1] = VID_ROWS;
            break;
        case VID_WRITE_RAW:
            vid_putchar_at(c, (uint8_t)arg3, (int)arg1, (int)arg2, (uint8_t)arg4);
            break;
        default:
            fprintf(stderr, "abios: unknown video fn 0x%02X\n", fn);
        }
        break;
    }

    /* ── DISK ── */
    case VEC_DISK:
        switch (fn) {
        case DISK_READ:
            c->a[0] = disk_read_sectors(c,(int)arg1,arg2,arg3,arg4);
            break;
        case DISK_WRITE:
            c->a[0] = disk_write_sectors(c,(int)arg1,arg2,arg3,arg4);
            break;
        case DISK_SECTORS:
            c->a[0] = (arg1<(uint32_t)c->disk_count) ? c->disks[arg1].sector_count : 0;
            break;
        case DISK_COUNT:
            c->a[0] = (uint32_t)c->disk_count;
            break;
        case DISK_SECTOR_SZ:
            c->a[0] = DISK_SECTOR_SIZE;
            break;
        case DISK_FLUSH:
            for (int d=0;d<c->disk_count;d++)
                if (c->disks[d].fp) fflush(c->disks[d].fp);
            break;
        default:
            fprintf(stderr,"abios: unknown disk fn 0x%02X\n",fn);
        }
        break;

    /* ── MEMORY ── */
    case VEC_MEMORY:
        switch (fn) {
        case MEM_ALLOC: {
            uint32_t sz  = (arg1 + 7) & ~7u;
            uint32_t ptr = ada_r32(c, ADA_HEAP_PTR);
            if (ptr + sz < MEM_STACK_TOP - 0x10000) {
                c->a[0] = ptr;
                ada_w32(c, ADA_HEAP_PTR, ptr + sz);
            } else { c->a[0] = 0; }
            break;
        }
        case MEM_FREE: break;   /* bump allocator */
        case MEM_TOTAL_Q:
            c->a[0] = c->ram_size;
            break;
        case MEM_FREE_Q: {
            uint32_t used = ada_r32(c, ADA_HEAP_PTR) - MEM_HEAP_BASE;
            uint32_t total = MEM_STACK_TOP - 0x10000 - MEM_HEAP_BASE;
            c->a[0] = (used < total) ? total - used : 0;
            break;
        }
        case MEM_MAP_Q:
            c->a[0] = MEM_MAP_ADDR;
            c->a[1] = MEM_MAP_ENTRIES;
            break;
        case MEM_COPY:
            if (arg1 + arg3 <= c->ram_size && arg2 + arg3 <= c->ram_size)
                memmove(c->ram + arg1, c->ram + arg2, arg3);
            break;
        case MEM_SET:
            if (arg1 + arg3 <= c->ram_size)
                memset(c->ram + arg1, (int)arg2, arg3);
            break;
        case MEM_COMPARE:
            if (arg1 + arg3 <= c->ram_size && arg2 + arg3 <= c->ram_size)
                c->a[0] = (uint64_t)(int64_t)memcmp(c->ram+arg1, c->ram+arg2, arg3);
            break;
        default:
            fprintf(stderr,"abios: unknown memory fn 0x%02X\n",fn);
        }
        break;

    /* ── TIMER ── */
    case VEC_TIMER:
        switch (fn) {
        case TIMER_TICKS_NS:
            c->l[0] = (int64_t)elapsed_ns(c);
            break;
        case TIMER_TICKS_MS:
            c->a[0] = (uint32_t)elapsed_ms(c);
            break;
        case TIMER_SLEEP_MS: {
            uint64_t end = elapsed_ms(c) + arg1;
            while (elapsed_ms(c) < end);
            break;
        }
        case TIMER_UPTIME:
            c->a[0] = (uint32_t)(elapsed_ns(c) / 1000000000ULL);
            break;
        case TIMER_INSTALL:
            for (int t = 0; t < MAX_TIMERS; t++) {
                if (!c->timers[t].active) {
                    c->timers[t].period_ms   = arg1;
                    c->timers[t].isr_addr    = arg2;
                    c->timers[t].next_fire_ns= elapsed_ns(c)+(uint64_t)arg1*1000000ULL;
                    c->timers[t].active      = 1;
                    c->a[0] = (uint32_t)t;
                    break;
                }
            }
            break;
        case TIMER_REMOVE:
            if (arg1 < MAX_TIMERS) c->timers[arg1].active = 0;
            break;
        default:
            fprintf(stderr,"abios: unknown timer fn 0x%02X\n",fn);
        }
        break;

    /* ── POWER ── */
    case VEC_POWER:
        switch (fn) {
        case PWR_SHUTDOWN:
            fprintf(stderr,"\ncxemu: shutdown (code=%u, steps=%lld)\n",
                    arg1, c->steps);
            exit((int)arg1);
        case PWR_REBOOT:
            fprintf(stderr,"\ncxemu: reboot requested (not implemented — halt)\n");
            c->halted = 1; c->running = 0;
            break;
        case PWR_HALT:
            c->halted = 1; c->running = 0;
            break;
        }
        break;

    /* ── SYSINFO ── */
    case VEC_SYSINFO:
        switch (fn) {
        case SI_CPUID:
            c->a[0] = CXIS_SIG;
            c->a[1] = CXIS_VER;
            c->a[2] = CXIS_FEAT_FLOAT|CXIS_FEAT_DOUBLE|CXIS_FEAT_ABIOS;
            break;
        case SI_MEMSIZE:
            c->a[0] = c->ram_size;
            break;
        case SI_DISK_COUNT:
            c->a[0] = (uint32_t)c->disk_count;
            break;
        case SI_BIOS_VER: {
            const char *ver = "ABIOS v1.0 (cxemu)";
            uint32_t dst = MEM_ABIOS_STR;
            strncpy((char*)(c->ram + dst), ver, 64);
            c->a[0] = dst;
            break;
        }
        case SI_BOOT_DISK:
            c->a[0] = ada_r8(c, ADA_BOOT_DISK);
            break;
        case SI_MEM_MAP:
            c->a[0] = MEM_MAP_ADDR;
            c->a[1] = MEM_MAP_ENTRIES;
            break;
        default:
            fprintf(stderr,"abios: unknown sysinfo fn 0x%02X\n",fn);
        }
        break;

    /* ── IRQ ── */
    case VEC_IRQ:
        switch (fn) {
        case IRQ_INSTALL:
            if (arg1 < 256) mw32(c, IVT_ENTRY(arg1), arg2);
            break;
        case IRQ_REMOVE:
            if (arg1 < 256) mw32(c, IVT_ENTRY(arg1), 0);
            break;
        case IRQ_ENABLE:  c->intf = 1; break;
        case IRQ_DISABLE: c->intf = 0; break;
        case IRQ_FIRE: {
            uint32_t isr = mr32(c, IVT_ENTRY(arg1 & 0xFF));
            if (isr) { push32(c, c->pc); c->pc = isr; }
            break;
        }
        }
        break;

    default:
        fprintf(stderr,"abios: unhandled vector 0x%02X at pc=0x%08X\n",
                vector, c->pc);
    }
}

/* ════════════════════════════════════════════════════════════════
   INSTRUCTION DECODE & EXECUTE  (mirrors cxvm.c + fixes)
════════════════════════════════════════════════════════════════ */

static void step(CPU *c) {
    c->steps++;

    uint8_t opcode = f8(c);
    uint8_t mod    = f8(c);

    int nops    = (mod >> 6) & 0x3;
    int has_imm = (mod >> 5) & 0x1;
    int has_mem = (mod >> 4) & 0x1;
    int has_ext = (mod >> 3) & 0x1;

    if (has_ext) { f8(c); f8(c); f8(c); } /* skip ext type bytes */

    uint8_t rv[4] = {0,0,0,0};
    int     nr    = 0;
    int64_t imm   = 0;
    uint32_t label_addr = 0;

    int nregs = nops - (has_imm ? 1 : 0) - (has_mem ? 1 : 0);
    if (nregs < 0) nregs = 0;
    for (int k = 0; k < nregs && k < 4; k++) rv[nr++] = f8(c);
    if (has_imm) {
        imm = (int64_t)(int32_t)f32(c);
        label_addr = (uint32_t)(int32_t)imm;
    }

#define GR(id)    getreg32(c,id)
#define SR(id,v)  setreg32(c,id,(int32_t)(v))
#define RV0 rv[0]
#define RV1 rv[1]
#define RV2 rv[2]

    switch (opcode) {

    case OP_NOP:  break;
    case OP_HALT: c->halted=1; c->running=0; break;

    /* ── DATA MOVEMENT ── */
    case OP_MOV: {
        if (has_imm && !has_mem) { SR(RV0,(int32_t)imm); }
        else if (has_mem) {
            uint32_t addr = decode_mem_addr(c);
            if (mod & 0x04) mw32(c,addr,(uint32_t)GR(RV0));
            else            SR(RV0,(int32_t)mr32(c,addr));
        } else { SR(RV1,GR(RV0)); }
        break; }
    case OP_MOVB: {
        if (has_mem) {
            uint32_t addr = decode_mem_addr(c);
            if (mod & 0x04) mw8(c,addr,(uint8_t)GR(RV0));
            else            SR(RV0,(int32_t)(uint32_t)mr8(c,addr));
        } else { SR(RV1,GR(RV0)&0xFF); }
        break; }
    case OP_MOVW: {
        if (has_mem) {
            uint32_t addr = decode_mem_addr(c);
            if (mod & 0x04) { uint16_t v=(uint16_t)GR(RV0); memcpy(c->ram+addr,&v,2); }
            else { uint16_t v; memcpy(&v,c->ram+addr,2); SR(RV0,(int32_t)(uint32_t)v); }
        } else { SR(RV1,GR(RV0)&0xFFFF); }
        break; }
    case OP_FMOV: {
        if (has_imm) { float fv; memcpy(&fv,&imm,4); set_f(c,RV0,fv); }
        else         set_f(c,RV1,get_f(c,RV0));
        break; }
    case OP_DMOV: {
        if (has_imm) set_d(c,RV0,(double)imm);
        else         set_d(c,RV1,get_d(c,RV0));
        break; }
    case OP_LMOV: {
        if (has_imm) set_l(c,RV0,imm);
        else         set_l(c,RV1,get_l(c,RV0));
        break; }
    case OP_MOVSX:  SR(RV1,(int32_t)(int16_t)(GR(RV0)&0xFFFF)); break;
    case OP_MOVZX:  SR(RV1,(int32_t)(uint16_t)(uint32_t)GR(RV0)); break;
    case OP_MOVSXD: set_l(c,RV1,(int64_t)(int32_t)GR(RV0)); break;
    case OP_LEA: {
        uint32_t addr = decode_mem_addr(c);
        uint8_t  dst  = f8(c);
        SR(dst,(int32_t)addr); break; }
    case OP_CMOV:   if(get_c(c,RV0)) SR(RV2,GR(RV1)); break;
    case OP_PUSH:   push32(c,(uint32_t)(has_imm?(int32_t)imm:GR(RV0))); break;
    case OP_POP:    SR(RV0,(int32_t)pop32(c)); break;
    case OP_PUSHA:  for(int k=31;k>=0;k--) push32(c,(uint32_t)c->i[k]); break;
    case OP_POPA:   for(int k=0;k<32;k++) c->i[k]=(int32_t)pop32(c); break;

    /* ── ARITHMETIC i32 ── */
    case OP_ADD: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; int64_t r=(int64_t)a+b; c->carry=(r>(int64_t)0x7FFFFFFF||r<(int64_t)(int32_t)0x80000000)?1:0; SR(d,(int32_t)r); break; }
    case OP_ADDC:{ int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; int64_t r=(int64_t)a+b+c->carry; c->carry=(uint32_t)(r>>32)?1:0; SR(d,(int32_t)r); break; }
    case OP_SUB: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; int64_t r=(int64_t)a-b; c->carry=(r<0)?1:0; SR(d,(int32_t)r); break; }
    case OP_SUBB:{ int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; int64_t r=(int64_t)a-b-c->carry; c->carry=(r<0)?1:0; SR(d,(int32_t)r); break; }
    case OP_MUL: { uint32_t a=(uint32_t)GR(RV0),b=has_imm?(uint32_t)imm:(uint32_t)GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)(a*b)); break; }
    case OP_IMUL:{ int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a*b); break; }
    case OP_DIV: { uint32_t a=(uint32_t)GR(RV0),b=has_imm?(uint32_t)imm:(uint32_t)GR(RV1); uint8_t d=has_imm?RV1:RV2; if(!b){fprintf(stderr,"cxemu: div/0\n");c->running=0;break;} SR(d,(int32_t)(a/b)); break; }
    case OP_IDIV:{ int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; if(!b){fprintf(stderr,"cxemu: div/0\n");c->running=0;break;} SR(d,a/b); break; }
    case OP_INC: SR(RV0,GR(RV0)+1); break;
    case OP_DEC: SR(RV0,GR(RV0)-1); break;
    case OP_NEG: SR(RV1,-GR(RV0)); break;

    /* ── ARITHMETIC i64 ── */
    case OP_LNEG: set_l(c,RV1,-get_l(c,RV0)); break;
    case OP_LADD: set_l(c,RV2,get_l(c,RV0)+get_l(c,RV1)); break;
    case OP_LSUB: set_l(c,RV2,get_l(c,RV0)-get_l(c,RV1)); break;
    case OP_LMUL: set_l(c,RV2,get_l(c,RV0)*get_l(c,RV1)); break;
    case OP_LDIV: { if(!get_l(c,RV1)){fprintf(stderr,"cxemu: div/0\n");c->running=0;break;} set_l(c,RV2,get_l(c,RV0)/get_l(c,RV1)); break; }

    /* ── ARITHMETIC float ── */
    case OP_FADD: set_f(c,RV2,get_f(c,RV0)+get_f(c,RV1)); break;
    case OP_FSUB: set_f(c,RV2,get_f(c,RV0)-get_f(c,RV1)); break;
    case OP_FMUL: set_f(c,RV2,get_f(c,RV0)*get_f(c,RV1)); break;
    case OP_FDIV: set_f(c,RV2,get_f(c,RV0)/get_f(c,RV1)); break;
    case OP_FNEG: set_f(c,RV1,-get_f(c,RV0)); break;

    /* ── ARITHMETIC double ── */
    case OP_DADD: set_d(c,RV2,get_d(c,RV0)+get_d(c,RV1)); break;
    case OP_DSUB: set_d(c,RV2,get_d(c,RV0)-get_d(c,RV1)); break;
    case OP_DMUL: set_d(c,RV2,get_d(c,RV0)*get_d(c,RV1)); break;
    case OP_DDIV: set_d(c,RV2,get_d(c,RV0)/get_d(c,RV1)); break;
    case OP_DNEG: set_d(c,RV1,-get_d(c,RV0)); break;

    /* ── BITWISE ── */
    case OP_AND: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a&b); break; }
    case OP_OR:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a|b); break; }
    case OP_XOR: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; SR(d,a^b); break; }
    case OP_NOT:    SR(RV1,~GR(RV0)); break;
    case OP_SHL: { int32_t sv=GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31); uint8_t d=has_imm?RV1:RV2; SR(d,sv<<n); break; }
    case OP_SHR: { uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31); uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)(sv>>n)); break; }
    case OP_SAR: { int32_t sv=GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31); uint8_t d=has_imm?RV1:RV2; SR(d,sv>>n); break; }
    case OP_ROL: { uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31); uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)((sv<<n)|(sv>>(32-n)))); break; }
    case OP_ROR: { uint32_t sv=(uint32_t)GR(RV0); uint8_t n=has_imm?(uint8_t)(imm&31):(uint8_t)(GR(RV1)&31); uint8_t d=has_imm?RV1:RV2; SR(d,(int32_t)((sv>>n)|(sv<<(32-n)))); break; }
    case OP_BSF: { uint32_t v=(uint32_t)GR(RV0); int idx=0; while(idx<32&&!((v>>idx)&1))idx++; SR(RV1,idx); break; }
    case OP_BSR: { uint32_t v=(uint32_t)GR(RV0); int idx=31; while(idx>=0&&!((v>>idx)&1))idx--; SR(RV1,idx); break; }
    case OP_POPCNT:{ uint32_t v=(uint32_t)GR(RV0); int cnt=0; while(v){cnt+=v&1;v>>=1;} SR(RV1,cnt); break; }
    case OP_LZCNT: { uint32_t v=(uint32_t)GR(RV0); int cnt=0; for(int k=31;k>=0;k--){if(!((v>>k)&1))cnt++;else break;} SR(RV1,cnt); break; }
    case OP_TZCNT: { uint32_t v=(uint32_t)GR(RV0); int cnt=0; while(cnt<32&&!((v>>cnt)&1))cnt++; SR(RV1,cnt); break; }
    case OP_TEST:   set_c(c,RV2,(GR(RV0)&GR(RV1))?1:0); break;
    case OP_XCHG: { int32_t t=GR(RV0); SR(RV0,GR(RV1)); SR(RV1,t); break; }
    case OP_BT:   set_c(c,RV2,((uint32_t)GR(RV0)>>(imm&31))&1); break;
    case OP_BTS:  SR(RV2,GR(RV0)|(1<<(imm&31))); break;
    case OP_BTR:  SR(RV2,GR(RV0)&~(1<<(imm&31))); break;
    case OP_BTC:  SR(RV2,GR(RV0)^(1<<(imm&31))); break;

    /* ── COMPARE ── */
    case OP_CMP:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,(a<b)?-1:(a>b)?1:0); break; }
    case OP_LCMP: { int64_t a=get_l(c,RV0),b=get_l(c,RV1); set_c(c,RV2,(a<b)?-1:(a>b)?1:0); break; }
    case OP_FCMP: { float a=get_f(c,RV0),b=get_f(c,RV1); set_c(c,RV2,(a<b)?-1:(a>b)?1:0); break; }
    case OP_DCMP: { double a=get_d(c,RV0),b=get_d(c,RV1); set_c(c,RV2,(a<b)?-1:(a>b)?1:0); break; }
    case OP_EQ:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a==b?1:0); break; }
    case OP_NE:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a!=b?1:0); break; }
    case OP_GT:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a>b?1:0);  break; }
    case OP_LT:  { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a<b?1:0);  break; }
    case OP_GTE: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a>=b?1:0); break; }
    case OP_LTE: { int32_t a=GR(RV0),b=has_imm?(int32_t)imm:GR(RV1); uint8_t d=has_imm?RV1:RV2; set_c(c,d,a<=b?1:0); break; }

    /* ── CONTROL FLOW ── */
    case OP_JMP:
    case OP_GOTO:  c->pc = label_addr; break;
    case OP_JCC:
    case OP_JE:    if ( get_c(c,RV0))    c->pc=label_addr; break;
    case OP_JNE:   if (!get_c(c,RV0))    c->pc=label_addr; break;
    case OP_JG:    if ( get_c(c,RV0)>0)  c->pc=label_addr; break;
    case OP_JGE:   if ( get_c(c,RV0)>=0) c->pc=label_addr; break;
    case OP_JL:    if ( get_c(c,RV0)<0)  c->pc=label_addr; break;
    case OP_JLE:   if ( get_c(c,RV0)<=0) c->pc=label_addr; break;
    case OP_JA:    if ((uint32_t)get_c(c,RV0)>0)  c->pc=label_addr; break;
    case OP_JB:    if ((uint32_t)get_c(c,RV0)==0)  c->pc=label_addr; break;
    case OP_LOOP:  c->i[0]--; if(c->i[0]!=0) c->pc=label_addr; break;
    case OP_CALL:  push32(c,c->pc); c->pc=label_addr; break;
    case OP_RET:   c->pc=pop32(c); break;
    case OP_RETN:  c->pc=pop32(c); c->sp+=(uint32_t)imm; break;
    case OP_EXIT:  exit((int)imm); break;

    /* ── SYSTEM ── */
    case OP_INT:   abios_handle(c,(uint8_t)imm); break;
    case OP_IRET:  c->pc=pop32(c); c->intf=1; break;
    case OP_CLI:   c->intf=0; break;
    case OP_STI:   c->intf=1; break;
    case OP_CPUID: c->a[0]=CXIS_SIG; c->a[1]=CXIS_VER; c->a[2]=CXIS_FEAT_FLOAT|CXIS_FEAT_DOUBLE|CXIS_FEAT_ABIOS; c->a[3]=0; break;
    case OP_RDTSC: c->l[0]=(int64_t)elapsed_ns(c); break;
    case OP_WAIT:  break;
    case OP_PAUSE: break;
    case OP_UD:
        fprintf(stderr,"cxemu: #UD at pc=0x%08X\n", c->pc-2);
        c->running=0; break;
    case OP_IN:    SR(RV1,0); break;
    case OP_OUT:   break;

    /* ── TYPE CONVERSIONS ── */
    case OP_ITOF: set_f(c,RV1,(float)get_i(c,RV0)); break;
    case OP_ITOD: set_d(c,RV1,(double)get_i(c,RV0)); break;
    case OP_ITOL: set_l(c,RV1,(int64_t)get_i(c,RV0)); break;
    case OP_LTOF: set_f(c,RV1,(float)get_l(c,RV0)); break;
    case OP_LTOD: set_d(c,RV1,(double)get_l(c,RV0)); break;
    case OP_FTOI: set_i(c,RV1,(int32_t)get_f(c,RV0)); break;
    case OP_FTOD: set_d(c,RV1,(double)get_f(c,RV0)); break;
    case OP_FTOL: set_l(c,RV1,(int64_t)get_f(c,RV0)); break;
    case OP_DTOI: set_i(c,RV1,(int32_t)get_d(c,RV0)); break;
    case OP_DTOF: set_f(c,RV1,(float)get_d(c,RV0)); break;
    case OP_DTOL: set_l(c,RV1,(int64_t)get_d(c,RV0)); break;
    case OP_LTOI: set_i(c,RV1,(int32_t)get_l(c,RV0)); break;

    default:
        fprintf(stderr,"cxemu: unknown opcode 0x%02X at pc=0x%08X\n",
                opcode, c->pc-2);
        c->running=0; break;
    }

#undef GR
#undef SR
#undef RV0
#undef RV1
#undef RV2
}

/* ════════════════════════════════════════════════════════════════
   CXE LOADER
════════════════════════════════════════════════════════════════ */

static int load_cxe(CPU *c, const char *path) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"cxemu: cannot open '%s': %s\n",path,strerror(errno)); return 0; }

    CxeHeader hdr;
    if (fread(&hdr,sizeof(hdr),1,f) != 1 || hdr.magic != CXE_MAGIC) {
        fprintf(stderr,"cxemu: '%s' is not a valid .cxe file\n",path);
        fclose(f); return 0;
    }

    for (int i = 0; i < hdr.section_count; i++) {
        CxeSection sec;
        fread(&sec,sizeof(sec),1,f);
        long saved = ftell(f);
        if (sec.vaddr + sec.mem_size > c->ram_size) {
            fprintf(stderr,"cxemu: section %d out of RAM\n",i);
            fclose(f); return 0;
        }
        if (sec.flags & CXE_SEC_ZERO) {
            memset(c->ram + sec.vaddr, 0, sec.mem_size);
        } else {
            fseek(f, sec.offset, SEEK_SET);
            fread(c->ram + sec.vaddr, 1, sec.file_size, f);
            if (sec.mem_size > sec.file_size)
                memset(c->ram + sec.vaddr + sec.file_size, 0, sec.mem_size - sec.file_size);
        }
        fseek(f, saved, SEEK_SET);
    }

    c->pc = hdr.entry_point;
    fclose(f);
    fprintf(stderr,"cxemu: loaded '%s' entry=0x%08X\n", path, c->pc);
    return 1;
}

/* ════════════════════════════════════════════════════════════════
   ABIOS INIT — populate IVT, data area, VRAM, memory map
════════════════════════════════════════════════════════════════ */

static void abios_init(CPU *c) {
    /* Clear IVT (all vectors = 0, handled internally) */
    memset(c->ram + MEM_IVT_BASE, 0, MEM_IVT_SIZE);

    /* ABIOS data area */
    memset(c->ram + MEM_ABIOS_DATA, 0, MEM_ABIOS_DATA_SIZE);
    ada_w32(c, ADA_HEAP_PTR, MEM_HEAP_BASE);
    ada_w8 (c, ADA_ATTR,     VATTR(COL_LGRAY, COL_BLACK));
    ada_w8 (c, ADA_DISK_CNT, (uint8_t)c->disk_count);
    ada_w8 (c, ADA_BOOT_DISK, c->disk_count > 0 ? 0 : 0xFF);

    /* blank VRAM */
    uint8_t attr = VATTR(COL_LGRAY, COL_BLACK);
    for (uint32_t i = 0; i < (uint32_t)(VID_COLS * VID_ROWS); i++) {
        c->ram[MEM_VRAM_BASE + i*2]   = ' ';
        c->ram[MEM_VRAM_BASE + i*2+1] = attr;
    }

    /* write memory map into RAM */
    abios_write_memmap(c);

    /* print ABIOS banner to terminal */
    printf("\033[2J\033[H");   /* clear terminal */
    printf("\033[1;96m");
    printf("ABIOS v1.0 — CXIS Advanced BIOS   [%dMB RAM] [%d disk(s)]\033[0m\n",
           c->ram_size / (1024*1024), c->disk_count);
    printf("\033[90m%s\033[0m\n",
           "────────────────────────────────────────────────────────────────────────────────");
    fflush(stdout);
}

/* ════════════════════════════════════════════════════════════════
   MAIN
════════════════════════════════════════════════════════════════ */

static void usage(void) {
    fprintf(stderr,
        "cxemu — CXIS Machine Emulator + ABIOS\n"
        "Usage:  cxemu [options] <kernel.cxe> [disk0.img [disk1.img ...]]\n"
        "Options:\n"
        "  --trace      print pc/regs every instruction\n"
        "  --help       this message\n"
        "\n"
        "ABIOS interrupt vectors:\n"
        "  int 0x01  console   int 0x02  keyboard  int 0x03  video\n"
        "  int 0x04  disk      int 0x05  memory    int 0x06  timer\n"
        "  int 0x07  power     int 0x08  sysinfo   int 0x09  IRQ\n"
        "\n"
        "Memory map (64MB):\n"
        "  0x00000000  IVT (1KB)          0x00001000  ABIOS data\n"
        "  0x00002000  code/text          0x00C00000  VRAM 80x25\n"
        "  0x01000000  kernel load area   0x02000000  heap\n"
        "  0x03F00000  stack top\n"
    );
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(); return 1; }

    int trace = 0;
    const char *kernel = NULL;

    CPU *cpu = calloc(1, sizeof(CPU));
    cpu->ram      = calloc(1, MEM_TOTAL);
    cpu->ram_size = MEM_TOTAL;
    cpu->sp       = MEM_STACK_TOP;
    cpu->sf       = MEM_STACK_TOP;
    cpu->bp       = MEM_TEXT_BASE;
    cpu->bf       = 0;
    cpu->intf     = 1;
    cpu->boot_ns  = now_ns();

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--trace") == 0) { trace = 1; continue; }
        if (strcmp(argv[i], "--help")  == 0) { usage(); return 0; }
        if (!kernel) { kernel = argv[i]; continue; }
        /* rest are disk images */
        if (cpu->disk_count >= DISK_MAX_DRIVES) {
            fprintf(stderr,"cxemu: max %d disks\n", DISK_MAX_DRIVES);
            continue;
        }
        int id = cpu->disk_count++;
        cpu->disks[id].fp = fopen(argv[i], "r+b");
        if (!cpu->disks[id].fp) {
            fprintf(stderr,"cxemu: disk '%s': %s\n", argv[i], strerror(errno));
            cpu->disk_count--;
            continue;
        }
        fseek(cpu->disks[id].fp, 0, SEEK_END);
        long sz = ftell(cpu->disks[id].fp);
        cpu->disks[id].sector_count = (uint32_t)(sz / DISK_SECTOR_SIZE);
        strncpy(cpu->disks[id].path, argv[i], 255);
        fprintf(stderr,"cxemu: disk%d '%s' (%u sectors)\n",
                id, argv[i], cpu->disks[id].sector_count);
    }

    if (!kernel) { fprintf(stderr,"cxemu: no kernel specified\n"); usage(); return 1; }

    abios_init(cpu);
    if (!load_cxe(cpu, kernel)) { return 1; }

    cpu->trace   = trace;
    cpu->running = 1;

    while (cpu->running && !cpu->halted) {
        if (trace)
            fprintf(stderr, "  pc=%08X sp=%08X i0=%d i1=%d a0=%u\n",
                    cpu->pc, cpu->sp, cpu->i[0], cpu->i[1], (uint32_t)cpu->a[0]);
        step(cpu);
        check_timers(cpu);
    }

    fprintf(stderr,"\ncxemu: halted  steps=%lld  uptime=%llus\n",
            cpu->steps, (unsigned long long)(elapsed_ns(cpu)/1000000000ULL));

    for (int i = 0; i < cpu->disk_count; i++)
        if (cpu->disks[i].fp) fclose(cpu->disks[i].fp);

    free(cpu->ram);
    free(cpu);
    return 0;
}
