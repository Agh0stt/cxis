#ifndef ABIOS_H
#define ABIOS_H

#include <stdint.h>

/* ================================================================
   ABIOS — Advanced BIOS for CXIS  (SeaBIOS equivalent)
   Firmware services layer for OS development on cxemu.

   OS calls ABIOS via:   int <vector>
   Arguments in a0–a9. Return values in a0 (a1 for wide results).
   a0 == 0xFFFFFFFF signals error unless noted.
   Sub-function selected by a0 for multi-function vectors.
================================================================ */

/* ── memory map (64 MB) ──────────────────────────────────────── */
#define MEM_TOTAL            (64u * 1024 * 1024)

#define MEM_IVT_BASE         0x00000000u  /* IVT: 256×4 = 1 KB         */
#define MEM_IVT_SIZE         0x00001000u
#define MEM_ABIOS_DATA       0x00001000u  /* ABIOS data area   4 KB     */
#define MEM_ABIOS_DATA_SIZE  0x00001000u
#define MEM_TEXT_BASE        0x00002000u  /* code start                 */
#define MEM_VRAM_BASE        0x00C00000u  /* text VRAM 80×25×2  4 KB   */
#define MEM_VRAM_SIZE        0x00001000u
#define MEM_KBUF_BASE        0x00C01000u  /* keyboard ring     256 B    */
#define MEM_KBUF_SIZE        0x00000100u
#define MEM_DISK_CACHE_BASE  0x00C02000u  /* disk cache        16 KB    */
#define MEM_DISK_CACHE_SIZE  0x00004000u
#define MEM_ABIOS_STR        0x00C06000u  /* ABIOS string table  2 KB   */
#define MEM_KERNEL_BASE      0x01000000u  /* kernel load area  16 MB    */
#define MEM_HEAP_BASE        0x02000000u  /* heap                        */
#define MEM_STACK_TOP        0x03F00000u  /* stack top                   */

/* ── IVT: each entry = 4 bytes (uint32_t ISR address, 0=unset) ─
   Vectors 0x00–0x0F: ABIOS internal.
   Vectors 0x10–0xFF: OS-installable.                            */
#define IVT_ENTRY(n)   (MEM_IVT_BASE + (uint32_t)(n) * 4)

/* ── ABIOS data area (at MEM_ABIOS_DATA) ────────────────────── */
#define ADA_TICK_LO    0x00   /* uint32_t tick lo (ms)           */
#define ADA_TICK_HI    0x04   /* uint32_t tick hi                */
#define ADA_KHEAD      0x08   /* uint8_t  keyboard ring head     */
#define ADA_KTAIL      0x09   /* uint8_t  keyboard ring tail     */
#define ADA_KCOUNT     0x0A   /* uint8_t  keyboard pending count */
#define ADA_CUR_X      0x0C   /* uint8_t  text cursor col        */
#define ADA_CUR_Y      0x0D   /* uint8_t  text cursor row        */
#define ADA_ATTR       0x0E   /* uint8_t  current text attribute */
#define ADA_DISK_CNT   0x0F   /* uint8_t  number of disks        */
#define ADA_HEAP_PTR   0x10   /* uint32_t heap bump pointer      */
#define ADA_BOOT_DISK  0x14   /* uint8_t  boot disk id           */

/* ── interrupt vector numbers ───────────────────────────────── */
#define VEC_CONSOLE    0x01
#define VEC_KEYBOARD   0x02
#define VEC_VIDEO      0x03
#define VEC_DISK       0x04
#define VEC_MEMORY     0x05
#define VEC_TIMER      0x06
#define VEC_POWER      0x07
#define VEC_SYSINFO    0x08
#define VEC_IRQ        0x09

/* ── VEC_CONSOLE (a0 = sub-function) ───────────────────────── */
#define CON_PUTCHAR    0x00   /* a1=char                         */
#define CON_WRITE      0x01   /* a1=ptr a2=len                   */
#define CON_PUTS       0x02   /* a1=null-terminated ptr          */
#define CON_GETCHAR    0x03   /* → a0=char (blocking)            */
#define CON_READLINE   0x04   /* a1=buf a2=maxlen → a0=len       */
#define CON_PRINT_INT  0x05   /* a1=i32 decimal                  */
#define CON_PRINT_HEX  0x06   /* a1=u32 hex 8 digits             */
#define CON_EXIT       0xFF   /* a1=exit_code                    */

/* ── VEC_KEYBOARD ───────────────────────────────────────────── */
#define KB_READ        0x00   /* → a0=scancode (blocking)        */
#define KB_POLL        0x01   /* → a0=scancode or 0 (non-block)  */
#define KB_FLUSH       0x02   /* clear ring                      */
#define KB_AVAILABLE   0x03   /* → a0=pending count              */

/* ── VEC_VIDEO — text mode 80×25 ────────────────────────────
   VRAM: cell = [char byte][attr byte]  attr=[bg4|fg4]
   Colors: 0=black 1=blue 2=green 3=cyan 4=red 5=mag 6=brown
           7=lgray 8=dgray 9=lblue A=lgreen B=lcyan C=lred
           D=lmag E=yellow F=white                              */
#define VID_COLS       80
#define VID_ROWS       25

#define VID_CLEAR      0x00   /* a1=attr                         */
#define VID_PUTCHAR    0x01   /* a1=char a2=col a3=row a4=attr   */
#define VID_PUTS       0x02   /* a1=ptr  a2=col a3=row a4=attr   */
#define VID_SCROLL_UP  0x03   /* a1=lines a2=attr(fill)          */
#define VID_SET_CURSOR 0x04   /* a1=col a2=row                   */
#define VID_GET_CURSOR 0x05   /* → a0=col a1=row                 */
#define VID_SET_ATTR   0x06   /* a1=attr (set default)           */
#define VID_GET_CHAR   0x07   /* a1=col a2=row → a0=char a1=attr */
#define VID_VRAM_ADDR  0x08   /* → a0=MEM_VRAM_BASE              */
#define VID_DIMENSIONS 0x09   /* → a0=cols a1=rows               */
#define VID_WRITE_RAW  0x0A   /* a1=col a2=row a3=char a4=attr   */

/* ── VEC_DISK ───────────────────────────────────────────────── */
/* a1=disk_id  a2=LBA  a3=buf_ptr  a4=sector_count             */
#define DISK_READ      0x00   /* → a0=0 ok, 1=err                */
#define DISK_WRITE     0x01   /* → a0=0 ok, 1=err                */
#define DISK_SECTORS   0x02   /* a1=disk_id → a0=sector count    */
#define DISK_COUNT     0x03   /* → a0=disk count                 */
#define DISK_SECTOR_SZ 0x04   /* → a0=512                        */
#define DISK_FLUSH     0x05   /* flush write-through cache       */

/* ── VEC_MEMORY ─────────────────────────────────────────────── */
#define MEM_ALLOC      0x00   /* a1=bytes → a0=ptr (0=fail)      */
#define MEM_FREE       0x01   /* a1=ptr                          */
#define MEM_TOTAL_Q    0x02   /* → a0=total RAM bytes            */
#define MEM_FREE_Q     0x03   /* → a0=approx free bytes          */
#define MEM_MAP_Q      0x04   /* → a0=ptr to AbiosMemMap         */
#define MEM_COPY       0x05   /* a1=dst a2=src a3=len            */
#define MEM_SET        0x06   /* a1=dst a2=byte a3=len           */
#define MEM_COMPARE    0x07   /* a1=p1 a2=p2 a3=len → a0=cmp    */

/* ── VEC_TIMER ──────────────────────────────────────────────── */
#define TIMER_TICKS_NS 0x00   /* → l0=nanoseconds since boot     */
#define TIMER_TICKS_MS 0x01   /* → a0=milliseconds since boot    */
#define TIMER_SLEEP_MS 0x02   /* a1=ms (busy-wait)               */
#define TIMER_UPTIME   0x03   /* → a0=seconds since boot         */
#define TIMER_INSTALL  0x04   /* a1=period_ms a2=isr_addr        */
#define TIMER_REMOVE   0x05

/* ── VEC_POWER ──────────────────────────────────────────────── */
#define PWR_SHUTDOWN   0x00   /* a1=exit_code                    */
#define PWR_REBOOT     0x01   /* soft reboot (re-runs ABIOS)     */
#define PWR_HALT       0x02   /* halt CPU until next interrupt   */

/* ── VEC_SYSINFO ────────────────────────────────────────────── */
#define SI_CPUID       0x00   /* → a0=sig a1=ver a2=features     */
#define SI_MEMSIZE     0x01   /* → a0=total RAM bytes            */
#define SI_DISK_COUNT  0x02   /* → a0=disk count                 */
#define SI_BIOS_VER    0x03   /* → a0=ptr to version cstring     */
#define SI_BOOT_DISK   0x04   /* → a0=boot disk id               */
#define SI_MEM_MAP     0x05   /* → a0=ptr to map a1=entry count  */

/* ── VEC_IRQ ────────────────────────────────────────────────── */
#define IRQ_INSTALL    0x00   /* a1=vector a2=isr_addr           */
#define IRQ_REMOVE     0x01   /* a1=vector                       */
#define IRQ_ENABLE     0x02
#define IRQ_DISABLE    0x03
#define IRQ_FIRE       0x04   /* a1=vector (software trigger)    */

/* ── memory map entry ───────────────────────────────────────── */
typedef struct {
    uint32_t base;
    uint32_t size;
    uint8_t  type;
    uint8_t  pad[3];
} AbiosMemEntry;

#define ABIOS_MEM_FREE     1
#define ABIOS_MEM_RESERVED 2
#define ABIOS_MEM_FIRMWARE 3
#define ABIOS_MEM_VRAM     4
#define ABIOS_MEM_KERNEL   5

/* ── video attribute macro ──────────────────────────────────── */
#define VATTR(fg,bg)   (uint8_t)(((bg)<<4)|(fg))
#define COL_BLACK   0
#define COL_BLUE    1
#define COL_GREEN   2
#define COL_CYAN    3
#define COL_RED     4
#define COL_MAGENTA 5
#define COL_BROWN   6
#define COL_LGRAY   7
#define COL_DGRAY   8
#define COL_YELLOW  14
#define COL_WHITE   15

/* ── CPUID ──────────────────────────────────────────────────── */
#define CXIS_SIG          0x43584953u  /* "CXIS" */
#define CXIS_VER          0x00010001u  /* 1.1    */
#define CXIS_FEAT_FLOAT   0x00000001u
#define CXIS_FEAT_DOUBLE  0x00000002u
#define CXIS_FEAT_ABIOS   0x00000004u

/* ── disk sector size ───────────────────────────────────────── */
#define DISK_SECTOR_SIZE   512u
#define DISK_MAX_DRIVES    4

#endif /* ABIOS_H */

/* ── New ABIOS vectors for OS code (avoids legacy cxvm conflict) ─
   Old cxvm binaries use int 0x01–0x07. OS code should use these: */
#define ABIOS_VEC_CONSOLE   0x10   /* same sub-fns as VEC_CONSOLE  */
#define ABIOS_VEC_KEYBOARD  0x11
#define ABIOS_VEC_VIDEO     0x12
#define ABIOS_VEC_DISK      0x13
#define ABIOS_VEC_MEMORY    0x14
#define ABIOS_VEC_TIMER     0x15
#define ABIOS_VEC_POWER     0x16
#define ABIOS_VEC_SYSINFO   0x17
#define ABIOS_VEC_IRQ       0x18
