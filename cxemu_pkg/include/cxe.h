#ifndef CXE_H
#define CXE_H

#include <stdint.h>

/* ── .cxe executable format ──────────────────────────────────────
   magic(4) version(2) flags(2)
   entry_point(4)       — virtual address of _start
   section_count(2)
   reserved(6)
   [section_headers...]
   [section_data...]
────────────────────────────────────────────────────────────────── */

#define CXE_MAGIC   0x45585843   /* "CXEX" little-endian */
#define CXE_VERSION 0x0101

/* section flags */
#define CXE_SEC_READ    0x01
#define CXE_SEC_WRITE   0x02
#define CXE_SEC_EXEC    0x04
#define CXE_SEC_ZERO    0x08   /* bss: zero-fill, no data in file */

#pragma pack(push, 1)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t entry_point;
    uint16_t section_count;
    uint8_t  reserved[6];
} CxeHeader;

typedef struct {
    uint32_t vaddr;         /* virtual load address */
    uint32_t offset;        /* file offset to data (0 if zero-fill) */
    uint32_t file_size;     /* bytes in file       */
    uint32_t mem_size;      /* bytes in memory (>= file_size for bss) */
    uint8_t  flags;         /* CXE_SEC_* */
    uint8_t  reserved[3];
} CxeSection;

#pragma pack(pop)

#endif /* CXE_H */
