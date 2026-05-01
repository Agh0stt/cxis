#ifndef CXO_H
#define CXO_H

#include <stdint.h>

/* ── .cxo object file format ─────────────────────────────────────
   magic(4) version(2) flags(2) section_count(2) sym_count(2)
   rel_count(2) reserved(2)
   [section_headers...]
   [symbol_table...]
   [relocation_table...]
   [section_data...]
────────────────────────────────────────────────────────────────── */

#define CXO_MAGIC   0x4F585843   /* "CXIO" little-endian */
#define CXO_VERSION 0x0101       /* v1.1 */

/* section types */
#define CXO_SEC_TEXT   0x01
#define CXO_SEC_DATA   0x02
#define CXO_SEC_RODATA 0x03
#define CXO_SEC_BSS    0x04

/* symbol flags */
#define CXO_SYM_GLOBAL 0x01
#define CXO_SYM_LOCAL  0x02
#define CXO_SYM_EXTERN 0x04

/* relocation types */
#define CXO_REL_ABS32  0x01   /* absolute 32-bit address */
#define CXO_REL_REL32  0x02   /* relative 32-bit offset  */

#pragma pack(push, 1)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint16_t section_count;
    uint16_t sym_count;
    uint16_t rel_count;
    uint16_t reserved;
} CxoHeader;

typedef struct {
    uint8_t  type;          /* CXO_SEC_* */
    uint8_t  flags;
    uint16_t name_off;      /* offset into string table */
    uint32_t offset;        /* offset in file to section data */
    uint32_t size;          /* size in bytes */
    uint32_t load_addr;     /* preferred load address (0=relocatable) */
    uint32_t align;         /* alignment requirement */
} CxoSection;

typedef struct {
    uint16_t name_off;      /* offset into string table */
    uint8_t  section;       /* section index (0xFF = extern) */
    uint8_t  flags;         /* CXO_SYM_* */
    uint32_t offset;        /* offset within section */
    uint32_t size;
} CxoSymbol;

typedef struct {
    uint32_t offset;        /* offset in section where reloc applies */
    uint16_t sym_index;     /* index into symbol table */
    uint8_t  type;          /* CXO_REL_* */
    uint8_t  section;       /* which section this reloc is in */
} CxoReloc;

#pragma pack(pop)

#endif /* CXO_H */
