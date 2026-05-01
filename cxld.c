#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "include/cxis.h"
#include "include/cxo.h"
#include "include/cxe.h"

/* ════════════════════════════════════════════════════════════════
   LOADED OBJECT
════════════════════════════════════════════════════════════════ */

typedef struct {
    CxoHeader  hdr;
    CxoSection sects[16];
    CxoSymbol  syms[4096];
    CxoReloc   relocs[8192];
    char      *strtab;
    uint8_t   *sectdata[16];   /* raw section data */
    uint32_t   sect_vaddr[16]; /* assigned virtual address */
    const char *path;
} ObjFile;

static ObjFile *load_obj(const char *path) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"ld: cannot open '%s': %s\n",path,strerror(errno)); return NULL; }

    ObjFile *obj = calloc(1,sizeof(ObjFile));
    obj->path = path;

    fread(&obj->hdr, sizeof(CxoHeader), 1, f);
    if (obj->hdr.magic != CXO_MAGIC) {
        fprintf(stderr,"ld: '%s' is not a valid .cxo file\n",path);
        free(obj); fclose(f); return NULL;
    }
    if (obj->hdr.section_count > 16 || obj->hdr.sym_count > 4096 || obj->hdr.rel_count > 8192) {
        fprintf(stderr,"ld: '%s' has too many sections/symbols/relocs\n",path);
        free(obj); fclose(f); return NULL;
    }

    fread(obj->sects,  sizeof(CxoSection), obj->hdr.section_count, f);
    fread(obj->syms,   sizeof(CxoSymbol),  obj->hdr.sym_count,     f);
    fread(obj->relocs, sizeof(CxoReloc),   obj->hdr.rel_count,     f);

    /* read string table — it follows relocs, occupies space up to first section data */
    uint32_t strtab_start = sizeof(CxoHeader)
                          + obj->hdr.section_count * sizeof(CxoSection)
                          + obj->hdr.sym_count     * sizeof(CxoSymbol)
                          + obj->hdr.rel_count     * sizeof(CxoReloc);
    uint32_t data_start = obj->hdr.section_count > 0 ? obj->sects[0].offset : strtab_start;
    uint32_t strtab_len = data_start - strtab_start;
    obj->strtab = malloc(strtab_len + 1);
    fseek(f, strtab_start, SEEK_SET);
    fread(obj->strtab, 1, strtab_len, f);
    obj->strtab[strtab_len] = 0;

    /* read section data */
    for (int i = 0; i < obj->hdr.section_count; i++) {
        CxoSection *s = &obj->sects[i];
        if (s->type == CXO_SEC_BSS || s->size == 0) { obj->sectdata[i] = NULL; continue; }
        obj->sectdata[i] = malloc(s->size);
        fseek(f, s->offset, SEEK_SET);
        fread(obj->sectdata[i], 1, s->size, f);
    }

    fclose(f); return obj;
}

static void free_obj(ObjFile *obj) {
    for (int i = 0; i < obj->hdr.section_count; i++) free(obj->sectdata[i]);
    free(obj->strtab); free(obj);
}

/* ════════════════════════════════════════════════════════════════
   LINKER
════════════════════════════════════════════════════════════════ */

#define MAX_OBJS 64

typedef struct {
    const char *name;
    uint32_t    vaddr;
    int         obj_idx;
    int         sym_idx;
} GlobalSym;

typedef struct {
    ObjFile   *objs[MAX_OBJS];
    int        nobj;
    GlobalSym  gsyms[MAX_OBJS * 4096];
    int        ngsym;
    int        errors;
    uint32_t   base_addr;   /* 0 = use VM_TEXT_BASE default */
} Linker;

static GlobalSym *find_gsym(Linker *ld, const char *name) {
    for (int i = 0; i < ld->ngsym; i++)
        if (strcmp(ld->gsyms[i].name, name)==0) return &ld->gsyms[i];
    return NULL;
}

static void collect_symbols(Linker *ld) {
    for (int oi = 0; oi < ld->nobj; oi++) {
        ObjFile *obj = ld->objs[oi];
        for (int si = 0; si < obj->hdr.sym_count; si++) {
            CxoSymbol *sym = &obj->syms[si];
            if (!(sym->flags & CXO_SYM_GLOBAL)) continue;
            if (sym->flags & CXO_SYM_EXTERN)    continue;
            const char *name = obj->strtab + sym->name_off;
            if (find_gsym(ld, name)) {
                fprintf(stderr,"ld: duplicate symbol '%s'\n", name);
                ld->errors++;
            } else {
                GlobalSym *g = &ld->gsyms[ld->ngsym++];
                g->name    = name;
                g->obj_idx = oi;
                g->sym_idx = si;
                g->vaddr   = 0; /* resolved later */
            }
        }
    }
}

/* assign virtual addresses to sections */
static void assign_vaddrs(Linker *ld) {
    /* layout: text | rodata | data | bss */
    uint8_t order[] = {CXO_SEC_TEXT, CXO_SEC_RODATA, CXO_SEC_DATA, CXO_SEC_BSS};
    uint32_t vaddr = ld->base_addr ? ld->base_addr : VM_TEXT_BASE;

    for (int ot = 0; ot < 4; ot++) {
        for (int oi = 0; oi < ld->nobj; oi++) {
            ObjFile *obj = ld->objs[oi];
            for (int si = 0; si < obj->hdr.section_count; si++) {
                if (obj->sects[si].type != order[ot]) continue;
                /* align to 4 */
                vaddr = (vaddr + 3) & ~3;
                obj->sect_vaddr[si] = vaddr;
                vaddr += obj->sects[si].size;
            }
        }
        /* page-align between segments */
        vaddr = (vaddr + 3) & ~3;
    }

    /* now resolve global symbol vaddrs */
    for (int i = 0; i < ld->ngsym; i++) {
        GlobalSym *g = &ld->gsyms[i];
        ObjFile   *obj = ld->objs[g->obj_idx];
        CxoSymbol *sym = &obj->syms[g->sym_idx];
        if (sym->section == 0xFF) { g->vaddr = 0; continue; }
        g->vaddr = obj->sect_vaddr[sym->section] + sym->offset;
    }
}

static void apply_relocs(Linker *ld) {
    for (int oi = 0; oi < ld->nobj; oi++) {
        ObjFile *obj = ld->objs[oi];
        for (int ri = 0; ri < obj->hdr.rel_count; ri++) {
            CxoReloc  *rel = &obj->relocs[ri];
            CxoSymbol *sym = &obj->syms[rel->sym_index];
            const char *sname = obj->strtab + sym->name_off;
            GlobalSym *g = find_gsym(ld, sname);

            uint32_t target = 0;
            if (g) {
                target = g->vaddr;
            } else if (!(sym->flags & CXO_SYM_EXTERN) && sym->section != 0xFF) {
                /* local symbol resolved within object */
                target = obj->sect_vaddr[sym->section] + sym->offset;
            } else {
                fprintf(stderr,"ld: undefined symbol '%s'\n", sname);
                ld->errors++; continue;
            }

            uint8_t *sec_data = obj->sectdata[rel->section];
            if (!sec_data) continue;
            uint8_t *patch = sec_data + rel->offset;

            uint32_t patch_vaddr = obj->sect_vaddr[rel->section] + rel->offset;
            uint32_t value;
            if (rel->type == CXO_REL_REL32)
                value = target - (patch_vaddr + 4);
            else
                value = target;

            patch[0] = (value      ) & 0xFF;
            patch[1] = (value >>  8) & 0xFF;
            patch[2] = (value >> 16) & 0xFF;
            patch[3] = (value >> 24) & 0xFF;
        }
    }
}

static int write_cxe(Linker *ld, const char *outpath, const char *entry_sym) {
    /* find entry point */
    GlobalSym *entry = find_gsym(ld, entry_sym);
    if (!entry) {
        fprintf(stderr,"ld: entry point '%s' not found\n", entry_sym);
        ld->errors++; return 0;
    }

    FILE *f = fopen(outpath,"wb");
    if (!f) { fprintf(stderr,"ld: cannot write '%s': %s\n",outpath,strerror(errno)); return 0; }

    /* count output sections (one per type with data) */
    uint8_t order[] = {CXO_SEC_TEXT, CXO_SEC_RODATA, CXO_SEC_DATA, CXO_SEC_BSS};
    const char *onames[] = {"text","rodata","data","bss"};

    /* merge sections of same type across objects */
    typedef struct { uint32_t vaddr; uint32_t total_size; uint8_t type; uint8_t flags; } OutSect;
    OutSect outsects[4]; int nout = 0;
    uint32_t sect_file_off[4];

    for (int ot = 0; ot < 4; ot++) {
        uint32_t total = 0, first_vaddr = 0; int found = 0;
        for (int oi = 0; oi < ld->nobj; oi++) {
            ObjFile *obj = ld->objs[oi];
            for (int si = 0; si < obj->hdr.section_count; si++) {
                if (obj->sects[si].type != order[ot]) continue;
                if (!found) { first_vaddr = obj->sect_vaddr[si]; found = 1; }
                total += obj->sects[si].size;
            }
        }
        if (!found) continue;
        outsects[nout].vaddr = first_vaddr;
        outsects[nout].total_size = total;
        outsects[nout].type = order[ot];
        outsects[nout].flags = (order[ot]==CXO_SEC_TEXT) ? (CXE_SEC_READ|CXE_SEC_EXEC) :
                               (order[ot]==CXO_SEC_DATA) ? (CXE_SEC_READ|CXE_SEC_WRITE) :
                               (order[ot]==CXO_SEC_BSS)  ? (CXE_SEC_READ|CXE_SEC_WRITE|CXE_SEC_ZERO) :
                                                            CXE_SEC_READ;
        nout++;
    }

    /* compute file offsets — simulate same 4-byte alignment used during write */
    uint32_t hdr_off  = sizeof(CxeHeader) + nout * sizeof(CxeSection);
    uint32_t cur_off  = hdr_off;
    for (int i = 0; i < nout; i++) {
        if (outsects[i].flags & CXE_SEC_ZERO) { sect_file_off[i] = 0; continue; }
        /* simulate per-subsection 4-byte alignment */
        uint32_t sim = cur_off;
        uint8_t ord = outsects[i].type;
        sect_file_off[i] = sim; /* will be updated by simulation below */
        for (int oi2 = 0; oi2 < ld->nobj; oi2++) {
            ObjFile *obj2 = ld->objs[oi2];
            for (int si2 = 0; si2 < obj2->hdr.section_count; si2++) {
                if (obj2->sects[si2].type != ord) continue;
                if (!obj2->sectdata[si2] || obj2->sects[si2].size == 0) continue;
                sim = (sim + 3) & ~3;
                if (oi2 == 0 && si2 == 0) sect_file_off[i] = sim; /* first sub-section */
                /* track first sub-section start */
                int is_first = 1;
                for (int x=0;x<oi2;x++) { for(int y=0;y<ld->objs[x]->hdr.section_count;y++) { if(ld->objs[x]->sects[y].type==ord && ld->objs[x]->sectdata[y] && ld->objs[x]->sects[y].size>0){is_first=0;break;} } if(!is_first)break; }
                if (is_first) { int found2=0; for(int y=0;y<si2;y++){if(obj2->sects[y].type==ord&&obj2->sectdata[y]&&obj2->sects[y].size>0){found2=1;break;}} if(!found2) sect_file_off[i]=sim; }
                sim += obj2->sects[si2].size;
            }
        }
        cur_off = sim;
    }

    /* write header */
    CxeHeader hdr = {0};
    hdr.magic         = CXE_MAGIC;
    hdr.version       = CXE_VERSION;
    hdr.entry_point   = entry->vaddr;
    hdr.section_count = nout;
    fwrite(&hdr, sizeof(hdr), 1, f);

    /* write section headers (offsets filled in after data is written) */
    long sect_hdr_pos = ftell(f);
    for (int i = 0; i < nout; i++) {
        CxeSection cs = {0};
        cs.vaddr     = outsects[i].vaddr;
        cs.offset    = 0; /* placeholder */
        cs.file_size = (outsects[i].flags & CXE_SEC_ZERO) ? 0 : outsects[i].total_size;
        cs.mem_size  = outsects[i].total_size;
        cs.flags     = outsects[i].flags;
        fwrite(&cs, sizeof(cs), 1, f);
    }

    /* write section data tracking actual file offsets */
    uint32_t actual_off[4];
    for (int i = 0; i < 4; i++) actual_off[i] = 0;
    for (int ot = 0; ot < 4; ot++) {
        if (order[ot] == CXO_SEC_BSS) continue;
        int type_idx = -1;
        for (int i = 0; i < nout; i++) if (outsects[i].type == order[ot]) { type_idx=i; break; }
        if (type_idx < 0) continue;
        int first = 1;
        for (int oi = 0; oi < ld->nobj; oi++) {
            ObjFile *obj = ld->objs[oi];
            for (int si = 0; si < obj->hdr.section_count; si++) {
                if (obj->sects[si].type != order[ot]) continue;
                if (!obj->sectdata[si] || obj->sects[si].size == 0) continue;
                long pos = ftell(f);
                while (pos % 4) { fputc(0,f); pos++; }
                if (first) { actual_off[type_idx] = (uint32_t)ftell(f); first = 0; }
                fwrite(obj->sectdata[si], 1, obj->sects[si].size, f);
            }
        }
    }
    /* seek back and fix section header offsets */
    fseek(f, sect_hdr_pos, SEEK_SET);
    for (int i = 0; i < nout; i++) {
        CxeSection cs = {0};
        cs.vaddr     = outsects[i].vaddr;
        cs.offset    = (outsects[i].flags & CXE_SEC_ZERO) ? 0 : actual_off[i];
        cs.file_size = (outsects[i].flags & CXE_SEC_ZERO) ? 0 : outsects[i].total_size;
        cs.mem_size  = outsects[i].total_size;
        cs.flags     = outsects[i].flags;
        fwrite(&cs, sizeof(cs), 1, f);
    }
    fseek(f, 0, SEEK_END);

    fclose(f);
    printf("linked '%s'  entry=0x%08X  %d section(s)\n", outpath, entry->vaddr, nout);
    return 1;
}

/* ════════════════════════════════════════════════════════════════
   MAIN
════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,"usage: cxld <file1.cxo> [file2.cxo ...] [-o output.cxe] [-e entry]\n");
        return 1;
    }

    Linker ld; memset(&ld,0,sizeof(ld));
    const char *outfile   = "a.cxe";
    const char *entry_sym = "_start";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i],"-o")==0 && i+1<argc) { outfile = argv[++i]; continue; }
        if (strcmp(argv[i],"-e")==0 && i+1<argc) { entry_sym = argv[++i]; continue; }
        if (strcmp(argv[i],"-b")==0 && i+1<argc) { ld.base_addr=(uint32_t)strtoul(argv[++i],NULL,0); continue; }
        if (ld.nobj >= MAX_OBJS) { fprintf(stderr,"ld: too many input files\n"); return 1; }
        ObjFile *obj = load_obj(argv[i]);
        if (!obj) return 1;
        ld.objs[ld.nobj++] = obj;
    }

    collect_symbols(&ld);
    assign_vaddrs(&ld);
    apply_relocs(&ld);

    if (ld.errors) { fprintf(stderr,"ld: %d error(s)\n", ld.errors); return 1; }

    if (!write_cxe(&ld, outfile, entry_sym)) return 1;

    for (int i = 0; i < ld.nobj; i++) free_obj(ld.objs[i]);
    return 0;
}
