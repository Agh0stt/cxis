#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>

#include "include/cxis.h"
#include "include/cxo.h"

/* ════════════════════════════════════════════════════════════════
   LEXER
════════════════════════════════════════════════════════════════ */

typedef enum {
    TK_IDENT, TK_NUMBER, TK_FLOAT, TK_STRING,
    TK_COMMA, TK_COLON, TK_LBRACKET, TK_RBRACKET,
    TK_PLUS, TK_MINUS, TK_STAR,
    TK_NEWLINE, TK_EOF, TK_ERROR
} TokKind;

typedef struct {
    TokKind  kind;
    char     text[256];
    int64_t  ival;
    double   fval;
    int      line;
} Token;

typedef struct {
    const char *src;
    int         pos;
    int         line;
    Token       cur;
} Lexer;

static void lex_init(Lexer *l, const char *src) {
    l->src = src; l->pos = 0; l->line = 1;
}

static char lc(Lexer *l) { return l->src[l->pos]; }
static char la(Lexer *l) { return l->src[l->pos+1]; }
static char lnext(Lexer *l) { return l->src[l->pos++]; }

static void lex_skip_ws(Lexer *l) {
    while (lc(l) == ' ' || lc(l) == '\t' || lc(l) == '\r') lnext(l);
    if (lc(l) == ';') while (lc(l) && lc(l) != '\n') lnext(l);
}

static Token lex_next(Lexer *l) {
    Token t; t.line = l->line;
    lex_skip_ws(l);
    char c = lc(l);

    if (!c) { t.kind = TK_EOF; strcpy(t.text,"<eof>"); return t; }
    if (c == '\n') { lnext(l); l->line++; t.kind = TK_NEWLINE; strcpy(t.text,"\\n"); return t; }
    if (c == ',')  { lnext(l); t.kind = TK_COMMA;    strcpy(t.text,",");  return t; }
    if (c == ':')  { lnext(l); t.kind = TK_COLON,    strcpy(t.text,":");  return t; }
    if (c == '[')  { lnext(l); t.kind = TK_LBRACKET; strcpy(t.text,"[");  return t; }
    if (c == ']')  { lnext(l); t.kind = TK_RBRACKET; strcpy(t.text,"]");  return t; }
    if (c == '+')  { lnext(l); t.kind = TK_PLUS;     strcpy(t.text,"+");  return t; }
    if (c == '-')  { lnext(l); t.kind = TK_MINUS;    strcpy(t.text,"-");  return t; }
    if (c == '*')  { lnext(l); t.kind = TK_STAR;     strcpy(t.text,"*");  return t; }

    if (c == '"') {
        lnext(l); int i = 0;
        while (lc(l) && lc(l) != '"') {
            if (lc(l) == '\\') {
                lnext(l);
                char esc = lnext(l);
                switch(esc) {
                    case 'n': t.text[i++]='\n'; break;
                    case 't': t.text[i++]='\t'; break;
                    case '0': t.text[i++]='\0'; break;
                    default:  t.text[i++]=esc;  break;
                }
            } else t.text[i++] = lnext(l);
        }
        if (lc(l)=='"') lnext(l);
        t.text[i]=0; t.kind=TK_STRING; t.ival=i; return t;
    }

    if (isdigit(c) || (c=='0' && la(l)=='x')) {
        int i = 0;
        if (c=='0' && la(l)=='x') { t.text[i++]=lnext(l); t.text[i++]=lnext(l); }
        while (isxdigit(lc(l)) || lc(l)=='.') t.text[i++]=lnext(l);
        t.text[i]=0;
        if (strchr(t.text,'.')) {
            t.fval = atof(t.text); t.kind = TK_FLOAT;
        } else {
            t.ival = (int64_t)strtoll(t.text,NULL,0); t.kind = TK_NUMBER;
        }
        return t;
    }

    if (isalpha(c) || c=='_') {
        int i = 0;
        while (isalnum(lc(l)) || lc(l)=='_') t.text[i++]=lnext(l);
        t.text[i]=0; t.kind=TK_IDENT; return t;
    }

    t.kind = TK_ERROR; t.text[0]=lnext(l); t.text[1]=0; return t;
}

/* ════════════════════════════════════════════════════════════════
   REGISTER LOOKUP
════════════════════════════════════════════════════════════════ */

typedef struct { const char *name; uint8_t id; } RegEntry;

static RegEntry reg_table[] = {
    /* i registers */
    {"i0",0},{"i1",1},{"i2",2},{"i3",3},{"i4",4},{"i5",5},{"i6",6},{"i7",7},
    {"i8",8},{"i9",9},{"i10",10},{"i11",11},{"i12",12},{"i13",13},{"i14",14},{"i15",15},
    {"i16",16},{"i17",17},{"i18",18},{"i19",19},{"i20",20},{"i21",21},{"i22",22},{"i23",23},
    {"i24",24},{"i25",25},{"i26",26},{"i27",27},{"i28",28},{"i29",29},{"i30",30},{"i31",31},
    /* l registers */
    {"l0",32},{"l1",33},{"l2",34},{"l3",35},{"l4",36},{"l5",37},{"l6",38},{"l7",39},
    {"l8",40},{"l9",41},{"l10",42},{"l11",43},{"l12",44},{"l13",45},{"l14",46},{"l15",47},
    {"l16",48},{"l17",49},{"l18",50},{"l19",51},{"l20",52},{"l21",53},{"l22",54},{"l23",55},
    {"l24",56},{"l25",57},{"l26",58},{"l27",59},{"l28",60},{"l29",61},{"l30",62},{"l31",63},
    /* f registers */
    {"f0",64},{"f1",65},{"f2",66},{"f3",67},{"f4",68},{"f5",69},{"f6",70},{"f7",71},
    {"f8",72},{"f9",73},{"f10",74},{"f11",75},{"f12",76},{"f13",77},{"f14",78},{"f15",79},
    {"f16",80},{"f17",81},{"f18",82},{"f19",83},{"f20",84},{"f21",85},{"f22",86},{"f23",87},
    {"f24",88},{"f25",89},{"f26",90},{"f27",91},{"f28",92},{"f29",93},{"f30",94},{"f31",95},
    /* d registers */
    {"d0",96},{"d1",97},{"d2",98},{"d3",99},{"d4",100},{"d5",101},{"d6",102},{"d7",103},
    {"d8",104},{"d9",105},{"d10",106},{"d11",107},{"d12",108},{"d13",109},{"d14",110},{"d15",111},
    {"d16",112},{"d17",113},{"d18",114},{"d19",115},{"d20",116},{"d21",117},{"d22",118},{"d23",119},
    {"d24",120},{"d25",121},{"d26",122},{"d27",123},{"d28",124},{"d29",125},{"d30",126},{"d31",127},
    /* c registers */
    {"c0",128},{"c1",129},{"c2",130},{"c3",131},{"c4",132},{"c5",133},{"c6",134},{"c7",135},
    {"c8",136},{"c9",137},{"c10",138},{"c11",139},{"c12",140},{"c13",141},{"c14",142},{"c15",143},
    {"c16",144},{"c17",145},{"c18",146},{"c19",147},{"c20",148},{"c21",149},{"c22",150},{"c23",151},
    {"c24",152},{"c25",153},{"c26",154},{"c27",155},{"c28",156},{"c29",157},{"c30",158},{"c31",159},
    /* s registers */
    {"s0",160},{"s1",161},{"s2",162},{"s3",163},{"s4",164},{"s5",165},{"s6",166},{"s7",167},
    {"s8",168},{"s9",169},{"s10",170},{"s11",171},{"s12",172},{"s13",173},{"s14",174},{"s15",175},
    {"s16",176},{"s17",177},{"s18",178},{"s19",179},{"s20",180},{"s21",181},{"s22",182},{"s23",183},
    {"s24",184},{"s25",185},{"s26",186},{"s27",187},{"s28",188},{"s29",189},{"s30",190},{"s31",191},
    /* a registers */
    {"a0",192},{"a1",193},{"a2",194},{"a3",195},{"a4",196},{"a5",197},
    {"a6",198},{"a7",199},{"a8",200},{"a9",201},
    /* special */
    {"sp",202},{"sf",203},{"bp",204},{"bf",205},
    {NULL,0}
};

static uint8_t lookup_reg(const char *name) {
    for (int i = 0; reg_table[i].name; i++)
        if (strcmp(reg_table[i].name, name)==0) return reg_table[i].id;
    return REG_INVALID;
}

static uint8_t reg_class(uint8_t id) {
    if (id < 32)  return RC_I;
    if (id < 64)  return RC_L;
    if (id < 96)  return RC_F;
    if (id < 128) return RC_D;
    if (id < 160) return RC_C;
    if (id < 192) return RC_S;
    if (id < 202) return RC_A;
    return RC_SP;
}

/* ════════════════════════════════════════════════════════════════
   OPCODE TABLE
════════════════════════════════════════════════════════════════ */

typedef struct { const char *name; uint8_t opcode; int nops; } OpcodeEntry;

static OpcodeEntry opcode_table[] = {
    {"mov",0x01,2},{"movsx",0x02,2},{"movzx",0x03,2},{"movsxd",0x04,2},
    {"movb",0x0E,2},{"movw",0x0F,2},
    {"push",0x05,1},{"pop",0x06,1},{"pusha",0x07,0},{"popa",0x08,0},
    {"lea",0x09,2},{"fmov",0x0A,2},{"dmov",0x0B,2},{"lmov",0x0C,2},{"cmov",0x0D,3},
    {"add",0x10,3},{"addc",0x11,3},{"sub",0x12,3},{"subb",0x13,3},
    {"mul",0x14,3},{"imul",0x15,3},{"div",0x16,3},{"idiv",0x17,3},
    {"inc",0x18,1},{"dec",0x19,1},{"neg",0x1A,2},{"lneg",0x1B,2},
    {"ladd",0x1C,3},{"lsub",0x1D,3},{"lmul",0x1E,3},{"ldiv",0x1F,3},
    {"fadd",0x20,3},{"fsub",0x21,3},{"fmul",0x22,3},{"fdiv",0x23,3},{"fneg",0x24,2},
    {"dadd",0x25,3},{"dsub",0x26,3},{"dmul",0x27,3},{"ddiv",0x28,3},{"dneg",0x29,2},
    {"and",0x30,3},{"or",0x31,3},{"xor",0x32,3},{"not",0x33,2},
    {"shl",0x34,3},{"shr",0x35,3},{"sar",0x36,3},
    {"rol",0x37,3},{"ror",0x38,3},{"rcl",0x39,3},{"rcr",0x3A,3},
    {"shld",0x3B,4},{"shrd",0x3C,4},
    {"bsf",0x3D,2},{"bsr",0x3E,2},
    {"bt",0x3F,3},{"bts",0x40,3},{"btr",0x41,3},{"btc",0x42,3},
    {"popcnt",0x43,2},{"lzcnt",0x44,2},{"tzcnt",0x45,2},
    {"test",0x46,3},{"xchg",0x47,2},
    {"cmp",0x50,3},{"lcmp",0x51,3},{"fcmp",0x52,3},{"dcmp",0x53,3},
    {"eq",0x54,3},{"ne",0x55,3},{"gt",0x56,3},{"lt",0x57,3},{"gte",0x58,3},{"lte",0x59,3},
    {"jmp",0x60,1},{"goto",0x61,1},{"jcc",0x62,2},
    {"je",0x63,2},{"jne",0x64,2},{"jg",0x65,2},{"jge",0x66,2},
    {"jl",0x67,2},{"jle",0x68,2},{"ja",0x69,2},{"jb",0x6A,2},
    {"loop",0x6B,2},{"call",0x6C,1},{"ret",0x6D,0},{"retn",0x6E,1},
    {"exit",0x6F,1},{"halt",0x70,0},
    {"movs",0x78,2},{"stos",0x79,1},{"lods",0x7A,1},
    {"cmps",0x7B,3},{"scas",0x7C,1},
    {"nop",0x80,0},{"wait",0x81,0},{"in",0x82,2},{"out",0x83,2},
    {"int",0x84,1},{"iret",0x85,0},{"cpuid",0x86,0},
    {"rdtsc",0x87,0},{"pause",0x88,0},{"ud",0x89,0},
    {"cli",0x8A,0},{"sti",0x8B,0},
    {"itof",0x90,2},{"itod",0x91,2},{"itol",0x92,2},
    {"ltof",0x93,2},{"ltod",0x94,2},{"ftoi",0x95,2},
    {"ftod",0x96,2},{"ftol",0x97,2},{"dtoi",0x98,2},
    {"dtof",0x99,2},{"dtol",0x9A,2},{"ltoi",0x9B,2},
    {NULL,0,0}
};

static OpcodeEntry *lookup_opcode(const char *name) {
    for (int i = 0; opcode_table[i].name; i++)
        if (strcmp(opcode_table[i].name, name)==0) return &opcode_table[i];
    return NULL;
}

/* ════════════════════════════════════════════════════════════════
   ASSEMBLER STATE
════════════════════════════════════════════════════════════════ */

#define MAX_SYMS    4096
#define MAX_RELOCS  8192
#define MAX_SECTS   16
#define BUF_SIZE    (4*1024*1024)  /* 4MB per section */
#define STR_SIZE    (64*1024)

typedef struct {
    char     name[128];
    uint8_t  section;
    uint8_t  flags;       /* CXO_SYM_* */
    uint32_t offset;      /* offset within section */
    int      defined;
} Symbol;

typedef struct {
    char     label[128];
    uint8_t  section;
    uint32_t offset;      /* where the 4-byte slot to patch is */
    int      is_rel;      /* relative or absolute */
    int      line;
} Reloc;

typedef struct {
    uint8_t  type;
    uint8_t *data;
    uint32_t size;
    uint32_t cap;
    uint32_t load_addr;
} Section;

typedef struct {
    Symbol   syms[MAX_SYMS];
    int      sym_count;
    Reloc    relocs[MAX_RELOCS];
    int      reloc_count;
    Section  sects[MAX_SECTS];
    int      sect_count;
    int      cur_sect;    /* index into sects[] */
    char     strtab[STR_SIZE];
    int      strtab_len;
    int      errors;
    int      line;
} Asm;

static void asm_init(Asm *a) {
    memset(a, 0, sizeof(*a));
    /* pre-allocate section buffers */
    for (int i = 0; i < MAX_SECTS; i++) {
        a->sects[i].data = malloc(BUF_SIZE);
        a->sects[i].cap  = BUF_SIZE;
    }
    a->cur_sect = -1;
    /* seed string table with empty string at offset 0 */
    a->strtab[0] = 0; a->strtab_len = 1;
    /* pre-create .text at slot 0 so sects[0].offset is always the text
       section — cxld relies on sects[0].offset to find the string table end */
    a->sects[0].type = CXO_SEC_TEXT;
    a->sects[0].size = 0;
    a->sect_count    = 1;
}

static void asm_free(Asm *a) {
    for (int i = 0; i < MAX_SECTS; i++) free(a->sects[i].data);
}

static int strtab_add(Asm *a, const char *s) {
    int off = a->strtab_len;
    int len = strlen(s)+1;
    if (a->strtab_len + len >= STR_SIZE) { fprintf(stderr,"strtab overflow\n"); return 0; }
    memcpy(a->strtab + a->strtab_len, s, len);
    a->strtab_len += len;
    return off;
}

static void emit_byte(Asm *a, uint8_t b) {
    if (a->cur_sect < 0) { fprintf(stderr,"line %d: emit outside section\n", a->line); a->errors++; return; }
    Section *s = &a->sects[a->cur_sect];
    if (s->size >= s->cap) { fprintf(stderr,"section overflow\n"); a->errors++; return; }
    s->data[s->size++] = b;
}

static void emit_u16(Asm *a, uint16_t v) { emit_byte(a,v&0xFF); emit_byte(a,(v>>8)&0xFF); }
static void emit_u32(Asm *a, uint32_t v) { emit_u16(a,v&0xFFFF); emit_u16(a,(v>>16)&0xFFFF); }
static void emit_u64(Asm *a, uint64_t v) { emit_u32(a,(uint32_t)(v&0xFFFFFFFF)); emit_u32(a,(uint32_t)(v>>32)); }
static void emit_f32(Asm *a, float f)    { uint32_t v; memcpy(&v,&f,4); emit_u32(a,v); }
static void emit_f64(Asm *a, double f)   { uint64_t v; memcpy(&v,&f,8); emit_u64(a,v); }

static uint32_t cur_offset(Asm *a) {
    return a->cur_sect >= 0 ? a->sects[a->cur_sect].size : 0;
}

static Symbol *find_sym(Asm *a, const char *name) {
    for (int i = 0; i < a->sym_count; i++)
        if (strcmp(a->syms[i].name, name)==0) return &a->syms[i];
    return NULL;
}

static Symbol *add_sym(Asm *a, const char *name) {
    if (a->sym_count >= MAX_SYMS) { fprintf(stderr,"symbol table full\n"); a->errors++; return NULL; }
    Symbol *s = &a->syms[a->sym_count++];
    strncpy(s->name, name, 127);
    s->defined = 0; s->section = 0xFF; s->flags = CXO_SYM_LOCAL;
    return s;
}

static void define_sym(Asm *a, const char *name, uint8_t flags) {
    Symbol *s = find_sym(a, name);
    if (!s) s = add_sym(a, name);
    if (!s) return;
    if (s->defined) { fprintf(stderr,"line %d: symbol '%s' redefined\n", a->line, name); a->errors++; return; }
    s->section = a->cur_sect;
    s->offset  = cur_offset(a);
    s->flags   = flags;
    s->defined = 1;
}

static void add_reloc(Asm *a, const char *label, int is_rel) {
    if (a->reloc_count >= MAX_RELOCS) { fprintf(stderr,"reloc table full\n"); a->errors++; return; }
    Reloc *r = &a->relocs[a->reloc_count++];
    strncpy(r->label, label, 127);
    r->section = a->cur_sect;
    r->offset  = cur_offset(a);
    r->is_rel  = is_rel;
    r->line    = a->line;
    emit_u32(a, 0);  /* placeholder */
}

/* ════════════════════════════════════════════════════════════════
   OPERAND PARSER
════════════════════════════════════════════════════════════════ */

#define OP_KIND_REG   0
#define OP_KIND_IMM   1
#define OP_KIND_MEM   2
#define OP_KIND_LABEL 3
#define OP_KIND_FLOAT 4

typedef struct {
    int     kind;
    uint8_t reg;         /* register id */
    int64_t ival;        /* immediate */
    double  fval;        /* float immediate */
    /* memory: [base + index*scale + disp] */
    uint8_t mem_base;
    uint8_t mem_index;
    int32_t mem_scale;
    int32_t mem_disp;
    char    label[128];
} Operand;

static Token tok;
static Lexer *glex;

static void advance(void) { tok = lex_next(glex); }

static void skip_newlines(void) {
    while (tok.kind == TK_NEWLINE) advance();
}

static int parse_mem(Operand *op) {
    /* parse [base+index*scale+disp] */
    op->kind = OP_KIND_MEM;
    op->mem_base = REG_INVALID;
    op->mem_index = REG_INVALID;
    op->mem_scale = 1;
    op->mem_disp  = 0;
    advance(); /* eat [ */

    /* first term: reg or number */
    if (tok.kind == TK_IDENT) {
        uint8_t r = lookup_reg(tok.text);
        if (r != REG_INVALID) { op->mem_base = r; advance(); }
        else return 0;
    } else if (tok.kind == TK_NUMBER) {
        op->mem_disp = (int32_t)tok.ival; advance();
    }

    while (tok.kind == TK_PLUS || tok.kind == TK_MINUS) {
        int neg = (tok.kind == TK_MINUS); advance();
        if (tok.kind == TK_IDENT) {
            uint8_t r = lookup_reg(tok.text);
            if (r != REG_INVALID) {
                op->mem_index = r; advance();
                if (tok.kind == TK_STAR) {
                    advance();
                    if (tok.kind == TK_NUMBER) { op->mem_scale = (int32_t)tok.ival; advance(); }
                }
            } else op->mem_disp += neg ? -(int32_t)tok.ival : (int32_t)tok.ival;
        } else if (tok.kind == TK_NUMBER) {
            op->mem_disp += neg ? -(int32_t)tok.ival : (int32_t)tok.ival;
            advance();
        }
    }

    if (tok.kind != TK_RBRACKET) return 0;
    advance(); /* eat ] */
    return 1;
}

static int parse_operand(Operand *op) {
    memset(op, 0, sizeof(*op));
    op->mem_base = REG_INVALID;
    op->mem_index = REG_INVALID;

    if (tok.kind == TK_LBRACKET) return parse_mem(op);

    if (tok.kind == TK_IDENT) {
        uint8_t r = lookup_reg(tok.text);
        if (r != REG_INVALID) {
            op->kind = OP_KIND_REG; op->reg = r; advance(); return 1;
        }
        /* treat as label reference */
        op->kind = OP_KIND_LABEL;
        strncpy(op->label, tok.text, 127);
        advance(); return 1;
    }

    if (tok.kind == TK_MINUS) {
        advance();
        if (tok.kind == TK_NUMBER) { op->kind=OP_KIND_IMM; op->ival=-tok.ival; advance(); return 1; }
        if (tok.kind == TK_FLOAT)  { op->kind=OP_KIND_FLOAT; op->fval=-tok.fval; advance(); return 1; }
        return 0;
    }

    if (tok.kind == TK_NUMBER) { op->kind=OP_KIND_IMM; op->ival=tok.ival; advance(); return 1; }
    if (tok.kind == TK_FLOAT)  { op->kind=OP_KIND_FLOAT; op->fval=tok.fval; advance(); return 1; }

    return 0;
}

/* ════════════════════════════════════════════════════════════════
   INSTRUCTION EMITTER
════════════════════════════════════════════════════════════════ */

static void emit_operand_value(Asm *a, Operand *op, int is_branch) {
    switch (op->kind) {
    case OP_KIND_REG:
        emit_byte(a, op->reg);
        break;
    case OP_KIND_IMM:
        /* size depends on context — emit 32-bit by default */
        emit_u32(a, (uint32_t)op->ival);
        break;
    case OP_KIND_FLOAT:
        emit_f32(a, (float)op->fval);
        break;
    case OP_KIND_LABEL:
        add_reloc(a, op->label, is_branch);
        break;
    case OP_KIND_MEM:
        /* encode mem: 1 byte flags | base reg | index reg | scale(1) | disp(4) */
        {
            uint8_t mflags = 0;
            if (op->mem_base  != REG_INVALID) mflags |= 0x01;
            if (op->mem_index != REG_INVALID) mflags |= 0x02;
            emit_byte(a, mflags);
            if (mflags & 0x01) emit_byte(a, op->mem_base);
            if (mflags & 0x02) { emit_byte(a, op->mem_index); emit_byte(a,(uint8_t)op->mem_scale); }
            emit_u32(a, (uint32_t)op->mem_disp);
        }
        break;
    }
}

static void emit_instr(Asm *a, OpcodeEntry *oe, Operand *ops, int nops) {
    /* build mod byte */
    uint8_t mod = MOD_NOPERANDS(nops);
    uint8_t has_imm = 0, has_mem = 0, has_reg = 0;
    uint8_t dst_class=0, s1_class=0, s2_class=0;

    for (int i = 0; i < nops; i++) {
        if (ops[i].kind == OP_KIND_IMM || ops[i].kind == OP_KIND_FLOAT) has_imm = 1;
        if (ops[i].kind == OP_KIND_MEM) has_mem = 1;
        if (ops[i].kind == OP_KIND_REG) has_reg = 1;
    }
    if (has_imm) mod |= MOD_IMM;
    if (has_mem) mod |= MOD_MEM;

    /* if memory operand is LAST (store direction), set bit 2 as store flag */
    if (has_mem && nops >= 1 && ops[nops-1].kind == OP_KIND_MEM) mod |= 0x04;

    /* type bytes when any register operand present */
    if (has_reg) {
        mod |= MOD_EXT;
        /* dst is always last register operand */
        for (int i = nops-1; i >= 0; i--) {
            if (ops[i].kind == OP_KIND_REG) { dst_class = reg_class(ops[i].reg); break; }
        }
        /* src1 is first reg, src2 is second reg */
        int rc = 0;
        for (int i = 0; i < nops-1; i++) {
            if (ops[i].kind == OP_KIND_REG) {
                if (rc==0) s1_class = reg_class(ops[i].reg);
                else       s2_class = reg_class(ops[i].reg);
                rc++;
            }
        }
    }

    /* also set MOD_EXT for label operands so vm knows a 4-byte addr follows */
    for (int i = 0; i < nops; i++)
        if (ops[i].kind == OP_KIND_LABEL) { mod |= MOD_IMM; break; }

    emit_byte(a, oe->opcode);
    emit_byte(a, mod);
    if (mod & MOD_EXT) {
        emit_byte(a, dst_class);
        emit_byte(a, s1_class);
        emit_byte(a, s2_class);
    }

    /* emit operands: registers first (in order), then immediates/labels/mem */
    for (int i = 0; i < nops; i++)
        if (ops[i].kind == OP_KIND_REG) emit_byte(a, ops[i].reg);

    for (int i = 0; i < nops; i++) {
        if (ops[i].kind == OP_KIND_IMM)   emit_u32(a, (uint32_t)ops[i].ival);
        if (ops[i].kind == OP_KIND_FLOAT)  emit_f32(a, (float)ops[i].fval);
        if (ops[i].kind == OP_KIND_LABEL)  add_reloc(a, ops[i].label, 0);
        if (ops[i].kind == OP_KIND_MEM) {
            uint8_t mf = 0;
            if (ops[i].mem_base  != REG_INVALID) mf |= 0x01;
            if (ops[i].mem_index != REG_INVALID) mf |= 0x02;
            emit_byte(a, mf);
            if (mf & 0x01) emit_byte(a, ops[i].mem_base);
            if (mf & 0x02) { emit_byte(a, ops[i].mem_index); emit_byte(a,(uint8_t)ops[i].mem_scale); }
            emit_u32(a, (uint32_t)ops[i].mem_disp);
        }
    }
}

/* ════════════════════════════════════════════════════════════════
   DATA DIRECTIVES
════════════════════════════════════════════════════════════════ */

static void parse_data_directive(Asm *a, const char *dir) {
    do {
        if (tok.kind == TK_COMMA) { advance(); continue; }
        if (tok.kind == TK_STRING) {
            /* emit raw bytes of string — no implicit null terminator */
            for (int i = 0; i < (int)strlen(tok.text); i++)
                emit_byte(a, (uint8_t)tok.text[i]);
            advance();
        } else if (tok.kind == TK_NUMBER || tok.kind == TK_MINUS) {
            int64_t v = 0;
            if (tok.kind == TK_MINUS) { advance(); v = -tok.ival; } else v = tok.ival;
            advance();
            if      (strcmp(dir,"db")==0) emit_byte(a,(uint8_t)v);
            else if (strcmp(dir,"dw")==0) emit_u16(a,(uint16_t)v);
            else if (strcmp(dir,"dd")==0) emit_u32(a,(uint32_t)v);
            else if (strcmp(dir,"dq")==0) emit_u64(a,(uint64_t)v);
        } else if (tok.kind == TK_FLOAT) {
            double fv = tok.fval; advance();
            if      (strcmp(dir,"df")==0)  emit_f32(a,(float)fv);
            else if (strcmp(dir,"ddf")==0) emit_f64(a,fv);
            else emit_u32(a,0);
        } else break;
    } while (tok.kind == TK_COMMA);
}

static void parse_res_directive(Asm *a, const char *dir) {
    if (tok.kind != TK_NUMBER) return;
    uint32_t n = (uint32_t)tok.ival; advance();
    uint32_t sz = 1;
    if (strcmp(dir,"resw")==0) sz=2;
    else if (strcmp(dir,"resd")==0) sz=4;
    else if (strcmp(dir,"resq")==0) sz=8;
    for (uint32_t i = 0; i < n*sz; i++) emit_byte(a,0);
}

/* ════════════════════════════════════════════════════════════════
   SECTION MANAGEMENT
════════════════════════════════════════════════════════════════ */

static int find_or_create_section(Asm *a, const char *name, uint8_t type) {
    for (int i = 0; i < a->sect_count; i++) {
        if (a->sects[i].type == type) return i;
    }
    int idx = a->sect_count++;
    a->sects[idx].type = type;
    a->sects[idx].size = 0;
    return idx;
}

static void switch_section(Asm *a, const char *name) {
    uint8_t type = CXO_SEC_TEXT;
    if (strcmp(name,"data")==0)   type = CXO_SEC_DATA;
    if (strcmp(name,"rodata")==0) type = CXO_SEC_RODATA;
    if (strcmp(name,"bss")==0)    type = CXO_SEC_BSS;
    a->cur_sect = find_or_create_section(a, name, type);
}

/* ════════════════════════════════════════════════════════════════
   MAIN PARSE LOOP
════════════════════════════════════════════════════════════════ */

static void parse_line(Asm *a) {
    if (tok.kind == TK_NEWLINE || tok.kind == TK_EOF) return;
    if (tok.kind != TK_IDENT) {
        fprintf(stderr,"line %d: unexpected token '%s'\n", a->line, tok.text);
        a->errors++;
        while (tok.kind != TK_NEWLINE && tok.kind != TK_EOF) advance();
        return;
    }

    char word[256]; strncpy(word, tok.text, 255); advance();

    /* section directive */
    if (strcmp(word,"section")==0) {
        if (tok.kind != TK_IDENT) { fprintf(stderr,"line %d: expected section name\n",a->line); a->errors++; return; }
        char sname[128]; strncpy(sname,tok.text,127); advance();
        if (tok.kind == TK_COLON) advance();
        switch_section(a, sname);
        return;
    }

    /* global / local directive */
    if (strcmp(word,"global")==0 || strcmp(word,"local")==0) {
        uint8_t flags = (strcmp(word,"global")==0) ? CXO_SYM_GLOBAL : CXO_SYM_LOCAL;
        if (tok.kind == TK_IDENT) {
            char sname[128]; strncpy(sname,tok.text,127); advance();
            if (tok.kind == TK_COLON) {
                advance();
                define_sym(a, sname, flags);
            } else {
                /* forward declaration */
                Symbol *s = find_sym(a,sname);
                if (!s) s = add_sym(a,sname);
                if (s) s->flags = flags;
            }
        }
        return;
    }

    /* data directives */
    if (strcmp(word,"db")==0||strcmp(word,"dw")==0||strcmp(word,"dd")==0||
        strcmp(word,"dq")==0||strcmp(word,"df")==0||strcmp(word,"ddf")==0) {
        parse_data_directive(a, word); return;
    }
    if (strcmp(word,"resb")==0||strcmp(word,"resw")==0||strcmp(word,"resd")==0||strcmp(word,"resq")==0) {
        parse_res_directive(a, word); return;
    }

    /* align directive */
    if (strcmp(word,"align")==0) {
        if (tok.kind == TK_NUMBER) {
            uint32_t al = (uint32_t)tok.ival; advance();
            if (al > 1 && a->cur_sect >= 0) {
                uint32_t off = a->sects[a->cur_sect].size;
                uint32_t pad = (al - (off % al)) % al;
                for (uint32_t i = 0; i < pad; i++) emit_byte(a,0x80); /* nop pad */
            }
        }
        return;
    }

    /* check if next token is colon → label definition */
    if (tok.kind == TK_COLON) {
        advance();
        define_sym(a, word, CXO_SYM_LOCAL);
        /* might have instruction on same line */
        if (tok.kind != TK_NEWLINE && tok.kind != TK_EOF) parse_line(a);
        return;
    }

    /* must be an instruction */
    OpcodeEntry *oe = lookup_opcode(word);
    if (!oe) {
        fprintf(stderr,"line %d: unknown instruction '%s'\n", a->line, word);
        a->errors++;
        while (tok.kind != TK_NEWLINE && tok.kind != TK_EOF) advance();
        return;
    }

    Operand ops[8]; int nops = 0;
    while (tok.kind != TK_NEWLINE && tok.kind != TK_EOF) {
        if (tok.kind == TK_COMMA) { advance(); continue; }
        if (!parse_operand(&ops[nops])) break;
        nops++;
        if (nops >= 8) break;
    }

    emit_instr(a, oe, ops, nops);
}

static void assemble(Asm *a, const char *src) {
    Lexer lex; lex_init(&lex, src); glex = &lex;
    advance();
    while (tok.kind != TK_EOF) {
        skip_newlines();
        if (tok.kind == TK_EOF) break;
        a->line = tok.line;
        parse_line(a);
        skip_newlines();
    }
}

/* ════════════════════════════════════════════════════════════════
   SYMBOL RESOLUTION (patch relocs)
════════════════════════════════════════════════════════════════ */

static void resolve(Asm *a) {
    for (int i = 0; i < a->reloc_count; i++) {
        Reloc *r = &a->relocs[i];
        Symbol *s = find_sym(a, r->label);
        if (!s || !s->defined) {
            /* mark as external */
            if (!s) s = add_sym(a, r->label);
            if (s) { s->flags = CXO_SYM_EXTERN; }
            /* leave placeholder zero — linker resolves */
            continue;
        }
        /* same section: patch the value */
        Section *sec = &a->sects[r->section];
        uint32_t target = s->offset;
        uint32_t patch_offset = r->offset;
        uint32_t value;
        if (r->is_rel) {
            /* relative: target - (patch_site + 4) */
            value = target - (patch_offset + 4);
        } else {
            value = target;
        }
        sec->data[patch_offset+0] = (value      ) & 0xFF;
        sec->data[patch_offset+1] = (value >>  8) & 0xFF;
        sec->data[patch_offset+2] = (value >> 16) & 0xFF;
        sec->data[patch_offset+3] = (value >> 24) & 0xFF;
    }
}

/* ════════════════════════════════════════════════════════════════
   OBJECT FILE WRITER
════════════════════════════════════════════════════════════════ */

static const char *sect_names[] = {"text","data","rodata","bss"};

static int write_cxo(Asm *a, const char *outpath) {
    FILE *f = fopen(outpath,"wb");
    if (!f) { fprintf(stderr,"cannot open output '%s': %s\n", outpath, strerror(errno)); return 0; }

    /* count real sections */
    int nsect = a->sect_count;

    /* build string table: add section names + symbol names */
    int sect_name_offs[MAX_SECTS];
    for (int i = 0; i < nsect; i++) {
        uint8_t t = a->sects[i].type;
        const char *sn = (t>=1&&t<=4) ? sect_names[t-1] : "unknown";
        sect_name_offs[i] = strtab_add(a, sn);
    }
    int sym_name_offs[MAX_SYMS];
    for (int i = 0; i < a->sym_count; i++)
        sym_name_offs[i] = strtab_add(a, a->syms[i].name);

    /* compute file offsets */
    uint32_t hdr_size   = sizeof(CxoHeader);
    uint32_t sect_size  = nsect * sizeof(CxoSection);
    uint32_t sym_size   = a->sym_count * sizeof(CxoSymbol);
    uint32_t rel_size   = a->reloc_count * sizeof(CxoReloc);
    uint32_t strtab_off = hdr_size + sect_size + sym_size + rel_size;
    uint32_t data_off   = strtab_off + a->strtab_len;
    /* align to 4 */
    data_off = (data_off + 3) & ~3;

    /* section data offsets */
    uint32_t sec_offs[MAX_SECTS];
    uint32_t cur = data_off;
    for (int i = 0; i < nsect; i++) {
        sec_offs[i] = cur;
        cur += a->sects[i].size;
    }

    /* write header */
    CxoHeader hdr = {0};
    hdr.magic         = CXO_MAGIC;
    hdr.version       = CXO_VERSION;
    hdr.section_count = nsect;
    hdr.sym_count     = a->sym_count;
    hdr.rel_count     = a->reloc_count;
    fwrite(&hdr, sizeof(hdr), 1, f);

    /* write section headers */
    for (int i = 0; i < nsect; i++) {
        CxoSection cs = {0};
        cs.type      = a->sects[i].type;
        cs.name_off  = sect_name_offs[i];
        cs.offset    = sec_offs[i];
        cs.size      = a->sects[i].size;
        cs.align     = 4;
        fwrite(&cs, sizeof(cs), 1, f);
    }

    /* write symbol table */
    for (int i = 0; i < a->sym_count; i++) {
        CxoSymbol cs = {0};
        cs.name_off  = sym_name_offs[i];
        cs.section   = a->syms[i].section;
        cs.flags     = a->syms[i].flags;
        cs.offset    = a->syms[i].offset;
        fwrite(&cs, sizeof(cs), 1, f);
    }

    /* write reloc table */
    for (int i = 0; i < a->reloc_count; i++) {
        Reloc *r = &a->relocs[i];
        Symbol *s = find_sym(a, r->label);
        CxoReloc cr = {0};
        cr.offset    = r->offset;
        cr.sym_index = 0;
        if (s) {
            for (int j = 0; j < a->sym_count; j++)
                if (&a->syms[j]==s) { cr.sym_index=j; break; }
        }
        cr.type      = r->is_rel ? CXO_REL_REL32 : CXO_REL_ABS32;
        cr.section   = r->section;
        fwrite(&cr, sizeof(cr), 1, f);
    }

    /* write string table */
    fwrite(a->strtab, a->strtab_len, 1, f);
    /* pad to 4-byte alignment */
    uint32_t written = strtab_off + a->strtab_len;
    while (written % 4) { fputc(0,f); written++; }

    /* write section data */
    for (int i = 0; i < nsect; i++) {
        if (a->sects[i].type != CXO_SEC_BSS)
            fwrite(a->sects[i].data, a->sects[i].size, 1, f);
    }

    fclose(f);
    return 1;
}

/* ════════════════════════════════════════════════════════════════
   MAIN
════════════════════════════════════════════════════════════════ */

static char *read_file(const char *path) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"cannot open '%s': %s\n", path, strerror(errno)); return NULL; }
    fseek(f,0,SEEK_END); long sz = ftell(f); rewind(f);
    char *buf = malloc(sz+1);
    fread(buf,1,sz,f); buf[sz]=0; fclose(f);
    return buf;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,"usage: cxas <file.cxis> [-o output.cxo]\n");
        return 1;
    }

    const char *infile  = argv[1];
    const char *outfile = "out.cxo";
    for (int i = 2; i < argc-1; i++)
        if (strcmp(argv[i],"-o")==0) outfile = argv[i+1];

    char *src = read_file(infile);
    if (!src) return 1;

    Asm a; asm_init(&a);
    assemble(&a, src);
    resolve(&a);
    free(src);

    if (a.errors) {
        fprintf(stderr,"%d error(s) — no output written\n", a.errors);
        asm_free(&a); return 1;
    }

    if (!write_cxo(&a, outfile)) { asm_free(&a); return 1; }

    printf("assembled '%s' → '%s'  (%d symbol(s), %d reloc(s))\n",
           infile, outfile, a.sym_count, a.reloc_count);

    asm_free(&a); return 0;
}
