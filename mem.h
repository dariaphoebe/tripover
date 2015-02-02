// mem.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define MFLN (__LINE__|msgfile)

struct strpool {
  char *pool;
  ub4 len;
};
typedef struct strpool spool;


struct memblk {
  ub4 magic;
  void *base;
  size_t elems;
  ub4 elsize;
  ub8 stamp;
  ub4 fln;
  ub4 seq;
  const char *selems;
  const char *selsize;
  ub4 desclen;
  char desc[256];
};
typedef struct memblk block;

// global name.
struct gname {
  spool *strpool;  // vars below index here 
  ub4 name;        // common name
  ub4 code;        // formal coded name, e.g. IATA
  ub4 locname;     // name in local language
  ub4 intname;     // international name
  ub4 descr;
};

enum Blkopts { Noinit, Init0, Init1 };

#define alloc(cnt,el,fill,desc,arg) (el*)alloc_fln((cnt),sizeof(el),#cnt,#el,(fill),(desc),(arg),MFLN)
#define mkblock(blk,cnt,el,opt,...) (el*)mkblock_fln((blk),(cnt),sizeof(el),(opt),#cnt,#el,MFLN,__VA_ARGS__)

#define afree(ptr,desc) afree_fln((ptr),MFLN,(desc))

#define blkdata(blk,pos,el) (el*)(blk)->base + (pos)

#define bound(blk,pos,el) bound_fln((blk),(pos),sizeof(el),#pos,#el,MFLN)

extern void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,const char *desc,ub4 arg,ub4 fln);
extern void * __attribute__ ((format (printf,8,9))) mkblock_fln(block *blk,size_t elems,ub4 elsize,enum Blkopts opts,const char *selems,const char *selsize,ub4 fln,const char *fmt,...);
extern void bound_fln(block *blk,size_t pos,ub4 elsize,const char *spos,const char *sel,ub4 fln);
extern int afree_fln(void *p,ub4 fln, const char *desc);

extern void showmemsums(void);

extern size_t nearblock(size_t adr);

extern ub4 meminfo(void);

extern void inimem(void);
extern void eximem(void);
