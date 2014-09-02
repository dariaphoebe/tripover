// mem.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define MFLN __LINE__|msgfile

#define alloc(len,el,fill,desc,arg) (el*)alloc_fln((len),sizeof(el),#len,#el,(fill),(desc),(arg),MFLN)

extern void *alloc_fln(ub4 elems,ub4 elsize,const char *slen,const char *sel,ub1 fill,char *desc,ub4 arg,ub4 fln);

extern void inimem(void);
