// os.h - operating system specifics

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern ub8 gettime_usec(void);
extern char *getoserr(void);
extern int osopen(const char *name);
extern int osread(int fd,void *buf,size_t len);
extern int oswrite(int fd, const void *buf,ub4 len);
extern int oscreate(const char *name);
extern int osfdinfo(struct myfile *mf,int fd);
extern int osclose(int fd);
extern int setsigs(void);
extern void inios(void);
