// util.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

struct cmdval { // commandline arg results
  ub4 uval;
  ub4 valcnt;
  char *sval;
  struct cmdarg *args;
  ub4 argndx;
  int retval;
};

struct cmdarg { // commandline arg defines
  const char *arg;
  const char *val;
  const char *desc;
  int (*fn)(struct cmdval *cp);
};

extern int cmdline(int argc, char *argv[], struct cmdarg *cap);

extern int str2ub4(const char *s, ub4 *pv);

extern void iniutil(void);
