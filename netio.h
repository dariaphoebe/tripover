// netio.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

extern int net2pdf(netbase *net);
extern void ininetio(void);
extern int net2ext(netbase *net);
extern int readextnet(netbase *net,const char *dir);
extern int wrportrefs(netbase *net);
