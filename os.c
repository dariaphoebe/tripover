// os.c - operating system specifics

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <unistd.h>

#include <sys/time.h>
#include <errno.h>

#include <string.h>

#include "base.h"

#include "os.h"

int oswrite(int fd, const void *buf,ub4 len)
{
  return (int)write(fd,buf,len);
}

char *getoserr(void)
{
  return strerror(errno);
}

ub8 gettime_usec(void)
{
  struct timeval tv;
  ub8 usec;

  gettimeofday(&tv,0);
  usec = (ub8)tv.tv_sec * 1000UL * 1000UL;
  usec += tv.tv_usec;
  return usec;
}
