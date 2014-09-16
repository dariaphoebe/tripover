// os.c - operating system specifics

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#define _POSIX_C_SOURCE 199309L

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/time.h>
#include <errno.h>
#include <signal.h>

#include <string.h>

#include "base.h"

static ub4 msgfile;
#include "msg.h"

#include "os.h"

int oscreate(const char *name)
{
  int fd = creat(name,0644);
  return fd;
}

int oswrite(int fd, const void *buf,ub4 len)
{
  return (int)write(fd,buf,len);
}

int osclose(int fd)
{
  return close(fd);
}

char *getoserr(void)
{
  return strerror(errno);
}

static void __attribute__ ((noreturn)) mysigact(int sig,siginfo_t *si,void * __attribute__ ((unused)) p)
{
  char buf[4096];
  ub4 pos;
  size_t adr;

  switch(sig) {
  case SIGSEGV:
    adr = (size_t)(si->si_addr);
    pos = mysnprintf(buf,0,sizeof buf,"\nsigsegv at %lx\n", (unsigned long)adr);
    break;
  default: pos = mysnprintf(buf,0,sizeof buf,"\nsignal %u\n", sig);
  }
  oswrite(2,buf,pos);
  _exit(1);
}

int setsigs(void)
{
  struct sigaction sa;

  memset(&sa,0,sizeof(sa));
  sa.sa_sigaction = mysigact;
  sa.sa_flags = SA_SIGINFO;

  sigaction(SIGSEGV, &sa,NULL);
  return 0;
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

void inios(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}
