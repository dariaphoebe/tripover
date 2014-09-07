// net.c - main network setup

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* Initialize the network :

   - Separate ports in full and minor.
     Full ports have enough connectivity to include in a (full x full) matrix.
     Minor ports connect only to one or two single transfer stops.
     Actual plannig is done on these transfer ports.

   - Build connectivity matrix between any 2 full ports
     base matrix for direct (non-stop) hops
     derived matrix for each of n intermediate hops

   - Prepare various metrics used for heuristics
 */

#include "base.h"
#include "mem.h"

static ub4 msgfile;
#include "msg.h"

#include "util.h"
#include "bitfields.h"
#include "netbase.h"
#include "net.h"

void ininet(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}

// generate hypothetical network for testing purposes
int mkrandnet(netbase *basenet)
{
  ub4 portcnt = basenet->portcnt;
  ub4 hopcnt = basenet->hopcnt;

  struct portbase *ports;
  struct hopbase *hops;

  if (portcnt == 0 || hopcnt == 0) return 0;

  ports = alloc(portcnt,struct portbase,0,"baseports",portcnt);
  hops = alloc(hopcnt,struct hopbase,0,"basehops",hopcnt);

  // todo create random net in ports,hops

  basenet->ports = ports;
  basenet->hops = hops;
  return 0;
}
