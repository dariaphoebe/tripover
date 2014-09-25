// server.h

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* server side of local client-server interface

  tripover server runs on a server machine and answers journey queries
  it loads and inialises the network, then loops for queries
  it may restart to get a fresh network snapshot, or update real-time changes
  multiple server instances are running.

  The client interface posts its queries, and one server picks it up
  at timeout, client can re-post

  a separate network proxy can provide http-style interface, forwarding to a local client
 */

extern int serverloop(void);

extern void iniserver(void);
