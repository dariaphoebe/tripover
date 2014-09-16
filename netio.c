// netio.c - networks to and from file

/*
   This file is part of Tripover, a broad-search journey planner.

   Copyright (C) 2014 Joris van der Geer.

   This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/
 */

/* functions to read and write tripover internal networks, utility functions to write diagnostics
 */

#include <string.h>
#include <stdlib.h>

#include "base.h"
#include "mem.h"
#include "util.h"
#include "os.h"

static ub4 msgfile;
#include "msg.h"

#include "bitfields.h"
#include "netbase.h"

#include "netio.h"

static ub4 pdfscale_lat = 2000;
static ub4 pdfscale_lon = 2000;

static ub4 lat2pdf(ub4 lat) { return lat * pdfscale_lat / (180 * Latscale); }
static ub4 lon2pdf(ub4 lon) { return lon * pdfscale_lon / (360 * Lonscale); }

// write network as page content
static ub4 addnetpdf(netbase *net, char *buf, ub4 len)
{
  struct portbase *pp,*pdep,*parr,*ports = net->ports;
  struct hopbase *hp,*hops = net->hops;
  ub4 port,portcnt = net->portcnt;
  ub4 hop,hopcnt = net->hopcnt;
  ub4 x,y,x0,y0,x1,y1,arr,dep;
  ub4 pos,n;

  pos = mysnprintf(buf,0,len,"BT /F1 18 Tf 25 25 Td (title Tj ET\n");

  pos += mysnprintf(buf,pos,len,"BT /F1 20 Tf\n");

  // temporary: port as number
  for (port = 0; port < portcnt; port++) {
    pp = ports + port;
    y = lat2pdf(pp->lat);
    x = lon2pdf(pp->lon);
    n = mysnprintf(buf,pos,len,"1 0 0 1 %u %u Tm (%u) Tj ", x,y,port);
    if (n == 0) break;
    pos += n;
  }
  pos += mysnprintf(buf,pos,len,"ET\n");

  pos += mysnprintf(buf,pos,len,"3 w\n");

  // draw direct connection as straight line
  for (hop = 0; hop < hopcnt; hop++) {
    hp = hops + hop;
    dep = hp->dep;
    arr = hp->arr;
    pdep = ports + dep;
    parr = ports + arr;
    y0 = lat2pdf(pdep->lat);
    x0 = lon2pdf(pdep->lon);
    y1 = lat2pdf(parr->lat);
    x1 = lon2pdf(parr->lon);
    n = mysnprintf(buf,pos,len,"%u %u m %u %u l s\n",x0,y0,x1,y1);
    if (n == 0) break;
    pos += n;
  }
  if (pos == len) warning(0,"pdf output buffer limit \ah%u reached: truncated", len);
  return pos;
}

/* write graphic representation of network:
  ports, hops
  currently single page only
 */
int net2pdf(netbase *net)
{
static char content[64 * 1024 * 1024];
static char pagebuf[64 * 1024];

  int fd;
  ub4 pos,xpos,cpos,xrefpos,obj;
  ub4 plen = sizeof pagebuf;
  ub4 xref[16];
  enum objnos { pdfnil, pdfcat, pdftree,pdfnode,pdfcontent, pdflast };

  fd = oscreate("net.pdf");

  pos = mysnprintf(pagebuf,0,plen,"%%PDF-1.4\n");

  // patterned after simple examples in PDF reference
  xref[pdfcat] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Catalog /Pages %u 0 R >>\nendobj\n",pdfcat,pdftree);

  xref[pdftree] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Pages /Kids [%u 0 R] /Count 1 >>\nendobj\n", pdftree,pdfnode);

  xref[pdfnode] = pos;
  pos += mysnprintf(pagebuf,pos,plen,"%u 0 obj\n << /Type /Page /Parent %u 0 R "
    "/MediaBox [ 0 0 %u %u ] /Contents %u 0 R /Resources"
    " << /Font << /F1 << /Type /Font /Subtype /Type1 /BaseFont /Helvetica >> >> >> >>\n"
    "endobj\n", pdfnode,pdftree,pdfscale_lon, pdfscale_lat,pdfcontent);

  xref[pdfcontent] = pos;

  xpos = pos;
  oswrite(fd,pagebuf,pos);
 
  cpos = addnetpdf(net,content,sizeof content);

  pos = mysnprintf(pagebuf,0, plen,"%u 0 obj\n << /Length %u >>\nstream\n", pdfcontent, cpos);
  oswrite(fd,pagebuf,pos);
  xpos += pos + cpos;

  oswrite(fd,content,cpos);

  pos = mysnprintf(pagebuf,0,plen,"endstream\nendobj\n");

  xrefpos = xpos + pos;
  pos += mysnprintf(pagebuf,pos,plen,"xref\n0 %u\n",pdflast);

  pos += mysnprintf(pagebuf,pos,plen,"%010u 65535 f \n", 0);
  for (obj = 1; obj <= pdfcontent; obj++) pos += mysnprintf(pagebuf,pos,plen,"%010u 00000 n \n", xref[obj]);

  pos += mysnprintf(pagebuf,pos,plen,"trailer\n << /Size %u /Root %u 0 R >>\n", pdflast, pdfcat);

  pos += mysnprintf(pagebuf,pos,plen,"startxref\n%u\n%s\n", xrefpos,"%%EOF");

  oswrite(fd,pagebuf,pos);
  osclose(fd);

  return 0;
}

void ininetio(void)
{
  msgfile = setmsgfile(__FILE__);
  iniassert();
}
