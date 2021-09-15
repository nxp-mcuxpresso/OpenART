/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 */
#include "test_udp.h"

#include "lwip/udp.h"
#include "lwip/stats.h"

#if !LWIP_STATS || !UDP_STATS || !MEMP_STATS
#error "This tests needs UDP- and MEMP-statistics enabled"
#endif

/* Helper functions */
static void
udp_remove_all(void)
{
  struct udp_pcb *pcb = udp_pcbs;
  struct udp_pcb *pcb2;

  while(pcb != NULL) {
    pcb2 = pcb;
    pcb = pcb->next;
    udp_remove(pcb2);
  }
  fail_unless(lwip_stats.memp[MEMP_UDP_PCB].used == 0);
}

/* Setups/teardown functions */

static void
udp_setup(void)
{
  udp_remove_all();
}

static void
udp_teardown(void)
{
  udp_remove_all();
}


/* Test functions */

START_TEST(test_udp_new_remove)
{
  struct udp_pcb* pcb;
  LWIP_UNUSED_ARG(_i);

  fail_unless(lwip_stats.memp[MEMP_UDP_PCB].used == 0);

  pcb = udp_new();
  fail_unless(pcb != NULL);
  if (pcb != NULL) {
    fail_unless(lwip_stats.memp[MEMP_UDP_PCB].used == 1);
    udp_remove(pcb);
    fail_unless(lwip_stats.memp[MEMP_UDP_PCB].used == 0);
  }
}
END_TEST


/** Create the suite including all tests for this module */
Suite *
udp_suite(void)
{
  TFun tests[] = {
    test_udp_new_remove,
  };
  return create_suite("UDP", tests, sizeof(tests)/sizeof(TFun), udp_setup, udp_teardown);
}
