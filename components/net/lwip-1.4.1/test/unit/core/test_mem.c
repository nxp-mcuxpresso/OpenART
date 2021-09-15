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
#include "test_mem.h"

#include "lwip/mem.h"
#include "lwip/stats.h"

#if !LWIP_STATS || !MEM_STATS
#error "This tests needs MEM-statistics enabled"
#endif
#if LWIP_DNS
#error "This test needs DNS turned off (as it mallocs on init)"
#endif

/* Setups/teardown functions */

static void
mem_setup(void)
{
}

static void
mem_teardown(void)
{
}


/* Test functions */

/** Call mem_malloc, mem_free and mem_trim and check stats */
START_TEST(test_mem_one)
{
#define SIZE1   16
#define SIZE1_2 12
#define SIZE2   16
  void *p1, *p2;
  mem_size_t s1, s2;
  LWIP_UNUSED_ARG(_i);

#if LWIP_DNS
  fail("This test needs DNS turned off (as it mallocs on init)");
#endif

  fail_unless(lwip_stats.mem.used == 0);

  p1 = mem_malloc(SIZE1);
  fail_unless(p1 != NULL);
  fail_unless(lwip_stats.mem.used >= SIZE1);
  s1 = lwip_stats.mem.used;

  p2 = mem_malloc(SIZE2);
  fail_unless(p2 != NULL);
  fail_unless(lwip_stats.mem.used >= SIZE2 + s1);
  s2 = lwip_stats.mem.used;

  mem_trim(p1, SIZE1_2);

  mem_free(p2);
  fail_unless(lwip_stats.mem.used <= s2 - SIZE2);

  mem_free(p1);
  fail_unless(lwip_stats.mem.used == 0);
}
END_TEST


/** Create the suite including all tests for this module */
Suite *
mem_suite(void)
{
  TFun tests[] = {
    test_mem_one
  };
  return create_suite("MEM", tests, sizeof(tests)/sizeof(TFun), mem_setup, mem_teardown);
}
