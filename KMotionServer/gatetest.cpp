// gatetest.cpp - BoardGate.h driven with a clock of its own, no board and no server:
//   make -C KMotionServer check
// Connections are plain numbers; the lock's results are the gate's own (in_use = 1).
#include "BoardGate.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static double fake_t = 100.0;
static double fake_clock() { return fake_t; }

static int failures = 0;
static void check(bool ok, const char *what)
{
	printf("  %s  %s\n", ok ? "pass" : "FAIL", what);
	if (!ok) failures++;
}

typedef BoardGates<4> Gates;
enum { A = 10, B = 11, C = 12 };
static const int IN_USE = 1;

// a lock request from c, as the server runs it: enter, the library's answer, leave
static int lock(Gates &gt, int c, bool library_grants = true)
{
	int result = -99;
	if (!gt.enter(0, c, GATE_LOCK, &result)) return result;
	result = library_grants ? 0 : IN_USE;
	gt.leave(0, c, GATE_LOCK, library_grants);
	return result;
}

static int release(Gates &gt, int c)
{
	int result = -99;
	if (!gt.enter(0, c, GATE_RELEASE, &result)) return result;
	gt.leave(0, c, GATE_RELEASE, false);
	return 0;
}

struct Blocked { Gates *gt; volatile bool through; };
static void *tokenless(void *p)
{
	Blocked *bl = (Blocked *) p;
	int result;
	bl->gt->enter(0, B, GATE_OTHER, &result);
	bl->through = true;
	bl->gt->leave(0, B, GATE_OTHER, false);
	return NULL;
}

int main()
{
	Gates gt(IN_USE, fake_clock);

	check(lock(gt, A) == 0 && gt.g[0].owner == A, "A takes the free token");
	check(lock(gt, B) == IN_USE && gt.g[0].waiter == B, "B is refused at once and remembered as the waiter");
	check(lock(gt, C) == IN_USE && gt.g[0].waiter == B, "C is refused too; the first waiter keeps its turn");
	check(release(gt, B) == 0 && gt.g[0].owner == A, "B's release is not A's token");
	release(gt, A);
	check(gt.g[0].owner == -1 && gt.g[0].reserved == B && gt.g[0].reserved_until == fake_t + 0.03 && gt.g[0].waiter == -1,
	      "A's release keeps the token for B for 30 ms");
	fake_t += 0.005;
	check(lock(gt, A) == IN_USE && gt.g[0].owner == -1, "A asking again within the hold is refused");
	check(lock(gt, C) == IN_USE, "so is C");
	fake_t += 0.005;
	check(lock(gt, B) == 0 && gt.g[0].owner == B && gt.g[0].reserved == -1, "B gets it and the hold ends");
	check(lock(gt, A) == IN_USE && gt.g[0].waiter == A, "now A waits");
	release(gt, B);
	fake_t += 0.01;
	check(lock(gt, A) == 0 && gt.g[0].owner == A, "and gets the token after B's release: round robin");

	// the waiter that gave up: the hold expires
	check(lock(gt, B) == IN_USE, "B refused while A holds");
	release(gt, A);
	fake_t += 0.05;
	check(lock(gt, C) == 0 && gt.g[0].owner == C, "50 ms later anybody may take it");
	release(gt, C);

	// the library refuses although the gate let the request through (a token the library
	// still counts as held, after a failure): the gate's owner stays free
	check(lock(gt, A, false) == IN_USE && gt.g[0].owner == -1, "a lock the library refuses leaves no owner");

	// a connection closing with the token hands it over; closing as the waiter or the
	// reserved one is forgotten
	lock(gt, A);
	lock(gt, B);
	int released[4], nrel = 0;
	gt.drop(A, [&](int board) { released[nrel++] = board; });
	check(nrel == 1 && released[0] == 0 && gt.g[0].owner == -1 && gt.g[0].reserved == B, "A closing: its token released and kept for B");
	gt.drop(B, [&](int) { nrel++; });
	check(nrel == 1 && gt.g[0].reserved == -1, "B closing: nothing to release, the hold gone");
	check(lock(gt, C) == 0, "C takes it");
	lock(gt, A);
	gt.drop(A, [&](int) { nrel++; });
	check(nrel == 1 && gt.g[0].waiter == -1, "A closing as the waiter is forgotten");
	release(gt, C);

	// a tokenless request from another connection waits for the release (the real clock,
	// the thread blocks in the gate)
	Gates rt(IN_USE);
	lock(rt, A);
	Blocked bl = { &rt, false };
	pthread_t th;
	pthread_create(&th, NULL, tokenless, &bl);
	usleep(30000);
	check(!bl.through, "B's tokenless request waits while A holds the token");
	release(rt, A);
	pthread_join(th, NULL);
	check(bl.through, "and goes through after A's release");

	printf(failures ? "gate test FAILED (%d)\n" : "gate test passed\n", failures);
	return failures ? 1 : 0;
}
