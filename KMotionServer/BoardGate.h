// BoardGate.h - a board's token, by connection: which of KMotionServer's connections holds it,
// and its hand-over to a connection that was refused while another held it. The rules are in
// the comment above BoardGates in KMotionServerX.cpp; this header holds them without the
// server's other dependencies, so that gatetest.cpp can drive them with a clock of its own.
#pragma once
#include <pthread.h>
#include <time.h>

enum GateReq { GATE_LOCK, GATE_RELEASE, GATE_OTHER };   // KMotionLock (or its recovery), ReleaseToken, any other request for the board

static inline double gate_monotonic_s()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

template<int N> struct BoardGates
{
	struct Gate
	{
		int owner;              // the connection (its socket) that holds the token, -1: free
		bool active;            // a request for this board is being carried out
		int waiter;             // a connection whose lock request was refused while another held the token, -1: none
		int reserved;           // the token is kept for this connection, -1: nobody ...
		double reserved_until;  // ... until this time (the gate's clock)
	};
	Gate g[N];
	pthread_mutex_t mx;
	pthread_cond_t cv;
	double (*clock)();          // seconds, monotonic
	int in_use;                 // the result a refused lock request is answered with (KMOTION_IN_USE)
	double hold_s;              // how long the token is kept for a waiter: three of the library's 10 ms retries (WaitToken)

	BoardGates(int in_use_result, double (*clk)() = gate_monotonic_s, double hold = 0.03)
		: clock(clk), in_use(in_use_result), hold_s(hold)
	{
		pthread_mutex_init(&mx, NULL);
		pthread_cond_init(&cv, NULL);
		for (int i = 0; i < N; i++) { g[i].owner = -1; g[i].active = false; g[i].waiter = -1; g[i].reserved = -1; g[i].reserved_until = 0; }
	}

	// before a request for the board from connection c. false: it is answered here with
	// *result and the library is not called
	bool enter(int board, int c, GateReq req, int *result)
	{
		pthread_mutex_lock(&mx);
		Gate &b = g[board];
		for (;;)
		{
			bool others = b.owner != -1 && b.owner != c;
			if (req == GATE_LOCK)
			{
				if (others)
				{
					if (b.waiter == -1) b.waiter = c;       // its turn comes at the release
					*result = in_use;                       // without waiting, and without the device mutex
					pthread_mutex_unlock(&mx);
					return false;
				}
				if (b.owner == -1 && b.reserved != -1 && b.reserved != c && clock() < b.reserved_until)
				{
					*result = in_use;                       // the token is being handed to the waiter
					pthread_mutex_unlock(&mx);
					return false;
				}
			}
			if (others && req == GATE_RELEASE)
			{
				*result = 0;                                // not this connection's token to release
				pthread_mutex_unlock(&mx);
				return false;
			}
			if (!others && !b.active) break;
			pthread_cond_wait(&cv, &mx);
		}
		b.active = true;
		pthread_mutex_unlock(&mx);
		return true;
	}

	// after the library answered the request; granted: a lock request answered KMOTION_LOCKED
	void leave(int board, int c, GateReq req, bool granted)
	{
		pthread_mutex_lock(&mx);
		Gate &b = g[board];
		if (req == GATE_LOCK && granted)
		{
			b.owner = c;
			b.reserved = -1;
			if (b.waiter == c) b.waiter = -1;
		}
		if (req == GATE_RELEASE && b.owner == c) release_locked(b);
		b.active = false;
		pthread_cond_broadcast(&cv);
		pthread_mutex_unlock(&mx);
	}

	// a connection has closed: a token it still held is given back through release(board),
	// under the lock, and its waits and reservations are forgotten
	template<class F> void drop(int c, F release)
	{
		pthread_mutex_lock(&mx);
		for (int i = 0; i < N; i++)
		{
			Gate &b = g[i];
			if (b.owner == c) { release(i); release_locked(b); }
			if (b.waiter == c) b.waiter = -1;
			if (b.reserved == c) b.reserved = -1;
		}
		pthread_cond_broadcast(&cv);
		pthread_mutex_unlock(&mx);
	}

private:
	// the token freed: kept for the waiter, if there is one, for hold_s
	void release_locked(Gate &b)
	{
		b.owner = -1;
		if (b.waiter != -1)
		{
			b.reserved = b.waiter;
			b.reserved_until = clock() + hold_s;
			b.waiter = -1;
		}
	}
};
