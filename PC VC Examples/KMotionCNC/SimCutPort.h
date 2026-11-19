// SimCutPort.h : what SimCut.cpp needs of the operating system
/*********************************************************************/
/*         Copyright (c) 2003-2026  DynoMotion Incorporated          */
/*  MIT License: SimCut-LICENSE.txt (SPDX-License-Identifier: MIT)   */
/*********************************************************************/
//
// For Windows, and for POSIX (Linux): the few calls of the material removal engine that are not
// standard C++.  SimCut.cpp includes it after stdafx.h and SimCut.h; nothing else needs it (file
// names in the engine's interface are SimChar strings, declared in SimCut.h).
//
//   SimHeap             memory for the lines of one set, given back to the system as a whole
//                       (Windows: a heap of its own, HeapCreate; POSIX: an arena mapped from the
//                       system in large blocks, with lists of freed blocks by size)
//   SimGetMemoryStatus  the memory of the machine and what the program may still take
//   SimBelowNormal      the calling thread below the priority of the screen and the Interpreter
//   SimFopen, SimFseek, SimFtell, SimRename, SimRemove
//                       files by SimChar names, 64 bit sizes
//   SimFinite           a number that is neither infinite nor NaN
//   SIM_64BIT           the program has a 64 bit address space

#pragma once

#include <stdio.h>
#include <stddef.h>

#if defined(_WIN64) || (defined(__SIZEOF_POINTER__) && __SIZEOF_POINTER__ == 8)
#define SIM_64BIT 1
#else
#define SIM_64BIT 0
#endif

struct SimMemoryStatus
{
	double totalPhysical;			// bytes of memory in the machine
	double availCommit;				// bytes that the program could still take (memory and page file or swap that are free)
	double totalVirtual;			// address space of the program (what matters in a 32 bit program) ...
	double availVirtual;			// ... and what is left of it
};


#ifdef _WIN32
//////////////////////////////////////////////////////////////////////////////
// Windows (stdafx.h has brought windows.h): the calls as SimCut.cpp has always made them
//////////////////////////////////////////////////////////////////////////////

struct SimHeap;						// a heap of the program's (HANDLE)

static inline SimHeap *SimHeapCreate() { return (SimHeap *)HeapCreate(0, 0, 0); }
static inline void SimHeapDestroy(SimHeap *h) { HeapDestroy((HANDLE)h); }
static inline void *SimHeapAlloc(SimHeap *h, size_t bytes, bool zero) { return HeapAlloc((HANDLE)h, zero ? HEAP_ZERO_MEMORY : 0, bytes); }
static inline void SimHeapFree(SimHeap *h, void *p) { HeapFree((HANDLE)h, 0, p); }

static inline bool SimGetMemoryStatus(SimMemoryStatus *s)
{
	MEMORYSTATUSEX ms;
	ms.dwLength = sizeof(ms);
	if (!GlobalMemoryStatusEx(&ms)) return false;
	s->totalPhysical = (double)ms.ullTotalPhys;
	s->availCommit = (double)ms.ullAvailPageFile;
	s->totalVirtual = (double)ms.ullTotalVirtual;
	s->availVirtual = (double)ms.ullAvailVirtual;
	return true;
}

static inline void SimBelowNormal() { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL); }

static inline FILE *SimFopen(const SimChar *file, const char *mode)
{
	wchar_t wm[8];
	int i;
	for (i = 0; i < 7 && mode[i]; i++) wm[i] = (wchar_t)mode[i];
	wm[i] = 0;
	return _wfopen(file, wm);
}
#define SimFseek _fseeki64
#define SimFtell _ftelli64
static inline bool SimFinite(double x) { return _finite(x) != 0; }
static inline bool SimRename(const SimChar *from, const SimChar *to) { return MoveFileExW(from, to, MOVEFILE_REPLACE_EXISTING) != 0; }
static inline void SimRemove(const SimChar *file) { _wremove(file); }


#else
//////////////////////////////////////////////////////////////////////////////
// POSIX (Linux)
//////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <new>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/resource.h>
#include <sys/syscall.h>

// Memory that goes back to the system as a whole when the heap goes: it is mapped from the system in
// large blocks (mmap) and unmapped by the destructor, never through malloc, which may keep what is
// freed.  A block that is freed before that waits on a list for the next request of its size
// (rounded up to ALIGN bytes): the lines grow and are copied a great deal, in sizes that repeat.
// Every block has a head of HEAD bytes with its size (which is what SIM_BLOCK_BYTES counts).  Any
// thread may allocate and free, under one lock per heap, as the threads of a Win32 heap do.
struct SimHeap
{
	enum { ALIGN = 16, HEAD = 16, ARENA = 4 << 20, CLASSES = 4096 };	// (lists by size for blocks up to 64 KB)
	struct Mapped { void *p; size_t bytes; };

	std::mutex lock;
	std::vector<Mapped> mapped;		// what was taken from the system
	char *cur;						// the arena that is being carved, and what is left of it
	size_t left;
	void *lists[CLASSES];			// freed blocks by size / ALIGN, each pointing at the next
	void *large;					// ... that are larger: one list, their sizes in their heads

	SimHeap() : cur(NULL), left(0), large(NULL) { memset(lists, 0, sizeof(lists)); }
	~SimHeap() { for (size_t i = 0; i < mapped.size(); i++) munmap(mapped[i].p, mapped[i].bytes); }

	char *Map(size_t bytes)
	{
		void *p = mmap(NULL, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (p == MAP_FAILED) return NULL;
		Mapped m = { p, bytes };
		try { mapped.push_back(m); } catch (...) { munmap(p, bytes); return NULL; }
		return (char *)p;
	}

	static size_t Size(const void *p) { return *(const size_t *)((const char *)p - HEAD); }

	void *Alloc(size_t bytes, bool zero)
	{
		size_t n = bytes ? (bytes + ALIGN - 1) & ~(size_t)(ALIGN - 1) : ALIGN;
		std::lock_guard<std::mutex> lk(lock);

		// a block that was freed, of this size
		void *p = NULL;
		if (n / ALIGN < CLASSES)
		{
			void **list = &lists[n / ALIGN];
			if (*list) { p = *list; *list = *(void **)p; }
		}
		else
			for (void **pp = &large; *pp; pp = (void **)*pp)
				if (Size(*pp) == n) { p = *pp; *pp = *(void **)p; break; }
		if (p)
		{
			if (zero) memset(p, 0, n);
			return p;
		}

		// a new one, carved from the arena, or mapped by itself when it is large (what is new from
		// the system is zero, and the arena is carved once: so this is zero without a memset)
		size_t need = n + HEAD;
		char *b;
		if (need > ARENA / 4)
		{
			b = Map(need);
			if (!b) return NULL;
		}
		else
		{
			if (need > left)
			{
				char *a = Map(ARENA);
				if (!a) return NULL;
				cur = a;
				left = ARENA;
			}
			b = cur;
			cur += need;
			left -= need;
		}
		*(size_t *)b = n;
		return b + HEAD;
	}

	void Free(void *p)
	{
		if (!p) return;
		size_t n = Size(p);
		std::lock_guard<std::mutex> lk(lock);
		void **list = n / ALIGN < CLASSES ? &lists[n / ALIGN] : &large;
		*(void **)p = *list;
		*list = p;
	}
};

static inline SimHeap *SimHeapCreate() { return new (std::nothrow) SimHeap; }
static inline void SimHeapDestroy(SimHeap *h) { delete h; }
static inline void *SimHeapAlloc(SimHeap *h, size_t bytes, bool zero) { return h->Alloc(bytes, zero); }
static inline void SimHeapFree(SimHeap *h, void *p) { h->Free(p); }

// From /proc/meminfo (MemAvailable counts what the system can give up: free memory and cache), else
// from sysinfo.  The address space is not looked at: a 64 bit program has more than enough.
static inline bool SimGetMemoryStatus(SimMemoryStatus *s)
{
	double total = -1, avail = -1, swapFree = 0;
	FILE *f = fopen("/proc/meminfo", "r");
	if (f)
	{
		char line[256];
		double kb;
		while (fgets(line, sizeof(line), f))
		{
			if (sscanf(line, "MemTotal: %lf", &kb) == 1) total = kb * 1024.0;
			else if (sscanf(line, "MemAvailable: %lf", &kb) == 1) avail = kb * 1024.0;
			else if (sscanf(line, "SwapFree: %lf", &kb) == 1) swapFree = kb * 1024.0;
		}
		fclose(f);
	}
	if (total < 0 || avail < 0)
	{
		struct sysinfo si;
		if (sysinfo(&si)) return false;
		total = (double)si.totalram * si.mem_unit;
		avail = ((double)si.freeram + (double)si.bufferram) * si.mem_unit;
		swapFree = (double)si.freeswap * si.mem_unit;
	}
	s->totalPhysical = total;
	s->availCommit = avail + swapFree;
	s->totalVirtual = s->availVirtual = 1e18;
	return true;
}

// Nice 5 for this thread (Linux keeps the nice value per thread): below the screen and the
// Interpreter, which run at 0.  Nothing if it is refused.
static inline void SimBelowNormal() { setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 5); }

static inline FILE *SimFopen(const SimChar *file, const char *mode)
{
	char m[8];
	int n = 0;
	for (; *mode && n < 7; mode++)
		if (*mode != 't') m[n++] = *mode;	// ("t", text mode, is Windows' and means nothing here)
	m[n] = 0;
	return fopen(file, m);
}
#define SimFseek fseeko					// (64 bit offsets: in a 32 bit program _FILE_OFFSET_BITS must be 64)
#define SimFtell ftello
static inline bool SimFinite(double x) { return isfinite(x) != 0; }
static inline bool SimRename(const SimChar *from, const SimChar *to) { return rename(from, to) == 0; }
static inline void SimRemove(const SimChar *file) { remove(file); }

#endif
