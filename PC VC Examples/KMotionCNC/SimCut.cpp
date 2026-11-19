// SimCut.cpp : implementation file
/*********************************************************************/
/*         Copyright (c) 2003-2026  DynoMotion Incorporated          */
/*  MIT License: SimCut-LICENSE.txt (SPDX-License-Identifier: MIT)   */
/*********************************************************************/
//
// Material removal for the G Code Viewer (see SimCut.h)

#include "stdafx.h"
#include "SimCut.h"
#include "SimCutPort.h"

#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_CS			32			// cells along a side of a block of the surface (35937 points at most, so 16 bit indices do)
#define SIM_SUB			4			// cells along a side of a sub cell of a face of the Stock: a block has 8 x 8 of them
#define SIM_RELAX		2			// times that the points of the surface are moved toward their neighbours (see MeshBlock)
#define SIM_HALO		(SIM_RELAX + 1)	// cells beyond its own that a block needs the points of, for that and for the directions
#ifdef SIMCUT_DEBUG
int SimDebugRelax = SIM_RELAX;		// (tests: fewer times, to compare)
#define SIM_RELAX_NOW	SimDebugRelax
#else
#define SIM_RELAX_NOW	SIM_RELAX
#endif
#define SIM_ONE_BITS	15
#define SIM_ONE			(1 << SIM_ONE_BITS)	// a position along a line is a whole number, this many to a cell
#define SIM_TOL			8			// the hair (see SimGrid) in those units
#define SIM_EPS			33			// leftovers of a line that are not longer than this are dropped
#define SIM_SHARP		0.02		// faces whose directions differ by less than about 16 degrees count as one curved face
#define SIM_LC_BITS		4
#define SIM_LC			(1 << SIM_LC_BITS)	// lines along a side of a chunk of lines
#define SIM_PG_BITS		6
#define SIM_PG			(1 << SIM_PG_BITS)	// chunks along a side of a page of the table of chunks
#define SIM_MAX_NODES	131072		// nodes along one side at most
#define SIM_AUTO_FINEST	0.001		// resolution to begin with when it is chosen here (the units are inches): the lattice
#define SIM_AUTO_CELLS	1500		// becomes coarser as the memory fills.  A small Stock begins with this many cells along its longest side
#if SIM_64BIT
#define SIM_BUDGET		2000e6		// bytes that the lines and the surface may take before the lattice is made coarser
#else
#define SIM_BUDGET		450e6		// (a 32 bit program with 2 GB of address space for all it does)
#define SIM_BUDGET_SPACE	2147483648.0	// with more address space than this (the program is large address aware: 4 GB under
#define SIM_BUDGET_SHARE	0.27		// a 64 bit Windows, 3 GB under a 32 bit Windows set up for it) this share of what is more
#endif
#define SIM_BUDGET_RAM_SHARE 0.25		// ... and (64 bit) never more than this share of the memory of the machine
#define SIM_BUDGET_AUTO	100e6		// ... when the resolution is chosen here
#define SIM_TRI_BYTES	33.0		// what a triangle of the surface takes (its indices and its share of the points): here until
									// it is drawn, and then where the driver of the graphics card keeps it (measured)
#define SIM_MESH_BYTES	4096.0		// ... and each block of the surface besides
#define SIM_LOW_SPACE	600e6		// (32 bit) when less address space than this is left to the program the Stock gives way
#define SIM_LOW_COMMIT	300e6		// ... or less memory than this to the system
#define SIM_REFINE_WAIT	1000		// milliseconds that a path must have stayed as it is before the lattice is made finer for it
#define SIM_BLOCK_BYTES	16			// what a block of memory takes besides what is in it
#define SIM_WORK_MAX	4e7			// lines looked at between two looks at the memory and at whether the path is still the same
#define SIM_WORK_TIME	0.2			// ... which a tilted tool makes about this many seconds apart (its lines take much longer)
#define SIM_MAX_PIECES	32000		// pieces of material on one line at most
#define SIM_BATCH_TIME	0.08		// seconds of cutting between two updates of the surface
#define SIM_BATCH_LONG	0.3			// ... and when a batch of moves is given up for now because it takes too long
#define SIM_TILT_CELLS	500			// A resolution that is chosen here, and a tool that stands tilted: the lattice begins with this many
									// cells along the longest side of the Stock.
#define SIM_AUTO_REFINE	10.0		// When all of the path is cut it is made finer if cutting it all again takes no more seconds than this.
#define SIM_AUTO_AFTER	1.0			// And after this many seconds of cutting on a lattice, if what is asked for would take more seconds
#define SIM_AUTO_BEHIND	15.0		// than this to cut on it, it is made coarser: so that all of the path that is known takes about
#define SIM_AUTO_SECONDS	6.0		// this long
#ifdef SIMCUT_DEBUG
double SimDebugSlow = 1.0;			// (tests: cutting counts as taking this many times as long as it does)
#define SIM_SPENT(t)	((t) * SimDebugSlow)
#else
#define SIM_SPENT(t)	(t)
#endif

#define PRIM_POINT		0			// the tool stands at a
#define PRIM_FLAT		1			// travels from a to b at one height
#define PRIM_PLUNGE		2			// travels from a to b straight up or down
#define PRIM_TILT		3			// stands at b, tilted: its axis is along d and not along Z

static inline double Min2(double a, double b) { return a < b ? a : b; }
static inline double Max2(double a, double b) { return a > b ? a : b; }
static inline int MinI(int a, int b) { return a < b ? a : b; }
static inline int MaxI(int a, int b) { return a > b ? a : b; }

static double Seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Is the program about to run out of memory?  (A 32 bit program: of address space, which the
// Stock shares with everything else that the program and the driver of the graphics card keep.)
static bool LowMemory()
{
	SimMemoryStatus ms;
	if (!SimGetMemoryStatus(&ms)) return false;
#if !SIM_64BIT
	if (ms.availVirtual < SIM_LOW_SPACE) return true;
#endif
	return ms.availCommit < SIM_LOW_COMMIT;
}

// Bytes that the lines and the surface of the Stock may take before the lattice is made coarser
static double Budget(bool automatic)
{
	if (automatic) return SIM_BUDGET_AUTO;
	SimMemoryStatus ms;
	bool known = SimGetMemoryStatus(&ms);
#if !SIM_64BIT
	if (known && ms.totalVirtual > SIM_BUDGET_SPACE)
		return SIM_BUDGET + SIM_BUDGET_SHARE * (ms.totalVirtual - SIM_BUDGET_SPACE);
	return SIM_BUDGET;
#else
	// (a machine with little memory, a Raspberry Pi: not more than a share of what it has)
	if (known && ms.totalPhysical * SIM_BUDGET_RAM_SHARE < SIM_BUDGET) return ms.totalPhysical * SIM_BUDGET_RAM_SHARE;
	return SIM_BUDGET;
#endif
}


//////////////////////////////////////////////////////////////////////////////
// CSimCutTool
//////////////////////////////////////////////////////////////////////////////

void CSimCutTool::Clear()
{
	h.clear();
	r.clear();
	Rmax = 0.0;
	Open = false;
}

void CSimCutTool::Add(double height, double radius)
{
	h.push_back(height);
	r.push_back(radius > 0.0 ? radius : 0.0);
}

bool CSimCutTool::Finish()
{
	Rmax = 0.0;
	for (size_t i = 0; i < h.size(); i++)
	{
		if (i > 0 && h[i] < h[i - 1]) h[i] = h[i - 1];	// heights never go down
		if (r[i] > Rmax) Rmax = r[i];
	}
	return Valid();
}

bool CSimCutTool::SetFlat(double diameter)
{
	Clear();
	if (!(diameter > 0.0)) return false;
	Add(0.0, diameter / 2.0);
	Add(diameter, diameter / 2.0);
	Open = true;					// whatever holds the tool is above it
	return Finish();
}

int CSimCutTool::BasicParams(int type)
{
	static const int n[SIMTOOL_TYPES] = { 2, 2, 4, 4, 5 };
	return type >= 0 && type < SIMTOOL_TYPES ? n[type] : 0;
}

// A basic tool from its sizes (see SimCut.h).  The profile ends at the length (not Open).
const char *CSimCutTool::SetBasic(int type, const double *p)
{
	const double PI_ = 3.14159265358979323846;
	Clear();

	if (BasicParams(type) == 0) return "Unknown type of Tool";
	double D = p[0], L = p[BasicParams(type) - 1];
	if (!(D > 0.0)) return "Diameter must be greater than zero";
	if (!(L > 0.0)) return "Length must be greater than zero";
	double R = D / 2.0;

	if (type == SIMTOOL_ENDMILL)
	{
		Add(0.0, R);
		Add(L, R);
	}
	else if (type == SIMTOOL_BALL)
	{
		if (L < R) return "Length must be at least half the Diameter";
		Add(0.0, 0.0);
		for (int i = 1; i <= 9; i++)
		{
			double a = i * 10.0 * PI_ / 180.0;
			Add(i == 9 ? R : R * (1.0 - cos(a)), i == 9 ? R : R * sin(a));
		}
		if (L > R) Add(L, R);
	}
	else if (type == SIMTOOL_V)
	{
		double A = p[1], tip = p[2] / 2.0;
		if (!(A > 0.0 && A < 180.0)) return "Angle must be between 0 and 180 degrees";
		if (!(tip >= 0.0 && tip < R)) return "Tip Diameter must be less than the Diameter (0 for a sharp point)";
		double hc = (R - tip) / tan(A / 2.0 * PI_ / 180.0);		// height of the point
		if (L < hc * (1.0 - 1e-9)) return "Length must be at least the height of the point (make it longer or the Angle larger)";
		Add(0.0, tip);
		if (L > hc * (1.0 + 1e-9))
		{
			Add(hc, R);
			Add(L, R);
		}
		else
			Add(L, R);			// all point, no shank
	}
	else if (type == SIMTOOL_TSLOT || type == SIMTOOL_DOVETAIL)
	{
		bool dove = type == SIMTOOL_DOVETAIL;
		double H = p[dove ? 2 : 1], N = p[dove ? 3 : 2] / 2.0, top = R;
		if (!(H > 0.0)) return "Height must be greater than zero";
		if (!(H < L)) return "Length must be greater than the Height";
		if (dove)
		{
			double A = p[1];
			if (!(A > 0.0 && A <= 90.0)) return "Angle must be greater than 0 and at most 90 degrees";
			top = A < 90.0 ? R - H / tan(A * PI_ / 180.0) : R;
			if (!(top > 0.0)) return "The sides meet below the Height (make the Height smaller or the Angle larger)";
		}
		if (!(N > 0.0)) return "Neck Diameter must be greater than zero";
		if (!(N <= top)) return dove ? "Neck Diameter must not be greater than the top of the cutter" : "Neck Diameter must not be greater than the Diameter";
		Add(0.0, R);
		Add(H, top);
		if (N < top) Add(H, N);
		Add(L, N);
	}
	return Finish() ? NULL : "Unknown type of Tool";
}

// The tool as a tool image: the profile turned about Z in steps, closed where it does not
// start or end on the axis.  Triangles only, laid out like the tool images that came from
// Art of Illusion, which is what the viewer reads.
bool CSimCutTool::WriteVrml(const SimChar *file, int sides, const char *comment) const
{
	const double PI_ = 3.14159265358979323846;
	size_t n = h.size();
	if (n < 2 || sides < 3) return false;

	// rows of points going up the outside of the tool: on the axis one point, otherwise a ring
	std::vector<double> ph, pr;
	if (r[0] > 0.0) { ph.push_back(h[0]); pr.push_back(0.0); }
	for (size_t i = 0; i < n; i++)
	{
		if (!ph.empty() && ph.back() == h[i] && pr.back() == r[i]) continue;
		ph.push_back(h[i]);
		pr.push_back(r[i]);
	}
	if (pr.back() > 0.0) { ph.push_back(ph.back()); pr.push_back(0.0); }

	std::vector<int> first(ph.size());
	int nv = 0;
	for (size_t k = 0; k < ph.size(); k++)
	{
		first[k] = nv;
		nv += pr[k] > 0.0 ? sides : 1;
	}

	FILE *f = SimFopen(file, "wt");
	if (!f) return false;
	fprintf(f, "#VRML V2.0 utf8\n");
	if (comment && *comment) fprintf(f, "#%s\n", comment);
	fprintf(f, "DEF Tool Transform {\n translation 0.0 0.0 0.0\n rotation 0.0 0.0 1.0 0.0\n children [\n  Shape {\n"
			   "   appearance Appearance {\n    material Material {\n     diffuseColor 1.0 1.0 1.0\n     emissiveColor 0.0 0.0 0.0\n"
			   "     specularColor 0.0 0.0 0.0\n     shininess 0.8\n     transparency 0.0\n     ambientIntensity 1\n    }\n   }\n"
			   "   geometry DEF Tool IndexedFaceSet {\n    solid TRUE\n    coord Coordinate { point [\n");
	for (size_t k = 0; k < ph.size(); k++)
	{
		if (pr[k] > 0.0)
			for (int i = 0; i < sides; i++)
			{
				double a = 2.0 * PI_ * i / sides, x = pr[k] * cos(a), y = pr[k] * sin(a);
				if (fabs(x) < 5e-7) x = 0.0;		// (no "-0.000000")
				if (fabs(y) < 5e-7) y = 0.0;
				fprintf(f, "     %.6f %.6f %.6f,\n", x, y, ph[k]);
			}
		else
			fprintf(f, "     0.0 0.0 %.6f,\n", ph[k]);
	}
	fprintf(f, "    ] }\n    coordIndex [\n");
	// between a row and the next one the outside is to the right of the way up the profile
	for (size_t k = 0; k + 1 < ph.size(); k++)
	{
		bool ringA = pr[k] > 0.0, ringB = pr[k + 1] > 0.0;
		if (!ringA && !ringB) continue;
		for (int i = 0; i < sides; i++)
		{
			int j = (i + 1) % sides;
			int Ai = first[k] + (ringA ? i : 0), Aj = first[k] + (ringA ? j : 0);
			int Bi = first[k + 1] + (ringB ? i : 0), Bj = first[k + 1] + (ringB ? j : 0);
			if (ringA) fprintf(f, "     %d, %d, %d, -1,\n", Ai, Aj, Bj);
			if (ringB) fprintf(f, "     %d, %d, %d, -1,\n", Ai, Bj, Bi);
		}
	}
	fprintf(f, "    ]\n   }\n  }\n ]\n}\n");
	bool ok = !ferror(f);
	if (fclose(f)) ok = false;
	return ok;
}

// Radius of the tool at a height.  The tool is an open set: a height exactly at its bottom, at its
// closed top or at a step gives what is strictly inside (0, or the smaller radius of the step).
double CSimCutTool::RadiusAt(double height) const
{
	size_t n = h.size();
	if (n < 2 || height <= h[0]) return 0.0;
	if (height > h[n - 1]) return Open ? r[n - 1] : 0.0;

	size_t i = std::lower_bound(h.begin(), h.end(), height) - h.begin();	// first point at or above (i >= 1)
	if (h[i] == height)
	{
		double rr = r[i];
		size_t j = i;
		while (j + 1 < n && h[j + 1] == height) { j++; if (r[j] < rr) rr = r[j]; }
		if (j == n - 1 && !Open) return 0.0;
		return rr;
	}
	return r[i - 1] + (r[i] - r[i - 1]) * (height - h[i - 1]) / (h[i] - h[i - 1]);
}

// Radius of the tool just below a height: at a step the lower of its two radii, at the closed
// top the radius there
double CSimCutTool::RadiusBelow(double height) const
{
	size_t n = h.size();
	if (n < 2 || height <= h[0]) return 0.0;
	if (height > h[n - 1]) return Open ? r[n - 1] : 0.0;

	size_t i = std::lower_bound(h.begin(), h.end(), height) - h.begin();	// first point at or above (i >= 1)
	if (h[i] == height) return r[i];
	return r[i - 1] + (r[i] - r[i - 1]) * (height - h[i - 1]) / (h[i] - h[i - 1]);
}

// Largest radius of the tool strictly between two heights (what a plane sees of a tool that
// travels along its axis)
double CSimCutTool::MaxRadius(double h0, double h1) const
{
	size_t n = h.size();
	if (n < 2) return 0.0;
	double a = h0 > h[0] ? h0 : h[0];
	double top = Open ? SIMCUT_BIG : h[n - 1];
	double b = h1 < top ? h1 : top;
	if (!(a < b)) return 0.0;

	double m = 0.0, v;

	// just above a
	size_t i = std::upper_bound(h.begin(), h.end(), a) - h.begin();		// first point above a (i >= 1)
	if (i == n) v = r[n - 1];
	else if (h[i - 1] == a) v = r[i - 1];
	else v = r[i - 1] + (r[i] - r[i - 1]) * (a - h[i - 1]) / (h[i] - h[i - 1]);
	if (v > m) m = v;

	// just below b
	i = std::lower_bound(h.begin(), h.end(), b) - h.begin();				// first point at or above b (i >= 1)
	if (i == n) v = r[n - 1];
	else if (h[i] == b) v = r[i];
	else v = r[i - 1] + (r[i] - r[i - 1]) * (b - h[i - 1]) / (h[i] - h[i - 1]);
	if (v > m) m = v;

	// the points between
	for (i = 0; i < n; i++)
		if (h[i] > a && h[i] < b && r[i] > m) m = r[i];
	return m;
}

// How fast the radius of the tool grows with the height just below a height (0 where the tool
// is straight, and above an open top)
double CSimCutTool::SlopeBelow(double height) const
{
	size_t n = h.size();
	if (n < 2 || height <= h[0] || height > h[n - 1]) return 0.0;
	size_t i = std::lower_bound(h.begin(), h.end(), height) - h.begin();	// first point at or above (i >= 1)
	return h[i] > h[i - 1] ? (r[i] - r[i - 1]) / (h[i] - h[i - 1]) : 0.0;
}

// Stretches of height where the radius of the tool is greater than d: what a line parallel to
// the axis, at the distance d from it, has inside the tool.  slope, if given, gets for each end
// of a stretch how fast the radius grows with the height where the line passes through the side
// of the tool there, SIMCUT_BIG where it is a face of the tool that looks down and -SIMCUT_BIG
// one that looks up.
int CSimCutTool::Above(double d, double *out, double *slope) const
{
	size_t n = h.size();
	int m = 0;
	for (size_t i = 0; i + 1 < n; i++)
	{
		double h0 = h[i], h1 = h[i + 1], r0 = r[i], r1 = r[i + 1];
		if (!(h0 < h1)) continue;		// a step
		double a, b, sa, sb, s = (r1 - r0) / (h1 - h0);
		if (r0 > d)
		{
			a = h0;
			sa = SIMCUT_BIG;
			if (r1 > d) { b = h1; sb = -SIMCUT_BIG; }
			else { b = h0 + (d - r0) / (r1 - r0) * (h1 - h0); sb = s; }
		}
		else if (r1 > d)
		{
			a = h0 + (d - r0) / (r1 - r0) * (h1 - h0);
			sa = s;
			b = h1;
			sb = -SIMCUT_BIG;
		}
		else
			continue;
		if (!(a < b)) continue;
		if (m > 0 && out[2 * m - 1] == a)	// goes on from the stretch before
		{
			out[2 * m - 1] = b;
			if (slope) slope[2 * m - 1] = sb;
		}
		else
		{
			out[2 * m] = a;
			out[2 * m + 1] = b;
			if (slope) { slope[2 * m] = sa; slope[2 * m + 1] = sb; }
			m++;
		}
	}
	if (Open && n > 0 && r[n - 1] > d)
	{
		if (m > 0 && out[2 * m - 1] == h[n - 1]) out[2 * m - 1] = SIMCUT_BIG;
		else
		{
			out[2 * m] = h[n - 1];
			out[2 * m + 1] = SIMCUT_BIG;
			if (slope) { slope[2 * m] = SIMCUT_BIG; slope[2 * m + 1] = 0.0; }
			m++;
		}
	}
	return m;
}

// The tool image (VRML IndexedFaceSet, the kind the viewer displays): spin the model about the
// Z axis through the tool position.  The radius of the tool at a height is the largest distance
// from the axis of the model's surface at that height, which is always found on an edge of a face.
bool CSimCutTool::SetFromVrml(const SimChar *file, double scale, const double offset[3])
{
	Clear();

	FILE *f = SimFopen(file, "rt");
	if (!f) return false;

	std::vector<double> P;			// x y z of every vertex
	std::vector<int> E;				// two vertices of every edge
	std::vector<int> face;
	double tx = 0, ty = 0, tz = 0, xyz[3];
	int mode = 0, base = 0, nxyz = 0;	// mode 1: reading points, 2: reading the faces
	char line[4096];

	while (fgets(line, sizeof(line), f))
	{
		char *p = line;
		if (mode == 0)
		{
			char *q;
			if (strstr(line, "Transform")) tx = ty = tz = 0;
			if ((q = strstr(line, "translation")) != NULL)
			{
				double a, b, c;
				if (sscanf(q + 11, "%lf %lf %lf", &a, &b, &c) == 3) { tx = a; ty = b; tz = c; }
			}
			if ((q = strstr(line, "Coordinate { point [")) != NULL && !(q - line >= 7 && strncmp(q - 7, "Texture", 7) == 0))
			{
				mode = 1;
				base = (int)(P.size() / 3);
				nxyz = 0;
				p = q + 20;
			}
			else if ((q = strstr(line, "coordIndex [")) != NULL)
			{
				mode = 2;
				face.clear();
				p = q + 12;
			}
			else
				continue;
		}

		// numbers separated by blanks and commas, up to the closing bracket
		while (*p)
		{
			while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\r' || *p == '\n') p++;
			if (!*p) break;
			if (*p == ']') { mode = 0; break; }
			char *end;
			double val = strtod(p, &end);
			if (end == p) { p++; continue; }	// not a number, skip the character
			p = end;
			if (mode == 1)
			{
				xyz[nxyz++] = val;
				if (nxyz == 3)
				{
					P.push_back((xyz[0] + tx) * scale + offset[0]);
					P.push_back((xyz[1] + ty) * scale + offset[1]);
					P.push_back((xyz[2] + tz) * scale + offset[2]);
					nxyz = 0;
				}
			}
			else
			{
				int vi = (int)val;
				if (vi >= 0) face.push_back(base + vi);
				else
				{
					for (size_t k = 0; k < face.size(); k++)
					{
						E.push_back(face[k]);
						E.push_back(face[(k + 1) % face.size()]);
					}
					face.clear();
				}
			}
		}
	}
	fclose(f);

	int nv = (int)(P.size() / 3);
	if (nv < 3 || E.empty()) return false;

	// edges with their lower end first, ignoring those that lie at one height or name a missing vertex
	struct Edge { double z0, x0, y0, z1, x1, y1; };
	std::vector<Edge> edges;
	std::vector<double> hs;
	for (size_t k = 0; k + 1 < E.size(); k += 2)
	{
		int a = E[k], b = E[k + 1];
		if (a < 0 || b < 0 || a >= nv || b >= nv) continue;
		if (P[3 * a + 2] > P[3 * b + 2]) std::swap(a, b);
		Edge e = { P[3 * a + 2], P[3 * a], P[3 * a + 1], P[3 * b + 2], P[3 * b], P[3 * b + 1] };
		hs.push_back(e.z0);
		hs.push_back(e.z1);
		if (e.z1 > e.z0) edges.push_back(e);
	}
	if (edges.empty()) return false;

	// the heights where something of the model changes
	std::sort(hs.begin(), hs.end());
	double range = hs.back() - hs.front();
	if (!(range > 0.0)) return false;
	double tol = range * 1e-6;
	std::vector<double> H;
	for (size_t k = 0; k < hs.size(); k++)
		if (H.empty() || hs[k] - H.back() > tol) H.push_back(hs[k]);

	// between two such heights the radius is taken from the edges that pass through
	for (size_t k = 0; k + 1 < H.size(); k++)
	{
		double za = H[k], zb = H[k + 1], zm = (za + zb) / 2.0;
		double ra = 0, rb = 0, rm = 0;
		for (size_t e = 0; e < edges.size(); e++)
		{
			const Edge &g = edges[e];
			if (g.z0 > za + tol || g.z1 < zb - tol) continue;
			double dz = g.z1 - g.z0;
			for (int s = 0; s < 3; s++)
			{
				double z = s == 0 ? za : s == 1 ? zb : zm;
				double t = (z - g.z0) / dz;
				if (t < 0) t = 0; else if (t > 1) t = 1;
				double x = g.x0 + t * (g.x1 - g.x0), y = g.y0 + t * (g.y1 - g.y0);
				double rr = sqrt(x * x + y * y);
				if (s == 0) { if (rr > ra) ra = rr; }
				else if (s == 1) { if (rr > rb) rb = rr; }
				else { if (rr > rm) rm = rr; }
			}
		}
		Add(za, ra);
		Add(zm, rm);
		Add(zb, rb);
	}

	// one point where two meet with the same radius, and none on a straight stretch
	double big = 0;
	for (size_t k = 0; k < r.size(); k++) if (r[k] > big) big = r[k];
	if (!(big > 0.0)) { Clear(); return false; }
	for (double rtol = big * 1e-4; ; rtol *= 4.0)
	{
		for (size_t k = 0; k + 1 < h.size();)
		{
			if (h[k + 1] == h[k] && fabs(r[k + 1] - r[k]) <= rtol)
			{
				h.erase(h.begin() + k + 1);
				r.erase(r.begin() + k + 1);
			}
			else
				k++;
		}
		for (size_t k = 1; k + 1 < h.size();)
		{
			if (h[k] > h[k - 1] && h[k + 1] > h[k])
			{
				double v = r[k - 1] + (r[k + 1] - r[k - 1]) * (h[k] - h[k - 1]) / (h[k + 1] - h[k - 1]);
				if (fabs(v - r[k]) <= rtol)
				{
					h.erase(h.begin() + k);
					r.erase(r.begin() + k);
					continue;
				}
			}
			k++;
		}
		if (h.size() <= 400) break;		// a very detailed model: accept a little less detail
	}

	Open = true;					// whatever holds the tool is above it
	return Finish();
}


//////////////////////////////////////////////////////////////////////////////
// Material on one line of the lattice
//////////////////////////////////////////////////////////////////////////////

// The direction of the surface at a point, as a code: 0 = not known, otherwise the unit vector
// that points out of the material (the octahedron with 255 x 255 steps: the three axes are exact)
static inline unsigned short DirCode(double x, double y, double z)
{
	double s = fabs(x) + fabs(y) + fabs(z);
	if (!(s > 0.0)) return 0;
	double px = x / s, py = y / s;
	if (z < 0.0)
	{
		double qx = (1.0 - fabs(py)) * (px >= 0.0 ? 1.0 : -1.0), qy = (1.0 - fabs(px)) * (py >= 0.0 ? 1.0 : -1.0);
		px = qx;
		py = qy;
	}
	int ix = (int)floor((px + 1.0) * 127.0 + 0.5), iy = (int)floor((py + 1.0) * 127.0 + 0.5);
	if (ix < 0) ix = 0; else if (ix > 254) ix = 254;
	if (iy < 0) iy = 0; else if (iy > 254) iy = 254;
	return (unsigned short)(1 + ix + 255 * iy);
}

struct SimDirTable
{
	float v[255 * 255 + 1][3];
	SimDirTable()
	{
		v[0][0] = v[0][1] = v[0][2] = 0;
		for (int iy = 0; iy < 255; iy++)
			for (int ix = 0; ix < 255; ix++)
			{
				double px = ix / 127.0 - 1.0, py = iy / 127.0 - 1.0, pz = 1.0 - fabs(px) - fabs(py);
				if (pz < 0.0)
				{
					double qx = (1.0 - fabs(py)) * (px >= 0.0 ? 1.0 : -1.0), qy = (1.0 - fabs(px)) * (py >= 0.0 ? 1.0 : -1.0);
					px = qx;
					py = qy;
				}
				double l = sqrt(px * px + py * py + pz * pz);
				float *o = v[1 + ix + 255 * iy];
				o[0] = (float)(px / l);
				o[1] = (float)(py / l);
				o[2] = (float)(pz / l);
			}
	}
};
static const SimDirTable g_Dir;

// ... of the surface that the side of a tool leaves: (ux, uy) points from the axis of the tool to
// the point, slope is how fast the radius grows with the height there (see CSimCutTool::Above)
static inline unsigned short ToolDir(double ux, double uy, double slope)
{
	if (slope >= SIMCUT_BIG / 2) return DirCode(0, 0, 1);
	if (slope <= -SIMCUT_BIG / 2) return DirCode(0, 0, -1);
	return DirCode(-ux, -uy, slope);
}

// Positions along a line are counted from its first node in steps of 1 / SIM_ONE of the
// distance between two nodes.  So they are exact, and as fine, wherever the Stock lies and
// however large it is (a float in the coordinates of the viewer is not: 4 millionths at 36).
typedef unsigned int SimPos;

// The lines of one set take their memory from a heap of their own (see SimLines).  For one job
// that cuts them:
struct SimRoom
{
	SimHeap *heap;
	long long grown;				// bytes that the job took for pieces
};

struct SimDex
{
	unsigned short n;				// pieces of material, in ascending order
	unsigned short cap;				// room for pieces in v (0: the one piece is kept in place)
	union
	{
		struct { SimPos lo, hi; unsigned short nlo, nhi; unsigned char flo, fhi; } one;
		SimPos *v;					// lo and hi of each piece, then the direction of the surface at each end (a code,
									// 0 = not known), then what made each end (SIMKIND_xxx)
	};

	void Init(SimPos lo, SimPos hi, unsigned short nlo = 0, unsigned short nhi = 0)
	{
		n = 1;
		cap = 0;
		one.lo = lo;
		one.hi = hi;
		one.nlo = nlo;
		one.nhi = nhi;
		one.flo = one.fhi = SIMKIND_STOCK;
	}
	unsigned short *N() { return (unsigned short *)(v + 2 * cap); }
	unsigned char *F() { return (unsigned char *)(v + 2 * cap) + 4 * cap; }
	SimPos Lo(int i) const { return cap ? v[2 * i] : one.lo; }
	SimPos Hi(int i) const { return cap ? v[2 * i + 1] : one.hi; }
	unsigned short NLo(int i) const { return cap ? ((const unsigned short *)(v + 2 * cap))[2 * i] : one.nlo; }
	unsigned short NHi(int i) const { return cap ? ((const unsigned short *)(v + 2 * cap))[2 * i + 1] : one.nhi; }
	unsigned char FLo(int i) const { return cap ? ((const unsigned char *)(v + 2 * cap) + 4 * cap)[2 * i] : one.flo; }
	unsigned char FHi(int i) const { return cap ? ((const unsigned char *)(v + 2 * cap) + 4 * cap)[2 * i + 1] : one.fhi; }

	bool Grow(int pieces, SimRoom *room);
	bool Cut(SimPos a, SimPos b, unsigned char flag, unsigned short na, unsigned short nb, SimRoom *room);
	bool Copy(const SimDex &s, SimRoom *room);
	bool Same(const SimDex &s) const;
};

#define SIM_PIECE_BYTES (2 * (sizeof(SimPos) + sizeof(unsigned short) + 1))		// of a piece that is not kept in place

bool SimDex::Grow(int pieces, SimRoom *room)
{
	if (pieces > SIM_MAX_PIECES) return false;
	int nc = cap ? cap * 2 : 2;
	if (nc < pieces) nc = pieces;
	if (nc > SIM_MAX_PIECES) nc = SIM_MAX_PIECES;
	SimPos *nv = (SimPos *)SimHeapAlloc(room->heap, (size_t)nc * SIM_PIECE_BYTES, false);
	if (!nv) return false;
	unsigned short *nn = (unsigned short *)(nv + 2 * nc);
	unsigned char *nf = (unsigned char *)(nv + 2 * nc) + 4 * nc;
	room->grown += (long long)(nc - cap) * (long long)SIM_PIECE_BYTES + (cap ? 0 : SIM_BLOCK_BYTES);
	if (cap)
	{
		memcpy(nv, v, (size_t)n * 2 * sizeof(SimPos));
		memcpy(nn, N(), (size_t)n * 2 * sizeof(unsigned short));
		memcpy(nf, F(), (size_t)n * 2);
		SimHeapFree(room->heap, v);
	}
	else if (n)
	{
		nv[0] = one.lo;
		nv[1] = one.hi;
		nn[0] = one.nlo;
		nn[1] = one.nhi;
		nf[0] = one.flo;
		nf[1] = one.fhi;
	}
	v = nv;
	cap = (unsigned short)nc;
	return true;
}

// The pieces of another line, which may be of another heap, in place of what this line has
// (nothing that is to be freed)
bool SimDex::Copy(const SimDex &s, SimRoom *room)
{
	cap = 0;
	n = s.n;
	if (n == 0) return true;
	if (n == 1)
	{
		one.lo = s.Lo(0);
		one.hi = s.Hi(0);
		one.nlo = s.NLo(0);
		one.nhi = s.NHi(0);
		one.flo = s.FLo(0);
		one.fhi = s.FHi(0);
		return true;
	}
	int nc = n;
	SimPos *nv = (SimPos *)SimHeapAlloc(room->heap, (size_t)nc * SIM_PIECE_BYTES, false);
	if (!nv)
	{
		n = 0;
		return false;
	}
	memcpy(nv, s.v, (size_t)nc * 2 * sizeof(SimPos));
	memcpy(nv + 2 * nc, s.v + 2 * s.cap, (size_t)nc * 2 * sizeof(unsigned short));
	memcpy((unsigned char *)(nv + 2 * nc) + 4 * nc, (const unsigned char *)(s.v + 2 * s.cap) + 4 * s.cap, (size_t)nc * 2);
	v = nv;
	cap = (unsigned short)nc;
	room->grown += (long long)nc * (long long)SIM_PIECE_BYTES + SIM_BLOCK_BYTES;
	return true;
}

bool SimDex::Same(const SimDex &s) const
{
	if (n != s.n) return false;
	for (int i = 0; i < n; i++)
		if (Lo(i) != s.Lo(i) || Hi(i) != s.Hi(i) || NLo(i) != s.NLo(i) || NHi(i) != s.NHi(i) || FLo(i) != s.FLo(i) || FHi(i) != s.FHi(i)) return false;
	return true;
}

// Remove the material between a and b.  What is left of a piece gets the flag at its new end
// and the direction of the surface there (na at a, nb at b); leftovers that are not longer than
// SIM_EPS are dropped.
bool SimDex::Cut(SimPos a, SimPos b, unsigned char flag, unsigned short na, unsigned short nb, SimRoom *room)
{
	if (n == 0 || !(a < b)) return false;

	if (cap == 0)
	{
		SimPos lo = one.lo, hi = one.hi;
		if (b <= lo || a >= hi) return false;
		bool keepL = a > lo + SIM_EPS, keepR = b + SIM_EPS < hi;
		if (keepL && keepR)
		{
			unsigned char fl = one.flo, fh = one.fhi;
			unsigned short nl = one.nlo, nh = one.nhi;
			if (!Grow(2, room)) return false;
			unsigned short *nn = N();
			unsigned char *f = F();
			v[0] = lo; v[1] = a; v[2] = b; v[3] = hi;
			nn[0] = nl; nn[1] = na; nn[2] = nb; nn[3] = nh;
			f[0] = fl; f[1] = flag; f[2] = flag; f[3] = fh;
			n = 2;
		}
		else if (keepL) { one.hi = a; one.fhi = flag; one.nhi = na; }
		else if (keepR) { one.lo = b; one.flo = flag; one.nlo = nb; }
		else n = 0;
		return true;
	}

	// first piece that reaches beyond a
	int i0;
	if (n > 8)
	{
		int lo = 0, hi = n;
		while (lo < hi)
		{
			int mid = (lo + hi) / 2;
			if (v[2 * mid + 1] <= a) lo = mid + 1; else hi = mid;
		}
		i0 = lo;
	}
	else
		for (i0 = 0; i0 < n && v[2 * i0 + 1] <= a; i0++);
	if (i0 == n || v[2 * i0] >= b) return false;

	// last piece that starts before b
	int i1 = i0;
	while (i1 + 1 < n && v[2 * (i1 + 1)] < b) i1++;

	unsigned short *nn = N();
	unsigned char *f = F();
	SimPos lo0 = v[2 * i0], hi1 = v[2 * i1 + 1];
	unsigned char fl0 = f[2 * i0], fh1 = f[2 * i1 + 1];
	unsigned short nl0 = nn[2 * i0], nh1 = nn[2 * i1 + 1];
	bool keepL = a > lo0 + SIM_EPS, keepR = b + SIM_EPS < hi1;
	int removed = i1 - i0 + 1, added = (keepL ? 1 : 0) + (keepR ? 1 : 0);
	int count = n - removed + added;

	if (count > cap)
	{
		if (!Grow(count, room)) return false;
		nn = N();
		f = F();
	}
	if (added != removed)
	{
		int tail = n - (i1 + 1);
		if (tail > 0)
		{
			memmove(v + 2 * (i0 + added), v + 2 * (i1 + 1), (size_t)tail * 2 * sizeof(SimPos));
			memmove(nn + 2 * (i0 + added), nn + 2 * (i1 + 1), (size_t)tail * 2 * sizeof(unsigned short));
			memmove(f + 2 * (i0 + added), f + 2 * (i1 + 1), (size_t)tail * 2);
		}
	}
	int k = i0;
	if (keepL)
	{
		v[2 * k] = lo0; v[2 * k + 1] = a;
		nn[2 * k] = nl0; nn[2 * k + 1] = na;
		f[2 * k] = fl0; f[2 * k + 1] = flag;
		k++;
	}
	if (keepR)
	{
		v[2 * k] = b; v[2 * k + 1] = hi1;
		nn[2 * k] = nb; nn[2 * k + 1] = nh1;
		f[2 * k] = flag; f[2 * k + 1] = fh1;
	}
	n = (unsigned short)count;
	return true;
}


//////////////////////////////////////////////////////////////////////////////
// The lattice
//////////////////////////////////////////////////////////////////////////////

struct SimSlice						// a tool that stands tilted is a stack of these along its axis (MakeSlices)
{
	double h0, h1;					// heights above the tool tip (h1 SIMCUT_BIG: without end)
	double r0, k;					// radius at h0, and how fast it grows with the height
};

struct SimPrim						// one elementary cut
{
	double a[3], b[3];				// tool tip from a to b (the same for PRIM_POINT)
	const CSimCutTool *T;
	double R;						// how far from its axis the tool can cut in this Stock (MakePrim)
	unsigned char flag;				// SIMKIND_FEED or SIMKIND_RAPID
	unsigned char kind;				// PRIM_xxx
	int lo[3], hi[3];				// nodes of the lattice that it may reach
	// PRIM_TILT:
	double d[3];					// which way the tool points, from its tip toward the spindle (a unit vector)
	double t0, t1;					// the stretch of the tool that is in the Stock, as heights above its tip (MakePrim)
	const std::vector<SimSlice> *slices;
};

struct SimChunk						// SIM_LC x SIM_LC lines that lie next to one another
{
	SimDex d[SIM_LC * SIM_LC];
	int live;						// lines of it that still hold material (of those that are lines of the lattice):
									// none, and whoever cuts passes all of the chunk by at once
};

struct SimPage						// SIM_PG x SIM_PG chunks
{
	SimChunk *c[SIM_PG * SIM_PG];
};

// The lines along one axis.  They are kept in chunks, and a chunk exists only where something
// was cut: every other line is uncut.  So the memory goes with what is cut and not with the
// size of the Stock.  Line (u, v): u counts along the axis that follows the one of the lines
// (X, Y, Z in turn) and v along the one after that.
//
// A Stock that was loaded from a file has the lines as the cuts of the file left them (base).
// They stay as they are: where there is no chunk here the chunk of those lines is read, and a
// chunk that is made here to be cut begins as a copy of it.  So the path is cut into what the
// file left, and when the chunks here go the lines are as the file left them again.
//
// All of it comes from a heap that the lines have for themselves, and that goes back to the
// system as a whole when the lines go (a coarser lattice, another Stock, a path cut anew).
// The heap that the program shares keeps what it once had as address space of its own, and in a
// 32 bit program a Stock that had once grown large and was long gone left none for anything
// else: the driver of the graphics card was the first to find no room, and the viewer stayed blank.
struct SimLines
{
	int nu, nv;						// lines
	int pu, pv;						// pages of the table of chunks
	std::atomic<SimPage *> *top;
	SimPos lo, hi;					// an uncut line: from the first node (0) to the last
	double o, inv;					// where the first node is, and SIM_ONE over the distance between two nodes
	SimDex full;					// (what is read where there is no chunk)
	std::atomic<long long> chunks, pages;	// that exist
	std::atomic<long long> pieceBytes;	// bytes of the pieces that are not kept in place
	SimHeap *heap;					// where the pages, the chunks and the pieces are
	const SimLines *base;			// the lines as they were before the path began to cut (NULL: uncut)

	SimLines() : nu(0), nv(0), pu(0), pv(0), top(NULL), lo(0), hi(0), o(0), inv(1), chunks(0), pages(0), pieceBytes(0), heap(NULL), base(NULL) { full.Init(0, 0); }
	~SimLines()
	{
		if (heap) SimHeapDestroy(heap);
		delete[] top;
	}

	// n nodes along the lines, the first at origin, g apart
	void Setup(int u, int v, int n, double origin, double g, unsigned short nl, unsigned short nh)
	{
		nu = u;
		nv = v;
		lo = 0;
		hi = (SimPos)(n - 1) << SIM_ONE_BITS;
		o = origin;
		inv = SIM_ONE / g;
		full.Init(lo, hi, nl, nh);
		pu = ((nu - 1) >> (SIM_LC_BITS + SIM_PG_BITS)) + 1;
		pv = ((nv - 1) >> (SIM_LC_BITS + SIM_PG_BITS)) + 1;
		top = new std::atomic<SimPage *>[(size_t)pu * pv];
		for (size_t i = 0; i < (size_t)pu * pv; i++) top[i].store(NULL, std::memory_order_relaxed);
		heap = SimHeapCreate();
		if (!heap) throw std::bad_alloc();
	}

	// every line as it was before the path began to cut (uncut, or as the lines of base have it):
	// the memory of the chunks here goes back to the system at once
	void Clear()
	{
		if (!top) return;
		for (size_t i = 0; i < (size_t)pu * pv; i++) top[i].store(NULL, std::memory_order_relaxed);
		if (heap) SimHeapDestroy(heap);
		heap = SimHeapCreate();		// (if there is none MakeChunk says so)
		chunks = 0;
		pages = 0;
		pieceBytes = 0;
	}

	SimChunk *Chunk(int cu, int cv) const		// NULL: nothing was cut there
	{
		SimPage *pg = top[(size_t)(cv >> SIM_PG_BITS) * pu + (cu >> SIM_PG_BITS)].load(std::memory_order_acquire);
		return pg ? pg->c[((cv & (SIM_PG - 1)) << SIM_PG_BITS) | (cu & (SIM_PG - 1))] : NULL;
	}

	const SimChunk *BaseChunk(int cu, int cv) const		// ... as the lines of before the path have it
	{
		return base ? base->Chunk(cu, cv) : NULL;
	}

	// The chunk, made if there is none.  Several threads may do this at the same time as long
	// as no two of them work on the same chunk (the pages they share).
	SimChunk *MakeChunk(int cu, int cv)
	{
		std::atomic<SimPage *> &slot = top[(size_t)(cv >> SIM_PG_BITS) * pu + (cu >> SIM_PG_BITS)];
		SimPage *pg = slot.load(std::memory_order_acquire);
		if (!heap) throw std::bad_alloc();
		if (!pg)
		{
			SimPage *np = (SimPage *)SimHeapAlloc(heap, sizeof(SimPage), true);
			if (!np) throw std::bad_alloc();
			if (slot.compare_exchange_strong(pg, np))
			{
				pg = np;
				pages++;
			}
			else
				SimHeapFree(heap, np);	// another thread was first: pg is its page
		}
		SimChunk *&c = pg->c[((cv & (SIM_PG - 1)) << SIM_PG_BITS) | (cu & (SIM_PG - 1))];
		if (!c)
		{
			SimChunk *nc = (SimChunk *)SimHeapAlloc(heap, sizeof(SimChunk), false);
			if (!nc) throw std::bad_alloc();
			const SimChunk *bc = BaseChunk(cu, cv);
			if (bc)
			{
				// the lines as they were before the path: a copy of them to cut into
				SimRoom room = { heap, 0 };
				for (int l = 0; l < SIM_LC * SIM_LC; l++)
					if (!nc->d[l].Copy(bc->d[l], &room)) throw std::bad_alloc();
				pieceBytes += room.grown;
			}
			else
				for (int l = 0; l < SIM_LC * SIM_LC; l++) nc->d[l].Init(lo, hi, full.one.nlo, full.one.nhi);
			Count(nc, cu, cv);
			c = nc;
			chunks++;
		}
		return c;
	}

	static int At(int u, int v) { return ((v & (SIM_LC - 1)) << SIM_LC_BITS) | (u & (SIM_LC - 1)); }

	// how many lines of a chunk hold material (a chunk at the edge has lines that are not lines of the lattice)
	void Count(SimChunk *c, int cu, int cv) const
	{
		int live = 0;
		for (int l = 0; l < SIM_LC * SIM_LC; l++)
			if (c->d[l].n && ((cu << SIM_LC_BITS) | (l & (SIM_LC - 1))) < nu && ((cv << SIM_LC_BITS) | (l >> SIM_LC_BITS)) < nv) live++;
		c->live = live;
	}

	// a coordinate of the viewer as a position along the lines (what is beyond an end of the line is at that end)
	SimPos Fix(double x) const
	{
		double f = (x - o) * inv + 0.5;
		return f <= 0.0 ? 0 : f >= (double)hi ? hi : (SimPos)f;
	}

	const SimDex &Get(int u, int v) const		// a line to read
	{
		const SimChunk *c = Chunk(u >> SIM_LC_BITS, v >> SIM_LC_BITS);
		if (!c && base) c = base->Chunk(u >> SIM_LC_BITS, v >> SIM_LC_BITS);
		return c ? c->d[At(u, v)] : full;
	}

	// The lines as they are go to d, which has none, and stay there as they are.  What is left
	// here reads them wherever it has no chunk of its own, and what is cut from now on is cut
	// into copies of their chunks.  nt and nh: a table of chunks and a heap for what is left here.
	void Freeze(SimLines &d, std::atomic<SimPage *> *nt, SimHeap *nh)
	{
		d.nu = nu;
		d.nv = nv;
		d.pu = pu;
		d.pv = pv;
		d.lo = lo;
		d.hi = hi;
		d.o = o;
		d.inv = inv;
		d.full = full;
		d.top = top;
		d.heap = heap;
		d.chunks = chunks.load();
		d.pages = pages.load();
		d.pieceBytes = pieceBytes.load();
		for (size_t i = 0; i < (size_t)pu * pv; i++) nt[i].store(NULL, std::memory_order_relaxed);
		top = nt;
		heap = nh;
		chunks = 0;
		pages = 0;
		pieceBytes = 0;
		base = &d;
	}

	double Bytes() const
	{
		return (double)chunks * (sizeof(SimChunk) + SIM_BLOCK_BYTES) + (double)pages * (sizeof(SimPage) + SIM_BLOCK_BYTES) + (double)pieceBytes;
	}
};

// Every chunk of the lines: their own, or where they have none the one of the lines of before
// the path.  fn(cu, cv, chunk)
template <class F> static void EachChunk(const SimLines &L, F fn)
{
	for (int pv = 0; pv < L.pv; pv++)
		for (int pu = 0; pu < L.pu; pu++)
		{
			const SimPage *pg = L.top[(size_t)pv * L.pu + pu].load();
			const SimPage *bg = L.base ? L.base->top[(size_t)pv * L.pu + pu].load() : NULL;
			if (!pg && !bg) continue;
			for (int c = 0; c < SIM_PG * SIM_PG; c++)
			{
				const SimChunk *ch = pg ? pg->c[c] : NULL;
				if (!ch && bg) ch = bg->c[c];
				if (ch) fn((pu << SIM_PG_BITS) | (c & (SIM_PG - 1)), (pv << SIM_PG_BITS) | (c >> SIM_PG_BITS), ch);
			}
		}
}

// The lines of a Stock as the cuts of a Stock file left them, on one lattice, and where the
// surface is built in detail for them.  The Stock that was loaded reads them wherever the path
// has not cut (SimLines::base): they are there once, however often the path is cut anew.
struct SimFrozen
{
	int n[3];						// nodes of the lattice that they are of
	double lo[3], hi[3];			// ... and its Stock
	double res, finest;				// resolution of that lattice, and the finest that could be had when it was made
	SimLines S[3];
	std::vector<unsigned long long> key;	// blocks whose surface is built in detail
	std::unordered_map<unsigned long long, unsigned long long> fine[6];	// ... and the sub cells of the faces of the Stock
	double Bytes() const { return S[0].Bytes() + S[1].Bytes() + S[2].Bytes(); }
};

#define SIM_SKIN (~0ULL)			// in place of a block: the mesh is a part of the skin

struct SimGrid
{
	double o[3], g[3];				// first node and distance between nodes
	int n[3];						// nodes along X, Y and Z
	SimLines S[3];					// the lines along X: (j, k),  along Y: (k, i),  along Z: (i, j)
	int nc[3];						// blocks of the surface along X, Y and Z

	// The surface is built in detail only in the blocks that a cut may have reached.  Each of
	// them has a mesh, a slot in the list that is handed over.  What is left of the faces of
	// the Stock is covered by the skin: a few rectangles, in slots of their own.
	std::unordered_map<unsigned long long, int> slotOf;
	std::vector<unsigned long long> key;	// of each slot: its block, or SIM_SKIN
	std::vector<unsigned char> dirty;		// the surface of the slot is to be built again
	std::vector<int> tris;					// triangles of the slot as last built
	// A face of the Stock is built in detail only in the sub cells that a cut may have reached
	// (the skin has the rest of it, also inside a block that is cut elsewhere)
	std::vector<unsigned char> face[6];		// the faces -X +X -Y +Y -Z +Z: the blocks on them that have such sub cells
	std::unordered_map<unsigned long long, unsigned long long> fine[6];	// ... and which of the 8 x 8 sub cells of a block they are
	int ns[3];						// sub cells along X, Y and Z
	std::vector<int> skinSlots;
	bool anyDirty, skinDirty;
	double triangles;				// of all the slots

	double guessed;

	SimFrozen *frozen;				// the lines as the cuts of before the path left them (NULL: the path cuts an uncut Stock)

	std::atomic<bool> over;			// the lines take more memory than there is for the Stock (Over)

	double boxLo[3], boxHi[3];		// the Stock as it was asked for
	double wanted;					// resolution asked for (0: chosen here)
	double res;						// resolution in use
	double finest;					// the finest that this Stock can have
	double budget;					// bytes before the lattice is made coarser
	int coarsened;					// times the lattice was made anew (coarser, or finer again)
	bool tilted;					// a tool that stands tilted has cut in it

	// A face of a tool or of the Stock that lies exactly on a plane of the lattice must look the
	// same to the lines along X and Y in that plane and to the lines along Z through it.  Two
	// things see to that.  The planes of the lattice count as lying a hair (tol) away from their
	// height, the lowest one above it and all the others below (so the bottom and the top face
	// of the Stock are a hair outside of them): a tool that only touches a plane from above has
	// not cut in it, and where the radius of a tool jumps (the top of a dovetail or T slot
	// cutter) the plane sees what is below the jump.  And a tool cuts a hair (infl) beyond its
	// radius: two cuts that just touch leave no wall of no thickness between them.  Both are far
	// below what can be seen.
	double tol, infl;
	double tolZ;					// the hair along Z: exactly SIM_TOL of the positions along a line
	double PlaneZ(int k) const { return o[2] + k * g[2] + (k == 0 ? tolZ : -tolZ); }

	// A tool, or the tip of one, that is thinner than the lattice would only cut the few lines
	// that it happens to pass: a row of dots where it went.  Wherever a tool is thinner than
	// this it cuts as if it had this radius, which leaves a groove without gaps in any direction
	// (no point is farther than 0.71 of the distance between the lines from one of them).
	double rmin;
	double rminTilt;				// ... for a tool that stands tilted (it passes the lines along Z as well)

	double C(int a, int i) const { return o[a] + i * g[a]; }

	static unsigned long long Key(int bx, int by, int bz)
	{
		return (unsigned long long)bx | ((unsigned long long)by << 21) | ((unsigned long long)bz << 42);
	}

	// no block built in detail: the skin covers every face
	void ResetBlocks()
	{
		slotOf.clear();
		key.clear();
		dirty.clear();
		tris.clear();
		skinSlots.clear();
		for (int f = 0; f < 6; f++)
		{
			std::fill(face[f].begin(), face[f].end(), (unsigned char)0);
			fine[f].clear();
		}
		triangles = 0;
		anyDirty = skinDirty = true;
	}

	// the surface of this block is to be built (again)
	void Touch(int bx, int by, int bz)
	{
		unsigned long long k = Key(bx, by, bz);
		std::unordered_map<unsigned long long, int>::iterator it = slotOf.find(k);
		if (it != slotOf.end())
			dirty[it->second] = 1;
		else
		{
			slotOf[k] = (int)key.size();
			key.push_back(k);
			dirty.push_back(1);
			tris.push_back(0);
		}
		anyDirty = true;
	}

	// these sub cells of a face are built in detail from now on
	void MarkFace(int f, int u0, int u1, int v0, int v1)
	{
		int u = (f / 2 + 1) % 3;
		for (int bv = v0 >> 3; bv <= (v1 >> 3); bv++)
			for (int bu = u0 >> 3; bu <= (u1 >> 3); bu++)
			{
				unsigned long long bits = 0;
				for (int t = MaxI(v0, bv << 3); t <= MinI(v1, (bv << 3) + 7); t++)
					for (int s = MaxI(u0, bu << 3); s <= MinI(u1, (bu << 3) + 7); s++)
						bits |= 1ULL << (((t & 7) << 3) | (s & 7));
				unsigned long long &m = fine[f][Key(bu, bv, 0)];
				if ((m | bits) != m)
				{
					m |= bits;
					face[f][(size_t)bv * nc[u] + bu] = 1;
					skinDirty = anyDirty = true;
				}
			}
	}

	bool Detailed(int f, int s, int t) const		// is this sub cell of a face built in detail?
	{
		int a = f / 2, u = (a + 1) % 3, v = (a + 2) % 3;
		if (s < 0 || t < 0 || s >= ns[u] || t >= ns[v]) return false;
		if (!face[f][(size_t)(t >> 3) * nc[u] + (s >> 3)]) return false;
		std::unordered_map<unsigned long long, unsigned long long>::const_iterator it = fine[f].find(Key(s >> 3, t >> 3, 0));
		return it != fine[f].end() && ((it->second >> (((t & 7) << 3) | (s & 7))) & 1);
	}

	// everything is to be handed over again
	void MarkAll()
	{
		std::fill(dirty.begin(), dirty.end(), (unsigned char)1);
		anyDirty = skinDirty = true;
	}

	// as before the path began to cut: uncut, or as the cuts of the Stock file left it
	void Fill()
	{
		for (int s = 0; s < 3; s++) S[s].Clear();
		ResetBlocks();
		UseLayout();
		over = false;
	}

	// the blocks and the sub cells that the lines of before the path are built in detail in
	void UseLayout()
	{
		if (!frozen) return;
		for (size_t i = 0; i < frozen->key.size(); i++)
		{
			unsigned long long k = frozen->key[i];
			Touch((int)(k & 0x1FFFFF), (int)((k >> 21) & 0x1FFFFF), (int)(k >> 42));
		}
		for (int f = 0; f < 6; f++)
		{
			int u = (f / 2 + 1) % 3;
			fine[f] = frozen->fine[f];
			for (std::unordered_map<unsigned long long, unsigned long long>::const_iterator it = fine[f].begin(); it != fine[f].end(); ++it)
				face[f][(size_t)((it->first >> 21) & 0x1FFFFF) * nc[u] + (size_t)(it->first & 0x1FFFFF)] = 1;
		}
		skinDirty = anyDirty = true;
	}

	// lines of before the path that are of this lattice: the path is cut into them from here on
	void Attach(SimFrozen *F)
	{
		frozen = F;
		for (int a = 0; a < 3; a++) S[a].base = &F->S[a];
		UseLayout();
	}

	// The cuts of before the path are all made: the lines as they are now are what the path is
	// cut into from here on
	SimFrozen *Freeze()
	{
		std::atomic<SimPage *> *nt[3] = { NULL, NULL, NULL };
		SimHeap *nh[3] = { NULL, NULL, NULL };
		SimFrozen *F = NULL;
		try
		{
			F = new SimFrozen;
			for (size_t s = 0; s < key.size(); s++)
				if (key[s] != SIM_SKIN) F->key.push_back(key[s]);
			for (int f = 0; f < 6; f++) F->fine[f] = fine[f];
			for (int a = 0; a < 3; a++)
			{
				nt[a] = new std::atomic<SimPage *>[(size_t)S[a].pu * S[a].pv];
				nh[a] = SimHeapCreate();
				if (!nh[a]) throw std::bad_alloc();
			}
		}
		catch (...)
		{
			for (int a = 0; a < 3; a++)
			{
				delete[] nt[a];
				if (nh[a]) SimHeapDestroy(nh[a]);
			}
			delete F;
			throw;
		}
		for (int a = 0; a < 3; a++)
		{
			F->n[a] = n[a];
			F->lo[a] = boxLo[a];
			F->hi[a] = boxHi[a];
			S[a].Freeze(F->S[a], nt[a], nh[a]);
		}
		F->res = res;
		F->finest = finest;
		frozen = F;
		return F;
	}

	double LineBytes() const { return S[0].Bytes() + S[1].Bytes() + S[2].Bytes() + (frozen ? frozen->Bytes() : 0.0); }

	// Whoever cuts asks this often: do the lines take more memory than there is for the Stock?
	// Then the cutting stops where it is (the lattice is made coarser and all is cut again).
	bool Over()
	{
		if (over.load(std::memory_order_relaxed)) return true;
		if (!CanCoarsen() || LineBytes() <= budget) return false;
		over = true;
		return true;
	}
	double Bytes() const { return LineBytes() + triangles * SIM_TRI_BYTES + (double)key.size() * SIM_MESH_BYTES; }
	bool CanCoarsen() const { return n[0] > 9 || n[1] > 9 || n[2] > 9; }

	// nodes whose coordinate along an axis may lie between two values
	bool Range(int a, double c0, double c1, int *i0, int *i1) const
	{
		double f0 = floor((c0 - o[a]) / g[a]), f1 = ceil((c1 - o[a]) / g[a]);
		if (f0 > n[a] - 1 || f1 < 0) return false;
		*i0 = f0 < 0 ? 0 : (int)f0;
		*i1 = f1 > n[a] - 1 ? n[a] - 1 : (int)f1;
		return *i0 <= *i1;
	}

	// A tool that stands tilted (PRIM_TILT, at b with its axis along d): the stretch of the tool
	// that is in the Stock, as heights above its tip, how wide the tool is there, and the nodes
	// around that stretch.  False: nothing of the tool reaches the Stock.
	bool MakeTilt(SimPrim &P) const
	{
		const CSimCutTool &T = *P.T;
		double R = Max2(T.Rmax, rminTilt) + infl;
		double t0 = T.Bottom(), t1 = T.Top();		// (the top is SIMCUT_BIG for a tool without an end)
		for (int a = 0; a < 3; a++)
		{
			// where the axis is in the Stock made larger by the radius of the tool
			double lo = o[a] - R, hi = o[a] + (n[a] - 1) * g[a] + R, da = P.d[a];
			if (fabs(da) < 1e-12)
			{
				if (P.b[a] <= lo || P.b[a] >= hi) return false;
				continue;
			}
			double ta = (lo - P.b[a]) / da, tb = (hi - P.b[a]) / da;
			if (ta > tb) std::swap(ta, tb);
			if (ta > t0) t0 = ta;
			if (tb < t1) t1 = tb;
		}
		if (!(t0 < t1)) return false;
		R = T.MaxRadius(t0, t1);
		if (!(R > 0.0)) return false;
		P.R = (R > rminTilt ? R : rminTilt) + infl;
		P.t0 = t0;
		P.t1 = t1;
		for (int a = 0; a < 3; a++)
		{
			// (a disc across the axis reaches this far along a)
			double c0 = P.b[a] + t0 * P.d[a], c1 = P.b[a] + t1 * P.d[a], Ra = P.R * sqrt(Max2(0.0, 1.0 - P.d[a] * P.d[a])) + infl;
			if (!Range(a, Min2(c0, c1) - Ra, Max2(c0, c1) + Ra, &P.lo[a], &P.hi[a])) return false;
		}
		return true;
	}

	bool MakePrim(SimPrim &P) const
	{
		if (P.kind == PRIM_TILT) return MakeTilt(P);
		const CSimCutTool &T = *P.T;
		double zmin = Min2(P.a[2], P.b[2]), zmax = Max2(P.a[2], P.b[2]);
		if (!Range(2, zmin + T.Bottom() - tol, zmax + T.Top() + tol, &P.lo[2], &P.hi[2])) return false;
		// Only the part of the tool that is below the top of the Stock can cut.  A tool that is in
		// the Stock with its point is as wide as the point is there: no line beyond that is looked
		// at (a V shaped tool that engraves would otherwise cost its whole diameter at every move).
		double R = T.MaxRadius(-SIMCUT_BIG, o[2] + (n[2] - 1) * g[2] + tol - zmin);
		if (!(R > 0.0)) return false;
		P.R = (R > rmin ? R : rmin) + infl;
		for (int a = 0; a < 2; a++)
			if (!Range(a, Min2(P.a[a], P.b[a]) - P.R, Max2(P.a[a], P.b[a]) + P.R, &P.lo[a], &P.hi[a])) return false;
		return true;
	}

	// the surface near the nodes that a cut may reach is to be built again
	void MarkPrim(const SimPrim &P)
	{
		int c0[3], c1[3];
		for (int a = 0; a < 3; a++)
		{
			// node m is a corner of the cells m - 1 and m; cell c is in block (c + 1) / SIM_CS
			// and block b also uses the cells just below its own, and the points of the cells
			// SIM_HALO beyond those on every side (MeshBlock)
			c0[a] = MaxI((P.lo[a] - 2 + 1 - SIM_HALO) / SIM_CS, 0);
			c1[a] = MinI((P.hi[a] + 2 + 1 + SIM_HALO) / SIM_CS, nc[a] - 1);
			if (P.lo[a] - 2 + 1 - SIM_HALO < 0) c0[a] = 0;
		}
		for (int z = c0[2]; z <= c1[2]; z++)
			for (int y = c0[1]; y <= c1[1]; y++)
				for (int x = c0[0]; x <= c1[0]; x++)
					Touch(x, y, z);

		// and the faces of the Stock near the cut, by sub cells
		int s0[3], s1[3];
		for (int a = 0; a < 3; a++)
		{
			s0[a] = MaxI((P.lo[a] - 2 + 1) / SIM_SUB, 0);
			if (P.lo[a] - 2 + 1 < 0) s0[a] = 0;
			s1[a] = MinI((P.hi[a] + 2 + 1) / SIM_SUB, ns[a] - 1);
		}
		for (int a = 0; a < 3; a++)
		{
			int u = (a + 1) % 3, v = (a + 2) % 3;
			if (s0[a] == 0) MarkFace(2 * a, s0[u], s1[u], s0[v], s1[v]);
			if (s1[a] == ns[a] - 1) MarkFace(2 * a + 1, s0[u], s1[u], s0[v], s1[v]);
		}
	}
};

// Squared distance from a point to a segment; (ex, ey) from the nearest point of the segment to the point
static inline double PointSeg2(double px, double py, double ax, double ay, double bx, double by, double *ex, double *ey)
{
	double dx = bx - ax, dy = by - ay;
	double L2 = dx * dx + dy * dy;
	double t = L2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0.0;
	if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
	*ex = px - (ax + t * dx);
	*ey = py - (ay + t * dy);
	return *ex * *ex + *ey * *ey;
}

// A disc of radius R travels from A to B in a plane with the axes (a, b).  The part of the line
// b = c that it sweeps: t0 < a < t1.
static bool Capsule(double Aa, double Ab, double Ba, double Bb, double R, double c, double *t0, double *t1)
{
	double lo = 1e300, hi = -1e300;

	// the disc at each end
	double q = c - Ab, s2 = R * R - q * q;
	if (s2 > 0.0)
	{
		double s = sqrt(s2);
		lo = Aa - s;
		hi = Aa + s;
	}
	q = c - Bb;
	s2 = R * R - q * q;
	if (s2 > 0.0)
	{
		double s = sqrt(s2);
		if (Ba - s < lo) lo = Ba - s;
		if (Ba + s > hi) hi = Ba + s;
	}

	// the strip between the ends: 0 < along < L and |across| < R
	double da = Ba - Aa, db = Bb - Ab, L = sqrt(da * da + db * db);
	if (L > 0.0)
	{
		double ua = da / L, ub = db / L, w = c - Ab;
		double x0 = -1e300, x1 = 1e300;		// of a - Aa
		bool ok = true;
		if (ua != 0.0)
		{
			double p = -w * ub / ua, p2 = (L - w * ub) / ua;
			if (p > p2) std::swap(p, p2);
			x0 = p;
			x1 = p2;
		}
		else if (!(w * ub > 0.0 && w * ub < L))
			ok = false;
		if (ok)
		{
			if (ub != 0.0)
			{
				double p = (w * ua - R) / ub, p2 = (w * ua + R) / ub;
				if (p > p2) std::swap(p, p2);
				if (p > x0) x0 = p;
				if (p2 < x1) x1 = p2;
			}
			else if (!(fabs(w * ua) < R))
				ok = false;
		}
		if (ok && x0 < x1)
		{
			if (Aa + x0 < lo) lo = Aa + x0;
			if (Aa + x1 > hi) hi = Aa + x1;
		}
	}

	if (!(lo < hi)) return false;
	*t0 = lo;
	*t1 = hi;
	return true;
}

// One cut against the lines along Z with j0 <= j <= j1.  A line parallel to the tool axis is
// inside the tool where the radius of the tool is greater than its distance from the axis.
static void CutZ(SimGrid &G, const SimPrim &P, int j0, int j1, std::vector<double> &iv)
{
	const CSimCutTool &T = *P.T;
	int ja = MaxI(P.lo[1], j0), jb = MinI(P.hi[1], j1);
	if (ja > jb) return;

	SimLines &Z = G.S[2];
	double R2 = P.R * P.R;
	double zmin = Min2(P.a[2], P.b[2]), zmax = Max2(P.a[2], P.b[2]);
	SimPos zlo = Z.Fix(zmin + T.Bottom()), zhi = Z.Fix(zmax + T.Top());
	if (Z.hi <= zlo || Z.lo >= zhi) return;			// the whole tool is above or below the Stock
	bool flat = P.kind == PRIM_FLAT;
	size_t room = 2 * T.h.size() + 2;
	iv.resize(2 * room);
	double *sl = &iv[room];			// (see CSimCutTool::Above)
	SimRoom mem = { Z.heap, 0 };

	for (int j = ja; j <= jb; j++)
	{
		if (G.Over()) break;
		if (mem.grown > 65536)
		{
			Z.pieceBytes += mem.grown;
			mem.grown = 0;
		}
		double y = G.C(1, j);
		SimChunk *ch = NULL;			// of the lines that are being looked at
		const SimChunk *bch = NULL;		// ... as they were before the path, where the path has not cut
		int at = -1;
		for (int i = P.lo[0]; i <= P.hi[0]; i++)
		{
			double x = G.C(0, i), d2, ex, ey;
			if (flat) d2 = PointSeg2(x, y, P.a[0], P.a[1], P.b[0], P.b[1], &ex, &ey);
			else
			{
				ex = x - P.a[0];
				ey = y - P.a[1];
				d2 = ex * ex + ey * ey;
			}
			if (d2 >= R2) continue;
			if ((i >> SIM_LC_BITS) != at)
			{
				at = i >> SIM_LC_BITS;
				ch = Z.Chunk(at, j >> SIM_LC_BITS);
				bch = ch ? NULL : Z.BaseChunk(at, j >> SIM_LC_BITS);
			}
			if (ch ? !ch->live : bch && !bch->live)		// no material left in any line of the chunk: on to the next one
			{
				i |= SIM_LC - 1;
				continue;
			}
			SimDex *d = ch ? &ch->d[SimLines::At(i, j)] : NULL;
			const SimDex *was = d ? d : bch ? &bch->d[SimLines::At(i, j)] : &Z.full;		// the line as it is
			if (!was->n || was->Hi(was->n - 1) <= zlo || was->Lo(0) >= zhi) continue;	// no material where the tool is
			double away = sqrt(d2), dist = away - G.infl;
			bool least = dist < G.rmin;
			if (least) dist = 0.0;				// as close as that: inside the tool at every height where there is tool
			int m = T.Above(dist, &iv[0], sl);
			if (m == 0) continue;
			if (away > 0.0) { ex /= away; ey /= away; }
			if (least && T.RadiusBelow(iv[0] + G.g[2]) < G.rmin) sl[0] = SIMCUT_BIG;	// the floor of the slot that a thin tip leaves
			if (!d)
			{
				// a line that the path has not cut so far: it gets its chunk if the tool reaches the material
				if (Z.Fix(zmin + iv[0]) >= was->Hi(was->n - 1) || Z.Fix(zmax + iv[2 * m - 1]) <= was->Lo(0)) continue;
				ch = Z.MakeChunk(at, j >> SIM_LC_BITS);
				d = &ch->d[SimLines::At(i, j)];
			}
			bool had = d->n != 0;
			for (int q = 0; q < m; q++)
				d->Cut(Z.Fix(zmin + iv[2 * q]), Z.Fix(zmax + iv[2 * q + 1]), P.flag, ToolDir(ex, ey, sl[2 * q]), ToolDir(ex, ey, sl[2 * q + 1]), &mem);
			if (had && !d->n) ch->live--;
		}
	}
	if (mem.grown) Z.pieceBytes += mem.grown;
}

// One cut against the lines along X (s = 0, the levels v0 <= k <= v1) or along Y (s = 1, the
// lines with v0 <= i <= v1).  In the plane of such a line the tool is a disc.
static void CutXY(SimGrid &G, int s, const SimPrim &P, int v0, int v1, std::vector<double> &Rk)
{
	const CSimCutTool &T = *P.T;
	int a = s, b = 1 - s;			// the line runs along a
	int k0 = P.lo[2], k1 = P.hi[2], m0 = P.lo[b], m1 = P.hi[b];
	if (s == 0) { k0 = MaxI(k0, v0); k1 = MinI(k1, v1); }
	else { m0 = MaxI(m0, v0); m1 = MinI(m1, v1); }
	if (k0 > k1 || m0 > m1) return;

	// radius of the disc at each level, and how fast the radius of the tool grows with the height where it leaves the surface there
	double zmin = Min2(P.a[2], P.b[2]), zmax = Max2(P.a[2], P.b[2]);
	int levels = k1 - k0 + 1;
	Rk.resize(2 * levels);
	double *Sk = &Rk[levels];
	bool any = false;
	for (int k = k0; k <= k1; k++)
	{
		double z = G.PlaneZ(k), R, S;
		if (P.kind == PRIM_PLUNGE)
		{
			// the widest of what passes the level; if that is the tool where it stops (or where it began), the side of the tool there
			R = T.MaxRadius(z - zmax, z - zmin);
			double R0 = T.RadiusBelow(z - zmax), R1 = T.RadiusBelow(z - zmin);
			if (R0 > R) R = R0;
			S = R1 >= R ? T.SlopeBelow(z - zmin) : R0 >= R ? T.SlopeBelow(z - zmax) : 0.0;
		}
		else
		{
			R = T.RadiusBelow(z - P.a[2]);
			S = T.SlopeBelow(z - P.a[2]);
		}
		if (R > 0.0)
		{
			if (R < G.rmin)
			{
				R = G.rmin;
				S = 0.0;				// the upright wall of the slot that a thin tip leaves
			}
			R += G.infl;
			any = true;
		}
		Rk[k - k0] = R;
		Sk[k - k0] = S;
	}
	if (!any) return;

	SimLines &L = G.S[s];
	bool seg = P.kind == PRIM_FLAT;
	double bmin = Min2(P.a[b], P.b[b]), bmax = Max2(P.a[b], P.b[b]);
	// the lines along X are (j, k), those along Y (k, i): the second index is the one that the bands are made of
	int outer0 = s == 0 ? k0 : m0, outer1 = s == 0 ? k1 : m1;
	int inner0 = s == 0 ? m0 : k0, inner1 = s == 0 ? m1 : k1;
	SimRoom mem = { L.heap, 0 };

	for (int u = outer0; u <= outer1; u++)
	{
		if (G.Over()) break;
		if (mem.grown > 65536)
		{
			L.pieceBytes += mem.grown;
			mem.grown = 0;
		}
		SimChunk *ch = NULL;
		const SimChunk *bch = NULL;
		int at = -1;
		for (int w = inner0; w <= inner1; w++)
		{
			int k = s == 0 ? u : w, m = s == 0 ? w : u;
			double R = Rk[k - k0];
			if (!(R > 0.0)) continue;
			double c = G.C(b, m);
			if (c <= bmin - R || c >= bmax + R) continue;
			if ((w >> SIM_LC_BITS) != at)
			{
				at = w >> SIM_LC_BITS;
				ch = L.Chunk(at, u >> SIM_LC_BITS);
				bch = ch ? NULL : L.BaseChunk(at, u >> SIM_LC_BITS);
			}
			if (ch ? !ch->live : bch && !bch->live)		// no material left in any line of the chunk: on to the next one
			{
				w |= SIM_LC - 1;
				continue;
			}
			SimDex *d = ch ? &ch->d[SimLines::At(w, u)] : NULL;
			const SimDex *was = d ? d : bch ? &bch->d[SimLines::At(w, u)] : &L.full;		// the line as it is
			if (!was->n) continue;
			double t0, t1;
			if (seg)
			{
				if (!Capsule(P.a[a], P.a[b], P.b[a], P.b[b], R, c, &t0, &t1)) continue;
			}
			else
			{
				double q = c - P.a[b], s2 = R * R - q * q;
				if (!(s2 > 0.0)) continue;
				double sq = sqrt(s2);
				t0 = P.a[a] - sq;
				t1 = P.a[a] + sq;
			}
			SimPos f0 = L.Fix(t0), f1 = L.Fix(t1);
			if (!d)
			{
				// a line that the path has not cut so far: it gets its chunk if the tool reaches the material
				if (f0 >= was->Hi(was->n - 1) || f1 <= was->Lo(0)) continue;
				ch = L.MakeChunk(at, u >> SIM_LC_BITS);
				d = &ch->d[SimLines::At(w, u)];
			}
			// the direction of the surface at the two ends: away from the nearest point of the axis of the tool, tilted by its side
			double ea0, eb0, ea1, eb1;
			if (seg)
			{
				PointSeg2(t0, c, P.a[a], P.a[b], P.b[a], P.b[b], &ea0, &eb0);
				PointSeg2(t1, c, P.a[a], P.a[b], P.b[a], P.b[b], &ea1, &eb1);
			}
			else
			{
				ea0 = t0 - P.a[a];
				ea1 = t1 - P.a[a];
				eb0 = eb1 = c - P.a[b];
			}
			double S = Sk[k - k0] * R;			// (the vectors have the length R)
			unsigned short n0 = s == 0 ? DirCode(-ea0, -eb0, S) : DirCode(-eb0, -ea0, S);
			unsigned short n1 = s == 0 ? DirCode(-ea1, -eb1, S) : DirCode(-eb1, -ea1, S);
			if (d->Cut(f0, f1, P.flag, n0, n1, &mem) && !d->n) ch->live--;
		}
	}
	if (mem.grown) L.pieceBytes += mem.grown;
}

//////////////////////////////////////////////////////////////////////////////
// A tool that stands tilted
//////////////////////////////////////////////////////////////////////////////

// Seen from its tip the tool is a stack of slices along its axis, each between two heights with a
// radius that changes evenly from the one to the other: a cylinder or a piece of a cone.  The
// radii are those that the tool cuts with in this Stock: nowhere thinner than the lattice can
// tell (SimGrid::rminTilt), and a hair wider (SimGrid::infl).
static void MakeSlices(const CSimCutTool &T, double rmin, double infl, std::vector<SimSlice> &out)
{
	out.clear();
	size_t n = T.h.size();
	if (n < 2) return;
	for (size_t i = 0; i < n; i++)
	{
		double h0, h1, r0, r1;
		if (i + 1 < n)
		{
			h0 = T.h[i];
			h1 = T.h[i + 1];
			r0 = T.r[i];
			r1 = T.r[i + 1];
			if (!(h0 < h1)) continue;			// a step
		}
		else
		{
			if (!T.Open) break;
			h0 = T.h[n - 1];					// whatever holds the tool, above it without end
			h1 = SIMCUT_BIG;
			r0 = r1 = T.r[n - 1];
		}
		if (!(r0 > 0.0) && !(r1 > 0.0)) continue;	// nothing of the tool there
		bool ends = h1 < SIMCUT_BIG / 2;
		// thinner than the lattice can tell up to a height, or from a height on: two slices
		double hm = (r0 < rmin) != (r1 < rmin) ? h0 + (rmin - r0) / (r1 - r0) * (h1 - h0) : h1;
		for (int part = 0; part < 2; part++)
		{
			double a = part ? hm : h0, b = part ? h1 : hm;
			if (!(a < b)) continue;
			double ra = ends ? r0 + (a - h0) / (h1 - h0) * (r1 - r0) : r0, rb = ends ? r0 + (b - h0) / (h1 - h0) * (r1 - r0) : r0;
			if (ra < rmin) ra = rmin;
			if (rb < rmin) rb = rmin;
			SimSlice S = { a, b, ra + infl, ends ? (rb - ra) / (b - a) : 0.0 };
			out.push_back(S);
		}
	}
}

// How far a tool may travel sideways (across its axis) from one position at which it stands to
// the next so that what the two leave standing between them is not higher than eps.  At its
// lowest point that is as far as the tool is wide at the height eps: any farther and the two
// positions do not meet below that height.  Higher up the round side of the tool leaves a ridge
// between the two, s * s / (8 r) deep across the axis where the tool has the radius r and the
// positions are s apart, and less than that square to a side that slopes.  A face across the
// axis leaves none.
static double SideStep(const CSimCutTool &T, double rmin, double eps)
{
	size_t n = T.h.size();
	if (n < 2) return 0.0;
	double at = T.Bottom() + eps;
	double step = 2.0 * Max2(T.RadiusBelow(at), rmin);
	for (size_t i = 0; i + 1 < n; i++)
	{
		double h0 = T.h[i], h1 = T.h[i + 1], r0 = T.r[i], r1 = T.r[i + 1];
		if (!(h1 > h0) || h1 <= at) continue;			// a face across the axis, or below that height
		double slope = (r1 - r0) / (h1 - h0);
		if (h0 < at) r0 += slope * (at - h0);
		double r = Max2(Min2(r0, r1), rmin);
		step = Min2(step, sqrt(8.0 * eps * r * sqrt(1.0 + slope * slope)));
	}
	if (T.Open) step = Min2(step, sqrt(8.0 * eps * Max2(T.r[n - 1], rmin)));
	return step;
}

// One cut with a tool that stands tilted (PRIM_TILT) against the lines of one set: those along X
// (s = 0) of the levels v0 <= k <= v1, along Y (s = 1) with v0 <= i <= v1, or along Z (s = 2)
// with v0 <= j <= v1.  A point is inside the tool where it is nearer to the axis of the tool
// than the radius that the tool has at the height of the point along that axis.  Along a line
// that is, for each slice of the tool, one stretch: between the points where the line passes
// through the side of the slice (a cone or a cylinder) or through its lower or upper face.
static void CutTilt(SimGrid &G, int s, const SimPrim &P, int v0, int v1, std::vector<double> &buf)
{
	const std::vector<SimSlice> &SL = *P.slices;
	int a = s, u = (s + 1) % 3, v = (s + 2) % 3;		// the lines run along a: line (iu, iv)
	int iv0 = MaxI(P.lo[v], v0), iv1 = MinI(P.hi[v], v1);
	if (iv0 > iv1 || SL.empty()) return;

	SimLines &L = G.S[s];
	const double *d = P.d, *T = P.b;
	double de = d[a], c2 = 1.0 - de * de, R2 = P.R * P.R;
	bool along = fabs(de) > 1e-12;					// the height along the tool changes along the lines
	// the stretch of the axis of the tool that is in the Stock, as the lines see it
	double U0 = T[u] + P.t0 * d[u], V0 = T[v] + P.t0 * d[v], U1 = T[u] + P.t1 * d[u], V1 = T[v] + P.t1 * d[v];
	// what the surface looks like where a line passes through a face of a slice: the lower face of
	// the tool leaves material that looks up along the tool, an upper face material that looks down
	unsigned short faceLo = DirCode(d[0], d[1], d[2]), faceHi = DirCode(-d[0], -d[1], -d[2]);
	buf.resize(4 * (SL.size() + 1));
	double *span = &buf[0];			// of each stretch: from, to, and which side of which slice each end is (-1: its lower face, -2: its upper face)
	SimRoom mem = { L.heap, 0 };

	for (int iv = iv0; iv <= iv1; iv++)
	{
		if (G.Over()) break;
		if (mem.grown > 65536)
		{
			L.pieceBytes += mem.grown;
			mem.grown = 0;
		}
		double cv = v == 2 ? G.PlaneZ(iv) : G.C(v, iv);
		// the lines of this row that pass the axis within the radius of the tool
		double ua, ub;
		if (!Capsule(U0, V0, U1, V1, P.R, cv, &ua, &ub)) continue;
		int iu0 = MaxI(P.lo[u], (int)floor((ua - G.o[u]) / G.g[u])), iu1 = MinI(P.hi[u], (int)ceil((ub - G.o[u]) / G.g[u]));
		SimChunk *ch = NULL;
		const SimChunk *bch = NULL;
		int at = -1;
		for (int iu = iu0; iu <= iu1; iu++)
		{
			double cu = u == 2 ? G.PlaneZ(iu) : G.C(u, iu);
			// The line as the tool tip sees it: the points q + x e, where e is the direction of the
			// lines and x the coordinate along them.  The height of such a point along the axis of
			// the tool is  h0 + de x,  and its distance from the axis squared  c0 + c1 x + c2 x^2.
			double q[3];
			q[a] = -T[a];
			q[u] = cu - T[u];
			q[v] = cv - T[v];
			double h0 = q[0] * d[0] + q[1] * d[1] + q[2] * d[2];
			double c1 = 2.0 * (q[a] - h0 * de), c0 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] - h0 * h0;

			// where the line is within the widest radius of the tool, and within the stretch of the tool that is in the Stock
			double xa = -SIMCUT_BIG, xb = SIMCUT_BIG;
			if (c2 > 1e-12)
			{
				double disc = c1 * c1 - 4.0 * c2 * (c0 - R2);
				if (!(disc > 0.0)) continue;
				double sq = sqrt(disc);
				xa = (-c1 - sq) / (2.0 * c2);
				xb = (-c1 + sq) / (2.0 * c2);
			}
			else if (!(c0 < R2)) continue;
			if (along)
			{
				double x0 = (P.t0 - h0) / de, x1 = (P.t1 - h0) / de;
				if (x0 > x1) std::swap(x0, x1);
				if (x0 > xa) xa = x0;
				if (x1 < xb) xb = x1;
			}
			else if (h0 <= P.t0 || h0 >= P.t1) continue;
			if (!(xa < xb)) continue;
			SimPos fa = L.Fix(xa), fb = L.Fix(xb);
			if (!(fa < fb)) continue;

			if ((iu >> SIM_LC_BITS) != at)
			{
				at = iu >> SIM_LC_BITS;
				ch = L.Chunk(at, iv >> SIM_LC_BITS);
				bch = ch ? NULL : L.BaseChunk(at, iv >> SIM_LC_BITS);
			}
			if (ch ? !ch->live : bch && !bch->live)		// no material left in any line of the chunk: on to the next one
			{
				iu |= SIM_LC - 1;
				continue;
			}
			SimDex *dex = ch ? &ch->d[SimLines::At(iu, iv)] : NULL;
			const SimDex *was = dex ? dex : bch ? &bch->d[SimLines::At(iu, iv)] : &L.full;		// the line as it is
			bool any = false;
			for (int p = 0; p < was->n && !any; p++) any = was->Lo(p) < fb && was->Hi(p) > fa;
			if (!any) continue;					// no material where the tool is

			// the stretches of the line that are inside the slices of the tool, from the lowest slice up
			double hlo = h0 + de * xa, hhi = h0 + de * xb;
			if (hlo > hhi) std::swap(hlo, hhi);
			size_t i0 = 0, i1 = SL.size();
			while (i0 < i1)						// the first slice that reaches above hlo
			{
				size_t mid = (i0 + i1) / 2;
				if (SL[mid].h1 <= hlo) i0 = mid + 1; else i1 = mid;
			}
			int m = 0;
			for (size_t i = i0; i < SL.size() && SL[i].h0 < hhi; i++)
			{
				const SimSlice &S = SL[i];
				double lo = xa, hi = xb, elo = (double)i, ehi = (double)i;
				if (along)
				{
					// between the lower and the upper face of the slice
					double x0 = (S.h0 - h0) / de, x1 = S.h1 < SIMCUT_BIG / 2 ? (S.h1 - h0) / de : de > 0.0 ? SIMCUT_BIG : -SIMCUT_BIG;
					// (also where the face is exactly the end of what is looked at, as the lower face of
					// a tool whose tip is in the Stock always is: that end is the face, not the side)
					if (de > 0.0)
					{
						if (x0 >= lo) { lo = x0; elo = -1.0; }
						if (x1 <= hi) { hi = x1; ehi = -2.0; }
					}
					else
					{
						if (x1 >= lo) { lo = x1; elo = -2.0; }
						if (x0 <= hi) { hi = x0; ehi = -1.0; }
					}
				}
				else if (h0 < S.h0 || h0 >= S.h1 || (h0 == S.h0 && (i == 0 || SL[i - 1].h1 < S.h0))) continue;
				// (all of the line is at one height of the tool: the face between two slices counts as of the upper one)
				// inside the side of the slice: nearer to the axis than its radius there, e0 + e1 x
				double e0 = S.r0 + S.k * (h0 - S.h0), e1 = S.k * de;
				double a2 = c2 - e1 * e1, a1 = c1 - 2.0 * e0 * e1, a0 = c0 - e0 * e0;
				if (fabs(a2) <= 1e-12)
				{
					if (fabs(a1) <= 1e-12)
					{
						if (!(a0 < 0.0)) continue;
					}
					else
					{
						double r = -a0 / a1;
						if (a1 > 0.0)
						{
							if (r < hi) { hi = r; ehi = (double)i; }
						}
						else if (r > lo) { lo = r; elo = (double)i; }
					}
				}
				else
				{
					double disc = a1 * a1 - 4.0 * a2 * a0;
					if (a2 > 0.0)
					{
						if (!(disc > 0.0)) continue;
						double sq = sqrt(disc), r1 = (-a1 - sq) / (2.0 * a2), r2 = (-a1 + sq) / (2.0 * a2);
						if (r1 > lo) { lo = r1; elo = (double)i; }
						if (r2 < hi) { hi = r2; ehi = (double)i; }
					}
					else if (disc > 0.0)
					{
						// (inside the cone on both sides of two points: the slice is on the side where its radius is positive)
						double sq = sqrt(disc), r1 = (-a1 + sq) / (2.0 * a2), r2 = (-a1 - sq) / (2.0 * a2);
						if (e1 > 0.0)
						{
							if (r2 > lo) { lo = r2; elo = (double)i; }
						}
						else if (r1 < hi) { hi = r1; ehi = (double)i; }
					}
				}
				if (!(lo < hi)) continue;

				// it goes on from the stretch of the slice below where the two meet in the face between them
				if (m > 0 && along && de > 0.0 && lo <= span[4 * m - 3] + 1e-9)
				{
					span[4 * m - 3] = hi;
					span[4 * m - 1] = ehi;
				}
				else if (m > 0 && along && de < 0.0 && hi >= span[4 * m - 4] - 1e-9)
				{
					span[4 * m - 4] = lo;
					span[4 * m - 2] = elo;
				}
				else
				{
					span[4 * m] = lo;
					span[4 * m + 1] = hi;
					span[4 * m + 2] = elo;
					span[4 * m + 3] = ehi;
					m++;
				}
			}

			for (int k = 0; k < m; k++)
			{
				SimPos f0 = L.Fix(span[4 * k]), f1 = L.Fix(span[4 * k + 1]);
				if (!(f0 < f1)) continue;
				if (!dex)
				{
					// a line that the path has not cut so far: it gets its chunk if the tool reaches the material
					any = false;
					for (int p = 0; p < was->n && !any; p++) any = was->Lo(p) < f1 && was->Hi(p) > f0;
					if (!any) continue;
					ch = L.MakeChunk(at, iv >> SIM_LC_BITS);
					dex = &ch->d[SimLines::At(iu, iv)];
				}
				// The direction of the surface that is left at each end: at a face of the tool along the
				// tool, at its side away from the axis of the tool, tilted as the side is
				unsigned short dir[2];
				for (int e = 0; e < 2; e++)
				{
					double x = span[4 * k + e], kind = span[4 * k + 2 + e];
					if (kind < -1.5) dir[e] = faceHi;
					else if (kind < 0.0) dir[e] = faceLo;
					else
					{
						const SimSlice &S = SL[(size_t)kind];
						double h = h0 + de * x, lift = S.k * (S.r0 + S.k * (h - S.h0)), w[3];
						for (int c = 0; c < 3; c++) w[c] = q[c] - h * d[c];
						w[a] += x;
						dir[e] = DirCode(lift * d[0] - w[0], lift * d[1] - w[1], lift * d[2] - w[2]);
					}
				}
				if (dex->Cut(f0, f1, P.flag, dir[0], dir[1], &mem) && !dex->n) ch->live--;
			}
		}
	}
	if (mem.grown) L.pieceBytes += mem.grown;
}

// Run jobs 0 .. jobs-1 on several threads.  fn(job, thread)
template <class F> static void Parallel(int jobs, int threads, F fn)
{
	if (threads > jobs) threads = jobs;
	if (threads <= 1)
	{
		for (int j = 0; j < jobs; j++) fn(j, 0);
		return;
	}
	std::atomic<int> next(0);
	std::atomic<bool> failed(false);
	auto body = [&](int t)
	{
		try
		{
			for (;;)
			{
				int j = next++;
				if (j >= jobs) break;
				fn(j, t);
			}
		}
		catch (...)					// (no memory) must not leave a thread: it would end the application
		{
			failed = true;
		}
	};
	std::vector<std::thread> th;
	for (int t = 1; t < threads; t++)
	{
		try
		{
			th.emplace_back([&body, t]()
			{
				SimBelowNormal();
				body(t);
			});
		}
		catch (...)
		{
			break;					// no more threads to be had: the others do the work
		}
	}
	body(0);
	for (size_t t = 0; t < th.size(); t++) th[t].join();
	if (failed) throw std::bad_alloc();
}


//////////////////////////////////////////////////////////////////////////////
// The surface
//////////////////////////////////////////////////////////////////////////////

struct SimScratch
{
	std::vector<unsigned char> state;		// node inside material
	std::vector<SimPos> cpos[3];			// where the edge from a node along X, Y, Z crosses the surface
	std::vector<unsigned char> cflag[3];	// what made the surface there, 255 = the edge does not cross it
	std::vector<unsigned short> cdir[3];	// direction of the surface there (a code, 0 = not known)
	std::vector<int> vid;					// point of the surface in a cell
	std::vector<int> cell;					// which of the points below is the one of a cell (-1: the surface does not pass through it)
	std::vector<size_t> pq;					// of each point: its cell,
	std::vector<int> pc;					// ... the cell's indices (three each),
	std::vector<double> pos, pos2;			// ... where it is (three each)
	std::vector<unsigned char> fix;			// ... and whether it stays where it is
	double guessed;
};

// Where the surface crosses an edge of the lattice (c0 to c1 along the line d).  One end of the
// edge is inside material.
static bool FindCross(const SimDex &d, long long c0, long long c1, bool lowInside, long long tol, SimPos *pos, unsigned char *flag, unsigned short *dir)
{
	int n = d.n;
	if (lowInside)
	{
		// the piece that holds c0 ends on the edge
		for (int i = n - 1; i >= 0; i--)
		{
			if ((long long)d.Lo(i) > c0 + tol) continue;
			long long e = d.Hi(i);
			if (e < c0 - tol || e > c1 + tol) return false;
			*pos = (SimPos)(e < c0 ? c0 : e > c1 ? c1 : e);
			*flag = d.FHi(i);
			*dir = d.NHi(i);
			return true;
		}
	}
	else
	{
		// the piece that holds c1 starts on the edge
		for (int i = 0; i < n; i++)
		{
			if ((long long)d.Hi(i) < c1 - tol) continue;
			long long e = d.Lo(i);
			if (e < c0 - tol || e > c1 + tol) return false;
			*pos = (SimPos)(e < c0 ? c0 : e > c1 ? c1 : e);
			*flag = d.FLo(i);
			*dir = d.NLo(i);
			return true;
		}
	}
	return false;
}

// A coordinate of the point of a cell: the mean of what the points on its edges have, which is
// for some of them the coordinate of a node (their indices added up) and for the others a
// position along a line (added up as well).  The points of an uncut face come out exactly
// where the skin has them.
static inline float MeanCoord(double o, double g, int sumIdx, long long sumPos, int cnt, double off = 0.0)
{
	double m = o + g * (((double)sumIdx + (double)sumPos / SIM_ONE) / cnt);
	return (float)(off == 0.0 ? m : m + off);
}

// Surface inside one block of cells: one point in every cell that the surface passes through
// (the mean of the surface points on the edges of the cell) and one quad across every edge of
// the lattice that has one end in material and the other outside.  A layer of nodes outside
// the Stock closes the surface at its faces.
//
// Where the surface has detail that is smaller than a cell (the low ridge between two passes of
// a tool, say) the three sets of lines catch it differently from cell to cell and the points
// come out jagged.  So every point is moved half way toward the mean of its neighbours (the
// points of the cells that it shares a quad with), SIM_RELAX times.  A point on an edge or a
// corner (the directions of the surface in its cell say so) stays where it is, and so does a
// point on a face of the Stock: flat faces stay flat and edges sharp, as the tool made them.
// Every point also gets the direction of the surface there, the mean of the quads around it,
// for a shading without facets; a point that stays where it is gets none.
// A block is built by itself, and its neighbours must come to the same points along the border
// that they share: everything about a point depends only on the lines within SIM_HALO cells of
// it, and each block makes the points that far beyond the cells of its own quads.
static void MeshBlock(const SimGrid &G, int bx, int by, int bz, SimScratch &S, SimMesh &M)
{
	M.v.clear();
	M.t.clear();

	// own cells c0 <= c < c1 (the quads lie between the points of the cells from c0 - 1 on),
	// cells looked at ca <= c < cb, their nodes ca <= i <= cb
	int bb[3] = { bx, by, bz }, c0[3], c1[3], ca[3], cb[3], N[3];
	for (int a = 0; a < 3; a++)
	{
		c0[a] = -1 + bb[a] * SIM_CS;
		c1[a] = MinI(c0[a] + SIM_CS, G.n[a]);
		ca[a] = MaxI(c0[a] - 1 - SIM_HALO, -1);
		cb[a] = MinI(c1[a] + SIM_HALO, G.n[a]);
		N[a] = cb[a] - ca[a] + 1;
	}
	size_t NN = (size_t)N[0] * N[1] * N[2];
	size_t step[3] = { 1, (size_t)N[0], (size_t)N[0] * N[1] };
#define LI(i, j, k) (((size_t)((k) - ca[2]) * N[1] + ((j) - ca[1])) * N[0] + ((i) - ca[0]))

	// nodes inside material, as the lines along Z have it
	S.state.assign(NN, 0);
	size_t inside = 0;
	int ia = MaxI(ca[0], 0), ib = MinI(cb[0], G.n[0] - 1);
	int ja = MaxI(ca[1], 0), jb = MinI(cb[1], G.n[1] - 1);
	int ka = MaxI(ca[2], 0), kb = MinI(cb[2], G.n[2] - 1);
	for (int j = ja; j <= jb; j++)
		for (int i = ia; i <= ib; i++)
		{
			const SimDex &d = G.S[2].Get(i, j);
			for (int p = 0; p < d.n; p++)
			{
				// (the nodes are a hair below their height, those of the lowest plane a hair above)
				long long lo = d.Lo(p), hi = d.Hi(p);
				int k0 = (int)((lo + SIM_TOL + SIM_ONE - 1) >> SIM_ONE_BITS), k1 = (int)((hi + SIM_TOL) >> SIM_ONE_BITS);
				if (k0 < 1) k0 = 1;
				if (k1 > kb) k1 = kb;
				if (k0 < ka) k0 = ka;
				if (ka == 0 && lo <= SIM_TOL && hi >= SIM_TOL)
				{
					unsigned char &st = S.state[LI(i, j, 0)];
					if (!st) { st = 1; inside++; }
				}
				for (int k = k0; k <= k1; k++)
				{
					unsigned char &st = S.state[LI(i, j, k)];
					if (!st) { st = 1; inside++; }
				}
			}
		}
	if (inside == 0 || inside == NN) return;

	// where the edges cross the surface
	for (int a = 0; a < 3; a++)
	{
		S.cpos[a].resize(NN);
		S.cflag[a].assign(NN, 255);
		S.cdir[a].resize(NN);
	}
	for (int k = ca[2]; k <= cb[2]; k++)
		for (int j = ca[1]; j <= cb[1]; j++)
			for (int i = ca[0]; i <= cb[0]; i++)
			{
				size_t q = LI(i, j, k);
				unsigned char s0 = S.state[q];
				int idx[3] = { i, j, k };
				bool own = i >= c0[0] && i < c1[0] && j >= c0[1] && j < c1[1] && k >= c0[2] && k < c1[2];
				for (int a = 0; a < 3; a++)
				{
					if (idx[a] + 1 > cb[a]) continue;
					if (S.state[q + step[a]] == s0) continue;
					const SimDex *d;
					if (a == 0) d = &G.S[0].Get(j, k);
					else if (a == 1) d = &G.S[1].Get(k, i);
					else d = &G.S[2].Get(i, j);
					long long e0 = (long long)idx[a] * SIM_ONE, e1 = e0 + SIM_ONE;
					SimPos pos = 0;
					unsigned char fl;
					unsigned short dir = 0;
					bool found;
					if (a == 2)
					{
						// where the planes of the two nodes count as lying; half a hair decides without doubt
						long long p0 = k >= 0 && k < G.n[2] ? e0 + (k == 0 ? SIM_TOL : -SIM_TOL) : e0 - SIM_TOL;
						long long p1 = k + 1 >= 0 && k + 1 < G.n[2] ? e1 + (k + 1 == 0 ? SIM_TOL : -SIM_TOL) : e1 - SIM_TOL;
						found = FindCross(*d, p0, p1, s0 != 0, SIM_TOL / 2, &pos, &fl, &dir);
						if (found && (long long)pos < e0) pos = (SimPos)e0;
						if (found && (long long)pos > e1) pos = (SimPos)e1;
					}
					else
						found = FindCross(*d, e0, e1, s0 != 0, SIM_TOL, &pos, &fl, &dir);
					if (!found)
					{
						// the line does not agree with the nodes: the middle of the edge (of what there is of it on the line)
						long long mid = (e0 + e1) / 2;
						pos = mid < 0 ? 0 : mid > (long long)G.S[a].hi ? G.S[a].hi : (SimPos)mid;
						fl = SIMKIND_FEED;
						dir = 0;
						if (own) S.guessed += 1.0;		// (counted by the block that the edge belongs to)
#ifdef SIMCUT_DEBUG
						if (own && S.guessed <= 40.0)
						{
							fprintf(stderr, "guessed: edge along %c from node %d %d %d (%.6f %.6f %.6f), low end %s, line has %d pieces:",
								"XYZ"[a], i, j, k, G.C(0, i), G.C(1, j), G.C(2, k), s0 ? "inside" : "outside", (int)d->n);
							for (int pp = 0; pp < d->n && pp < 6; pp++)
								fprintf(stderr, " [%.7f %.7f]", G.o[a] + G.g[a] * d->Lo(pp) / SIM_ONE, G.o[a] + G.g[a] * d->Hi(pp) / SIM_ONE);
							fprintf(stderr, "\n");
						}
#endif
					}
					S.cpos[a][q] = pos;
					S.cflag[a][q] = fl;
					S.cdir[a][q] = dir;
				}
			}

	// The point of every cell that the surface passes through, as the surface points on the edges
	// of the cell give it
	auto Raw = [&](size_t q, int i, int j, int k, double *out, bool &edge) -> bool
	{
		int cnt = 0, si[3] = { 0, 0, 0 };
		long long sp[3] = { 0, 0, 0 };
		unsigned char kind = 0;
		double pl[12][3];				// the surface points, in cells from the first node of the cell
		unsigned short pd[12];			// and the direction of the surface at each
		for (int v = 0; v < 2; v++)
			for (int u = 0; u < 2; u++)
			{
				size_t e = q + u * step[1] + v * step[2];		// along X from (i, j+u, k+v)
				if (S.cflag[0][e] != 255)
				{
					sp[0] += S.cpos[0][e]; si[1] += j + u; si[2] += k + v;
					pl[cnt][0] = (double)S.cpos[0][e] / SIM_ONE - i; pl[cnt][1] = u; pl[cnt][2] = v;
					pd[cnt++] = S.cdir[0][e];
					if (S.cflag[0][e] > kind) kind = S.cflag[0][e];
				}
				e = q + u * step[0] + v * step[2];				// along Y from (i+u, j, k+v)
				if (S.cflag[1][e] != 255)
				{
					si[0] += i + u; sp[1] += S.cpos[1][e]; si[2] += k + v;
					pl[cnt][0] = u; pl[cnt][1] = (double)S.cpos[1][e] / SIM_ONE - j; pl[cnt][2] = v;
					pd[cnt++] = S.cdir[1][e];
					if (S.cflag[1][e] > kind) kind = S.cflag[1][e];
				}
				e = q + u * step[0] + v * step[1];				// along Z from (i+u, j+v, k)
				if (S.cflag[2][e] != 255)
				{
					si[0] += i + u; si[1] += j + v; sp[2] += S.cpos[2][e];
					pl[cnt][0] = u; pl[cnt][1] = v; pl[cnt][2] = (double)S.cpos[2][e] / SIM_ONE - k;
					pd[cnt++] = S.cdir[2][e];
					if (S.cflag[2][e] > kind) kind = S.cflag[2][e];
				}
			}
		if (!cnt) return false;

		// The mean of the surface points lies on the surface where that is flat or gently curved.
		// Where faces meet in the cell (the directions differ) it is inside the edge: the point
		// is moved to where the planes through the surface points meet, as well as they do.
		double off[3] = { 0, 0, 0 };
		edge = false;
		bool differ = false;
		for (int p = 1; p < cnt && !differ; p++) differ = pd[p] != pd[0];
		if (differ)
		{
			double ml[3] = { 0, 0, 0 };
			for (int p = 0; p < cnt; p++)
				for (int c = 0; c < 3; c++) ml[c] += pl[p][c];
			for (int c = 0; c < 3; c++) ml[c] /= cnt;
			double A[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }, rhs[3] = { 0, 0, 0 };
			int planes = 0;
			for (int p = 0; p < cnt; p++)
			{
				if (!pd[p]) continue;
				const float *nv = g_Dir.v[pd[p]];
				double w = 0;
				for (int c = 0; c < 3; c++) w += nv[c] * (pl[p][c] - ml[c]) * G.g[c];
				for (int c = 0; c < 3; c++)
				{
					rhs[c] += nv[c] * w;
					for (int e2 = 0; e2 < 3; e2++) A[c][e2] += (double)nv[c] * nv[e2];
				}
				planes++;
			}
			if (planes >= 2)
			{
				// the directions in which the planes say something (those along which they all slide are left alone)
				double V[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
				for (int sweep = 0; sweep < 12; sweep++)
				{
					double offd = fabs(A[0][1]) + fabs(A[0][2]) + fabs(A[1][2]);
					if (offd < 1e-12) break;
					for (int p = 0; p < 2; p++)
						for (int r = p + 1; r < 3; r++)
						{
							if (fabs(A[p][r]) < 1e-15) continue;
							double theta = (A[r][r] - A[p][p]) / (2.0 * A[p][r]);
							double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
							double cs = 1.0 / sqrt(t * t + 1.0), sn = t * cs;
							for (int c = 0; c < 3; c++)
							{
								double x = A[c][p], y = A[c][r];
								A[c][p] = cs * x - sn * y;
								A[c][r] = sn * x + cs * y;
							}
							for (int c = 0; c < 3; c++)
							{
								double x = A[p][c], y = A[r][c];
								A[p][c] = cs * x - sn * y;
								A[r][c] = sn * x + cs * y;
							}
							for (int c = 0; c < 3; c++)
							{
								double x = V[c][p], y = V[c][r];
								V[c][p] = cs * x - sn * y;
								V[c][r] = sn * x + cs * y;
							}
						}
				}
				double big = Max2(A[0][0], Max2(A[1][1], A[2][2]));
				int kept = 0, loose = -1;
				for (int e2 = 0; e2 < 3; e2++)
				{
					if (!(A[e2][e2] > SIM_SHARP * big)) { loose = e2; continue; }
					double proj = (V[0][e2] * rhs[0] + V[1][e2] * rhs[1] + V[2][e2] * rhs[2]) / A[e2][e2];
					for (int c = 0; c < 3; c++) off[c] += V[c][e2] * proj;
					kept++;
				}
				edge = kept >= 2;			// faces meet in the cell: an edge, or a corner
				double l[3];
				for (int c = 0; c < 3; c++) l[c] = ml[c] + off[c] / G.g[c];
				if (kept == 2 && (l[0] < 0.0 || l[0] > 1.0 || l[1] < 0.0 || l[1] > 1.0 || l[2] < 0.0 || l[2] > 1.0))
				{
					// An edge between two faces, and its point nearest the middle of the surface points is
					// outside the cell: along the edge to where it is inside the cell, if it is anywhere
					double t0 = -1e300, t1 = 1e300;
					for (int c = 0; c < 3; c++)
					{
						double dl = V[c][loose] / G.g[c];
						if (fabs(dl) < 1e-12)
						{
							if (l[c] < 0.0 || l[c] > 1.0) { t0 = 1.0; t1 = 0.0; }
							continue;
						}
						double ta = (0.0 - l[c]) / dl, tb = (1.0 - l[c]) / dl;
						if (ta > tb) std::swap(ta, tb);
						if (ta > t0) t0 = ta;
						if (tb < t1) t1 = tb;
					}
					if (t0 <= t1)
					{
						double t = t0 > 0.0 ? t0 : t1;			// (0 is outside: the nearer end)
						for (int c = 0; c < 3; c++) l[c] += t * V[c][loose] / G.g[c];
					}
				}
				// not out of the cell
				for (int c = 0; c < 3; c++)
				{
					if (l[c] < 0.0) l[c] = 0.0; else if (l[c] > 1.0) l[c] = 1.0;
					off[c] = (l[c] - ml[c]) * G.g[c];
					if (fabs(off[c]) < 1e-9 * G.g[c]) off[c] = 0.0;
				}
			}
		}
		for (int c = 0; c < 3; c++)
		{
			// (as MeanCoord has it: the points of an uncut face come out exactly where the skin has them)
			double m = G.o[c] + G.g[c] * (((double)si[c] + (double)sp[c] / SIM_ONE) / cnt);
			out[c] = off[c] == 0.0 ? m : m + off[c];
		}
		return true;
	};
	S.cell.assign(NN, -1);
	S.pq.clear();
	S.pc.clear();
	S.pos.clear();
	S.fix.clear();
	for (int k = ca[2]; k < cb[2]; k++)
		for (int j = ca[1]; j < cb[1]; j++)
			for (int i = ca[0]; i < cb[0]; i++)
			{
				size_t q = LI(i, j, k);
				// (no edge of the cell crosses the surface if its eight nodes are all alike)
				unsigned char s0 = S.state[q];
				if (S.state[q + step[0]] == s0 && S.state[q + step[1]] == s0 && S.state[q + step[0] + step[1]] == s0 &&
					S.state[q + step[2]] == s0 && S.state[q + step[2] + step[0]] == s0 && S.state[q + step[2] + step[1]] == s0 &&
					S.state[q + step[2] + step[0] + step[1]] == s0) continue;
				double p[3];
				bool edge;
				if (!Raw(q, i, j, k, p, edge)) continue;
				// in the layer of cells that holds a face of the Stock: where the skin has its points
				bool face = i == -1 || i == G.n[0] - 1 || j == -1 || j == G.n[1] - 1 || k == -1 || k == G.n[2] - 1;
				S.cell[q] = (int)S.pq.size();
				S.pq.push_back(q);
				S.pc.push_back(i); S.pc.push_back(j); S.pc.push_back(k);
				S.pos.push_back(p[0]); S.pos.push_back(p[1]); S.pos.push_back(p[2]);
				S.fix.push_back(edge || face ? 1 : 0);
			}
	size_t np = S.pq.size();

	// Every point half way toward the mean of its neighbours: for each edge of its cell that
	// crosses the surface the points of the three other cells around that edge.  After each
	// pass the points one cell further in from the border of what is looked at are right (a
	// border that is the border of the Stock has nothing beyond it).
	for (int pass = 1; pass <= SIM_RELAX_NOW; pass++)
	{
		int lo[3], hi[3];
		for (int a = 0; a < 3; a++)
		{
			lo[a] = ca[a] == -1 ? -1 : ca[a] + pass;
			hi[a] = cb[a] == G.n[a] ? G.n[a] - 1 : cb[a] - 1 - pass;
		}
		S.pos2 = S.pos;
		for (size_t t = 0; t < np; t++)
		{
			if (S.fix[t]) continue;
			const int *c = &S.pc[3 * t];
			if (c[0] < lo[0] || c[0] > hi[0] || c[1] < lo[1] || c[1] > hi[1] || c[2] < lo[2] || c[2] > hi[2]) continue;
			size_t q = S.pq[t];
			double sum[3] = { 0, 0, 0 };
			int cnt = 0;
			for (int a = 0; a < 3; a++)
			{
				size_t su = step[(a + 1) % 3], sv = step[(a + 2) % 3];
				for (int v = 0; v < 2; v++)
					for (int u = 0; u < 2; u++)
					{
						if (S.cflag[a][q + u * su + v * sv] == 255) continue;
						size_t qu = u ? q + su : q - su, qv = v ? q + sv : q - sv, quv = v ? qu + sv : qu - sv;
						int o[3] = { S.cell[qu], S.cell[qv], S.cell[quv] };
						for (int m = 0; m < 3; m++)
						{
							if (o[m] < 0) continue;
							sum[0] += S.pos[3 * o[m]]; sum[1] += S.pos[3 * o[m] + 1]; sum[2] += S.pos[3 * o[m] + 2];
							cnt++;
						}
					}
			}
			if (cnt)
				for (int a = 0; a < 3; a++) S.pos2[3 * t + a] = 0.5 * S.pos[3 * t + a] + 0.5 * sum[a] / cnt;
		}
		S.pos.swap(S.pos2);
	}

	// Which way the surface looks at a point: the mean of the quads around it (the larger the
	// quad the more it counts), out of the material
	auto Dir = [&](int t) -> unsigned
	{
		if (S.fix[t]) return 0;
		size_t q = S.pq[t];
		double n[3] = { 0, 0, 0 };
		for (int a = 0; a < 3; a++)
		{
			size_t su = step[(a + 1) % 3], sv = step[(a + 2) % 3];
			for (int v = 0; v < 2; v++)
				for (int u = 0; u < 2; u++)
				{
					size_t e = q + u * su + v * sv;			// the node that the edge starts at: the quad has the points of the four cells around it
					if (S.cflag[a][e] == 255) continue;
					int o[4] = { S.cell[e - su - sv], S.cell[e - sv], S.cell[e], S.cell[e - su] };
					if (o[0] < 0 || o[1] < 0 || o[2] < 0 || o[3] < 0) continue;
					const double *p0 = &S.pos[3 * o[0]], *p1 = &S.pos[3 * o[1]], *p2 = &S.pos[3 * o[2]], *p3 = &S.pos[3 * o[3]];
					double d1[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] }, d2[3] = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
					double c[3] = { d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0] };
					double s = S.state[e] ? 1.0 : -1.0;		// (as the quad is turned below)
					n[0] += s * c[0]; n[1] += s * c[1]; n[2] += s * c[2];
				}
		}
		double l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		if (!(l > 0.0)) return 0;
		int x = (int)floor(n[0] / l * 511.0 + 0.5), y = (int)floor(n[1] / l * 511.0 + 0.5), z = (int)floor(n[2] / l * 511.0 + 0.5);
		return ((unsigned)x & 0x3FF) | (((unsigned)y & 0x3FF) << 10) | (((unsigned)z & 0x3FF) << 20);
	};

	// The point of a cell in the mesh, put there when a quad needs it; -1: the surface does not
	// pass through the cell.  A quad has one colour (what made the surface where its edge crosses
	// it), so a point is there once for each kind of quad that uses it.
	S.vid.assign(NN * 3, -2);
	auto Point = [&](size_t q, int want) -> int
	{
		int *ids = &S.vid[q * 3];
		if (ids[want] != -2) return ids[want];
		for (int o = 0; o < 3; o++)
			if (ids[o] >= 0)
			{
				if (M.v.size() >= 65535) return ids[o];		// (no room for more points: the colour of the other kind)
				SimVertex same = M.v[ids[o]];
				same.kind = (float)want;
				ids[want] = (int)M.v.size();
				M.v.push_back(same);
				return ids[want];
			}
		int t = S.cell[q];
		if (t < 0 || M.v.size() >= 65535) return ids[0] = ids[1] = ids[2] = -1;
		SimVertex vx = { (float)S.pos[3 * t], (float)S.pos[3 * t + 1], (float)S.pos[3 * t + 2], (float)want, Dir(t) };
		ids[want] = (int)M.v.size();
		M.v.push_back(vx);
		return ids[want];
	};

	// the sub cells of the faces of the Stock that are built here (the skin has the others)
	unsigned long long fmask[6];
	for (int f = 0; f < 6; f++)
	{
		int a = f / 2, u = (a + 1) % 3, v = (a + 2) % 3;
		fmask[f] = 0;
		if ((f & 1) ? bb[a] != G.nc[a] - 1 : bb[a] != 0) continue;
		std::unordered_map<unsigned long long, unsigned long long>::const_iterator it = G.fine[f].find(SimGrid::Key(bb[u], bb[v], 0));
		if (it != G.fine[f].end()) fmask[f] = it->second;
	}

	// a quad across every crossing edge that starts at a node of this block, turned so that it
	// is seen counter clockwise from outside the material
	for (int k = c0[2]; k < c1[2]; k++)
		for (int j = c0[1]; j < c1[1]; j++)
			for (int i = c0[0]; i < c1[0]; i++)
			{
				size_t q = LI(i, j, k);
				int idx[3] = { i, j, k };
				for (int a = 0; a < 3; a++)
				{
					if (S.cflag[a][q] == 255) continue;
					int u = (a + 1) % 3, v = (a + 2) % 3;
					size_t su = step[u], sv = step[v];
					if (q < su + sv) continue;
					if (idx[a] == -1 || idx[a] + 1 == G.n[a])
					{
						// an edge through a face of the Stock
						int s = (idx[u] + 1) / SIM_SUB, t = (idx[v] + 1) / SIM_SUB;
						if (!((fmask[2 * a + (idx[a] == -1 ? 0 : 1)] >> (((t & 7) << 3) | (s & 7))) & 1)) continue;
					}
					int id[4], kind = S.cflag[a][q] <= SIMKIND_RAPID ? S.cflag[a][q] : SIMKIND_FEED;
					id[0] = Point(q - su - sv, kind);
					id[1] = Point(q - sv, kind);
					id[2] = Point(q, kind);
					id[3] = Point(q - su, kind);
					if (id[0] < 0 || id[1] < 0 || id[2] < 0 || id[3] < 0) continue;
					if (!S.state[q]) std::swap(id[1], id[3]);
					M.t.push_back((unsigned short)id[0]);
					M.t.push_back((unsigned short)id[1]);
					M.t.push_back((unsigned short)id[2]);
					M.t.push_back((unsigned short)id[0]);
					M.t.push_back((unsigned short)id[2]);
					M.t.push_back((unsigned short)id[3]);
				}
			}
#undef LI
}

// The skin: what is left of the faces of the Stock where they are not built in detail.  An
// uncut face has its points on the lines that run through the middle of the cells (and along
// the edges of the face).  Those lines are counted like the cells, -1 .. n - 1.
static inline float DualCoord(const SimGrid &G, int a, int m)
{
	int n = G.n[a];
	if (m < 0) return MeanCoord(G.o[a], G.g[a], 0, 0, 4);
	if (m >= n - 1) return MeanCoord(G.o[a], G.g[a], 2 * (n - 1), 2 * (long long)G.S[a].hi, 4);
	return MeanCoord(G.o[a], G.g[a], 4 * m + 2, 0, 4);
}

// the first and the last of those lines that a sub cell has
static inline int DualLo(int s)
{
	int m = SIM_SUB * s - 2;
	return m < -1 ? -1 : m;
}

static inline int DualHi(const SimGrid &G, int a, int s)
{
	int m = SIM_SUB * s + SIM_SUB - 2;
	return m > G.n[a] - 1 ? G.n[a] - 1 : m;
}

struct SkinRect { int u0, u1, v0, v1; };	// sub cells

// One rectangle of the skin as a fan of triangles around its middle.  Its outline has the
// points that its neighbours have there: a point at every corner of a sub cell, and where it
// lies next to a sub cell that is built in detail a point on every line.  So the whole surface
// stays closed, edge to edge.
static void SkinRectMesh(const SimGrid &G, int f, const SkinRect &r, std::vector<SimMesh> &out)
{
	int a = f / 2, u = (a + 1) % 3, v = (a + 2) % 3;
	std::vector<std::pair<int, int> > P;		// the lines (along u, along v) that the points of the outline are on
	int mu0 = DualLo(r.u0), mu1 = DualHi(G, u, r.u1), mv0 = DualLo(r.v0), mv1 = DualHi(G, v, r.v1);

	P.push_back(std::make_pair(mu0, mv0));
	for (int s = r.u0; s <= r.u1; s++)			// along u at the low side of v
	{
		int m0 = DualLo(s), m1 = DualHi(G, u, s);
		if (G.Detailed(f, s, r.v0 - 1))
			for (int m = m0 + 1; m < m1; m++) P.push_back(std::make_pair(m, mv0));
		P.push_back(std::make_pair(m1, mv0));
	}
	for (int s = r.v0; s <= r.v1; s++)			// along v at the high side of u
	{
		int m0 = DualLo(s), m1 = DualHi(G, v, s);
		if (G.Detailed(f, r.u1 + 1, s))
			for (int m = m0 + 1; m < m1; m++) P.push_back(std::make_pair(mu1, m));
		P.push_back(std::make_pair(mu1, m1));
	}
	for (int s = r.u1; s >= r.u0; s--)			// back along u at the high side of v
	{
		int m0 = DualLo(s), m1 = DualHi(G, u, s);
		if (G.Detailed(f, s, r.v1 + 1))
			for (int m = m1 - 1; m > m0; m--) P.push_back(std::make_pair(m, mv1));
		P.push_back(std::make_pair(m0, mv1));
	}
	for (int s = r.v1; s >= r.v0; s--)			// and back along v at the low side of u
	{
		int m0 = DualLo(s), m1 = DualHi(G, v, s);
		if (G.Detailed(f, r.u0 - 1, s))
			for (int m = m1 - 1; m > m0; m--) P.push_back(std::make_pair(mu0, m));
		if (s > r.v0) P.push_back(std::make_pair(mu0, m0));		// (the last one is the first one again)
	}

	if (P.size() + 1 > 60000)
	{
		// too many points for one mesh: two halves
		SkinRect r1 = r, r2 = r;
		if (r.u1 - r.u0 >= r.v1 - r.v0 && r.u1 > r.u0)
		{
			r1.u1 = (r.u0 + r.u1) / 2;
			r2.u0 = r1.u1 + 1;
		}
		else if (r.v1 > r.v0)
		{
			r1.v1 = (r.v0 + r.v1) / 2;
			r2.v0 = r1.v1 + 1;
		}
		else
			return;						// (one sub cell can not have that many)
		SkinRectMesh(G, f, r1, out);
		SkinRectMesh(G, f, r2, out);
		return;
	}

	if (out.empty() || out.back().v.size() + P.size() + 1 > 65000) out.push_back(SimMesh());
	SimMesh &M = out.back();
	float at = DualCoord(G, a, (f & 1) ? G.n[a] - 1 : -1);
	unsigned short base = (unsigned short)M.v.size();
	float p[3];
	p[a] = at;
	p[u] = (DualCoord(G, u, mu0) + DualCoord(G, u, mu1)) / 2;
	p[v] = (DualCoord(G, v, mv0) + DualCoord(G, v, mv1)) / 2;
	SimVertex mid = { p[0], p[1], p[2], (float)SIMKIND_STOCK };
	M.v.push_back(mid);
	for (size_t i = 0; i < P.size(); i++)
	{
		p[u] = DualCoord(G, u, P[i].first);
		p[v] = DualCoord(G, v, P[i].second);
		SimVertex vx = { p[0], p[1], p[2], (float)SIMKIND_STOCK };
		M.v.push_back(vx);
	}
	for (size_t i = 0; i < P.size(); i++)
	{
		unsigned short i0 = (unsigned short)(base + 1 + i), i1 = (unsigned short)(base + 1 + (i + 1) % P.size());
		M.t.push_back(base);
		// (seen counter clockwise from outside the material)
		M.t.push_back((f & 1) ? i0 : i1);
		M.t.push_back((f & 1) ? i1 : i0);
	}
}

// Rectangles that cover the cells of a grid that are free: row by row the runs of free cells,
// and a run that is the same as one of the row before makes that rectangle taller
template <class F> static void FreeRects(int nu, int nv, F taken, int offU, int offV, std::vector<SkinRect> &done)
{
	std::vector<SkinRect> open, next;
	for (int bv = 0; bv < nv; bv++)
	{
		next.clear();
		size_t i = 0;
		for (int bu = 0; bu < nu; )
		{
			if (taken(bu, bv)) { bu++; continue; }
			int e = bu;
			while (e + 1 < nu && !taken(e + 1, bv)) e++;
			SkinRect run = { bu + offU, e + offU, bv + offV, bv + offV };
			bu = e + 1;
			while (i < open.size() && open[i].u0 < run.u0) done.push_back(open[i++]);
			if (i < open.size() && open[i].u0 == run.u0 && open[i].u1 == run.u1)
			{
				run.v0 = open[i++].v0;
				next.push_back(run);
			}
			else
				next.push_back(run);
		}
		while (i < open.size()) done.push_back(open[i++]);
		open.swap(next);
	}
	done.insert(done.end(), open.begin(), open.end());
}

static void BuildSkin(const SimGrid &G, std::vector<SimMesh> &out)
{
	out.clear();
	for (int f = 0; f < 6; f++)
	{
		int a = f / 2, u = (a + 1) % 3, v = (a + 2) % 3;
		int nu = G.nc[u], nv = G.nc[v];
		const std::vector<unsigned char> &T = G.face[f];
		std::vector<SkinRect> blocks, rects;

		// the blocks of the face without anything built in detail, as rectangles of sub cells
		FreeRects(nu, nv, [&](int bu, int bv) { return T[(size_t)bv * nu + bu] != 0; }, 0, 0, blocks);
		for (size_t k = 0; k < blocks.size(); k++)
		{
			SkinRect r = { blocks[k].u0 << 3, MinI((blocks[k].u1 << 3) + 7, G.ns[u] - 1), blocks[k].v0 << 3, MinI((blocks[k].v1 << 3) + 7, G.ns[v] - 1) };
			rects.push_back(r);
		}

		// and in the other blocks the sub cells that are not
		for (std::unordered_map<unsigned long long, unsigned long long>::const_iterator it = G.fine[f].begin(); it != G.fine[f].end(); ++it)
		{
			int bu = (int)(it->first & 0x1FFFFF), bv = (int)((it->first >> 21) & 0x1FFFFF);
			unsigned long long mask = it->second;
			if (mask == ~0ULL) continue;
			int cu = MinI(8, G.ns[u] - (bu << 3)), cv = MinI(8, G.ns[v] - (bv << 3));
			FreeRects(cu, cv, [&](int s, int t) { return ((mask >> ((t << 3) | s)) & 1) != 0; }, bu << 3, bv << 3, rects);
		}

		for (size_t k = 0; k < rects.size(); k++) SkinRectMesh(G, f, rects[k], out);
	}
}

// A mesh waits as it is until it is drawn: no room in it that is not used
static void Tight(SimMesh &M)
{
	if (M.v.capacity() > M.v.size()) std::vector<SimVertex>(M.v).swap(M.v);
	if (M.t.capacity() > M.t.size()) std::vector<unsigned short>(M.t).swap(M.t);
}


// What a Stock was cut with before the path: the cuts of the Stock file that it was loaded from
struct SimBefore
{
	std::vector<SimPose> poses;			// (tool: which of tools)
	std::vector<CSimCutTool> tools;
	double res, bytes;					// the resolution that the Stock had when it was saved, and what it took then
	SimFrozen *frozen;					// the lines as these cuts leave them, on one lattice: those of the file, or kept
										// when the cuts were made again (the cutting thread's own once it has this)
	SimBefore() : res(0), bytes(0), frozen(NULL) {}
	~SimBefore() { delete frozen; }
private:
	SimBefore(const SimBefore &);
	SimBefore &operator=(const SimBefore &);
};

// The finest lattice that a Stock can have: the resolution that is asked for (0: chosen here), but
// not finer than the nodes can be counted, and than the surface can be drawn (its points are floats)
static double Finest(const double lo[3], const double hi[3], double wanted)
{
	double longest = 0, farthest = 0;
	for (int a = 0; a < 3; a++)
	{
		if (hi[a] - lo[a] > longest) longest = hi[a] - lo[a];
		farthest = Max2(farthest, Max2(fabs(lo[a]), fabs(hi[a])));
	}
	double finest = wanted > 0.0 ? wanted : Min2(SIM_AUTO_FINEST, longest / SIM_AUTO_CELLS);
	finest = Max2(finest, longest / (SIM_MAX_NODES - 2));
	return Max2(finest, farthest * 64.0 * FLT_EPSILON);
}


//////////////////////////////////////////////////////////////////////////////
// The Stock file
//////////////////////////////////////////////////////////////////////////////
//
//   head      SimFileHead
//   tools     of each: number of points of its profile, open or not (two 32 bit numbers), the
//             heights and then the radii (doubles)
//   poses     SimPose each; tool is one of the tools above (any other number: no tool, the pose
//             cuts nothing).  In a file of version 1 a pose is its first 16 bytes: there the tool
//             always stands upright.
//   lines     (SIMFILE_LINES) of the lines along X, Y and Z in turn: number of chunks, and of
//             each chunk which it is (cu, cv), how many bytes follow, and its SIM_LC x SIM_LC
//             lines: a 16 bit number that is the number of pieces of the line, SIMFILE_FULL (the
//             line is uncut) or SIMFILE_SAME (as the line before it in the chunk), and for a
//             number of pieces the two ends of each (32 bit), then the directions of the
//             surface at all the ends (16 bit), then what made them (8 bit);
//             then the number of blocks whose surface is built in detail and their keys
//             (64 bit), and of each of the six faces of the Stock the number of blocks with
//             sub cells built in detail, and their keys and sub cells (two 64 bit numbers)
//   sum       of all the bytes before it (64 bit, see SimSum)
//
// All numbers as an x86 keeps them.

#define SIMFILE_MAGIC		"KMotionCNC Stock"	// (16 characters)
#define SIMFILE_VERSION		2
#define SIMFILE_LINES		1			// flag: the file has the lines
#define SIMFILE_FULL		0xFFFF		// in place of the number of pieces of a line: the line is uncut
#define SIMFILE_SAME		0xFFFE		// ... it is the same as the line before it in the chunk
#define SIMFILE_MAX_POINTS	100000		// points of the profile of a tool at most
#define SIMFILE_MAX_CHUNK	((size_t)SIM_LC * SIM_LC * (2 + (size_t)SIM_MAX_PIECES * SIM_PIECE_BYTES))

#pragma pack(push, 1)
struct SimFileHead
{
	char magic[16];
	unsigned version;
	unsigned flags;					// SIMFILE_xxx
	double lo[3], hi[3];			// the Stock
	double wanted;					// the resolution that was asked for it (0: chosen by the Stock)
	double res;						// ... and the one that its lattice had
	double bytes;					// what the lines and the surface took on that lattice
	int n[3];						// nodes of that lattice
	unsigned tools, poses;			// how many follow
};
#pragma pack(pop)

#define SIMFILE_POSE1		16			// bytes of a pose in a file of version 1 (without the direction of the tool)

static_assert(sizeof(SimFileHead) == 116 && sizeof(SimPose) == 28 && sizeof(SimPos) == 4, "the layout of a Stock file");

static const char SimFileDamaged[] = "The Stock file is damaged";
static const char SimFileMemory[] = "There is not enough memory to load the Stock";

// The sum that a Stock file ends with: all the bytes before it, eight at a time
struct SimSum
{
	unsigned long long h, part, total;
	int have;
	SimSum() : h(14695981039346656037ULL), part(0), total(0), have(0) {}
	void Mix(unsigned long long w)
	{
		h = (h ^ w) * 1099511628211ULL;
		h ^= h >> 29;
	}
	void Add(const void *p, size_t n)
	{
		const unsigned char *b = (const unsigned char *)p;
		total += n;
		for (; n && have; n--)
		{
			part |= (unsigned long long)*b++ << (8 * have);
			if (++have == 8)
			{
				Mix(part);
				part = 0;
				have = 0;
			}
		}
		for (; n >= 8; n -= 8, b += 8)
		{
			unsigned long long w;
			memcpy(&w, b, 8);
			Mix(w);
		}
		for (; n; n--) part |= (unsigned long long)*b++ << (8 * have++);
	}
	unsigned long long End()
	{
		if (have) Mix(part);
		Mix(total);
		return h;
	}
};

struct SimOut
{
	FILE *f;
	SimSum sum;
	bool bad;
	SimOut(FILE *file) : f(file), bad(false) {}
	void Put(const void *p, size_t n)
	{
		if (bad || !n) return;
		if (fwrite(p, 1, n, f) != n) bad = true;
		sum.Add(p, n);
	}
	template <class T> void Val(const T &v) { Put(&v, sizeof(T)); }
};

struct SimIn
{
	FILE *f;
	SimSum sum;
	long long left;					// bytes to come before the sum
	bool bad;
	SimIn(FILE *file, long long bytes) : f(file), left(bytes), bad(false) {}
	bool Get(void *p, size_t n)
	{
		if (!n) return !bad;
		if (bad || (long long)n > left || fread(p, 1, n, f) != n)
		{
			bad = true;
			return false;
		}
		left -= (long long)n;
		sum.Add(p, n);
		return true;
	}
	template <class T> bool Val(T &v) { return Get(&v, sizeof(T)); }
};

// The lines of a chunk as the file has them
static void PackChunk(const SimLines &L, int cu, int cv, const SimChunk *ch, std::vector<unsigned char> &buf)
{
	buf.clear();
	const SimDex *prev = &L.full;
	for (int l = 0; l < SIM_LC * SIM_LC; l++)
	{
		int u = (cu << SIM_LC_BITS) | (l & (SIM_LC - 1)), v = (cv << SIM_LC_BITS) | (l >> SIM_LC_BITS);
		const SimDex &d = ch->d[l];
		bool in = u < L.nu && v < L.nv;		// (a chunk at the edge has lines that are not of the lattice)
		unsigned short code = !in || d.Same(L.full) ? (unsigned short)SIMFILE_FULL : d.Same(*prev) ? (unsigned short)SIMFILE_SAME : d.n;
		size_t at = buf.size(), n = code < SIMFILE_SAME ? code : 0;
		buf.resize(at + 2 + n * SIM_PIECE_BYTES);
		memcpy(&buf[at], &code, 2);
		if (n)
		{
			unsigned char *p = &buf[at + 2];
			SimPos *pos = (SimPos *)p;
			unsigned short *dir = (unsigned short *)(p + n * 2 * sizeof(SimPos));
			unsigned char *flg = p + n * 2 * (sizeof(SimPos) + sizeof(unsigned short));
			for (size_t i = 0; i < n; i++)
			{
				SimPos a = d.Lo((int)i), b = d.Hi((int)i);
				unsigned short na = d.NLo((int)i), nb = d.NHi((int)i);
				memcpy(pos + 2 * i, &a, sizeof(a));
				memcpy(pos + 2 * i + 1, &b, sizeof(b));
				memcpy(dir + 2 * i, &na, sizeof(na));
				memcpy(dir + 2 * i + 1, &nb, sizeof(nb));
				flg[2 * i] = d.FLo((int)i);
				flg[2 * i + 1] = d.FHi((int)i);
			}
		}
		prev = code == SIMFILE_FULL ? &L.full : &d;
	}
}

// The lines of a chunk from a file into the chunk (cu, cv) of L, which is made.  False: they are
// not lines that a Stock can have.  (No memory: std::bad_alloc.)
static bool UnpackChunk(SimLines &L, int cu, int cv, const std::vector<unsigned char> &buf)
{
	SimChunk *ch = L.MakeChunk(cu, cv);		// (every line of it uncut)
	SimRoom room = { L.heap, 0 };
	const unsigned char *p = buf.empty() ? NULL : &buf[0], *end = p + buf.size();
	const SimDex *prev = &L.full;
	bool ok = true;
	for (int l = 0; l < SIM_LC * SIM_LC && ok; l++)
	{
		SimDex &d = ch->d[l];
		unsigned short code;
		if (end - p < 2)
		{
			ok = false;
			break;
		}
		memcpy(&code, p, 2);
		p += 2;
		if (code == SIMFILE_FULL)
		{
			prev = &L.full;
			continue;
		}
		if (code == SIMFILE_SAME)
		{
			if (!d.Copy(*prev, &room)) throw std::bad_alloc();
			prev = &d;
			continue;
		}
		size_t n = code;
		if (n > SIM_MAX_PIECES || (size_t)(end - p) < n * SIM_PIECE_BYTES)
		{
			ok = false;
			break;
		}
		const unsigned char *pos = p, *dir = p + n * 2 * sizeof(SimPos), *flg = dir + n * 2 * sizeof(unsigned short);
		SimPos a = 0, b = 0, last = 0;
		unsigned short na = 0, nb = 0;
		for (size_t i = 0; i < n && ok; i++)
		{
			memcpy(&a, pos + 8 * i, 4);
			memcpy(&b, pos + 8 * i + 4, 4);
			memcpy(&na, dir + 4 * i, 2);
			memcpy(&nb, dir + 4 * i + 2, 2);
			// pieces in ascending order inside the line, directions that the table has, and made by something that there is
			if (!(a < b) || b > L.hi || (i > 0 && a < last) || na > 255 * 255 || nb > 255 * 255 ||
				flg[2 * i] > SIMKIND_RAPID || flg[2 * i + 1] > SIMKIND_RAPID) ok = false;
			last = b;
		}
		if (!ok) break;
		if (n == 0)
		{
			d.n = 0;
			d.cap = 0;
		}
		else if (n == 1)
		{
			d.Init(a, b, na, nb);
			d.one.flo = flg[0];
			d.one.fhi = flg[1];
		}
		else
		{
			SimPos *nv = (SimPos *)SimHeapAlloc(room.heap, n * SIM_PIECE_BYTES, false);
			if (!nv) throw std::bad_alloc();
			memcpy(nv, p, n * SIM_PIECE_BYTES);		// (the file has them as they lie in memory)
			d.v = nv;
			d.n = d.cap = (unsigned short)n;
			room.grown += (long long)n * (long long)SIM_PIECE_BYTES + SIM_BLOCK_BYTES;
		}
		p += n * SIM_PIECE_BYTES;
		prev = &d;
	}
	L.pieceBytes += room.grown;
	L.Count(ch, cu, cv);
	return ok && p == end;
}

// Read a Stock file.  Returns NULL and what it holds, or what is wrong with it.  The lines are
// left out if they would take more memory than there is for a Stock (the cuts are then made again).
static const char *ReadStock(const SimChar *file, std::shared_ptr<SimBefore> &out, double lo[3], double hi[3], double *wanted)
{
	FILE *f = SimFopen(file, "rb");
	if (!f) return "The Stock file could not be opened";
	struct Closer
	{
		FILE *f;
		~Closer() { fclose(f); }
	} closer = { f };
	setvbuf(f, NULL, _IOFBF, 1 << 20);

	if (SimFseek(f, 0, SEEK_END)) return SimFileDamaged;
	long long size = SimFtell(f);
	if (SimFseek(f, 0, SEEK_SET)) return SimFileDamaged;

	SimFileHead h;
	SimIn in(f, size - 8);
	if (size < (long long)sizeof(h) + 8 || !in.Get(&h, sizeof(h)) || memcmp(h.magic, SIMFILE_MAGIC, 16)) return "This is not a Stock file";
	if (h.version > SIMFILE_VERSION) return "The Stock file was made by a later version of KMotionCNC";
	if (h.version < 1) return SimFileDamaged;
	// (sizes as the G Viewer Setup takes them, and a lattice that there can be)
	for (int a = 0; a < 3; a++)
		if (!SimFinite(h.lo[a]) || !SimFinite(h.hi[a]) || !(h.hi[a] > h.lo[a]) || fabs(h.lo[a]) > 1e9 || fabs(h.hi[a]) > 1e9 ||
			h.n[a] < 2 || h.n[a] > SIM_MAX_NODES) return SimFileDamaged;
	if (!SimFinite(h.wanted) || h.wanted < 0.0 || h.wanted > 1e9 || !SimFinite(h.res) || !(h.res > 0.0) || h.res > 1e9 ||
		!SimFinite(h.bytes) || h.bytes < 0.0 || h.tools > 65535) return SimFileDamaged;

	try
	{
		std::shared_ptr<SimBefore> B(new SimBefore);
		B->res = h.res;
		B->bytes = h.bytes;

		for (unsigned t = 0; t < h.tools; t++)
		{
			unsigned points, open;
			if (!in.Val(points) || !in.Val(open) || points > SIMFILE_MAX_POINTS || (long long)points * 16 > in.left) return SimFileDamaged;
			CSimCutTool T;
			T.h.resize(points);
			T.r.resize(points);
			if (points && (!in.Get(&T.h[0], (size_t)points * 8) || !in.Get(&T.r[0], (size_t)points * 8))) return SimFileDamaged;
			for (unsigned i = 0; i < points; i++)
				if (!SimFinite(T.h[i]) || !SimFinite(T.r[i]) || T.r[i] < 0.0) return SimFileDamaged;
			T.Open = open != 0;
			T.Finish();
			B->tools.push_back(T);
		}

		size_t each = h.version >= 2 ? sizeof(SimPose) : SIMFILE_POSE1;
		if ((long long)h.poses * (long long)each > in.left) return SimFileDamaged;
		B->poses.resize(h.poses);
		if (h.version >= 2)
		{
			if (h.poses && !in.Get(&B->poses[0], (size_t)h.poses * sizeof(SimPose))) return SimFileDamaged;
		}
		else
			for (unsigned i = 0; i < h.poses; i++)
			{
				SimPose &p = B->poses[i];
				if (!in.Get(&p, SIMFILE_POSE1)) return SimFileDamaged;
				p.ax = p.ay = 0.0f;
				p.az = 1.0f;
			}
		for (unsigned i = 0; i < h.poses; i++)
		{
			const SimPose &p = B->poses[i];
			if (!SimFinite(p.x) || !SimFinite(p.y) || !SimFinite(p.z) || !SimFinite(p.ax) || !SimFinite(p.ay) || !SimFinite(p.az)) return SimFileDamaged;
		}

		if (h.flags & SIMFILE_LINES)
		{
			// The lines are of use if they, and the surface that goes with them, are within what
			// there is for a Stock in this program.  If not they are only read past.
			SimFrozen *F = NULL;
			int nc[3];
			for (int a = 0; a < 3; a++) nc[a] = (h.n[a] + 1 + SIM_CS - 1) / SIM_CS;
			if (h.bytes <= Budget(!(h.wanted > 0.0)) && !LowMemory())
			{
				F = B->frozen = new SimFrozen;
				for (int a = 0; a < 3; a++)
				{
					F->n[a] = h.n[a];
					F->lo[a] = h.lo[a];
					F->hi[a] = h.hi[a];
				}
				F->res = h.res;
				F->finest = Finest(h.lo, h.hi, h.wanted);
				for (int a = 0; a < 3; a++)
				{
					int u = (a + 1) % 3, v = (a + 2) % 3;
					double ax[3] = { 0, 0, 0 };
					ax[a] = 1;
					F->S[a].Setup(h.n[u], h.n[v], h.n[a], (double)(float)h.lo[a], (h.hi[a] - h.lo[a]) / (h.n[a] - 1),
						DirCode(-ax[0], -ax[1], -ax[2]), DirCode(ax[0], ax[1], ax[2]));
				}
			}

			std::vector<unsigned char> buf;
			for (int a = 0; a < 3; a++)
			{
				unsigned chunks;
				if (!in.Val(chunks)) return SimFileDamaged;
				for (unsigned c = 0; c < chunks; c++)
				{
					unsigned head[3];			// cu, cv, bytes
					if (!in.Get(head, sizeof(head)) || head[2] > SIMFILE_MAX_CHUNK || (long long)head[2] > in.left) return SimFileDamaged;
					buf.resize(head[2]);
					if (head[2] && !in.Get(&buf[0], head[2])) return SimFileDamaged;
					if (!F) continue;
					SimLines &L = F->S[a];
					if (head[0] > (unsigned)((L.nu - 1) >> SIM_LC_BITS) || head[1] > (unsigned)((L.nv - 1) >> SIM_LC_BITS) ||
						L.Chunk((int)head[0], (int)head[1])) return SimFileDamaged;
					try
					{
						if (!UnpackChunk(L, (int)head[0], (int)head[1], buf)) return SimFileDamaged;
					}
					catch (std::bad_alloc &)		// no memory for them after all
					{
						delete F;
						F = B->frozen = NULL;
					}
				}
			}

			unsigned keys;
			if (!in.Val(keys) || (long long)keys * 8 > in.left) return SimFileDamaged;
			std::vector<unsigned long long> k(keys);
			if (keys && !in.Get(&k[0], (size_t)keys * 8)) return SimFileDamaged;
			for (unsigned i = 0; i < keys; i++)
				if ((int)(k[i] & 0x1FFFFF) >= nc[0] || (int)((k[i] >> 21) & 0x1FFFFF) >= nc[1] || (k[i] >> 42) >= (unsigned long long)nc[2]) return SimFileDamaged;
			if (F) F->key.swap(k);
			for (int fc = 0; fc < 6; fc++)
			{
				int u = (fc / 2 + 1) % 3, v = (fc / 2 + 2) % 3;
				unsigned cnt;
				if (!in.Val(cnt) || (long long)cnt * 16 > in.left) return SimFileDamaged;
				for (unsigned i = 0; i < cnt; i++)
				{
					unsigned long long kv[2];
					if (!in.Get(kv, sizeof(kv))) return SimFileDamaged;
					if ((int)(kv[0] & 0x1FFFFF) >= nc[u] || (int)((kv[0] >> 21) & 0x1FFFFF) >= nc[v] || (kv[0] >> 42) != 0) return SimFileDamaged;
					if (F) F->fine[fc][kv[0]] = kv[1];
				}
			}
		}

		unsigned long long sum = 0;
		if (in.bad || in.left != 0 || fread(&sum, 1, 8, f) != 8 || sum != in.sum.End()) return SimFileDamaged;

		for (int a = 0; a < 3; a++)
		{
			lo[a] = h.lo[a];
			hi[a] = h.hi[a];
		}
		*wanted = h.wanted;
		out = B;
	}
	catch (std::bad_alloc &)
	{
		return SimFileMemory;
	}
	return NULL;
}


//////////////////////////////////////////////////////////////////////////////
// CSimCutStock
//////////////////////////////////////////////////////////////////////////////

CSimCutStock::CSimCutStock()
{
	m_Target = 0;
	m_All = false;
	m_DirtyFrom = INT_MAX;
	m_BoxPending = m_BoxWanted = false;
	m_BoxRes = 0;
	for (int a = 0; a < 3; a++) m_BoxLo[a] = m_BoxHi[a] = 0;
	m_Republish = false;
	m_Awaited = false;
	m_StatsWanted = false;
	memset(&m_Stats, 0, sizeof(m_Stats));
	m_Quit = false;
	m_Idle = true;
	m_Started = false;
	m_Failed = false;
	m_NewPath = m_Waiting = false;
	m_Changes = 0;
	m_RefineLeft = true;
	m_ResInUse = 0;
	m_BaseCount = m_BaseApplied = 0;
	m_BoxMaking = false;
	m_SaveWanted = m_SaveBusy = false;
	m_SaveError = NULL;
	m_Generation = 0;
	m_Grid = NULL;
	m_Applied = 0;
	m_Stale = false;
	m_Shown = 0;
	m_Batch = 64;
	m_CutSeconds = m_MeshSeconds = 0;
	m_LatticeCut = 0;
	m_TiltWork = 16;
	m_TiltStart = true;
}

CSimCutStock::~CSimCutStock()
{
	{
		std::lock_guard<std::mutex> lk(m_Lock);
		m_Quit = true;
	}
	m_Wake.notify_all();
	if (m_Thread.joinable()) m_Thread.join();
	delete m_Grid;
}

int CSimCutStock::ThreadsToUse()
{
	int hc = (int)std::thread::hardware_concurrency();
	int t = hc > 3 ? hc - 2 : hc > 1 ? hc - 1 : 1;	// leave some for the Interpreter and the screen
	return t > 16 ? 16 : t;
}

void CSimCutStock::Start()		// m_Lock held
{
	if (m_Started) return;
	m_Started = true;
	m_Idle = false;
	m_Thread = std::thread([this]() { Run(); });
}

void CSimCutStock::Define(const double lo[3], const double hi[3], double resolution)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	for (int a = 0; a < 3; a++)
	{
		m_BoxLo[a] = Min2(lo[a], hi[a]);
		m_BoxHi[a] = Max2(lo[a], hi[a]);
	}
	m_BoxRes = resolution;
	m_BoxWanted = m_BoxPending = true;
	m_Changes++;
	Start();
	m_Wake.notify_all();
}

void CSimCutStock::Remove()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (!m_Started) return;
	m_BoxWanted = false;
	m_BoxPending = true;
	m_Changes++;
	m_Wake.notify_all();
}

bool CSimCutStock::Defined()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return m_BoxPending ? m_BoxWanted : m_Grid != NULL;
}

void CSimCutStock::ClearPoses()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	m_Poses.clear();
	m_Target = 0;
	m_All = false;
	m_DirtyFrom = 0;
	m_NewPath = true;
	m_Changes++;
	m_Wake.notify_all();
}

int CSimCutStock::AddTool(const SimChar *image, double scale, const double offset[3], double diameter)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (!image) image = SIMTEXT("");
	for (int i = (int)m_ToolDefs.size() - 1; i >= 0; i--)
	{
		const ToolDef &t = m_ToolDefs[i];
		if (!t.given && !t.old && t.diameter == diameter && t.scale == scale && t.offset[0] == offset[0] &&
			t.offset[1] == offset[1] && t.offset[2] == offset[2] && t.image == image) return i;
	}
	if (m_ToolDefs.size() >= 65535) return 0;
	ToolDef t;
	t.image = image;
	t.old = false;
	t.scale = scale;
	t.offset[0] = offset[0];
	t.offset[1] = offset[1];
	t.offset[2] = offset[2];
	t.diameter = diameter;
	t.given = false;
	m_ToolDefs.push_back(t);
	return (int)m_ToolDefs.size() - 1;
}

// Tool image files may have been written again: a tool that is added from now on is read from
// its file even if a tool of that file is known.  What was cut stays as it was cut.
void CSimCutStock::ForgetToolImages()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	for (size_t i = 0; i < m_ToolDefs.size(); i++)
		if (!m_ToolDefs[i].given) m_ToolDefs[i].old = true;
}

int CSimCutStock::AddTool(const CSimCutTool &profile)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (m_ToolDefs.size() >= 65535) return 0;
	ToolDef t;
	t.scale = 1.0;
	t.offset[0] = t.offset[1] = t.offset[2] = 0.0;
	t.diameter = 0.0;
	t.given = true;
	t.old = false;
	t.profile = profile;
	t.profile.Finish();
	m_ToolDefs.push_back(t);
	return (int)m_ToolDefs.size() - 1;
}

// a pose stands upright: the tool points along +Z
static inline bool Upright(const SimPose &p)
{
	return fabs(p.ax) < 1e-7f && fabs(p.ay) < 1e-7f && p.az > 0.0f;
}

void CSimCutStock::AddPose(float x, float y, float z, int tool, int flags, const double *axis)
{
	// which way the tool points, as a unit vector (nothing given, or nothing that is a direction: upright)
	double len = axis ? sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) : 0.0;
	bool given = axis && len > 1e-12 && len < 1e30;

	std::lock_guard<std::mutex> lk(m_Lock);
	SimPose p;
	p.x = x;
	p.y = y;
	p.z = z;
	p.tool = (unsigned short)tool;
	p.flags = (unsigned char)flags;
	p.spare = 0;
	p.ax = given ? (float)(axis[0] / len) : 0.0f;
	p.ay = given ? (float)(axis[1] / len) : 0.0f;
	p.az = given ? (float)(axis[2] / len) : 1.0f;
	m_Poses.push_back(p);
	m_Changes++;
	if ((m_Idle || m_Waiting) && m_All) m_Wake.notify_all();
}

void CSimCutStock::SetPoseCount(int count)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (count < 0) count = 0;
	if (count >= (int)m_Poses.size()) return;
	m_Poses.resize(count);
	if (count < m_DirtyFrom) m_DirtyFrom = count;
	m_Changes++;
	m_Wake.notify_all();
}

void CSimCutStock::MovePose(int index, float x, float y, float z)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (index < 0 || index >= (int)m_Poses.size()) return;
	SimPose &p = m_Poses[index];
	if (p.x == x && p.y == y && p.z == z) return;
	if ((p.flags & SIMPOSE_MOVE) && index > 0)
	{
		// A move that ends sooner (the tool was stopped on its way): the tool has turned from the
		// direction that it had at the pose before by as much less as it has travelled less
		const SimPose &q = m_Poses[index - 1];
		double full = sqrt(((double)p.x - q.x) * ((double)p.x - q.x) + ((double)p.y - q.y) * ((double)p.y - q.y) + ((double)p.z - q.z) * ((double)p.z - q.z));
		double part = sqrt(((double)x - q.x) * ((double)x - q.x) + ((double)y - q.y) * ((double)y - q.y) + ((double)z - q.z) * ((double)z - q.z));
		if (part < full && (p.ax != q.ax || p.ay != q.ay || p.az != q.az))
		{
			double f = part / full, v[3] = { q.ax + f * ((double)p.ax - q.ax), q.ay + f * ((double)p.ay - q.ay), q.az + f * ((double)p.az - q.az) };
			double len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			if (len > 1e-6)
			{
				p.ax = (float)(v[0] / len);
				p.ay = (float)(v[1] / len);
				p.az = (float)(v[2] / len);
			}
		}
	}
	p.x = x;
	p.y = y;
	p.z = z;
	if (index < m_DirtyFrom) m_DirtyFrom = index;
	m_Changes++;
	m_Wake.notify_all();
}

// A stretch of the path that the tool does not travel after all (what is left of a preview
// when a run plots its own path after it).  The poses stay in their places, so that the path
// of the viewer and this copy of it still go together, but nothing is cut with them.
void CSimCutStock::DisablePoses(int first, int count)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (first < 0) first = 0;
	int end = first + count, dirty = INT_MAX;
	if (end > (int)m_Poses.size()) end = (int)m_Poses.size();
	for (int i = first; i < end; i++)
		if (!(m_Poses[i].flags & SIMPOSE_OFF))
		{
			m_Poses[i].flags |= SIMPOSE_OFF;
			if (dirty == INT_MAX) dirty = i;
		}
	if (dirty == INT_MAX) return;
	if (dirty < m_DirtyFrom) m_DirtyFrom = dirty;
	m_Changes++;
	m_Wake.notify_all();
}

int CSimCutStock::NbPoses()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return (int)m_Poses.size();
}

bool CSimCutStock::GetPose(int index, SimPose *p)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (index < 0 || index >= (int)m_Poses.size()) return false;
	*p = m_Poses[index];
	return true;
}

void CSimCutStock::CutAll()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (m_All) return;
	m_All = true;
	m_Changes++;
	m_Wake.notify_all();
}

void CSimCutStock::CutTo(int count)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (m_All || count <= m_Target) return;
	m_Target = count;
	m_Changes++;
	m_Wake.notify_all();
}

void CSimCutStock::UncutFrom(int count)
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (count < 0) count = 0;
	int t = EffTarget();
	m_All = false;
	m_Target = t < count ? t : count;
	if (count < m_DirtyFrom) m_DirtyFrom = count;
	m_Changes++;
	m_Wake.notify_all();
}

void CSimCutStock::Republish()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (!m_Started) return;
	m_Republish = true;
	m_Awaited = m_Grid != NULL || m_BoxPending;
	m_Wake.notify_all();
}

// After Republish: waits until the cutting thread has handed all the surface over again.  For an
// uncut Stock, which is what there is when the viewer has just cleared its path, that takes it
// a moment; with a surface that takes longer the wait ends when the time is up.
bool CSimCutStock::WaitSurface(int milliseconds)
{
	std::unique_lock<std::mutex> lk(m_Lock);
	if (!m_Started) return true;
	return m_IdleCv.wait_for(lk, std::chrono::milliseconds(milliseconds), [this]() { return !m_Awaited || m_Quit; });
}

int CSimCutStock::EffTarget() const		// m_Lock held
{
	int n = (int)m_Poses.size();
	return m_All || m_Target > n ? n : m_Target;
}

bool CSimCutStock::Pending() const		// m_Lock held: is there something for the cutting thread to do?
{
	return m_BoxPending ||
		(m_Grid && (m_DirtyFrom < m_Applied || m_Republish || m_Applied < EffTarget() || m_BaseApplied < m_BaseCount || m_SaveWanted ||
					m_Grid->anyDirty || RefineDue()));
}

// m_Lock held: the lattice is coarser than was asked for, and for the path that is cut now it has
// not been looked at yet whether a finer one would do (see Run).  Not for a Stock that was loaded:
// it keeps the lattice that it has, on which the cuts of its file need not be made again.
bool CSimCutStock::RefineDue() const
{
	return m_RefineLeft && m_Grid && m_Applied > 0 && m_BaseCount == 0 && m_Grid->res > m_Grid->finest * 1.02;
}

bool CSimCutStock::Busy()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return m_Started && (!m_Idle || Pending());
}

double CSimCutStock::ResolutionInUse()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return m_ResInUse;
}

bool CSimCutStock::Failed()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return m_Failed;
}

bool CSimCutStock::WaitIdle(int milliseconds)
{
	std::unique_lock<std::mutex> lk(m_Lock);
	if (!m_Started) return true;
	m_Wake.notify_all();
	return m_IdleCv.wait_for(lk, std::chrono::milliseconds(milliseconds), [this]() { return m_Idle && !Pending(); });
}

void CSimCutStock::GetStats(SimStats *s)
{
	std::unique_lock<std::mutex> lk(m_Lock);
	memset(s, 0, sizeof(*s));
	if (!m_Started) return;
	m_StatsWanted = true;
	m_Wake.notify_all();
	m_IdleCv.wait(lk, [this]() { return !m_StatsWanted || m_Quit; });
	*s = m_Stats;
}

void CSimCutStock::FillStats(SimStats *s)	// cutting thread, nothing else running
{
	memset(s, 0, sizeof(*s));
	s->applied = m_Applied;
	s->cutSeconds = m_CutSeconds;
	s->meshSeconds = m_MeshSeconds;
	s->before = m_BaseCount;
	s->beforeApplied = m_BaseApplied;
	SimGrid *G = m_Grid;
	if (!G) return;
	for (int a = 0; a < 3; a++)
	{
		s->nodes[a] = G->n[a];
		s->origin[a] = G->o[a];
		s->step[a] = G->g[a];
	}
	for (int a = 0; a < 3; a++)
	{
		int u = (a + 1) % 3, v = (a + 2) % 3;
		const SimLines &L = G->S[a];
		double lines = (double)L.nu * L.nv;
		// A line stands for the cell of the cross section around it, half a cell at the faces.
		// Uncut, all of them together hold the Stock; the lines that exist tell what is missing.
		double full = (double)L.hi - (double)L.lo, missing = 0;		// (in positions along the line)
		s->pieces += lines;
		EachChunk(L, [&](int cu, int cv, const SimChunk *ch)
		{
			int ub = cu << SIM_LC_BITS, vb = cv << SIM_LC_BITS;
			for (int lv = 0; lv < SIM_LC && vb + lv < L.nv; lv++)
				for (int lu = 0; lu < SIM_LC && ub + lu < L.nu; lu++)
				{
					const SimDex &d = ch->d[(lv << SIM_LC_BITS) | lu];
					int iu = ub + lu, iv = vb + lv;
					double len = 0;
					for (int p = 0; p < d.n; p++) len += (double)d.Hi(p) - (double)d.Lo(p);
					double w = (iu == 0 || iu == L.nu - 1 ? 0.5 : 1.0) * (iv == 0 || iv == L.nv - 1 ? 0.5 : 1.0);
					missing += (full - len) * w;
					s->pieces += d.n - 1;
					if (d.n > 1) s->multi[a] += 1.0;
					s->stored += 1.0;
				}
		});
		s->volume[a] = (full * (L.nu - 1) * (L.nv - 1) - missing) / SIM_ONE * G->g[a] * G->g[u] * G->g[v];
		s->lines += lines;
	}
	s->triangles = G->triangles;
	s->guessed = G->guessed;
	s->bytes = G->Bytes();
	s->wanted = G->wanted;
	s->coarsened = G->coarsened;
}

// m_MeshLock held: the surface that was handed over is no more (the object that draws it frees
// what it has of it when it sees that the blocks are laid out anew)
void CSimCutStock::ForgetSurface()
{
	std::vector<SimMesh>().swap(m_Meshes);
	m_Generation++;
	m_Stale = false;
}

// A new lattice (lo == NULL: none).  wanted is the resolution that was asked for (0: chosen
// here), res the one to use if the lattice is made anew because it had to become coarser.
// The surface is laid out anew.  keep: the Stock is the same and is only cut again on another
// lattice, so the surface of the lattice before stays on the screen until this one has its own
// to show (MeshDirty hands it over in its place): the Stock does not disappear meanwhile.
void CSimCutStock::MakeGrid(const double lo[3], const double hi[3], double wanted, double res, int coarsened, bool keep)
{
	SimGrid *G = NULL;
	SimBefore *B = m_Before.get();
	SimFrozen *F = B ? B->frozen : NULL;

	if (lo)
	{
		double size[3];
		for (int a = 0; a < 3; a++) size[a] = hi[a] - lo[a];
		if (size[0] > 0 && size[1] > 0 && size[2] > 0)
		{
			bool automatic = !(wanted > 0.0);
			double finest = Finest(lo, hi, wanted), budget = Budget(automatic);

			// A Stock that was loaded has the lines of one lattice (those of its file, or kept when
			// the cuts of the file were made again).  Are they of this Stock?
			bool same = F != NULL;
			for (int a = 0; a < 3 && same; a++) same = F->lo[a] == lo[a] && F->hi[a] == hi[a];
			if (B && !coarsened)
			{
				// Then their lattice again, so that they serve as they are, unless a coarser one is
				// asked for now, or a finer one than could be had when they were made (the cuts of
				// the file are then made again).  Without lines to go by: not finer than the memory
				// allows, judged by what the Stock took when it was saved.
				if (same && finest <= F->res && (F->res > F->finest * 1.02 || finest >= F->finest * 0.98)) res = F->res;
				else if (B->bytes > budget && B->res > 0.0) res = B->res * sqrt(B->bytes / budget) * 1.1;
			}
			if (!(res > finest)) res = finest;

			try
			{
				G = new SimGrid;
				G->frozen = NULL;
				double gmin = 1e300;
				for (int a = 0; a < 3; a++)
				{
					double c = floor(size[a] / res + 0.5);
					if (c < 1) c = 1;
					G->n[a] = (int)c + 1;
					G->o[a] = (double)(float)lo[a];
					G->g[a] = size[a] / (G->n[a] - 1);
					if (G->g[a] < gmin) gmin = G->g[a];
					G->nc[a] = (G->n[a] + 1 + SIM_CS - 1) / SIM_CS;		// cells -1 .. n-1
					G->ns[a] = (G->n[a] + 1 + SIM_SUB - 1) / SIM_SUB;
					G->boxLo[a] = lo[a];
					G->boxHi[a] = hi[a];
				}
				G->wanted = automatic ? 0.0 : wanted;
				G->res = res;
				G->finest = finest;
				G->budget = budget;
				G->coarsened = coarsened;
				G->tol = gmin * SIM_TOL / SIM_ONE;
				G->tolZ = G->g[2] * SIM_TOL / SIM_ONE;
				G->infl = 2.0 * G->tol;
				G->rmin = 0.75 * Max2(G->g[0], G->g[1]);
				G->rminTilt = 0.75 * Max2(G->g[0], Max2(G->g[1], G->g[2]));
				G->guessed = 0;
				G->over = false;
				G->tilted = false;
				for (int a = 0; a < 3; a++)
				{
					int u = (a + 1) % 3, v = (a + 2) % 3;
					double ax[3] = { 0, 0, 0 };
					ax[a] = 1;
					G->S[a].Setup(G->n[u], G->n[v], G->n[a], G->o[a], G->g[a], DirCode(-ax[0], -ax[1], -ax[2]), DirCode(ax[0], ax[1], ax[2]));
					G->face[2 * a].assign((size_t)G->nc[u] * G->nc[v], 0);
					G->face[2 * a + 1].assign((size_t)G->nc[u] * G->nc[v], 0);
				}
				G->ResetBlocks();
				if (same && F->n[0] == G->n[0] && F->n[1] == G->n[1] && F->n[2] == G->n[2]) G->Attach(F);
			}
			catch (...)				// not enough memory
			{
				delete G;
				G = NULL;
			}
		}
	}

	// lines of another lattice than this are of no use any more: the cuts of the file are made again
	if (G && F && !G->frozen)
	{
		delete F;
		B->frozen = NULL;
	}

	if (keep && G)
		m_Stale = true;
	else
	{
		std::lock_guard<std::mutex> ml(m_MeshLock);
		ForgetSurface();
	}

	std::lock_guard<std::mutex> lk(m_Lock);
	m_Grid = G;
	m_ResInUse = G ? G->g[0] : 0.0;
	m_Applied = 0;
	m_BaseCount = G && B ? (int)B->poses.size() : 0;
	m_BaseApplied = G && G->frozen ? m_BaseCount : 0;
	m_LatticeCut = 0;
	if (!coarsened) m_CutSeconds = m_MeshSeconds = 0;
}

// The path changed where it had cut already: the Stock as it was before the path again (uncut,
// or as the cuts of its file left it), which is cut with the path as it is now.  What is on the
// screen stays there until that is done (see MakeGrid).
void CSimCutStock::Restart()
{
	m_Grid->Fill();
	m_Grid->tilted = false;
	m_Stale = true;
	m_LatticeCut = 0;
}

// Cut with a stretch of the path, which begins at the pose first.  If havePrev the first of the
// poses given is the one before the stretch.  Returns how many poses of the stretch are cut:
// fewer than all of them if the lines came to take more memory than there is for the Stock (the
// lattice is then made coarser and all of the path cut again) or if the stretch takes long (the
// rest of it is cut after the surface has been looked at), and all of them if the path or the
// Stock changed while it was cut (the cutting thread sees that for itself; stopped tells it).
// tools: the tools that the poses name.  before: the stretch is not of the path but of the cuts
// of the Stock file, which come before the path and do not change.
// slow: this lattice is too fine for what it is to be cut with, by so much (otherwise 0): the
// stretch is given up as far as it is, and the Stock is cut again on a lattice that is so much
// coarser.  That is when a tool stands tilted and the lattice is finer than tiltRes (given when
// no tool has stood tilted in this Stock so far and its resolution is chosen here), and when the
// cutting has fallen far behind (see slowBy below).
int CSimCutStock::CutBatch(const std::vector<SimPose> &poses, const std::vector<CSimCutTool> &tools, bool havePrev, int first, bool before, bool *stopped,
						   double tiltRes, double *slow)
{
	SimGrid &G = *m_Grid;
	*stopped = false;
	*slow = 0.0;
	double began = Seconds();
	std::vector<SimPrim> prims;
	double gmin = Min2(G.g[0], Min2(G.g[1], G.g[2]));
	double work = 0;
	int base = havePrev ? 1 : 0, count = (int)poses.size() - base;
	const bool *quit = &m_Quit;
	std::vector<std::vector<SimSlice> > slices(tools.size());		// of the tools that stand tilted (made when the first of them comes)
	std::vector<char> sliced(tools.size(), 0);
	std::vector<double> sideStep(tools.size(), 0.0);
	bool tilts = false;				// a tilted tool is among the cuts that wait to be made

	// A resolution that is chosen here, and a tool that stands tilted: is the cutting far behind
	// what is asked for, as when the path arrives much faster than this lattice is cut?  part:
	// how many poses of this stretch are cut by now.  Cutting with a tilted tool takes about
	// eight times as long on a lattice that is twice as fine: the answer is how much coarser a
	// lattice cuts all of the path that is known so far in a few seconds.  (m_Lock held)
	auto slowBy = [&](double part) -> double
	{
		if (!G.tilted || G.wanted != 0.0 || !G.CanCoarsen()) return 0.0;
		double spent = SIM_SPENT(m_LatticeCut + (Seconds() - began)), done = (double)(m_Applied + m_BaseApplied) + part;
		int target = EffTarget();
		double asked = (double)(target + m_BaseCount), known = (double)(MaxI(target, (int)m_Poses.size()) + m_BaseCount);
		if (!(spent > SIM_AUTO_AFTER) || !(done > 0.0) || !(spent / done * (asked - done) > SIM_AUTO_BEHIND)) return 0.0;
		double f = pow(spent / done * known / SIM_AUTO_SECONDS, 1.0 / 3.0);
		return f < 1.5 ? 1.5 : f > 5.0 ? 5.0 : f;
	};

	// The three sets of lines, each in bands, are independent of one another: every job cuts
	// one band with all the moves in their order.  A band is made of whole rows of chunks, so
	// that no two jobs work on the same chunk.
	auto cut = [&]()
	{
		if (prims.empty()) return;
		int threads = work > 200000.0 ? ThreadsToUse() : 1;
		int bands = threads > 1 ? threads * 2 : 1;
		int outer[3] = { G.n[2], G.n[0], G.n[1] };		// lines along X in bands of k, along Y of i, along Z of j
		Parallel(3 * bands, threads, [&](int job, int)
		{
			int s = job % 3, band = job / 3;			// (the three sets side by side: each has a heap of its own)
			int rows = (outer[s] + SIM_LC - 1) >> SIM_LC_BITS;
			int v0 = (int)((long long)rows * band / bands) << SIM_LC_BITS;
			int v1 = ((int)((long long)rows * (band + 1) / bands) << SIM_LC_BITS) - 1;
			if (v1 > outer[s] - 1) v1 = outer[s] - 1;
			if (v0 > v1) return;
			std::vector<double> buf;
			for (size_t i = 0; i < prims.size(); i++)
			{
				if (*quit || G.Over()) return;
				if (prims[i].kind == PRIM_TILT) CutTilt(G, s, prims[i], v0, v1, buf);
				else if (s == 2) CutZ(G, prims[i], v0, v1, buf);
				else CutXY(G, s, prims[i], v0, v1, buf);
			}
		});
		prims.clear();
		work = 0;
	};

	for (size_t i = base; i < poses.size(); i++)
	{
		const SimPose &p = poses[i];
		if (p.flags & SIMPOSE_OFF) continue;
		if (p.tool >= tools.size() || !tools[p.tool].Valid()) continue;

		SimPrim P;
		P.T = &tools[p.tool];
		P.flag = (p.flags & SIMPOSE_RAPID) ? SIMKIND_RAPID : SIMKIND_FEED;
		P.b[0] = p.x; P.b[1] = p.y; P.b[2] = p.z;
		P.d[0] = P.d[1] = 0.0;
		P.d[2] = 1.0;
		P.t0 = P.t1 = 0.0;
		P.slices = NULL;
		bool move = (p.flags & SIMPOSE_MOVE) && i > 0 && !(poses[i - 1].flags & SIMPOSE_OFF);
		const SimPose &q = move ? poses[i - 1] : p;
		P.a[0] = q.x; P.a[1] = q.y; P.a[2] = q.z;

		// A tool that is tilted at either end of the move (4 and 5 axis work) is cut as it stands
		// at positions along the move, turning evenly from the one direction to the other
		bool tilt = !Upright(p) || !Upright(q);
		double dA[3] = { q.ax, q.ay, q.az }, dB[3] = { p.ax, p.ay, p.az }, turn = 0.0;
		if (tilt)
		{
			double la = sqrt(dA[0] * dA[0] + dA[1] * dA[1] + dA[2] * dA[2]), lb = sqrt(dB[0] * dB[0] + dB[1] * dB[1] + dB[2] * dB[2]);
			if (!(la > 1e-6) || !(lb > 1e-6)) continue;		// (no direction: nothing to cut with)
			for (int a = 0; a < 3; a++)
			{
				dA[a] /= la;
				dB[a] /= lb;
			}
			double dot = dA[0] * dB[0] + dA[1] * dB[1] + dA[2] * dB[2];
			turn = acos(dot > 1.0 ? 1.0 : dot < -1.0 ? -1.0 : dot);
		}

		// The positions are floats.  Two that differ by no more than the last bits of a float, or
		// than a position along a line can tell, are the same: a planner that reports a level move
		// in small pieces may give them heights that differ like that, and they are exact sweeps
		// all the same.
		double same[3];
		for (int a = 0; a < 3; a++)
			same[a] = Max2(G.g[a] / SIM_ONE, 4.0 * FLT_EPSILON * Max2(fabs(P.a[a]), fabs(P.b[a])));
		double dx = P.b[0] - P.a[0], dy = P.b[1] - P.a[1], dz = P.b[2] - P.a[2];
		bool still = fabs(dx) <= same[0] && fabs(dy) <= same[1], level = fabs(dz) <= same[2];
		if (level) P.a[2] = P.b[2] = Min2(P.a[2], P.b[2]);
		if (still)
		{
			P.a[0] = P.b[0];
			P.a[1] = P.b[1];
		}
		// any move that is neither level nor straight up or down: the tool pressed in at steps of the lattice
		bool stamps = tilt || (!still && !level);
		P.kind = tilt ? PRIM_TILT : level ? (still ? PRIM_POINT : PRIM_FLAT) : still ? PRIM_PLUNGE : PRIM_POINT;

		// nothing of the whole move near the Stock: skip it at once
		SimPrim W = P;
		if (!tilt)
		{
			if (stamps) W.kind = PRIM_FLAT;
			if (!G.MakePrim(W)) continue;
		}

		int pieces = 1;
		if (tilt)
		{
			// The first time that a tool stands tilted in a Stock whose resolution is chosen here: on
			// to a lattice that such a tool cuts quickly (unless this one is nearly as coarse)
			if (tiltRes > G.res * 1.3 && G.CanCoarsen())
			{
				cut();
				*slow = tiltRes / G.res;
				return (int)(i - base);
			}
			G.tilted = true;
			if (!sliced[p.tool])
			{
				MakeSlices(tools[p.tool], G.rminTilt, G.infl, slices[p.tool]);
				sideStep[p.tool] = Max2(SideStep(tools[p.tool], G.rminTilt, gmin / 4.0), gmin);
				sliced[p.tool] = 1;
			}
			P.slices = &slices[p.tool];
			P.a[0] = q.x; P.a[1] = q.y; P.a[2] = q.z;		// (as they are: nothing of the above applies)
			P.b[0] = p.x; P.b[1] = p.y; P.b[2] = p.z;
			if (move)
			{
				// How many positions.  What the tool leaves standing between two of them is to be less
				// than the lattice shows.  A face of the tool across its axis (the end of an end mill)
				// leaves a step as high as the tool travels along its axis, or as its rim rises when the
				// tool turns: no more than three quarters of the distance between the lines of the
				// lattice from one position to the next.  The side of the tool leaves a ridge that is much
				// lower than the tool travels sideways (SideStep).  As the tool turns, a point of its side
				// travels the farther the higher up the tool it is: as far up as the tool is in the Stock
				// at the two ends of the move (and a little more, for the way between).  If it is not in
				// the Stock at both ends: no point that is in the Stock is farther from the tip than the
				// farthest corner of the Stock.
				double reach = 0.0;
				bool both = true;
				for (int e = 0; e < 2; e++)
				{
					SimPrim E = P;
					for (int a = 0; a < 3; a++)
					{
						E.b[a] = e ? P.b[a] : P.a[a];
						E.d[a] = e ? dB[a] : dA[a];
					}
					if (!G.MakePrim(E)) both = false;
					else if (E.t1 > reach) reach = E.t1;
				}
				reach *= 1.25;
				for (int e = 0; e < 2 && !both; e++)
				{
					double far2 = 0.0;
					for (int a = 0; a < 3; a++)
					{
						double c = e ? P.b[a] : P.a[a], lo = G.o[a] - P.T->Rmax, hi = G.o[a] + (G.n[a] - 1) * G.g[a] + P.T->Rmax;
						double w = Max2(fabs(c - lo), fabs(c - hi));
						far2 += w * w;
					}
					if (far2 > reach * reach) reach = sqrt(far2);
				}
				double side = sideStep[p.tool], face = 0.75 * gmin;
				double along = Max2(fabs(dx * dA[0] + dy * dA[1] + dz * dA[2]), fabs(dx * dB[0] + dy * dB[1] + dz * dB[2]));
				double steps = sqrt(dx * dx + dy * dy + dz * dz) / side + along / face + turn * (P.T->Rmax / face + reach / side);
				pieces = steps < 2000000.0 ? (int)ceil(steps) : 2000000;
			}
		}
		else if (stamps)
			pieces = (int)ceil(sqrt(dx * dx + dy * dy + dz * dz) / gmin);
		else if (P.kind == PRIM_FLAT)
		{
			// a long slanted move in pieces, so that each has few lines around it that it does not reach
			double shorter = Min2(fabs(dx), fabs(dy)), longer = Max2(fabs(dx), fabs(dy));
			if (shorter > 4.0 * W.R) pieces = (int)ceil(longer / (8.0 * W.R));
		}
		if (pieces < 1) pieces = 1;
		if (pieces > 2000000) pieces = 2000000;

		double A[3] = { P.a[0], P.a[1], P.a[2] };
		for (int m = 1; m <= pieces; m++)
		{
			double f0 = (double)(m - 1) / pieces, f1 = (double)m / pieces;
			SimPrim Q = P;
			for (int a = 0; a < 3; a++)
			{
				Q.b[a] = m == pieces ? P.b[a] : A[a] + f1 * (P.b[a] - A[a]);
				Q.a[a] = stamps ? Q.b[a] : m == 1 ? A[a] : A[a] + f0 * (P.b[a] - A[a]);
			}
			if (tilt)
			{
				// the direction of the tool at this position
				double sn = sin(turn), wa = 0.0, wb = 1.0;
				if (m < pieces && sn > 1e-9)
				{
					wa = sin((1.0 - f1) * turn) / sn;
					wb = sin(f1 * turn) / sn;
				}
				for (int a = 0; a < 3; a++) Q.d[a] = wa * dA[a] + wb * dB[a];
			}
			if (!G.MakePrim(Q)) continue;
			G.MarkPrim(Q);
			work += (tilt ? m_TiltWork : 1.0) * ((double)(Q.hi[0] - Q.lo[0] + 1) * (Q.hi[1] - Q.lo[1] + 1) +
											   (double)(Q.hi[0] - Q.lo[0] + Q.hi[1] - Q.lo[1] + 2) * (Q.hi[2] - Q.lo[2] + 1));
			if (tilt) tilts = true;
			prims.push_back(Q);
			if (m_Quit)
			{
				*stopped = true;
				return count;
			}

			// Now and then, also in the middle of a long move: is there still memory for the
			// lines, and are this still the path and the Stock to cut?  A line that a tilted tool
			// passes takes many times as long as one that an upright tool passes: it counts for as
			// much more as makes these looks come a few times a second.
			if (work > SIM_WORK_MAX)
			{
				double t0 = Seconds();
				cut();
				if (tilts)
				{
					double dt = Seconds() - t0;
					if (dt > SIM_WORK_TIME * 1.5 && m_TiltWork < 65536.0) m_TiltWork *= 2;
					else if (dt < SIM_WORK_TIME / 2 && m_TiltWork > 1.0) m_TiltWork /= 2;
					tilts = false;
				}
				if (G.Over() || (G.CanCoarsen() && G.LineBytes() > 0.1 * G.budget && LowMemory())) return (int)(i - base);
				std::lock_guard<std::mutex> lk(m_Lock);
				if (m_BoxPending || (!before && m_DirtyFrom < first + count))
				{
					*stopped = true;
					return count;
				}
				if ((*slow = slowBy((double)(i - base) + (double)(m - 1) / pieces)) > 0.0) return (int)(i - base);
				// (the move that is being cut is cut once more from its beginning, which changes nothing)
				if (i > (size_t)base && Seconds() - began > SIM_BATCH_LONG) return (int)(i - base);
			}
		}
	}
	cut();
	if (m_Quit) *stopped = true;
	else
	{
		std::lock_guard<std::mutex> lk(m_Lock);
		*slow = slowBy(count);
	}
	return count;
}

// Build the surface of the blocks that were cut, and the skin if it changed, and hand them over
void CSimCutStock::MeshDirty()
{
	SimGrid &G = *m_Grid;
	std::vector<int> list;
	for (size_t s = 0; s < G.dirty.size(); s++)
		if (G.dirty[s])
		{
			if (G.key[s] != SIM_SKIN) list.push_back((int)s);
			G.dirty[s] = 0;
		}
	bool skin = G.skinDirty;
	G.anyDirty = G.skinDirty = false;
	if (list.empty() && !skin && !m_Stale) return;

	int threads = list.size() > 16 ? ThreadsToUse() : 1;
	std::vector<SimScratch> scratch(threads);
	for (int t = 0; t < threads; t++) scratch[t].guessed = 0;
	std::vector<SimMesh> out(list.size());
	const bool *quit = &m_Quit;
	Parallel((int)list.size(), threads, [&](int job, int t)
	{
		if (*quit) return;
		unsigned long long k = G.key[list[job]];
		MeshBlock(G, (int)(k & 0x1FFFFF), (int)((k >> 21) & 0x1FFFFF), (int)(k >> 42), scratch[t], out[job]);
		Tight(out[job]);
	});
	for (int t = 0; t < threads; t++) G.guessed += scratch[t].guessed;

	std::vector<SimMesh> skinOut;
	if (skin)
	{
		BuildSkin(G, skinOut);
		for (size_t k = 0; k < skinOut.size(); k++) Tight(skinOut[k]);
		while (G.skinSlots.size() < skinOut.size())
		{
			G.skinSlots.push_back((int)G.key.size());
			G.key.push_back(SIM_SKIN);
			G.dirty.push_back(0);
			G.tris.push_back(0);
		}
	}

	std::lock_guard<std::mutex> ml(m_MeshLock);
	if (m_Stale) ForgetSurface();		// the surface of before goes in the moment that this one comes
	if (m_Meshes.size() < G.key.size()) m_Meshes.resize(G.key.size());
	for (size_t k = 0; k < list.size(); k++)
	{
		SimMesh &M = m_Meshes[list[k]];
		int nt = (int)(out[k].t.size() / 3);
		G.triangles += nt - G.tris[list[k]];
		G.tris[list[k]] = nt;
		M.v.swap(out[k].v);
		M.t.swap(out[k].t);
		M.changed = true;
	}
	if (skin)
		for (size_t k = 0; k < G.skinSlots.size(); k++)
		{
			int slot = G.skinSlots[k];
			SimMesh &M = m_Meshes[slot];
			SimMesh none;
			SimMesh &N = k < skinOut.size() ? skinOut[k] : none;
			int nt = (int)(N.t.size() / 3);
			G.triangles += nt - G.tris[slot];
			G.tris[slot] = nt;
			M.v.swap(N.v);
			M.t.swap(N.t);
			M.changed = true;
		}
}

// Write the Stock as it is now to a Stock file: the cuts of the file that it was loaded from and
// what the path has cut so far, and the lines as all of that left them.  (The cutting thread,
// which has made sure that the Stock is whole.)
const char *CSimCutStock::WriteStock(const SimChar *file)
{
	static const char cant[] = "The Stock file could not be written";
	SimGrid &G = *m_Grid;

	// the cuts of the file come first, then the path as far as it is cut
	std::vector<CSimCutTool> tools;
	std::vector<SimPose> poses;
	if (m_Before)
	{
		tools = m_Before->tools;
		poses = m_Before->poses;
	}
	size_t nb = poses.size();
	{
		std::lock_guard<std::mutex> lk(m_Lock);
		int n = MinI(m_Applied, (int)m_Poses.size());
		if (n > 0) poses.insert(poses.end(), m_Poses.begin(), m_Poses.begin() + n);
	}
	if (poses.size() > nb) poses[nb].flags &= ~SIMPOSE_MOVE;		// (the path does not go on from where the cuts of the file ended)

	// the tools of the path among those of the file: one that is the same is not there twice
	std::vector<int> where(m_Tools.size(), -1);
	for (size_t i = nb; i < poses.size(); i++)
	{
		unsigned t = poses[i].tool;
		if (t >= m_Tools.size() || !m_Tools[t].Valid())
		{
			poses[i].tool = 0xFFFF;			// (no tool: it cuts nothing)
			continue;
		}
		if (where[t] < 0)
		{
			const CSimCutTool &T = m_Tools[t];
			for (size_t k = 0; k < tools.size() && where[t] < 0; k++)
				if (tools[k].Open == T.Open && tools[k].h == T.h && tools[k].r == T.r) where[t] = (int)k;
			if (where[t] < 0)
			{
				if (tools.size() >= 65535) return "The Stock has been cut with too many different tools to be saved";
				where[t] = (int)tools.size();
				tools.push_back(T);
			}
		}
		poses[i].tool = (unsigned short)where[t];
	}

	// Written beside the file and put in its place when all of it is there: a file of that name
	// that is there already is not lost if this goes wrong
	SimString tmp = SimString(file) + SIMTEXT(".tmp");
	FILE *f = SimFopen(tmp.c_str(), "wb");
	if (!f) return cant;
	setvbuf(f, NULL, _IOFBF, 1 << 20);
	SimOut out(f);

	SimFileHead h;
	memset(&h, 0, sizeof(h));
	memcpy(h.magic, SIMFILE_MAGIC, 16);
	h.version = SIMFILE_VERSION;
	h.flags = SIMFILE_LINES;
	for (int a = 0; a < 3; a++)
	{
		h.lo[a] = G.boxLo[a];
		h.hi[a] = G.boxHi[a];
		h.n[a] = G.n[a];
	}
	h.wanted = G.wanted;
	h.res = G.res;
	h.bytes = G.Bytes();
	h.tools = (unsigned)tools.size();
	h.poses = (unsigned)poses.size();
	out.Val(h);

	for (size_t t = 0; t < tools.size(); t++)
	{
		unsigned points = (unsigned)tools[t].h.size(), open = tools[t].Open ? 1 : 0;
		out.Val(points);
		out.Val(open);
		if (points)
		{
			out.Put(&tools[t].h[0], (size_t)points * 8);
			out.Put(&tools[t].r[0], (size_t)points * 8);
		}
	}
	if (!poses.empty()) out.Put(&poses[0], poses.size() * sizeof(SimPose));

	std::vector<unsigned char> buf;
	for (int a = 0; a < 3 && !out.bad; a++)
	{
		const SimLines &L = G.S[a];
		unsigned chunks = 0;
		EachChunk(L, [&](int, int, const SimChunk *) { chunks++; });
		out.Val(chunks);
		EachChunk(L, [&](int cu, int cv, const SimChunk *ch)
		{
			if (out.bad) return;
			PackChunk(L, cu, cv, ch, buf);
			unsigned head[3] = { (unsigned)cu, (unsigned)cv, (unsigned)buf.size() };
			out.Put(head, sizeof(head));
			if (!buf.empty()) out.Put(&buf[0], buf.size());
		});
	}

	std::vector<unsigned long long> keys;
	for (size_t s = 0; s < G.key.size(); s++)
		if (G.key[s] != SIM_SKIN) keys.push_back(G.key[s]);
	unsigned cnt = (unsigned)keys.size();
	out.Val(cnt);
	if (cnt) out.Put(&keys[0], (size_t)cnt * 8);
	for (int fc = 0; fc < 6; fc++)
	{
		cnt = (unsigned)G.fine[fc].size();
		out.Val(cnt);
		for (std::unordered_map<unsigned long long, unsigned long long>::const_iterator it = G.fine[fc].begin(); it != G.fine[fc].end(); ++it)
		{
			unsigned long long kv[2] = { it->first, it->second };
			out.Put(kv, sizeof(kv));
		}
	}

	unsigned long long sum = out.sum.End();
	bool ok = !out.bad && fwrite(&sum, 1, 8, f) == 8 && fflush(f) == 0;
	if (fclose(f)) ok = false;
	if (ok && !SimRename(tmp.c_str(), file)) ok = false;
	if (!ok)
	{
		SimRemove(tmp.c_str());
		return cant;
	}
	return NULL;
}

// The Stock as it is cut now into a Stock file.  Waits until the cutting thread has written it.
// The cutting thread writes it when the Stock is whole (see Run).  That is at once, also while a
// job runs, except while the Stock is being cut again (on another lattice, or with the cuts of a
// Stock file): then it is not waited for longer than wait seconds, and nothing is saved.
const char *CSimCutStock::Save(const SimChar *file, int wait)
{
	std::unique_lock<std::mutex> lk(m_Lock);
	if (!m_Started) return "There is no Stock to save";
	m_SaveFile = file;
	m_SaveError = "The Stock could not be saved";
	m_SaveBusy = false;
	m_SaveWanted = true;
	m_Wake.notify_all();
	if (!m_IdleCv.wait_for(lk, std::chrono::seconds(wait), [this]() { return !m_SaveWanted || m_SaveBusy || m_Quit; }))
	{
		m_SaveWanted = false;
		return "The Stock is being cut again at the moment. Save it when that is done.";
	}
	m_IdleCv.wait(lk, [this]() { return !m_SaveWanted || m_Quit; });
	return m_SaveError;
}

// The Stock of a Stock file in place of the one that there is: its size, and its cuts before
// anything that the path cuts.  The file is read here; the cutting thread does the rest.
const char *CSimCutStock::Load(const SimChar *file, double lo[3], double hi[3], double *resolution)
{
	std::shared_ptr<SimBefore> before;
	double flo[3], fhi[3], wanted = 0;
	const char *err = ReadStock(file, before, flo, fhi, &wanted);
	if (err) return err;

	std::lock_guard<std::mutex> lk(m_Lock);
	m_BeforeWanted = before;
	for (int a = 0; a < 3; a++)
	{
		m_BoxLo[a] = lo[a] = flo[a];
		m_BoxHi[a] = hi[a] = fhi[a];
	}
	m_BoxRes = *resolution = wanted;
	m_BoxWanted = m_BoxPending = true;
	m_Changes++;
	Start();
	m_Wake.notify_all();
	return NULL;
}

// The cuts of the Stock file are no more.  The Stock that there is, if there is one, is made
// anew: uncut, and then cut with the path as far as it was.
void CSimCutStock::Unload()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	if (!m_BeforeWanted) return;
	m_BeforeWanted.reset();
	if (!m_Started) return;
	m_BoxPending = true;
	m_Changes++;
	m_Wake.notify_all();
}

bool CSimCutStock::Loaded()
{
	std::lock_guard<std::mutex> lk(m_Lock);
	return m_BeforeWanted != nullptr;
}

// After Define or Load: waits until the cutting thread has made the lattice of that Stock (true),
// so that ResolutionInUse tells about it, but not longer than this.  Not until anything is cut.
bool CSimCutStock::WaitLattice(int milliseconds)
{
	std::unique_lock<std::mutex> lk(m_Lock);
	if (!m_Started) return true;
	return m_IdleCv.wait_for(lk, std::chrono::milliseconds(milliseconds), [this]() { return (!m_BoxPending && !m_BoxMaking) || m_Quit; });
}

// The cutting thread
void CSimCutStock::Run()
{
	SimBelowNormal();

	double lastMesh = 0;				// when the surface was last built
	double quietFrom = 0;				// since when all of the path is cut and nothing about it has changed ...
	unsigned quietChanges = 0;			// ... which is told by this
	bool quiet = false;
	bool showNow = false;				// nothing of the Stock is on the screen: it is handed over as it is before it is cut any further

	std::unique_lock<std::mutex> lk(m_Lock);
	for (;;)
	{
		if (m_Quit) break;

		if (m_StatsWanted)
		{
			FillStats(&m_Stats);
			m_StatsWanted = false;
			m_IdleCv.notify_all();
		}

		if (m_SaveWanted && !m_BoxPending && !m_Grid)		// a Stock file is asked for, and there is no Stock
		{
			m_SaveError = "There is no Stock to save";
			m_SaveWanted = false;
			m_IdleCv.notify_all();
		}

		if (!Pending())
		{
			if (!m_Grid)
			{
				m_DirtyFrom = INT_MAX;
				m_Awaited = false;
			}
			m_Idle = true;
			m_IdleCv.notify_all();
			m_Wake.wait(lk);
			continue;
		}
		m_Idle = false;

		if (m_BoxPending)
		{
			double lo[3], hi[3], res = m_BoxRes;
			bool want = m_BoxWanted;
			std::shared_ptr<SimBefore> before = m_BeforeWanted;
			for (int a = 0; a < 3; a++) { lo[a] = m_BoxLo[a]; hi[a] = m_BoxHi[a]; }
			m_BoxPending = false;
			m_BoxMaking = true;
			if (want) m_Failed = false;
			SimGrid *old = m_Grid;
			m_Grid = NULL;
			m_DirtyFrom = INT_MAX;
			m_Republish = false;
			// Another Stock shows itself at once, as it is before the path.  One with the cuts of a
			// Stock file may take a while to build its surface: what is on the screen stays until then.
			bool keep = want && old != NULL && before != nullptr;
			lk.unlock();
			delete old;
			m_Before = before;			// (the cuts of another Stock file, or none, go with what they left on the lattice)
			before.reset();
			MakeGrid(want ? lo : NULL, hi, res, 0.0, 0, keep);
			lk.lock();
			if (want && !m_Grid) m_Failed = true;
			m_RefineLeft = true;
			m_TiltStart = true;
			m_Shown = 0;
			showNow = m_Grid != NULL;
			if (!m_Grid) m_Awaited = false;
			m_BoxMaking = false;
			m_IdleCv.notify_all();
			continue;
		}

		if (m_NewPath)				// another path begins: for it the lattice may be made finer once more
		{
			m_NewPath = false;
			m_RefineLeft = true;
			m_Shown = 0;			// (and what is on the screen is of the path before: nothing to wait for)
		}

		if (m_DirtyFrom < m_Applied)		// the path changed where it had cut already: start again
		{
			m_DirtyFrom = INT_MAX;
			lk.unlock();
			Restart();
			lk.lock();
			m_Applied = 0;
			m_BaseApplied = m_Grid->frozen ? m_BaseCount : 0;
			if (m_Shown == 0) showNow = true;	// another path: the Stock is shown as it is before the path at once
			continue;
		}
		m_DirtyFrom = INT_MAX;

		if (m_Republish)			// the object that draws the Stock has lost what it had
		{
			m_Republish = false;
			m_Grid->MarkAll();
			m_Shown = 0;
			showNow = true;
		}

		// A Stock file is asked for.  It is written when the Stock on the lattice is whole: the
		// cuts of the file that it was loaded from are all made, and, while it is being cut again
		// with its surface of before still on the screen, as much of the path is cut as that
		// surface shows.  What the path has cut by then is what the file gets.
		if (m_SaveWanted && m_BaseApplied >= m_BaseCount && !(m_Stale && m_Applied < MinI(m_Shown, EffTarget())))
		{
			SimString file = m_SaveFile;
			const char *err;
			m_SaveBusy = true;			// (whoever waits for it waits to the end now)
			m_IdleCv.notify_all();
			lk.unlock();
			try
			{
				err = WriteStock(file.c_str());
			}
			catch (...)
			{
				err = "There is not enough memory to save the Stock";
			}
			lk.lock();
			m_SaveError = err;
			m_SaveBusy = false;
			m_SaveWanted = false;
			m_IdleCv.notify_all();
		}

		// Running out of memory must never take the application down with it: the lattice is
		// made coarser, and if that is not possible the Stock is given up
		bool failed = false;

		// Nothing of the Stock is on the screen (it was just made, another path begins, or what was
		// drawn of it is lost): it goes there as it is, before it is cut any further
		if (showNow)
		{
			showNow = false;
			lk.unlock();
			try
			{
				double t0 = Seconds();
				MeshDirty();
				lastMesh = Seconds();
				m_MeshSeconds += lastMesh - t0;
			}
			catch (...)
			{
				failed = true;
			}
			lk.lock();
			if (!failed) m_Shown = m_Applied;
			m_Awaited = false;
			m_IdleCv.notify_all();
		}

		// The cuts of the Stock file that the Stock was loaded from come first, unless the lines
		// that they left are there for this lattice.  Then the path.
		int target = EffTarget();
		bool before = m_BaseApplied < m_BaseCount;
		double slow = 0.0;			// (the lattice is too fine for what it is to be cut with: see CutBatch)
		if (!failed && (before || m_Applied < target))
		{
			int first = before ? m_BaseApplied : m_Applied;
			int count = MinI((before ? m_BaseCount : target) - first, m_Batch), applied = count;
			bool stopped = false;
			// A resolution that is chosen here, and no tool has stood tilted in this Stock so far: the
			// lattice that a tilted tool begins with.  (Not for a Stock that was loaded: it keeps the
			// lattice that it has.)
			double tiltRes = 0.0;
			if (m_TiltStart && m_BaseCount == 0 && m_Grid->wanted == 0.0)
			{
				for (int a = 0; a < 3; a++) tiltRes = Max2(tiltRes, m_Grid->boxHi[a] - m_Grid->boxLo[a]);
				tiltRes /= SIM_TILT_CELLS;
			}
			lk.unlock();
			try
			{
				std::vector<SimPose> batch;
				std::vector<ToolDef> defs;
				if (before)
					batch.assign(m_Before->poses.begin() + (first > 0 ? first - 1 : 0), m_Before->poses.begin() + first + count);
				else
				{
					std::lock_guard<std::mutex> lk2(m_Lock);
					if (first + count <= (int)m_Poses.size())		// (the path may just have been cleared)
						batch.assign(m_Poses.begin() + (first > 0 ? first - 1 : 0), m_Poses.begin() + first + count);
					if (m_ToolDefs.size() > m_Tools.size()) defs.assign(m_ToolDefs.begin() + m_Tools.size(), m_ToolDefs.end());
				}

				for (size_t i = 0; i < defs.size(); i++)	// tools met for the first time
				{
					CSimCutTool T;
					if (defs[i].given) T = defs[i].profile;
					else if (defs[i].image.empty() || !T.SetFromVrml(defs[i].image.c_str(), defs[i].scale, defs[i].offset))
						T.SetFlat(defs[i].diameter);
					m_Tools.push_back(T);
				}

				double t0 = Seconds();
				if (!batch.empty()) applied = CutBatch(batch, before ? m_Before->tools : m_Tools, first > 0, first, before, &stopped, tiltRes, &slow);
				double dt = Seconds() - t0;
				m_CutSeconds += dt;
				m_LatticeCut += dt;
				if (slow > 0.0) ;										// (given up for another lattice)
				else if (applied < count) m_Batch = MaxI(applied, 1);	// (given up for now: that many take long enough)
				else if (dt < SIM_BATCH_TIME / 2 && m_Batch < 1000000) m_Batch *= 2;
				else if (dt > SIM_BATCH_TIME * 2 && m_Batch > 1) m_Batch /= 2;

				// The cuts of the file are all made: the lines as they are now are what the path is
				// cut into, however often it is cut anew
				if (before && !stopped && first + applied >= m_BaseCount && !m_Grid->Over() && !m_Before->frozen)
					m_Before->frozen = m_Grid->Freeze();
			}
			catch (...)
			{
				failed = true;
			}
			lk.lock();
			if (!before) m_Applied = first + applied;
			else if (!stopped) m_BaseApplied = first + applied;
			if (m_Grid->tilted) m_TiltStart = false;
		}

		if (m_Quit) break;

		// The surface of before that is still on the screen (the Stock is being cut again) is kept
		// for the eye only: when memory runs short it goes at once
		if (m_Stale && (failed || LowMemory()))
		{
			lk.unlock();
			{
				std::lock_guard<std::mutex> ml(m_MeshLock);
				ForgetSurface();
			}
			lk.lock();
		}

		// The surface is built when all that was asked for is cut, and while the cutting is
		// catching up (with a path that arrives faster than it is cut, or with the cuts of a Stock
		// file) a few times a second.  Not at all if the lines alone take more memory than there
		// is for the Stock.  And while the Stock is being cut again with the surface of before
		// still on the screen, not until as much is cut as that surface shows: what is on the
		// screen never goes back.
		bool over = m_Grid->CanCoarsen() && m_Grid->LineBytes() > m_Grid->budget;
		bool behind = m_BaseApplied < m_BaseCount;
		bool hold = m_Stale && (behind || m_Applied < MinI(m_Shown, target));
		if (!failed && !over && !hold && slow == 0.0 && ((!behind && m_Applied >= target) || Seconds() - lastMesh > 0.25))
		{
			lk.unlock();
			try
			{
				double t0 = Seconds();
				MeshDirty();
				lastMesh = Seconds();
				m_MeshSeconds += lastMesh - t0;
			}
			catch (...)
			{
				failed = true;
			}
			lk.lock();
			if (!failed) m_Shown = m_Applied;
		}

		// What is cut takes more memory than there is for it (or the program as a whole is about
		// to run out, and the Stock has a share in that): a coarser lattice, and the path once
		// more from its start.  When part of the path is still to be cut the lattice is made as
		// much coarser as all of it is expected to need.
		double factor = 0.0;
		if (m_Grid && m_Grid->CanCoarsen())
		{
			if (failed) factor = 2.0;
			else
			{
				double bytes = m_Grid->Bytes();
				if (bytes > m_Grid->budget || (bytes > 0.1 * m_Grid->budget && LowMemory()))
				{
					// (of all the path that is known, also where only a part of it is to be cut so
					// far, and of the cuts of the Stock file)
					int all = MaxI(target, (int)m_Poses.size()) + m_BaseCount;
					double done = all > 0 ? (double)(m_Applied + m_BaseApplied) / all : 1.0;
					if (done < 0.05) done = 0.05;
					factor = sqrt(bytes / done / m_Grid->budget) * 1.1;
					if (factor < 1.5) factor = 1.5;
					if (factor > 8.0) factor = 8.0;
				}
				else if (slow > 0.0)
				{
					factor = slow;			// a coarser lattice for the time that cutting takes
					m_TiltStart = false;
				}
			}
		}

		// The other way round, at most once for a path: the lattice is coarser than was asked
		// for (made so for this path, maybe by too much while most of it was still to be cut, or
		// for the path before it), all of the path is cut and nothing about it has changed for a
		// while.  If it takes much less than there is: a finer lattice that uses most of it.
		// Never while the path is still arriving or the tool is on its way along it: the Stock
		// would be cut again and again.
		if (factor == 0.0 && !failed && RefineDue() && m_Applied >= EffTarget() && m_DirtyFrom >= m_Applied &&
			!m_Grid->anyDirty && !m_BoxPending && !m_NewPath && !m_Quit)
		{
			if (!quiet || quietChanges != m_Changes)
			{
				quiet = true;
				quietChanges = m_Changes;
				quietFrom = Seconds();
			}
			double left = SIM_REFINE_WAIT / 1000.0 - (Seconds() - quietFrom);
			if (left > 0.0)
			{
				// (whatever ends this wait, the way leads back here)
				m_Waiting = true;
				m_Wake.wait_for(lk, std::chrono::duration<double>(left));
				m_Waiting = false;
				if (m_Quit) break;
			}
			else
			{
				quiet = false;
				m_RefineLeft = false;
				double use = m_Grid->Bytes();
				if (use > 0.0 && use < 0.45 * m_Grid->budget && !LowMemory())
				{
					factor = sqrt(use / (0.8 * m_Grid->budget));
					if (factor < 0.1) factor = 0.1;
					// A resolution that is chosen here, and a tool that stood tilted: only as fine as
					// cutting the path again does not take long, and not at all for a little
					if (m_Grid->tilted && m_Grid->wanted == 0.0)
					{
						double least = pow(SIM_SPENT(m_LatticeCut) / SIM_AUTO_REFINE, 1.0 / 3.0);
						if (factor < least) factor = least;
						if (factor > 0.7) factor = 0.0;
					}
				}
			}
		}
		else
			quiet = false;

		if (factor > 0.0 && !m_BoxPending)
		{
			double lo[3], hi[3], wanted = m_Grid->wanted, res = m_Grid->res * factor;
			int times = m_Grid->coarsened + 1;
			for (int a = 0; a < 3; a++) { lo[a] = m_Grid->boxLo[a]; hi[a] = m_Grid->boxHi[a]; }
			SimGrid *old = m_Grid;
			m_Grid = NULL;
			lk.unlock();
			delete old;
			// (the surface of the lattice before stays on the screen meanwhile, unless memory is what is short)
			MakeGrid(lo, hi, wanted, res, times, !failed && !LowMemory());
			lk.lock();
			failed = !m_Grid;
		}

		if (failed)
		{
			m_Failed = true;
			if (!m_BoxPending)			// (unless a new Stock was asked for meanwhile)
			{
				m_BoxWanted = false;
				m_BoxPending = true;
			}
		}
	}
	m_Idle = true;
	m_IdleCv.notify_all();
}
