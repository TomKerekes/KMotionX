// SimCut.h : header file
/*********************************************************************/
/*         Copyright (c) 2003-2026  DynoMotion Incorporated          */
/*  MIT License: SimCut-LICENSE.txt (SPDX-License-Identifier: MIT)   */
/*********************************************************************/
//
// Material removal for the G Code Viewer.
//
// The Stock is a block that holds, on a lattice of lines along X, Y and Z,
// the pieces of each line that are inside material.  Each end of a piece is
// a point of the Stock surface.  A tool that passes through a line removes
// the part of the line that is inside the tool: the surface point slides
// along its line to the surface of the tool, a piece cut in the middle
// becomes two pieces and a piece that is swallowed disappears.  Through
// holes, undercuts and parts that separate need nothing special.
//
// The tool is a shape of revolution (radius as a function of the height
// above the tool tip).  Each pose of the tool path tells, besides where the
// tool tip is, which way the tool points (from its tip toward the spindle).
// A tool that stands upright, along +Z, is cut with exactly where it moves
// at one height or straight up and down.  In any other move it is cut as
// it stands at positions along the move, no farther apart than the lines of
// the lattice.  A tool that is tilted (4 and 5 axis work) is cut as it
// stands at positions along every move, turning evenly from the direction
// of the pose before to the one of the pose: so close to one another that
// what it leaves standing between two of them is less than the lattice
// shows (as close as the lines of the lattice where the tool moves along
// its axis or turns, farther apart where its round side leads).
//
// The lines are only kept where something was cut (every other line is
// uncut), so the memory goes with what is cut and not with the size of the
// Stock: a fine lattice is possible in a large Stock.  If what is cut needs
// more memory than is set aside the Stock is cut again on a coarser lattice,
// which then also serves the next path (made finer again, once, if that path
// turns out to need much less).  While the Stock is cut again the surface
// that was drawn before stays on the screen.  A tool that stands tilted takes
// much longer to cut with than one that stands upright.  When the resolution
// is left to the Stock (0) the lattice becomes coarser the first time that a
// tool stands tilted: the Stock then keeps up with the path as it arrives.
// When all of the path is cut the lattice is made finer, once, as far as
// the memory allows and cutting it all again does not take long.
// Positions along a line are whole numbers (a fixed fraction of the distance
// between two lines), exact wherever the Stock lies.
//
// Each surface point also knows the direction of the surface there, from the
// shape of the tool that cut it.  The surface that is drawn has one point in
// every cell of the lattice that the surface passes through, put where the
// planes through the surface points of the cell meet: flat faces come out
// flat and edges sharp at any resolution, which then only limits the detail
// of small shapes.  The surface is built in detail only near the cuts; the
// rest of the faces of the Stock is a few rectangles.
//
// The tool path is handed over as a list of poses (tool tip positions).  A
// thread of its own cuts the Stock and builds the surface to draw, so the
// G Code Interpreter and the screen never wait for it.
//
// A Stock can be saved in a file and loaded again.  The file holds the size
// of the Stock, the cuts that were made in it (the tools and the tool path, in
// the order that they cut) and the lines of the lattice as those cuts left
// them.  A Stock that was loaded has the cuts of its file in it before any
// path cuts: the path is cut into what the file left, and when the path goes
// the Stock is as the file had it.  On the lattice that the file was saved
// with nothing is cut again, the lines are taken as they are; on any other
// lattice (another resolution is asked for, or the memory asks for a coarser
// one) the cuts of the file are made again.  The lines of the file are there
// once: the path is cut into copies of the chunks of them that it reaches.
//
// This file and SimCut.cpp use nothing of MFC or OpenGL.  What they need of the operating
// system (memory, files, threads) is in SimCutPort.h, for Windows and for POSIX (Linux); file
// names are SimChar strings (below), wchar_t on Windows and UTF-8 char elsewhere.

#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

// File names, as the operating system takes them: UTF-16 wchar_t on Windows, UTF-8 char elsewhere
#ifdef _WIN32
typedef wchar_t SimChar;
#define SIMTEXT(s) L##s
#else
typedef char SimChar;
#define SIMTEXT(s) s
#endif
typedef std::basic_string<SimChar> SimString;

#define SIMCUT_BIG 1e30				// "without end" for a height


// Cutter: radius as a function of the height above the tool tip
class CSimCutTool
{
public:
	std::vector<double> h, r;		// points of the profile, heights ascending (two points at one height make a step)
	double Rmax;					// largest radius
	bool Open;						// the last radius continues upward without end (shank, holder)

	CSimCutTool() { Clear(); }
	void Clear();
	void Add(double height, double radius);
	bool Finish();					// false if there is nothing to cut with
	bool SetFlat(double diameter);	// flat end mill
	bool SetFromVrml(const SimChar *file, double scale, const double offset[3]);	// spin a tool image about Z

	// A basic tool from its sizes (the Create Tool dialog), tip at height 0, closed at the length:
	//   SIMTOOL_ENDMILL   Diameter, Length
	//   SIMTOOL_BALL      Diameter, Length
	//   SIMTOOL_V         Diameter, Angle of the point (degrees, side to side), Tip Diameter, Length
	//   SIMTOOL_TSLOT     Diameter, Height of the cutter, Neck Diameter, Length
	//   SIMTOOL_DOVETAIL  Diameter, Angle between side and bottom (degrees), Height of the cutter, Neck Diameter, Length
	// Returns NULL, or what is wrong with the sizes.
	const char *SetBasic(int type, const double *p);
	static int BasicParams(int type);	// how many sizes the type has

	// Write the tool as a tool image (VRML) that the viewer displays and SetFromVrml reads back
	bool WriteVrml(const SimChar *file, int sides, const char *comment) const;

	bool Valid() const { return h.size() >= 2 && Rmax > 0.0; }
	double Bottom() const { return h.front(); }
	double Top() const { return Open ? SIMCUT_BIG : h.back(); }

	// The tool is the open set inside the profile, so what touches its surface is not cut
	double RadiusAt(double height) const;			// 0 outside the tool
	double RadiusBelow(double height) const;		// the radius just below a height
	double MaxRadius(double h0, double h1) const;	// largest radius between two heights
	int Above(double d, double *out, double *slope = 0) const;	// stretches of height where the radius is greater than d
	double SlopeBelow(double height) const;			// how fast the radius grows with the height just below a height
													// (out needs room for 2 * h.size() values), returns how many
};


enum { SIMTOOL_ENDMILL, SIMTOOL_BALL, SIMTOOL_V, SIMTOOL_TSLOT, SIMTOOL_DOVETAIL, SIMTOOL_TYPES };


// One position of the tool tip along the path
struct SimPose
{
	float x, y, z;					// in the coordinates of the viewer
	unsigned short tool;			// from AddTool
	unsigned char flags;			// SIMPOSE_xxx
	unsigned char spare;
	float ax, ay, az;				// which way the tool points, from its tip toward the spindle (a unit vector; 0 0 1: upright)
};

#define SIMPOSE_MOVE	1			// the tool travelled here in a straight line from the pose before
									// (without it the tool only appears here)
#define SIMPOSE_RAPID	2			// ... at the rapid rate
#define SIMPOSE_OFF		4			// the tool has not been here: nothing is cut with the pose, and no move begins at it

// What made a part of the surface
#define SIMKIND_STOCK	0			// face of the Stock as it was
#define SIMKIND_FEED	1			// cut by a feed move
#define SIMKIND_RAPID	2			// cut by a rapid move


// Surface to draw
struct SimVertex
{
	float x, y, z;
	float kind;						// SIMKIND_xxx (a value between two of them where they meet)
	unsigned dir;					// which way the surface looks there, for the shading: x, y and z as signed 10 bit
									// numbers (511 for 1), x in the lowest bits.  0: the point is on an edge or a
									// corner, or on a face of the Stock, and the triangles show their own faces
};

struct SimMesh						// surface inside one block of the lattice
{
	std::vector<SimVertex> v;
	std::vector<unsigned short> t;	// triangles, three indices each
	bool changed;					// set when v and t are new, cleared by who draws them
	SimMesh() : changed(false) {}
};

struct SimStats
{
	int nodes[3];					// lattice nodes along X, Y and Z
	double origin[3];				// the first of them
	double step[3];					// distance between them
	double lines;					// number of lines of the lattice
	double stored;					// ... of which exist (the others are uncut and take no memory)
	double pieces;					// pieces of material on them
	double multi[3];				// lines along X, Y and Z that hold more than one piece
	double volume[3];				// volume of the material as the lines along X, Y and Z see it
	double triangles;				// triangles of the surface as last built
	double guessed;					// surface points that had to be guessed (the three sets of lines disagreed)
	double cutSeconds, meshSeconds;	// time spent since the Stock was defined
	int applied;					// poses cut so far
	double bytes;					// memory that the lines and the surface take
	double wanted;					// resolution that was asked for (0: chosen from what is cut); step has what is in use
	int coarsened;					// times the lattice was made coarser because of the memory
	int before;						// cuts of the Stock file that the Stock was loaded from (0: it was not loaded)
	int beforeApplied;				// ... that are in the Stock
};

struct SimGrid;
struct SimBefore;


class CSimCutStock
{
public:
	CSimCutStock();
	~CSimCutStock();

	// --- from any thread -------------------------------------------------

	// A new uncut Stock.  resolution is the distance between the lines of the lattice (0 = as fine as
	// the memory for it allows).  The lines take memory only where something is cut, so a fine
	// lattice is possible in a large Stock; if what is cut needs more than there is the lattice is
	// made coarser (GetStats tells the one in use).  The poses are kept and cut again as far as they were.
	void Define(const double lo[3], const double hi[3], double resolution);
	void Remove();						// no Stock (the poses are still collected)
	bool Defined();

	void ClearPoses();					// forget the path (nothing is cut)
	int AddTool(const SimChar *image, double scale, const double offset[3], double diameter);
										// image: tool image file ("" = none), spun about Z after
										// scaling and offsetting it; diameter: flat end mill used
										// when there is no image or it can not be read
	int AddTool(const CSimCutTool &profile);	// a tool given by its profile
	void ForgetToolImages();			// tool image files may have changed: tools added from now on are read again
	void AddPose(float x, float y, float z, int tool, int flags, const double *axis = 0);
										// axis: which way the tool points, from its tip toward the spindle
										// (any length; none: upright, along +Z)
	void SetPoseCount(int count);		// the path lost its end
	void MovePose(int index, float x, float y, float z);	// (a move that ends sooner: the tool has turned as much less)
	void DisablePoses(int first, int count);	// the tool does not go there after all: these poses do not cut
	int NbPoses();
	bool GetPose(int index, SimPose *p);

	void CutAll();						// cut with all the poses, also those added later
	void CutTo(int count);				// cut with (at least) the first count poses
	void UncutFrom(int count);			// the poses from count on have not cut

	bool WaitIdle(int milliseconds);	// true when everything asked for is cut and handed over
	bool Busy();
	bool Failed();						// the Stock was given up (not enough memory)
	void GetStats(SimStats *s);			// (waits for the cutting thread to pause)
	double ResolutionInUse();			// distance between the lines of the lattice as it is now, 0 without a Stock (does not wait)

	// A Stock file (see above).  Save and Load return NULL, or what went wrong (a text for the user).
	const char *Save(const SimChar *file, int wait = 6);
											// the Stock as it is cut now: the cuts of the file that it was loaded from
											// and what the path has cut so far.  Waits for the cutting thread, but for
											// a Stock that is being cut again not longer than wait seconds.
	const char *Load(const SimChar *file, double lo[3], double hi[3], double *resolution);
											// the Stock of the file in place of this one; tells its size and the
											// resolution that was asked for when it was saved.  The poses are kept
											// and cut again as far as they were, into what the file left.
	void Unload();						// the cuts of the file are no more: the Stock is uncut, but for what the path cuts
	bool Loaded();						// the Stock has the cuts of a file
	bool WaitLattice(int milliseconds);	// after Define or Load: waits until the lattice of that Stock is made (true), not longer than this

	// --- for the object that draws the Stock -----------------------------

	void LockMeshes() { m_MeshLock.lock(); }
	void UnlockMeshes() { m_MeshLock.unlock(); }
	int Generation() const { return m_Generation; }		// changes when the blocks are laid out anew
	int NbMeshes() const { return (int)m_Meshes.size(); }
	SimMesh &Mesh(int i) { return m_Meshes[i]; }
	void Republish();					// the drawing buffers were lost: hand all the surface over again
	bool WaitSurface(int milliseconds);	// ... and wait until that is done (true), but not longer than this

	static int ThreadsToUse();

private:
	struct ToolDef
	{
		SimString image;
		double scale, offset[3], diameter;
		bool given;						// profile holds the shape
		bool old;						// its image file may have changed since: not the tool of a new pose
		CSimCutTool profile;
	};

	// shared, under m_Lock
	std::mutex m_Lock;
	std::condition_variable m_Wake, m_IdleCv;
	std::vector<SimPose> m_Poses;
	std::vector<ToolDef> m_ToolDefs;
	int m_Target;						// poses to cut with
	bool m_All;							// ... all of them
	int m_DirtyFrom;					// poses from here on changed since they were cut
	bool m_BoxPending, m_BoxWanted;
	double m_BoxLo[3], m_BoxHi[3], m_BoxRes;
	bool m_Republish;
	bool m_Awaited;						// all the surface was asked for again and is not handed over yet
	bool m_StatsWanted;
	SimStats m_Stats;
	bool m_Quit, m_Idle, m_Started, m_Failed;
	bool m_NewPath;						// the path was cleared: what follows is another one
	unsigned m_Changes;					// counts the changes of the path and of how much of it is to be cut
	bool m_Waiting;						// the cutting thread waits for the path to stay as it is for a while
	bool m_RefineLeft;					// the lattice may still be made finer for this path (once)
	double m_ResInUse;
	bool m_BoxMaking;					// the cutting thread is making the lattice of the Stock that was asked for
	std::shared_ptr<SimBefore> m_BeforeWanted;	// what the Stock is to have been cut with before the path (a Stock file)
	int m_BaseCount, m_BaseApplied;		// cuts of that kind that the Stock on the lattice has to have, and has
	bool m_SaveWanted, m_SaveBusy;		// a Stock file is to be written, is being written ...
	SimString m_SaveFile;
	const char *m_SaveError;			// ... and how that went
	std::thread m_Thread;

	// handed over to the drawing object, under m_MeshLock
	std::mutex m_MeshLock;
	std::vector<SimMesh> m_Meshes;
	std::atomic<int> m_Generation;

	// the cutting thread's own
	SimGrid *m_Grid;
	int m_Applied;
	bool m_Stale;						// the surface that is handed over is of the lattice before, or of the path as it was:
										// it stays on the screen until the Stock as it is now has one to show in its place
	int m_Shown;						// poses that the surface which was handed over last shows as cut
	std::vector<CSimCutTool> m_Tools;	// profiles of m_ToolDefs
	std::shared_ptr<SimBefore> m_Before;	// what the Stock on the lattice was cut with before the path
	int m_Batch;
	double m_CutSeconds, m_MeshSeconds;
	double m_LatticeCut;				// seconds of cutting since the Stock on this lattice was last as before the path
	double m_TiltWork;					// what a line that a tilted tool passes counts for, in lines that an upright tool passes (see CutBatch)
	bool m_TiltStart;					// no tool has stood tilted in this Stock yet

	void Start();
	void Run();
	int EffTarget() const;
	bool Pending() const;
	bool RefineDue() const;
	void MakeGrid(const double lo[3], const double hi[3], double wanted, double res, int coarsened, bool keep);
	void Restart();
	void ForgetSurface();
	int CutBatch(const std::vector<SimPose> &poses, const std::vector<CSimCutTool> &tools, bool havePrev, int first, bool before, bool *stopped,
				 double tiltRes, double *slow);
	void MeshDirty();
	void FillStats(SimStats *s);
	const char *WriteStock(const SimChar *file);
};
