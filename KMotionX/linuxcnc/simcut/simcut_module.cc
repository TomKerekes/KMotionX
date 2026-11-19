// simcut_module.cc : the material removal engine of KMotionCNC (SimCut.cpp) as a Python module
// Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT (see SimCut-LICENSE.txt
// beside SimCut.cpp)
//
// Module kmotion_simcut, class Stock: CSimCutStock with its poses and meshes as numpy arrays.  The
// engine cuts on threads of its own; the calls that wait release the GIL.  kmotion_stock.py drives
// it from the preview of a LinuxCNC GUI; README.md says how.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <array>
#include <vector>
#include <string>
#include <stdexcept>

#include "SimCut.h"

namespace py = pybind11;

namespace {

typedef std::array<double, 3> Vec3;

class Stock
{
	CSimCutStock S;

public:
	// --- the Stock ---------------------------------------------------------
	void define(Vec3 lo, Vec3 hi, double resolution) { S.Define(lo.data(), hi.data(), resolution); }
	void remove() { S.Remove(); }
	bool defined() { return S.Defined(); }

	// --- tools -------------------------------------------------------------
	int add_tool_image(const std::string &file, double scale, Vec3 offset, double diameter)
	{
		return S.AddTool(file.c_str(), scale, offset.data(), diameter);
	}
	int add_tool_flat(double diameter)
	{
		CSimCutTool t;
		if (!t.SetFlat(diameter)) throw std::invalid_argument("a flat end mill needs a diameter above 0");
		return S.AddTool(t);
	}
	int add_tool_profile(const std::vector<double> &h, const std::vector<double> &r, bool open)
	{
		if (h.size() != r.size() || h.size() < 2) throw std::invalid_argument("a profile is two or more (height, radius) pairs");
		CSimCutTool t;
		for (size_t i = 0; i < h.size(); i++) t.Add(h[i], r[i]);
		t.Open = open;
		if (!t.Finish()) throw std::invalid_argument("the profile has nothing to cut with");
		return S.AddTool(t);
	}
	void forget_tool_images() { S.ForgetToolImages(); }

	// What the engine makes of a tool image (for checking a tool table): (heights, radii, open)
	static py::tuple read_tool_image(const std::string &file, double scale, Vec3 offset)
	{
		CSimCutTool t;
		if (!t.SetFromVrml(file.c_str(), scale, offset.data())) throw std::runtime_error("the tool image could not be read: " + file);
		return py::make_tuple(t.h, t.r, t.Open);
	}

	// --- the path ----------------------------------------------------------
	// xyz: (n, 3) float32 tool tip positions; tool: (n,) tool indices from add_tool_*; flags: (n,)
	// SIMPOSE_xxx; axis: (n, 3) float32 which way the tool points (tip toward spindle), or None for
	// upright.  Pose i of the engine is vertex i of the path.
	void add_poses(py::array_t<float, py::array::c_style | py::array::forcecast> xyz,
				   py::array_t<int, py::array::c_style | py::array::forcecast> tool,
				   py::array_t<int, py::array::c_style | py::array::forcecast> flags,
				   py::object axis)
	{
		if (xyz.ndim() != 2 || xyz.shape(1) != 3) throw std::invalid_argument("xyz must be (n, 3)");
		py::ssize_t n = xyz.shape(0);
		if (tool.ndim() != 1 || tool.shape(0) != n || flags.ndim() != 1 || flags.shape(0) != n)
			throw std::invalid_argument("tool and flags must have one entry per pose");
		py::array_t<float, py::array::c_style | py::array::forcecast> ax;
		bool haveAxis = !axis.is_none();
		if (haveAxis)
		{
			ax = axis.cast<py::array_t<float, py::array::c_style | py::array::forcecast>>();
			if (ax.ndim() != 2 || ax.shape(1) != 3 || ax.shape(0) != n) throw std::invalid_argument("axis must be (n, 3) or None");
		}
		auto p = xyz.unchecked<2>();
		auto t = tool.unchecked<1>();
		auto f = flags.unchecked<1>();
		py::gil_scoped_release nogil;
		for (py::ssize_t i = 0; i < n; i++)
		{
			double a[3];
			if (haveAxis)
			{
				auto d = ax.unchecked<2>();
				a[0] = d(i, 0); a[1] = d(i, 1); a[2] = d(i, 2);
			}
			S.AddPose(p(i, 0), p(i, 1), p(i, 2), t(i), f(i), haveAxis ? a : NULL);
		}
	}
	void clear_poses() { S.ClearPoses(); }
	void set_pose_count(int count) { S.SetPoseCount(count); }
	void move_pose(int index, float x, float y, float z) { S.MovePose(index, x, y, z); }
	void disable_poses(int first, int count) { S.DisablePoses(first, count); }
	int nb_poses() { return S.NbPoses(); }

	void cut_all() { S.CutAll(); }
	void cut_to(int count) { S.CutTo(count); }
	void uncut_from(int count) { S.UncutFrom(count); }

	bool wait_idle(int ms) { py::gil_scoped_release nogil; return S.WaitIdle(ms); }
	bool busy() { return S.Busy(); }
	bool failed() { return S.Failed(); }
	double resolution_in_use() { return S.ResolutionInUse(); }

	py::dict stats()
	{
		SimStats s;
		{
			py::gil_scoped_release nogil;
			S.GetStats(&s);
		}
		py::dict d;
		d["nodes"] = py::make_tuple(s.nodes[0], s.nodes[1], s.nodes[2]);
		d["origin"] = py::make_tuple(s.origin[0], s.origin[1], s.origin[2]);
		d["step"] = py::make_tuple(s.step[0], s.step[1], s.step[2]);
		d["lines"] = s.lines;
		d["stored"] = s.stored;
		d["pieces"] = s.pieces;
		d["volume"] = py::make_tuple(s.volume[0], s.volume[1], s.volume[2]);
		d["triangles"] = s.triangles;
		d["guessed"] = s.guessed;
		d["cut_seconds"] = s.cutSeconds;
		d["mesh_seconds"] = s.meshSeconds;
		d["applied"] = s.applied;
		d["bytes"] = s.bytes;
		d["wanted"] = s.wanted;
		d["coarsened"] = s.coarsened;
		d["before"] = s.before;
		d["before_applied"] = s.beforeApplied;
		return d;
	}

	// --- Stock files -------------------------------------------------------
	// save and load return None, or the text that says what went wrong
	py::object save(const std::string &file, int wait)
	{
		const char *err;
		{
			py::gil_scoped_release nogil;
			err = S.Save(file.c_str(), wait);
		}
		return err ? py::cast(std::string(err)) : py::none();
	}
	// (error, lo, hi, resolution)
	py::tuple load(const std::string &file)
	{
		double lo[3], hi[3], res = 0;
		const char *err;
		{
			py::gil_scoped_release nogil;
			err = S.Load(file.c_str(), lo, hi, &res);
		}
		return py::make_tuple(err ? py::cast(std::string(err)) : py::none(),
							  py::make_tuple(lo[0], lo[1], lo[2]), py::make_tuple(hi[0], hi[1], hi[2]), res);
	}
	void unload() { S.Unload(); }
	bool loaded() { return S.Loaded(); }
	bool wait_lattice(int ms) { py::gil_scoped_release nogil; return S.WaitLattice(ms); }

	// --- for the drawing -----------------------------------------------------
	int generation() { return S.Generation(); }
	int nb_meshes() { S.LockMeshes(); int n = S.NbMeshes(); S.UnlockMeshes(); return n; }

	// The meshes that changed since the last call: a list of (slot, vertices, indices).  vertices is
	// a uint8 array of 20 bytes a vertex (x y z float, kind float, direction as GL_INT_2_10_10_10_REV),
	// indices a uint16 array, three a triangle; both empty when the slot has no surface any more.
	// The engine's copy is dropped: the surface lives in the drawing buffers alone (republish asks
	// for all of it again).
	py::list take_meshes()
	{
		py::list out;
		S.LockMeshes();
		try
		{
			int n = S.NbMeshes();
			for (int i = 0; i < n; i++)
			{
				SimMesh &m = S.Mesh(i);
				if (!m.changed) continue;
				py::array_t<unsigned char> v((py::ssize_t)(m.v.size() * sizeof(SimVertex)));
				if (!m.v.empty()) memcpy(v.mutable_data(), m.v.data(), m.v.size() * sizeof(SimVertex));
				py::array_t<unsigned short> t((py::ssize_t)m.t.size());
				if (!m.t.empty()) memcpy(t.mutable_data(), m.t.data(), m.t.size() * sizeof(unsigned short));
				m.changed = false;
				std::vector<SimVertex>().swap(m.v);
				std::vector<unsigned short>().swap(m.t);
				out.append(py::make_tuple(i, v, t));
			}
		}
		catch (...)
		{
			S.UnlockMeshes();
			throw;
		}
		S.UnlockMeshes();
		return out;
	}
	void republish() { S.Republish(); }
	bool wait_surface(int ms) { py::gil_scoped_release nogil; return S.WaitSurface(ms); }
};

} // namespace

PYBIND11_MODULE(kmotion_simcut, m)
{
	m.doc() = "KMotionCNC's material removal engine (SimCut.cpp): a Stock that the tool path cuts";
	m.attr("SIMPOSE_MOVE") = SIMPOSE_MOVE;
	m.attr("SIMPOSE_RAPID") = SIMPOSE_RAPID;
	m.attr("SIMPOSE_OFF") = SIMPOSE_OFF;
	m.attr("SIMKIND_STOCK") = SIMKIND_STOCK;
	m.attr("SIMKIND_FEED") = SIMKIND_FEED;
	m.attr("SIMKIND_RAPID") = SIMKIND_RAPID;
	m.attr("VERTEX_BYTES") = (int)sizeof(SimVertex);
	m.def("threads", &CSimCutStock::ThreadsToUse, "how many threads the engine cuts on");
	m.def("read_tool_image", &Stock::read_tool_image, py::arg("file"), py::arg("scale") = 1.0, py::arg("offset") = Vec3{ 0, 0, 0 },
		  "the profile (heights, radii, open) that the engine makes of a tool image");

	py::class_<Stock>(m, "Stock")
		.def(py::init<>())
		.def("define", &Stock::define, py::arg("lo"), py::arg("hi"), py::arg("resolution") = 0.0,
			 "a new uncut Stock between lo and hi (viewer coordinates); resolution 0 = chosen by the Stock")
		.def("remove", &Stock::remove)
		.def("defined", &Stock::defined)
		.def("add_tool_image", &Stock::add_tool_image, py::arg("file"), py::arg("scale") = 1.0, py::arg("offset") = Vec3{ 0, 0, 0 }, py::arg("diameter") = 0.0,
			 "a tool from a tool image file (VRML), else a flat end mill of the diameter; the same file and sizes give the same index")
		.def("add_tool_flat", &Stock::add_tool_flat, py::arg("diameter"))
		.def("add_tool_profile", &Stock::add_tool_profile, py::arg("heights"), py::arg("radii"), py::arg("open") = true)
		.def("forget_tool_images", &Stock::forget_tool_images)
		.def("add_poses", &Stock::add_poses, py::arg("xyz"), py::arg("tool"), py::arg("flags"), py::arg("axis") = py::none())
		.def("clear_poses", &Stock::clear_poses)
		.def("set_pose_count", &Stock::set_pose_count, py::arg("count"))
		.def("move_pose", &Stock::move_pose, py::arg("index"), py::arg("x"), py::arg("y"), py::arg("z"))
		.def("disable_poses", &Stock::disable_poses, py::arg("first"), py::arg("count"))
		.def("nb_poses", &Stock::nb_poses)
		.def("cut_all", &Stock::cut_all)
		.def("cut_to", &Stock::cut_to, py::arg("count"))
		.def("uncut_from", &Stock::uncut_from, py::arg("count"))
		.def("wait_idle", &Stock::wait_idle, py::arg("milliseconds"))
		.def("busy", &Stock::busy)
		.def("failed", &Stock::failed)
		.def("resolution_in_use", &Stock::resolution_in_use)
		.def("stats", &Stock::stats)
		.def("save", &Stock::save, py::arg("file"), py::arg("wait") = 6)
		.def("load", &Stock::load, py::arg("file"))
		.def("unload", &Stock::unload)
		.def("loaded", &Stock::loaded)
		.def("wait_lattice", &Stock::wait_lattice, py::arg("milliseconds"))
		.def("generation", &Stock::generation)
		.def("nb_meshes", &Stock::nb_meshes)
		.def("take_meshes", &Stock::take_meshes)
		.def("republish", &Stock::republish)
		.def("wait_surface", &Stock::wait_surface, py::arg("milliseconds"));
}
