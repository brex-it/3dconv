#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "catch.hpp"

#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

using namespace linalg;
using namespace std;

using Catch::Matchers::Message;

TEST_CASE("Insert vertices and v.normals into Model", "[model]") {
	auto m = Model::create();
	random_device rdev;
	default_random_engine gen{rdev()};
	uniform_int_distribution<int> idist(10, 47);
	uniform_real_distribution<float> rdist(-50.f, 50.f);

	/* First vertex */
	m->add_vertex({-10.01f, -.77f, -2.f, 1.f});

	/* Random number of random vertices and texture vertices */
	auto vnum1 = idist(gen);
	for (int i = 0; i < vnum1; ++i) {
		m->add_vertex({rdist(gen), rdist(gen), rdist(gen), 1.f});
		m->add_texture_vertex({rdist(gen), rdist(gen), rdist(gen)});
	}

	/* Vertex, texture vertex and vertex normal with fixed value */
	m->add_vertex({3.88f, -224.7f, 63.1f, 1.f});
	m->add_texture_vertex({.931f, 224.7f, -3.3f});
	m->add_vertex_normal({-3.52f, -7.242047f, 63.1f});

	/* Random number of random vertices and vertex normals */
	auto vnum2 = idist(gen);
	for (int i = 0; i < vnum2; ++i) {
		m->add_vertex({rdist(gen), rdist(gen), rdist(gen), 1.f});
		m->add_vertex_normal({rdist(gen), rdist(gen), rdist(gen)});
	}

	REQUIRE(m->vertices().size() == vnum1 + vnum2 + 2);
	REQUIRE(m->vertices()[0] == FVec<float, 4>{-10.01f, -.77f, -2.f, 1.f});
	REQUIRE(m->vertices()[vnum1 + 1]
		== FVec<float, 4>{3.88f, -224.7f, 63.1f, 1.f});

	REQUIRE(m->texture_vertices().size() == vnum1 + 1);
	REQUIRE(m->texture_vertices()[vnum1]
		== FVec<float, 3>{.931f, 224.7f, -3.3f});

	REQUIRE(m->vertex_normals().size() == vnum2 + 1);
	REQUIRE(m->vertex_normals()[0]
		== FVec<float, 3>{-3.52f, -7.242047f, 63.1f});
}

TEST_CASE("Face creation and index insertion", "[face]") {
	auto m = Model::create();
	Face f{m};

	/* Vertex indices should be unique */
	f.add_vertex(371);
	f.add_vertex(4);
	f.add_vertex(234);
	REQUIRE(f.vertices().size() == 3);
	f.add_vertex(371);
	REQUIRE(f.vertices().size() == 3);
	f.add_vertex(233);
	REQUIRE(f.vertices().size() == 4);
	REQUIRE(f.vertices() == vector<size_t>{371, 4, 234, 233});

	/* Texture vertex indices should be unique, too */
	f.add_texture_vertex(5);
	f.add_texture_vertex(11);
	REQUIRE(f.texture_vertices().size() == 2);
	f.add_texture_vertex(11);
	REQUIRE(f.texture_vertices().size() == 2);
	f.add_texture_vertex(16);
	REQUIRE(f.texture_vertices().size() == 3);
	REQUIRE(f.texture_vertices() == vector<size_t>{5, 11, 16});

	/* But we can use the same vertex normal for more vertices */
	f.add_vertex_normal(192);
	f.add_vertex_normal(8);
	REQUIRE(f.vertex_normals().size() == 2);
	f.add_vertex_normal(8);
	REQUIRE(f.vertex_normals().size() == 3);
	REQUIRE(f.vertex_normals() == vector<size_t>{192, 8, 8});
}

TEST_CASE("Face insertion into Model", "[face][model]") {
	auto m = Model::create();
	/* Create a Face with some vertices */
	Face f{m};
	f.add_vertex(0);
	f.add_vertex(1);
	f.add_vertex(2);

	/* Add the face to the model */
	m->add_face(f);
	REQUIRE(m->faces().size() == 1);
	auto f_it = m->faces().find(Face{m, {0, 1, 2}});
	REQUIRE(f_it != m->faces().end());

	/* After second addition m should still contain
	 * only one face. No duplicates allowed. */
	m->add_face(f);
	REQUIRE(m->faces().size() == 1);

	/* A face which has more than two common vertices with any
	 * other already inserted face should not be inserted. */
	f.add_vertex(3);
	m->add_face(f);
	REQUIRE(m->faces().size() == 1);

	/* Adding a "totally" different face is allowed. */
	Face f_new{m, {2, 4, 6, 8}};
	m->add_face(f_new);
	REQUIRE(m->faces().size() == 2);
	f_it = m->faces().find(Face{m, {2, 4, 6, 8}});
	REQUIRE(f_it != m->faces().end());

	/* We cannot insert f into a Model object which is
	 * not associated with it at construction time. */
	auto m2 = Model::create();
	REQUIRE_THROWS_MATCHES(m2->add_face(f), ModelError,
		Message("Faces can only be added to their associated Model."));
	REQUIRE(m2->faces().size() == 0);
}


TEST_CASE("weak_ptr to the assoc. Model should prevent reference cycles",
		"[face][error-handling]") {
	/* NOTE: Situations like this are not likely to happen because Face
	 *       objects are copied as a part of their associated Model object.
	 *       Fortunately, there are no other currently known situation
	 *       where calling any Model or Face related public member
	 *       function results in Model expiration error.
	 * POSSIBLE IMPROVEMENT: Maybe Face class should be a private type of
	 *                       Model. ?? */
	auto m = Model::create();
	m->add_vertex({1.43, 5.6, 17, 1});
	m->add_vertex({-4.2, .66, -1.4, 1});
	m->add_vertex({1, -6.4, 11, 1});
	Face f1{m};
	{
		auto m_expiring = Model::create(m);
		Face f2{m_expiring, {0, 1, 2}};
		/* Copy f2's state before normal computation. */
		Face f3{f2};
		/* Model object pointed by m_expiring is alive here. */
		REQUIRE_NOTHROW(f2.normal());
		/* Weak pointer in f3 is copied into f1. */
		f1 = f3;
		/* Model object pointed by m_expiring is destructed here. */
	}
	/* The locking of weak_ptr in f1 yields a nullptr. */
	REQUIRE_THROWS_MATCHES(f1.normal(), ModelError,
		Message("The associated Model object has been expired."));
}

TEST_CASE("Test model validation", "[face][model][error-handling]") {
	auto m = Model::create();

	/* Random co-planar vertices except for vertex number 3 */
	m->add_vertex({-16.6043, 35.1819, 44.1489, 1.});  /* 0 */
	m->add_vertex({38.1404, -10.3665, -34.869, 1.});  /* 1 */
	m->add_vertex({1.38671, -46.6433, -35.3043, 1.}); /* 2 */
	m->add_vertex({-2., 70., -70, 1.});               /* 3 */
	m->add_vertex({29.0947, -73.2367, -78.1297, 1.}); /* 4 */

	/* Random texture vertices and vertex normals */
	m->add_texture_vertex({2.99646, 41.2849, 33.7862}); /* 0 */
	m->add_texture_vertex({-26.656, 20.1958, 39.978});  /* 1 */
	m->add_texture_vertex({1.40029, 43.9466, 37.1571}); /* 2 */

	m->add_vertex_normal({-13.9796, 30.2638, 38.173});   /* 0 */
	m->add_vertex_normal({14.0015, -28.2315, -30.3864}); /* 1 */
	m->add_vertex_normal({-18.5836, 28.4579, 40.3091});  /* 2 */

	Face f{m};
	f.add_vertex(1);
	f.add_vertex(0);

	SECTION("Not enough vertices") {
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0) Face must contain at least 3 vertices."));
	}

	f.add_vertex(2);

	SECTION("Not the proper amount of texture vertices") {
		f.add_texture_vertex(3);
		f.add_texture_vertex(1);
		f.add_texture_vertex(2);
		f.add_texture_vertex(0);
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0:2) Face must either contain no texture "
			"vertices or the same number of texture vertices as "
			"geometric vertices."));
	}
	SECTION("Not the proper amount of vertex normals") {
		f.add_vertex_normal(2);
		f.add_vertex_normal(1);
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0:2) Face must either contain no vertex "
			"normals or the same number of vertex normals as "
			"geometric vertices."));
	}

	SECTION("Vertex index out of range") {
		f.add_vertex(14);
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0:2:14) Invalid vertex index."));
	}
	SECTION("Texture vertex index out of range") {
		f.add_texture_vertex(0);
		f.add_texture_vertex(2);
		f.add_texture_vertex(9);
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0:2) Invalid texture vertex index."));
	}
	SECTION("Vertex normal index out of range") {
		f.add_vertex_normal(2);
		f.add_vertex_normal(1);
		f.add_vertex_normal(20);
		m->add_face(f);
		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
			Message("(Face:1:0:2) Invalid vertex normal index."));
	}

	/* NOTE: This section is disabled because co-planarity
	 *       checking is currently not built into the model.
	 *       (see: src/model.cpp)
	 *       The check is left out because of the inaccuracy
	 *       of many test models. */
//	SECTION("Violate co-planarity") {
//		f.add_vertex(3);
//		m->add_face(f);
//		REQUIRE_THROWS_MATCHES(m->validate(), ModelError,
//			Message("(Face:1:0:2:3) The given points are not co-planar."));
//	}

	SECTION("Everything is fine") {
		f.add_vertex(4);
		REQUIRE_NOTHROW(m->validate());
	}
}

TEST_CASE("Face convexification test", "[face][model]") {
	auto m = Model::create();
	vector<FVec<float, 4>> verts;
	typename Face::IndexVecT f_indices;
	vector<typename Face::IndexVecT> exp_f_indices;

	SECTION("Only one concavity") {
		verts = {
			{1.f, .5f, 0.f, 1.f},    /* 0 */
			{0.f, .5f, 0.f, 1.f},    /* 1 */
			{.25f, .5f, 1.f, 1.f},   /* 2 */
			{-1.f, .5f, .5f, 1.f},   /* 3 */
			{-1.f, .5f, -1.f, 1.f},  /* 4 */
			{-.25f, .5f, -1.f, 1.f}, /* 5 */
		};
		f_indices = {0, 1, 2, 3, 4, 5};
		exp_f_indices = {
			{0, 1, 4, 5},
			{1, 2, 3, 4},
		};
	}

	SECTION("More concavities") {
		verts = {
			{-7.8f, 2.5f, -2.f, 1.f}, /* 0 */
			{-7.8f, 3.f, 0.f, 1.f},   /* 1 */
			{-7.8f, 2.f, 1.f, 1.f},   /* 2 */
			{-7.8f, 3.f, 1.5f, 1.f},  /* 3 */
			{-7.8f, 4.f, 1.f, 1.f},   /* 4 */
			{-7.8f, 6.f, 2.5f, 1.f},  /* 5 */
			{-7.8f, 6.f, -1.5f, 1.f}, /* 6 */
			{-7.8f, 5.f, -1.f, 1.f},  /* 7 */
		};
		f_indices = {0, 1, 2, 3, 4, 5, 6, 7};
		exp_f_indices = {
			{7, 0, 1, 4},
			{1, 2, 3, 4},
			{6, 7, 4, 5},
		};
	}

	for (const auto &v : verts) {
		m->add_vertex(FVec<float, 4>{v});
	}
	m->add_face(Face{m, f_indices});

	REQUIRE(m->faces().size() == 1);

	/* After convexification there should be more than one faces */
	m->convexify_faces();
	REQUIRE(m->faces().size() == exp_f_indices.size());

	/* Check whether we got a valid partitioning */
	for (const auto &fi : exp_f_indices) {
		auto f_it = m->faces().find(Face{m, fi});
		REQUIRE(f_it != m->faces().end());
	}
}

TEST_CASE("Transformation tests", "[face][model]") {
	auto m = Model::create();
	m->add_vertex({3.f, 4.f, 2.f, 1.f});
	m->add_vertex_normal({-1.f, 2.f, -2.f});

	SECTION("Translation") {
		m->transform({
				1,0,0,2,
				0,1,0,4,
				0,0,1,6,
				0,0,0,1});
		REQUIRE(m->vertices()[0] == FVec<float, 4>{5.f, 8.f, 8.f, 1.f});
		/* Translation should not change vertex normals,
		 * since vectors are translation-invariant.
		 * (They are zero extended during matrix multiplication.) */
		REQUIRE(m->vertex_normals()[0] == FVec<float, 3>{-1.f, 2.f, -2.f});
	}

	SECTION("Scaling") {
		m->transform({
				2,0,0,0,
				0,1.5,0,0,
				0,0,-3,0,
				0,0,0,1});
		REQUIRE(m->vertices()[0] == FVec<float, 4>{6.f, 6.f, -6.f, 1.f});
		REQUIRE(m->vertex_normals()[0] == FVec<float, 3>{-2.f, 3.f, 6.f});
	}
}

TEST_CASE("Triangulation test", "[face][model]") {
	auto m = Model::create();
	Face f{m};

	/* Create 9 vertices lying on a circle and add them to one face */
	for (int i = 0; i < 9; ++i) {
		float angle = 2 * M_PI * i / 9;
		m->add_vertex({cosf(angle), 1.f, sinf(angle), 1.f});
		f.add_vertex(i);
	}

	m->add_face(f);
	REQUIRE(m->is_triangulated() == false);

	m->triangulate();

	/* After the triangulation we get 7 triangles */
	REQUIRE(m->is_triangulated() == true);
	REQUIRE(m->faces().size() == 7);

	/* The triangles are created by partitioning the polygon
	 * by a zig-zag shaped path */
	auto f_it = m->faces().begin();
	REQUIRE(f_it++->vertices() == vector<size_t>{0, 1, 2});
	REQUIRE(f_it++->vertices() == vector<size_t>{8, 0, 2});
	REQUIRE(f_it++->vertices() == vector<size_t>{8, 2, 3});
	REQUIRE(f_it++->vertices() == vector<size_t>{7, 3, 4});
	REQUIRE(f_it++->vertices() == vector<size_t>{7, 8, 3});
	REQUIRE(f_it++->vertices() == vector<size_t>{6, 4, 5});
	REQUIRE(f_it++->vertices() == vector<size_t>{6, 7, 4});
	REQUIRE(f_it == m->faces().end());
}

TEST_CASE("Connectivity test", "[face][model]") {
	auto m = Model::create();

	/* Add 6 vertices */
	for (size_t i = 0; i < 6; ++i) {
		m->add_vertex({1.f, 2.f, 3.f, 1.f});
	}

	/* Add faces connecting all vertices */
	Face faces[3]{
		{m, {0, 1, 2}},
		{m, {0, 1, 3}},
		{m, {4, 5, 3}},
	};
	for (size_t i = 0; i < 3; ++i) {
		m->add_face(faces[i]);
	}
	REQUIRE(m->is_connected());

	/* Adding more vertices without connecting them
	 * results in a disconnected model. */
	for (size_t i = 0; i < 3; ++i) {
		m->add_vertex({1.f, 2.f, 3.f, 1.f});
	}
	REQUIRE(!m->is_connected());

	/* We cover all of the vertices with a new face but
	 * it is not connected to the previous faces so the
	 * model remains disconnected. */
	m->add_face({m, {6, 7, 8}});
	REQUIRE(!m->is_connected());
}

TEST_CASE("Convexity test", "[face][model]") {
	auto m_convex = Model::create();

	m_convex->add_vertex({1.f, 1.f, -1.f});   /* 0 */
	m_convex->add_vertex({1.f, 1.f, 0.f});    /* 1 */
	m_convex->add_vertex({1.f, 0.f, 0.f});    /* 2 */
	m_convex->add_vertex({0.f, 1.5f, -1.5f}); /* 3 */

	m_convex->add_face({m_convex, {0, 3, 1}});
	m_convex->add_face({m_convex, {0, 2, 3}});
	m_convex->add_face({m_convex, {1, 3, 2}});

	/* Copy the model at this stage */
	auto m_concave = Model::create(m_convex);

	/* One more closing face for m_convex */
	m_convex->add_face({m_convex, {0, 1, 2}});

	REQUIRE(m_convex->is_convex());

	/* Complete the copy model to form a concave shape */
	m_concave->add_vertex({2.f, 1.f, 0.f});  /* 4 */
	m_concave->add_vertex({2.f, 0.f, -1.f}); /* 5 */

	m_concave->add_face({m_concave, {0, 1, 4}});
	m_concave->add_face({m_concave, {1, 2, 4}});
	m_concave->add_face({m_concave, {0, 5, 2}});
	m_concave->add_face({m_concave, {0, 4, 5}});
	m_concave->add_face({m_concave, {2, 5, 4}});

	REQUIRE(!m_concave->is_convex());
}

TEST_CASE("Water tightness test", "[face][model]") {
	auto m = Model::create();
	bool expected_water_tightness{true};
	string expected_msg;
	vector<typename Face::IndexVecT> face_index_vecs;

	vector<FVec<float, 4>> verts{
		{2.f, -1.f, 1.f, 1.f},   /* 0 */
		{1.f, -1.f, 1.f, 1.f},   /* 1 */
		{-1.f, -1.f, 1.f, 1.f},  /* 2 */
		{-2.f, -1.f, 1.f, 1.f},  /* 3 */

		{1.f, 1.f, 1.f, 1.f},    /* 4 */
		{-1.f, 1.f, 1.f, 1.f},   /* 5 */

		{2.f, 2.f, 1.f, 1.f},    /* 6 */
		{1.f, 2.f, 1.f, 1.f},    /* 7 */
		{-1.f, 2.f, 1.f, 1.f},   /* 8 */
		{-2.f, 2.f, 1.f, 1.f},   /* 9 */

		{2.f, -1.f, -1.f, 1.f},  /* 10 */
		{1.f, -1.f, -1.f, 1.f},  /* 11 */
		{-1.f, -1.f, -1.f, 1.f}, /* 12 */
		{-2.f, -1.f, -1.f, 1.f}, /* 13 */

		{1.f, 1.f, -1.f, 1.f},   /* 14 */
		{-1.f, 1.f, -1.f, 1.f},  /* 15 */

		{2.f, 2.f, -1.f, 1.f},   /* 16 */
		{1.f, 2.f, -1.f, 1.f},   /* 17 */
		{-1.f, 2.f, -1.f, 1.f},  /* 18 */
		{-2.f, 2.f, -1.f, 1.f},  /* 19 */
	};

	SECTION("Watertight model") {
		face_index_vecs = {
			{0, 6, 7, 4, 1},
			{1, 4, 5, 2},
			{2, 5, 8, 9, 3},
			{10, 11, 14, 17, 16},
			{11, 12, 15, 14},
			{12, 13, 19, 18, 15},
			{0, 10, 16, 6},
			{4, 7, 17, 14},
			{5, 15, 18, 8},
			{3, 9, 19, 13},
			{6, 16, 17, 7},
			{4, 14, 15, 5},
			{8, 18, 19, 9},
			{0, 1, 2, 3, 13, 12, 11, 10},
		};
		expected_water_tightness = true;
		expected_msg = "";
	}


	SECTION("Self intersection") {
		m->add_vertex({1.f, 3.f, 1.f, 1.f});  /* 20 */
		m->add_vertex({1.f, 3.f, -1.f, 1.f}); /* 21 */

		face_index_vecs = {
			{0, 6, 7, 4, 1},
			{1, 4, 5, 2},
			{2, 5, 9, 3},
			{10, 11, 14, 17, 16},
			{11, 12, 15, 14},
			{12, 13, 19, 15},
			{0, 10, 16, 6},
			{4, 8, 18, 14},
			{5, 15, 21, 20},
			{3, 9, 19, 13},
			{6, 16, 17, 18, 8, 7},
			{4, 14, 15, 5},
			{9, 20, 21, 19},
			{0, 1, 2, 3, 13, 12, 11, 10},
			{5, 20, 9},
			{4, 7, 8},
			{14, 18, 17},
			{15, 19, 21},
		};
		expected_water_tightness = false;
		expected_msg = "(Face:0:10:16:6) Self intersection";
	}

	SECTION("Boundary edge (occurrence: 1)") {
		face_index_vecs = {
			{1, 4, 5, 2},
			{1, 11, 14, 4},
			{11, 12, 15, 14},
			{2, 5, 15, 12},
			{4, 14, 15, 5},
		};
		expected_water_tightness = false;
		expected_msg = "(Edge:1:2) Boundary edge";
	}

	SECTION("Non-manifold edge (occurrence: 4)") {
		face_index_vecs = {
			{1, 4, 5, 2},
			{1, 11, 14, 4},
			{11, 12, 15, 14},
			{2, 5, 15, 12},
			{4, 14, 15, 5},
			{1, 2, 12, 11},
			{1, 6, 16, 11},
			{0, 6, 1},
			{10, 11, 16},
			{0, 10, 16, 6},
			{0, 1, 11, 10},
		};
		expected_water_tightness = false;
		expected_msg = "(Edge:1:11) Non-manifold edge";
	}

	SECTION("Non-manifold vertex") {
		face_index_vecs = {
			{1, 4, 5, 2},
			{1, 11, 14, 4},
			{11, 12, 15, 14},
			{2, 5, 15, 12},
			{4, 14, 15, 5},
			{1, 2, 12, 11},
			{2, 9, 3},
			{3, 9, 19, 13},
			{2, 19, 9},
			{2, 3, 13},
			{2, 13, 19},
		};
		expected_water_tightness = false;
		expected_msg = "(Vertex:2) Non-manifold vertex";
	}

	SECTION("Disconnected but watertight model") {
		face_index_vecs = {
			{0, 4, 1},
			{10, 11, 14},
			{2, 5, 3},
			{12, 13, 15},
			{0, 10, 14, 4},
			{3, 5, 15, 13},
			{1, 4, 14, 11},
			{2, 12, 15, 5},
			{0, 1, 11, 10},
			{2, 3, 13, 12},
		};
		expected_water_tightness = true;
		expected_msg = "";
	}

	for (const auto &v : verts) {
		m->add_vertex(FVec<float, 4>{v});
	}
	for (const auto &fiv : face_index_vecs) {
		m->add_face(Face{m, fiv});
	}

	string msg;
	bool water_tightness = m->is_watertight(msg);
	REQUIRE(water_tightness == expected_water_tightness);
	REQUIRE(msg == expected_msg);

	REQUIRE(m->is_watertight() == expected_water_tightness);
}
