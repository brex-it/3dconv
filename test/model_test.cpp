#include <iostream>
#include <memory>
#include <random>
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

	/* After modifying the face vertices it counts as
	 * a different face so we can insert it into m. */
	f.add_vertex(3);
	m->add_face(f);
	REQUIRE(m->faces().size() == 2);
	f_it = m->faces().find(Face{m, {0, 1, 2, 3}});
	REQUIRE(f_it != m->faces().end());

	/* We cannot insert f into a Model object which is
	 * not associated with it at construction time. */
	auto m2 = Model::create();
	REQUIRE_THROWS_MATCHES(m2->add_face(f), ModelError,
		Message("Faces can only be added to their associated Model."));
	REQUIRE(m2->faces().size() == 0);
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
	random_device rdev;
	default_random_engine gen{rdev()};
	uniform_int_distribution<int> idist(10, 47);
	uniform_real_distribution<float> rdist(-50.f, 50.f);

	auto m = Model::create();
	Face f{m};

	/* Create 9 random vertices an add them to one face */
	for (int i = 0; i < 9; ++i) {
		m->add_vertex({rdist(gen), rdist(gen), rdist(gen), 1.f});
		f.add_vertex(i);
	}

	m->add_face(f);
	m->triangulate();

	/* After the triangulation we get 7 triangles */
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
