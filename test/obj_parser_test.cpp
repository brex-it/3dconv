#include <filesystem>
#include <iostream>
#include <memory>

#include "catch.hpp"
#include "test_tools.hpp"

#include <3dconv/io.hpp>
#include <3dconv/model.hpp>

using namespace std;

using Catch::Matchers::Message;

auto test_files_dir = get_test_files_dir_path() / "obj_parser_test";

TEST_CASE("Reading OBJ file", "[parser]") {
	auto parser = IOMap<Parser>::get("obj");
	shared_ptr<Model> model;

	SECTION("Should handle comments") {
		REQUIRE_NOTHROW(parser->open(test_files_dir / "comment.obj"));
		model = (*parser)();

		/* Model should be empty */
		REQUIRE(model->vertices().empty());
		REQUIRE(model->texture_vertices().empty());
		REQUIRE(model->vertex_normals().empty());
		REQUIRE(model->faces().empty());
	}

	SECTION("Reading supported_statements.obj") {
		REQUIRE_NOTHROW(parser->open(test_files_dir
			/ "supported_statements.obj"));
		model = (*parser)();

		/* Required model properties */
		REQUIRE(model->vertices().size() == 8);
		REQUIRE(model->texture_vertices().size() == 6);
		REQUIRE(model->vertex_normals().size() == 8);
		REQUIRE(model->faces().size() == 2);
		REQUIRE(!model->is_triangulated());

		/* Check a random texture vertex */
		REQUIRE(model->texture_vertices()[2] == FVec<float, 3>{.5, 5.7, 1.9});

		/* Search faces and check their normals */
		auto f1 = model->faces().find(
			Face(model, {0, 1, 2, 3, 4}, {}, {0, 1, 2, 3, 4}));
		REQUIRE(f1 != model->faces().end());

		REQUIRE(f1->normal()[0] == Approx(-.577324));
		REQUIRE(f1->normal()[1] == Approx(-.577376));
		REQUIRE(f1->normal()[2] == Approx(.57735));

		auto f2 = model->faces().find(
			Face(model, {5, 6, 7}, {1, 3, 4}, {5, 6, 7}));
		REQUIRE(f2 != model->faces().end());

		REQUIRE(f2->normal()[0] == Approx(-.6626));
		REQUIRE(f2->normal()[1] == Approx(-.156932));
		REQUIRE(f2->normal()[2] == Approx(.732348));
	}
}

TEST_CASE("Test parsing errors", "[error-handling]") {
	auto parser = IOMap<Parser>::get("obj");

	SECTION("Invalid statement") {
		parser->open(test_files_dir / "invalid_statement.obj");
		REQUIRE_THROWS_MATCHES((*parser)(), ParseError,
			Message("Invalid statement: g groupname"));
	}

	/* Face syntax errors */
	SECTION("Wrong face syntax (inconsistent groups)") {
		parser->open(test_files_dir / "wrong_face_syntax_groups.obj");
		CHECK_THROWS_MATCHES((*parser)(), ParseError,
			Message("Every index group must contain "
				"the same amount of elements."));
	}
	SECTION("Wrong face syntax (slashes with omitted tv and vn arguments)") {
		parser->open(test_files_dir / "wrong_face_syntax_slashes.obj");
		CHECK_THROWS_MATCHES((*parser)(), ParseError,
			Message("Last char cannot be slash: 3//"));
	}
	SECTION("Wrong face syntax (not enough arguments)") {
		parser->open(test_files_dir / "wrong_face_syntax_args.obj");
		CHECK_THROWS_MATCHES((*parser)(), ParseError,
			Message("Faces must contain at least three "
				"distinct vertex indices."));
	}

	SECTION("Wrong relative indexing") {
		parser->open(test_files_dir / "wrong_vertex_index.obj");
		CHECK_THROWS_MATCHES((*parser)(), ParseError,
			Message("Invalid relative index: -4"));
	}
}
