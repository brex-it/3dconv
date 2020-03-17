#include <string>

#include "catch.hpp"

#include <3dconv/cli.hpp>
#include <3dconv/linalg.hpp>

#include "test_tools.hpp"

using namespace linalg;
using namespace std;

using Catch::Matchers::Message;

TEST_CASE("File format parsing", "[iotypes]") {
	string ifile, ofile, iotypes, itype, otype;
	string itype_expected, otype_expected;

	ifile = "in-filename.in-ext";
	ofile = "out-filename.out-ext";

	SECTION("Both from iotypes") {
		iotypes = "in-type:out-type";
		itype_expected = "in-type";
		otype_expected = "out-type";
	}

	SECTION("itype from iotypes") {
		iotypes = "in-type:";
		itype_expected = "in-type";
		otype_expected = "out-ext";
	}

	SECTION("otype from iotypes") {
		iotypes = ":out-type";
		itype_expected = "in-ext";
		otype_expected = "out-type";
	}

	SECTION("Both from file extensions") {
		iotypes = "";
		itype_expected = "in-ext";
		otype_expected = "out-ext";
	}

	parse_iotypes(ifile, ofile, iotypes, itype, otype);

	REQUIRE(itype == itype_expected);
	REQUIRE(otype == otype_expected);
}

TEST_CASE("File format parsing errors", "[iotypes][error-handling]") {
	string itype, otype;

	SECTION("Invalid iotypes string (no colon)") {
		REQUIRE_THROWS_MATCHES(
			parse_iotypes("", "", "some-type", itype, otype),
			CLIError,
			Message("':' character cannot be omitted."));
	}

	SECTION("Invalid iotypes string (too many colons)") {
		REQUIRE_THROWS_MATCHES(
			parse_iotypes("", "", "type1:type2:type3", itype, otype),
			CLIError,
			Message("Too many arguments for type specification."));
	}

	SECTION("Cannot deduce input type") {
		REQUIRE_THROWS_MATCHES(
			parse_iotypes("", "", "", itype, otype),
			CLIError,
			Message("Unable to determine input file format."));
	}

	SECTION("Cannot deduce output type") {
		REQUIRE_THROWS_MATCHES(
			parse_iotypes("", "", "in-type:", itype, otype),
			CLIError,
			Message("Unable to determine output file format."));
	}
}

TEST_CASE("Transformation parsing", "[transformation]") {
	string transforms;
	FMatSq<float, 4> matrix;

	SECTION("Rotation") {
		transforms = "ro:-.5:3:1.2:1.570796";
		matrix = {
			0.0233863, -0.50734, 0.861428, 0,
			0.226704, 0.841908, 0.489689, 0,
			-0.973683, 0.183837, 0.134705, 0,
			0, 0, 0, 1};
	}

	SECTION("Scaling") {
		transforms = "sc:-1.5";
		matrix = {
			-1.5, 0, 0, 0,
			0, -1.5, 0, 0,
			0, 0, -1.5, 0,
			0, 0, 0, 1};
	}

	SECTION("Translation") {
		transforms = "tr:1:-2:4";
		matrix = {
			1, 0, 0, 1,
			0, 1, 0, -2,
			0, 0, 1, 4,
			0, 0, 0, 1};
	}

	REQUIRE(matrices_approx_equal(parse_transforms(transforms), matrix));
}

TEST_CASE("Transformation parsing errors", "[transformation][error-handling]") {
	/* Missing transformation */
	REQUIRE_THROWS_MATCHES(parse_transforms("sc:2.1,,tr:2:2:-7"),
		CLIError, Message("Missing transformation."));

	/* Not enough arguments */
	REQUIRE_THROWS_MATCHES(parse_transforms("ro:1:2:3"),
		CLIError, Message("Not enough arguments for rotation."));

	REQUIRE_THROWS_MATCHES(parse_transforms("sc"),
		CLIError, Message("Not enough arguments for scaling."));

	REQUIRE_THROWS_MATCHES(parse_transforms("tr:1:2"),
		CLIError, Message("Not enough arguments for translation."));

	/* Too many arguments */
	REQUIRE_THROWS_MATCHES(parse_transforms("ro:1:2:3:4:5"),
		CLIError, Message("Too many arguments for rotation."));

	REQUIRE_THROWS_MATCHES(parse_transforms("sc:1:2"),
		CLIError, Message("Too many arguments for scaling."));

	REQUIRE_THROWS_MATCHES(parse_transforms("tr:1:2:3:4"),
		CLIError, Message("Too many arguments for translation."));

	/* Unknown transformation */
	REQUIRE_THROWS_MATCHES(parse_transforms("un:1:2:3"),
		CLIError, Message("Unknown transformation: un"));
}
