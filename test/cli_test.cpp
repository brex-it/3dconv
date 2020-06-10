#include <string>

#include "catch.hpp"

#include <3dconv/cli.hpp>
#include <3dconv/linalg.hpp>

#include "test_tools.hpp"

using namespace linalg;
using namespace std;

using Catch::Matchers::Message;

TEST_CASE("File format parsing", "[ioformats]") {
	string ifile, ofile, ioformats, iformat, oformat;
	string iformat_expected, oformat_expected;

	ifile = "in-filename.in-ext";
	ofile = "out-filename.out-ext";

	SECTION("Both from ioformats") {
		ioformats = "in-format:out-format";
		iformat_expected = "in-format";
		oformat_expected = "out-format";
	}

	SECTION("iformat from ioformats") {
		ioformats = "in-format:";
		iformat_expected = "in-format";
		oformat_expected = "out-ext";
	}

	SECTION("oformat from ioformats") {
		ioformats = ":out-format";
		iformat_expected = "in-ext";
		oformat_expected = "out-format";
	}

	SECTION("Both from file extensions") {
		ioformats = "";
		iformat_expected = "in-ext";
		oformat_expected = "out-ext";
	}

	parse_ioformats(ifile, ofile, ioformats, iformat, oformat);

	REQUIRE(iformat == iformat_expected);
	REQUIRE(oformat == oformat_expected);
}

TEST_CASE("File format parsing errors", "[ioformats][error-handling]") {
	string iformat, oformat;

	SECTION("Invalid ioformats string (no colon)") {
		REQUIRE_THROWS_MATCHES(
			parse_ioformats("", "", "some-format", iformat, oformat),
			CLIError,
			Message("':' character cannot be omitted."));
	}

	SECTION("Invalid ioformats string (too many colons)") {
		REQUIRE_THROWS_MATCHES(
			parse_ioformats("", "", "format1:format2:format3", iformat, oformat),
			CLIError,
			Message("Too many arguments for format specification."));
	}

	SECTION("Cannot deduce input format") {
		REQUIRE_THROWS_MATCHES(
			parse_ioformats("", "", "", iformat, oformat),
			CLIError,
			Message("Unable to determine input file format."));
	}

	SECTION("Cannot deduce output format") {
		REQUIRE_THROWS_MATCHES(
			parse_ioformats("", "", "in-format:", iformat, oformat),
			CLIError,
			Message("Unable to determine output file format."));
	}
}

bool operator==(const FaceTransforms &lft, const FaceTransforms &rft)
{
	return lft.convexify == rft.convexify
		&& lft.triangulate == rft.triangulate;
}

TEST_CASE("Face transformation parsing", "[transformation]") {
	string transforms;
	FaceTransforms ft;

	SECTION("Only convexification") {
		transforms = "c";
		ft = { .convexify = true };
	}

	SECTION("Only triangulation") {
		transforms = "t";
		ft = { .triangulate = true };
	}

	SECTION("Both of them") {
		transforms = "t,c";
		ft = { .convexify = true, .triangulate = true };
	}

	SECTION("Multiple times") {
		transforms = "t,c,c,t,t,c";
		ft = { .convexify = true, .triangulate = true };
	}

	REQUIRE(parse_face_transforms(transforms) == ft);
}

TEST_CASE("Face transformation parsing errors", "[transformation][error-handling]") {
	REQUIRE_THROWS_MATCHES(parse_face_transforms("f"),
		CLIError, Message("Unknown face transformation: f"));

	REQUIRE_THROWS_MATCHES(parse_face_transforms("wo31c"),
		CLIError, Message("Invalid face transformation: wo31c"));
}

TEST_CASE("Model transformation parsing", "[transformation]") {
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

	SECTION("Skew") {
		SECTION("x -> y") {
			transforms = "sk:xy:.7853981";
			matrix = {
				1, 0, 0, 0,
				1, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1};
		}
		SECTION("x -> z") {
			transforms = "sk:xz:.4636476";
			matrix = {
				1, 0, 0, 0,
				0, 1, 0, 0,
				.5, 0, 1, 0,
				0, 0, 0, 1};
		}
		SECTION("y -> x") {
			transforms = "sk:yx:.4636476";
			matrix = {
				1, .5, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1};
		}
		SECTION("y -> z") {
			transforms = "sk:yz:.7853981";
			matrix = {
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 1, 1, 0,
				0, 0, 0, 1};
		}
		SECTION("z -> x") {
			transforms = "sk:zx:.7853981";
			matrix = {
				1, 0, 1, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1};
		}
		SECTION("z -> y") {
			transforms = "sk:zy:.4636476";
			matrix = {
				1, 0, 0, 0,
				0, 1, .5, 0,
				0, 0, 1, 0,
				0, 0, 0, 1};
		}
	}

	SECTION("Translation") {
		transforms = "tr:1:-2:4";
		matrix = {
			1, 0, 0, 1,
			0, 1, 0, -2,
			0, 0, 1, 4,
			0, 0, 0, 1};
	}

	REQUIRE(matrices_approx_equal(parse_model_transforms(transforms), matrix));
}

TEST_CASE("Model transformation parsing errors",
		"[transformation][error-handling]") {
	/* Missing transformation */
	REQUIRE_THROWS_MATCHES(parse_model_transforms("sc:2.1,,tr:2:2:-7"),
		CLIError, Message("Missing transformation."));

	/* Not enough arguments */
	REQUIRE_THROWS_MATCHES(parse_model_transforms("ro:1:2:3"),
		CLIError, Message("Not enough arguments for rotation."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sc"),
		CLIError, Message("Not enough arguments for scaling."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:zx"),
		CLIError, Message("Not enough arguments for skew."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("tr:1:2"),
		CLIError, Message("Not enough arguments for translation."));

	/* Too many arguments */
	REQUIRE_THROWS_MATCHES(parse_model_transforms("ro:1:2:3:4:5"),
		CLIError, Message("Too many arguments for rotation."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sc:1:2"),
		CLIError, Message("Too many arguments for scaling."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:yz:1:2"),
		CLIError, Message("Too many arguments for skew."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("tr:1:2:3:4"),
		CLIError, Message("Too many arguments for translation."));

	/* Skew specific errors */
	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:yxz:1.2"),
		CLIError, Message("Invalid skew map."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:z:2.3"),
		CLIError, Message("Invalid skew map."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:xx:3.4"),
		CLIError, Message("Invalid skew map."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:ay:4.5"),
		CLIError, Message("Invalid skew domain."));

	REQUIRE_THROWS_MATCHES(parse_model_transforms("sk:yp:5.6"),
		CLIError, Message("Invalid skew range."));

	/* Unknown transformation */
	REQUIRE_THROWS_MATCHES(parse_model_transforms("un:1:2:3"),
		CLIError, Message("Unknown transformation: un"));
}
