#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "catch.hpp"

#include <3dconv/io.hpp>

#include "test_tools.hpp"

using namespace std;

TEST_CASE("Convert OBJ file to STL-BIN") {
	filesystem::path out_file_path{"obj2stlbin.stl"};
	auto test_files_dir = get_test_files_dir_path() / "obj2stlbin_test";

	{
		auto parser = IOMap<Parser>::get("obj");
		parser->open(test_files_dir / "source.obj");
		auto model = (*parser)();

		auto writer = IOMap<Writer>::get("stl-bin");
		writer->open(out_file_path);
		(*writer)(model);
	}

	{
		/* Compare the result with the excpected file */
		ifstream write_result{out_file_path, ios::binary};
		ifstream expected{test_files_dir / "expected.stl", ios::binary};

		istream_iterator<uint8_t> wr_it{write_result};
		istream_iterator<uint8_t> ex_it{expected};
		istream_iterator<uint8_t> end_it;

		REQUIRE(equal(wr_it, end_it, ex_it, end_it));
	}

	filesystem::remove(out_file_path);
}
