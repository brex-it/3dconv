#include <cmath>
#include <filesystem>
#include <system_error>

#include <3dconv/linalg.hpp>

using namespace linalg;

namespace fs = std::filesystem;

inline auto
get_test_files_dir_path()
{
	auto test_files_dir_ptr = getenv("TEST_FILES_DIR");
	if (test_files_dir_ptr == nullptr) {
		throw fs::filesystem_error("Test files directory is not reachable.",
			std::error_code{});
	}
	return fs::path{test_files_dir_ptr};
}

template<typename CoordT, size_t Dim1, size_t Dim2>
inline bool
matrices_approx_equal(const Mat<CoordT, Dim1, Dim2> &m1,
		const Mat<CoordT, Dim1, Dim2> &m2)
{
	for (size_t i = 0; i < Dim1 * Dim2; ++i) {
		if (abs(m1.elements[i] - m2.elements[i]) > EPSILON<CoordT>) {
			return false;
		}
	}
	return true;
}
