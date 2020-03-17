#include <iostream>

#include "catch.hpp"

#include <3dconv/linalg.hpp>

#include "test_tools.hpp"

using namespace linalg;
using namespace std;

TEST_CASE("Vec [] operator test", "[vec]") {
	Vec<long, 4> v{5, 3, 1, 5};
	REQUIRE(v[0] == 5);
	REQUIRE(v[1] == 3);
	REQUIRE(v[2] == 1);
	REQUIRE(v[3] == 5);
}

TEST_CASE("Vector slice test", "[vec]") {
	auto v = Vec<int, 6>{1, 2, 3, 4, 5, 6};
	auto s = vec_slice<2, 5>(v);
	auto exp = Vec<int, 3>{3, 4, 5};
	REQUIRE(matrices_approx_equal(s, exp));
}

TEST_CASE("Cross product", "[cross]") {
	/* 3D vectors */
	FVec<float, 3> v1{3.6f, 7.3f, 2.f};
	FVec<float, 3> v2{4.1f, 7.5f, 9.f};

	auto cr1 = cross_product(v1, v2);
	auto cr1n = cr1 / euclidean_norm(cr1);

	REQUIRE(cr1n[0] == Approx(.90124f));
	REQUIRE(cr1n[1] == Approx(-.430178f));
	REQUIRE(cr1n[2] == Approx(-.0520835f));

	/* 4D vectors (homogeneous coordinates) */
	FVec<float, 4> v3{-5.5f, 1.04f, 1.9f, 1.f};
	FVec<float, 4> v4{2.4f, -5.f, 3.02f, 1.f};

	auto cr2n = cross_product(v3, v4, Normalize::Yes);

	REQUIRE(cr2n[0] == Approx(.359969f));
	REQUIRE(cr2n[1] == Approx(.602853f));
	REQUIRE(cr2n[2] == Approx(.712033f));
	REQUIRE(cr2n[3] == 0.f);
}

TEST_CASE("Determinant computation", "[determinant]") {
	Mat<int, 2, 2> int_sq_mat{{2,5,-2,4}};
	REQUIRE(determinant(int_sq_mat) == 18);

	FMatSq<float, 3> float_sq_mat{4.f,7.1,3.1, 2,2.3,6, 4.78,0,1};
	REQUIRE(determinant(float_sq_mat) == Approx(164.5466));

	FMatSq<double, 3> double_sq_mat{4.f,7.1,-3.1, 2,2.3,6, 4.78,0,1};
	REQUIRE(determinant(double_sq_mat) == Approx(232.7094));
}
