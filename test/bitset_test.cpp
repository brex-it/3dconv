#include <stdexcept>

#include "catch.hpp"

#include <3dconv/utils.hpp>

using namespace std;

using Catch::Matchers::Message;

TEST_CASE("Test [] and per-bit operations", "[bitset]") {
	Bitset bs{150};

	REQUIRE_THROWS_AS(bs[152], out_of_range);

	REQUIRE(bs[100] == false);
	bs.set(100);
	REQUIRE(bs[100] == true);
	bs.set(100, false);
	REQUIRE(bs[100] == false);

	REQUIRE(bs[88] == false);
	bs.flip(88);
	REQUIRE(bs[88] == true);
	bs.flip(88);
	REQUIRE(bs[88] == false);
}

TEST_CASE("Test bitwise operators", "[bitset]") {
	Bitset bs1{105};
	bs1.set(3);
	bs1.set(93);

	Bitset bs2{105};
	bs2.flip(0);
	bs2.flip(2);
	bs2.flip(3);
	bs2.flip(93);
	bs2.flip(94);

	/* OR */
	auto bs_or = bs1 | bs2;
	REQUIRE(bs_or[0] == true);
	REQUIRE(bs_or[1] == false);
	REQUIRE(bs_or[2] == true);
	REQUIRE(bs_or[3] == true);
	REQUIRE(bs_or[4] == false);
	REQUIRE(bs_or[93] == true);
	REQUIRE(bs_or[94] == true);

	/* AND */
	auto bs_and = bs1 & bs2;
	REQUIRE(bs_and[0] == false);
	REQUIRE(bs_and[1] == false);
	REQUIRE(bs_and[2] == false);
	REQUIRE(bs_and[3] == true);
	REQUIRE(bs_and[4] == false);
	REQUIRE(bs_and[93] == true);
	REQUIRE(bs_and[94] == false);

	/* XOR */
	auto bs_xor = bs1 ^ bs2;
	REQUIRE(bs_xor[0] == true);
	REQUIRE(bs_xor[1] == false);
	REQUIRE(bs_xor[2] == true);
	REQUIRE(bs_xor[3] == false);
	REQUIRE(bs_xor[4] == false);
	REQUIRE(bs_xor[93] == false);
	REQUIRE(bs_xor[94] == true);
}

TEST_CASE("Test global operations and queries", "[bitset]") {
	constexpr auto BIT_CNT = 23;
	Bitset bs{BIT_CNT};

	REQUIRE(!bs.all());
	REQUIRE(!bs.any());
	REQUIRE(bs.none());

	bs.set(5);
	bs.set(14);
	bs.set(15);
	bs.set(21);

	REQUIRE(!bs.all());
	REQUIRE(bs.any());
	REQUIRE(!bs.none());

	/* Maybe implement global set instead of this */
	for (size_t i = 0; i < BIT_CNT; ++i) {
		bs.set(i);
	}

	REQUIRE(bs.all());
	REQUIRE(bs.any());
	REQUIRE(!bs.none());

	bs.reset();

	REQUIRE(!bs.all());
	REQUIRE(!bs.any());
	REQUIRE(bs.none());
}
