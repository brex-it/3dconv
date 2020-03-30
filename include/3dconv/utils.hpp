#ifndef _3DCONV_UTILS_HPP
#define _3DCONV_UTILS_HPP

#include <type_traits>

/**
 * Enum for indicating endiannes in byte-to-word
 * and word-to-byte types of conversions.
 */
enum struct Endian : bool {
	Little,
	Big
};

/**
 * Takes an arbitrary unsigned integral multi-byte value
 * and copies it into the first nbytes bytes of the
 * destination byte array.
 */
template<Endian E, typename SRC,
	typename = std::enable_if_t<std::is_unsigned_v<SRC>>>
inline void
uint2bytes(SRC src, uint8_t *const dst, size_t nbytes)
{
	constexpr SRC mask = 0xff;
	size_t offset;

	for (size_t i = 0; i < nbytes; ++i) {
		if constexpr (E == Endian::Little) {
			offset = i * 8;
		} else {
			offset = (nbytes - i - 1) * 8;
		}
		dst[i] = (uint8_t)((src & (mask << offset)) >> offset);
	}
}

/**
 * Similar to the std::bitset container but its size is
 * set at construction time.
 */
class Bitset {
public:
	/** Takes the number of bits to be stored by the constructed instance. */
	Bitset(size_t n);
	/** Copy constructor. */
	Bitset(const Bitset &);
	/** Custom destructor for RAII. */
	~Bitset();

	/* Explicitly deleted copy and move operations */

	///
	Bitset(Bitset &&) = delete;
	///
	Bitset &operator=(const Bitset &) = delete;
	///
	Bitset &operator=(Bitset &&) = delete;

	/* Query methods */

	/** Returns the number of bits stored in the current object. */
	size_t bit_count() const;
	/** Returns true if all of the stored bits are set to true. */
	bool all() const;
	/** Returns true if any of the stored bits is set to true. */
	bool any() const;
	/** Returns true if none of the stored bits is set to true. */
	bool none() const;
	/** Returns the value of the bit stored at the position ind. */
	bool operator[](size_t ind) const;

	/* Per-bit operations */

	/** Set the value of the bit at the position ind to val. */
	Bitset &set(size_t ind, bool val = true);
	/** Flip the bit at the position ind. */
	Bitset &flip(size_t ind);

	/* Global bitwise operations */

	/** Reset all bit's value to false. */
	Bitset &reset();
	/** Bitwise or. */
	Bitset operator|(const Bitset &other) const;
	/** Bitwise or-assignment. */
	Bitset &operator|=(const Bitset &other);
	/** Bitwise and. */
	Bitset operator&(const Bitset &other) const;
	/** Bitwise and-assignment. */
	Bitset &operator&=(const Bitset &other);
	/** Bitwise xor. */
	Bitset operator^(const Bitset &other) const;
	/** Bitwise xor-assignment. */
	Bitset &operator^=(const Bitset &other);

private:
	/* Storage type dependent parameters */
	using WordT = uintmax_t;
	static constexpr size_t WordS = std::numeric_limits<WordT>::digits;
	static constexpr WordT full_mask_ = std::numeric_limits<WordT>::max();

	/* Size dependent parameters */
	const size_t bitcnt_;
	const size_t wordcnt_;
	const WordT lastw_mask_;

	WordT *words_;

	void check_index(const size_t ind) const;
	void compare_sizes(const size_t other_bitcnt) const;
};

#endif
