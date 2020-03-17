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

#endif
