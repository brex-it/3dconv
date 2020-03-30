#include <sstream>

#include <3dconv/utils.hpp>

using namespace std;

Bitset::Bitset(size_t n)
	: bitcnt_{n}
	, wordcnt_{n % WordS == 0 ? n / WordS : n / WordS + 1}
	, lastw_mask_{(WordT)((1 << (n % WordS)) - 1)}
{
	words_ = new WordT[wordcnt_]{};
}

Bitset::Bitset(const Bitset &other)
	: bitcnt_{other.bitcnt_}
	, wordcnt_{other.wordcnt_}
	, lastw_mask_{other.lastw_mask_}
{
	words_ = new WordT[wordcnt_];
	for (size_t i = 0; i < wordcnt_; ++i) {
		words_[i] = other.words_[i];
	}
}

Bitset::~Bitset()
{
	delete [] words_;
}

size_t
Bitset::bit_count() const
{
	return bitcnt_;
}

bool
Bitset::all() const
{
	const size_t lastw_index = wordcnt_ - 1;

	for (size_t i = 0; i < lastw_index; ++i) {
		if (words_[i] != full_mask_) {
			return false;
		}
	}

	if (words_[lastw_index] != lastw_mask_) {
		return false;
	}

	return true;
}

bool
Bitset::any() const
{
	for (size_t i = 0; i < wordcnt_; ++i) {
		if (words_[i]) {
			return true;
		}
	}
	return false;
}

bool
Bitset::none() const
{
	for (size_t i = 0; i < wordcnt_; ++i) {
		if (words_[i]) {
			return false;
		}
	}
	return true;
}

inline void
Bitset::check_index(const size_t ind) const
{
	if (ind >= bitcnt_) {
		ostringstream err_msg;
		err_msg << "Bitset index must be in the range [0, " << bitcnt_ << ").";
		throw std::out_of_range(err_msg.str());
	}
}

bool
Bitset::operator[](size_t ind) const
{
	check_index(ind);
	const WordT BIT_MASK = 1 << (ind % WordS);
	return words_[ind / WordS] & BIT_MASK;
}

Bitset &
Bitset::set(size_t ind, bool val)
{
	/* Implicit index checking through [] and flip. */
	if ((*this)[ind] != val) {
		flip(ind);
	}
	return *this;
}

Bitset &
Bitset::flip(size_t ind)
{
	check_index(ind);
	const WordT BIT_MASK = 1 << (ind % WordS);
	words_[ind / WordS] ^= BIT_MASK;
	return *this;
}

Bitset &
Bitset::reset()
{
	for (size_t i = 0; i < wordcnt_; ++i) {
		words_[i] = 0;
	}
	return *this;
}

inline void
Bitset::compare_sizes(const size_t other_bitcnt) const
{
	if (bitcnt_ != other_bitcnt) {
		throw std::logic_error("The two Bitset must have the same size.");
	}
}

/* NOTE: The following functions are heavily boilerplated but the tested
 *       template solutions generated more inefficient code. */

Bitset
Bitset::operator|(const Bitset &other) const
{
	compare_sizes(other.bitcnt_);
	auto res = Bitset(bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		res.words_[i] = words_[i] | other.words_[i];
	}
	return res;
}

Bitset &
Bitset::operator|=(const Bitset &other)
{
	compare_sizes(other.bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		words_[i] |= other.words_[i];
	}
	return *this;
}

Bitset
Bitset::operator&(const Bitset &other) const
{
	compare_sizes(other.bitcnt_);
	auto res = Bitset(bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		res.words_[i] = words_[i] & other.words_[i];
	}
	return res;
}

Bitset &
Bitset::operator&=(const Bitset &other)
{
	compare_sizes(other.bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		words_[i] &= other.words_[i];
	}
	return *this;
}

Bitset
Bitset::operator^(const Bitset &other) const
{
	compare_sizes(other.bitcnt_);
	auto res = Bitset(bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		res.words_[i] = words_[i] ^ other.words_[i];
	}
	return res;
}

Bitset &
Bitset::operator^=(const Bitset &other)
{
	compare_sizes(other.bitcnt_);
	for (size_t i = 0; i < wordcnt_; ++i) {
		words_[i] ^= other.words_[i];
	}
	return *this;
}
