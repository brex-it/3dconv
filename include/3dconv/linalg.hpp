#ifndef _3DCONV_LINALG_HPP
#define _3DCONV_LINALG_HPP

#include <array>
#include <cmath>
#include <type_traits>

namespace linalg {

/**
 * Constants for numerical computations.
 */
template<typename T>
constexpr T EPSILON = std::numeric_limits<T>::epsilon() * 100;

template<typename T>
constexpr T INF = std::numeric_limits<T>::infinity();

/**
 * Bounds-checked matrix index type.
 */
template<size_t DimCnt>
struct MatInd {
	std::array<size_t, DimCnt> i;
	size_t operator[](size_t d) const { return i.at(d); }
};

/**
 * General matrix struct template.
 */
template<typename CoordT, size_t Dim1, size_t Dim2,
	typename = std::enable_if_t<std::is_arithmetic_v<CoordT>>>
struct Mat {
	std::array<CoordT, Dim1 * Dim2> elements;

	CoordT &operator[](const MatInd<2> &ind) {
		return elements.at(ind[0] * Dim2 + ind[1]);
	}
	const CoordT &operator[](const MatInd<2> &ind) const {
		return elements.at(ind[0] * Dim2 + ind[1]);
	}

	auto begin() { return elements.begin(); }
	auto end() { return elements.end(); }
	auto begin() const { return elements.cbegin(); }
	auto end() const { return elements.cend(); }

	template<typename NewT>
	operator Mat<NewT, Dim1, Dim2>() const {
		Mat<NewT, Dim1, Dim2> nmat;
		std::copy(this->elements.cbegin(), this->elements.cend(),
				nmat.elements.begin());
		return nmat;
	}
};

/**
 * Type specialization for square matrices.
 */
template<typename CoordT, size_t Dim>
using MatSq = Mat<CoordT, Dim, Dim>;

/**
 * Type specialization for matrices with floating point elements.
 */
template<typename CoordT, size_t Dim1, size_t Dim2,
	typename = std::enable_if_t<std::is_floating_point_v<CoordT>>>
using FMat = Mat<CoordT, Dim1, Dim2>;

/**
 * Type specialization for square matrices
 * with floating point elements.
 */
template<typename CoordT, size_t Dim>
using FMatSq = FMat<CoordT, Dim, Dim>;


/**
 * Special matrix template struct for vector representation.
 */
template<typename CoordT, size_t Dim>
struct Vec : public Mat<CoordT, Dim, 1> {
	CoordT &operator[](const size_t ind) {
		return this->elements.at(ind);
	}
	const CoordT &operator[](const size_t ind) const {
		return this->elements.at(ind);
	}
	CoordT &operator[](const MatInd<2> &ind) {
		return Mat<CoordT, Dim, 1>::operator[](ind);
	}
	const CoordT &operator[](const MatInd<2> &ind) const {
		return Mat<CoordT, Dim, 1>::operator[](ind);
	}
	template<typename NewT>
	operator Vec<NewT, Dim>() const {
		Vec<NewT, Dim> nvec;
		std::copy(this->elements.cbegin(), this->elements.cend(),
				nvec.elements.begin());
		return nvec;
	}
};

/**
 * Type specialization for vectors with floating point elements.
 */
template<typename CoordT, size_t Dim,
	typename = std::enable_if_t<std::is_floating_point_v<CoordT>>>
using FVec = Vec<CoordT, Dim>;

/**
 * Returns a Vec object with dimension B - A filled
 * with the corresponding element of the original vector.
 */
template<size_t A, size_t B, typename CoordT, size_t Dim>
auto
vec_slice(const Vec<CoordT, Dim> &v)
{
	constexpr size_t newsz = B - A;
	Vec<CoordT, newsz> res;
	for (size_t i = 0; i < newsz; ++i) {
		res[i] = v[A + i];
	}
	return res;
}

/**
 * Convenience helper function for homogenization
 * of 3D vectors.
 */
template<typename CoordT>
auto
vec_homogenize(const Vec<CoordT, 3> &v, CoordT ext_val)
{
	return Vec<CoordT, 4>{{v[0], v[1], v[2], ext_val}};
}

/**
 * Returns a zero-filled Mat or Vec object depending
 * on the given second dimension.
 */
template<typename CoordT, size_t Dim1, size_t Dim2 = 1>
auto
make_zero_mat()
{
	std::conditional_t<Dim2 == 1,
		Vec<CoordT, Dim1>,
		Mat<CoordT, Dim1, Dim2>> z;

	for (auto &e : z) {
		e = (CoordT)0;
	}
	return z;
}

/**
 * Returns an identity matrix with the given dimension.
 */
template<typename CoordT, size_t Dim>
auto
make_id_mat()
{
	MatSq<CoordT, Dim> id;

	for (size_t i = 0; i < Dim; ++i) {
		for (size_t j = 0; j < Dim; ++j) {
			id[{i, j}] = (CoordT)(i == j ? 1 : 0);
		}
	}
	return id;
}

/**
 * Returns the transposed version of the given matrix.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto
transpose(const Mat<CoordT, Dim1, Dim2> &m)
{
	Mat<CoordT, Dim2, Dim1> res;
	for (size_t i = 0; i < Dim1; ++i) {
		for (size_t j = 0; i < Dim2; ++i) {
			res[{j, i}] = m[{i, j}];
		}
	}
	return res;
}

/* Operators for matrix objects */

/**
 * Matrix/vector equality operator.
 */
template<typename CoordT, size_t Dim1L, size_t Dim2L,
	size_t Dim1R, size_t Dim2R>
bool
operator==(const Mat<CoordT, Dim1L, Dim2L> &ml,
		const Mat<CoordT, Dim1R, Dim2R> &mr)
{
	if constexpr (Dim1L != Dim1R || Dim2L != Dim2R) {
		return false;
	} else {
		return ml.elements == mr.elements;
	}
}

/**
 * Matrix/vector multiplication operator.
 */
template<typename CoordT, size_t Dim1L, size_t DimComm, size_t Dim2R>
auto
operator*(const Mat<CoordT, Dim1L, DimComm> &ml,
		const Mat<CoordT, DimComm, Dim2R> &mr)
{
	auto res = make_zero_mat<CoordT, Dim1L, Dim2R>();

	for (size_t i = 0; i < Dim1L; ++i) {
		for (size_t j = 0; j < DimComm; ++j) {
			for (size_t k = 0; k < Dim2R; ++k) {
				res[{i, k}] += ml[{i, j}] * mr[{j, k}];
			}
		}
	}

	return res;
}

/**
 * Matrix/vector multiply-assign operator.
 */
template<typename CoordT, size_t Dim1L, size_t DimComm>
auto &
operator*=(Mat<CoordT, Dim1L, DimComm> &ml,
		const MatSq<CoordT, DimComm> &mr)
{
	ml = ml * mr;
	return ml;
}

/**
 * Matrix/vector addition operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto
operator+(const Mat<CoordT, Dim1, Dim2> &ml,
		const Mat<CoordT, Dim1, Dim2> &mr)
{
	std::conditional_t<Dim2 == 1,
		Vec<CoordT, Dim1>,
		Mat<CoordT, Dim1, Dim2>> res{ml.elements};

	for (size_t i = 0; i < ml.elements.size(); ++i) {
		res.elements[i] += mr.elements[i];
	}
	return res;
}

/**
 * Matrix/vector add-assign operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto &
operator+=(Mat<CoordT, Dim1, Dim2> &ml,
		const Mat<CoordT, Dim1, Dim2> &mr)
{
	ml = ml + mr;
	return ml;
}

/**
 * Matrix/vector subtraction operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto
operator-(const Mat<CoordT, Dim1, Dim2> &ml,
		const Mat<CoordT, Dim1, Dim2> &mr)
{
	std::conditional_t<Dim2 == 1,
		Vec<CoordT, Dim1>,
		Mat<CoordT, Dim1, Dim2>> res{ml.elements};

	for (size_t i = 0; i < ml.elements.size(); ++i) {
		res.elements[i] -= mr.elements[i];
	}
	return res;
}

/**
 * Matrix/vector division operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto
operator/(const Mat<CoordT, Dim1, Dim2> &ml,
		const CoordT n)
{
	std::conditional_t<Dim2 == 1,
		Vec<CoordT, Dim1>,
		Mat<CoordT, Dim1, Dim2>> res{ml.elements};

	for (size_t i = 0; i < res.elements.size(); ++i) {
		res.elements[i] /= n;
	}
	return res;
}

/**
 * Matrix/vector divide-assign operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto &
operator/=(Mat<CoordT, Dim1, Dim2> &ml,
		const CoordT n)
{
	ml = ml / n;
	return ml;
}

/**
 * Matrix/vector remainder operator.
 */
template<typename CoordT, size_t Dim1, size_t Dim2>
auto
operator%(const Mat<CoordT, Dim1, Dim2> &ml,
		const CoordT n)
{
	std::conditional_t<Dim2 == 1,
		Vec<CoordT, Dim1>,
		Mat<CoordT, Dim1, Dim2>> res{ml.elements};

	for (size_t i = 0; i < res.elements.size(); ++i) {
		res.elements[i] %= n;
	}
	return res;
}

/* Algorithms */

/**
 * Used as a flag in algorithms which optionally
 * can return normalized results.
 */
enum struct Normalize {
	Yes,
	No
};

/**
 * Returns the cross product of two 3D or 4D
 * (homogeneous) vectors.
 * Also can normalize its result.
 */
template<typename CoordT, size_t Dim,
	typename = std::enable_if_t<Dim == 3 || Dim == 4>>
auto
cross_product(const FVec<CoordT, Dim> &lv,
		const FVec<CoordT, Dim> &rv, Normalize normalize = Normalize::No)
{
	FVec<CoordT, Dim> res;

	res[0] = lv[1] * rv[2] - lv[2] * rv[1];
	res[1] = lv[2] * rv[0] - lv[0] * rv[2];
	res[2] = lv[0] * rv[1] - lv[1] * rv[0];

	if (normalize == Normalize::Yes) {
		CoordT l = std::sqrt(res[0] * res[0] + res[1] * res[1]
				+ res[2] * res[2]);

		res[0] /= l;
		res[1] /= l;
		res[2] /= l;
	}

	if constexpr (Dim == 4) {
		res[3] = (CoordT)0;
	}

	return res;
}

/**
 * Dot product of two n-dimensional vectors.
 */
template<typename CoordT, size_t Dim>
CoordT
dot_product(const FVec<CoordT, Dim> &lv,
		const FVec<CoordT, Dim> &rv)
{
	CoordT res = 0;
	for (size_t i = 0; i < Dim; ++i) {
		res += lv[i] * rv[i];
	}
	return res;
}

/**
 * Euclidean norm of an n-dimensional vector.
 */
template<typename CoordT, size_t Dim>
CoordT
euclidean_norm(const FVec<CoordT, Dim> &v)
{
	CoordT ssum = (CoordT)0;
	for (const auto &e : v) {
		ssum += e * e;
	}
	return std::sqrt(ssum);
}

//template<typename CoordT, size_t Dim>
//CoordT
//determinant(const Mat<CoordT, Dim, Dim> &m)
//{
//	// TODO: Implement the general determinant calculation
//}

/**
 * Determinant specialization for 2x2 matrices.
 */
template<typename CoordT>
CoordT
determinant(const Mat<CoordT, 2, 2> &m)
{
	return m[{0, 0}] * m[{1, 1}] - m[{1, 0}] * m[{0, 1}];
}

/**
 * Determinant specialization for 3x3 matrices.
 */
template<typename CoordT>
CoordT
determinant(const Mat<CoordT, 3, 3> &m)
{
	return m[{0, 0}] * (m[{1, 1}] * m[{2, 2}] - m[{1, 2}] * m[{2, 1}])
		- m[{0, 1}] * (m[{1, 0}] * m[{2, 2}] - m[{2, 0}] * m[{1, 2}])
		+ m[{0, 2}] * (m[{1, 0}] * m[{2, 1}] - m[{2, 0}] * m[{1, 1}]);
}

template<typename CoordT, size_t Dim>
auto
make_rotation_mat(FVec<CoordT, Dim> axis, CoordT angle)
{
	return make_id_mat<CoordT, Dim + 1>();
}

/**
 * Constructs a 4x4 3D rotation matrix (for use
 * with homogeneous coordinates). Algorithm is
 * from <a href="https://doi.org/10.1002/qua.560320310">
 * this book</a>.
 */
template<typename CoordT>
auto
make_rotation_mat(FVec<CoordT, 3> axis, CoordT angle)
{
	auto mat = make_id_mat<CoordT, 4>();
	axis /= euclidean_norm(axis);

	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			mat[{i, j}] = axis[i] * axis[j] * (1 - std::cos(angle));
			if (i == j) {
				mat[{i, j}] += std::cos(angle);
			} else {
				mat[{i, j}] += ((i + 1) % 3 == j ? -1 : +1)
					* axis[((i + j) * 2 % 3)] * std::sin(angle);
			}
		}
	}

	return mat;
}

/**
 * Constructs a 4x4 3D translation matrix (for use
 * with homogeneous coordinates).
 */
template<typename CoordT, size_t Dim>
auto
make_translation_mat(FVec<CoordT, Dim> tr_vector)
{
	auto mat = make_id_mat<CoordT, Dim + 1>();
	for (size_t i = 0; i < Dim; ++i) {
		mat[{i, Dim}] = tr_vector[i];
	}
	return mat;
}

/**
 * Constructs a 4x4 3D scaling matrix (for use
 * with homogeneous coordinates).
 */
template<typename CoordT, size_t Dim>
auto
make_scaling_mat(CoordT factor)
{
	auto mat = make_id_mat<CoordT, Dim + 1>();
	for (size_t i = 0; i < Dim; ++i) {
		mat[{i, i}] = factor;
	}
	return mat;
}

} // namespace linalg

#endif
