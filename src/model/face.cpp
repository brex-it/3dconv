#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <set>
#include <vector>

#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

using namespace linalg;
using namespace std;

/* -------- Constructors -------- */

Face::Face(const shared_ptr<const Model> model)
	: model_{model}
{}

Face::Face(const shared_ptr<const Model> model, const IndexVecT &vertices,
			const IndexVecT &texture_vertices,
			const IndexVecT &vertex_normals,
			const FVec<float, 3> &normal)
	: model_{model}, vertices_{vertices}, texture_vertices_{texture_vertices}
	, vertex_normals_{vertex_normals}, normal_{normal}
{
//	compute_normal();
//	validate();
}

Face::Face(const Face &orig_face, const IndexVecT &indices)
	: model_{orig_face.model_}
{
	for (auto i : indices) {
		vertices_.push_back(orig_face.vertices_[i]);
	}
	if (!orig_face.texture_vertices_.empty()) {
		for (auto i : indices) {
			texture_vertices_.push_back(orig_face.texture_vertices_[i]);
		}
	}
	if (!orig_face.vertex_normals_.empty()) {
		for (auto i : indices) {
			vertex_normals_.push_back(orig_face.vertex_normals_[i]);
		}
	}
}

/* -------- Getters -------- */

const Face::IndexVecT &
Face::vertices() const
{
	return vertices_;
}

const Face::IndexVecT &
Face::texture_vertices() const
{
	return texture_vertices_;
}

const Face::IndexVecT &
Face::vertex_normals() const
{
	return vertex_normals_;
}

const FVec<float, 3> &
Face::normal() const
{
	if (!is_normal_set_) {
		normal_ = compute_normal();
		is_normal_set_ = true;
	}
	return normal_;
}

/* -------- Modifiers -------- */

void
Face::add_vertex(const size_t v)
{
	auto found = find(vertices_.cbegin(), vertices_.cend(), v);
	if (found == vertices_.cend()) {
		vertices_.push_back(v);
	}
}

void
Face::add_texture_vertex(const size_t tv) {
	auto found = find(texture_vertices_.cbegin(),
			texture_vertices_.cend(), tv);
	if (found == texture_vertices_.cend()) {
		texture_vertices_.push_back(tv);
	}
}

void
Face::add_vertex_normal(const size_t vn) {
		vertex_normals_.push_back(vn);
}

void
Face::set_normal(const FVec<float, 3> &n) {
	normal_ = n;
	is_normal_set_ = true;
}

/* -------- Operators -------- */

bool
Face::operator<(const Face &r) const {
	auto ls = set<size_t>(this->vertices_.cbegin(),
			this->vertices_.cend());
	auto rs = set<size_t>(r.vertices_.cbegin(),
			r.vertices_.cend());

	vector<size_t> intersection;
	set_intersection(ls.cbegin(), ls.cend(), rs.cbegin(), rs.cend(),
		back_inserter(intersection));

	return ls < rs && intersection.size() < 3;
}

/* -------- Private methods -------- */

shared_ptr<const Model>
Face::get_model_shptr() const
{
	auto model_shptr = model_.lock();
	if (!model_shptr) {
		throw ModelError("The associated Model object has been expired.");
	}
	return model_shptr;
}

FVec<float, 3>
Face::compute_normal(Normalize normalize) const
{
	const auto &vsz = vertices_.size();
	if (vsz < 3) {
		throw ModelError("Face must contain at least 3 vertices.");
	}

	auto model_shptr = get_model_shptr();
	const auto &mv = model_shptr->vertices();

	auto v0 = mv[vertices_[0]];
	auto v1 = mv[vertices_[1]];
	auto v2 = mv[vertices_[2]];
	auto normal = cross_product(v1 - v0, v2 - v0, normalize);

	/* This branch handles non-convex faces, where determining the
	 * correct winding direction is trickier than in the convex case*/
	if (vsz > 3) {
		float distance{0.f};
		size_t i0{0}, i1{1}, i2{2};
		/* Select the indices (in Face's vertex list) of the two
		 * vertices with the largest distance between them
		 * FIXME: Use an asymptotically better algorithm, if possible */
		for (size_t i = 0; i < vsz; ++i) {
			for (size_t j = i + 1; j < vsz; ++j) {
				float new_dist = euclidean_norm(mv[vertices_[i]]
					- mv[vertices_[j]]);
				if (new_dist > distance) {
					distance = new_dist;
					i0 = i;
					i1 = j;
				}
			}
		}

		/* Select the third index belonging to the furthest vertex
		 * from the previously selected line (v0<->v1) */
		v0 = mv[vertices_[i0]];
		v1 = mv[vertices_[i1]];
		auto line_normal = cross_product(normal, v1 - v0, Normalize::Yes);
		distance = 0.f;
		for (size_t i = 0; i < vsz; ++i) {
			float new_dist = abs(dot_product(mv[vertices_[i]] - v0,
				line_normal));
			if (new_dist > distance) {
				distance = new_dist;
				i2 = i;
			}
		}

		/* Sort selected indices */
		size_t tmp;
		if (i0 > i1) { tmp = i0; i0 = i1; i1 = tmp; }
		if (i1 > i2) { tmp = i1; i1 = i2; i2 = tmp; }
		if (i0 > i1) { tmp = i0; i0 = i1; i1 = tmp; }

		/* Recompute the valid normal */
		v0 = mv[vertices_[i0]];
		v1 = mv[vertices_[i1]];
		v2 = mv[vertices_[i2]];
		normal = cross_product(v1 - v0, v2 - v0, normalize);
	}

	return vec_slice<0, 3>(normal);
}

void
Face::validate() const
{
	const auto vsz = vertices_.size();
	if (vsz < 3) {
		throw ModelError("Face must contain at least 3 vertices.");
	}
	if (texture_vertices_.size() != 0
			&& texture_vertices_.size() != vertices_.size()) {
		throw ModelError("Face must either contain no texture "
			"vertices or the same number of texture vertices as "
			"geometric vertices.");
	}
	if (vertex_normals_.size() != 0
			&& vertex_normals_.size() != vertices_.size()) {
		throw ModelError("Face must either contain no vertex "
			"normals or the same number of vertex normals as "
			"geometric vertices.");
	}

	auto model_shptr = get_model_shptr();
	for (const auto &v : vertices_) {
		if (v >= model_shptr->vertices().size()) {
			throw ModelError("Invalid vertex index.");
		}
	}
	for (const auto &tv : texture_vertices_) {
		if (tv >= model_shptr->texture_vertices().size()) {
			throw ModelError("Invalid texture vertex index.");
		}
	}
	for (const auto &vn : vertex_normals_) {
		if (vn >= model_shptr->vertex_normals().size()) {
			throw ModelError("Invalid vertex normal index.");
		}
	}

	/* Checking co-planarity */
//	if (vsz > 3) {
//		const auto normvec = compute_normal(Normalize::No);
//		const auto &mv = model_shptr->vertices();
//		for (size_t i = 3; i < vsz; ++i) {
//			const auto plane_vec =
//				vec_slice<0, 3>(mv[vertices_[i]] - mv[vertices_[0]]);
//			if (fabs(dot_product(normvec, plane_vec) / euclidean_norm(normvec))
//					> EPSILON<float>) {
//				throw ModelError("The given points are not co-planar.");
//			}
//		}
//	}
}
