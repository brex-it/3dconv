#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>

#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

using namespace linalg;
using namespace std;

Face::Face(const shared_ptr<Model> model)
	: model_{model}
{}

Face::Face(const shared_ptr<Model> model, const IndexVecT &vertices,
			const IndexVecT &texture_vertices,
			const IndexVecT &vertex_normals,
			const FVec<float, 3> &normal)
	: model_{model}, vertices_{vertices}, texture_vertices_{texture_vertices}
	, vertex_normals_{vertex_normals}, normal_{normal}
{
//	compute_normal();
//	validate();
}

inline shared_ptr<Model>
Face::get_model_shptr() const
{
	auto model_shptr = model_.lock();
	if (!model_shptr) {
		throw ModelError("The associated Model object has been expired.");
	}
	return model_shptr;
}

inline FVec<float, 3>
Face::compute_normal(Normalize normalize) const
{
	if (vertices_.size() < 3) {
		throw ModelError("Face must contain at least 3 vertices.");
	}

	auto model_shptr = get_model_shptr();
	const auto &v = model_shptr->vertices();
	FVec<float, 4> plane_vec1 = v[vertices_[1]] - v[vertices_[0]];
	FVec<float, 4> plane_vec2 = v[vertices_[2]] - v[vertices_[0]];

	return vec_slice<0, 3>(cross_product(plane_vec1,
		plane_vec2, normalize));
}

void
Face::validate() const
{
	auto vsz = vertices_.size();
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

namespace {

/* Struct derived from Model because Model's constructors are protected
 * and std::make_shared() needs an accessible constructor. */
struct ModelCreateHelper : public Model {
	ModelCreateHelper() = default;
	ModelCreateHelper(const Model &model) : Model{model} {}
};

} // namespace

shared_ptr<Model>
Model::create()
{
	return make_shared<ModelCreateHelper>();
}

shared_ptr<Model>
Model::create(const std::shared_ptr<const Model> other)
{
	auto model = make_shared<ModelCreateHelper>(*other);
	for (auto &f : model->faces_) {
		/* NOTE: const_cast because std::set
		 *       provides only const iterators */
		const_cast<Face &>(f).model_ = model;
	}
	return model;
}

void
Model::transform(const FMatSq<float, 4> &tmat)
{
	/* Apply tmat to every vertex */
	for (auto &v : vertices_) {
		v = tmat * v;
	}

	/* Apply tmat to every vertex normal */
	for (auto &vn : vertex_normals_) {
		vn = vec_slice<0, 3>(tmat * vec_homogenize(vn, 0.f));
	}
}

void
Model::triangulate()
{
	if (is_triangulated_) {
		return;
	}

	if (!is_validated_) {
		validate();
	}

	set<Face> new_faces;

	for (auto f = faces_.begin(); f != faces_.end();) {
		auto &fv = f->vertices();
		if (fv.size() >= static_cast<size_t>(numeric_limits<long>::max())) {
			ostringstream err_msg;
			err_msg << "Triangulation algorithm can only be run on faces "
				"with maximum " << numeric_limits<long>::max() << " vertices.";
			throw ModelError(err_msg.str());
		}
		long fvcnt = fv.size();
		if (fvcnt > 3) {
			Vec<long, 3> trind[2]{{0, 1, 2}, {-1, 0, 2}};
			Vec<long, 3> trind_step[2]{{-1, +1, +1}, {-1, -1, +1}};
			Vec<long, 3> fvcnt_vec{fvcnt, fvcnt, fvcnt};

			for (;;) {
				for (const size_t i : {0, 1}) {
					Vec<size_t, 3> ivec = (fvcnt_vec + trind[i]) % fvcnt;

					if (ivec[0] == ivec[2]) {
						goto nomoretriangles;
					}

					Face newf{shared_from_this()};
					newf.add_vertex(fv[ivec[0]]);
					newf.add_vertex(fv[ivec[1]]);
					newf.add_vertex(fv[ivec[2]]);
					newf.compute_normal();

					auto &ftv = f->texture_vertices();
					if (ftv.size() != 0) {
						newf.add_texture_vertex(ftv[ivec[0]]);
						newf.add_texture_vertex(ftv[ivec[1]]);
						newf.add_texture_vertex(ftv[ivec[2]]);
					}

					auto &fvn = f->vertex_normals();
					if (fvn.size() != 0) {
						newf.add_vertex_normal(fvn[ivec[0]]);
						newf.add_vertex_normal(fvn[ivec[1]]);
						newf.add_vertex_normal(fvn[ivec[2]]);
					}

					new_faces.insert(newf);

					trind[i] += trind_step[i];
				}
			}
		nomoretriangles:
			f = faces_.erase(f);
		} else {
			++f;
		}
	}
	faces_.merge(new_faces);
}

float
Model::surface_area()
{
	if (!is_validated_) {
		validate();
	}

	/* Copy if not triangulated to avoid modification
	 * of the original faces. */
	auto model = shared_from_this();
	if (!is_triangulated_) {
		model = Model::create(shared_from_this());
		model->triangulate();
	}

	float sum{0.f};
	for (const auto &f : model->faces_) {
		sum += .5f * euclidean_norm(f.compute_normal(Normalize::No));
	}
	return sum;
}

/* Algorithm from here: https://doi.org/10.1109/ICIP.2001.958278 */
float
Model::volume()
{
	if (!is_validated_) {
		validate();
	}

	/* Copy if not triangulated to avoid modification
	 * of the original faces. */
	auto model = shared_from_this();
	if (!is_triangulated_) {
		model = Model::create(shared_from_this());
		model->triangulate();
	}

	float sum{0.f};
	const auto &v = model->vertices_;
	for (const auto &f : model->faces_) {
		const auto &fv = f.vertices();
		sum += determinant(FMatSq<float, 3>{
				v[fv[0]][0], v[fv[1]][0], v[fv[2]][0],
				v[fv[0]][1], v[fv[1]][1], v[fv[2]][1],
				v[fv[0]][2], v[fv[1]][2], v[fv[2]][2]})
			/ 6;
	}
	return sum;
}

void
Model::validate()
{
	if (is_validated_) {
		return;
	}

	for (const auto &f : faces_) {
		try{
			f.validate();
		} catch (const exception &e) {
			ostringstream err_msg;
			err_msg << "(Face";
			for (const auto &v : f.vertices()) {
				err_msg << ":" << v;
			}
			err_msg << ") " << e.what();
			throw ModelError(err_msg.str());
		}
	}
	is_validated_ = true;
}
