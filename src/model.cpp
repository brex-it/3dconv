#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>
#include <3dconv/utils.hpp>

using namespace linalg;
using namespace std;

/* ======== Face implementation ======== */

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
	auto found = std::find(vertices_.cbegin(), vertices_.cend(), v);
	if (found == vertices_.cend()) {
		vertices_.push_back(v);
	}
}

void
Face::add_texture_vertex(const size_t tv) {
	auto found = std::find(texture_vertices_.cbegin(),
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
	auto ls = std::set<size_t>(this->vertices_.cbegin(),
			this->vertices_.cend());
	auto rs = std::set<size_t>(r.vertices_.cbegin(),
			r.vertices_.cend());

	vector<size_t> intersection;
	set_intersection(ls.cbegin(), ls.cend(), rs.cbegin(), rs.cend(),
		back_inserter(intersection));

	return ls < rs && intersection.size() < 3;
}

/* -------- Private methods -------- */

inline shared_ptr<const Model>
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


/* ======== Model implementation ======== */

/* -------- Constructors (static creators) -------- */

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
		f.model_ = model;
	}
	return model;
}

/* -------- Getters -------- */

const F32_4D_VecT &
Model::vertices() const
{
	return vertices_;
}

const F32_3D_VecT &
Model::texture_vertices() const
{
	return texture_vertices_;
}

const F32_3D_VecT &
Model::vertex_normals() const
{
	return vertex_normals_;
}

const set<Face> &
Model::faces() const
{
	return faces_;
}

/* -------- Modifiers -------- */

void
Model::add_vertex(const FVec<float, 4> &&v)
{
	vertices_.emplace_back(v);
	needs_recalc_properties();
}

void
Model::add_texture_vertex(const FVec<float, 3> &&tv)
{
	texture_vertices_.emplace_back(tv);
}

void
Model::add_vertex_normal(const FVec<float, 3> &&vn)
{
	vertex_normals_.emplace_back(vn);
}

void
Model::add_face(const Face &f)
{
	auto model_ptr = f.get_model_shptr().get();
	if (model_ptr != this) {
		throw ModelError("Faces can only be added to their "
				"associated Model.");
	}

	faces_.insert(f);

	if (f.vertices().size() > 3) {
		is_triangulated_ = false;
	}
	is_validated_ = false;
	needs_recalc_properties();
}

void
Model::convexify_faces()
{
	if (is_triangulated_) {
		return;
	}

	if (!is_validated_) {
		validate();
	}

	set<Face> new_faces;

	vector<size_t> inner_indices;
	vector<vector<size_t>> outer_indices;
	deque<Face> tmp_faces;
	set<array<size_t, 2>> visited_edges;

	for (auto f = faces_.begin(); f != faces_.end();) {
		/* Triangles are always convex */
		if (f->vertices().size() == 3) {
			++f;
			continue;
		} else {
			tmp_faces.emplace_back(*f);
			f = faces_.erase(f);
		}

		visited_edges.clear();
		while (!tmp_faces.empty()) {
			const auto &tmpf = tmp_faces.front();
			const auto &v = tmpf.vertices();
			const auto vsz = v.size();
			if (vsz == 3) {
				visited_edges.insert({v[0], v[1]});
				visited_edges.insert({v[1], v[2]});
				visited_edges.insert({v[2], v[0]});
				new_faces.insert(tmpf);
			} else {
				size_t i{0};
				for (; i < vsz; ++i) {
					const array<size_t, 2> e{v[i], v[(i + 1) % vsz]};
					if (visited_edges.find(e) != visited_edges.end()) {
						/* Already visited */
						continue;
					}

					inner_indices.clear();
					outer_indices.clear();

					const auto edge_vec_3d =
						vec_slice<0, 3>(vertices_[e[1]] - vertices_[e[0]]);
					const auto edge_vec_src_3d =
						vec_slice<0, 3>(vertices_[e[0]]);
					bool is_prev_inside{true};
					inner_indices.push_back(i);
					inner_indices.push_back((i + 1) % vsz);
					for (size_t j = 2; j < vsz; ++j) {
						const size_t ind = (i + j) % vsz;
						const bool is_inside = dot_product(
							cross_product(edge_vec_3d,
								vec_slice<0, 3>(vertices_[v[ind]])
									- edge_vec_src_3d),
							tmpf.normal()) > 0;

						if (is_inside) {
							inner_indices.push_back(ind);
							if (!is_prev_inside) {
								outer_indices.back().push_back(ind);
							}
						} else {
							if (is_prev_inside) {
								outer_indices.emplace_back();
								outer_indices.back()
									.push_back((vsz + ind - 1) % vsz);
							}
							outer_indices.back().push_back(ind);
						}

						is_prev_inside = is_inside;
					}
					if (!is_prev_inside) {
						outer_indices.back().push_back(i);
					}

					visited_edges.insert(e);

					if (!outer_indices.empty()) {
						/* The line of 'e' splits the face */
						break;
					}
				}
				if (i == vsz) {
					/* tmpf is convex */
					new_faces.insert(tmpf);
				} else {
					/* tmpf is concave, so slice it by the first-
					 * found concave edge */
					tmp_faces.emplace_back(tmpf, inner_indices);
					for (const auto &ind_vec : outer_indices) {
						tmp_faces.emplace_back(tmpf, ind_vec);
					}
				}
			}
			tmp_faces.pop_front();
		}
	}
	faces_.merge(new_faces);
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

	/* The algorithm below only works on convex faces,
	 * so ensure that all of them are convex */
	convexify_faces();

	set<Face> new_faces;

	for (auto f = faces_.begin(); f != faces_.end();) {
		const auto &fv = f->vertices();
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

					new_faces.emplace(*f,
						Face::IndexVecT{ivec[0], ivec[1], ivec[2]});

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
	is_triangulated_ = true;
}

/* -------- Queries, validation -------- */

float
Model::surface_area() const
{
	if (!is_validated_) {
		validate();
	}

	/* Copy if not triangulated to avoid modification
	 * of the original faces. */
	auto model = shared_from_this();
	if (!is_triangulated_) {
		auto model_cpy = Model::create(shared_from_this());
		model_cpy->triangulate();
		model = model_cpy;
	}

	float sum{0.f};
	for (const auto &f : model->faces_) {
		sum += .5f * euclidean_norm(f.compute_normal(Normalize::No));
	}
	return sum;
}

/* Algorithm from here: https://doi.org/10.1109/ICIP.2001.958278 */
float
Model::volume() const
{
	if (!is_validated_) {
		validate();
	}

	/* Copy if not triangulated to avoid modification
	 * of the original faces. */
	auto model = shared_from_this();
	if (!is_triangulated_) {
		auto model_cpy = Model::create(shared_from_this());
		model_cpy->triangulate();
		model = model_cpy;
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

bool
Model::is_triangulated() const
{
	return is_triangulated_;
}

bool
Model::is_connected() const
{
	if (!is_validated_) {
		validate();
	}
	if (recalc_connectivity_) {
		is_connected_ = check_connectivity(faces_, vertices_.size());
		recalc_connectivity_ = false;
	}
	return is_connected_;
}

bool
Model::is_convex() const
{
	if (!is_validated_) {
		validate();
	}
	if (recalc_convexity_) {
		is_convex_ = get_concave_vertices().size() == 0;
		recalc_convexity_ = false;
	}
	return is_convex_;
}

bool
Model::is_watertight() const
{
	string msg;
	return is_watertight(msg);
}

bool
Model::is_watertight(string &msg) const
{
	if (!is_validated_) {
		validate();
	}
	if (recalc_water_tightness_) {
		is_watertight_ = check_water_tightness(msg);
		recalc_water_tightness_ = false;
	}
	return is_watertight_;
}

void
Model::validate() const
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

/* -------- Private methods -------- */

inline void
Model::needs_recalc_properties()
{
	recalc_connectivity_ = true;
	recalc_convexity_ = true;
	recalc_water_tightness_ = true;
}

bool
Model::check_connectivity(const set<Face> &faces, const size_t nverts)
{
	if (faces.empty()) {
		return nverts ? false : true;
	}

	list<Bitset> connections;

	/* Create Bitsets from faces: every bit corresponding
	 * to vertex indices is set to true.  */
	Bitset bs{nverts};
	for (const auto &f : faces) {
		bs.reset();
		for (size_t v : f.vertices()) {
			bs.set(v);
		}
		connections.emplace_front(bs);
	}

	/* Walk through the edges in a BFS manner. */
	Bitset bs_union{connections.front()};
	connections.erase(connections.begin());
	/* NOTE: Maximum nconn iterations are needed to ensure
	 *       that no more connections are present. Without
	 *       this limit disconnected components would result
	 *       in infinite loop. */
	size_t nconn = connections.size();
	while (!connections.empty() && nconn--) {
		for (auto c = connections.cbegin(); c != connections.cend();) {
			if ((*c & bs_union).any()) {
				bs_union |= *c;
				c = connections.erase(c);
			} else {
				++c;
			}
		}
	}

	/* Return true if all vertices have been visited.
	 * NOTE: We also check if no faces remained in the connections
	 *       list, because we don't require faces to only contain
	 *       valid Face objects. (at least 3 vertices, no common
	 *       triangles or higher order shapes etc.) */
	return connections.empty() && bs_union.all();

}

/* Water tightness definition from here:
 * https://davidstutz.de/a-formal-definition-of-watertight-meshes/ */
bool
Model::check_water_tightness(string &msg) const
{
	map<array<size_t, 2>, size_t> edge_occurrences;
	ostringstream msg_stream;

	/* Check if every edge has exactly two incident faces */
	for (const auto &f : faces_) {
		const auto &v = f.vertices();
		const size_t vsz = v.size();
		for (size_t i = 0; i < vsz; ++i) {
			array<size_t, 2> edge{v[i], v[(i + 1) % vsz]};
			if (edge[0] > edge[1]) {
				swap(edge[0], edge[1]);
			}
			++edge_occurrences[edge];
		}
	}
	for (const auto &[e, o] : edge_occurrences) {
		if (o != 2) {
			msg_stream << "(Edge:" << e[0] << ":" << e[1] << ") "
				<< (o == 1 ? "Boundary edge" : "Non-manifold edge");
			msg = msg_stream.str();
			return false;
		}
	}

	/* Check if the model doesn't contain non-manifold vertices */
	for (size_t i = 0; i < vertices_.size(); ++i) {
		set<Face> face_group;
		size_t nverts{0};
		map<size_t, size_t> index_map;
		for (const auto &f : faces_) {
			vector<size_t> selected_verts;
			bool is_incident{false};
			for (size_t vi : f.vertices()) {
				if (vi == i) {
					is_incident = true;
				} else {
					selected_verts.push_back(vi);
				}
			}
			if (is_incident) {
				for (auto &vi : selected_verts) {
					if (index_map.find(vi) == index_map.end()) {
						index_map[vi] = nverts++;
					}
					vi = index_map[vi];
				}
				face_group.emplace(Face{shared_from_this(),
					selected_verts});
			}
		}
		if (!check_connectivity(face_group, nverts)) {
			msg_stream << "(Vertex:" << i << ") Non-manifold vertex";
			msg = msg_stream.str();
			return false;
		}
	}

	/* Check if there are no self intersections */
	auto conc_verts = get_concave_vertices();
	for (const auto &[f, ind_list] : conc_verts) {
		auto fvert = [&f, this](size_t i) {
			return vec_slice<0, 3>(vertices_[f.vertices()[i]]);
		};
		auto is_conc = [&ind_list](size_t vi) {
			for (const auto &i : ind_list) {
				if (vi == i) { return true; }
			}
			return false;
		};
		for (const auto &[e, o] : edge_occurrences) {
			/* The edge intersects the plane the current face lies on */
			if ((is_conc(e[0]) && !is_conc(e[1]))
					|| (!is_conc(e[0]) && is_conc(e[1]))) {
				/* Calculate the intersection point */
				const auto evert0 = vec_slice<0, 3>(vertices_[e[0]]);
				const auto evert1 = vec_slice<0, 3>(vertices_[e[1]]);
				const auto evec = evert1 - evert0;
				const auto intersection = evert0 + evec
					* (dot_product(fvert(0) - evert0, f.normal())
					/ dot_product(evec, f.normal()));

				/* Check whether the point is on the inside of the face */
				const size_t fvert_num = f.vertices().size();
				for (size_t i = 0; i < fvert_num; ++i) {
					const bool on_outer_side = dot_product(
						cross_product(fvert((i + 1) % fvert_num) - fvert(i),
							intersection - fvert(i)),
						f.normal()) <= 0; /*< EPSILON<float> ?*/
					if (on_outer_side) {
						goto nointersection;
					}
				}
				msg_stream << "(Face";
				for (const auto &v : f.vertices()) {
					msg_stream << ":" << v;
				}
				msg_stream << ") " << "Self intersection";
				msg = msg_stream.str();
				return false;
			nointersection:;
			}
		}
	}

	return true;
}

const typename Model::FaceToIndexVecMap
Model::get_concave_vertices() const
{
	if (!is_validated_) {
		validate();
	}

	FaceToIndexVecMap conc_verts;

	for (const auto &f : faces_) {
		for (size_t i = 0; i < vertices_.size(); ++i) {
			auto vec_to_point = vec_slice<0,3>(vertices_[i]
				- vertices_[f.vertices()[0]]);
			if (dot_product(vec_to_point, f.normal())
					> EPSILON<float>) {
				conc_verts[f].push_back(i);
			}
		}
	}

	return conc_verts;
}
