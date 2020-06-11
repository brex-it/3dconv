#include <array>
#include <deque>
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
Model::create(const shared_ptr<const Model> other)
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
							tmpf.normal()) >= 0;

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
