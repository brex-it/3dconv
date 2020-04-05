#ifndef _3DCONV_MODEL_HPP
#define _3DCONV_MODEL_HPP

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <3dconv/linalg.hpp>

using namespace linalg;

/**
 * Exception thrown by Model realted algorithms.
 */
struct ModelError : public std::logic_error {
	ModelError(const std::string &what_arg)
		: std::logic_error{what_arg} {}
};

/**
 * Safer (always bounds-checked) and shorter-named alias
 * for std::vector of 3D linalg::FVec vectors.
 */
struct F32_3D_VecT : public std::vector<FVec<float, 3>> {
	reference operator[](size_type i) { return this->at(i); }
	const_reference operator[](size_type i) const { return this->at(i); }
};

/**
 * Safer (always bounds-checked) and shorter-named alias
 * for std::vector of 4D linalg::FVec vectors.
 */
struct F32_4D_VecT : public std::vector<FVec<float, 4>> {
	reference operator[](size_type i) { return this->at(i); }
	const_reference operator[](size_type i) const { return this->at(i); }
};

class Model;

/**
 * The basic building block of a 3D mesh model.
 * Contains an ordered list of vertices. This order determines the directon
 * of the normal vector (cw = away from viewer, ccw = towards viewer).
 * Optionally, a Face object contains texture vertices and vertex normals
 * for every geometric vertex and either a predefined or an on-demand
 * calculated normal vector.
 *
 * Face objects can only be instantiated with a pre-existing associated Model
 * object because they store only the indices of the actual vertices and
 * vectors which in turn are stored in model-global arrays (std::vectors).
 */
class Face {
	friend class Model;

public:
	using IndexVecT = std::vector<size_t>;

	/** Creates an empty Face object with a std::shared_ptr
	 * to its associated Model object. */
	Face(const std::shared_ptr<const Model> model);

	/** Creates a Face object with pre-filled index vectors. */
	Face(const std::shared_ptr<const Model> model, const IndexVecT &vertices,
			const IndexVecT &texture_vertices = {},
			const IndexVecT &vertex_normals = {},
			const FVec<float, 3> &normal = {});

	/** Getter for index vector of vertices. */
	const IndexVecT &vertices() const;
	/** Getter for index vector of texture vertices. */
	const IndexVecT &texture_vertices() const;
	/** Getter for index vector of vertex normals. */
	const IndexVecT &vertex_normals() const;
	/** Getter for face normal vector. */
	const FVec<float, 3> &normal() const;

	/** Adds a new vertex index.
	 * At the time of model validation (see Model::validate()) the index
	 * must be in the index range of the associated Model's vertices vector.
	 *
	 * The given index must be unique in the actual object. If it's not
	 * unique, no index will be added. */
	void add_vertex(const size_t v);

	/** Adds a new texture vertex index.
	 * At the time of model validation (see Model::validate()) the index
	 * must be in the index range of the associated Model's
	 * texture_vertices vector.
	 *
	 * The given index must be unique in the actual object. If it's not
	 * unique, no index will be added. */
	void add_texture_vertex(const size_t tv);

	/** Adds a new vertex normal index.
	 * At the time of model validation (see Model::validate()) the index
	 * must be in the index range of the associated Model's
	 * vertex_normals vector. */
	void add_vertex_normal(const size_t vn);

	/** Sets a predefined face normal.
	 * If none is set, normal vector will be calculated on demand. */
	void set_normal(const FVec<float, 3> &n);

	/** Less-than operator for using the class as key value
	 * in ordered containers. */
	bool operator<(const Face &r) const;

private:
	mutable std::weak_ptr<const Model> model_;

	IndexVecT vertices_;
	IndexVecT texture_vertices_;
	IndexVecT vertex_normals_;

	mutable FVec<float, 3> normal_;
	mutable bool is_normal_set_{false};

	void validate() const;
	FVec<float, 3> compute_normal(Normalize normalize = Normalize::Yes) const;

	std::shared_ptr<const Model> get_model_shptr() const;
};

/**
 * Class representing the whole 3D model.
 * Contains global collections of geometric vertices, texture vertices,
 * vertex normals, faces etc. It is possible to perform several
 * transformations on it, such as triangulation and affine transformations.
 *
 * No public constructor or assignment operator is available. For object
 * creation use Model::create() functions.
 */
class Model : public std::enable_shared_from_this<Model> {
public:
	/** Creates a new Model instance and returns an std::shared_ptr to it. */
	static std::shared_ptr<Model> create();
	/** Creates a new Model instance from an another Model object
	 * and returns an std::shared_ptr to it. */
	static std::shared_ptr<Model> create(
		const std::shared_ptr<const Model> other);

	/** Getter for vertices vector. */
	const F32_4D_VecT &vertices() const;
	/** Getter for texture vertices vector. */
	const F32_3D_VecT  &texture_vertices() const;
	/** Getter for vertex normals vector. */
	const F32_3D_VecT &vertex_normals() const;
	/** Getter for the set containing all faces. */
	const std::set<Face> &faces() const;

	/** Adds a new geometric vertex of the type linalg::FVec<float, 4>
	 * to the global vertex vector. The fourth coordinate is an
	 * additional homogeneous coordinate and if it is not used for
	 * specific calculations it should be set to some positive value. */
	void add_vertex(const FVec<float, 4> &&v);
	/** Adds a new texture vertex of the type linalg::FVec<float, 3>
	 * to the global texture vertex vector. */
	void add_texture_vertex(const FVec<float, 3> &&tv);
	/** Adds a new vertex normal of the type linalg::FVec<float, 3>
	 * to the global vertex normal vector. */
	void add_vertex_normal(const FVec<float, 3> &&vn);
	/** Adds a new Face object to the set of faces. Only faces associated
	 * with the actual Model instances will be accepted. */
	void add_face(const Face &f);

	/** Accepts a four-dimensional transformation matrix and performs
	 * the transformation on the model. The matrix should operate on
	 * 3D objects with a fourth homogeneous coordinate. */
	void transform(const FMatSq<float, 4> &tmat);
	/** Makes all faces triangular by dividing faces with more than
	 * three vertices. */
	void triangulate();

	/** Calculates and returns the surface area of the entire model. */
	float surface_area() const;
	/** Calculates and returns the volume of the entire model. */
	float volume() const;
	/** Returns true if all faces are triangles and false otherwise. */
	bool is_triangulated() const;
	/** Returns true if all faces in the model are connected and
	 * false if the model consists of more than one components. */
	bool is_connected() const;
	/** Returns true if the model is convex. If any of the vertices
	 * violates convexity (even due to non co-planar face vertices)
	 * this function should return false. */
	bool is_convex() const;
	/** Returns true if the model is watertight and false otherwise. */
	bool is_watertight() const;
	/** Returns true if the model is watertight and false otherwise.
	 * After return the given std::string object contains a message
	 * describing the reason of water tightness violation, if any. */
	bool is_watertight(std::string &msg) const;

	/** Validates the model by checking the index consistency and
	 * geometric validity of every faces. */
	void validate() const;

protected:
	/* Prevent public construction without shared_ptr wrapper */
	Model() = default;
	Model(const Model &other) = default;

private:
	/* Prevent move and assignment */
	Model(Model &&) = delete;
	Model &operator=(const Model &) = delete;
	Model &operator=(Model &&) = delete;

	F32_4D_VecT vertices_;
	F32_3D_VecT texture_vertices_;
	F32_3D_VecT vertex_normals_;
	std::set<Face> faces_;

	mutable bool is_connected_{true};
	mutable bool is_convex_{true};
	bool is_triangulated_{true};
	mutable bool is_validated_{true};
	mutable bool is_watertight_{true};

	mutable bool recalc_connectivity_{true};
	mutable bool recalc_convexity_{true};
	mutable bool recalc_water_tightness_{true};

	void needs_recalc_properties();

	static bool check_connectivity(const std::set<Face> &faces,
			const size_t nverts);
	bool check_water_tightness(std::string &msg) const;
	using FaceToIndexVecMap = std::map<Face, typename Face::IndexVecT>;
	const FaceToIndexVecMap get_concave_vertices() const;
};

#endif
