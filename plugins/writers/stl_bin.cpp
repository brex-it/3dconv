#include <memory>
#include <sstream>
#include <type_traits>

#include <3dconv/io.hpp>
#include <3dconv/utils.hpp>

class STLBinWriter : public Writer {
public:
	STLBinWriter() : Writer{std::ios_base::binary} {}

	void operator()(const std::shared_ptr<const Model> orig_model) override {
		/* NOTE: Not so memory-efficient but it seems reasonable.
		 *       Copying can be avoided by making orig_model non-const but
		 *       Writer objects are not supposed to change their inputs,
		 *       so we run Model::triangulate() on a copy model. */
		auto model = Model::create(orig_model);
		std::ostringstream err_msg;
		if (file_->is_open()) try {
			/* STL can only contain triangular faces */
			if (!model->is_triangulated()) {
				model->triangulate();
			}

			/* Write 80-byte header filled with zeros */
			uint8_t zero_buf[80]{};
			file_->write((char *)zero_buf, 80);

			/* Number of triangles represented with
			 * 4 bytes in little-endian ordering */
			uint8_t trinum[4];
			uint2bytes<Endian::Little>(model->faces().size(), trinum, 4);
			file_->write((char *)trinum, 4);

			/* Write all triangle data */
			for (const auto &f : model->faces()) {
				/* Normal */
				float vec_buf[3];
				const auto &n = f.normal();
				vec_buf[0] = n[0];
				vec_buf[1] = n[1];
				vec_buf[2] = n[2];
				file_->write((char *)vec_buf, 3 * sizeof(float));

				/* Vertices */
				for (const auto &v : f.vertices()) {
					const auto &mv = model->vertices()[v];
					vec_buf[0] = mv[0];
					vec_buf[1] = mv[1];
					vec_buf[2] = mv[2];
					file_->write((char *)vec_buf, 3 * sizeof(float));
				}
				file_->write((char *)zero_buf, 2);
			}
		} catch (const std::exception &e) {
			file_->flush();
			throw WriteError(e.what(), filename_);
		}
		file_->flush();
	}
};

/* Register the writer into the IOMap<Writer> singleton */
REGISTER_WRITER("stl-bin", STLBinWriter);
