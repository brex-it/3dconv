#include <exception>
#include <functional>
#include <memory>
#include <unordered_map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <3dconv/model.hpp>
#include <3dconv/io.hpp>

class OBJParser : public Parser {
public:
	std::shared_ptr<Model> operator()() override {
		auto model = Model::create();

		if (file_->is_open()) {
			std::ostringstream err_msg;
			size_t lcnt = 0;
			try {
				std::string line, tok;

				/* Line reader loop */
				while (std::getline(*file_, line)) {
					++lcnt;
					auto hash_pos = line.find('#');
					if (hash_pos != std::string::npos) {
						line.erase(hash_pos);
					}
					std::istringstream lss{line};

					/* Read the first token and if there is
					 * a processing function for that token
					 * call it with the rest of the line */
					if (lss >> tok) {
						auto pf = proc_functions_.find(tok);
						if (pf == proc_functions_.end()) {
							err_msg << "Invalid statement: " << line;
							throw std::logic_error(err_msg.str());
						}

						pf->second(lss, model);
					}
				}
			} catch (const std::exception &e) {
				throw ParseError(e.what(), filename_, lcnt);
			}
		}

		return model;
	}

private:
	using ProcFunT = std::function<void(std::istringstream &,
		std::shared_ptr<Model>)>;
	using ProcFunMapT = std::unordered_map<std::string, ProcFunT>;
	static ProcFunMapT proc_functions_;
};

namespace {

/* Processing functions */

void
vertex(std::istringstream &lss, std::shared_ptr<Model> model)
{
	float coord;
	std::vector<float> coords;

	while (lss >> coord) {
		coords.push_back(coord);
	}

	if (coords.size() > 4) {
		throw std::logic_error("Too many arguments for vertex.");
	} else if (coords.size() == 3) {
		/* If we have only three coordinates we'll
		 * set weight to the default value 1.f. */
		coords.push_back(1.f);
	} else {
		throw std::logic_error("Not enough arguments for vertex.");
	}

	model->add_vertex(FVec<float, 4>{coords[0], coords[1], coords[2], coords[3]});
}

void
texture_vertex(std::istringstream &lss, std::shared_ptr<Model> model)
{
	float coord;
	std::vector<float> coords{0.f, 0.f, 0.f};

	size_t n = 0;
	while (lss >> coord && n < 3) {
		coords[n++] = coord;
	}

	if (n == 0) {
		throw std::logic_error("Not enough arguments for texture vertex.");
	} else if (n > 3) {
		throw std::logic_error("Too many arguments for texture vertex.");
	}

	model->add_texture_vertex(FVec<float, 3>{coords[0], coords[1], coords[2]});
}

void
vertex_normal(std::istringstream &lss, std::shared_ptr<Model> model)
{
	float coord;
	std::vector<float> coords;

	while (lss >> coord) {
		coords.push_back(coord);
	}

	if (coords.size() > 3) {
		throw std::logic_error("Too many arguments for vertex normal.");
	} else if (coords.size() < 3) {
		throw std::logic_error("Not enough arguments for vertex normal.");
	}

	model->add_vertex_normal(FVec<float, 3>{coords[0], coords[1], coords[2]});
}

/* Wrapper for std::stof(). Throws on non-digit trailing characters. */
inline long
str2long(const std::string &str)
{
	size_t proc_char_num;
	long val = std::stof(str, &proc_char_num);

	if (proc_char_num < str.size()) {
		std::ostringstream msg;
		msg << "Not a valid integer: " << str;
		throw std::invalid_argument(msg.str());
	}
	return val;
}

void
face(std::istringstream &lss, std::shared_ptr<Model> model)
{
	std::ostringstream err_msg;
	std::string group;
	Face face{model};

	/* Variables for group consistency checking */
	int slashes{-1};
	bool saved_has_texture{false};

	while (lss >> group) {
		if (group.back() == '/') {
			err_msg << "Last char cannot be slash: " << group;
			throw std::logic_error(err_msg.str());
		}

		std::istringstream grss{group};
		std::string part;
		long val;
		int n = 0;
		bool has_texture = false;

		while (std::getline(grss, part, '/')) {
			++n;
			if (n > 3) {
				err_msg << "Too many slashes: " << group;
				throw std::logic_error(err_msg.str());
			}
			if (part.empty()) {
				if (n == 2) {
					continue;
				}
				err_msg << ((n == 1) ? "Vertex index "
					: (n == 3) ? "Index of vertex normal " : "")
					<< "cannot be omitted: " << group;
				throw std::logic_error(err_msg.str());
			}

			/* For every part of the group call the appropriate
			 * add function */
			void (Face::*add_fun)(const size_t){nullptr};
			size_t cont_sz{0};
			switch (n) {
			default:
			case 1:
				add_fun = &Face::add_vertex;
				cont_sz = model->vertices().size();
				break;
			case 2:
				has_texture = true;
				add_fun = &Face::add_texture_vertex;
				cont_sz = model->texture_vertices().size();
				break;
			case 3:
				add_fun = &Face::add_vertex_normal;
				cont_sz = model->vertex_normals().size();
				break;
			}

			/* Handle relative indices */
			if ((val = str2long(part)) < 0) {
				if ((size_t)-val > cont_sz) {
					err_msg << "Invalid relative index: " << val;
					throw std::logic_error(err_msg.str());
				}
				(face.*add_fun)(cont_sz + val);
			} else {
				(face.*add_fun)(val - 1);
			}
		}

		/* Group consistency checking */
		if (group.find('/') == std::string::npos) {
			n = 0;
		}
		if (slashes == -1) {
			saved_has_texture = has_texture;
			slashes = n;
		}
		if (slashes != n || saved_has_texture != has_texture) {
			throw std::logic_error("Every index group must contain "
					"the same amount of elements.");
		}
	}

	/* Face validation */
	auto vsz = face.vertices().size();
	if (vsz < 3) {
		throw std::logic_error("Faces must contain at least three "
				"distinct vertex indices.");
	}
	auto tvsz = face.texture_vertices().size();
	if (tvsz > 0 && tvsz < vsz) {
		throw std::logic_error("Faces must either contain zero or the same "
				"number of texture vertex indices as vertex indices.");
	}
	auto vnsz = face.vertex_normals().size();
	if (vnsz > 0 && vnsz < vsz) {
		throw std::logic_error("Faces must either contain zero or the same "
				"number of vertex normal indices as vertex indices.");
	}

	model->add_face(face);
}

} // namespace

/* Register the processing funtions into our parser */
OBJParser::ProcFunMapT OBJParser::proc_functions_ = {
	{"v", vertex},
	{"vt", texture_vertex},
	{"vn", vertex_normal},
	{"f", face},
};

/* Register the parser itself into the IOMap<Parser> singleton */
REGISTER_PARSER("obj", OBJParser);
