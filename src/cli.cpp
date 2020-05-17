#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <CLI11/CLI11.hpp>

#include <3dconv/cli.hpp>
#include <3dconv/io.hpp>
#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

using namespace std;
using namespace linalg;

namespace fs = std::filesystem;

Properties::Properties(string prop_str)
{
	for (const auto c : prop_str) {
		switch (c) {
		case 'a':
			flags_ = numeric_limits<FlagWordT>::max();
			break;
		case 'c':
			flags_ |= static_cast<FlagWordT>(Flag::Connectivity);
			break;
		case 'x':
			flags_ |= static_cast<FlagWordT>(Flag::Convexity);
			break;
		case 's':
			flags_ |= static_cast<FlagWordT>(Flag::SurfaceArea);
			break;
		case 't':
			flags_ |= static_cast<FlagWordT>(Flag::Triangularity);
			break;
		case 'v':
			flags_ |= static_cast<FlagWordT>(Flag::Volume);
			break;
		case 'w':
			flags_ |= static_cast<FlagWordT>(Flag::WaterTightness);
			break;
		default:
			ostringstream err_msg;
			err_msg << "Unknown property flag: " << c;
			throw CLIError(err_msg.str());
		}
	}
}

bool
Properties::any() const
{
	return flags_ != 0;
}

bool
Properties::connectivity() const
{
	return flags_ & static_cast<FlagWordT>(Flag::Connectivity);
}

bool
Properties::convexity() const
{
	return flags_ & static_cast<FlagWordT>(Flag::Convexity);
}

bool
Properties::surface_area() const
{
	return flags_ & static_cast<FlagWordT>(Flag::SurfaceArea);
}

bool
Properties::triangularity() const
{
	return flags_ & static_cast<FlagWordT>(Flag::Triangularity);
}

bool
Properties::volume() const
{
	return flags_ & static_cast<FlagWordT>(Flag::Volume);
}

bool
Properties::water_tightness() const
{
	return flags_ & static_cast<FlagWordT>(Flag::WaterTightness);
}

CLIContext::CLIContext(int argc, char *argv[])
{
	std::string iotypes, prop_str;

	CLI::App cli_app;

	cli_app.add_option("-i,--input", ifile_, "Input file")->required()
		->check(CLI::ExistingFile);
	cli_app.add_option("-o,--output", ofile_, "Output file")->required();
	cli_app.add_option("-p,--properties", prop_str, "Print model properties");
	cli_app.add_option("-t,--file-types", iotypes,
			"Input and output file types in the form [in-type]:[out-type] "
			"(If not specified the input and output file extensions will "
			"be used to determine the file types.)");
	cli_app.add_option("-T,--transformation", transforms_,
			"Transformation string")->join(',');

	cli_app.footer([](){
		ostringstream txt;
		txt << print_file_types_help();
		txt << std::endl << std::endl;
		txt << print_properties_help();
		txt << std::endl << std::endl;
		txt << print_transforms_help();
		return txt.str();
	});

	/* Rethrow CLI11 exceptions and handle --help flag */
	try {
		cli_app.parse(argc, argv);
	} catch (const CLI::CallForHelp &h) {
		exit(cli_app.exit(h));
	} catch (const exception &e) {
		throw CLIError(e.what());
	}

	props_ = Properties{prop_str};
	parse_iotypes(ifile_, ofile_, iotypes, itype_, otype_);
}

const string &
CLIContext::ifile() const
{
	return ifile_;
}

const string &
CLIContext::ofile() const
{
	return ofile_;
}

const string &
CLIContext::itype() const
{
	return itype_;
}

const string &
CLIContext::otype() const
{
	return otype_;
}

const Properties &
CLIContext::props() const
{
	return props_;
}

const string &
CLIContext::transforms() const
{
	return transforms_;
}

string
print_file_types_help()
{
	ostringstream out;
	out << "Supported file types:" << endl;
	out << "---------------------" << endl;
	out << "  INPUT:" << endl;
	for (const auto &p : IOMap<Parser>::instance().map()) {
		out << "   * " << p.first << endl;
	}
	out << endl;
	out << "  OUTPUT:" << endl;
	for (const auto &w : IOMap<Writer>::instance().map()) {
		out << "   * " << w.first << endl;
	}
	return out.str();
}

void
print_properties(shared_ptr<Model> m, Properties props)
{
	if (props.any()) {
		cout << "Model properties:" << endl;
		cout << "-----------------" << endl;
	}
	if (props.connectivity()) {
		cout << "Is connected: " << (m->is_connected() ? "yes" : "no") << endl;
	}
	if (props.convexity()) {
		cout << "Is convex: " << (m->is_convex() ? "yes" : "no") << endl;
	}
	if (props.surface_area()) {
		cout << "Surface area: " << m->surface_area() << endl;
	}
	if (props.triangularity()) {
		cout << "Is triangulated: "
			<< (m->is_triangulated() ? "yes" : "no") << endl;
	}
	if (props.volume()) {
		cout << "Volume: " << m->volume() << endl;
	}
	if (props.water_tightness()) {
		cout << "Is watertight: "
			<< (m->is_watertight() ? "yes" : "no") << endl;
	}
}

string
print_properties_help()
{
	ostringstream out;
	out << "Supported properties and their flags:" << endl;
	out << "-------------------------------------" << endl;
	out << endl;
	out << "      Property name   |  Flag" << endl;
	out << "    ------------------+--------" << endl;
	out << "      connectivity    |   c" << endl;
	out << "      convexity       |   x" << endl;
	out << "      surface area    |   s" << endl;
	out << "      triangularity   |   t" << endl;
	out << "      volume          |   v" << endl;
	out << "      water tightness |   w" << endl;
	out << endl;
	out << "  Or simply write 'a' to print all of the listed properties."
		<< endl;
	out << endl;
	out << "  Any combination of these letter can be contained in the string"
		<< endl;
	out << "  given as an argument for --properties but unsupported letters"
		<< endl;
	out << "  will result in an error. If 'a' is present, other flags will be"
		<< endl;
	out << "  omitted." << endl;
	return out.str();
}

string
print_transforms_help()
{
	ostringstream out;
	out << "Supported transformations:" << endl;
	out << "--------------------------" << endl;
	out << "  Rotation    : ro:<axis-x>:<axis-y>:<axis-z>:<angle-in-rad>"
		<< endl;
	out << "  Scaling     : sc:<factor>" << endl;
	out << "  Translation : tr:<direction-x>:<direction-y>:<direction-z>"
		<< endl;
	out << endl;
	out << "  Any combination of these will be accepted. Multiple "
		"transformations" << endl;
	out << "  can be given as a comma separated list of the above "
		"commands." << endl;
	out << "  E.g.:" << endl;
	out << "       sc:3.7,ro:1:1:0:1.57,sc:2.4,tr:-4.2:-.3:3.6" << endl;
	return out.str();
}

void
parse_iotypes(const std::string &ifile, const std::string &ofile,
		const std::string &iotypes, std::string &itype, std::string &otype)
{
	itype.clear();
	otype.clear();

	/* Parse file-types string */
	if (!iotypes.empty()) {
		if (iotypes.find(':') == string::npos) {
			throw CLIError("':' character cannot be omitted.");
		}
		istringstream ioss{iotypes};
		string type;
		size_t i = 0;
		while (getline(ioss, type, ':')) {
			switch (i) {
			case 0:
				itype = type;
				break;
			case 1:
				otype = type;
				break;
			default:
				throw CLIError("Too many arguments for "
						"type specification.");
			}
			++i;
		}
	}

	/* If either of the types are unset, check the file extensions */
	if (itype.empty()) {
		fs::path ipath{ifile};
		if (ipath.has_extension()) {
			itype = ipath.extension().string().substr(1);
		} else {
			throw CLIError("Unable to determine input file format.");
		}
	}
	if (otype.empty()) {
		fs::path opath{ofile};
		if (opath.has_extension()) {
			otype = opath.extension().string().substr(1);
		} else {
			throw CLIError("Unable to determine output file format.");
		}
	}
}

FMatSq<float, 4>
parse_transforms(const std::string &trstr)
{
	auto trmat = make_id_mat<float, 4>();

	istringstream trss{trstr};
	ostringstream err_msg;
	string part;

	while (getline(trss, part, ',')) {
		istringstream pss{part};
		string opcode, arg;
		if (getline(pss, opcode, ':')) {
			size_t i = 0;
			if (opcode == "tr") {
				/* Translation */
				FVec<float, 3> tvec;
				while (getline(pss, arg, ':')) {
					if (i >= 3) {
						throw CLIError("Too many arguments "
								"for translation.");
					}
					tvec[i] = stof(arg);
					++i;
				}
				if (i < 3) {
					throw CLIError("Not enough arguments "
							"for translation.");
				}
				trmat *= make_translation_mat(tvec);
			} else if (opcode == "sc") {
				/* Scaling */
				float factor{1.f};
				while (getline(pss, arg, ':')) {
					if (i >= 1) {
						throw CLIError("Too many arguments "
								"for scaling.");
					}
					factor = stof(arg);
					++i;
				}
				if (i < 1) {
					throw CLIError("Not enough arguments "
							"for scaling.");
				}
				trmat *= make_scaling_mat<float, 3>(factor);
			} else if (opcode == "ro") {
				/* Rotation */
				FVec<float, 3> axis;
				float angle{0.f};
				while (getline(pss, arg, ':')) {
					if (i >= 4) {
						throw CLIError("Too many arguments "
								"for rotation.");
					}
					if (i == 3) {
						angle = stof(arg);
					} else {
						axis[i] = stof(arg);
					}
					++i;
				}
				if (i < 4) {
					throw CLIError("Not enough arguments "
							"for rotation.");
				}
				trmat *= make_rotation_mat(axis, angle);
			} else {
				err_msg << "Unknown transformation: " << opcode;
				throw CLIError(err_msg.str());
			}
		} else {
			err_msg << "Missing transformation." << part;
			throw CLIError(err_msg.str());
		}
	}

	return trmat;
}
