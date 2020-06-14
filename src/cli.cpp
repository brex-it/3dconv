#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <CLI11/CLI11.hpp>

#include <3dconv/cli.hpp>
#include <3dconv/io.hpp>
#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

const char *FACE_TRANSFORMS_HELP_MSG = R"MSG(

Supported face transformations:
--------------------------------

    Transformation  |  Command
  ------------------+-----------
    Convexification |     c
    Triangulation   |     t

  Any combination of these will be accepted. Multiple transformations
  can be given as a comma separated list of the above commands.

  E.g.:
       c,t,t,c,t

  These operations are idempotent, so they can be applied multiple
  times, like in the example above, with no further effect after the
  first application.
)MSG";

const char *MODEL_TRANSFORMS_HELP_MSG = R"MSG(

Supported model transformations:
--------------------------------

    Transformation  |                   Command
  ------------------+-----------------------------------------------
    Rotation        | ro:<axis-x>:<axis-y>:<axis-z>:<angle-in-rad>
    Scaling         | sc:<factor>
    Skew            | sk:<domain-letter><range-letter>:<angle>
    Translation     | tr:<direction-x>:<direction-y>:<direction-z>

  Any combination of these will be accepted. Multiple transformations
  can be given as a comma separated list of the above commands.

  E.g.:
       sc:3.7,ro:1:1:0:1.57,sc:2.4,tr:-4.2:-.3:3.6,sk:zy:1.57

  In the skew command the domain and range letters can either be 'x',
  'y' or 'z' meaning that in which direction should we move (domain)
  from the origin to achieve skewing in an another direction (range).
)MSG";

const char *PROPERTIES_HELP_MSG = R"MSG(

Supported properties and their flags:
-------------------------------------

      Property name   |  Flag
    ------------------+--------
      connectivity    |   c
      convexity       |   x
      surface area    |   s
      triangularity   |   t
      volume          |   v
      water tightness |   w

  Or simply write 'a' to print all of the listed properties.

  Any combination of these letter can be contained in the string
  given as an argument for --print-properties but unsupported
  letters will result in an error. If 'a' is present, other flags
  will be omitted.
)MSG";

using namespace std;
using namespace linalg;

namespace fs = std::filesystem;

string
print_file_formats_help()
{
	ostringstream out;
	out << endl;
	out << "Supported file formats:" << endl;
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
	out << endl;
	return out.str();
}

CLIContext::CLIContext(int argc, char *argv[])
{
	CLI::App cli_app;

	/* Order-insensitive options */
	string ioformats;
	cli_app.add_option("-i,--input", ifile_, "Input file")->required()
		->check(CLI::ExistingFile);
	cli_app.add_option("-o,--output", ofile_, "Output file");
	cli_app.add_option("-f,--file-formats", ioformats,
		"Input and output file formats in the form [in-format]:[out-format] "
		"(If not specified the input and output file extensions will "
		"be used to determine the file formats.)");
	cli_app.add_option("-v,--verbosity", verbosity_,
		"Sets the verbosity level (0 causes silent run)")->default_val(1);

	/* Order-sensitive options (so called "actions") */
	vector<string> props, ftransforms, mtransforms;
	auto prop_opt = cli_app.add_option("-p,--print-properties", props,
		"Print model properties");
	auto ftrans_opt = cli_app.add_option("-F,--face-transformation",
		ftransforms, "Face transformation string");
	auto mtrans_opt = cli_app.add_option("-T,--transformation,"
		"--model-transformation", mtransforms, "Model transformation string");

	/* Callback for creating a detailed help message
	 * TODO: Move this into a source-independent man page */
	cli_app.footer([](){
		ostringstream txt;
		txt << print_file_formats_help();
		txt << PROPERTIES_HELP_MSG;
		txt << FACE_TRANSFORMS_HELP_MSG;
		txt << MODEL_TRANSFORMS_HELP_MSG;
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

	/* Set the final value of iformat_ and oformat_ */
	parse_ioformats(ifile_, ofile_, ioformats, iformat_, oformat_);

	/* Pack order-sensitive arguments into actions_ in the original order */
	reverse(props.begin(), props.end());
	reverse(ftransforms.begin(), ftransforms.end());
	reverse(mtransforms.begin(), mtransforms.end());
	using AT = Action::ActionType;
	for (const auto &o : cli_app.parse_order()) {
		if (o == prop_opt) {
			actions_.emplace_back(AT::PrintProperties, props.back());
			props.pop_back();
		} else if (o == ftrans_opt) {
			actions_.emplace_back(AT::FaceTransform, ftransforms.back());
			ftransforms.pop_back();
		} else if (o == mtrans_opt) {
			actions_.emplace_back(AT::ModelTransform, mtransforms.back());
			mtransforms.pop_back();
		}
	}
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
CLIContext::iformat() const
{
	return iformat_;
}

const string &
CLIContext::oformat() const
{
	return oformat_;
}

const std::vector<Action> &
CLIContext::actions() const
{
	return actions_;
}

int
CLIContext::verbosity() const
{
	return verbosity_;
}

InfoPrinter::InfoPrinter(int verbosity)
	: verbosity_level_{verbosity}
{}

void
InfoPrinter::operator()(int verbosity, const std::string &s0,
		const std::string &s1)
{
	if (verbosity <= verbosity_level_) {
		cout << ">>> " << s0 << s1 << endl;
	}
}

FaceTransforms
parse_face_transforms(const string &trstr)
{
	FaceTransforms ft;

	istringstream trss{trstr};
	ostringstream err_msg;
	string command;

	while (getline(trss, command, ',')) {
		if (command.size() != 1) {
			err_msg << "Invalid face transformation: " << command;
			throw CLIError(err_msg.str());
		}
		switch (command[0]) {
		case 'c':
			ft.convexify = true;
			break;
		case 't':
			ft.triangulate = true;
			break;
		default:
			err_msg << "Unknown face transformation: " << command;
			throw CLIError(err_msg.str());
		}
	}

	return ft;
}

void
parse_ioformats(const string &ifile, const string &ofile,
		const string &ioformats, string &iformat, string &oformat)
{
	iformat.clear();
	oformat.clear();

	/* Parse file-formats string */
	if (!ioformats.empty()) {
		if (ioformats.find(':') == string::npos) {
			throw CLIError("':' character cannot be omitted.");
		}
		istringstream ioss{ioformats};
		string format;
		size_t i = 0;
		while (getline(ioss, format, ':')) {
			switch (i) {
			case 0:
				iformat = format;
				break;
			case 1:
				oformat = format;
				break;
			default:
				throw CLIError("Too many arguments for "
						"format specification.");
			}
			++i;
		}
	}

	/* If either of the formats are unset, check the file extensions */
	if (iformat.empty()) {
		fs::path ipath{ifile};
		if (ipath.has_extension()) {
			iformat = ipath.extension().string().substr(1);
		} else {
			throw CLIError("Unable to determine input file format.");
		}
	}
	if (oformat.empty()) {
		fs::path opath{ofile};
		if (opath.has_extension()) {
			oformat = opath.extension().string().substr(1);
		} else if (!opath.empty()) {
			throw CLIError("Unable to determine output file format.");
		}
	}
}

FMatSq<float, 4>
parse_model_transforms(const string &trstr)
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
			if (opcode == "ro") {
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
			} else if (opcode == "sk") {
				/* Skew */
				size_t domain_dim, range_dim;
				float angle{};
				while (getline(pss, arg, ':')) {
					if (i >= 2) {
						throw CLIError("Too many arguments "
								"for skew.");
					}
					if (i == 0) {
						if (arg.size() != 2 || arg[0] == arg[1]) {
							throw CLIError("Invalid skew map.");
						}
						switch (arg[0]) {
						case 'x':
							domain_dim = 0;
							break;
						case 'y':
							domain_dim = 1;
							break;
						case 'z':
							domain_dim = 2;
							break;
						default:
							throw CLIError("Invalid skew domain.");
						}
						switch (arg[1]) {
						case 'x':
							range_dim = 0;
							break;
						case 'y':
							range_dim = 1;
							break;
						case 'z':
							range_dim = 2;
							break;
						default:
							throw CLIError("Invalid skew range.");
						}
					} else {
						angle = stof(arg);
					}
					++i;
				}
				if (i < 2) {
					throw CLIError("Not enough arguments for skew.");
				}
				trmat *= make_skew_mat<float, 3>(domain_dim, range_dim, angle);
			} else if (opcode == "tr") {
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

void
print_properties(shared_ptr<Model> m, const string &prop_str)
{
	bool all{false};

	bool connectivity{false};
	bool convexity{false};
	bool surface_area{false};
	bool triangularity{false};
	bool volume{false};
	bool water_tightness{false};

	for (const auto c : prop_str) {
		switch (c) {
		case 'a':
			all = true;
			break;
		case 'c':
			connectivity = true;
			break;
		case 'x':
			convexity = true;
			break;
		case 's':
			surface_area = true;
			break;
		case 't':
			triangularity = true;
			break;
		case 'v':
			volume = true;
			break;
		case 'w':
			water_tightness = true;
			break;
		default:
			ostringstream err_msg;
			err_msg << "Unknown property flag: " << c;
			throw CLIError(err_msg.str());
		}
	}

	if (all || connectivity) {
		cout << " * Is connected: " << (m->is_connected() ? "yes" : "no")
			<< endl;
	}
	if (all || convexity) {
		cout << " * Is convex: " << (m->is_convex() ? "yes" : "no") << endl;
	}
	if (all || surface_area) {
		cout << " * Surface area: " << m->surface_area() << endl;
	}
	if (all || triangularity) {
		cout << " * Is triangulated: "
			<< (m->is_triangulated() ? "yes" : "no") << endl;
	}
	if (all || volume) {
		cout << " * Volume: " << m->volume() << endl;
	}
	if (all || water_tightness) {
		string msg;
		bool wt = m->is_watertight(msg);
		cout << " * Is watertight: "
			<< (wt ? "yes" : "no [" + msg + "]") << endl;
	}
}
