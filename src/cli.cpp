#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include <3dconv/cli.hpp>
#include <3dconv/io.hpp>
#include <3dconv/linalg.hpp>

using namespace std;
using namespace linalg;

namespace fs = std::filesystem;

void
print_file_types_help()
{
	cout << "Supported file types:" << endl;
	cout << "---------------------" << endl;
	cout << "  INPUT:" << endl;
	for (const auto &p : IOMap<Parser>::instance().map()) {
		cout << "   * " << p.first << endl;
	}
	cout << endl;
	cout << "  OUTPUT:" << endl;
	for (const auto &w : IOMap<Writer>::instance().map()) {
		cout << "   * " << w.first << endl;
	}
}

void
print_transforms_help()
{
	cout << "Supported tranformations:" << endl;
	cout << "-------------------------" << endl;
	cout << "Rotation    : ro:<axis-x>:<axis-y>:<axis-z>:<angle-in-rad>" << endl;
	cout << "Scaling     : sc:<factor>" << endl;
	cout << "Translation : tr:<direction-x>:<direction-y>:<direction-z>" << endl;
	cout << endl;
	cout << "Any combination of these will be accepted. Multiple "
		"transformations" << endl;
	cout << "can be given as a comma separated list of the above "
		"commands." << endl;
	cout << "E.g.:" << endl;
	cout << "     sc:3.7,ro:1:1:0:1.57,sc:2.4,tr:-4.2:-.3:3.6" << endl;
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
