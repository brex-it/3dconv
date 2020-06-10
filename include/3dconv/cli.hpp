#ifndef _3DCONV_CLI_HPP
#define _3DCONV_CLI_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include <3dconv/linalg.hpp>
#include <3dconv/model.hpp>

/**
 * Thrown on command line arguments related errors.
 */
struct CLIError : public std::logic_error {
	CLIError(const std::string &what_arg)
		: std::logic_error{what_arg} {}
};

/**
 * Represents an action to be performed on a Model object.
 * The type field determines the handling method and the
 * value field contains the input of that handler. */
struct Action {
	enum struct ActionType {
		PrintProperties,
		FaceTransform,
		ModelTransform,
	};

	Action(ActionType t, const std::string &v)
		: type{t}, value{v}
	{}

	ActionType type;
	std::string value;
};

/**
 * Helper struct for parsing face transformations.
 */
struct FaceTransforms {
	bool convexify{false};
	bool triangulate{false};
};

/**
 * Stores the parsed CLI context. The members are initialized
 * at construction time from the given command line arguments.
 * The constructor also handles the `--help` command line flag.
 */
class CLIContext {
public:
	///
	CLIContext(int argc, char *argv[]);

	///
	const std::string &ifile() const;
	///
	const std::string &ofile() const;
	///
	const std::string &iformat() const;
	///
	const std::string &oformat() const;
	///
	const std::vector<Action> &actions() const;

private:
	std::string ifile_;
	std::string ofile_;
	std::string iformat_;
	std::string oformat_;
	std::vector<Action> actions_;
};

/**
 * Parses the given face transformation string and returns a
 * FaceTransforms struct representing the requested transformations.
 */
FaceTransforms parse_face_transforms(const std::string &trstr);

/**
 * Selects the appropriate input and output file formats
 * either from the given file format specifications or,
 * if they are omitted, from the file extensions.
 */
void parse_ioformats(const std::string &ifile, const std::string &ofile,
	const std::string &ioformats, std::string &iformat, std::string &oformat);

/**
 * Parses the given model transformation string and constructs
 * a 4x4 matrix representing the homogeneous affine transformation.
 */
linalg::FMatSq<float, 4> parse_model_transforms(const std::string &trstr);

/**
 * Prints all properties of m for which props.<property-name>() returns true.
 */
void print_properties(std::shared_ptr<Model> m, const std::string &prop_str);

#endif
