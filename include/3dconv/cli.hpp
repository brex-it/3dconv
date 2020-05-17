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
 * Stores flags of the selected properties and provides getters
 * to make queries on these flags. It behaves like a regular C-style
 * "bit vector" but guarantees type-safety.
 */
class Properties {
public:
	///
	Properties(std::string prop_str);

	///
	bool any() const;
	///
	bool connectivity() const;
	///
	bool convexity() const;
	///
	bool surface_area() const;
	///
	bool triangularity() const;
	///
	bool volume() const;
	///
	bool water_tightness() const;

private:
	using FlagWordT = uint_fast8_t;
	enum struct Flag : FlagWordT {
		Connectivity   = 1 << 0,
		Convexity      = 1 << 1,
		SurfaceArea    = 1 << 2,
		Triangularity  = 1 << 3,
		Volume         = 1 << 4,
		WaterTightness = 1 << 5,
	};
	FlagWordT flags_{0};
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
	const std::string &itype() const;
	///
	const std::string &otype() const;
	///
	const Properties &props() const;
	///
	const std::string &transforms() const;

private:
	std::string ifile_;
	std::string ofile_;
	std::string itype_;
	std::string otype_;
	Properties props_{""};
	std::string transforms_;
};

/**
 * Prints all of the input and output file types
 * for which the respecting IOMap contains a
 * registered Parser or Writer object.
 */
std::string print_file_types_help();

/**
 * Prints all properties of m for which props.<property-name>() returns true.
 */
void print_properties(std::shared_ptr<Model> m, Properties props);

/**
 * Prints all of the supported properties which
 * can be queried by `--properties`.
 */
std::string print_properties_help();

/**
 * Prints all of the supported affine transformations
 * and their command line syntax.
 */
std::string print_transforms_help();

/**
 * Selects the appropriate input and output file types
 * either from the given file type specifications or if
 * they are omitted from the file extensions.
 */
void parse_iotypes(const std::string &ifile, const std::string &ofile,
		const std::string &iotypes, std::string &itype, std::string &otype);

/**
 * Parses the given transformation string and constructs
 * a 4x4 matrix representing the homogeneous affine transformation.
 */
linalg::FMatSq<float, 4> parse_transforms(const std::string &trstr);

#endif
