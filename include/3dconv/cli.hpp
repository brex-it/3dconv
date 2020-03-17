#ifndef _3DCONV_CLI_HPP
#define _3DCONV_CLI_HPP

#include <stdexcept>
#include <string>

#include <3dconv/linalg.hpp>

/**
 * Thrown on command line arguments related errors.
 */
struct CLIError : public std::logic_error {
	CLIError(const std::string &what_arg)
		: std::logic_error{what_arg} {}
};

/**
 * Prints all of the input and output file types
 * for which the respecting IOMap contains a
 * registered Parser or Writer object.
 */
void print_file_types_help();

/**
 * Prints all of the supported affine transformations
 * and their command line syntax.
 */
void print_transforms_help();

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
