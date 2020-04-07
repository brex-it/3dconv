#include <cstdlib>
#include <exception>
#include <string>

#include <CLI11/CLI11.hpp>

#include <3dconv/cli.hpp>
#include <3dconv/model.hpp>
#include <3dconv/io.hpp>

int
main(int argc, char *argv[])
try {
	/* CLI parsing phase */
	std::string ifile, ofile, iotypes, transforms;
	std::string prop_str;

	CLI::App cli_app;
	cli_app.require_subcommand(1);
	auto run_command = cli_app.add_subcommand("run",
			"Run the specified conversion");

	run_command->add_option("-i,--input", ifile, "Input file")->required()
		->check(CLI::ExistingFile);
	run_command->add_option("-o,--output", ofile, "Output file")->required();
	run_command->add_option("-p,--properties", prop_str, "Print model properties");
	run_command->add_option("-t,--file-types", iotypes,
			"Input and output file types in the form [in-type]:[out-type] "
			"(If not specified the input and output file extensions will "
			"be used to determine the file types.)");
	run_command->add_option("-T,--transformation", transforms,
			"Transformation string");

	auto man_command = cli_app.add_subcommand("man",
			"Detailed description of supported file "
			"types and transformations");
	man_command->callback([](){
		print_file_types_help();
		std::cout << std::endl << std::endl;
		print_properties_help();
		std::cout << std::endl << std::endl;
		print_transforms_help();
		std::exit(0);
	});
	CLI11_PARSE(cli_app, argc, argv);

	std::string itype, otype;
	parse_iotypes(ifile, ofile, iotypes, itype, otype);
	Properties props{prop_str};

	/* Model parsing phase */
	auto parser = IOMap<Parser>::get(itype);
	parser->open(ifile);
	auto model = (*parser)();
	model->validate();

	/* Print requested properties before transformation */
	print_properties(model, props);

	/* Transformation phase */
	if (!transforms.empty()) {
		auto tr_matrix = parse_transforms(transforms);
		model->transform(tr_matrix);

		/* Print requested properties after transformation */
		if (props.any()) {
			std::cout << std::endl;
		}
		print_properties(model, props);
	}

	/* Model writing phase */
	auto writer = IOMap<Writer>::get(otype);
	writer->open(ofile);
	(*writer)(model);

	return 0;
} catch (const CLIError &ce) {
	std::cerr << "[ERROR | CLI] " << ce.what() << std::endl;
} catch (const ModelError &me) {
	std::cerr << "[ERROR | MODEL] " << me.what() << std::endl;
} catch (const ParseError &pe) {
	std::cerr << "[ERROR | PARSE | " << pe.filename << ":"
		<< pe.line_num << "] " << pe.what() << std::endl;
} catch (const WriteError &we) {
	std::cerr << "[ERROR | WRITE | " << we.filename << "] "
		<< we.what() << std::endl;
} catch (const std::exception &oe) {
	std::cerr << "[ERROR | OTHER] " << oe.what() << std::endl;
} catch (...) {
	std::cerr << "[ERROR | UNKNOWN]" << std::endl;
}
