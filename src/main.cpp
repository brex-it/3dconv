#include <cstdlib>
#include <exception>

#include <CLI11/CLI11.hpp>

#include <3dconv/cli.hpp>
#include <3dconv/model.hpp>
#include <3dconv/io.hpp>

int
main(int argc, char *argv[])
try {
	/* CLI parsing phase */
	std::string ifile, ofile, iotypes, transforms;
	bool printprop;

	CLI::App cli_app;
	cli_app.require_subcommand(1);
	auto run_command = cli_app.add_subcommand("run",
			"Run the specified conversion");

	run_command->add_option("-i,--input", ifile, "Input file")->required()
		->check(CLI::ExistingFile);
	run_command->add_option("-o,--output", ofile, "Output file")->required();
	run_command->add_flag("-p,--properties", printprop, "Print model properties");
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
		print_transforms_help();
		std::exit(0);
	});
	CLI11_PARSE(cli_app, argc, argv);

	std::string itype, otype;
	parse_iotypes(ifile, ofile, iotypes, itype, otype);

	/* Parsing phase */
	auto parser = IOMap<Parser>::get(itype);
	parser->open(ifile);
	auto model = (*parser)();
	model->validate();

	if (printprop) {
		/* Calculate properties before transformation */
		std::cout << "Model properties:" << std::endl;
		std::cout << "-----------------" << std::endl;
		std::cout << "Surface area: " << model->surface_area() << std::endl;
		std::cout << "Volume: " << model->volume() << std::endl;
	}

	/* Transformation phase */
	if (!transforms.empty()) {
		auto tr_matrix = parse_transforms(transforms);
		model->transform(tr_matrix);

		if (printprop) {
			/* Calculate properties after transformation */
			std::cout << std::endl;
			std::cout << "Model properties after transformation:" << std::endl;
			std::cout << "--------------------------------------" << std::endl;
			std::cout << "Surface area: " << model->surface_area() << std::endl;
			std::cout << "Volume: " << model->volume() << std::endl;
		}
	}

	/* Writing phase */
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
